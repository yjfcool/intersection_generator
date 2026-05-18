#pragma once
#include "../data_types.h"
#include "../config.h"
#include "../bezier/cubic_bezier.h"
#include "../../utils/geom_utils.h"
#include "../../utils/logger.h"
#include <map>
#include <optional>

/**
 * 车道边线生成（优化版）
 *
 * 策略：
 *  1. 对中心线做等距偏移得到粗边线
 *  2. 首尾端点严格对齐路口外对应边线端点
 *  3. 首尾各用三次贝塞尔重拟合过渡段（保证 G1 平滑接入路口外边线）
 *  4. 中间段做移动平均平滑，消除偏移产生的局部抖动
 *
 * 关键改进（相对旧版）：
 *  - 法线计算：使用两侧线段的角平分线法线（而非方向平均再旋转）
 *  - 首尾平滑范围更大，用贝塞尔严格保 G1
 *  - 中间段平滑消除折角
 */
class EdgeLineGenerator {
    const Config& cfg_;

public:
    explicit EdgeLineGenerator(const Config& cfg) : cfg_(cfg) {}

    std::vector<GeneratedEdgeLine> generate(
        const std::vector<GeneratedCenterline>& centerlines,
        const IntersectionInput& inp)
    {
        std::vector<GeneratedEdgeLine> result;

        // 建立 connectionId → 已生成中心线 的映射
        std::map<std::string, const GeneratedCenterline*> clMap;
        for(auto& gcl : centerlines) clMap[gcl.connectionId] = &gcl;

        // 按进入组处理
        for(auto& [gid, grp] : inp.laneGroups){
            if(grp.type != GroupType::ENTER) continue;

            // 找到该组的所有连通关系（按laneOrder排列）
            std::vector<std::pair<int,const Connection*>> orderedConns;
            for(auto& conn : inp.connections){
                if(conn.enterGroupId != gid) continue;
                auto clit = inp.centerlines.find(conn.enterLineId);
                int order = (clit!=inp.centerlines.end()) ? clit->second.laneOrder : 0;
                orderedConns.push_back({order, &conn});
            }
            std::sort(orderedConns.begin(), orderedConns.end(),
                [](auto& a, auto& b){ return a.first < b.first; });

            // 为相邻中心线对生成共享边线
            generateGroupEdgeLines(orderedConns, grp, inp, clMap, result);
        }

        return result;
    }

private:
    void generateGroupEdgeLines(
        const std::vector<std::pair<int,const Connection*>>& orderedConns,
        const LaneGroup& grp,
        const IntersectionInput& inp,
        const std::map<std::string, const GeneratedCenterline*>& clMap,
        std::vector<GeneratedEdgeLine>& result)
    {
        if(orderedConns.empty()) return;

        for(int i=0;i<(int)orderedConns.size();++i){
            const Connection* conn = orderedConns[i].second;
            auto gclIt = clMap.find(conn->id);
            if(gclIt == clMap.end()) continue;
            const GeneratedCenterline& gcl = *gclIt->second;

            if(gcl.geom.size()<2) continue;

            // 估算左/右半宽
            double hwLeft  = estimateHalfWidth(conn->enterLineId, true,  grp, inp);
            double hwRight = estimateHalfWidth(conn->enterLineId, false, grp, inp);

            // 生成偏移折线（改进法线计算）
            Polyline leftPts  = offsetPolyline(gcl.geom, hwLeft);
            Polyline rightPts = offsetPolyline(gcl.geom, -hwRight);

            // 移除过于密集的点（避免偏移产生的近零长度段引起虚假高曲率）
            removeDuplicates(leftPts, 0.02);
            removeDuplicates(rightPts, 0.02);

            // 找路口外对应边线的连接端点和切线
            // 需要按具体组过滤，避免匹配到其它方向的 EXIT 边线
            std::string enterGrpId = conn->enterGroupId;
            std::string exitGrpId  = conn->exitGroupId;

            Point2D leftStartPt  = findEdgePtInGroup(conn->enterLineId, true,  enterGrpId, inp, hwLeft);
            Point2D leftEndPt    = findEdgePtInGroup(conn->exitLineId,  true,  exitGrpId,  inp, hwLeft);
            Point2D rightStartPt = findEdgePtInGroup(conn->enterLineId, false, enterGrpId, inp, hwRight);
            Point2D rightEndPt   = findEdgePtInGroup(conn->exitLineId,  false, exitGrpId,  inp, hwRight);

            Point2D enterTangLeft  = getEdgeTangent(conn->enterLineId, inp);
            Point2D exitTangLeft   = getEdgeTangent(conn->exitLineId,  inp);
            Point2D enterTangRight = enterTangLeft;
            Point2D exitTangRight  = exitTangLeft;

            // 中间段移动平均平滑（更多 passes 消除偏移噪声）
            smoothMiddle(leftPts, 5);
            smoothMiddle(rightPts, 5);

            // 首尾 G1 贝塞尔重拟合 + 端点对齐
            // K = 25% of total points, min 6, for sufficient transition
            int smoothPts = std::max(6, (int)leftPts.size() / 4);
            bezierAlignEnds(leftPts,  leftStartPt,  enterTangLeft,
                                      leftEndPt,    exitTangLeft,   smoothPts);
            smoothPts = std::max(6, (int)rightPts.size() / 4);
            bezierAlignEnds(rightPts, rightStartPt, enterTangRight,
                                      rightEndPt,   exitTangRight,  smoothPts);

            // 左边线
            {
                GeneratedEdgeLine el;
                el.id                = "gen_el_left_"+conn->id;
                el.geom              = leftPts;
                el.leftCenterlineId  = "";
                el.rightCenterlineId = gcl.id;
                el.qualityFlags      = 0;
                result.push_back(el);
            }
            // 右边线
            {
                GeneratedEdgeLine el;
                el.id                = "gen_el_right_"+conn->id;
                el.geom              = rightPts;
                el.leftCenterlineId  = gcl.id;
                el.rightCenterlineId = "";
                el.qualityFlags      = 0;
                result.push_back(el);
            }
        }
    }

    // ═══════════════════════════════════════════
    // 移除近重复点（距离 < minDist 的连续点）
    // ═══════════════════════════════════════════
    void removeDuplicates(Polyline& pts, double minDist) const {
        if(pts.size() < 3) return;
        Polyline out;
        out.push_back(pts.front());
        for(size_t i=1; i<pts.size()-1; ++i){
            if(dist(pts[i], out.back()) >= minDist){
                out.push_back(pts[i]);
            }
        }
        out.push_back(pts.back());
        pts = out;
    }

    // ═══════════════════════════════════════════
    // 改进的偏移折线：使用角平分线法线
    // offset > 0 向左偏移, < 0 向右
    // ═══════════════════════════════════════════
    Polyline offsetPolyline(const Polyline& pts, double offset) const {
        if(pts.size()<2) return pts;
        int n = (int)pts.size();
        Polyline out;
        out.reserve(n);

        for(int i=0;i<n;++i){
            Point2D normal;
            if(i==0){
                // 首点：用第一段方向的左法线
                Point2D dir = (pts[1]-pts[0]).normalized();
                normal = dir.rotLeft();
            } else if(i==n-1){
                // 尾点：用最后一段方向的左法线
                Point2D dir = (pts[n-1]-pts[n-2]).normalized();
                normal = dir.rotLeft();
            } else {
                // 中间点：用角平分线法线
                // 前后两段方向
                Point2D d1 = (pts[i]-pts[i-1]).normalized();
                Point2D d2 = (pts[i+1]-pts[i]).normalized();
                // 两段左法线
                Point2D n1 = d1.rotLeft();
                Point2D n2 = d2.rotLeft();
                // 角平分线法线（求平均并归一化）
                Point2D avg = n1 + n2;
                if(avg.norm() < EPS){
                    // 方向完全反向（180°折角），取 n1
                    normal = n1;
                } else {
                    normal = avg.normalized();
                    // 修正偏移量：对尖角进行 miter 补偿
                    // miter = 1 / cos(半角) = 1 / (n1·avg_normalized)
                    double cosHalf = n1.dot(normal);
                    if(cosHalf > 0.3){
                        // miter 补偿，但限制最大放大 3 倍
                        double miter = std::min(1.0/cosHalf, 3.0);
                        out.push_back(pts[i] + normal * (offset * miter));
                        continue;
                    }
                }
            }
            out.push_back(pts[i] + normal * offset);
        }
        return out;
    }

    // ═══════════════════════════════════════════
    // 中间段移动平均平滑（保留首尾各 margin 个点不动）
    // ═══════════════════════════════════════════
    void smoothMiddle(Polyline& pts, int passes) const {
        int n = (int)pts.size();
        if(n < 5) return;
        int margin = std::max(2, n/8); // 首尾保留区域

        for(int pass=0; pass<passes; ++pass){
            Polyline tmp = pts;
            for(int i=margin; i<n-margin; ++i){
                // 5 点加权平均：1-2-4-2-1
                Point2D sum = pts[i]*4.0;
                int cnt = 4;
                if(i-1>=0)   { sum += pts[i-1]*2.0; cnt+=2; }
                if(i+1<n)    { sum += pts[i+1]*2.0; cnt+=2; }
                if(i-2>=0)   { sum += pts[i-2]*1.0; cnt+=1; }
                if(i+2<n)    { sum += pts[i+2]*1.0; cnt+=1; }
                tmp[i] = sum / (double)cnt;
            }
            pts = tmp;
        }
    }

    // ═══════════════════════════════════════════
    // 首尾 G1 贝塞尔重拟合 + 端点对齐
    // 用三次贝塞尔重构首端 [0..K] 和尾端 [n-1-K..n-1] 的点，
    // 保证：
    //   - pts[0] == startPt, pts[n-1] == endPt
    //   - 首端切线 == startTang（G1 接入路口外边线）
    //   - 尾端切线 == 沿 endTang 方向（G1 接出路口外边线）
    // ═══════════════════════════════════════════
    void bezierAlignEnds(Polyline& pts,
        const Point2D& startPt, const Point2D& startTang,
        const Point2D& endPt,   const Point2D& endTang,
        int K) const
    {
        int n = (int)pts.size();
        if(n < 6) return;

        // K 应覆盖首/尾端点到最近匹配区域，
        // 使用 n 的比例（20%~30%）+ 最少 4 个点
        K = std::max(4, std::min(K, n/3));

        // 强制端点对齐
        pts.front() = startPt;
        pts.back()  = endPt;

        // ── 首端重拟合 [0..K] ──
        // 找到 pts[K] 作为锚点（不变），从 startPt 到 pts[K] 做贝塞尔
        {
            Point2D p0 = startPt;
            Point2D p3 = pts[K];
            double d = dist(p0, p3);
            if(d > EPS){
                // p3 处切线：由后续点方向估算
                Point2D t3;
                if(K+1 < n){
                    t3 = (pts[K+1] - pts[K-1]).normalized();
                } else {
                    t3 = (pts[K] - pts[K-1]).normalized();
                }
                double alpha = 0.38;
                Point2D p1 = p0 + startTang * (alpha * d);
                Point2D p2 = p3 - t3 * (alpha * d);
                CubicBezier cb(p0, p1, p2, p3);
                for(int i=1; i<K; ++i){
                    double t = (double)i / K;
                    pts[i] = cb.eval(t);
                }
            }
        }

        // ── 尾端重拟合 [n-1-K..n-1] ──
        {
            int startIdx = n-1-K;
            if(startIdx < K) startIdx = K; // 避免与首端重叠
            Point2D p0 = pts[startIdx];
            Point2D p3 = endPt;
            double d = dist(p0, p3);
            if(d > EPS){
                // p0 处切线：由相邻点方向估算
                Point2D t0;
                if(startIdx > 0 && startIdx+1 < n){
                    t0 = (pts[startIdx] - pts[startIdx-1]).normalized();
                } else {
                    t0 = (p3 - p0).normalized();
                }
                // endTang 指向路口内，P2 = P3 + endTang * alpha * d
                double alpha = 0.38;
                Point2D p1 = p0 + t0 * (alpha * d);
                Point2D p2 = p3 + endTang * (alpha * d);
                CubicBezier cb(p0, p1, p2, p3);
                int count = n-1 - startIdx;
                for(int i=1; i<count; ++i){
                    double t = (double)i / count;
                    pts[startIdx + i] = cb.eval(t);
                }
            }
        }

        // 最终确保端点精确
        pts.front() = startPt;
        pts.back()  = endPt;
    }

    // ═══════════════════════════════════════════
    // 半宽估算
    // ═══════════════════════════════════════════
    double estimateHalfWidth(const std::string& enterLineId, bool isLeft,
                              const LaneGroup& grp, const IntersectionInput& inp) const
    {
        auto clit = inp.centerlines.find(enterLineId);
        if(clit==inp.centerlines.end()) return cfg_.edgeLine.defaultLaneWidth * 0.5;

        const Point2D& clPt = clit->second.connectionPt;
        Point2D tangent = clit->second.tangentDir;
        Point2D normal  = tangent.rotLeft();

        // 找最近的同侧边线端点
        double bestDist = -1;
        for(auto& eid : grp.edgelineIds){
            auto elit = inp.edgelines.find(eid);
            if(elit==inp.edgelines.end()) continue;
            if(elit->second.geom.empty()) continue;
            const Point2D& ep = elit->second.connectionPt;
            double lateral = (ep-clPt).dot(normal);
            if(isLeft && lateral > 0.01 && lateral < 10.0){
                if(bestDist<0 || lateral<bestDist) bestDist=lateral;
            } else if(!isLeft && lateral < -0.01 && lateral > -10.0){
                if(bestDist<0 || (-lateral)<bestDist) bestDist=-lateral;
            }
        }

        if(bestDist > 0) return bestDist;
        return cfg_.edgeLine.defaultLaneWidth * 0.5;
    }

    // ═══════════════════════════════════════════
    // 找边线端点（限定在指定组内）
    // ═══════════════════════════════════════════
    Point2D findEdgePtInGroup(const std::string& lineId, bool isLeft,
                              const std::string& groupId,
                              const IntersectionInput& inp, double hw) const
    {
        auto clit = inp.centerlines.find(lineId);
        if(clit==inp.centerlines.end()) return {0,0};

        const Point2D& clPt  = clit->second.connectionPt;
        const Point2D& tang  = clit->second.tangentDir;
        Point2D normal = tang.rotLeft();

        // 在指定组的边线中查找
        auto git = inp.laneGroups.find(groupId);
        if(git != inp.laneGroups.end()){
            for(auto& eid : git->second.edgelineIds){
                auto elit = inp.edgelines.find(eid);
                if(elit == inp.edgelines.end()) continue;
                if(elit->second.geom.empty()) continue;
                const Point2D& ep = elit->second.connectionPt;
                double lat = (ep-clPt).dot(normal);
                if(isLeft && lat > 0.01 && std::abs(lat-hw) < hw*0.8) return ep;
                if(!isLeft && lat < -0.01 && std::abs(-lat-hw) < hw*0.8) return ep;
            }
        }
        // 回退：用中心线连接点+法线偏移
        return clPt + normal*(isLeft?hw:-hw);
    }

    // ═══════════════════════════════════════════
    // 获取中心线切线方向（用于边线端点切线）
    // ═══════════════════════════════════════════
    Point2D getEdgeTangent(const std::string& lineId, const IntersectionInput& inp) const {
        auto clit = inp.centerlines.find(lineId);
        if(clit==inp.centerlines.end()) return {0,1};
        return clit->second.tangentDir;
    }
};
