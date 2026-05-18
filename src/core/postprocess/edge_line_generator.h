#pragma once
#include "../data_types.h"
#include "../config.h"
#include "../bezier/cubic_bezier.h"
#include "../../utils/geom_utils.h"
#include "../../utils/logger.h"
#include <map>
#include <optional>

/**
 * 车道边线生成
 * 默认策略（b）：固定偏移法，偏移量 = 车道宽度的一半
 * 约束：路口内边线首尾端点必须与路口外对应边线端点严格重合
 * 平滑：连接点附近做局部贝塞尔重拟合
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

        // 建立 centerlineId → 已生成中心线 的映射
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
            // 边线在：最内侧车道左边线、每对相邻车道间的边线、最外侧车道右边线
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

        // 对每条生成中心线，生成左右边线
        for(int i=0;i<(int)orderedConns.size();++i){
            const Connection* conn = orderedConns[i].second;
            auto gclIt = clMap.find(conn->id);
            if(gclIt == clMap.end()) continue;
            const GeneratedCenterline& gcl = *gclIt->second;

            if(gcl.geom.size()<2) continue;

            // 估算左/右半宽
            double hwLeft  = estimateHalfWidth(conn->enterLineId, true,  grp, inp);
            double hwRight = estimateHalfWidth(conn->enterLineId, false, grp, inp);

            // 生成偏移折线
            Polyline leftPts  = offsetPolyline(gcl.geom, hwLeft);
            Polyline rightPts = offsetPolyline(gcl.geom, -hwRight);

            // 找路口外对应边线的连接端点
            Point2D leftStartPt  = findEnterEdgePt(conn->enterLineId, true,  inp, hwLeft);
            Point2D leftEndPt    = findExitEdgePt (conn->exitLineId,  true,  inp, hwLeft);
            Point2D rightStartPt = findEnterEdgePt(conn->enterLineId, false, inp, hwRight);
            Point2D rightEndPt   = findExitEdgePt (conn->exitLineId,  false, inp, hwRight);

            // 对齐端点并局部平滑
            Point2D enterTangLeft  = getEnterEdgeTangent(conn->enterLineId, true,  inp);
            Point2D exitTangLeft   = getExitEdgeTangent (conn->exitLineId,  true,  inp);
            Point2D enterTangRight = getEnterEdgeTangent(conn->enterLineId, false, inp);
            Point2D exitTangRight  = getExitEdgeTangent (conn->exitLineId,  false, inp);

            int K = cfg_.edgeLine.endpointSmoothPoints;
            alignAndSmooth(leftPts,  leftStartPt,  enterTangLeft,
                                     leftEndPt,    exitTangLeft,   K);
            alignAndSmooth(rightPts, rightStartPt, enterTangRight,
                                     rightEndPt,   exitTangRight,  K);

            // 左边线
            {
                GeneratedEdgeLine el;
                el.id                = "gen_el_left_"+conn->id;
                el.geom              = leftPts;
                el.leftCenterlineId  = ""; // 最内侧
                el.rightCenterlineId = gcl.id;
                el.qualityFlags      = gcl.qualityFlags & QF_ERROR_NO_SOLUTION;
                result.push_back(el);
            }
            // 右边线
            {
                GeneratedEdgeLine el;
                el.id                = "gen_el_right_"+conn->id;
                el.geom              = rightPts;
                el.leftCenterlineId  = gcl.id;
                el.rightCenterlineId = ""; // 最外侧
                el.qualityFlags      = gcl.qualityFlags & QF_ERROR_NO_SOLUTION;
                result.push_back(el);
            }
        }
    }

    // 固定偏移法：将折线向左(offset>0)或右(offset<0)偏移
    Polyline offsetPolyline(const Polyline& pts, double offset) const {
        if(pts.size()<2) return pts;
        Polyline out;
        out.reserve(pts.size());

        for(size_t i=0;i<pts.size();++i){
            // 计算该点处的法线方向
            Point2D normal{0,1};
            if(i==0){
                Point2D dir=(pts[1]-pts[0]).normalized();
                normal=dir.rotLeft();
            } else if(i==pts.size()-1){
                Point2D dir=(pts[i]-pts[i-1]).normalized();
                normal=dir.rotLeft();
            } else {
                Point2D d1=(pts[i]-pts[i-1]).normalized();
                Point2D d2=(pts[i+1]-pts[i]).normalized();
                normal=((d1+d2)*0.5).normalized().rotLeft();
                if(normal.norm()<EPS) normal=d1.rotLeft();
                normal=normal.normalized();
            }
            out.push_back(pts[i]+normal*offset);
        }
        return out;
    }

    // 估算进入线一侧的半宽
    double estimateHalfWidth(const std::string& enterLineId, bool isLeft,
                              const LaneGroup& grp, const IntersectionInput& inp) const
    {
        // 方法1：若有路口外边线，从边线端点与中心线端点的距离估算
        auto clit = inp.centerlines.find(enterLineId);
        if(clit==inp.centerlines.end()) return cfg_.edgeLine.defaultLaneWidth * 0.5;

        const Point2D& clPt = clit->second.connectionPt;
        Point2D tangent = clit->second.tangentDir;
        Point2D normal  = tangent.rotLeft();

        // 找最近的边线端点
        double bestDist = -1;
        for(auto& eid : grp.edgelineIds){
            auto elit = inp.edgelines.find(eid);
            if(elit==inp.edgelines.end()) continue;
            if(elit->second.geom.empty()) continue;
            const Point2D& ep = elit->second.connectionPt;
            double lateral = (ep-clPt).dot(normal);
            if(isLeft && lateral > 0 && lateral < 10.0){
                if(bestDist<0 || lateral<bestDist) bestDist=lateral;
            } else if(!isLeft && lateral < 0 && lateral > -10.0){
                if(bestDist<0 || (-lateral)<bestDist) bestDist=-lateral;
            }
        }

        if(bestDist > 0) return bestDist;
        return cfg_.edgeLine.defaultLaneWidth * 0.5;
    }

    // 找进入线一侧在路口外的边线连接点
    Point2D findEnterEdgePt(const std::string& enterLineId, bool isLeft,
                             const IntersectionInput& inp, double hw) const
    {
        auto clit = inp.centerlines.find(enterLineId);
        if(clit==inp.centerlines.end()) return {0,0};

        const Point2D& clPt  = clit->second.connectionPt;
        const Point2D& tang  = clit->second.tangentDir;
        Point2D normal = tang.rotLeft();

        // 先查找实际边线
        for(auto& [id,el] : inp.edgelines){
            if(el.groupType != GroupType::ENTER) continue;
            if(el.geom.empty()) continue;
            const Point2D& ep = el.connectionPt;
            double lat = (ep-clPt).dot(normal);
            if(isLeft && lat>0 && std::abs(lat-hw)<hw*0.5) return ep;
            if(!isLeft && lat<0 && std::abs(lat+hw)<hw*0.5) return ep;
        }
        // 回退：用中心线连接点+法线偏移估算
        return clPt + normal*(isLeft?hw:-hw);
    }

    Point2D findExitEdgePt(const std::string& exitLineId, bool isLeft,
                            const IntersectionInput& inp, double hw) const
    {
        auto clit = inp.centerlines.find(exitLineId);
        if(clit==inp.centerlines.end()) return {0,0};

        const Point2D& clPt = clit->second.connectionPt;
        const Point2D& tang = clit->second.tangentDir; // 指向路口内
        Point2D normal = tang.rotLeft(); // 退出线的左侧

        for(auto& [id,el] : inp.edgelines){
            if(el.groupType != GroupType::EXIT) continue;
            if(el.geom.empty()) continue;
            const Point2D& ep = el.connectionPt;
            double lat = (ep-clPt).dot(normal);
            if(isLeft && lat>0 && std::abs(lat-hw)<hw*0.5) return ep;
            if(!isLeft && lat<0 && std::abs(lat+hw)<hw*0.5) return ep;
        }
        return clPt + normal*(isLeft?hw:-hw);
    }

    Point2D getEnterEdgeTangent(const std::string& enterLineId, bool isLeft,
                                 const IntersectionInput& inp) const {
        auto clit = inp.centerlines.find(enterLineId);
        if(clit==inp.centerlines.end()) return {0,1};
        return clit->second.tangentDir; // 与中心线同向
    }

    Point2D getExitEdgeTangent(const std::string& exitLineId, bool isLeft,
                                const IntersectionInput& inp) const {
        auto clit = inp.centerlines.find(exitLineId);
        if(clit==inp.centerlines.end()) return {0,1};
        return clit->second.tangentDir;
    }

    // 端点对齐 + 局部平滑（用贝塞尔重拟合首尾K个点段）
    void alignAndSmooth(Polyline& pts,
        const Point2D& startPt, const Point2D& startTang,
        const Point2D& endPt,   const Point2D& endTang,
        int K) const
    {
        if(pts.size()<2) return;

        int n = (int)pts.size();
        K = std::min(K, n/2);
        if(K<1) K=1;

        // 强制首端点对齐
        pts.front() = startPt;
        // 强制尾端点对齐
        pts.back() = endPt;

        if(K<2 || n<4) return;

        // 重拟合首端 [0..K]：用贝塞尔
        {
            Point2D p0 = startPt;
            Point2D p3 = pts[K];
            // 计算 p3 处的切线（由后续点估算）
            Point2D t3 = (K+1<n) ?
                (pts[K+1]-pts[K-1]).normalized() :
                (pts[K]-pts[K-1]).normalized();
            double d = dist(p0,p3);
            if(d > EPS){
                double alpha = 0.35;
                Point2D p1 = p0 + startTang*(alpha*d);
                Point2D p2 = p3 - t3*(alpha*d);
                CubicBezier cb(p0,p1,p2,p3);
                // 替换首端K个点
                for(int i=1;i<K;++i){
                    double t = (double)i/K;
                    pts[i] = cb.eval(t);
                }
            }
        }

        // 重拟合尾端 [n-1-K..n-1]
        {
            Point2D p0 = pts[n-1-K];
            Point2D p3 = endPt;
            Point2D t0 = (n-1-K>0) ?
                (pts[n-K]-pts[n-2-K]).normalized() :
                (pts[1]-pts[0]).normalized();
            // endTang 指向路口内，即从P3沿endTang方向进入，
            // 所以P2 = P3 + endTang * alpha * d
            double d = dist(p0,p3);
            if(d > EPS){
                double alpha = 0.35;
                Point2D p1 = p0 + t0*(alpha*d);
                Point2D p2 = p3 + endTang*(alpha*d); // endTang指向路口内
                CubicBezier cb(p0,p1,p2,p3);
                for(int i=1;i<K;++i){
                    double t = (double)i/K;
                    pts[n-1-K+i] = cb.eval(t);
                }
            }
        }
    }
};
