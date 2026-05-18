#pragma once
#include "../data_types.h"
#include "../config.h"
#include "../../utils/geom_utils.h"
#include "../../utils/logger.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <set>

/**
 * 精细路口面构建
 * 输入：道路边缘线（独立线段）+ 生成的车道边线端点
 * 输出：封闭多边形（路口面）
 *
 * 流程：
 *  1. 确定路口范围（rough_area 或 由连接点包围盒扩展）
 *  2. 裁剪/截取每条道路边缘线在路口侧的端点
 *  3. 按极角排序端点，连接成封闭多边形
 *  4. 相邻端点间若属同一边缘线则沿线连接，否则直连
 */
class IntersectionPolygonBuilder {
    const PolygonConfig& cfg_;

public:
    explicit IntersectionPolygonBuilder(const PolygonConfig& cfg) : cfg_(cfg) {}

    IntersectionPolygon build(
        const IntersectionInput& inp,
        const std::vector<GeneratedCenterline>& centerlines,
        const std::vector<GeneratedEdgeLine>&   edgelines)
    {
        IntersectionPolygon poly;
        poly.id = "int_polygon_" + inp.scenarioId;

        // 1. 确定路口中心和范围
        Point2D center = computeCenter(inp, centerlines);
        double  radius = computeRadius(inp, centerlines, center);

        Logger::info("Polygon: center=(" + std::to_string(center.x) + "," +
                     std::to_string(center.y) + ") radius=" + std::to_string(radius));

        // 2. 裁剪道路边缘线，获取路口侧端点集
        std::vector<EdgeEndpoint> endpoints;

        for(int i=0; i<(int)inp.roadEdges.size(); ++i){
            const RoadEdgeLine& re = inp.roadEdges[i];
            if(re.geom.size() < 2) continue;

            // 找到路口侧的端点（靠近center的端点）
            double d0 = dist(re.geom.front(), center);
            double d1 = dist(re.geom.back(),  center);

            Point2D candidatePt;
            bool    isStart;
            if(d0 < d1){
                candidatePt = re.geom.front();
                isStart = true;
            } else {
                candidatePt = re.geom.back();
                isStart = false;
            }

            // 检查是否在路口范围内（在 rough_area 内或在 radius 内）
            bool inRange = false;
            if(inp.roughArea && !inp.roughArea->geom.empty()){
                inRange = pointInPolygon(candidatePt, inp.roughArea->geom) ||
                          dist(candidatePt, center) < radius * 1.5;
            } else {
                inRange = dist(candidatePt, center) < radius * 1.5;
            }

            if(inRange){
                // 吸附到最近的车道边线端点
                Point2D snapped = snapToEdgePt(candidatePt, edgelines,
                                               inp, cfg_.snapTolerance);
                endpoints.push_back({snapped, i, isStart});
                Logger::debug("Edge endpoint[" + std::to_string(i) + "]: ("
                    + std::to_string(snapped.x) + "," + std::to_string(snapped.y) + ")");
            }
        }

        // 若道路边缘线端点不足，从连通关系连接点的外侧边线端点补充
        if(endpoints.size() < 3){
            Logger::warn("Road edge endpoints insufficient (" +
                         std::to_string(endpoints.size()) + "), supplementing from lane edges.");
            supplementEndpointsFromLaneEdges(endpoints, inp, edgelines, center, radius);
        }

        if(endpoints.size() < 3){
            // 终极回退：用所有连接点的凸包
            Logger::warn("Still insufficient endpoints, using convex hull of connection points.");
            poly.geom = buildConvexHullFallback(inp, centerlines);
            poly.area = polygonArea(poly.geom);
            poly.qualityFlags = QF_WARN_CORRIDOR_TOO_NARROW;
            return poly;
        }

        // 3. 按极角排序
        std::sort(endpoints.begin(), endpoints.end(),
            [&](const EdgeEndpoint& a, const EdgeEndpoint& b){
                double angA = std::atan2(a.pt.y - center.y, a.pt.x - center.x);
                double angB = std::atan2(b.pt.y - center.y, b.pt.x - center.x);
                if(cfg_.winding == "clockwise")
                    return angA > angB;
                else
                    return angA < angB;
            });

        // 去重（距离过近的端点合并）
        deduplicateEndpoints(endpoints, 0.1);

        // 4. 连接端点构建多边形
        Polyline rawPoly;
        for(int i=0; i<(int)endpoints.size(); ++i){
            const EdgeEndpoint& cur  = endpoints[i];
            const EdgeEndpoint& next = endpoints[(i+1) % endpoints.size()];

            rawPoly.push_back(cur.pt);

            // 若相邻端点来自同一条道路边缘线，沿边缘线几何连接
            if(cur.roadEdgeIdx >= 0 && cur.roadEdgeIdx == next.roadEdgeIdx){
                Polyline seg = extractEdgeSegment(
                    inp.roadEdges[cur.roadEdgeIdx],
                    cur.pt, next.pt,
                    cfg_.snapTolerance);
                for(auto& p : seg) rawPoly.push_back(p);
            }
            // 否则直连（或弧线连，按参数）
        }

        // 5. 闭合多边形
        if(!rawPoly.empty() && dist(rawPoly.front(), rawPoly.back()) > EPS)
            rawPoly.push_back(rawPoly.front());

        // 6. 多边形修复（移除共线点，确保逆/顺时针）
        poly.geom = repairPolygon(rawPoly, center);

        // 7. 确保多边形包含所有中心线采样点（扩展检查）
        expandToContainCenterlines(poly.geom, centerlines, center);

        poly.area = polygonArea(poly.geom);
        Logger::info("Polygon built: vertices=" + std::to_string(poly.geom.size())
                     + " area=" + std::to_string(poly.area));

        if(poly.area < 1.0){
            poly.qualityFlags = QF_ERROR_NO_SOLUTION;
            Logger::warn("Polygon area too small: " + std::to_string(poly.area));
        }

        return poly;
    }

private:
    struct EdgeEndpoint {
        Point2D pt;
        int     roadEdgeIdx = -1;
        bool    isStart = true;
    };

    // 计算路口中心点（连接点的质心）
    Point2D computeCenter(const IntersectionInput& inp,
                          const std::vector<GeneratedCenterline>& cls) const
    {
        Point2D c{0,0};
        int cnt = 0;
        for(auto& gcl : cls){
            if(!gcl.geom.empty()){
                c += gcl.geom.front(); ++cnt;
                c += gcl.geom.back();  ++cnt;
            }
        }
        if(cnt > 0) return c * (1.0/cnt);

        // 回退：用中心线连接点
        for(auto& [id,cl] : inp.centerlines){
            c += cl.connectionPt; ++cnt;
        }
        return cnt > 0 ? c*(1.0/cnt) : Point2D{0,0};
    }

    // 估算路口半径（连接点到中心的最大距离）
    double computeRadius(const IntersectionInput& inp,
                         const std::vector<GeneratedCenterline>& cls,
                         const Point2D& center) const
    {
        double maxD = 5.0;
        for(auto& gcl : cls){
            if(!gcl.geom.empty()){
                maxD = std::max(maxD, dist(gcl.geom.front(), center));
                maxD = std::max(maxD, dist(gcl.geom.back(),  center));
            }
        }
        for(auto& [id,cl] : inp.centerlines){
            maxD = std::max(maxD, dist(cl.connectionPt, center));
        }
        return maxD;
    }

    // 将端点吸附到最近的车道边线端点（若在容差范围内）
    Point2D snapToEdgePt(const Point2D& pt,
                          const std::vector<GeneratedEdgeLine>& edgelines,
                          const IntersectionInput& inp,
                          double tol) const
    {
        double minD = tol;
        Point2D best = pt;

        // 检查生成的路口内边线端点
        for(auto& el : edgelines){
            if(el.geom.empty()) continue;
            double d0 = dist(pt, el.geom.front());
            double d1 = dist(pt, el.geom.back());
            if(d0 < minD){ minD=d0; best=el.geom.front(); }
            if(d1 < minD){ minD=d1; best=el.geom.back(); }
        }

        // 检查路口外边线连接点
        for(auto& [id,el] : inp.edgelines){
            if(el.geom.empty()) continue;
            double d = dist(pt, el.connectionPt);
            if(d < minD){ minD=d; best=el.connectionPt; }
        }

        return best;
    }

    // 从车道边线端点补充路口面顶点
    void supplementEndpointsFromLaneEdges(
        std::vector<struct EdgeEndpoint>& endpoints,
        const IntersectionInput& inp,
        const std::vector<GeneratedEdgeLine>& edgelines,
        const Point2D& center, double radius) const
    {
        // 收集路口外边线的连接点（这些是路口面的边界顶点）
        for(auto& [id,el] : inp.edgelines){
            if(el.geom.empty()) continue;
            Point2D cp = el.connectionPt;
            if(dist(cp,center) > radius*2.0) continue;

            // 检查是否已有相近的端点
            bool dup = false;
            for(auto& ep : endpoints){
                if(dist(ep.pt,cp)<0.5){ dup=true; break; }
            }
            if(!dup) endpoints.push_back({cp,-1,true});
        }

        // 补充中心线连接点的法线方向偏移点（近似边界点）
        for(auto& [id,cl] : inp.centerlines){
            if(cl.geom.size()<2) continue;
            Point2D cp = cl.connectionPt;
            Point2D n  = cl.tangentDir.rotLeft();
            // 左侧点
            Point2D lp = cp + n*cfg_.snapTolerance*2;
            Point2D rp = cp - n*cfg_.snapTolerance*2;
            bool dupL=false,dupR=false;
            for(auto& ep:endpoints){
                if(dist(ep.pt,lp)<0.5) dupL=true;
                if(dist(ep.pt,rp)<0.5) dupR=true;
            }
            if(!dupL) endpoints.push_back({lp,-1,true});
            if(!dupR) endpoints.push_back({rp,-1,true});
        }
    }

    // 从道路边缘线提取两点间的几何段
    Polyline extractEdgeSegment(const RoadEdgeLine& re,
                                 const Point2D& a, const Point2D& b,
                                 double tol) const
    {
        if(re.geom.size()<2) return {};

        // 找到 a 和 b 在 re.geom 上最近的索引
        int idxA=-1, idxB=-1;
        double minDA=1e18, minDB=1e18;
        for(int i=0;i<(int)re.geom.size();++i){
            double da=dist(re.geom[i],a);
            double db=dist(re.geom[i],b);
            if(da<minDA){ minDA=da; idxA=i; }
            if(db<minDB){ minDB=db; idxB=i; }
        }

        if(idxA<0||idxB<0||idxA==idxB) return {};

        Polyline seg;
        int step = (idxA<idxB)?1:-1;
        for(int i=idxA; i!=idxB; i+=step){
            seg.push_back(re.geom[i]);
        }
        // 不包含 b 本身（由下一个端点自己加）
        return seg;
    }

    // 去重端点（合并距离过近的点）
    void deduplicateEndpoints(std::vector<struct EdgeEndpoint>& eps, double tol) const {
        std::vector<bool> remove(eps.size(), false);
        for(size_t i=0;i<eps.size();++i){
            if(remove[i]) continue;
            for(size_t j=i+1;j<eps.size();++j){
                if(!remove[j] && dist(eps[i].pt,eps[j].pt)<tol){
                    remove[j]=true;
                }
            }
        }
        std::vector<struct EdgeEndpoint> out;
        for(size_t i=0;i<eps.size();++i)
            if(!remove[i]) out.push_back(eps[i]);
        eps = out;
    }

    // 多边形修复：移除共线点，确保方向正确
    Polyline repairPolygon(const Polyline& raw, const Point2D& center) const {
        if(raw.size()<3) return raw;

        // 移除重复点
        Polyline pts;
        for(auto& p : raw){
            if(pts.empty() || dist(pts.back(),p)>EPS) pts.push_back(p);
        }
        // 移除首尾重复
        while(pts.size()>1 && dist(pts.front(),pts.back())<EPS)
            pts.pop_back();

        if(pts.size()<3) return raw;

        // 检查方向（有符号面积）
        double area = polygonSignedArea(pts);
        bool isCCW = area > 0;

        if(cfg_.winding=="clockwise" && isCCW){
            std::reverse(pts.begin(), pts.end());
        } else if(cfg_.winding=="counter_clockwise" && !isCCW){
            std::reverse(pts.begin(), pts.end());
        }

        // 闭合
        if(dist(pts.front(),pts.back())>EPS)
            pts.push_back(pts.front());

        return pts;
    }

    // 扩展多边形以包含所有中心线点（简单的凸包扩展）
    void expandToContainCenterlines(
        Polyline& poly,
        const std::vector<GeneratedCenterline>& cls,
        const Point2D& center) const
    {
        if(poly.size()<3) return;

        bool expanded = false;
        for(auto& gcl : cls){
            for(auto& p : gcl.geom){
                if(!pointInPolygon(p, poly)){
                    // 将该点加入多边形（扩展最近边）
                    expandByPoint(poly, p, center);
                    expanded = true;
                }
            }
        }
        if(expanded){
            // 重新排序（保证凸包方向）
            poly = repairPolygon(poly, center);
        }
    }

    void expandByPoint(Polyline& poly, const Point2D& p, const Point2D& center) const {
        if(poly.size()<3) return;

        // 找到最近边，在该边插入点
        double minD = std::numeric_limits<double>::max();
        int insertAfter = 0;
        size_t n = poly.size();
        // 跳过最后一个（它是闭合点，等于第一个）
        size_t end = (dist(poly.front(),poly.back())<EPS) ? n-1 : n;
        for(size_t i=0;i<end;++i){
            size_t j=(i+1)%end;
            auto [d,t] = pointToSegment(p,poly[i],poly[j]);
            if(d<minD){ minD=d; insertAfter=(int)i; }
        }
        poly.insert(poly.begin()+insertAfter+1, p);
    }

    // 最终回退：所有连接点的凸包
    Polyline buildConvexHullFallback(
        const IntersectionInput& inp,
        const std::vector<GeneratedCenterline>& cls) const
    {
        std::vector<Point2D> pts;
        for(auto& gcl : cls){
            if(!gcl.geom.empty()){
                pts.push_back(gcl.geom.front());
                pts.push_back(gcl.geom.back());
            }
        }
        for(auto& [id,cl] : inp.centerlines){
            pts.push_back(cl.connectionPt);
        }
        if(pts.empty()) return {};
        return convexHull(pts);
    }

    // Andrew's monotone chain 凸包
    Polyline convexHull(std::vector<Point2D> pts) const {
        int n = pts.size();
        if(n<3) return pts;
        std::sort(pts.begin(),pts.end(),[](const Point2D&a,const Point2D&b){
            return a.x<b.x||(a.x==b.x&&a.y<b.y);
        });

        std::vector<Point2D> hull;
        // Lower hull
        for(int i=0;i<n;++i){
            while(hull.size()>=2){
                Point2D a=hull[hull.size()-2], b=hull.back();
                if((b-a).cross(pts[i]-a)<=0) hull.pop_back();
                else break;
            }
            hull.push_back(pts[i]);
        }
        // Upper hull
        int lower = hull.size();
        for(int i=n-2;i>=0;--i){
            while((int)hull.size()>lower){
                Point2D a=hull[hull.size()-2], b=hull.back();
                if((b-a).cross(pts[i]-a)<=0) hull.pop_back();
                else break;
            }
            hull.push_back(pts[i]);
        }
        hull.pop_back();
        if(!hull.empty()) hull.push_back(hull.front()); // 闭合
        return hull;
    }
};
