#pragma once
#include "../data_types.h"
#include "../../utils/geom_utils.h"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <limits>

/**
 * 障碍物均匀格网空间索引
 * 支持点/线/面三种障碍物的距离查询
 */
class ObstacleSpatialIndex {
    struct Cell {
        std::vector<int> obstacleIdx;
    };

    std::vector<Obstacle>            obstacles_;
    std::vector<Cell>                grid_;
    double bboxMinX_=0, bboxMinY_=0;
    double cellSize_=1.0;
    int    gridW_=1, gridH_=1;

    int cellIdx(int gx, int gy) const { return gy*gridW_+gx; }
    int toGX(double x) const { return std::max(0,std::min(gridW_-1,(int)((x-bboxMinX_)/cellSize_))); }
    int toGY(double y) const { return std::max(0,std::min(gridH_-1,(int)((y-bboxMinY_)/cellSize_))); }

    // 将一条折线对应的格网单元全部标记（供索引建立）
    void markLine(const Polyline& poly, int obsIdx){
        for(size_t i=0;i+1<poly.size();++i){
            double x0=poly[i].x,y0=poly[i].y;
            double x1=poly[i+1].x,y1=poly[i+1].y;
            int gx0=toGX(x0),gy0=toGY(y0);
            int gx1=toGX(x1),gy1=toGY(y1);
            // Bresenham-like covering
            int dx=std::abs(gx1-gx0), dy=std::abs(gy1-gy0);
            int sx=(gx0<gx1)?1:-1, sy=(gy0<gy1)?1:-1;
            int gx=gx0,gy=gy0;
            grid_[cellIdx(gx,gy)].obstacleIdx.push_back(obsIdx);
            while(gx!=gx1||gy!=gy1){
                if(dx>dy){ gx+=sx; dx-=dy; }
                else      { gy+=sy; dy-=dx; }
                dx=std::abs(gx1-gx); dy=std::abs(gy1-gy);
                grid_[cellIdx(gx,gy)].obstacleIdx.push_back(obsIdx);
            }
        }
    }

    void markPoint(const Point2D& p, int obsIdx){
        grid_[cellIdx(toGX(p.x),toGY(p.y))].obstacleIdx.push_back(obsIdx);
    }

    // 查询矩形范围内的候选障碍物
    std::vector<int> queryCandidates(double x0,double y0,double x1,double y1) const {
        std::unordered_set<int> seen;
        int gx0=std::max(0,toGX(x0)-1), gy0=std::max(0,toGY(y0)-1);
        int gx1=std::min(gridW_-1,toGX(x1)+1), gy1=std::min(gridH_-1,toGY(y1)+1);
        std::vector<int> result;
        for(int gy=gy0;gy<=gy1;++gy){
            for(int gx=gx0;gx<=gx1;++gx){
                for(int idx : grid_[cellIdx(gx,gy)].obstacleIdx){
                    if(seen.insert(idx).second) result.push_back(idx);
                }
            }
        }
        return result;
    }

public:
    ObstacleSpatialIndex() = default;

    void build(const std::vector<Obstacle>& obs, double bboxExt=5.0, int targetCells=400){
        obstacles_ = obs;
        if(obs.empty()) return;

        // 计算总包围盒
        double minX= 1e18,minY= 1e18,maxX=-1e18,maxY=-1e18;
        for(auto& o:obs){
            for(auto& p:o.geom){
                minX=std::min(minX,p.x); minY=std::min(minY,p.y);
                maxX=std::max(maxX,p.x); maxY=std::max(maxY,p.y);
            }
        }
        bboxMinX_ = minX - bboxExt;
        bboxMinY_ = minY - bboxExt;
        double W = maxX - minX + 2*bboxExt;
        double H = maxY - minY + 2*bboxExt;
        if(W<EPS) W=1; if(H<EPS) H=1;

        // 确定格网大小
        double ratio = W/H;
        gridH_ = std::max(1, (int)std::sqrt(targetCells/ratio));
        gridW_ = std::max(1, (int)(gridH_*ratio));
        cellSize_ = std::min(W/gridW_, H/gridH_);

        grid_.assign(gridW_*gridH_, Cell{});

        // 建立索引
        for(int i=0;i<(int)obs.size();++i){
            const Obstacle& o = obs[i];
            if(o.geom.empty()) continue;
            switch(o.geomType){
                case Obstacle::GeomType::POINT:
                    markPoint(o.geom[0], i);
                    break;
                case Obstacle::GeomType::LINE:
                    markLine(o.geom, i);
                    break;
                case Obstacle::GeomType::POLYGON:
                    markLine(o.geom, i); // 用边界索引（内部判断再处理）
                    break;
            }
        }
    }

    // 计算单点到所有障碍物的最短距离
    double minDist(const Point2D& p, double searchRadius=50.0) const {
        if(obstacles_.empty()) return std::numeric_limits<double>::max();
        auto cands = queryCandidates(p.x-searchRadius,p.y-searchRadius,
                                     p.x+searchRadius,p.y+searchRadius);
        double minD = std::numeric_limits<double>::max();
        for(int idx:cands){
            const Obstacle& o = obstacles_[idx];
            switch(o.geomType){
                case Obstacle::GeomType::POINT:
                    minD = std::min(minD, dist(p, o.geom[0]));
                    break;
                case Obstacle::GeomType::LINE:
                    minD = std::min(minD, pointToPolyline(p, o.geom));
                    break;
                case Obstacle::GeomType::POLYGON:
                    if(pointInPolygon(p, o.geom)) minD = 0.0;
                    else minD = std::min(minD, pointToPolyline(p, o.geom));
                    break;
            }
        }
        return minD;
    }

    // 检查折线采样点集合是否与障碍物距离过近
    // 返回违规点列表 {采样点索引, 到障碍物距离, 推开方向}
    struct Violation {
        int    ptIdx;
        double distToObs;
        Point2D pushDir;   // 从障碍物指向采样点的单位向量
        std::string obsId;
    };

    std::vector<Violation> checkViolations(
        const Polyline& pts, double safeMargin) const
    {
        std::vector<Violation> viols;
        if(obstacles_.empty()) return viols;

        for(int pi=0;pi<(int)pts.size();++pi){
            const Point2D& p = pts[pi];
            auto cands = queryCandidates(p.x-safeMargin*2,p.y-safeMargin*2,
                                         p.x+safeMargin*2,p.y+safeMargin*2);
            for(int idx:cands){
                const Obstacle& o = obstacles_[idx];
                double d = std::numeric_limits<double>::max();
                Point2D pushDir{0,1};

                switch(o.geomType){
                    case Obstacle::GeomType::POINT:{
                        d = dist(p, o.geom[0]);
                        if(d>EPS) pushDir=(p-o.geom[0]).normalized();
                        break;
                    }
                    case Obstacle::GeomType::LINE:{
                        d = std::numeric_limits<double>::max();
                        for(size_t k=0;k+1<o.geom.size();++k){
                            auto [dd,t] = pointToSegment(p,o.geom[k],o.geom[k+1]);
                            if(dd<d){
                                d=dd;
                                Point2D closest = o.geom[k]+(o.geom[k+1]-o.geom[k])*t;
                                if(dist(p,closest)>EPS) pushDir=(p-closest).normalized();
                            }
                        }
                        break;
                    }
                    case Obstacle::GeomType::POLYGON:{
                        if(pointInPolygon(p, o.geom)){
                            d = 0.0;
                            // 推移方向：到最近边界的方向
                            double minBd = std::numeric_limits<double>::max();
                            for(size_t k=0;k+1<o.geom.size();++k){
                                auto [dd,t]=pointToSegment(p,o.geom[k],o.geom[k+1]);
                                if(dd<minBd){
                                    minBd=dd;
                                    Point2D closest=o.geom[k]+(o.geom[k+1]-o.geom[k])*t;
                                    Point2D seg=o.geom[k+1]-o.geom[k];
                                    pushDir=seg.rotLeft().normalized(); // 法线向外
                                }
                            }
                        } else {
                            d = pointToPolyline(p, o.geom);
                            for(size_t k=0;k+1<o.geom.size();++k){
                                auto [dd,t]=pointToSegment(p,o.geom[k],o.geom[k+1]);
                                if(std::abs(dd-d)<EPS){
                                    Point2D closest=o.geom[k]+(o.geom[k+1]-o.geom[k])*t;
                                    if(dist(p,closest)>EPS) pushDir=(p-closest).normalized();
                                }
                            }
                        }
                        break;
                    }
                }

                if(d < safeMargin){
                    viols.push_back({pi, d, pushDir, o.id});
                }
            }
        }
        return viols;
    }

    bool empty() const { return obstacles_.empty(); }
    const std::vector<Obstacle>& obstacles() const { return obstacles_; }
};
