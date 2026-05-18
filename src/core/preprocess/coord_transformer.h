#pragma once
#include "../data_types.h"
#include "../../utils/geom_utils.h"
#include <cmath>

/**
 * 自实现方位角等距投影（AEQD）
 * 将WGS84经纬度坐标转换为以参考点为原点的平面坐标（单位：米）
 * 适用于路口级别（数百米范围）的精度需求
 */
class CoordTransformer {
    double refLon_, refLat_;  // 参考点（度）
    double refLonR_, refLatR_;// 参考点（弧度）
    static constexpr double R = 6378137.0; // WGS84长半轴（米）

public:
    CoordTransformer() : refLon_(0), refLat_(0), refLonR_(0), refLatR_(0) {}
    
    CoordTransformer(double refLon, double refLat)
        : refLon_(refLon), refLat_(refLat)
        , refLonR_(refLon*DEG2RAD), refLatR_(refLat*DEG2RAD)
    {}

    // WGS84(度) → 局部平面坐标（米）
    Point2D toLocal(double lon, double lat) const {
        double lonR = lon * DEG2RAD;
        double latR = lat * DEG2RAD;
        
        // 简化的球面投影（精度满足路口级别<10km范围）
        double dLon = lonR - refLonR_;
        double dLat = latR - refLatR_;
        
        // 经度差转东向距离（米）
        double cosLat = std::cos(refLatR_);
        double x = dLon * R * cosLat;
        // 纬度差转北向距离（米）
        double y = dLat * R;
        
        return {x, y};
    }

    // 局部平面坐标（米） → WGS84(度)
    std::pair<double,double> toWGS84(const Point2D& p) const {
        double cosLat = std::cos(refLatR_);
        double lon = refLon_ + (p.x / (R * cosLat)) * RAD2DEG;
        double lat = refLat_ + (p.y / R) * RAD2DEG;
        return {lon, lat};
    }

    Polyline toLocalLine(const Polyline& wgs84pts) const {
        Polyline out;
        out.reserve(wgs84pts.size());
        for(auto& p : wgs84pts) out.push_back(toLocal(p.x, p.y));
        return out;
    }

    // 变换整个IntersectionInput（修改in-place）
    void transformInput(IntersectionInput& inp) const {
        auto transLine = [&](Polyline& pl){
            for(auto& p : pl) p = toLocal(p.x, p.y);
        };

        for(auto& [id,cl] : inp.centerlines)  transLine(cl.geom);
        for(auto& [id,el] : inp.edgelines)    transLine(el.geom);
        for(auto& obs : inp.obstacles)         transLine(obs.geom);
        for(auto& sl  : inp.stopLines)         transLine(sl.geom);
        for(auto& re  : inp.roadEdges)         transLine(re.geom);
        if(inp.roughArea) transLine(inp.roughArea->geom);
    }

    double getRefLon() const { return refLon_; }
    double getRefLat() const { return refLat_; }
};
