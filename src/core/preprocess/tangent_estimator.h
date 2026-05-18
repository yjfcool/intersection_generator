#pragma once
#include "../data_types.h"
#include "../../utils/geom_utils.h"
#include "../config.h"

/**
 * 切线估算模块
 * 计算进入/退出线在连接点处的切线方向（指向路口内侧）
 */
class TangentEstimator {
public:
    /**
     * 估算进入线在连接点（尾点）的切线，方向：远端→连接点（指向路口内）
     * @param geom 进入线几何（远端→连接点方向）
     * @param fitPoints 参与切线估算的端部点数
     */
    static Point2D enterTangent(const Polyline& geom, int fitPoints=2){
        if(geom.size()<2) return {0,1};
        if(fitPoints<=2 || (int)geom.size()<=2){
            // 取最后两点方向
            return (geom.back()-geom[geom.size()-2]).normalized();
        }
        // 加权平均（末端更近的段权重更大）
        int n = std::min(fitPoints, (int)geom.size()-1);
        Point2D dir{0,0};
        double totalW = 0;
        for(int i=0;i<n;++i){
            // 第i段：从 geom[size-2-i] 到 geom[size-1-i]
            size_t s = geom.size()-1-i;
            Point2D seg = (geom[s]-geom[s-1]).normalized();
            double w = (double)(i+1); // 越靠近连接点权重越大
            dir += seg * w;
            totalW += w;
        }
        return dir.normalized();
    }

    /**
     * 估算退出线在连接点（首点）的切线，方向：连接点→远端（路口内→路口外）
     * 生成曲线时取其反向作为"从路口内射出"的方向
     */
    static Point2D exitTangentRaw(const Polyline& geom, int fitPoints=2){
        if(geom.size()<2) return {0,1};
        if(fitPoints<=2 || (int)geom.size()<=2){
            return (geom[1]-geom[0]).normalized();
        }
        int n = std::min(fitPoints, (int)geom.size()-1);
        Point2D dir{0,0};
        double totalW = 0;
        for(int i=0;i<n;++i){
            Point2D seg = (geom[i+1]-geom[i]).normalized();
            double w = (double)(i+1);
            dir += seg * w;
            totalW += w;
        }
        return dir.normalized();
    }

    /**
     * 退出线在连接点处的切线（指向路口内侧，即进入方向）
     */
    static Point2D exitTangentInward(const Polyline& geom, int fitPoints=2){
        Point2D raw = exitTangentRaw(geom, fitPoints);
        return raw * (-1.0); // 取反：指向路口内
    }

    // 批量估算所有中心线和边线的连接点切线
    static void estimateAll(IntersectionInput& inp, const TangentConfig& cfg){
        int fp = cfg.fitPoints;

        for(auto& [id, cl] : inp.centerlines){
            if(cl.geom.size()<2) continue;
            if(cl.groupType == GroupType::ENTER){
                cl.connectionPt = cl.geom.back();
                cl.tangentDir   = enterTangent(cl.geom, fp);
            } else {
                cl.connectionPt = cl.geom.front();
                cl.tangentDir   = exitTangentInward(cl.geom, fp);
            }
        }

        for(auto& [id, el] : inp.edgelines){
            if(el.geom.size()<2) continue;
            if(el.groupType == GroupType::ENTER){
                el.connectionPt = el.geom.back();
                el.tangentDir   = enterTangent(el.geom, fp);
            } else {
                el.connectionPt = el.geom.front();
                el.tangentDir   = exitTangentInward(el.geom, fp);
            }
        }
    }
};
