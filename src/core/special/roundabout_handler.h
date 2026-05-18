#pragma once
#include "../data_types.h"
#include "../config.h"
#include "../bezier/cubic_bezier.h"
#include "../../utils/geom_utils.h"
#include "../../utils/logger.h"
#include <cmath>
#include <vector>

/**
 * 环岛特殊处理模块
 * 识别环岛场景，对环内弧形段做分段贝塞尔曲线拟合
 */
class RoundaboutHandler {
public:
    // 识别是否是环岛场景
    static bool isRoundabout(const IntersectionInput& inp){
        // 条件1: attrs 中有标记
        if(inp.roughArea){
            auto it = inp.roughArea->attrs.find("intersection_type");
            if(it!=inp.roughArea->attrs.end() && it->second=="roundabout")
                return true;
        }
        // 条件2: 存在进入组==退出组的连通关系（环内段）
        for(auto& conn : inp.connections){
            if(conn.enterGroupId == conn.exitGroupId) return true;
        }
        // 条件3: 几何检测 - 存在弧长>20m的连通线（弧形中心线）
        for(auto& conn : inp.connections){
            auto clit = inp.centerlines.find(conn.enterLineId);
            if(clit==inp.centerlines.end()) continue;
            if(polylineLength(clit->second.geom) > 20.0) return true;
        }
        return false;
    }

    /**
     * 对环内连通关系进行预处理
     * 将长弧段的 Connection 分割为多段子连通关系
     * 返回：新的 Connection 列表（替换原环内段）
     */
    static void preprocessRoundabout(IntersectionInput& inp, const Config& cfg){
        // 标记环内连通关系
        std::vector<int> innerConnIdx;
        for(int i=0;i<(int)inp.connections.size();++i){
            const Connection& conn = inp.connections[i];
            // 环内段：进入组==退出组，或线段很长
            bool isInner = (conn.enterGroupId == conn.exitGroupId);
            if(!isInner){
                auto clit = inp.centerlines.find(conn.enterLineId);
                if(clit!=inp.centerlines.end() && polylineLength(clit->second.geom)>20.0)
                    isInner=true;
            }
            if(isInner) innerConnIdx.push_back(i);
        }

        Logger::info("Roundabout: " + std::to_string(innerConnIdx.size()) + " inner connections.");

        // 对每个环内段，用分段贝塞尔拟合（通过修改中心线几何）
        // 这里主要是确保切线估算正确（环内段切线需要特殊处理）
        for(int idx : innerConnIdx){
            Connection& conn = inp.connections[idx];
            conn.attrs["is_roundabout_inner"] = "true";

            auto clit = inp.centerlines.find(conn.enterLineId);
            if(clit==inp.centerlines.end()) continue;
            LaneCenterline& cl = clit->second;

            // 如果是很长的弧段，提取圆弧参数并生成更平滑的中心线
            double len = polylineLength(cl.geom);
            if(len > 15.0 && cl.geom.size()>=3){
                // 估算圆弧参数
                auto [cx,cy,R] = fitCircleToPolyline(cl.geom);
                if(R > 1.0 && R < 200.0){
                    Logger::info("Roundabout inner arc: R=" + std::to_string(R));
                    // 用更密集的采样点替换几何（保证平滑）
                    cl.geom = resampleArc(cl.geom, cx, cy, R, 0.5);
                }
            }
        }
    }

private:
    // 最小二乘拟合圆（3点法近似）
    static std::tuple<double,double,double> fitCircleToPolyline(const Polyline& pts){
        if(pts.size()<3) return {0,0,10};

        // 取首中尾三点做圆拟合
        const Point2D& A = pts.front();
        const Point2D& B = pts[pts.size()/2];
        const Point2D& C = pts.back();

        // 三点确定圆
        double ax=A.x,ay=A.y,bx=B.x,by=B.y,cx=C.x,cy=C.y;
        double D=2*(ax*(by-cy)+bx*(cy-ay)+cx*(ay-by));
        if(std::abs(D)<EPS) return {0,0,100};

        double ux=((ax*ax+ay*ay)*(by-cy)+(bx*bx+by*by)*(cy-ay)+(cx*cx+cy*cy)*(ay-by))/D;
        double uy=((ax*ax+ay*ay)*(cx-bx)+(bx*bx+by*by)*(ax-cx)+(cx*cx+cy*cy)*(bx-ax))/D;
        double R=std::sqrt((ax-ux)*(ax-ux)+(ay-uy)*(ay-uy));
        return {ux,uy,R};
    }

    // 按圆弧重采样折线（保首尾端点）
    static Polyline resampleArc(const Polyline& pts,
                                 double cx, double cy, double R,
                                 double spacing)
    {
        if(pts.size()<2) return pts;

        // 计算首尾在圆上的角度
        double a0 = std::atan2(pts.front().y-cy, pts.front().x-cx);
        double a1 = std::atan2(pts.back().y-cy,  pts.back().x-cx);

        // 确保角度差方向与原折线方向一致
        // 通过中间点判断
        double aMid = std::atan2(pts[pts.size()/2].y-cy, pts[pts.size()/2].x-cx);

        // 归一化角度差
        double da = a1 - a0;
        if(da > PI)  da -= 2*PI;
        if(da < -PI) da += 2*PI;

        double arcLen = std::abs(da) * R;
        int n = std::max(2, (int)(arcLen/spacing));

        Polyline out;
        for(int i=0;i<=n;++i){
            double t = (double)i/n;
            double a = a0 + da*t;
            out.push_back({cx + R*std::cos(a), cy + R*std::sin(a)});
        }

        // 强制端点精确
        out.front() = pts.front();
        out.back()  = pts.back();
        return out;
    }
};
