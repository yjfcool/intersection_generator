#pragma once
#include "../data_types.h"
#include "../../utils/geom_utils.h"
#include <array>
#include <vector>
#include <cmath>
#include <stdexcept>

/**
 * 三次贝塞尔曲线
 * B(t) = (1-t)³P₀ + 3(1-t)²t·P₁ + 3(1-t)t²·P₂ + t³P₃
 */
class CubicBezier {
public:
    std::array<Point2D, 4> ctrl; // P0, P1, P2, P3

    CubicBezier() : ctrl{{{0,0},{0,0},{0,0},{0,0}}} {}
    CubicBezier(Point2D p0, Point2D p1, Point2D p2, Point2D p3)
        : ctrl{p0,p1,p2,p3} {}

    // B(t) 求值
    Point2D eval(double t) const {
        double u = 1.0-t;
        return ctrl[0]*(u*u*u) +
               ctrl[1]*(3*u*u*t) +
               ctrl[2]*(3*u*t*t) +
               ctrl[3]*(t*t*t);
    }

    // B'(t) 一阶导数
    Point2D evalDeriv1(double t) const {
        double u = 1.0-t;
        return (ctrl[1]-ctrl[0])*(3*u*u) +
               (ctrl[2]-ctrl[1])*(6*u*t) +
               (ctrl[3]-ctrl[2])*(3*t*t);
    }

    // B''(t) 二阶导数
    Point2D evalDeriv2(double t) const {
        return (ctrl[2]-ctrl[1]*2.0+ctrl[0])*(6*(1-t)) +
               (ctrl[3]-ctrl[2]*2.0+ctrl[1])*(6*t);
    }

    // 曲率 κ(t) = |B'×B''| / |B'|³
    double curvature(double t) const {
        Point2D d1 = evalDeriv1(t);
        Point2D d2 = evalDeriv2(t);
        double cross = std::abs(d1.cross(d2));
        double denom = std::pow(d1.norm(), 3);
        if(denom < EPS) return 0;
        return cross / denom;
    }

    // 最大曲率（采样估算）
    double maxCurvature(int samples=50) const {
        double maxK = 0;
        for(int i=0;i<=samples;++i){
            double t = i*1.0/samples;
            maxK = std::max(maxK, curvature(t));
        }
        return maxK;
    }

    // 弧长（数值积分 - 辛普森法）
    double arcLength(int n=100) const {
        if(n%2!=0) ++n;
        double h = 1.0/n;
        double sum = evalDeriv1(0).norm() + evalDeriv1(1).norm();
        for(int i=1;i<n;++i){
            double t = i*h;
            double w = (i%2==0) ? 2.0 : 4.0;
            sum += w * evalDeriv1(t).norm();
        }
        return sum*h/3.0;
    }

    // 固定数量采样
    Polyline sampleCount(int n) const {
        Polyline pts;
        pts.reserve(n+1);
        for(int i=0;i<=n;++i) pts.push_back(eval(i*1.0/n));
        return pts;
    }

    // 固定间距采样（弧长近似）
    Polyline sampleBySpacing(double spacing) const {
        // 先粗采样，再按间距重采样
        Polyline dense = sampleCount(200);
        return resampleBySpacing(dense, spacing);
    }

    // 自适应采样（基于角度误差）
    Polyline sampleAdaptive(double maxAngleDeg, double maxSeg, double minSeg) const {
        double maxAngleRad = maxAngleDeg * DEG2RAD;
        Polyline pts;
        pts.push_back(ctrl[0]);
        subdivide(0.0, 1.0, maxAngleRad, maxSeg, minSeg, pts, 0);
        // 确保端点精确
        if(pts.empty() || dist(pts.back(), ctrl[3])>EPS)
            pts.push_back(ctrl[3]);
        return pts;
    }

    // 检查G1：进入端
    bool checkG1Enter(const Point2D& enterTangent, double tolDeg=2.0) const {
        Point2D d = evalDeriv1(0.0);
        if(d.norm()<EPS) return false;
        return angleDeg(d.normalized(), enterTangent) < tolDeg;
    }
    // 检查G1：退出端（退出切线：指向路口外，即路口内曲线末端切线方向取反）
    bool checkG1Exit(const Point2D& exitTangentInward, double tolDeg=2.0) const {
        Point2D d = evalDeriv1(1.0);
        if(d.norm()<EPS) return false;
        // exitTangentInward 指向路口内（从退出线看进来）
        // 曲线末端切线 d 应与 exitTangentInward 相反
        return angleDeg(d.normalized(), exitTangentInward*(-1.0)) < tolDeg;
    }

    bool isValid() const {
        // 检查控制点不退化
        return dist(ctrl[0],ctrl[3]) > EPS;
    }

    // 通过 α,β 参数重新设置内部控制点（保持端点和切线方向）
    static CubicBezier fromAlphaBeta(
        const Point2D& P0, const Point2D& T0,
        const Point2D& P3, const Point2D& T3,
        double alpha, double beta)
    {
        double d = dist(P0, P3);
        Point2D P1 = P0 + T0 * (alpha * d);
        Point2D P2 = P3 - T3 * (beta  * d); // T3指向路口内，G1：B'(1)=3(P3-P2)与T3同向
        return CubicBezier(P0, P1, P2, P3);
    }

    // 获取 α（P1相对P0的标量，T0方向）
    double getAlpha(const Point2D& T0) const {
        double d = dist(ctrl[0], ctrl[3]);
        if(d < EPS) return 0.35;
        return (ctrl[1]-ctrl[0]).dot(T0) / d;
    }
    double getBeta(const Point2D& T3) const {
        double d = dist(ctrl[0], ctrl[3]);
        if(d < EPS) return 0.35;
        // P2 = P3 + T3*beta*d → beta = (P2-P3)·T3 / d
        return (ctrl[2]-ctrl[3]).dot(T3.normalized()) / d;
    }

private:
    void subdivide(double t0, double t1, double maxAngle, double maxSeg, double minSeg,
                   Polyline& pts, int depth) const
    {
        if(depth > 20) {
            pts.push_back(eval(t1));
            return;
        }
        Point2D p0 = eval(t0);
        Point2D p1 = eval(t1);
        double segLen = dist(p0, p1);

        if(segLen < minSeg){
            pts.push_back(p1);
            return;
        }

        double tMid = 0.5*(t0+t1);
        Point2D pMid = eval(tMid);

        // 检查角度误差
        Point2D d01 = (pMid - p0).normalized();
        Point2D d12 = (p1 - pMid).normalized();
        double angle = std::acos(std::max(-1.0, std::min(1.0, d01.dot(d12))));

        bool needSplit = (angle > maxAngle) || (segLen > maxSeg);

        if(!needSplit){
            pts.push_back(p1);
        } else {
            subdivide(t0, tMid, maxAngle, maxSeg, minSeg, pts, depth+1);
            subdivide(tMid, t1,  maxAngle, maxSeg, minSeg, pts, depth+1);
        }
    }
};

/**
 * 两段复合三次贝塞尔（G1拼接）
 * seg1: P0→M, seg2: M→P3
 * G1条件：seg1末切线 = seg2首切线方向
 */
class CompositeBezier {
public:
    CubicBezier seg1, seg2;

    CompositeBezier() = default;
    CompositeBezier(const CubicBezier& s1, const CubicBezier& s2)
        : seg1(s1), seg2(s2) {}

    Point2D eval(double t) const {
        if(t <= 0.5) return seg1.eval(t*2.0);
        else         return seg2.eval((t-0.5)*2.0);
    }

    bool checkG1AtMid(double tolDeg=2.0) const {
        Point2D d1 = seg1.evalDeriv1(1.0).normalized();
        Point2D d2 = seg2.evalDeriv1(0.0).normalized();
        return angleDeg(d1, d2) < tolDeg;
    }

    double maxCurvature(int samples=100) const {
        return std::max(seg1.maxCurvature(samples/2), seg2.maxCurvature(samples/2));
    }

    Polyline sampleAdaptive(double maxAngleDeg, double maxSeg, double minSeg) const {
        Polyline pts;
        pts.push_back(seg1.ctrl[0]);

        Polyline p1 = seg1.sampleAdaptive(maxAngleDeg, maxSeg, minSeg);
        for(auto& p : p1) pts.push_back(p);

        Polyline p2 = seg2.sampleAdaptive(maxAngleDeg, maxSeg, minSeg);
        for(size_t i=1;i<p2.size();++i) pts.push_back(p2[i]); // 跳过重复拼接点

        return pts;
    }

    Polyline sampleBySpacing(double spacing) const {
        Polyline p1 = seg1.sampleBySpacing(spacing);
        Polyline p2 = seg2.sampleBySpacing(spacing);
        for(size_t i=1;i<p2.size();++i) p1.push_back(p2[i]);
        return p1;
    }

    Polyline sampleCount(int n) const {
        Polyline p1 = seg1.sampleCount(n/2);
        Polyline p2 = seg2.sampleCount(n/2);
        for(size_t i=1;i<p2.size();++i) p1.push_back(p2[i]);
        return p1;
    }
};
