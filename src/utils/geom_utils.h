#pragma once
#include "../core/data_types.h"
#include <cmath>
#include <limits>
#include <tuple>

static constexpr double PI = 3.14159265358979323846;
static constexpr double DEG2RAD = PI / 180.0;
static constexpr double RAD2DEG = 180.0 / PI;
static constexpr double EPS = 1e-9;

// ============================================================
// 基础几何工具
// ============================================================

// 点到线段的最短距离（和最近点参数t∈[0,1]）
inline std::pair<double,double> pointToSegment(
    const Point2D& p, const Point2D& a, const Point2D& b)
{
    Point2D ab = b - a;
    double len2 = ab.norm2();
    if(len2 < EPS*EPS){
        return {dist(p,a), 0.0};
    }
    double t = std::max(0.0, std::min(1.0, (p-a).dot(ab)/len2));
    Point2D closest = a + ab * t;
    return {dist(p, closest), t};
}

// 点到折线的最短距离
inline double pointToPolyline(const Point2D& p, const Polyline& poly){
    if(poly.empty()) return std::numeric_limits<double>::max();
    if(poly.size()==1) return dist(p, poly[0]);
    double minD = std::numeric_limits<double>::max();
    for(size_t i=0;i+1<poly.size();++i){
        auto [d, t] = pointToSegment(p, poly[i], poly[i+1]);
        minD = std::min(minD, d);
    }
    return minD;
}

// 两线段是否相交（不含共端点）
// 返回 true 表示严格相交（内部相交）
inline bool segmentsIntersectStrict(
    const Point2D& a, const Point2D& b,
    const Point2D& c, const Point2D& d)
{
    // 使用叉积方向判断
    auto sign = [](double v) -> int {
        if(v >  EPS) return  1;
        if(v < -EPS) return -1;
        return 0;
    };
    Point2D ab = b-a, ac = c-a, ad = d-a;
    Point2D cd = d-c, ca = a-c, cb = b-c;
    int d1 = sign(ab.cross(ac));
    int d2 = sign(ab.cross(ad));
    int d3 = sign(cd.cross(ca));
    int d4 = sign(cd.cross(cb));

    if(((d1>0&&d2<0)||(d1<0&&d2>0)) && ((d3>0&&d4<0)||(d3<0&&d4>0)))
        return true;
    return false;
}

// 两折线是否有内部相交（排除端点）
inline bool polylinesIntersect(const Polyline& A, const Polyline& B){
    if(A.size()<2 || B.size()<2) return false;
    for(size_t i=0;i+1<A.size();++i){
        for(size_t j=0;j+1<B.size();++j){
            if(segmentsIntersectStrict(A[i],A[i+1],B[j],B[j+1])){
                // 还需排除真正的端点（首点或尾点相交）
                // 检查交点是否恰好是折线的首/尾端点
                // 如果两段的端点相同则不算相交
                if(dist(A[i],  B[j])   < 1e-4 ||
                   dist(A[i],  B[j+1]) < 1e-4 ||
                   dist(A[i+1],B[j])   < 1e-4 ||
                   dist(A[i+1],B[j+1]) < 1e-4 ) continue;
                return true;
            }
        }
    }
    return false;
}

// 检查两条折线间排除首尾连接点后的相交
// endPtsA: A的允许相交端点集; endPtsB: B的
inline bool polylinesIntersectExcludeEndpoints(
    const Polyline& A, const Polyline& B,
    double endPtTol = 1e-4)
{
    if(A.size()<2 || B.size()<2) return false;
    for(size_t i=0;i+1<A.size();++i){
        for(size_t j=0;j+1<B.size();++j){
            if(!segmentsIntersectStrict(A[i],A[i+1],B[j],B[j+1])) continue;
            // 判断是否是端点附近的共点（允许的连接点相交）
            bool nearEndA = (i==0 || i+1==A.size()-1);
            bool nearEndB = (j==0 || j+1==B.size()-1);
            // 如果交点附近存在公共端点 → 跳过
            auto checkEndpts = [&](){
                const Point2D* ptsA[2] = {&A[i],&A[i+1]};
                const Point2D* ptsB[2] = {&B[j],&B[j+1]};
                // 仅当两折线的真正首/尾端点重合时才允许
                bool aStart = (dist(A.front(),A[i])<endPtTol || dist(A.front(),A[i+1])<endPtTol);
                bool aEnd   = (dist(A.back() ,A[i])<endPtTol || dist(A.back() ,A[i+1])<endPtTol);
                bool bStart = (dist(B.front(),B[j])<endPtTol || dist(B.front(),B[j+1])<endPtTol);
                bool bEnd   = (dist(B.back() ,B[j])<endPtTol || dist(B.back() ,B[j+1])<endPtTol);
                (void)ptsA; (void)ptsB;
                if((aStart||aEnd)&&(bStart||bEnd)){
                    // 检查端点是否重合
                    if(dist(A.front(),B.front())<endPtTol || dist(A.front(),B.back())<endPtTol
                    || dist(A.back(), B.front())<endPtTol || dist(A.back(), B.back())<endPtTol)
                        return true; // 端点相交，允许
                }
                return false;
            };
            (void)nearEndA; (void)nearEndB;
            if(checkEndpts()) continue;
            return true; // 真正的内部相交
        }
    }
    return false;
}

// 计算折线总长度
inline double polylineLength(const Polyline& p){
    double len = 0;
    for(size_t i=1;i<p.size();++i) len += dist(p[i-1],p[i]);
    return len;
}

// 点是否在多边形内（射线法）
inline bool pointInPolygon(const Point2D& pt, const Polyline& poly){
    if(poly.size()<3) return false;
    int cnt = 0;
    size_t n = poly.size();
    for(size_t i=0,j=n-1;i<n;j=i++){
        const Point2D& pi = poly[i];
        const Point2D& pj = poly[j];
        if(((pi.y>pt.y)!=(pj.y>pt.y)) &&
           (pt.x < (pj.x-pi.x)*(pt.y-pi.y)/(pj.y-pi.y)+pi.x))
            cnt++;
    }
    return cnt%2==1;
}

// 多边形面积（Shoelace公式，有符号）
inline double polygonSignedArea(const Polyline& poly){
    double A = 0;
    size_t n = poly.size();
    for(size_t i=0;i<n;++i){
        const Point2D& a = poly[i];
        const Point2D& b = poly[(i+1)%n];
        A += a.x*b.y - b.x*a.y;
    }
    return A * 0.5;
}
inline double polygonArea(const Polyline& p){ return std::abs(polygonSignedArea(p)); }

// 多边形质心
inline Point2D polygonCentroid(const Polyline& poly){
    if(poly.empty()) return {0,0};
    double A6 = 0, cx = 0, cy = 0;
    size_t n = poly.size();
    for(size_t i=0;i<n;++i){
        const Point2D& a = poly[i];
        const Point2D& b = poly[(i+1)%n];
        double cross = a.x*b.y - b.x*a.y;
        A6 += cross;
        cx += (a.x+b.x)*cross;
        cy += (a.y+b.y)*cross;
    }
    if(std::abs(A6)<EPS) {
        // 退化：取平均
        Point2D c{0,0};
        for(auto& p:poly){c.x+=p.x;c.y+=p.y;}
        return c*(1.0/n);
    }
    return {cx/(3*A6), cy/(3*A6)};
}

// 折线按arc-length重采样，间距spacing
inline Polyline resampleBySpacing(const Polyline& src, double spacing){
    if(src.size()<2 || spacing<EPS) return src;
    Polyline out;
    out.push_back(src.front());
    double acc = 0;
    for(size_t i=1;i<src.size();++i){
        double seg = dist(src[i-1],src[i]);
        double t = 0;
        while(acc+seg-t >= spacing){
            double dt = spacing-(acc-t*0);
            // 实际上：累积到下一个采样点
            double need = spacing - acc;
            if(need<0) need += spacing;
            // 简化：逐点累积
            (void)dt;
            break;
        }
        acc += seg;
    }
    // 重写：正确实现
    out.clear();
    out.push_back(src.front());
    double traveled = 0;
    double nextSample = spacing;
    for(size_t i=1;i<src.size();++i){
        double seg = dist(src[i-1],src[i]);
        while(traveled+seg >= nextSample){
            double t = (nextSample-traveled)/seg;
            Point2D p = src[i-1]*(1-t) + src[i]*t;
            out.push_back(p);
            nextSample += spacing;
        }
        traveled += seg;
    }
    if(dist(out.back(),src.back())>EPS*10) out.push_back(src.back());
    return out;
}

// 求两点间的插值（等分n段）
inline Polyline interpolateLine(const Point2D& a, const Point2D& b, int n){
    Polyline pts;
    for(int i=0;i<=n;++i){
        double t = i*1.0/n;
        pts.push_back(a*(1-t)+b*t);
    }
    return pts;
}

// 计算折线的包围盒
inline void polylineBBox(const Polyline& poly,
    double& minX, double& minY, double& maxX, double& maxY)
{
    minX=minY= std::numeric_limits<double>::max();
    maxX=maxY=-std::numeric_limits<double>::max();
    for(auto& p:poly){
        minX=std::min(minX,p.x); minY=std::min(minY,p.y);
        maxX=std::max(maxX,p.x); maxY=std::max(maxY,p.y);
    }
}

// 向量夹角（度）
inline double angleDeg(const Point2D& a, const Point2D& b){
    double c = a.normalized().dot(b.normalized());
    c = std::max(-1.0, std::min(1.0, c));
    return std::acos(c)*RAD2DEG;
}

// 判断点p是否在走廊左侧（boundary为折线，法线指向右侧）
inline bool isLeftOfPolyline(const Point2D& p, const Polyline& poly){
    if(poly.size()<2) return true;
    // 取整条折线最近的线段，判断点在该线段哪侧
    double minD = std::numeric_limits<double>::max();
    double sign = 1.0;
    for(size_t i=0;i+1<poly.size();++i){
        auto [d,t] = pointToSegment(p, poly[i], poly[i+1]);
        if(d<minD){
            minD = d;
            // 叉积判断左/右
            Point2D dir = (poly[i+1]-poly[i]).normalized();
            Point2D toP = p - (poly[i]+dir*((p-poly[i]).dot(dir)));
            sign = dir.cross(toP) > 0 ? 1.0 : -1.0;
        }
    }
    return sign > 0;
}
