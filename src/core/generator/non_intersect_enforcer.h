#pragma once
#include "../data_types.h"
#include "../bezier/cubic_bezier.h"
#include "../config.h"
#include "../../utils/geom_utils.h"
#include "../../utils/logger.h"
#include <vector>
#include <cmath>

struct EnforceResult {
    Polyline    finalPts;
    CubicBezier curve;
    bool        intersectionRemains = false;
};

/**
 * 非相交约束执行器
 * - 使用 alpha/beta + 受限法线偏移 (gamma) 调整控制点
 * - G1角度限制 < 5°（通过 gamma_max = alpha*d*tan(5°) 约束）
 * - 端点始终锁定在 P0/P3
 */
class NonIntersectEnforcer {
    const NonIntersectConfig& cfg_;

    // 最大允许G1角度（度）
    static constexpr double MAX_G1_DEG = 5.0;

public:
    explicit NonIntersectEnforcer(const NonIntersectConfig& cfg) : cfg_(cfg) {}

    EnforceResult enforce(
        const CubicBezier& candidate,
        const Corridor&    corridor,
        const std::vector<GeneratedCenterline>& existing,
        const Connection&  conn,
        const IntersectionInput& inp,
        const Point2D& T0,
        const Point2D& T3,
        const std::string& samplingMode,
        double samplingParam)
    {
        EnforceResult result;
        result.curve = candidate;

        if (conn.isMidUturn && cfg_.enableMidUturnExclude) {
            result.finalPts = makeSample(candidate, samplingMode, samplingParam);
            return result;
        }

        auto conflicts = collectConflicts(conn, existing, inp);
        if (conflicts.empty()) {
            result.finalPts = makeSample(candidate, samplingMode, samplingParam);
            return result;
        }

        const Point2D& P0 = candidate.ctrl[0];
        const Point2D& P3 = candidate.ctrl[3];
        double d = dist(P0, P3);
        if (d < EPS) {
            result.finalPts = makeSample(candidate, samplingMode, samplingParam);
            return result;
        }

        // 基础 alpha/beta（从候选曲线还原，钳位到安全范围）
        double alpha = std::max(0.10, std::min(0.85, candidate.getAlpha(T0)));
        double beta  = std::max(0.10, std::min(0.85, candidate.getBeta(T3)));

        // 法线方向（沿 P0→P3 的左法线）
        Point2D axis   = (P3 - P0).normalized();
        Point2D normal = axis.rotLeft();

        // 最大允许法线偏移（保证 G1 < 5°）
        double gammaMaxEnter = alpha * d * std::tan(MAX_G1_DEG * DEG2RAD);
        double gammaMaxExit  = beta  * d * std::tan(MAX_G1_DEG * DEG2RAD);

        double gammaEnter = 0.0;
        double gammaExit  = 0.0;

        auto buildCurve = [&]() -> CubicBezier {
            // 钳位 gamma
            double ge = std::max(-gammaMaxEnter, std::min(gammaMaxEnter, gammaEnter));
            double gx = std::max(-gammaMaxExit,  std::min(gammaMaxExit,  gammaExit));
            Point2D P1 = P0 + T0*(alpha*d) + normal*ge;
            Point2D P2 = P3 + T3*(beta*d)  + normal*gx;
            return CubicBezier(P0, P1, P2, P3);
        };

        CubicBezier cur = buildCurve();

        for (int iter = 0; iter < cfg_.maxFixIter; ++iter) {
            Polyline pts = makeSample(cur, samplingMode, samplingParam);

            const Polyline* worst = nullptr;
            for (auto* cl : conflicts) {
                if (polylinesIntersectExcludeEndpoints(pts, *cl)) { worst=cl; break; }
            }
            if (!worst) {
                result.curve    = cur;
                result.finalPts = pts;
                return result;
            }

            // 判断偏移方向
            Point2D myMid = cur.eval(0.5);
            Point2D cfMid = (*worst)[worst->size()/2];
            double  side  = (myMid - cfMid).dot(normal);
            double  step  = 0.15 * (1.0 + iter * 0.05);

            if (side >= 0) { gammaEnter += step; gammaExit += step; }
            else           { gammaEnter -= step; gammaExit -= step; }

            cur = buildCurve();
        }

        result.curve    = cur;
        result.finalPts = makeSample(cur, samplingMode, samplingParam);
        if (!result.finalPts.empty()) {
            result.finalPts.front() = P0;
            result.finalPts.back()  = P3;
        }
        result.intersectionRemains = true;
        Logger::warn("NonIntersect: remains for conn " + conn.id);
        return result;
    }

private:
    Polyline makeSample(const CubicBezier& c,
        const std::string& mode, double param) const
    {
        Polyline pts;
        if      (mode=="fixed_spacing") pts=c.sampleBySpacing(param>0?param:0.5);
        else if (mode=="fixed_count")   pts=c.sampleCount((int)(param>0?param:50));
        else                            pts=c.sampleAdaptive(0.5,2.0,0.05);
        if (!pts.empty()) { pts.front()=c.ctrl[0]; pts.back()=c.ctrl[3]; }
        return pts;
    }

    std::vector<const Polyline*> collectConflicts(
        const Connection& conn,
        const std::vector<GeneratedCenterline>& existing,
        const IntersectionInput& inp) const
    {
        std::vector<const Polyline*> res;
        for (auto& gcl : existing) {
            if (gcl.connectionId==conn.id) continue;
            const Connection* oc=nullptr;
            for (auto& c:inp.connections) if(c.id==gcl.connectionId){oc=&c;break;}
            if (!oc) continue;
            bool rel = (oc->enterGroupId==conn.enterGroupId)
                     ||(oc->exitGroupId ==conn.exitGroupId)
                     ||(oc->enterLineId ==conn.enterLineId)
                     ||(oc->exitLineId  ==conn.exitLineId);
            if (rel) res.push_back(&gcl.geom);
        }
        return res;
    }
};
