#pragma once
#include "../data_types.h"
#include "../bezier/cubic_bezier.h"
#include "../preprocess/obstacle_index.h"
#include "../config.h"
#include "local_detour.h"
#include "../../utils/geom_utils.h"
#include "../../utils/logger.h"
#include <cmath>
#include <vector>
#include <string>
#include <limits>

// ─────────────────────────────────────
// 避障结果结构
// ─────────────────────────────────────
struct AvoidanceResult {
    // 当 useDetour=false 时，curve 有效（整体曲线未变形）
    // 当 useDetour=true  时，detourPts 有效（局部绕障折线）
    CubicBezier curve;
    Polyline    detourPts;    // Phase3 局部绕障折线（非空时优先使用）
    bool        useDetour         = false;
    bool        obstacleViolation = false;
    bool        localDetourRelaxNI= false; // Phase3绕障时非相交可降级
    std::vector<std::string> violatedObstacleIds;
};

struct AvoidanceResultComposite {
    CompositeBezier curve;
    Polyline        detourPts;
    bool            useDetour         = false;
    bool            obstacleViolation = false;
    bool            localDetourRelaxNI= false;
    std::vector<std::string> violatedObstacleIds;
};

/**
 * 避障处理器（三阶段策略）
 *
 * Phase 1 — 全局控制点推移（仅调整 alpha/beta，严格保 G1）
 *   迭代推移 P1/P2，使整体曲线偏离障碍物。
 *
 * Phase 2 — 全局梯度下降优化（调整 alpha/beta）
 *   目标函数：曲率能量 + 障碍物惩罚 + 长度惩罚。
 *
 * Phase 3 — 局部绕障（Phase1/2 均失败时触发）
 *   仅在违规参数区间 [t_in, t_out] 内用 1~3 段三次贝塞尔局部绕行，
 *   原曲线首尾段保持不变，G1 连续，整体形态小变形。
 *   此时非相交约束降级（relaxedNI=true）。
 */
class ObstacleAvoider {
    const ObstacleConfig&       cfg_;
    const ObstacleSpatialIndex& idx_;

public:
    ObstacleAvoider(const ObstacleConfig& cfg, const ObstacleSpatialIndex& idx)
        : cfg_(cfg), idx_(idx) {}

    // ══════════════════════════════════
    // 主入口（单段贝塞尔）
    // ══════════════════════════════════
    AvoidanceResult avoid(
        const CubicBezier& initial,
        const Corridor&    corridor,
        double             safeMargin,
        const Point2D&     T0,
        const Point2D&     T3,
        const std::string& samplingMode  = "adaptive",
        double             samplingParam = 0.5,
        const std::vector<Polyline>* existingCurves = nullptr)
    {
        AvoidanceResult result;
        result.curve = initial;

        if (idx_.empty()) return result;

        // ── Phase 1 ──
        bool ok = phase1(result.curve, safeMargin, T0, T3);
        if (ok) {
            Logger::debug("ObstacleAvoider Phase1 success");
            // Cross-check with existing curves: try to avoid introducing new intersections
            if (existingCurves) {
                result.curve = crossCheckExisting(initial, result.curve, safeMargin,
                                                  T0, T3, *existingCurves);
            }
            return result;
        }

        // ── Phase 2 ──
        if (cfg_.enablePhase2) {
            ok = phase2(result.curve, safeMargin, T0, T3);
            if (ok) {
                Logger::debug("ObstacleAvoider Phase2 success");
                // Cross-check with existing curves
                if (existingCurves) {
                    result.curve = crossCheckExisting(initial, result.curve, safeMargin,
                                                      T0, T3, *existingCurves);
                }
                return result;
            }
        }

        // ── Phase 3：局部绕障（基于 initial curve 保持整体形态）──
        if (cfg_.enablePhase3) {
            Logger::info("ObstacleAvoider: Phase1/2 failed, trying Phase3 local detour...");
            LocalDetour detour(idx_, safeMargin, cfg_.phase3CheckSpacing,
                               cfg_.phase3MaxOffsetRatio, cfg_.rightSidePreferThreshold,
                               cfg_.minGapWidth, cfg_.gapSafeMarginRatio);
            // 用 initial 而非 Phase2 的变形结果，保持整体形态
            auto dr = detour.compute(initial, samplingMode, samplingParam);

            if (dr.success) {
                Logger::info("ObstacleAvoider Phase3 local detour success, segs="
                    + std::to_string(dr.detourSegs));
                result.useDetour        = true;
                result.detourPts        = dr.finalPts;
                result.localDetourRelaxNI = true;
                // 强制端点精确
                if (!result.detourPts.empty()) {
                    result.detourPts.front() = initial.ctrl[0];
                    result.detourPts.back()  = initial.ctrl[3];
                }
                return result;
            } else {
                Logger::warn("ObstacleAvoider Phase3 local detour partial/failed.");
                // 即使局部绕障不完全成功，仍使用局部绕障结果
                // （比穿越障碍物好，违规点减少）
                if (!dr.finalPts.empty()) {
                    auto remaining = idx_.checkViolations(dr.finalPts, safeMargin);
                    auto origViols = idx_.checkViolations(
                        result.curve.sampleBySpacing(cfg_.checkSpacing), safeMargin);
                    // 只有局部绕障违规更少时才采用
                    if (remaining.size() < origViols.size()) {
                        result.useDetour        = true;
                        result.detourPts        = dr.finalPts;
                        result.localDetourRelaxNI = true;
                        result.obstacleViolation  = true; // 仍有残余违规
                        if (!result.detourPts.empty()) {
                            result.detourPts.front() = initial.ctrl[0];
                            result.detourPts.back()  = initial.ctrl[3];
                        }
                        for (auto& v : remaining)
                            result.violatedObstacleIds.push_back(v.obsId);
                        return result;
                    }
                }
            }
        }

        // ── 三阶段均失败：标记违规，返回 Phase2 最优结果 ──
        result.obstacleViolation = true;
        auto pts  = result.curve.sampleBySpacing(cfg_.checkSpacing);
        auto viols = idx_.checkViolations(pts, safeMargin);
        for (auto& v : viols) result.violatedObstacleIds.push_back(v.obsId);
        Logger::warn("ObstacleAvoider: all phases failed, obstacle penetrated.");
        return result;
    }

    // ══════════════════════════════════
    // 复合贝塞尔（两段）版
    // ══════════════════════════════════
    AvoidanceResultComposite avoidComposite(
        const CompositeBezier& initial,
        const Corridor&        corridor,
        double                 safeMargin,
        const Point2D&         T0,
        const Point2D&         Tmid,
        const Point2D&         T3,
        const std::string&     samplingMode  = "adaptive",
        double                 samplingParam = 0.5)
    {
        AvoidanceResultComposite result;
        result.curve = initial;
        if (idx_.empty()) return result;

        // 对两段分别做三阶段避障
        AvoidanceResult r1 = avoid(initial.seg1, corridor, safeMargin,
                                   T0, Tmid, samplingMode, samplingParam);
        Point2D Tmid2 = r1.useDetour
            ? tangentAtEnd(r1.detourPts)
            : r1.curve.evalDeriv1(1.0).normalized();

        AvoidanceResult r2 = avoid(initial.seg2, corridor, safeMargin,
                                   Tmid2, T3, samplingMode, samplingParam);

        // 若任一段使用了局部绕障，整体也返回折线
        if (r1.useDetour || r2.useDetour) {
            // 拼合两段折线
            Polyline combined;
            Polyline p1 = r1.useDetour ? r1.detourPts
                                    : r1.curve.sampleBySpacing(cfg_.checkSpacing);
            Polyline p2 = r2.useDetour ? r2.detourPts
                                    : r2.curve.sampleBySpacing(cfg_.checkSpacing);
            combined = p1;
            for (size_t i = 1; i < p2.size(); ++i) combined.push_back(p2[i]);

            if (!combined.empty()) {
                combined.front() = initial.seg1.ctrl[0];
                combined.back()  = initial.seg2.ctrl[3];
            }
            result.useDetour        = true;
            result.detourPts        = combined;
            result.localDetourRelaxNI = true;
        } else {
            result.curve.seg1 = r1.curve;
            result.curve.seg2 = r2.curve;
        }

        if (r1.obstacleViolation || r2.obstacleViolation) {
            result.obstacleViolation = true;
            for (auto& s : r1.violatedObstacleIds) result.violatedObstacleIds.push_back(s);
            for (auto& s : r2.violatedObstacleIds) result.violatedObstacleIds.push_back(s);
        }
        return result;
    }

private:
    // ══════════════════════════════════
    // Phase 1：控制点走廊推移（保 G1）
    // ══════════════════════════════════
    bool phase1(CubicBezier& curve, double safeMargin,
                const Point2D& T0, const Point2D& T3)
    {
        const Point2D& P0 = curve.ctrl[0];
        const Point2D& P3 = curve.ctrl[3];
        double d = dist(P0, P3);
        if (d < EPS) return false;

        double alpha = std::max(0.10, std::min(0.85, curve.getAlpha(T0)));
        double beta  = std::max(0.10, std::min(0.85, curve.getBeta(T3)));
        double step  = safeMargin * 0.5;

        for (int iter = 0; iter < cfg_.phase1MaxIter; ++iter) {
            auto pts  = curve.sampleBySpacing(cfg_.checkSpacing);
            auto viols = idx_.checkViolations(pts, safeMargin);
            if (viols.empty()) return true;

            // 聚合前半段/后半段的推力
            Point2D pushFront{0,0}, pushBack{0,0};
            int cntF = 0, cntB = 0;
            int n = (int)pts.size();
            for (auto& v : viols) {
                double w = 1.0 / std::max(v.distToObs, EPS);
                if (v.ptIdx < n / 2) { pushFront += v.pushDir * w; cntF++; }
                else                 { pushBack  += v.pushDir * w; cntB++; }
            }

            // 只调整 alpha/beta（保持 G1：P1 在 T0 方向，P2 在 T3 方向）
            // 若推力与切线方向夹角 < 90°，增大 alpha/beta（拉长控制手柄，整体绕过）
            // 若夹角 > 90°，说明需要改变曲线整体弯向，暂用 step 增量
            if (cntF > 0) {
                double proj = pushFront.normalized().dot(T0.normalized());
                // proj > 0：推力沿切线方向（延伸 P1）
                // proj < 0：推力与切线相反（缩短 P1）
                // 无论如何，增大 alpha 都能使曲线更"饱满"，更容易绕
                alpha += step * 0.4;
            }
            if (cntB > 0) {
                beta += step * 0.4;
            }

            alpha = std::max(0.10, std::min(0.85, alpha));
            beta  = std::max(0.10, std::min(0.85, beta));
            curve = CubicBezier::fromAlphaBeta(P0, T0, P3, T3, alpha, beta);
            step *= cfg_.phase1StepDecay;
        }
        return false;
    }

    // ══════════════════════════════════
    // Phase 2：梯度下降优化（保 G1）
    // ══════════════════════════════════
    bool phase2(CubicBezier& curve, double safeMargin,
                const Point2D& T0, const Point2D& T3)
    {
        const Point2D& P0 = curve.ctrl[0];
        const Point2D& P3 = curve.ctrl[3];

        double alpha = std::max(0.10, std::min(0.85, curve.getAlpha(T0)));
        double beta  = std::max(0.10, std::min(0.85, curve.getBeta(T3)));

        auto objective = [&](double a, double b) -> double {
            CubicBezier cb = CubicBezier::fromAlphaBeta(P0, T0, P3, T3, a, b);
            auto pts = cb.sampleBySpacing(cfg_.checkSpacing);

            double curvE = 0;
            for (int i = 0; i <= 20; ++i) {
                double t = i / 20.0;
                double k = cb.curvature(t);
                curvE += k * k * (1.0 / 20.0);
            }

            double obsP = 0;
            auto viols = idx_.checkViolations(pts, safeMargin);
            for (auto& v : viols) {
                double pen = safeMargin - v.distToObs;
                obsP += pen * pen;
            }

            double lenP = cb.arcLength(30);

            return cfg_.phase2WCurvature * curvE
                 + cfg_.phase2WObstacle  * obsP
                 + cfg_.phase2WLength    * lenP * 0.01;
        };

        double lr  = 0.05;
        double eps = 1e-4;
        for (int iter = 0; iter < cfg_.phase2MaxIter; ++iter) {
            double f0 = objective(alpha, beta);
            double gA = (objective(alpha + eps, beta) - f0) / eps;
            double gB = (objective(alpha, beta + eps) - f0) / eps;

            double nA = std::max(0.10, std::min(0.85, alpha - lr * gA));
            double nB = std::max(0.10, std::min(0.85, beta  - lr * gB));

            if (objective(nA, nB) < f0) { alpha = nA; beta = nB; }
            else                         lr *= 0.5;

            if (lr < 1e-8) break;
        }

        curve = CubicBezier::fromAlphaBeta(P0, T0, P3, T3, alpha, beta);
        auto pts = curve.sampleBySpacing(cfg_.checkSpacing);
        return idx_.checkViolations(pts, safeMargin).empty();
    }

    // ══════════════════════════════════
    // Cross-check: after Phase1/2 success, try to avoid introducing new
    // intersections with existing curves. Best-effort only - never compromises
    // obstacle avoidance. Tries 3 interpolation points (25%, 50%, 75%)
    // to find a viable middle-ground.
    // ══════════════════════════════════
    CubicBezier crossCheckExisting(
        const CubicBezier& original,
        const CubicBezier& avoidResult,
        double safeMargin,
        const Point2D& T0,
        const Point2D& T3,
        const std::vector<Polyline>& existingCurves)
    {
        if (existingCurves.empty()) return avoidResult;

        Polyline avoidPts = avoidResult.sampleBySpacing(cfg_.checkSpacing);
        Polyline origPts  = original.sampleBySpacing(cfg_.checkSpacing);

        // Check if avoidance result intersects existing curves
        bool avoidIntersects = false;
        for (auto& ec : existingCurves) {
            if (polylinesIntersectExcludeEndpoints(avoidPts, ec)) {
                avoidIntersects = true;
                break;
            }
        }
        if (!avoidIntersects) return avoidResult; // no problem

        // Check if original did NOT intersect (avoidance introduced the intersection)
        bool origIntersects = false;
        for (auto& ec : existingCurves) {
            if (polylinesIntersectExcludeEndpoints(origPts, ec)) {
                origIntersects = true;
                break;
            }
        }
        if (origIntersects) return avoidResult; // original already intersected, can't help

        // Try 3 interpolation points (25%, 50%, 75%) between original and avoidance
        const Point2D& P0 = avoidResult.ctrl[0];
        const Point2D& P3 = avoidResult.ctrl[3];
        double alphaOrig  = std::max(0.10, std::min(0.85, original.getAlpha(T0)));
        double betaOrig   = std::max(0.10, std::min(0.85, original.getBeta(T3)));
        double alphaAvoid = std::max(0.10, std::min(0.85, avoidResult.getAlpha(T0)));
        double betaAvoid  = std::max(0.10, std::min(0.85, avoidResult.getBeta(T3)));

        // Try lerp factors closest to avoidance first (prefer obstacle clearance)
        static constexpr double lerpFactors[] = {0.75, 0.50, 0.25};

        for (double factor : lerpFactors) {
            double alphaLerp = alphaOrig + factor * (alphaAvoid - alphaOrig);
            double betaLerp  = betaOrig  + factor * (betaAvoid  - betaOrig);

            CubicBezier lerpCurve = CubicBezier::fromAlphaBeta(P0, T0, P3, T3, alphaLerp, betaLerp);
            Polyline lerpPts = lerpCurve.sampleBySpacing(cfg_.checkSpacing);

            // Check if this lerp point still avoids obstacles
            auto viols = idx_.checkViolations(lerpPts, safeMargin);
            if (!viols.empty()) continue; // obstacle violation - skip this factor

            // Check if this lerp point avoids intersections with existing curves
            bool lerpIntersects = false;
            for (auto& ec : existingCurves) {
                if (polylinesIntersectExcludeEndpoints(lerpPts, ec)) {
                    lerpIntersects = true;
                    break;
                }
            }
            if (!lerpIntersects) {
                Logger::debug("ObstacleAvoider: crossCheck found viable solution at lerp=" +
                    std::to_string(factor));
                return lerpCurve;
            }
        }

        // None of the interpolation points worked - keep the avoidance result (obstacle priority)
        return avoidResult;
    }

    // ══════════════════════════════════
    // 辅助：取折线末尾切線方向
    // ══════════════════════════════════
    static Point2D tangentAtEnd(const Polyline& pts) {
        if (pts.size() < 2) return {0, 1};
        return (pts.back() - pts[pts.size()-2]).normalized();
    }
};
