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
 * Non-intersection constraint enforcer (rewritten).
 *
 * Key improvements over the previous version:
 *  A. Expanded conflict scope: checks ALL curves sharing the same enter line
 *     OR same exit line (not just same enterGroup+exitGroup).
 *  B. Configurable U-turn intersection allowance.
 *  C. Extreme-case detection: skips enforcement when geometry is impossible.
 *  D. Multi-strategy iterative solver (alpha/beta adjust, gamma offset, combined).
 *  E. Gradual spreading: tapers lateral offset near curve endpoints.
 */
class NonIntersectEnforcer {
    const NonIntersectConfig& cfg_;

    static constexpr double MAX_G1_DEG = 30.0;

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

        // Mid-U-turn exclusion
        if (conn.isMidUturn && cfg_.enableMidUturnExclude) {
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

        // C. Extreme case detection: angle between T0 and (P3-P0) direction
        {
            Point2D chordDir = (P3 - P0).normalized();
            double crossAngle = angleDeg(T0, chordDir);
            if (crossAngle > cfg_.extremeCrossAngleThreshold) {
                Logger::warn("NonIntersect: extreme crossing angle (" +
                    std::to_string(crossAngle) + " deg) for conn " + conn.id +
                    ", skipping enforcement");
                result.finalPts = makeSample(candidate, samplingMode, samplingParam);
                return result;
            }
        }

        // A. Expanded conflict scope
        auto conflicts = collectConflicts(conn, existing, inp);
        if (conflicts.empty()) {
            result.finalPts = makeSample(candidate, samplingMode, samplingParam);
            return result;
        }

        // D. Multi-strategy iterative solver
        double alpha = std::max(0.10, std::min(0.85, candidate.getAlpha(T0)));
        double beta  = std::max(0.10, std::min(0.85, candidate.getBeta(T3)));

        Point2D axis   = (P3 - P0).normalized();
        Point2D normal = axis.rotLeft();

        double gammaMaxEnter = alpha * d * std::tan(MAX_G1_DEG * DEG2RAD);
        double gammaMaxExit  = beta  * d * std::tan(MAX_G1_DEG * DEG2RAD);

        double gammaEnter = 0.0;
        double gammaExit  = 0.0;
        double alphaAdj   = alpha;
        double betaAdj    = beta;

        // E. Gradual spreading weight factor
        // For P1 (near start): use reduced gamma (spreadGradualRatio)
        // For P2 (near end): use reduced gamma (spreadGradualRatio)
        double spreadWeight = 1.0 - cfg_.spreadGradualRatio; // weight applied to P1/P2 offset

        auto buildCurve = [&]() -> CubicBezier {
            double ge = std::max(-gammaMaxEnter, std::min(gammaMaxEnter, gammaEnter));
            double gx = std::max(-gammaMaxExit,  std::min(gammaMaxExit,  gammaExit));
            // Apply gradual spreading: taper gamma near endpoints
            double geWeighted = ge * spreadWeight;
            double gxWeighted = gx * spreadWeight;
            Point2D P1 = P0 + T0 * (alphaAdj * d) + normal * geWeighted;
            Point2D P2 = P3 + T3 * (betaAdj * d)  + normal * gxWeighted;
            return CubicBezier(P0, P1, P2, P3);
        };

        CubicBezier cur = buildCurve();
        int maxIter = cfg_.globalMaxIter;

        // Determine offset direction from centroid of intersection points
        auto computeOffsetDir = [&](const Polyline& pts,
                                    const std::vector<const Polyline*>& cfs) -> double {
            Point2D centroid{0, 0};
            int cnt = 0;
            Point2D myMid = pts[pts.size() / 2];
            for (auto* cl : cfs) {
                if (polylinesIntersectExcludeEndpoints(pts, *cl)) {
                    Point2D cfMid = (*cl)[cl->size() / 2];
                    centroid += cfMid;
                    cnt++;
                }
            }
            if (cnt == 0) return 0.0;
            centroid = centroid * (1.0 / cnt);
            double side = (myMid - centroid).dot(normal);
            return (side >= 0) ? 1.0 : -1.0;
        };

        // Strategy phases: A=alpha/beta, B=gamma, C=combined
        enum Strategy { STRAT_A, STRAT_B, STRAT_C };
        Strategy currentStrat = STRAT_A;
        int stratFailCount = 0;

        for (int iter = 0; iter < maxIter; ++iter) {
            Polyline pts = makeSample(cur, samplingMode, samplingParam);

            bool hasConflict = false;
            for (auto* cl : conflicts) {
                if (polylinesIntersectExcludeEndpoints(pts, *cl)) {
                    hasConflict = true;
                    break;
                }
            }
            if (!hasConflict) {
                result.curve    = cur;
                result.finalPts = pts;
                return result;
            }

            double dir = computeOffsetDir(pts, conflicts);
            double step = 0.4 * (1.0 + iter * 0.08);

            switch (currentStrat) {
            case STRAT_A: {
                // Strategy A: adjust alpha/beta to reshape
                alphaAdj = std::max(0.12, std::min(0.85, alphaAdj - 0.015 * dir));
                betaAdj  = std::max(0.12, std::min(0.85, betaAdj  - 0.015 * dir));
                gammaMaxEnter = alphaAdj * d * std::tan(MAX_G1_DEG * DEG2RAD);
                gammaMaxExit  = betaAdj  * d * std::tan(MAX_G1_DEG * DEG2RAD);
                stratFailCount++;
                if (stratFailCount > maxIter / 3) {
                    currentStrat = STRAT_B;
                    stratFailCount = 0;
                }
                break;
            }
            case STRAT_B: {
                // Strategy B: lateral gamma offset
                gammaEnter += step * dir;
                gammaExit  += step * dir;
                stratFailCount++;
                if (stratFailCount > maxIter / 3) {
                    currentStrat = STRAT_C;
                    stratFailCount = 0;
                }
                break;
            }
            case STRAT_C: {
                // Strategy C: combined alpha/beta + gamma
                alphaAdj = std::max(0.12, std::min(0.85, alphaAdj - 0.01 * dir));
                betaAdj  = std::max(0.12, std::min(0.85, betaAdj  - 0.01 * dir));
                gammaEnter += step * 0.5 * dir;
                gammaExit  += step * 0.5 * dir;
                gammaMaxEnter = alphaAdj * d * std::tan(MAX_G1_DEG * DEG2RAD);
                gammaMaxExit  = betaAdj  * d * std::tan(MAX_G1_DEG * DEG2RAD);
                break;
            }
            }

            cur = buildCurve();
        }

        // All strategies exhausted
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
        if      (mode == "fixed_spacing") pts = c.sampleBySpacing(param > 0 ? param : 0.5);
        else if (mode == "fixed_count")   pts = c.sampleCount((int)(param > 0 ? param : 50));
        else                              pts = c.sampleAdaptive(0.5, 2.0, 0.05);
        if (!pts.empty()) { pts.front() = c.ctrl[0]; pts.back() = c.ctrl[3]; }
        return pts;
    }

    /**
     * Expanded conflict collection (requirement A):
     * Collect ALL existing curves that share the same enter line OR same exit line.
     * Optionally exclude U-turn vs U-turn pairs (requirement B).
     */
    std::vector<const Polyline*> collectConflicts(
        const Connection& conn,
        const std::vector<GeneratedCenterline>& existing,
        const IntersectionInput& inp) const
    {
        std::vector<const Polyline*> res;
        for (auto& gcl : existing) {
            if (gcl.connectionId == conn.id) continue;

            const Connection* oc = nullptr;
            for (auto& c : inp.connections) {
                if (c.id == gcl.connectionId) { oc = &c; break; }
            }
            if (!oc) continue;

            // Expanded scope: share same enter line OR same exit line
            bool shareEnterLine = (oc->enterLineId == conn.enterLineId);
            bool shareExitLine  = (oc->exitLineId  == conn.exitLineId);

            if (!shareEnterLine && !shareExitLine) continue;

            // B. Configurable U-turn intersection allowance
            if (cfg_.allowUturnIntersect) {
                bool candidateIsUturn = (conn.turnType == TurnType::U_TURN_LEFT ||
                                         conn.turnType == TurnType::U_TURN_RIGHT);
                bool otherIsUturn = (oc->turnType == TurnType::U_TURN_LEFT ||
                                     oc->turnType == TurnType::U_TURN_RIGHT);
                if (candidateIsUturn && otherIsUturn) continue;
            }

            res.push_back(&gcl.geom);
        }
        return res;
    }
};
