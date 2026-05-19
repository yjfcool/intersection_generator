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

        // Detect shared-entry conflicts
        bool hasSharedEntry = false;
        for (auto& gcl : existing) {
            if (gcl.connectionId == conn.id) continue;
            const Connection* oc = nullptr;
            for (auto& c : inp.connections) {
                if (c.id == gcl.connectionId) { oc = &c; break; }
            }
            if (!oc) continue;
            if (oc->enterLineId == conn.enterLineId) {
                hasSharedEntry = true;
                break;
            }
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

        // E. Gradual spreading: positional taper near endpoints
        // For shared-entry curves: asymmetric taper (full weight at entry, taper at exit)
        // For non-shared: symmetric taper at both endpoints
        double taperRatio = cfg_.spreadGradualRatio;
        auto positionalWeight = [&](double t) -> double {
            if (hasSharedEntry) {
                // Asymmetric: full weight at entry (t~0 corresponds to alphaAdj),
                // taper only at exit end (t~1 corresponds to 1-betaAdj)
                if (t > (1.0 - taperRatio)) {
                    double dt = (t - (1.0 - taperRatio)) / taperRatio;
                    return 1.0 - taperRatio * dt;
                }
                return 1.0;
            }
            // Symmetric taper
            if (t < taperRatio) {
                return (1.0 - taperRatio) + taperRatio * (t / taperRatio);
            } else if (t > (1.0 - taperRatio)) {
                double dt = (t - (1.0 - taperRatio)) / taperRatio;
                return 1.0 - taperRatio * dt;
            }
            return 1.0;
        };

        // P1 parametric position is approximately alpha, P2 is approximately (1-beta)
        double weightP1 = positionalWeight(alphaAdj);
        double weightP2 = positionalWeight(1.0 - betaAdj);

        auto buildCurve = [&]() -> CubicBezier {
            double ge = std::max(-gammaMaxEnter, std::min(gammaMaxEnter, gammaEnter));
            double gx = std::max(-gammaMaxExit,  std::min(gammaMaxExit,  gammaExit));
            // Apply positional tapering: P1 near start gets reduced offset, P2 near end too
            weightP1 = positionalWeight(alphaAdj);
            weightP2 = positionalWeight(1.0 - betaAdj);
            double geWeighted = ge * weightP1;
            double gxWeighted = gx * weightP2;
            Point2D P1 = P0 + T0 * (alphaAdj * d) + normal * geWeighted;
            Point2D P2 = P3 + T3 * (betaAdj * d)  + normal * gxWeighted;
            return CubicBezier(P0, P1, P2, P3);
        };

        CubicBezier cur = buildCurve();
        int maxIter = cfg_.globalMaxIter;

        // Determine offset direction from centroid of intersection points
        // Enhanced for shared-entry: use angular separation between exit directions
        auto computeOffsetDir = [&](const Polyline& pts,
                                    const std::vector<const Polyline*>& cfs) -> double {
            if (hasSharedEntry) {
                // For shared-entry curves: determine direction based on angular
                // relationship of exit directions
                Point2D myExitDir = (P3 - P0).normalized();
                Point2D conflictExitDir{0, 0};
                int cnt = 0;
                for (auto* cl : cfs) {
                    if (polylinesIntersectExcludeEndpoints(pts, *cl)) {
                        // Use chord direction of conflict curve as its exit direction
                        if (cl->size() >= 2) {
                            Point2D cfDir = (cl->back() - cl->front()).normalized();
                            conflictExitDir += cfDir;
                            cnt++;
                        }
                    }
                }
                if (cnt > 0) {
                    conflictExitDir = conflictExitDir * (1.0 / cnt);
                    // Cross product determines which side candidate is relative to conflict
                    // cross < 0: candidate exits to the RIGHT of conflict -> push RIGHT (negative normal)
                    // cross > 0: candidate exits to the LEFT of conflict -> push LEFT (positive normal)
                    double cross = conflictExitDir.cross(myExitDir);
                    if (std::abs(cross) > 1e-6) {
                        return (cross > 0) ? 1.0 : -1.0;
                    }
                }
            }

            // Fallback: midpoint centroid comparison
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
        // For shared-entry conflicts, skip STRAT_A (reshaping doesn't separate near entry)
        // and go directly to STRAT_B (lateral offset is more effective)
        enum Strategy { STRAT_A, STRAT_B, STRAT_C };
        Strategy currentStrat = hasSharedEntry ? STRAT_B : STRAT_A;
        int stratFailCount = 0;

        // Issue 1 fix: latch direction at start of each strategy phase
        // to prevent oscillation in symmetric cases
        double latchedDir = 0.0;
        bool dirLatched = false;

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

            double rawDir = computeOffsetDir(pts, conflicts);

            // Latch direction at start of each strategy phase
            if (!dirLatched) {
                latchedDir = (rawDir != 0.0) ? rawDir : 1.0;
                dirLatched = true;
            }
            double dir = latchedDir;

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
                    dirLatched = false; // re-latch for new phase
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
                    dirLatched = false; // re-latch for new phase
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

    /**
     * Composite Bezier non-intersection enforcement.
     * Adjusts the mid-point M and segment alpha/beta parameters to avoid
     * intersections while maintaining G1 continuity and smooth shape.
     * This avoids the S-shape problem that polyline-based enforcement creates.
     */
    CompositeBezier enforceComposite(
        const CompositeBezier& composite,
        const std::vector<GeneratedCenterline>& existing,
        const Connection& conn,
        const IntersectionInput& inp,
        const Point2D& T0,
        const Point2D& T3,
        const std::string& samplingMode,
        double samplingParam,
        bool& intersectionRemains)
    {
        intersectionRemains = false;

        auto conflicts = collectConflicts(conn, existing, inp);
        if (conflicts.empty()) return composite;

        // Sample the current composite curve
        auto sampleComp = [&](const CompositeBezier& cb) -> Polyline {
            Polyline pts;
            if (samplingMode == "fixed_spacing") pts = cb.sampleBySpacing(samplingParam > 0 ? samplingParam : 0.5);
            else if (samplingMode == "fixed_count") pts = cb.sampleCount((int)(samplingParam > 0 ? samplingParam : 50));
            else pts = cb.sampleAdaptive(0.5, 2.0, 0.05);
            if (!pts.empty()) { pts.front() = composite.seg1.ctrl[0]; pts.back() = composite.seg2.ctrl[3]; }
            return pts;
        };

        // Check if current composite curve has conflicts
        Polyline currentPts = sampleComp(composite);
        bool hasConflict = false;
        for (auto* cl : conflicts) {
            if (polylinesIntersectExcludeEndpoints(currentPts, *cl)) {
                hasConflict = true;
                break;
            }
        }
        if (!hasConflict) return composite;

        // Get fixed endpoints and tangents
        const Point2D& P0 = composite.seg1.ctrl[0];
        const Point2D& P3 = composite.seg2.ctrl[3];
        Point2D M = composite.seg1.ctrl[3]; // current mid-point (= seg2.ctrl[0])
        Point2D Tmid = composite.seg1.evalDeriv1(1.0).normalized();

        // Determine the offset direction for mid-point M
        Point2D chord = (P3 - P0);
        double chordLen = chord.norm();
        if (chordLen < EPS) {
            intersectionRemains = true;
            return composite;
        }
        Point2D chordN = chord.normalized();

        // Compute offset direction: perpendicular to conflict's chord, pointing away from conflict
        double offsetDir = 1.0; // default: push left of chord
        for (auto* cl : conflicts) {
            if (cl->size() >= 2 && polylinesIntersectExcludeEndpoints(currentPts, *cl)) {
                Point2D cfChord = (cl->back() - cl->front()).normalized();
                // Cross product: if candidate exits to right of conflict, push right (negative)
                double cross = cfChord.cross(chordN);
                if (std::abs(cross) > 1e-6) {
                    offsetDir = (cross > 0) ? 1.0 : -1.0;
                }
                break;
            }
        }

        // Normal direction for M shift: perpendicular to the Tmid direction
        // This shifts M laterally without changing the general flow direction
        Point2D mNormal = Tmid.rotLeft() * offsetDir;

        // Iteratively shift M and rebuild the composite Bezier
        double d1 = dist(P0, M);
        double d2 = dist(M, P3);
        double alpha1 = std::max(0.10, std::min(0.85, composite.seg1.getAlpha(T0)));
        double alpha2 = std::max(0.10, std::min(0.85, composite.seg2.getAlpha(Tmid)));
        double beta1  = std::max(0.10, std::min(0.85, composite.seg1.getBeta(Tmid * (-1.0))));
        double beta2  = std::max(0.10, std::min(0.85, composite.seg2.getBeta(T3)));

        CompositeBezier best = composite;
        int bestConflictCount = (int)conflicts.size();

        // Try increasing M offsets until no intersection or max reached
        double maxOffset = std::min(d1, d2) * 0.5; // don't shift M more than half the segment length
        double step = maxOffset / 40.0;
        if (step < 0.05) step = 0.05;

        for (int iter = 0; iter < 40; ++iter) {
            double offset = step * (iter + 1);
            Point2D Mshift = M + mNormal * offset;

            // Recompute Tmid to maintain smooth flow through shifted M
            // Tmid should bisect the directions P0->Mshift and Mshift->P3
            Point2D dirToM = (Mshift - P0).normalized();
            Point2D dirFromM = (P3 - Mshift).normalized();
            Point2D newTmid = (dirToM + dirFromM);
            if (newTmid.norm() > EPS) {
                newTmid = newTmid.normalized();
            } else {
                newTmid = Tmid; // fallback
            }

            // Rebuild segments with updated M and Tmid
            double nd1 = dist(P0, Mshift);
            double nd2 = dist(Mshift, P3);
            if (nd1 < EPS || nd2 < EPS) continue;

            // Segment 1: P0 -> Mshift, tangents T0 and newTmid
            Point2D P1_s1 = P0 + T0 * (alpha1 * nd1);
            Point2D P2_s1 = Mshift - newTmid * (beta1 * nd1);
            CubicBezier seg1(P0, P1_s1, P2_s1, Mshift);

            // Segment 2: Mshift -> P3, tangents newTmid and T3
            Point2D P1_s2 = Mshift + newTmid * (alpha2 * nd2);
            Point2D P2_s2 = P3 + T3 * (beta2 * nd2);
            CubicBezier seg2(Mshift, P1_s2, P2_s2, P3);

            CompositeBezier trial(seg1, seg2);
            Polyline trialPts = sampleComp(trial);

            // Check for conflicts
            int conflictCount = 0;
            for (auto* cl : conflicts) {
                if (polylinesIntersectExcludeEndpoints(trialPts, *cl)) {
                    conflictCount++;
                }
            }

            if (conflictCount < bestConflictCount) {
                bestConflictCount = conflictCount;
                best = trial;
            }

            if (conflictCount == 0) {
                Logger::debug("NonIntersect: enforceComposite resolved at M offset=" +
                    std::to_string(offset));
                return trial;
            }
        }

        // If shifting M in one direction didn't work, try the other direction
        if (bestConflictCount > 0) {
            mNormal = mNormal * (-1.0); // reverse direction
            for (int iter = 0; iter < 40; ++iter) {
                double offset = step * (iter + 1);
                Point2D Mshift = M + mNormal * offset;

                Point2D dirToM = (Mshift - P0).normalized();
                Point2D dirFromM = (P3 - Mshift).normalized();
                Point2D newTmid = (dirToM + dirFromM);
                if (newTmid.norm() > EPS) {
                    newTmid = newTmid.normalized();
                } else {
                    newTmid = Tmid;
                }

                double nd1 = dist(P0, Mshift);
                double nd2 = dist(Mshift, P3);
                if (nd1 < EPS || nd2 < EPS) continue;

                Point2D P1_s1 = P0 + T0 * (alpha1 * nd1);
                Point2D P2_s1 = Mshift - newTmid * (beta1 * nd1);
                CubicBezier seg1(P0, P1_s1, P2_s1, Mshift);

                Point2D P1_s2 = Mshift + newTmid * (alpha2 * nd2);
                Point2D P2_s2 = P3 + T3 * (beta2 * nd2);
                CubicBezier seg2(Mshift, P1_s2, P2_s2, P3);

                CompositeBezier trial(seg1, seg2);
                Polyline trialPts = sampleComp(trial);

                int conflictCount = 0;
                for (auto* cl : conflicts) {
                    if (polylinesIntersectExcludeEndpoints(trialPts, *cl)) {
                        conflictCount++;
                    }
                }

                if (conflictCount < bestConflictCount) {
                    bestConflictCount = conflictCount;
                    best = trial;
                }

                if (conflictCount == 0) {
                    Logger::debug("NonIntersect: enforceComposite resolved at M offset=" +
                        std::to_string(-offset) + " (reversed)");
                    return trial;
                }
            }
        }

        // Return best attempt
        if (bestConflictCount > 0) {
            intersectionRemains = true;
            Logger::warn("NonIntersect: enforceComposite failed for conn " + conn.id);
        }
        return best;
    }

    /**
     * Lightweight polyline-based non-intersection enforcement.
     * Used for Phase3 local detour results and composite bezier curves.
     * Applies iterative lateral point displacement on the middle portion,
     * keeping endpoints fixed and maintaining smoothness via weighted kernel.
     *
     * Enhanced for shared-entry curves: uses asymmetric weighting (full weight
     * near entry, tapered near exit) and angular-based offset direction.
     */
    Polyline enforcePolyline(
        const Polyline& pts,
        const std::vector<GeneratedCenterline>& existing,
        const Connection& conn,
        const IntersectionInput& inp,
        bool& intersectionRemains)
    {
        intersectionRemains = false;

        if (pts.size() < 3) return pts;

        // Collect conflicting curves
        auto conflicts = collectConflicts(conn, existing, inp);
        if (conflicts.empty()) return pts;

        // Check if current polyline intersects any conflict curve
        bool hasConflict = false;
        for (auto* cl : conflicts) {
            if (polylinesIntersectExcludeEndpoints(pts, *cl)) {
                hasConflict = true;
                break;
            }
        }
        if (!hasConflict) return pts;

        // Detect shared-entry conflicts: curves that share the same start point
        bool hasSharedEntry = false;
        std::vector<const Polyline*> sharedEntryConflicts;
        for (auto* cl : conflicts) {
            if (!cl->empty() && !pts.empty() &&
                dist(pts.front(), cl->front()) < 0.01) {
                hasSharedEntry = true;
                sharedEntryConflicts.push_back(cl);
            }
        }

        // Determine displacement direction
        Point2D axis = (pts.back() - pts.front()).normalized();
        Point2D normal = axis.rotLeft();

        // For shared-entry: override normal to be perpendicular to the conflict's
        // direction, which more effectively separates curves diverging from a shared point
        if (hasSharedEntry && !sharedEntryConflicts.empty()) {
            // Use the average conflict chord direction to compute normal
            Point2D conflictAxis{0, 0};
            int cnt = 0;
            for (auto* cl : sharedEntryConflicts) {
                if (cl->size() >= 2) {
                    conflictAxis += (cl->back() - cl->front()).normalized();
                    cnt++;
                }
            }
            if (cnt > 0) {
                conflictAxis = conflictAxis * (1.0 / cnt);
                if (conflictAxis.norm() > 1e-6) {
                    normal = conflictAxis.normalized().rotLeft();
                }
            }
        }

        auto computeDisplacementDir = [&](const Polyline& candidate) -> double {
            if (hasSharedEntry && !sharedEntryConflicts.empty()) {
                // For shared-entry curves: compute direction based on the angular
                // relationship between exit directions. A right turn from the same
                // entry as a straight should be pushed to the right of the straight.
                Point2D myExit = candidate.back() - candidate.front();
                Point2D myExitDir = myExit.normalized();

                Point2D conflictExitDir{0, 0};
                int cnt = 0;
                for (auto* cl : sharedEntryConflicts) {
                    if (polylinesIntersectExcludeEndpoints(candidate, *cl)) {
                        Point2D cfExit = cl->back() - cl->front();
                        conflictExitDir += cfExit.normalized();
                        cnt++;
                    }
                }
                if (cnt > 0) {
                    conflictExitDir = conflictExitDir * (1.0 / cnt);
                    // Cross product determines which side candidate is relative to conflict
                    // cross < 0: candidate exits to the RIGHT of conflict -> push RIGHT (negative normal)
                    // cross > 0: candidate exits to the LEFT of conflict -> push LEFT (positive normal)
                    double cross = conflictExitDir.cross(myExitDir);
                    if (std::abs(cross) > 1e-6) {
                        return (cross > 0) ? 1.0 : -1.0;
                    }
                }
            }

            // Fallback: midpoint-based direction
            Point2D myMid = candidate[candidate.size() / 2];
            Point2D conflictCentroid{0, 0};
            int cnt = 0;
            for (auto* cl : conflicts) {
                if (polylinesIntersectExcludeEndpoints(candidate, *cl)) {
                    Point2D cfMid = (*cl)[cl->size() / 2];
                    conflictCentroid += cfMid;
                    cnt++;
                }
            }
            if (cnt == 0) return 1.0;
            conflictCentroid = conflictCentroid * (1.0 / cnt);
            double side = (myMid - conflictCentroid).dot(normal);
            return (side >= 0) ? 1.0 : -1.0;
        };

        // Iterative lateral point displacement
        Polyline current = pts;
        Polyline best = pts;
        int bestIntersections = (int)conflicts.size(); // worst case

        // More iterations and higher base displacement for shared-entry conflicts
        int maxPolyIter = hasSharedEntry ? 40 : 20;
        double baseDisp = hasSharedEntry ? 0.25 : 0.15;

        // Latch direction at start to avoid oscillation
        double latchedDir = computeDisplacementDir(current);
        if (latchedDir == 0.0) latchedDir = 1.0;

        for (int iter = 0; iter < maxPolyIter; ++iter) {
            // Check current state
            bool stillIntersects = false;
            for (auto* cl : conflicts) {
                if (polylinesIntersectExcludeEndpoints(current, *cl)) {
                    stillIntersects = true;
                    break;
                }
            }
            if (!stillIntersects) {
                // Resolved
                return current;
            }

            double dir = latchedDir;
            double disp = baseDisp * (1.0 + iter * 0.12);

            // Apply weighted displacement
            int n = (int)current.size();
            for (int i = 1; i < n - 1; ++i) {
                double t = (double)i / (double)(n - 1); // 0..1
                double weight;
                if (hasSharedEntry) {
                    // Asymmetric weight for shared-entry: full weight near entry (t~0),
                    // tapered near exit (t~1) where curves naturally diverge.
                    // This is crucial because the intersection happens near the entry.
                    if (t < 0.6) {
                        // Full weight in the entry region (first 60% of curve)
                        weight = 1.0;
                    } else {
                        // Taper from 1.0 to 0.2 in the exit region
                        double fadeT = (t - 0.6) / 0.4; // 0..1
                        weight = 1.0 - 0.8 * fadeT;
                    }
                } else {
                    // Symmetric Gaussian for non-shared-entry conflicts
                    weight = std::exp(-0.5 * ((t - 0.5) / 0.25) * ((t - 0.5) / 0.25));
                }
                current[i] += normal * (dir * disp * weight);
            }

            // Smoothing pass: prevent zig-zag artifacts
            if (n >= 4) {
                Polyline smoothed = current;
                int smoothPasses = (n < 8) ? 2 : 1;
                for (int sp = 0; sp < smoothPasses; ++sp) {
                    for (int i = 1; i < n - 1; ++i) {
                        smoothed[i] = current[i - 1] * 0.25 + current[i] * 0.5 + current[i + 1] * 0.25;
                    }
                    smoothed[0] = current[0];
                    smoothed[n - 1] = current[n - 1];
                    current = smoothed;
                }
            }

            // Count remaining intersections for best tracking
            int curIntersections = 0;
            for (auto* cl : conflicts) {
                if (polylinesIntersectExcludeEndpoints(current, *cl))
                    curIntersections++;
            }
            if (curIntersections < bestIntersections) {
                bestIntersections = curIntersections;
                best = current;
            }
        }

        // Max iterations reached - check final state
        bool stillIntersects = false;
        for (auto* cl : conflicts) {
            if (polylinesIntersectExcludeEndpoints(current, *cl)) {
                stillIntersects = true;
                break;
            }
        }
        if (!stillIntersects) return current;

        // Return best attempt, mark intersection remains
        intersectionRemains = true;
        Logger::warn("NonIntersect: enforcePolyline failed after max iterations for conn " + conn.id);
        return best;
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
