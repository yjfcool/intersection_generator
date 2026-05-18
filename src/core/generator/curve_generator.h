#pragma once
#include "../data_types.h"
#include "../config.h"
#include "../preprocess/obstacle_index.h"
#include "../preprocess/tangent_estimator.h"
#include "../bezier/cubic_bezier.h"
#include "corridor_allocator.h"
#include "control_point_init.h"
#include "obstacle_avoider.h"
#include "non_intersect_enforcer.h"
#include "conflict_resolver.h"
#include "../../utils/geom_utils.h"
#include "../../utils/logger.h"
#include <map>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <cmath>

/**
 * 核心曲线生成器（主流程）
 * 按 直行→右转→左转→调头 顺序逐条生成路口内中心线
 */
class CurveGenerator {
    const Config& cfg_;
    const ObstacleSpatialIndex& obsIdx_;

public:
    explicit CurveGenerator(const Config& cfg, const ObstacleSpatialIndex& obsIdx)
        : cfg_(cfg), obsIdx_(obsIdx) {}

    std::vector<GeneratedCenterline> generate(
        const IntersectionInput& inp)
    {
        std::vector<GeneratedCenterline> results;

        // 初始化各模块
        CorridorAllocator allocator(inp);
        auto corridors = allocator.allocate(cfg_.nonIntersect.corridorMinHalfWidth);
        auto sortedConns = allocator.sortedConnections();

        ObstacleAvoider avoider(cfg_.obstacle, obsIdx_);
        NonIntersectEnforcer enforcer(cfg_.nonIntersect);
        ConflictResolver resolver(cfg_.conflict, cfg_.obstacle, obsIdx_);

        for(auto& conn : sortedConns){
            Logger::info("Generating conn: " + conn.id +
                         " turn=" + turnTypeStr(conn.turnType));

            try {
                auto gcl = generateOne(conn, inp, corridors, results,
                                       avoider, enforcer, resolver);
                results.push_back(gcl);

                // 更新走廊边界
                allocator.updateCorridorBounds(conn, gcl.geom, corridors);

            } catch(const std::exception& e){
                Logger::error("generateOne failed for conn " + conn.id +
                              ": " + e.what());
                // 生成退化直线作为后备
                auto fallback = generateFallback(conn, inp);
                fallback.qualityFlags |= QF_ERROR_NO_SOLUTION;
                fallback.warnDesc += "FALLBACK:"+std::string(e.what())+";";
                results.push_back(fallback);
            }
        }

        return results;
    }

private:
    GeneratedCenterline generateOne(
        const Connection& conn,
        const IntersectionInput& inp,
        std::map<std::string, Corridor>& corridors,
        const std::vector<GeneratedCenterline>& existing,
        ObstacleAvoider& avoider,
        NonIntersectEnforcer& enforcer,
        ConflictResolver& resolver)
    {
        // 1. 获取连接点和切线
        auto enterIt = inp.centerlines.find(conn.enterLineId);
        auto exitIt  = inp.centerlines.find(conn.exitLineId);

        if(enterIt==inp.centerlines.end() || exitIt==inp.centerlines.end()){
            throw std::runtime_error("Centerline not found for conn "+conn.id);
        }

        const Point2D& P0 = enterIt->second.connectionPt;
        const Point2D& T0 = enterIt->second.tangentDir; // 指向路口内

        const Point2D& P3 = exitIt->second.connectionPt;
        const Point2D& T3 = exitIt->second.tangentDir;  // 指向路口内（退出方向的进入端）

        // 输入退化检测
        if(dist(P0,P3) < 0.01){
            GeneratedCenterline gcl;
            gcl.id           = "gen_cl_"+conn.id;
            gcl.connectionId = conn.id;
            gcl.enterLineId  = conn.enterLineId;
            gcl.exitLineId   = conn.exitLineId;
            gcl.turnType     = conn.turnType;
            gcl.geom         = {P0, P3};
            gcl.qualityFlags = QF_ERROR_DEGENERATE_INPUT;
            gcl.warnDesc     = "DEGENERATE:P0==P3;";
            return gcl;
        }

        // 2. 初始化控制点
        auto initRes = ControlPointInit::init(P0, T0, P3, T3, conn.turnType, cfg_.bezier);

        // 3. 获取走廊
        Corridor& corridor = corridors.count(conn.id)
            ? corridors[conn.id]
            : corridors["_default_"];

        // 4. Obstacle avoidance and NI enforcement with 4-level cascade
        Polyline ptsRaw, ptsAfterAvoid, ptsAfterEnforce;
        bool obstViol1=false, interViol1=false;
        bool obstViol2=false, interViol2=false;
        bool localDetourActive = false;
        int cascadeLevel = 1;
        int qualFlags = 0;

        if(!initRes.useComposite){
            // Single-segment Bezier: 4-level cascade
            ptsRaw = sampleCurve(initRes.single);

            // Collect existing curves polylines for cross-check
            std::vector<Polyline> existingPolys;
            existingPolys.reserve(existing.size());
            for (auto& gcl : existing) existingPolys.push_back(gcl.geom);

            // ══ Level 1: Normal flow (both constraints active) ══
            auto avoidRes = avoider.avoid(initRes.single, corridor,
                                          cfg_.obstacle.safeMargin, T0, T3,
                                          cfg_.sampling.mode, getSamplingParam(),
                                          &existingPolys);
            obstViol1 = avoidRes.obstacleViolation;

            if (avoidRes.useDetour) {
                ptsAfterAvoid     = avoidRes.detourPts;
                localDetourActive = true;
            } else {
                ptsAfterAvoid = sampleCurve(avoidRes.curve);
            }

            // Non-intersection enforcement
            if (localDetourActive) {
                // Phase3 local detour was triggered by obstacle avoider
                // Go directly to cascade level 3
                cascadeLevel = 3;
                ptsAfterEnforce = ptsAfterAvoid;

                // Attempt lightweight polyline NI fix
                bool niRemains = false;
                auto fixedPts = enforcer.enforcePolyline(
                    ptsAfterAvoid, existing, conn, inp, niRemains);
                ptsAfterEnforce = fixedPts;

                if (niRemains) {
                    cascadeLevel = 4;
                    qualFlags |= QF_WARN_INTERSECTION_REMAIN;
                }
                interViol1 = niRemains;

                // Check obstacle violations on the fixed result
                obstViol2 = false;
                auto fixedViols = obsIdx_.checkViolations(ptsAfterEnforce, cfg_.obstacle.safeMargin);
                if (!fixedViols.empty()) {
                    obstViol2 = true;
                    // NI fix moved into obstacle - revert to pure detour
                    ptsAfterEnforce = ptsAfterAvoid;
                }
                interViol2 = niRemains;
                qualFlags |= QF_INFO_TWO_SEGMENT_USED;
            } else {
                // Normal enforcement (Bezier-based)
                auto enforceRes = enforcer.enforce(
                    avoidRes.curve, corridor, existing, conn, inp,
                    T0, T3,
                    cfg_.sampling.mode, getSamplingParam());
                ptsAfterEnforce  = enforceRes.finalPts;
                interViol1 = enforceRes.intersectionRemains;

                // ══ Post-enforcement obstacle check (Level 1 -> Level 2 transition) ══
                auto enforceViols = obsIdx_.checkViolations(ptsAfterEnforce, cfg_.obstacle.safeMargin);
                obstViol2 = !enforceViols.empty();

                if (obstViol2 && !obstViol1) {
                    // Enforcement moved curve into obstacle - fall back to avoidance result
                    // This is Level 2: obstacle priority
                    cascadeLevel = 2;
                    ptsAfterEnforce = ptsAfterAvoid;

                    // Re-check intersection on avoidance result for warning
                    interViol2 = false;
                    for (auto& gcl : existing) {
                        if (polylinesIntersectExcludeEndpoints(ptsAfterEnforce, gcl.geom)) {
                            interViol2 = true;
                            break;
                        }
                    }
                    obstViol2 = false; // avoidance result was obstacle-clean
                    Logger::info("  cascade L2: enforcement reintroduced obstacle, using avoidance result");
                } else if (obstViol1) {
                    // Avoidance itself had obstacle violation (Phase1/2 partial fail, no Phase3)
                    // We are still at Level 1 but with violations
                    // The resolver will handle priority
                    interViol2 = interViol1;
                } else {
                    // Level 1 success path
                    interViol2 = interViol1;
                }
            }

            if(conn.isMidUturn){
                qualFlags |= QF_INFO_UTURN_MID_EXCLUDED;
            }

        } else {
            // 复合贝塞尔（两段）
            qualFlags |= QF_INFO_TWO_SEGMENT_USED;

            // 中间切线
            Point2D Tmid = initRes.composite.seg1.evalDeriv1(1.0).normalized();

            auto avoidRes = avoider.avoidComposite(
                initRes.composite, corridor,
                cfg_.obstacle.safeMargin, T0, Tmid, T3,
                cfg_.sampling.mode, getSamplingParam());
            obstViol1 = avoidRes.obstacleViolation;
            if (avoidRes.useDetour) {
                localDetourActive = true;
            }

            ptsRaw        = sampleCompositeCurve(initRes.composite);
            ptsAfterAvoid = sampleCompositeCurve(avoidRes.curve);

            // Non-intersection enforcement for composite curves (Bezier-level)
            bool niRemains = false;
            CompositeBezier enforced = enforcer.enforceComposite(
                avoidRes.curve, existing, conn, inp,
                T0, T3, cfg_.sampling.mode, getSamplingParam(), niRemains);
            ptsAfterEnforce = sampleCompositeCurve(enforced);
            interViol1 = niRemains;

            // Check if NI enforcement moved into obstacle
            auto compositeViols = obsIdx_.checkViolations(ptsAfterEnforce, cfg_.obstacle.safeMargin);
            if (!compositeViols.empty() && !obstViol1) {
                // NI enforcement moved into obstacle - revert to avoidance result
                ptsAfterEnforce = ptsAfterAvoid;
                interViol1 = false;
                for (auto& gcl : existing) {
                    if (polylinesIntersectExcludeEndpoints(ptsAfterEnforce, gcl.geom)) {
                        interViol1 = true;
                        break;
                    }
                }
            }
            obstViol2 = obstViol1;
            interViol2 = interViol1;

            if (localDetourActive) cascadeLevel = 3;
        }

        // 5. 冲突协调 with cascade level
        auto resolveRes = resolver.resolve(
            ptsRaw, ptsAfterAvoid, ptsAfterEnforce,
            obstViol1, interViol1,
            obstViol2, interViol2,
            localDetourActive,
            cascadeLevel);

        qualFlags |= resolveRes.qualityFlags;

        // 检查曲率
        if(!initRes.useComposite &&
           initRes.single.maxCurvature(20) > cfg_.bezier.maxCurvature * 2.0){
            qualFlags |= QF_WARN_CURVATURE_HIGH;
        }

        // 6. 封装结果
        GeneratedCenterline gcl;
        gcl.id           = "gen_cl_"+conn.id;
        gcl.connectionId = conn.id;
        gcl.enterLineId  = conn.enterLineId;
        gcl.exitLineId   = conn.exitLineId;
        gcl.turnType     = conn.turnType;
        gcl.geom         = resolveRes.finalPts;
        gcl.qualityFlags = qualFlags;
        gcl.warnDesc     = resolveRes.warnDesc;

        // 确保端点锁定
        if(!gcl.geom.empty()){
            gcl.geom.front() = P0;
            gcl.geom.back()  = P3;
        }

        Logger::info("  -> generated pts=" + std::to_string(gcl.geom.size()) +
                     " flags=" + std::to_string(gcl.qualityFlags) +
                     " cascade=" + std::to_string(cascadeLevel));

        return gcl;
    }

    Polyline sampleCurve(const CubicBezier& curve) const {
        const auto& sc = cfg_.sampling;
        if(sc.mode=="fixed_spacing"){
            return curve.sampleBySpacing(sc.fixedSpacing);
        } else if(sc.mode=="fixed_count"){
            return curve.sampleCount(sc.fixedCount);
        } else {
            return curve.sampleAdaptive(sc.adaptiveMaxAngleDeg,
                                        sc.adaptiveMaxSegLength,
                                        sc.adaptiveMinSegLength);
        }
    }

    Polyline sampleCompositeCurve(const CompositeBezier& curve) const {
        const auto& sc = cfg_.sampling;
        if(sc.mode=="fixed_spacing"){
            return curve.sampleBySpacing(sc.fixedSpacing);
        } else if(sc.mode=="fixed_count"){
            return curve.sampleCount(sc.fixedCount);
        } else {
            return curve.sampleAdaptive(sc.adaptiveMaxAngleDeg,
                                        sc.adaptiveMaxSegLength,
                                        sc.adaptiveMinSegLength);
        }
    }

    double getSamplingParam() const {
        if(cfg_.sampling.mode=="fixed_spacing") return cfg_.sampling.fixedSpacing;
        if(cfg_.sampling.mode=="fixed_count")   return cfg_.sampling.fixedCount;
        return cfg_.sampling.adaptiveMaxAngleDeg;
    }

    bool obsIdx_empty() const { return obsIdx_.empty(); }

    std::vector<ObstacleSpatialIndex::Violation> checkObstacleViols(
        const Polyline& pts) const
    {
        return obsIdx_.checkViolations(pts, cfg_.obstacle.safeMargin);
    }

    const Connection* findConn(const std::string& id, const IntersectionInput& inp) const {
        for(auto& c : inp.connections){
            if(c.id == id) return &c;
        }
        return nullptr;
    }

    GeneratedCenterline generateFallback(
        const Connection& conn, const IntersectionInput& inp) const
    {
        GeneratedCenterline gcl;
        gcl.id           = "gen_cl_"+conn.id;
        gcl.connectionId = conn.id;
        gcl.enterLineId  = conn.enterLineId;
        gcl.exitLineId   = conn.exitLineId;
        gcl.turnType     = conn.turnType;

        // 退化：直线连接
        auto enterIt = inp.centerlines.find(conn.enterLineId);
        auto exitIt  = inp.centerlines.find(conn.exitLineId);
        if(enterIt!=inp.centerlines.end() && exitIt!=inp.centerlines.end()){
            gcl.geom = {enterIt->second.connectionPt, exitIt->second.connectionPt};
        }
        return gcl;
    }
};
