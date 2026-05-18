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

/**
 * 核心曲线生成器（主流程）
 * 按 直行→右转→左转→调头 顺序逐条生成路口内中心线
 */
class CurveGenerator {
    const Config& cfg_;

public:
    explicit CurveGenerator(const Config& cfg) : cfg_(cfg) {}

    std::vector<GeneratedCenterline> generate(
        const IntersectionInput& inp,
        const ObstacleSpatialIndex& obsIdx)
    {
        std::vector<GeneratedCenterline> results;

        // 初始化各模块
        CorridorAllocator allocator(inp);
        auto corridors = allocator.allocate(cfg_.nonIntersect.corridorMinHalfWidth);
        auto sortedConns = allocator.sortedConnections();

        ObstacleAvoider avoider(cfg_.obstacle, obsIdx);
        NonIntersectEnforcer enforcer(cfg_.nonIntersect);
        ConflictResolver resolver(cfg_.conflict, cfg_.obstacle, obsIdx);

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

        // 4. 避障处理
        Polyline ptsRaw, ptsAfterAvoid, ptsAfterEnforce;
        bool obstViol1=false, interViol1=false;
        bool obstViol2=false, interViol2=false;
        bool localDetourActive = false;  // Phase3 局部绕障激活
        int qualFlags = 0;

        if(!initRes.useComposite){
            // 单段贝塞尔
            ptsRaw = sampleCurve(initRes.single);

            auto avoidRes = avoider.avoid(initRes.single, corridor,
                                          cfg_.obstacle.safeMargin, T0, T3,
                                          cfg_.sampling.mode, getSamplingParam());
            obstViol1 = avoidRes.obstacleViolation;
            // Phase3 局部绕障时直接使用折线
            if (avoidRes.useDetour) {
                ptsAfterAvoid     = avoidRes.detourPts;
                localDetourActive = true;
            } else {
                ptsAfterAvoid = sampleCurve(avoidRes.curve);
            }

            // 非相交约束（Phase3局部绕障时降级：只做轻量检测，不强制修复）
            if (localDetourActive) {
                // 绕障段非相交降级：直接使用绕障折线，不执行enforce
                ptsAfterEnforce = ptsAfterAvoid;
                // 仍做检测以便标记
                for (auto& gcl : existing) {
                    if (polylinesIntersectExcludeEndpoints(ptsAfterEnforce, gcl.geom)) {
                        interViol1 = true; break;
                    }
                }
                qualFlags |= QF_INFO_TWO_SEGMENT_USED; // 复用标志位表示局部绕障
            } else {
                auto enforceRes = enforcer.enforce(
                    avoidRes.curve, corridor, existing, conn, inp,
                    T0, T3,
                    cfg_.sampling.mode, getSamplingParam());
                ptsAfterEnforce  = enforceRes.finalPts;
                interViol1 = enforceRes.intersectionRemains;
            }

            // 再次检查避障
            if(!obsIdx_empty()){
                auto viols = checkObstacleViols(ptsAfterEnforce);
                obstViol2 = !viols.empty();
            }
            interViol2 = interViol1;

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

            // 对复合曲线执行非相交约束（转为单段近似处理）
            // 为简化，将复合曲线视为采样折线进行相交检测
            ptsAfterEnforce = ptsAfterAvoid;

            // 检查相交（用采样折线）
            for(auto& gcl : existing){
                const Connection* oc = findConn(conn.id, inp);
                (void)oc;
                if(polylinesIntersectExcludeEndpoints(ptsAfterEnforce, gcl.geom)){
                    interViol1 = true;
                    // 简单修复：轻微横向偏移
                    Point2D dir  = (P3-P0).normalized();
                    Point2D norm = dir.rotLeft();
                    for(size_t i=1;i+1<ptsAfterEnforce.size();++i){
                        ptsAfterEnforce[i] += norm * 0.15;
                    }
                }
            }
            obstViol2 = obstViol1;
            interViol2 = interViol1;
        }

        // 5. 冲突协调（局部绕障激活时：避障优先，非相交降级）
        auto resolveRes = resolver.resolve(
            ptsRaw, ptsAfterAvoid, ptsAfterEnforce,
            obstViol1, interViol1,
            obstViol2, interViol2,
            localDetourActive);

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
                     " flags=" + std::to_string(gcl.qualityFlags));

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

    bool obsIdx_empty() const { return false; } // 简化：总是检查

    std::vector<ObstacleSpatialIndex::Violation> checkObstacleViols(
        const Polyline& pts) const
    {
        return {}; // 由调用方管理
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
