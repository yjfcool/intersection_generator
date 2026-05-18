#pragma once
#include "../data_types.h"
#include "../bezier/cubic_bezier.h"
#include "../config.h"
#include "../preprocess/obstacle_index.h"
#include "../../utils/geom_utils.h"
#include "../../utils/logger.h"

struct ConflictResolveResult {
    Polyline    finalPts;
    int         qualityFlags = 0;
    std::string warnDesc;
};

/**
 * 避障 / 非相交冲突协调器
 *
 * 核心语义：
 *  - 当局部绕障（Phase3）激活时：避障优先（即使有轻微相交也先保证不穿障碍物）
 *  - 否则按配置 priority 决定：
 *      "non_intersect"（默认）：非相交优先，障碍物穿越仅标警告
 *      "obstacle"             ：避障优先，相交仅标警告
 *      "both_error"           ：两者均标错误
 */
class ConflictResolver {
    const ConflictConfig&       cfg_;
    const ObstacleConfig&       obsCfg_;
    const ObstacleSpatialIndex& idx_;

public:
    ConflictResolver(const ConflictConfig& cfg,
                     const ObstacleConfig& obsCfg,
                     const ObstacleSpatialIndex& idx)
        : cfg_(cfg), obsCfg_(obsCfg), idx_(idx) {}

    /**
     * 协调结果
     * @param ptsRaw              原始曲线（未避障）采样
     * @param ptsAfterAvoid       避障后曲线采样
     * @param ptsAfterEnforce     非相交约束后曲线采样
     * @param obstacleViol        避障阶段是否有违规
     * @param intersectViol       非相交阶段是否有违规
     * @param localDetourActive   Phase3 局部绕障激活标志
     */
    ConflictResolveResult resolve(
        const Polyline& ptsRaw,
        const Polyline& ptsAfterAvoid,
        const Polyline& ptsAfterEnforce,
        bool obstViolAvoid,
        bool interViolAvoid,
        bool obstViolEnforce,
        bool interViolEnforce,
        bool localDetourActive = false,
        int  cascadeLevel = 1)
    {
        ConflictResolveResult res;

        // Log cascade level for debugging
        res.warnDesc += "CASCADE_LEVEL=" + std::to_string(cascadeLevel) + ";";
        Logger::debug("ConflictResolver: cascade level " + std::to_string(cascadeLevel));

        // ── Cascade levels 3-4: obstacle avoidance absolute priority ──
        if (cascadeLevel >= 3) {
            // At high cascade levels, always use obstacle-avoidance result
            res.finalPts = ptsAfterEnforce; // already set to detour/avoidance result by caller

            if (obstViolEnforce) {
                res.qualityFlags |= QF_WARN_OBSTACLE_PENETRATED;
                res.warnDesc += "OBSTACLE_PENETRATED(cascade>=3);";
            }
            if (interViolEnforce) {
                // Non-intersection is just a warning at cascade >= 3
                res.qualityFlags |= QF_WARN_INTERSECTION_REMAIN;
                res.warnDesc += "INTERSECT_RELAXED(cascade>=3);";
            }

            if (!ptsRaw.empty() && !res.finalPts.empty()) {
                res.finalPts.front() = ptsRaw.front();
                res.finalPts.back()  = ptsRaw.back();
            }
            return res;
        }

        // ── 局部绕障激活：避障绝対優先 ──────────────────────────────
        if (localDetourActive) {
            // 使用局部绕障结果（ptsAfterAvoid=detourPts）
            res.finalPts = ptsAfterAvoid;

            if (obstViolAvoid) {
                // 局部绕障仍有穿越（極端情况，无法完全绕开）
                res.qualityFlags |= QF_WARN_OBSTACLE_PENETRATED;
                res.warnDesc += "OBSTACLE_PENETRATED(partial_detour);";
            }
            if (interViolAvoid) {
                // 非相交降级：仅标警告，不强制修復
                res.qualityFlags |= QF_WARN_INTERSECTION_REMAIN;
                res.warnDesc += "INTERSECT_RELAXED(local_detour);";
            }

            // 确保端点精确
            if (!ptsRaw.empty() && !res.finalPts.empty()) {
                res.finalPts.front() = ptsRaw.front();
                res.finalPts.back()  = ptsRaw.back();
            }
            return res;
        }

        // ── 普通三阶段（Phase1/2）结果協调 ────────────────────────────
        if (cfg_.priority == "non_intersect") {
            // 非相交優先：使用 ptsAfterEnforce，标记避障警告
            res.finalPts = ptsAfterEnforce;
            if (interViolEnforce) {
                res.qualityFlags |= QF_WARN_INTERSECTION_REMAIN;
                res.warnDesc += "INTERSECT_REMAIN;";
            }
            if (obstViolEnforce) {
                res.qualityFlags |= QF_WARN_OBSTACLE_PENETRATED;
                res.warnDesc += "OBSTACLE_PENETRATED;";
            }
        }
        else if(cfg_.priority == "obstacle"){
            // 避障優先：使用 ptsAfterAvoid，标记相交警告
            res.finalPts = ptsAfterAvoid;
            if (obstViolAvoid) {
                res.qualityFlags |= QF_WARN_OBSTACLE_PENETRATED;
                res.warnDesc += "OBSTACLE_PENETRATED;";
            }
            if (interViolAvoid) {
                res.qualityFlags |= QF_WARN_INTERSECTION_REMAIN;
                res.warnDesc += "INTERSECT_REMAIN;";
            }
        }
        else { // "both_error"
            // 两者都标记错误，输出非相交修復后的结果
            res.finalPts = ptsAfterEnforce;
            if (obstViolEnforce) {
                res.qualityFlags |= QF_WARN_OBSTACLE_PENETRATED;
                res.warnDesc += "OBSTACLE_PENETRATED;";
            }
            if (interViolEnforce) {
                res.qualityFlags |= QF_WARN_INTERSECTION_REMAIN;
                res.warnDesc += "INTERSECT_REMAIN;";
            }
        }

        // 验证端点是否正确（应与 ptsRaw 首尾一致）
        if(!ptsRaw.empty() && !res.finalPts.empty()){
            // 强制保证端点精确
            res.finalPts.front() = ptsRaw.front();
            res.finalPts.back()  = ptsRaw.back();
        }
        return res;
    }
};
