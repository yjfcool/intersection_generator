#pragma once
/**
 * LocalDetour — Phase3 局部绕障（G1 连续平滑版）
 *
 * 核心要点：
 *  1. 自适应扩展违规区间缓冲，使绕障"接入/接出"位置远离障碍且原曲线
 *     在该处近似直线；
 *  2. 严格使用原曲线在 tIn/tOut 处的精确切线作为绕障段端点切线
 *     （G1 连续，避免大角度拐点）；
 *  3. 中间绕行点处的切线由"沿弦方向"+"段间 G1"双重保证；
 *  4. 二分搜索最小所需偏移量，避免过度变形；
 *  5. 单段/两段/三段方案择优，按偏移量从小到大尝试；
 *  6. 整体形态保持：绕障变形仅在 [tIn, tOut] 区间内，首尾用原曲线平滑拼入。
 */
#include "../data_types.h"
#include "../bezier/cubic_bezier.h"
#include "../preprocess/obstacle_index.h"
#include "../config.h"
#include "../../utils/geom_utils.h"
#include "../../utils/logger.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <functional>

// ─── 违规区间 ───────────────────────────────────
struct ViolInterval {
    double  tIn  = 0.0, tOut = 1.0;     // 区间在原曲线上的参数（含缓冲后）
    Point2D pIn,  pOut;                 // 原曲线上对应坐标
    Point2D tangIn, tangOut;             // 原曲线在该处的精确单位切线
    Point2D pushDirLeft;                // P0→P3 的左法线（用于左/右选择）
};

// ─── 绕障结果 ────────────────────────────────────
struct LocalDetourResult {
    Polyline finalPts;
    bool     success    = false;
    bool     relaxedNI  = true;
    int      detourSegs = 0;
};

// ─────────────────────────────────────────────────
class LocalDetour {
    const ObstacleSpatialIndex& idx_;
    double safeMargin_;
    double checkSpacing_;
    // 最大允许的法线偏移/原段长比（保形约束）
    double maxOffsetRatio_ = 2.0;
    // 右侧通行优先阈值（路径长度差 <= 此值时优先右侧）
    double rightSidePreferThreshold_ = 2.0;
    // New optimization parameters
    double minGapWidth_ = 2.8;
    double maxCurvatureJump_ = 0.15;
    double phase3BufTMax_ = 0.35;
    bool   enableGapAnalysis_ = true;
    bool   enableCorridorConstraint_ = true;
    const Corridor* corridor_ = nullptr;

    // G1 拼接的角度容差（度），仅用于诊断日志
    static constexpr double G1_TOL_DEG = 1.0;

public:
    LocalDetour(const ObstacleSpatialIndex& idx,
                double safeMargin,
                double checkSpacing = 0.15,
                double maxOffsetRatio = 2.0,
                double rightSidePreferThreshold = 2.0,
                double minGapWidth = 2.8,
                double maxCurvatureJump = 0.15,
                double phase3BufTMax = 0.35,
                bool   enableGapAnalysis = true,
                bool   enableCorridorConstraint = true)
        : idx_(idx), safeMargin_(safeMargin)
        , checkSpacing_(checkSpacing)
        , maxOffsetRatio_(maxOffsetRatio)
        , rightSidePreferThreshold_(rightSidePreferThreshold)
        , minGapWidth_(minGapWidth)
        , maxCurvatureJump_(maxCurvatureJump)
        , phase3BufTMax_(phase3BufTMax)
        , enableGapAnalysis_(enableGapAnalysis)
        , enableCorridorConstraint_(enableCorridorConstraint)
    {}

    void setCorridor(const Corridor* c) { corridor_ = c; }

    // ══════════════════════════════════════════════
    // 主入口：对三次贝塞尔做局部绕障
    // ══════════════════════════════════════════════
    LocalDetourResult compute(
        const CubicBezier& curve,
        const std::string& samplingMode,
        double samplingParam)
    {
        LocalDetourResult res;

        const int N = 400; // 高密度采样以精确定位违规
        auto intervals = findViolIntervals(curve, N);
        if (intervals.empty()) {
            res.finalPts = sampleCurve(curve, samplingMode, samplingParam);
            res.success  = true;
            return res;
        }

        Logger::info("LocalDetour: " + std::to_string(intervals.size())
                     + " viol interval(s), building detour...");

        auto merged = mergeIntervals(intervals, curve);

        Polyline full;
        double tCur = 0.0;

        for (size_t ivIdx = 0; ivIdx < merged.size(); ++ivIdx) {
            auto& iv = merged[ivIdx];
            // Determine expansion limits to avoid overlap with neighbors
            double nextTIn = (ivIdx + 1 < merged.size()) ? merged[ivIdx + 1].tIn : 1.0;

            // Curvature continuity feedback: retry with expanded interval if junction angle too large
            Polyline det;
            ViolInterval expandedIv = iv;
            for (int retry = 0; retry < 4; ++retry) {
                det = buildDetour(curve, expandedIv);
                if (det.empty()) break;
                double jAngle = measureJunctionAngle(curve, expandedIv, det);
                if (jAngle <= maxCurvatureJump_) break;  // Smooth enough
                // Expand interval by 15% each retry
                double span = expandedIv.tOut - expandedIv.tIn;
                double expand = span * 0.15;
                expandedIv.tIn  = std::max(0.0, expandedIv.tIn - expand);
                expandedIv.tOut = std::min(1.0, expandedIv.tOut + expand);
                // Clamp to avoid overlap: tIn must not go below tCur, tOut must not exceed next interval's tIn
                expandedIv.tIn  = std::max(tCur, expandedIv.tIn);
                expandedIv.tOut = std::min(nextTIn, expandedIv.tOut);
                expandedIv.pIn  = curve.eval(expandedIv.tIn);
                expandedIv.pOut = curve.eval(expandedIv.tOut);
                expandedIv.tangIn  = safeNormalize(curve.evalDeriv1(expandedIv.tIn));
                expandedIv.tangOut = safeNormalize(curve.evalDeriv1(expandedIv.tOut));
                // Recompute pushDirLeft at new midpoint
                double newMidT = 0.5 * (expandedIv.tIn + expandedIv.tOut);
                expandedIv.pushDirLeft = safeNormalize(curve.evalDeriv1(newMidT)).rotLeft();
                Logger::debug("LocalDetour: curvature jump " + std::to_string(jAngle * 180.0 / M_PI)
                              + " deg > " + std::to_string(maxCurvatureJump_ * 180.0 / M_PI)
                              + " deg, expanding buffer (retry " + std::to_string(retry+1) + ")");
            }

            // Use expandedIv for the actual interval boundaries
            // Append original curve from tCur to the (possibly expanded) start
            double actualTIn = std::max(tCur, expandedIv.tIn);
            appendOrig(curve, tCur, actualTIn, full);

            if (!det.empty()) {
                appendPolyline(det, full);
                res.detourSegs++;
                tCur = expandedIv.tOut;
            } else {
                // 退化：追加原段（允许部分穿越）
                appendOrig(curve, actualTIn, iv.tOut, full);
                res.success = false;
                Logger::warn("LocalDetour: detour failed for interval ["
                    + std::to_string(iv.tIn) + "," + std::to_string(iv.tOut) + "]");
                tCur = iv.tOut;
            }
        }
        appendOrig(curve, tCur, 1.0, full);

        // 强制端点精确
        if (!full.empty()) {
            full.front() = curve.ctrl[0];
            full.back()  = curve.ctrl[3];
        }

        // 验证
        auto remain = idx_.checkViolations(full, safeMargin_);
        res.success  = remain.empty();
        res.finalPts = full;
        res.relaxedNI = true;

        if (res.success)
            Logger::info("LocalDetour: success, segs=" + std::to_string(res.detourSegs));
        else
            Logger::warn("LocalDetour: " + std::to_string(remain.size())
                         + " violation(s) remain.");
        return res;
    }

    // 复合贝塞尔版
    LocalDetourResult computeComposite(
        const CompositeBezier& comp,
        const std::string& samplingMode, double samplingParam)
    {
        auto r1 = compute(comp.seg1, samplingMode, samplingParam);
        auto r2 = compute(comp.seg2, samplingMode, samplingParam);
        LocalDetourResult res;
        res.finalPts = r1.finalPts;
        for (size_t i = 1; i < r2.finalPts.size(); ++i)
            res.finalPts.push_back(r2.finalPts[i]);
        res.success    = r1.success && r2.success;
        res.detourSegs = r1.detourSegs + r2.detourSegs;
        res.relaxedNI  = true;
        return res;
    }

private:
    // ══════════════════════════════════════════════
    // 1. 检测违规区间（含自适应缓冲扩展）
    //    缓冲量取决于：障碍物附近的曲线段长 + safeMargin（保证接入点处
    //    曲线偏离障碍物足够远，且接入处切线变化平缓，以利后续 G1 拼接）
    // ══════════════════════════════════════════════
    std::vector<ViolInterval> findViolIntervals(
        const CubicBezier& curve, int N) const
    {
        std::vector<Point2D> pts;
        pts.reserve(N + 1);
        for (int i = 0; i <= N; ++i)
            pts.push_back(curve.eval(i * 1.0 / N));

        auto viols = idx_.checkViolations(pts, safeMargin_);
        if (viols.empty()) return {};

        std::vector<bool> vio(N + 1, false);
        for (auto& v : viols) vio[v.ptIdx] = true;

        // 计算 P0→P3 的左法线（用于判断左/右的 fallback）
        Point2D axis   = (curve.ctrl[3] - curve.ctrl[0]).normalized();
        (void)axis; // kept for reference, per-interval tangent-based direction used instead

        // 自适应缓冲：以 t 参数空间为单位，至少 0.05（即 5%曲线参数）；
        // 同时根据 safeMargin 与平均段长动态调整。这是保证 G1 平滑过渡的关键：
        // tIn/tOut 处缓冲越宽，原曲线该处方向变化越平缓，与绕障段拼接越顺滑。
        const double curveLen = curve.arcLength(50);
        const double avgSegT  = (curveLen > EPS) ? (1.0 / std::max(8.0, curveLen / 0.5)) : 0.01;
        // 缓冲在 t 空间的目标宽度：不小于 0.06，且至少 = 2 倍 safeMargin 对应的弧长
        double bufT = std::max(0.06, std::min(phase3BufTMax_,
                        2.0 * safeMargin_ / std::max(curveLen, 1.0)));
        int bufN = std::max(6, (int)std::round(bufT * N));

        std::vector<ViolInterval> ivs;
        for (int i = 0; i <= N; ) {
            if (!vio[i]) { ++i; continue; }
            int j = i;
            while (j <= N && vio[j]) ++j;
            // [i, j-1] 是连续违规段；扩展缓冲
            int iS = std::max(0, i - bufN);
            int iE = std::min(N, j - 1 + bufN);

            ViolInterval iv;
            iv.tIn     = iS * 1.0 / N;
            iv.tOut    = iE * 1.0 / N;
            iv.pIn     = curve.eval(iv.tIn);
            iv.pOut    = curve.eval(iv.tOut);
            iv.tangIn  = safeNormalize(curve.evalDeriv1(iv.tIn));
            iv.tangOut = safeNormalize(curve.evalDeriv1(iv.tOut));
            // Use tangent at violation midpoint for true travel direction
            double tMidViol = (iS + iE) * 0.5 / N;
            Point2D tangAtMid = safeNormalize(curve.evalDeriv1(tMidViol));
            iv.pushDirLeft = tangAtMid.rotLeft();
            ivs.push_back(iv);
            i = iE + 1;
        }
        (void)avgSegT;
        return ivs;
    }

    // ══════════════════════════════════════════════
    // 2. 合并相邻/重叠区间
    // ══════════════════════════════════════════════
    std::vector<ViolInterval> mergeIntervals(
        const std::vector<ViolInterval>& ivs,
        const CubicBezier& curve) const
    {
        if (ivs.empty()) return {};
        std::vector<ViolInterval> merged;
        ViolInterval cur = ivs[0];
        for (size_t k = 1; k < ivs.size(); ++k) {
            // 区间紧邻或重叠（缓冲后），合并为一段处理
            if (ivs[k].tIn <= cur.tOut + 0.05) {
                cur.tOut = std::max(cur.tOut, ivs[k].tOut);
            } else {
                merged.push_back(cur);
                cur = ivs[k];
            }
        }
        merged.push_back(cur);
        // 重新计算合并后区间的端点和切线
        for (auto& iv : merged) {
            iv.pIn     = curve.eval(iv.tIn);
            iv.pOut    = curve.eval(iv.tOut);
            iv.tangIn  = safeNormalize(curve.evalDeriv1(iv.tIn));
            iv.tangOut = safeNormalize(curve.evalDeriv1(iv.tOut));
        }
        return merged;
    }

    // ══════════════════════════════════════════════
    // 3. 构建绕障折线（核心）
    //    对两侧均尝试，选择偏移量更小的成功方案
    //    右侧通行优先：当左右路径长度差 <= rightSidePreferThreshold_ 时优先右侧
    // ══════════════════════════════════════════════
    Polyline buildDetour(const CubicBezier& curve, const ViolInterval& iv) const
    {
        // Gap analysis: try passing through gaps between obstacles first
        if (enableGapAnalysis_) {
            auto gapResult = tryGapPassthrough(curve, iv);
            if (!gapResult.empty() && noViolation(gapResult)) {
                Logger::debug("LocalDetour: gap passthrough successful");
                return gapResult;
            }
        }

        Point2D leftDir  = iv.pushDirLeft;
        Point2D rightDir = iv.pushDirLeft * (-1.0);

        // 二分搜索确定两侧所需最小偏移（用最稳健的两段 G1 方案）
        double offL = binarySearchOffset(curve, iv, leftDir);
        double offR = binarySearchOffset(curve, iv, rightDir);

        Logger::debug("LocalDetour: offLeft=" + std::to_string(offL)
                    + " offRight=" + std::to_string(offR));

        // 构建两侧的最佳绕障路径，然后根据路径长度和右侧优先规则选择
        struct DetourCandidate {
            Polyline pts;
            double   pathLen = 0;
            bool     isRight = false;
            bool     valid   = false;
        };

        auto tryBuild = [&](const Point2D& dir, double off, bool isRight) -> DetourCandidate {
            DetourCandidate c;
            c.isRight = isRight;
            if (off < 0) return c;
            // For large offsets, try trapezoidal profile first (smoother transitions)
            if (off > 2.0 * safeMargin_) {
                c.pts = buildTrapezoidalDetour(curve, iv, dir, off);
                if (!c.pts.empty() && noViolation(c.pts)) {
                    c.pts = checkCorridorConstraint(c.pts);
                    if (!c.pts.empty()) { c.valid = true; c.pathLen = polylineLength(c.pts); return c; }
                }
            }
            // 两段
            c.pts = buildTwoSeg(curve, iv, dir, off);
            if (!c.pts.empty() && noViolation(c.pts)) {
                c.pts = checkCorridorConstraint(c.pts);
                if (!c.pts.empty()) { c.valid = true; c.pathLen = polylineLength(c.pts); return c; }
            }
            // 单段
            c.pts = buildOneSeg(iv, dir, off);
            if (!c.pts.empty() && noViolation(c.pts)) {
                c.pts = checkCorridorConstraint(c.pts);
                if (!c.pts.empty()) { c.valid = true; c.pathLen = polylineLength(c.pts); return c; }
            }
            // 三段
            c.pts = buildThreeSeg(curve, iv, dir, off);
            if (!c.pts.empty() && noViolation(c.pts)) {
                c.pts = checkCorridorConstraint(c.pts);
                if (!c.pts.empty()) { c.valid = true; c.pathLen = polylineLength(c.pts); return c; }
            }
            return c;
        };

        DetourCandidate candL = tryBuild(leftDir,  offL, false);
        DetourCandidate candR = tryBuild(rightDir, offR, true);

        // 若两侧均失败，用最大偏移再试一次
        if (!candL.valid && !candR.valid) {
            double segLen = dist(iv.pIn, iv.pOut);
            double maxOff = segLen * maxOffsetRatio_;
            candL = tryBuild(leftDir,  maxOff, false);
            candR = tryBuild(rightDir, maxOff, true);
        }

        // 决策逻辑：
        //  - 若仅一侧有效，用那侧
        //  - 若两侧均有效：路径长度差 <= threshold 时优先右侧；否则选较短的
        if (candL.valid && candR.valid) {
            double diff = candR.pathLen - candL.pathLen;
            if (diff <= rightSidePreferThreshold_) {
                // 右侧路径不比左侧长太多，优先右侧（右侧通行规则）
                Logger::debug("LocalDetour: prefer right (diff=" + std::to_string(diff) + "m)");
                logG1(iv, candR.pts, "right-prefer");
                return candR.pts;
            } else {
                // 左侧明显更短，选左侧
                Logger::debug("LocalDetour: prefer left (shorter by " + std::to_string(diff) + "m)");
                logG1(iv, candL.pts, "left-shorter");
                return candL.pts;
            }
        } else if (candR.valid) {
            logG1(iv, candR.pts, "right-only");
            return candR.pts;
        } else if (candL.valid) {
            logG1(iv, candL.pts, "left-only");
            return candL.pts;
        }
        return {};
    }

    // ── 二分搜索：确定某方向上使曲线清除障碍物的最小偏移量 ──────────
    // 返回 -1 表示该方向无法找到（超过 maxOffsetRatio）
    double binarySearchOffset(
        const CubicBezier& curve,
        const ViolInterval& iv,
        const Point2D& dir) const
    {
        double segLen = dist(iv.pIn, iv.pOut);
        if (segLen < EPS) return -1;

        double lo = 0.0;
        // 上界：以 segLen 与 maxOffsetRatio 为基准，但同时不超过 30m。
        // 过大的 hi 会让 buildTwoSeg 中的中间点 M 远离障碍物，
        // 反而生成形态扭曲的曲线。
        double hi = std::min(segLen * maxOffsetRatio_, 30.0);
        // 上界至少 = 2 倍 safeMargin，确保最小情况下也能搜到
        hi = std::max(hi, safeMargin_ * 4.0);

        // 先检查 hi 是否有效（若 hi 也无法绕过，直接返回 -1）
        {
            auto pts = buildTwoSeg(curve, iv, dir, hi);
            if (pts.empty() || !noViolation(pts)) return -1;
        }

        // 二分搜索最小偏移
        for (int iter = 0; iter < 18; ++iter) {
            double mid = 0.5 * (lo + hi);
            auto pts = buildTwoSeg(curve, iv, dir, mid);
            bool ok = !pts.empty() && noViolation(pts);
            if (ok) hi = mid;
            else    lo = mid;
        }
        return hi;
    }

    // ══════════════════════════════════════════════
    // 4a. 单段绕障贝塞尔
    //    严格使用 iv.tangIn/tangOut（来自原曲线在 tIn/tOut 处的精确切线）
    //    控制点向 dir 方向偏移 offset
    // ══════════════════════════════════════════════
    Polyline buildOneSeg(
        const ViolInterval& iv,
        const Point2D& dir, double offset) const
    {
        if (offset < EPS) return {};
        double d = dist(iv.pIn, iv.pOut);
        if (d < EPS) return {};

        // 一段绕障：保持 G1 连续要求 P1 仅沿 tangIn 方向延伸（不能有
        // dir 方向偏移分量），P2 同理。要让控制多边形覆盖 dir 方向，
        // 只能依赖 tangIn/tangOut 在 dir 上的投影；若两者都与 dir 近乎
        // 垂直，单段无法绕远——此时调用方会回退到两段方案。
        double alpha = 0.45, beta = 0.45;
        // 让控制手柄更长一些以增加形态自由度，但严格保留切线方向
        Point2D P1 = iv.pIn  + iv.tangIn  * (alpha * d);
        Point2D P2 = iv.pOut - iv.tangOut * (beta  * d);
        // 在两个内部控制点上同时加 dir*offset，对端点切线无影响：
        // B'(0) = 3*(P1-P0) = 3*tangIn*alpha*d + 3*dir*offset → 切线方向受影响！
        // 所以这里不再加 offset，单段绕障形态主要靠 tangIn/tangOut 自身倾斜。
        // 单段方案仅适用于 tangIn、tangOut 在 dir 上有足够投影的情形。
        CubicBezier cb(iv.pIn, P1, P2, iv.pOut);
        // 检查曲线是否真的偏向 dir 方向（中点偏移要 >= offset 一半）
        Point2D midPt = cb.eval(0.5);
        Point2D origMid = (iv.pIn + iv.pOut) * 0.5;
        if ((midPt - origMid).dot(dir) < offset * 0.4) {
            // 偏移不足，单段方案不可行
            return {};
        }
        auto pts = denseSample(cb);
        if (!pts.empty()) { pts.front() = iv.pIn; pts.back() = iv.pOut; }
        return pts;
    }

    // ══════════════════════════════════════════════
    // 4b. 两段绕障（经过一个中间绕障点 M）
    //    G1 拼接：
    //      段1 起点切线 = iv.tangIn （原曲线切线，G1接入原曲线）
    //      段1 末切线   = 段2 首切线（M 处 G1 自然连续）
    //      段2 末切线   = iv.tangOut（原曲线切线，G1接出原曲线）
    //    M 处切线方向 tangAtM 取与 dir 垂直、与弦同向那个。
    // ══════════════════════════════════════════════
    Polyline buildTwoSeg(
        const CubicBezier& origCurve,
        const ViolInterval& iv,
        const Point2D& dir, double offset) const
    {
        if (offset < EPS) return {};
        double tMid = 0.5 * (iv.tIn + iv.tOut);
        Point2D origM = origCurve.eval(tMid);
        Point2D M = origM + dir * offset;

        // 中间点切线方向：与 dir 垂直，取与弦方向同向那一个
        Point2D chord   = (iv.pOut - iv.pIn).normalized();
        Point2D perp    = dir.rotLeft().normalized();
        Point2D tangAtM = (perp.dot(chord) >= 0) ? perp : (perp * -1.0);

        // 控制手柄长度策略：
        //  - 应保证 P1 不会越过 M 太远，否则曲线在 P0→M 间出现 S 形回折；
        //  - 应保证 P2 不会越过 P0 太远，否则同样回折；
        //  - 经验值：基于"M 沿 tangIn 方向上的投影距离"作上限，
        //    再用 0.35 系数缩短，能在保形和绕障之间取得平衡。
        double d1 = dist(iv.pIn, M);
        double d2 = dist(M,  iv.pOut);
        if (d1 < EPS || d2 < EPS) return {};

        // 段1：手柄沿 tangIn 投影到 (M-pIn) 的标量长度
        double proj_in_1  = std::max(0.5, std::abs((M - iv.pIn).dot(iv.tangIn)));
        double proj_mid_1 = std::max(0.5, std::abs((M - iv.pIn).dot(tangAtM)));
        double h1_a = std::min(0.40 * d1, 0.55 * proj_in_1);
        double h1_b = std::min(0.40 * d1, 0.55 * proj_mid_1);

        // 段2：类似地处理
        double proj_mid_2 = std::max(0.5, std::abs((iv.pOut - M).dot(tangAtM)));
        double proj_out_2 = std::max(0.5, std::abs((iv.pOut - M).dot(iv.tangOut)));
        double h2_a = std::min(0.40 * d2, 0.55 * proj_mid_2);
        double h2_b = std::min(0.40 * d2, 0.55 * proj_out_2);

        // 段1: pIn → M（首切线 = iv.tangIn，末切线 = tangAtM）
        Point2D P1_1 = iv.pIn + iv.tangIn * h1_a;
        Point2D P1_2 = M      - tangAtM   * h1_b;
        CubicBezier seg1(iv.pIn, P1_1, P1_2, M);

        // 段2: M → pOut（首切线 = tangAtM，末切线 = iv.tangOut）
        Point2D P2_1 = M       + tangAtM    * h2_a;
        Point2D P2_2 = iv.pOut - iv.tangOut * h2_b;
        CubicBezier seg2(M, P2_1, P2_2, iv.pOut);

        Polyline pts = denseSample(seg1);
        auto p2      = denseSample(seg2);
        for (size_t i = 1; i < p2.size(); ++i) pts.push_back(p2[i]);
        if (!pts.empty()) { pts.front() = iv.pIn; pts.back() = iv.pOut; }
        return pts;
    }

    // ══════════════════════════════════════════════
    // 4c. 三段绕障（经过两个绕障点 M1, M2）
    //    G1 拼接：所有相邻段的接缝处切线一致
    // ══════════════════════════════════════════════
    Polyline buildThreeSeg(
        const CubicBezier& origCurve,
        const ViolInterval& iv,
        const Point2D& dir, double offset) const
    {
        if (offset < EPS) return {};
        double span = iv.tOut - iv.tIn;
        double tM1  = iv.tIn + span * 0.33;
        double tM2  = iv.tIn + span * 0.67;

        Point2D M1 = origCurve.eval(tM1) + dir * offset;
        Point2D M2 = origCurve.eval(tM2) + dir * offset;

        // 中间两点的切线统一沿 pIn→pOut 方向（与 dir 垂直分量），
        // 保证 G1 流畅；接入/接出端用 iv.tangIn/tangOut。
        Point2D chord   = (iv.pOut - iv.pIn).normalized();
        Point2D perp    = dir.rotLeft().normalized();
        Point2D tangMid = (perp.dot(chord) >= 0) ? perp : (perp * -1.0);

        auto mkSeg = [](const Point2D& A, const Point2D& tA,
                        const Point2D& B, const Point2D& tB) -> CubicBezier {
            double d = dist(A, B);
            if (d < EPS) return CubicBezier(A, A, B, B);
            double a = 0.38, b = 0.38;
            // P1 = A + tA*(a*d),  P2 = B - tB*(b*d)
            return CubicBezier(A, A + tA*(a*d), B - tB*(b*d), B);
        };

        CubicBezier s1 = mkSeg(iv.pIn, iv.tangIn, M1, tangMid);
        CubicBezier s2 = mkSeg(M1,     tangMid,   M2, tangMid);
        CubicBezier s3 = mkSeg(M2,     tangMid,   iv.pOut, iv.tangOut);

        Polyline pts = denseSample(s1);
        for (auto& s : {denseSample(s2), denseSample(s3)}) {
            for (size_t i = 1; i < s.size(); ++i) pts.push_back(s[i]);
        }
        if (!pts.empty()) { pts.front() = iv.pIn; pts.back() = iv.pOut; }
        return pts;
    }

    // ══════════════════════════════════════════════
    // 4d. Curvature continuity measurement at junction points
    // ══════════════════════════════════════════════
    double measureJunctionAngle(const CubicBezier& curve, const ViolInterval& iv, const Polyline& detour) const {
        if (detour.size() < 3) return 0.0;
        // Angle at entry junction
        Point2D beforeDir = safeNormalize(curve.evalDeriv1(iv.tIn));
        Point2D afterDir = (detour[1] - detour[0]).normalized();
        double dotIn = std::max(-1.0, std::min(1.0, beforeDir.dot(afterDir)));
        double angleIn = std::acos(dotIn);
        // Angle at exit junction
        Point2D beforeEnd = (detour[detour.size()-1] - detour[detour.size()-2]).normalized();
        Point2D afterEnd = safeNormalize(curve.evalDeriv1(iv.tOut));
        double dotOut = std::max(-1.0, std::min(1.0, beforeEnd.dot(afterEnd)));
        double angleOut = std::acos(dotOut);
        return std::max(angleIn, angleOut);
    }

    // ══════════════════════════════════════════════
    // 4e. Trapezoidal offset profile for smoother detours
    //     Uses graduated 5-point offset: pIn -> 0.4*off -> full off -> 0.4*off -> pOut
    // ══════════════════════════════════════════════
    Polyline buildTrapezoidalDetour(const CubicBezier& origCurve, const ViolInterval& iv,
                                     const Point2D& dir, double offset) const {
        if (offset < EPS) return {};
        double span = iv.tOut - iv.tIn;
        if (span < EPS) return {};

        // 5 key points: pIn, P25(0.4*offset), Pmid(full offset), P75(0.4*offset), pOut
        double t25 = iv.tIn + span * 0.25;
        double tMid = iv.tIn + span * 0.50;
        double t75 = iv.tIn + span * 0.75;

        Point2D P25 = origCurve.eval(t25) + dir * (0.4 * offset);
        Point2D Pmid = origCurve.eval(tMid) + dir * offset;
        Point2D P75 = origCurve.eval(t75) + dir * (0.4 * offset);

        // Tangent directions along path
        Point2D chord = (iv.pOut - iv.pIn).normalized();
        Point2D perp = dir.rotLeft().normalized();
        Point2D tangMid;
        // Fallback: when chord is nearly parallel to dir, perp.dot(chord) is near zero
        // and sign can flip unpredictably. Use chord directly as tangMid in that case.
        if (std::abs(perp.dot(chord)) < 0.1) {
            tangMid = chord;
        } else {
            tangMid = (perp.dot(chord) >= 0) ? perp : (perp * -1.0);
        }

        // Tangents at intermediate points: blend between tangIn/tangOut and tangMid
        Point2D tang25 = safeNormalize(iv.tangIn * 0.5 + tangMid * 0.5);
        Point2D tang75 = safeNormalize(tangMid * 0.5 + iv.tangOut * 0.5);

        auto mkSeg = [&](const Point2D& A, const Point2D& tA,
                        const Point2D& B, const Point2D& tB, double scale = 0.35) -> CubicBezier {
            double d = dist(A, B);
            if (d < 1e-9) return CubicBezier(A, A, B, B);
            return CubicBezier(A, A + tA * (scale * d), B - tB * (scale * d), B);
        };

        CubicBezier s1 = mkSeg(iv.pIn, iv.tangIn, P25, tang25);
        CubicBezier s2 = mkSeg(P25, tang25, Pmid, tangMid);
        CubicBezier s3 = mkSeg(Pmid, tangMid, P75, tang75);
        CubicBezier s4 = mkSeg(P75, tang75, iv.pOut, iv.tangOut);

        Polyline pts = denseSample(s1, 30);
        auto sp2 = denseSample(s2, 30);
        for (size_t i = 1; i < sp2.size(); ++i) pts.push_back(sp2[i]);
        auto sp3 = denseSample(s3, 30);
        for (size_t i = 1; i < sp3.size(); ++i) pts.push_back(sp3[i]);
        auto sp4 = denseSample(s4, 30);
        for (size_t i = 1; i < sp4.size(); ++i) pts.push_back(sp4[i]);

        if (!pts.empty()) { pts.front() = iv.pIn; pts.back() = iv.pOut; }
        return pts;
    }

    // ══════════════════════════════════════════════
    // 4f. Gap analysis: try passing through gaps between obstacles
    // ══════════════════════════════════════════════
    Polyline tryGapPassthrough(const CubicBezier& curve, const ViolInterval& iv) const {
        // Scan perpendicular to curve at violation midpoint
        double tMid = 0.5 * (iv.tIn + iv.tOut);
        Point2D midPt = curve.eval(tMid);
        Point2D tangent = safeNormalize(curve.evalDeriv1(tMid));
        Point2D perpDir = tangent.rotLeft(); // perpendicular scan direction

        // Sample distances to obstacles along perpendicular line
        double scanRange = 15.0; // meters each side
        int scanSteps = 30;
        double stepSize = scanRange / scanSteps;

        struct GapInfo { double center; double width; };
        std::vector<GapInfo> gaps;

        bool inGap = false;
        double gapStart = -scanRange;

        for (int i = -scanSteps; i <= scanSteps; ++i) {
            double offset = i * stepSize;
            Point2D probe = midPt + perpDir * offset;
            double d = idx_.minDist(probe, std::max(safeMargin_ * 3, 5.0));
            if (d < safeMargin_) {
                if (inGap && (offset - gapStart) >= (minGapWidth_ + 2 * safeMargin_)) {
                    gaps.push_back({(gapStart + offset) * 0.5, offset - gapStart});
                }
                inGap = false;
            } else {
                if (!inGap) { gapStart = offset; inGap = true; }
            }
        }
        // Check final gap
        if (inGap && (scanRange - gapStart) >= (minGapWidth_ + 2 * safeMargin_)) {
            gaps.push_back({(gapStart + scanRange) * 0.5, scanRange - gapStart});
        }

        if (gaps.empty()) return {};

        // Find the gap closest to the original curve position (offset=0)
        double bestGapCenter = 0;
        double bestDist = 1e18;
        for (auto& g : gaps) {
            if (std::abs(g.center) < bestDist) {
                bestDist = std::abs(g.center);
                bestGapCenter = g.center;
            }
        }

        // Only use gap if it requires less offset than a full detour would
        if (std::abs(bestGapCenter) < safeMargin_ * 0.5) return {}; // Already near gap

        // Build a path through the gap using buildTwoSeg with reduced offset
        Point2D gapDir = perpDir * (bestGapCenter > 0 ? 1.0 : -1.0);
        double gapOffset = std::abs(bestGapCenter);
        return buildTwoSeg(curve, iv, gapDir, gapOffset);
    }

    // ══════════════════════════════════════════════
    // 4g. Corridor constraint check (soft constraint)
    //     Returns candidate if within corridor, empty if exceeds
    //     Uses signed distance heuristic against corridor boundaries
    // ══════════════════════════════════════════════
    Polyline checkCorridorConstraint(const Polyline& candidate) const {
        if (!enableCorridorConstraint_ || !corridor_) return candidate;
        if (!corridor_->leftBoundary && !corridor_->rightBoundary) return candidate;

        bool hasLeft  = corridor_->leftBoundary  && corridor_->leftBoundary->size() >= 2;
        bool hasRight = corridor_->rightBoundary && corridor_->rightBoundary->size() >= 2;

        // Only enforce corridor constraint when BOTH boundaries are defined.
        // With a single boundary, the corridor is unbounded on one side and
        // rejecting based on distance from one edge is too aggressive.
        if (!hasLeft || !hasRight) return candidate;

        // When both boundaries exist, reject a candidate if any point is far
        // from both (i.e., outside the corridor envelope).
        // Threshold: if point is > 5x minHalfWidth from BOTH boundaries,
        // it's likely outside the corridor entirely.
        double threshold = std::max(corridor_->minHalfWidth * 5.0, 3.0); // at least 3m
        for (auto& pt : candidate) {
            double dL = pointToPolylineDist(pt, *corridor_->leftBoundary);
            double dR = pointToPolylineDist(pt, *corridor_->rightBoundary);
            if (dL > threshold && dR > threshold) {
                return {}; // Point is far from both boundaries - outside corridor
            }
        }
        return candidate;
    }

    static double pointToPolylineDist(const Point2D& pt, const Polyline& poly) {
        double minD = std::numeric_limits<double>::max();
        for (size_t i = 0; i + 1 < poly.size(); ++i) {
            double d = pointToSegDist(pt, poly[i], poly[i+1]);
            if (d < minD) minD = d;
        }
        return minD;
    }

    static double pointToSegDist(const Point2D& p, const Point2D& a, const Point2D& b) {
        Point2D ab = b - a;
        double len2 = ab.x * ab.x + ab.y * ab.y;
        if (len2 < 1e-12) return dist(p, a);
        double t = std::max(0.0, std::min(1.0, ((p - a).dot(ab)) / len2));
        Point2D proj = a + ab * t;
        return dist(p, proj);
    }

    // ══════════════════════════════════════════════
    // 辅助
    // ══════════════════════════════════════════════
    bool noViolation(const Polyline& pts) const {
        return idx_.checkViolations(pts, safeMargin_).empty();
    }

    static Point2D safeNormalize(const Point2D& v) {
        double n = v.norm();
        if (n < EPS) return {1.0, 0.0};
        return {v.x / n, v.y / n};
    }

    // 用足够密集的等间距采样保留曲线形态（默认 50 段）
    Polyline denseSample(const CubicBezier& cb, int n = 60) const {
        Polyline pts;
        pts.reserve(n + 1);
        for (int i = 0; i <= n; ++i)
            pts.push_back(cb.eval(i * 1.0 / n));
        return pts;
    }

    // 诊断：检测绕障段两端与原曲线切线的夹角，超阈值则记日志
    void logG1(const ViolInterval& iv, const Polyline& detour,
               const std::string& tag) const
    {
        if (detour.size() < 2) return;
        Point2D tStart = (detour[1] - detour[0]).normalized();
        Point2D tEnd   = (detour.back() - detour[detour.size()-2]).normalized();
        double dStart = angleDeg(tStart, iv.tangIn);
        double dEnd   = angleDeg(tEnd,   iv.tangOut);
        if (dStart > G1_TOL_DEG * 5 || dEnd > G1_TOL_DEG * 5) {
            Logger::debug("LocalDetour [" + tag + "] G1 angle: in="
                + std::to_string(dStart) + " out=" + std::to_string(dEnd));
        }
    }

    // 在 [t0, t1] 范围内沿原曲线追加采样点（首尾 G1 平滑由切线一致性保证）
    void appendOrig(const CubicBezier& c, double t0, double t1, Polyline& out) const {
        if (t1 <= t0 + 1e-6) return;
        // 弧长自适应采样：保证段间间距合理
        int N = std::max(4, (int)std::round((t1 - t0) * 200));
        bool first = out.empty();
        for (int i = (first ? 0 : 1); i <= N; ++i)
            out.push_back(c.eval(t0 + (t1 - t0) * i / N));
    }

    void appendPolyline(const Polyline& src, Polyline& out) const {
        for (size_t i = (out.empty() ? 0 : 1); i < src.size(); ++i)
            out.push_back(src[i]);
    }

    Polyline sampleCurve(const CubicBezier& c,
        const std::string& mode, double param) const
    {
        Polyline pts;
        if      (mode == "fixed_spacing") pts = c.sampleBySpacing(param > 0 ? param : 0.5);
        else if (mode == "fixed_count")   pts = c.sampleCount((int)(param > 0 ? param : 50));
        else                              pts = c.sampleAdaptive(0.5, 2.0, 0.05);
        if (!pts.empty()) { pts.front() = c.ctrl[0]; pts.back() = c.ctrl[3]; }
        return pts;
    }
};

