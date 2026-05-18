#pragma once
/**
 * LocalDetour — Phase3 局部绕障（鲁棒版）
 *
 * 核心改进：
 *  1. 精确估算绕障所需偏移量（基于障碍物实际几何，二分搜索）
 *  2. 两侧（左/右）均评估，选代价更小的方向
 *  3. 支持 1~3 段 G1 连续贝塞尔拼接
 *  4. 整体形态保持：绕障偏移仅在 [tIn, tOut] 区间内，
 *     首尾用原曲线平滑拼入
 *  5. 非相交约束在绕障段适当降级
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
    double  tIn  = 0.0, tOut = 1.0;
    Point2D pIn,  pOut;
    Point2D tangIn, tangOut;
    Point2D pushDirLeft;   // 区间主法线左方向（相对 P0→P3 的左法线）
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
    double maxOffsetRatio_ = 2.0;   // 允许最大 2× 段长的侧向偏移（较宽松）

public:
    LocalDetour(const ObstacleSpatialIndex& idx,
                double safeMargin,
                double checkSpacing = 0.15,
                double maxOffsetRatio = 2.0)
        : idx_(idx), safeMargin_(safeMargin)
        , checkSpacing_(checkSpacing)
        , maxOffsetRatio_(maxOffsetRatio)
    {}

    // ══════════════════════════════════════════════
    // 主入口：对三次贝塞尔做局部绕障
    // ══════════════════════════════════════════════
    LocalDetourResult compute(
        const CubicBezier& curve,
        const std::string& samplingMode,
        double samplingParam)
    {
        LocalDetourResult res;

        // 精细采样（300点）检测违规
        auto intervals = findViolIntervals(curve, 300);
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

        for (auto& iv : merged) {
            // 追加原曲线 [tCur, tIn] 段
            appendOrig(curve, tCur, iv.tIn, full);

            // 构建绕障段（自动选最优方向和段数）
            auto det = buildDetour(curve, iv);

            if (!det.empty()) {
                appendPolyline(det, full);
                res.detourSegs++;
            } else {
                // 退化：追加原段（允许部分穿越）
                appendOrig(curve, iv.tIn, iv.tOut, full);
                res.success = false;
                Logger::warn("LocalDetour: detour failed for interval ["
                    + std::to_string(iv.tIn) + "," + std::to_string(iv.tOut) + "]");
            }
            tCur = iv.tOut;
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
    // 1. 检测违规区间
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

        // 计算 P0→P3 的左法线（用于判断左/右）
        Point2D axis   = (curve.ctrl[3] - curve.ctrl[0]).normalized();
        Point2D normL  = axis.rotLeft();

        std::vector<ViolInterval> ivs;
        for (int i = 0; i <= N; ) {
            if (!vio[i]) { ++i; continue; }
            int iS = std::max(0, i - 3);   // 前缓冲
            int j = i;
            while (j <= N && vio[j]) ++j;
            int iE = std::min(N, j + 3);   // 后缓冲

            ViolInterval iv;
            iv.tIn     = iS * 1.0 / N;
            iv.tOut    = iE * 1.0 / N;
            iv.pIn     = curve.eval(iv.tIn);
            iv.pOut    = curve.eval(iv.tOut);
            iv.tangIn  = curve.evalDeriv1(iv.tIn).normalized();
            iv.tangOut = curve.evalDeriv1(iv.tOut).normalized();
            iv.pushDirLeft = normL;
            ivs.push_back(iv);
            i = iE + 1;
        }
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
            if (ivs[k].tIn <= cur.tOut + 0.05) {
                cur.tOut    = std::max(cur.tOut, ivs[k].tOut);
                cur.pOut    = curve.eval(cur.tOut);
                cur.tangOut = curve.evalDeriv1(cur.tOut).normalized();
            } else {
                merged.push_back(cur);
                cur = ivs[k];
            }
        }
        merged.push_back(cur);
        for (auto& iv : merged) {
            iv.pIn     = curve.eval(iv.tIn);
            iv.pOut    = curve.eval(iv.tOut);
            iv.tangIn  = curve.evalDeriv1(iv.tIn).normalized();
            iv.tangOut = curve.evalDeriv1(iv.tOut).normalized();
        }
        return merged;
    }

    // ══════════════════════════════════════════════
    // 3. 构建绕障折线（核心）
    //    对两侧均尝试，选择偏移量更小的成功方案
    // ══════════════════════════════════════════════
    Polyline buildDetour(const CubicBezier& curve, const ViolInterval& iv) const
    {
        Point2D leftDir  =  iv.pushDirLeft;
        Point2D rightDir = iv.pushDirLeft * (-1.0);

        // 二分搜索确定两侧所需最小偏移
        double offL = binarySearchOffset(curve, iv, leftDir);
        double offR = binarySearchOffset(curve, iv, rightDir);

        Logger::debug("LocalDetour: offLeft=" + std::to_string(offL)
                    + " offRight=" + std::to_string(offR));

        // 选偏移更小的方向（若两侧均失败则各尝试最大值）
        struct Candidate { Point2D dir; double off; };
        std::vector<Candidate> cands;
        if (offL >= 0) cands.push_back({leftDir,  offL});
        if (offR >= 0) cands.push_back({rightDir, offR});

        if (cands.empty()) {
            // 两侧二分均未找到，使用两侧最大偏移各尝试
            double segLen = dist(iv.pIn, iv.pOut);
            double maxOff = segLen * maxOffsetRatio_;
            cands.push_back({leftDir,  maxOff});
            cands.push_back({rightDir, maxOff});
        }

        // 按偏移量从小到大排序（先尝试小变形）
        std::sort(cands.begin(), cands.end(),
            [](const Candidate& a, const Candidate& b){ return a.off < b.off; });

        // 对每个候选，依次尝试 1/2/3 段贝塞尔
        for (auto& cand : cands) {
            // 1段
            auto pts = buildOneSeg(iv, cand.dir, cand.off);
            if (!pts.empty() && noViolation(pts)) return pts;
            // 2段
            pts = buildTwoSeg(curve, iv, cand.dir, cand.off);
            if (!pts.empty() && noViolation(pts)) return pts;
            // 3段
            pts = buildThreeSeg(curve, iv, cand.dir, cand.off);
            if (!pts.empty() && noViolation(pts)) return pts;
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
        double hi = segLen * maxOffsetRatio_;

        // 先检查 hi 是否有效（若 hi 也无法绕过，直接返回 -1）
        {
            auto pts = buildTwoSeg(curve, iv, dir, hi);
            if (pts.empty() || !noViolation(pts)) return -1;
        }

        // 二分搜索最小偏移
        for (int iter = 0; iter < 20; ++iter) {
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
    //    (pIn,tangIn) → (pOut,tangOut)，控制点向 dir 方向偏移 offset
    // ══════════════════════════════════════════════
    Polyline buildOneSeg(
        const ViolInterval& iv,
        const Point2D& dir, double offset) const
    {
        if (offset < EPS) return {};
        double d = dist(iv.pIn, iv.pOut);
        if (d < EPS) return {};

        double alpha = 0.40, beta = 0.40;
        Point2D P1 = iv.pIn  + iv.tangIn  * (alpha * d) + dir * offset;
        Point2D P2 = iv.pOut + iv.tangOut * (beta  * d) + dir * offset;
        CubicBezier cb(iv.pIn, P1, P2, iv.pOut);
        auto pts = cb.sampleAdaptive(0.3, 0.8, 0.04);
        if (!pts.empty()) { pts.front() = iv.pIn; pts.back() = iv.pOut; }
        return pts;
    }

    // ══════════════════════════════════════════════
    // 4b. 两段绕障（经过一个中间绕障点 M）
    //    G1 拼接：M 处切线由段1末切线决定
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

        // 中间点处的切线方向：沿 pIn→pOut 方向
        Point2D chord   = (iv.pOut - iv.pIn).normalized();
        Point2D tangAtM = chord; // 近似；更好的办法：与 dir 垂直

        // 段1: pIn → M
        double d1 = dist(iv.pIn, M);
        if (d1 < EPS) return {};
        double a1 = 0.38, b1 = 0.38;
        Point2D P1_1 = iv.pIn + iv.tangIn  * (a1 * d1);
        Point2D P1_2 = M      - tangAtM    * (b1 * d1);
        CubicBezier seg1(iv.pIn, P1_1, P1_2, M);

        // G1 拼接：段2首切线 = 段1末切线
        Point2D tM = seg1.evalDeriv1(1.0).normalized();

        // 段2: M → pOut
        double d2 = dist(M, iv.pOut);
        if (d2 < EPS) return {};
        double a2 = 0.38, b2 = 0.38;
        Point2D P2_1 = M       + tM           * (a2 * d2);
        Point2D P2_2 = iv.pOut + iv.tangOut   * (b2 * d2);
        CubicBezier seg2(M, P2_1, P2_2, iv.pOut);

        Polyline pts = seg1.sampleAdaptive(0.3, 0.8, 0.04);
        auto p2      = seg2.sampleAdaptive(0.3, 0.8, 0.04);
        for (size_t i = 1; i < p2.size(); ++i) pts.push_back(p2[i]);
        if (!pts.empty()) { pts.front() = iv.pIn; pts.back() = iv.pOut; }
        return pts;
    }

    // ══════════════════════════════════════════════
    // 4c. 三段绕障（经过两个绕障点 M1, M2）
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

        // 三段：pIn→M1, M1→M2, M2→pOut，G1 拼接
        auto mkSeg = [](const Point2D& A, const Point2D& tA,
                        const Point2D& B, const Point2D& tB) -> CubicBezier {
            double d = dist(A, B);
            if (d < EPS) return CubicBezier(A, A, B, B);
            double a = 0.35, b = 0.35;
            return CubicBezier(A, A + tA*(a*d), B + tB*(b*d), B);
        };

        Point2D t01 = (M1 - iv.pIn).normalized();
        Point2D t12 = (M2 - M1).normalized();
        Point2D t23 = (iv.pOut - M2).normalized();

        CubicBezier s1 = mkSeg(iv.pIn, iv.tangIn, M1, t01);
        Point2D tAtM1  = s1.evalDeriv1(1.0).normalized();
        CubicBezier s2 = mkSeg(M1, tAtM1, M2, t12);
        Point2D tAtM2  = s2.evalDeriv1(1.0).normalized();
        CubicBezier s3 = mkSeg(M2, tAtM2, iv.pOut, iv.tangOut);

        Polyline pts = s1.sampleAdaptive(0.3, 0.8, 0.04);
        for (auto& s : {s2.sampleAdaptive(0.3,0.8,0.04),
                        s3.sampleAdaptive(0.3,0.8,0.04)}) {
            for (size_t i = 1; i < s.size(); ++i) pts.push_back(s[i]);
        }
        if (!pts.empty()) { pts.front() = iv.pIn; pts.back() = iv.pOut; }
        return pts;
    }

    // ══════════════════════════════════════════════
    // 辅助
    // ══════════════════════════════════════════════
    bool noViolation(const Polyline& pts) const {
        return idx_.checkViolations(pts, safeMargin_).empty();
    }

    void appendOrig(const CubicBezier& c, double t0, double t1, Polyline& out) const {
        if (t1 <= t0 + 1e-6) return;
        int N = std::max(4, (int)((t1 - t0) * 200));
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
