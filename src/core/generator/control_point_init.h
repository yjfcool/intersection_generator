#pragma once
#include "../data_types.h"
#include "../bezier/cubic_bezier.h"
#include "../config.h"
#include "../../utils/geom_utils.h"

/**
 * 控制点初始化（经验公式）
 * 根据进入/退出端连接点、切线方向和转向类型初始化贝塞尔控制点
 */
class ControlPointInit {
public:
    struct InitResult {
        bool useComposite = false;  // 是否使用两段贝塞尔
        CubicBezier single;         // 单段贝塞尔
        CompositeBezier composite;  // 两段贝塞尔
    };

    static InitResult init(
        const Point2D& P0, const Point2D& T0,  // 进入连接点 & 切线（指向路口内）
        const Point2D& P3, const Point2D& T3,  // 退出连接点 & 切线（路口内→路口外，取反为进入方向）
        TurnType turn,
        const BezierConfig& cfg)
    {
        InitResult res;

        // 根据转向类型选择 alpha
        double alpha = getAlpha(turn, cfg);
        double beta  = alpha; // 对称

        double d = dist(P0, P3);
        if(d < EPS) {
            // 退化情况：起终点重合
            res.useComposite = false;
            res.single = CubicBezier(P0, P0+T0*0.1, P3-T3*0.1, P3);
            return res;
        }

        // 生成单段贝塞尔
        // T3 是指向路口外的方向（退出线的原始切线），
        // 作为P2→P3方向的反向，即 P2 = P3 - T3*beta*d
        // 注意：T3 此处传入的是 exitTangentInward（指向路口内），
        //       P2 = P3 + T3 * beta * d（从P3向路口内延伸beta*d）
        Point2D P1 = P0 + T0 * (alpha * d);
        Point2D P2 = P3 + T3 * (beta  * d); // T3指向路口内，P2在P3沿路口内侧，B'(1)=-T3方向

        res.single = CubicBezier(P0, P1, P2, P3);

        // 判断是否需要两段
        bool needTwo = false;
        if(turn == TurnType::U_TURN_LEFT || turn == TurnType::U_TURN_RIGHT){
            needTwo = true;
        } else {
            // 检查初始曲率（带容差：仅超阈值 10% 以上才触发复合贝塞尔）
            // 对于普通转弯，微超阈值时单段贝塞尔仍可接受，
            // 使用复合贝塞尔反而引入不必要的复杂性
            double maxK = res.single.maxCurvature(30);
            double threshold = cfg.maxCurvature * 1.10; // 10% tolerance
            if(maxK > threshold) needTwo = true;
        }

        if(needTwo){
            res.useComposite = true;
            res.composite = initComposite(P0,T0,P3,T3,turn,d,cfg);
            // 同时保留单段（若复合失败可回退）
        }

        return res;
    }

private:
    static double getAlpha(TurnType turn, const BezierConfig& cfg){
        switch(turn){
            case TurnType::STRAIGHT:     return cfg.alphaStraight;
            case TurnType::RIGHT:        return cfg.alphaRight;
            case TurnType::LEFT:         return cfg.alphaLeft;
            case TurnType::U_TURN_LEFT:
            case TurnType::U_TURN_RIGHT: return cfg.alphaUturn;
            default:                     return cfg.alphaStraight;
        }
    }

    static CompositeBezier initComposite(
        const Point2D& P0, const Point2D& T0,
        const Point2D& P3, const Point2D& T3,
        TurnType turn, double d,
        const BezierConfig& cfg)
    {
        // 调头/大曲率场景的两段 G1 复合贝塞尔
        //
        // 几何要点：
        //  - T0 指向路口内（进入方向）；T3 指向路口内（退出端的"反向"，
        //    即车辆离开时方向 = -T3）
        //  - 中间点 M 应位于"曲线自然外凸"那一侧（apex 方向）：
        //      apex 方向是垂直于弦(P3-P0)、且指向 T0 同侧的法线方向
        //      （这同时正确处理常规 90° 左/右转 与 180° 调头）
        //  - U-turn 场景下 P3 通常在 T0 的同侧或对侧（取决于车道对称性），
        //    chord 几乎与 T0 垂直，apex 方向 ≈ T0
        //  - 中间点切线 Tmid 与弦同向（保证段内不出现回折）
        //  - 两段使用相同 alpha 初始化，保证段间曲率分布平衡
        //  - 偏移量需保证形成完整 U 形：U-turn 用 ~0.55*max(d,基准)，
        //    其它大曲率用 cfg.twoSegMidOffsetRatio

        double alpha = cfg.alphaUturn;
        bool isUturn = (turn == TurnType::U_TURN_LEFT
                     || turn == TurnType::U_TURN_RIGHT);

        // ── 计算弦方向 ──
        Point2D chord = P3 - P0;
        double  chordLen = chord.norm();
        if (chordLen < EPS) {
            // P0/P3 极度接近：用 T0 的左/右法线作弦方向
            chord = (turn == TurnType::U_TURN_RIGHT) ? T0.rotRight() : T0.rotLeft();
            chordLen = chord.norm();
            if (chordLen < EPS) chord = {1.0, 0.0};
        }
        Point2D chordN = chord * (1.0 / chordLen);

        // ── 计算 apex（外凸）方向：垂直于弦，且与 T0 同侧 ──
        // chord.rotLeft 与 chord.rotRight 互为反向，选与 T0 点积更大的那个
        Point2D nL = chordN.rotLeft();
        Point2D nR = chordN.rotRight();
        Point2D offDir = (nL.dot(T0) >= nR.dot(T0)) ? nL : nR;

        // ── 计算偏移量 ──
        // U-turn 场景下弦长通常很小（车道间距，~3.5m），但需要深入路口
        // 形成完整 U 形。需要更大的绝对偏移量。
        // 参考：典型双车道道路调头，apex 距停止线约 8-12m。
        double offLen;
        if (isUturn) {
            // 调头：偏移 = max(2 × chordLen, 8m)，覆盖典型路口尺寸
            offLen = std::max(chordLen * 2.0, 8.0);
        } else {
            // 一般大曲率两段：使用配置的比例
            double base = std::max(chordLen, 6.0);
            offLen = base * cfg.twoSegMidOffsetRatio;
        }

        // ── 中间点 M 位于弦中点沿 apex 方向偏移 ──
        Point2D mid = (P0 + P3) * 0.5;
        Point2D M   = mid + offDir * offLen;

        // ── 中间点切线 Tmid：沿弦方向；选与 (P3-P0) 同向那个 ──
        Point2D Tmid = chordN;
        // 对于调头，需要 Tmid 与 chord 同方向（自然弧线绕行）
        // 为防止极端情况下 Tmid 与段内方向不匹配，做一次校正：
        // Tmid 应与 (M→P3) - (P0→M) 的"中间方向"近似
        Point2D dirToM   = (M - P0).normalized();
        Point2D dirFromM = (P3 - M).normalized();
        Point2D bisecMid = (dirToM + dirFromM);
        if (bisecMid.norm() > EPS) {
            Tmid = bisecMid.normalized();
        }

        // ── 两段曲线 ──
        double d1 = dist(P0, M);
        double d2 = dist(M,  P3);
        if (d1 < EPS || d2 < EPS) {
            // 极端退化：返回单段近似
            CubicBezier seg1(P0, P0 + T0*0.1, M  - Tmid*0.1, M);
            CubicBezier seg2(M,  M  + Tmid*0.1, P3 + T3*0.1, P3);
            return CompositeBezier(seg1, seg2);
        }

        // 段1: P0 → M
        // 起点切线 = T0；末切线 = Tmid（与段2首切线一致 ⇒ G1）
        // P1 = P0 + T0   * (alpha*d1)
        // P2 = M  - Tmid * (alpha*d1)
        Point2D P1_seg1 = P0 + T0   * (alpha * d1);
        Point2D P2_seg1 = M  - Tmid * (alpha * d1);
        CubicBezier seg1(P0, P1_seg1, P2_seg1, M);

        // 段2: M → P3
        // 起点切线 = Tmid；末切线 = -T3（B'(1)=3*(P3-P2)，P2 = P3 + T3*alpha*d2 ⇒ B'(1) = -3*T3*alpha*d2）
        Point2D P1_seg2 = M  + Tmid * (alpha * d2);
        Point2D P2_seg2 = P3 + T3   * (alpha * d2);
        CubicBezier seg2(M, P1_seg2, P2_seg2, P3);

        return CompositeBezier(seg1, seg2);
    }
};
