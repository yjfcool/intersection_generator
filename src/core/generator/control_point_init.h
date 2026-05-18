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
            // 检查初始曲率
            if(res.single.maxCurvature(30) > cfg.maxCurvature) needTwo = true;
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
        double alpha = cfg.alphaUturn;

        // 中间点 M：在 P0→P3 连线的中点附近，偏向曲线外侧
        Point2D mid = (P0+P3)*0.5;

        // 偏移方向：T0旋转90°（调头外侧）
        Point2D offDir;
        if(turn == TurnType::U_TURN_LEFT){
            offDir = T0.rotLeft(); // 向左偏（左掉头外侧在左）
        } else {
            offDir = T0.rotRight();
        }
        double offLen = d * cfg.twoSegMidOffsetRatio;
        Point2D M = mid + offDir * offLen;

        // 中间点处的切线：从P0→P3方向（近似）
        Point2D Tmid = (P3-P0).normalized();
        if(turn == TurnType::U_TURN_LEFT || turn == TurnType::U_TURN_RIGHT){
            // 调头时中间切线应垂直于P0P3
            Tmid = T0.rotLeft().normalized();
        }

        // 段1: P0, P1=P0+T0*alpha*d/2, P2=M-Tmid*alpha*d/2, M
        double dSeg1 = dist(P0,M);
        Point2D P1 = P0 + T0  * (alpha * dSeg1);
        Point2D P2 = M  - Tmid* (alpha * dSeg1);

        // 段2: M, P3=M+Tmid*alpha*dSeg2, P4=P3+T3*alpha*dSeg2, P3（终点）
        double dSeg2 = dist(M,P3);
        Point2D P3s = M + Tmid * (alpha * dSeg2);
        // P4 = P3(终点) + T3 * alpha * dSeg2
        // 注意：T3指向路口内，所以P4 = 终点P3 + T3 * alpha * dSeg2
        Point2D P4 = P3 + T3 * (alpha * dSeg2);

        CubicBezier seg1(P0, P1, P2, M);
        CubicBezier seg2(M, P3s, P4, P3);

        return CompositeBezier(seg1, seg2);
    }
};
