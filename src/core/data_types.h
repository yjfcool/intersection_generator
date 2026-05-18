#pragma once
#include <vector>
#include <string>
#include <map>
#include <optional>
#include <memory>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <cassert>
#include <functional>

// ============================================================
// 基础几何类型
// ============================================================

struct Point2D {
    double x = 0.0, y = 0.0;
    Point2D() = default;
    Point2D(double x_, double y_) : x(x_), y(y_) {}

    Point2D operator+(const Point2D& o) const { return {x+o.x, y+o.y}; }
    Point2D operator-(const Point2D& o) const { return {x-o.x, y-o.y}; }
    Point2D operator*(double s)          const { return {x*s,   y*s};   }
    Point2D operator/(double s)          const { return {x/s,   y/s};   }
    Point2D& operator+=(const Point2D& o){ x+=o.x; y+=o.y; return *this; }
    bool operator==(const Point2D& o) const { return x==o.x && y==o.y; }

    double dot  (const Point2D& o) const { return x*o.x + y*o.y; }
    double cross(const Point2D& o) const { return x*o.y - y*o.x; }
    double norm2() const { return x*x + y*y; }
    double norm()  const { return std::sqrt(norm2()); }

    Point2D normalized() const {
        double n = norm();
        if (n < 1e-12) return {1.0, 0.0};
        return {x/n, y/n};
    }
    // 旋转90°（向左，逆时针）
    Point2D rotLeft()  const { return {-y,  x}; }
    // 旋转90°（向右，顺时针）
    Point2D rotRight() const { return { y, -x}; }
};

inline double dist(const Point2D& a, const Point2D& b){
    return (a-b).norm();
}

using Polyline = std::vector<Point2D>;
using AttrMap  = std::map<std::string, std::string>;

// ============================================================
// 输入要素结构
// ============================================================

enum class GroupType { ENTER, EXIT };

enum class TurnType {
    STRAIGHT, LEFT, RIGHT, U_TURN_LEFT, U_TURN_RIGHT, UNKNOWN
};

inline std::string turnTypeStr(TurnType t){
    switch(t){
        case TurnType::STRAIGHT:    return "straight";
        case TurnType::LEFT:        return "left";
        case TurnType::RIGHT:       return "right";
        case TurnType::U_TURN_LEFT: return "u_turn_left";
        case TurnType::U_TURN_RIGHT:return "u_turn_right";
        default:                    return "unknown";
    }
}

inline TurnType parseTurnType(const std::string& s){
    if(s=="straight")     return TurnType::STRAIGHT;
    if(s=="left")         return TurnType::LEFT;
    if(s=="right")        return TurnType::RIGHT;
    if(s=="u_turn_left")  return TurnType::U_TURN_LEFT;
    if(s=="u_turn_right") return TurnType::U_TURN_RIGHT;
    return TurnType::UNKNOWN;
}

struct LaneCenterline {
    std::string id;
    Polyline    geom;       // 进入线: 远端→连接点; 退出线: 连接点→远端
    GroupType   groupType;  // 所属组类型（预处理填充）
    AttrMap     attrs;
    // 预处理填充
    Point2D     connectionPt;   // 连接点坐标
    Point2D     tangentDir;     // 连接点切线（指向路口内侧）
    int         laneOrder = 0;  // 组内横向排序（0=最内侧）
    std::string groupId;
    // 该车道左右两侧的边线ID（由拓扑分析填充）
    std::string leftEdgelineId;   // 沿行进方向，左侧边线ID
    std::string rightEdgelineId;  // 沿行进方向，右侧边线ID
};

struct LaneEdgeLine {
    std::string id;
    Polyline    geom;
    GroupType   groupType;
    AttrMap     attrs;
    // 预处理填充
    Point2D     connectionPt;
    Point2D     tangentDir;
    std::string groupId;
    int         lineOrder = 0;  // 组内横向排序（0=最内侧）
};

struct LaneGroup {
    std::string              id;
    GroupType                type;
    std::vector<std::string> centerlineIds;  // 由内→外排列
    std::vector<std::string> edgelineIds;    // 边线ID列表
    AttrMap                  attrs;
};

struct Connection {
    std::string id;
    std::string enterGroupId;
    std::string exitGroupId;
    std::string enterLineId;
    std::string exitLineId;
    TurnType    turnType = TurnType::UNKNOWN;
    AttrMap     attrs;
    // 分析后填充
    bool        isMidUturn = false;   // 中间调头
    int         lateralPriority = 0;  // 生成优先级（数值越小越先）
};

struct Obstacle {
    enum class GeomType { POINT, LINE, POLYGON };
    std::string id;
    GeomType    geomType;
    Polyline    geom;
    AttrMap     attrs;
};

struct StopLine {
    std::string id;
    Polyline    geom;
    std::string enterGroupId;
    AttrMap     attrs;
};

struct RoadEdgeLine {
    std::string id;
    Polyline    geom;
    AttrMap     attrs;
    // 预处理填充
    int         intersectionSideIdx = -1; // geom的哪侧端点靠近路口 (0或n-1)
};

struct IntersectionRoughArea {
    std::string id;
    Polyline    geom;
    AttrMap     attrs;
};

struct IntersectionInput {
    std::string scenarioId;
    double      refLon = 0.0, refLat = 0.0;
    bool        inputIsWGS84 = false;

    std::map<std::string, LaneGroup>      laneGroups;
    std::map<std::string, LaneCenterline> centerlines;
    std::map<std::string, LaneEdgeLine>   edgelines;
    std::vector<Connection>               connections;
    std::vector<Obstacle>                 obstacles;
    std::vector<StopLine>                 stopLines;
    std::vector<RoadEdgeLine>             roadEdges;
    std::optional<IntersectionRoughArea>  roughArea;
};

// ============================================================
// 输出要素结构
// ============================================================

enum QualityFlag : int {
    QF_OK                      = 0x000,
    QF_WARN_OBSTACLE_PENETRATED= 0x001,
    QF_WARN_INTERSECTION_REMAIN= 0x002,
    QF_WARN_CORRIDOR_TOO_NARROW= 0x004,
    QF_WARN_CURVATURE_HIGH     = 0x008,
    QF_WARN_EDGE_ALIGN_FAIL    = 0x010,
    QF_ERROR_NO_SOLUTION       = 0x020,
    QF_ERROR_DEGENERATE_INPUT  = 0x040,
    QF_INFO_UTURN_MID_EXCLUDED = 0x080,
    QF_INFO_TWO_SEGMENT_USED   = 0x100,
};

struct GeneratedCenterline {
    std::string id;
    std::string connectionId;
    std::string enterLineId;
    std::string exitLineId;
    Polyline    geom;
    TurnType    turnType = TurnType::UNKNOWN;
    int         qualityFlags = 0;
    std::string warnDesc;
};

struct GeneratedEdgeLine {
    std::string id;
    Polyline    geom;
    std::string leftCenterlineId;
    std::string rightCenterlineId;
    int         qualityFlags = 0;
};

struct IntersectionPolygon {
    std::string id;
    Polyline    geom;  // 闭合多边形
    int         qualityFlags = 0;
    double      area = 0.0;
};

struct IntersectionOutput {
    std::string                      scenarioId;
    std::vector<GeneratedCenterline> centerlines;
    std::vector<GeneratedEdgeLine>   edgelines;
    IntersectionPolygon              intersectionPolygon;
};

// ============================================================
// 走廊结构
// ============================================================
struct Corridor {
    std::string connectionId;
    std::optional<Polyline> leftBoundary;
    std::optional<Polyline> rightBoundary;
    double minHalfWidth = 0.3;
};
