#pragma once
#include <string>

struct CoordConfig {
    bool outputWGS84 = false;
};

struct TangentConfig {
    std::string mode = "last_segment"; // "last_segment" | "weighted_avg"
    int fitPoints = 2;
};

struct ConnectionModeConfig {
    std::string fanMode = "independent"; // "independent" | "merge_then_split"
    double mergeAtEntryLength = 0.0;
};

struct EdgeLineConfig {
    std::string mode = "fixed_offset";  // "fixed_offset" | "centerline_interp"
    int endpointSmoothPoints = 4;
    double defaultLaneWidth = 3.5;      // 无法从外部推断时的默认车道宽（米）
};

struct BezierConfig {
    double alphaStraight = 0.35;
    double alphaRight    = 0.30;
    double alphaLeft     = 0.45;
    double alphaUturn    = 0.50;
    double maxCurvature  = 0.333;       // 触发两段的曲率阈值(1/m)
    double twoSegMidOffsetRatio = 0.30;
};

struct SamplingConfig {
    std::string mode = "adaptive";      // "adaptive" | "fixed_spacing" | "fixed_count"
    double adaptiveMaxAngleDeg  = 0.5;
    double adaptiveMaxSegLength = 2.0;
    double adaptiveMinSegLength = 0.05;
    double fixedSpacing         = 0.5;
    int    fixedCount           = 50;
};

struct ObstacleConfig {
    bool avoid = true; //是否避障，false：不考虑避障
    double safeMargin    = 0.5;
    double checkSpacing  = 0.2;
    int    phase1MaxIter = 50;
    double phase1StepDecay = 0.9;
    int    phase2MaxIter = 200;
    double phase2WCurvature = 1.0;
    double phase2WLength    = 0.1;
    double phase2WObstacle  = 10.0;
    bool   enablePhase2 = true;
    // Phase 3：局部绕障（当Phase1/2无法整体避开时）
    bool   enablePhase3         = true;
    double phase3MaxOffsetRatio = 4.0;   // 允许最大偏移/段长比（较大可处理大障碍）
    double phase3CheckSpacing   = 0.15;
    // 右侧通行优先：当左右绕障路径长度差 <= 此阈值时，优先选右侧绕障
    double rightSidePreferThreshold = 2.0; // 米
};

struct NonIntersectConfig {
    double corridorMinHalfWidth  = 0.3;
    bool   enableMidUturnExclude = true;
    int    maxFixIter = 60;
};

struct ConflictConfig {
    std::string priority = "non_intersect"; // "non_intersect" | "obstacle" | "both_error"
    bool   enableJointOpt      = false;
    double jointWNonIntersect  = 100.0;
    double jointWObstacle      = 10.0;
};

struct PolygonConfig {
    double snapTolerance  = 0.5;
    std::string winding   = "clockwise";
    std::string interEdgeMode = "direct"; // "direct" | "arc"
};

struct OutputConfig {
    std::string format    = "shapefile"; // "shapefile" | "json" | "both"
    std::string outputDir = "./output";
    std::string shpEncoding = "UTF-8";
    bool jsonPretty = true;
};

struct PerformanceConfig {
    int gridCellCount = 400;
    int timeoutMs     = 10000;
};

struct Config {
    CoordConfig       coord;
    TangentConfig     tangent;
    ConnectionModeConfig connMode;
    EdgeLineConfig    edgeLine;
    BezierConfig      bezier;
    SamplingConfig    sampling;
    ObstacleConfig    obstacle;
    NonIntersectConfig nonIntersect;
    ConflictConfig    conflict;
    PolygonConfig     polygon;
    OutputConfig      output;
    PerformanceConfig performance;
    bool              verbose = false;
};
