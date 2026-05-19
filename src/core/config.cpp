#include "config.h"
#include "../../third_party/nlohmann/json.hpp"
#include <fstream>
#include <iostream>

using json = nlohmann::json;

// 安全读取json字段的辅助模板
template<typename T>
static T jget(const json& j, const std::string& key, const T& def){
    if(j.contains(key)) return j[key].get<T>();
    return def;
}

Config loadConfig(const std::string& path){
    Config cfg;
    if(path.empty()) return cfg;

    std::ifstream f(path);
    if(!f.is_open()){
        std::cerr << "[WARN] Config file not found: " << path << ", using defaults.\n";
        return cfg;
    }

    json j;
    try { f >> j; }
    catch(const std::exception& e){
        std::cerr << "[WARN] Config parse error: " << e.what() << ", using defaults.\n";
        return cfg;
    }

    if(j.contains("coordinate")){
        auto& c = j["coordinate"];
        cfg.coord.outputWGS84 = jget(c,"output_wgs84",false);
    }
    if(j.contains("tangent")){
        auto& c = j["tangent"];
        cfg.tangent.mode      = jget<std::string>(c,"mode","last_segment");
        cfg.tangent.fitPoints = jget(c,"fit_points",2);
    }
    if(j.contains("connection_mode")){
        auto& c = j["connection_mode"];
        cfg.connMode.fanMode             = jget<std::string>(c,"fan_mode","independent");
        cfg.connMode.mergeAtEntryLength  = jget(c,"merge_at_entry_length",0.0);
    }
    if(j.contains("edge_line")){
        auto& c = j["edge_line"];
        cfg.edgeLine.mode                = jget<std::string>(c,"mode","fixed_offset");
        cfg.edgeLine.endpointSmoothPoints= jget(c,"endpoint_smooth_points",4);
        cfg.edgeLine.defaultLaneWidth    = jget(c,"default_lane_width",3.5);
    }
    if(j.contains("bezier")){
        auto& c = j["bezier"];
        cfg.bezier.alphaStraight         = jget(c,"alpha_straight",0.35);
        cfg.bezier.alphaRight            = jget(c,"alpha_right",0.30);
        cfg.bezier.alphaLeft             = jget(c,"alpha_left",0.45);
        cfg.bezier.alphaUturn            = jget(c,"alpha_uturn",0.50);
        cfg.bezier.maxCurvature          = jget(c,"max_curvature",0.333);
        cfg.bezier.twoSegMidOffsetRatio  = jget(c,"two_segment_mid_offset_ratio",0.30);
        if(c.contains("force_single_segment"))
            cfg.bezier.forceSingleSegment = c["force_single_segment"].get<bool>();
        if(c.contains("uturn_composite_dot_threshold"))
            cfg.bezier.uturnCompositeDotThreshold = c["uturn_composite_dot_threshold"].get<double>();
    }
    if(j.contains("sampling")){
        auto& c = j["sampling"];
        cfg.sampling.mode                = jget<std::string>(c,"mode","adaptive");
        cfg.sampling.adaptiveMaxAngleDeg = jget(c,"adaptive_max_angle_deg",0.5);
        cfg.sampling.adaptiveMaxSegLength= jget(c,"adaptive_max_seg_length",2.0);
        cfg.sampling.fixedSpacing        = jget(c,"fixed_spacing",0.5);
        cfg.sampling.fixedCount          = jget(c,"fixed_count",50);
    }
    if(j.contains("obstacle")){
        auto& c = j["obstacle"];
        cfg.obstacle.avoid               = jget(c,"avoid",true);
        cfg.obstacle.safeMargin          = jget(c,"safe_margin",0.5);
        cfg.obstacle.checkSpacing        = jget(c,"check_spacing",0.2);
        cfg.obstacle.phase1MaxIter       = jget(c,"phase1_max_iter",50);
        cfg.obstacle.phase1StepDecay     = jget(c,"phase1_step_decay",0.9);
        cfg.obstacle.phase2MaxIter       = jget(c,"phase2_max_iter",200);
        cfg.obstacle.phase2WCurvature    = jget(c,"phase2_w_curvature",1.0);
        cfg.obstacle.phase2WLength       = jget(c,"phase2_w_length",0.1);
        cfg.obstacle.phase2WObstacle     = jget(c,"phase2_w_obstacle",10.0);
        cfg.obstacle.enablePhase2        = jget(c,"enable_phase2",true);
        cfg.obstacle.enablePhase3        = jget(c,"enable_phase3",true);
        cfg.obstacle.phase3MaxOffsetRatio= jget(c,"phase3_max_offset_ratio",0.75);
        cfg.obstacle.phase3CheckSpacing  = jget(c,"phase3_check_spacing",0.15);
        cfg.obstacle.rightSidePreferThreshold = jget(c,"right_side_prefer_threshold",2.0);
        cfg.obstacle.minGapWidth         = jget(c,"min_gap_width",2.8);
        cfg.obstacle.maxCurvatureJump    = jget(c,"max_curvature_jump",0.15);
        cfg.obstacle.phase3BufTMax       = jget(c,"phase3_buf_t_max",0.35);
        cfg.obstacle.enableGapAnalysis   = jget(c,"enable_gap_analysis",true);
        cfg.obstacle.enableCorridorConstraint = jget(c,"enable_corridor_constraint",true);
    }
    if(j.contains("non_intersect")){
        auto& c = j["non_intersect"];
        cfg.nonIntersect.corridorMinHalfWidth = jget(c,"corridor_min_half_width",0.3);
        cfg.nonIntersect.enableMidUturnExclude= jget(c,"enable_mid_uturn_exclude",true);
        cfg.nonIntersect.maxFixIter           = jget(c,"max_fix_iter",20);
        cfg.nonIntersect.allowUturnIntersect  = jget(c,"allow_uturn_intersect",true);
        cfg.nonIntersect.extremeCrossAngleThreshold = jget(c,"extreme_cross_angle_threshold",150.0);
        cfg.nonIntersect.spreadGradualRatio   = jget(c,"spread_gradual_ratio",0.3);
        cfg.nonIntersect.globalMaxIter        = jget(c,"global_max_iter",80);
    }
    if(j.contains("conflict")){
        auto& c = j["conflict"];
        cfg.conflict.priority             = jget<std::string>(c,"priority","non_intersect");
        cfg.conflict.enableJointOpt       = jget(c,"enable_joint_optimization",false);
        cfg.conflict.jointWNonIntersect   = jget(c,"joint_w_non_intersect",100.0);
        cfg.conflict.jointWObstacle       = jget(c,"joint_w_obstacle",10.0);
    }
    if(j.contains("intersection_polygon")){
        auto& c = j["intersection_polygon"];
        cfg.polygon.snapTolerance = jget(c,"snap_tolerance",0.5);
        cfg.polygon.winding       = jget<std::string>(c,"winding","clockwise");
        cfg.polygon.interEdgeMode = jget<std::string>(c,"inter_edge_mode","direct");
    }
    if(j.contains("output")){
        auto& c = j["output"];
        cfg.output.format     = jget<std::string>(c,"format","shapefile");
        cfg.output.outputDir  = jget<std::string>(c,"output_dir","./output");
        cfg.output.jsonPretty = jget(c,"json_pretty",true);
    }
    if(j.contains("performance")){
        auto& c = j["performance"];
        cfg.performance.gridCellCount = jget(c,"grid_cell_count",400);
        cfg.performance.timeoutMs     = jget(c,"timeout_ms",10000);
    }
    return cfg;
}
