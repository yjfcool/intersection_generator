#include <iostream>
#include <string>
#include <filesystem>
#include <chrono>
#include <stdexcept>

#include "core/data_types.h"
#include "core/config.h"
#include "core/preprocess/coord_transformer.h"
#include "core/preprocess/topology_analyzer.h"
#include "core/preprocess/tangent_estimator.h"
#include "core/preprocess/obstacle_index.h"
#include "core/generator/curve_generator.h"
#include "core/postprocess/edge_line_generator.h"
#include "core/postprocess/polygon_builder.h"
#include "core/special/roundabout_handler.h"
#include "io/json_parser.h"
#include "io/json_writer.h"
#include "io/shp_writer.h"
#include "utils/logger.h"

// 外部声明（config.cpp 中定义）
Config loadConfig(const std::string& path);

// ============================================================
// 命令行参数解析
// ============================================================
struct Args {
    std::string inputFile;
    std::string configFile = "./config.json";
    std::string outputDir;
    std::string format;
    double      safeMargin = -1;
    std::string sampling;
    double      spacing = -1;
    bool        noPhase2  = false;
    bool        verbose   = false;
    bool        help      = false;
    bool        noAvoid   = false; //默认避障
};

static void printHelp(const char* prog){
    std::cout <<
        "用法: " << prog << " [选项] <输入JSON文件>\n\n"
        "必需参数:\n"
        "  <输入JSON文件>           输入数据文件路径\n\n"
        "可选参数:\n"
        "  -c, --config <文件>      参数配置文件路径（默认：./config.json）\n"
        "  -o, --output-dir <目录>  输出目录（覆盖配置文件）\n"
        "  -f, --format <格式>      输出格式: shapefile|json|both\n"
        "  --no-avoid              不需要避障\n"
        "  --safe-margin <米>       障碍物安全距离\n"
        "  --sampling <模式>        采样模式: adaptive|fixed_spacing|fixed_count\n"
        "  --spacing <米>           固定采样间距\n"
        "  --no-phase2              禁用Phase2数值优化\n"
        "  --verbose                输出详细日志\n"
        "  -h, --help               显示帮助\n\n";
}

static Args parseArgs(int argc, char* argv[]){
    Args args;
    for(int i=1;i<argc;++i){
        std::string a = argv[i];
        if(a=="-h"||a=="--help"){ args.help=true; }
        else if(a=="--verbose"){  args.verbose=true; }
        else if(a=="--no-phase2"){ args.noPhase2=true; }
        else if((a=="-c"||a=="--config") && i+1<argc){  args.configFile=argv[++i]; }
        else if((a=="-o"||a=="--output-dir") && i+1<argc){ args.outputDir=argv[++i]; }
        else if((a=="-f"||a=="--format") && i+1<argc){  args.format=argv[++i]; }
        else if(a=="--no-avoid" && i+1<argc){  args.noAvoid=true; }
        else if(a=="--safe-margin" && i+1<argc){  args.safeMargin=std::stod(argv[++i]); }
        else if(a=="--sampling" && i+1<argc){     args.sampling=argv[++i]; }
        else if(a=="--spacing" && i+1<argc){      args.spacing=std::stod(argv[++i]); }
        else if(a[0]!='-'){  args.inputFile=a; }
        else { std::cerr<<"[WARN] Unknown arg: "<<a<<"\n"; }
    }
    return args;
}

// ============================================================
// 主处理函数
// ============================================================
IntersectionOutput processIntersection(
    const IntersectionInput& rawInp,
    const Config& cfg)
{
    auto t0 = std::chrono::steady_clock::now();
    Timer timer(cfg.performance.timeoutMs);

    // --- Stage 0: 坐标变换 ---
    IntersectionInput inp = rawInp; // 拷贝，后续修改
    CoordTransformer transformer(inp.refLon, inp.refLat);

    if(inp.inputIsWGS84){
        Logger::info("Converting WGS84 → local plane (AEQD)...");
        transformer.transformInput(inp);
    }

    // --- Stage 1: 预处理 ---
    Logger::info("Stage 1: Preprocessing...");

    // 拓扑分析（标记组类型、laneOrder、中间调头、优先级）
    TopologyAnalyzer::analyze(inp, cfg);

    // 切线估算
    TangentEstimator::estimateAll(inp, cfg.tangent);

    // 障碍物空间索引
    ObstacleSpatialIndex obsIdx;
    if (cfg.obstacle.avoid) {
        obsIdx.build(inp.obstacles, 5.0, cfg.performance.gridCellCount);
        Logger::info("  Obstacles indexed: " + std::to_string(inp.obstacles.size()));
    }

    // 环岛检测与预处理
    if(RoundaboutHandler::isRoundabout(inp)){
        Logger::info("  Roundabout detected, preprocessing...");
        RoundaboutHandler::preprocessRoundabout(inp, cfg);
    }

    if(timer.exceeded()) throw std::runtime_error("Timeout in Stage 1");

    // --- Stage 2: 核心曲线生成 ---
    Logger::info("Stage 2: Generating intersection centerlines...");
    CurveGenerator curveGen(cfg, obsIdx);
    auto centerlines = curveGen.generate(inp);
    Logger::info("  Generated " + std::to_string(centerlines.size()) + " centerlines.");

    if(timer.exceeded()) throw std::runtime_error("Timeout in Stage 2");

    // --- Stage 3: 边线与路口面 ---
    Logger::info("Stage 3: Generating edge lines and polygon...");

    EdgeLineGenerator edgeGen(cfg);
    auto edgelines = edgeGen.generate(centerlines, inp);
    Logger::info("  Generated " + std::to_string(edgelines.size()) + " edge lines.");

    IntersectionPolygonBuilder polyBuilder(cfg.polygon);
    auto poly = polyBuilder.build(inp, centerlines, edgelines);

    // --- 坐标逆变换（若输出WGS84）---
    if(cfg.coord.outputWGS84 && inp.inputIsWGS84){
        Logger::info("Converting local plane → WGS84...");
        auto transOut = [&](Polyline& pl){
            for(auto& p : pl){
                auto [lon,lat] = transformer.toWGS84(p);
                p = {lon, lat};
            }
        };
        for(auto& cl : centerlines) transOut(cl.geom);
        for(auto& el : edgelines)   transOut(el.geom);
        transOut(poly.geom);
    }

    // --- 封装输出 ---
    IntersectionOutput out;
    out.scenarioId          = inp.scenarioId;
    out.centerlines         = centerlines;
    out.edgelines           = edgelines;
    out.intersectionPolygon = poly;

    auto t1 = std::chrono::steady_clock::now();
    long ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count();
    Logger::info("Processing complete in " + std::to_string(ms) + " ms");

    return out;
}

// ============================================================
// main
// ============================================================
int main(int argc, char* argv[]){
    Args args = parseArgs(argc, argv);

    if(args.help || args.inputFile.empty()){
        printHelp(argv[0]);
        return args.help ? 0 : 1;
    }

    Logger::verbose = args.verbose;

    // 加载配置
    Config cfg = loadConfig(args.configFile);

    // 命令行覆盖配置
    if(!args.outputDir.empty())  cfg.output.outputDir     = args.outputDir;
    if(!args.format.empty())     cfg.output.format        = args.format;
    if(args.noAvoid)             cfg.obstacle.avoid       = !args.noAvoid;
    if(args.safeMargin > 0)      cfg.obstacle.safeMargin  = args.safeMargin;
    if(!args.sampling.empty())   cfg.sampling.mode        = args.sampling;
    if(args.spacing > 0)         cfg.sampling.fixedSpacing= args.spacing;
    if(args.noPhase2)            cfg.obstacle.enablePhase2= false;

    cfg.output.outputDir = cfg.output.outputDir+(cfg.obstacle.avoid ? "_avoid":"_noavoid");
    std::cout << "[INFO]  Input:  " << args.inputFile  << "\n";
    std::cout << "[INFO]  Output: " << cfg.output.outputDir << "\n";
    std::cout << "[INFO]  Format: " << cfg.output.format    << "\n";

    // 解析输入
    IntersectionInput inp;
    try {
        inp = JSONParser::parse(args.inputFile);
    } catch(const std::exception& e){
        std::cerr << "[ERROR] " << e.what() << "\n";
        return 2;
    }
    if (cfg.output.format == "shapefile" || cfg.output.format == "both") {
        ShapefileWriter::writeIntersectionInput(cfg.output.outputDir, inp, "input"); // 将输入数据输出为Shapefile
    }

    // 主处理
    IntersectionOutput out;
    try {
        out = processIntersection(inp, cfg);
    } catch(const std::exception& e){
        std::cerr << "[ERROR] Processing failed: " << e.what() << "\n";
        return 3;
    }

    // 输出
    std::filesystem::create_directories(cfg.output.outputDir);
    std::string& fmt = cfg.output.format;

    try {
        if(fmt=="shapefile"||fmt=="both"){
            ShapefileWriter::writeIntersectionOutput(cfg.output.outputDir, out, "output");
        }
        if(fmt=="json"||fmt=="both"){
            JSONWriter::write(out, cfg.output.outputDir, cfg.output.jsonPretty);
        }
    } catch(const std::exception& e){
        std::cerr << "[ERROR] Output failed: " << e.what() << "\n";
        return 4;
    }

    // 统计质量
    int warns=0, errors=0;
    for(auto& cl:out.centerlines){
        if(cl.qualityFlags & (QF_ERROR_NO_SOLUTION|QF_ERROR_DEGENERATE_INPUT)) ++errors;
        else if(cl.qualityFlags) ++warns;
    }
    std::cout << "[INFO]  Quality: errors=" << errors << " warnings=" << warns
              << " / " << out.centerlines.size() << " centerlines\n";

    return errors > 0 ? 5 : 0;
}
