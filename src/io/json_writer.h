#pragma once
#include "../core/data_types.h"
#include "../../third_party/nlohmann/json.hpp"
#include <fstream>
#include <string>
#include <filesystem>

using json = nlohmann::json;

class JSONWriter {
public:
    static void write(const IntersectionOutput& out,
                      const std::string& dir,
                      bool pretty = true)
    {
        std::filesystem::create_directories(dir);
        std::string path = dir + "/" + out.scenarioId + "_output.json";

        json j;
        j["scenario_id"] = out.scenarioId;

        // 质量汇总
        bool hasErrors=false, hasWarns=false;
        int warnCnt=0;
        for(auto& cl : out.centerlines){
            if(cl.qualityFlags & (QF_ERROR_NO_SOLUTION|QF_ERROR_DEGENERATE_INPUT)) hasErrors=true;
            if(cl.qualityFlags & (QF_WARN_OBSTACLE_PENETRATED|QF_WARN_INTERSECTION_REMAIN
                                  |QF_WARN_CURVATURE_HIGH|QF_WARN_EDGE_ALIGN_FAIL)){
                hasWarns=true; warnCnt++;
            }
        }
        j["quality_summary"]["has_errors"]   = hasErrors;
        j["quality_summary"]["has_warnings"] = hasWarns;
        j["quality_summary"]["warning_count"]= warnCnt;

        // 中心线
        json clArr = json::array();
        for(auto& cl : out.centerlines){
            json item;
            item["id"]           = cl.id;
            item["connection_id"]= cl.connectionId;
            item["enter_line_id"]= cl.enterLineId;
            item["exit_line_id"] = cl.exitLineId;
            item["turn_type"]    = turnTypeStr(cl.turnType);
            item["geom"]         = polylineToJson(cl.geom, "LineString");
            item["quality_flags"]= cl.qualityFlags;
            item["warnings"]     = cl.warnDesc;
            clArr.push_back(item);
        }
        j["centerlines"] = clArr;

        // 边线
        json elArr = json::array();
        for(auto& el : out.edgelines){
            json item;
            item["id"]                 = el.id;
            item["left_centerline_id"] = el.leftCenterlineId;
            item["right_centerline_id"]= el.rightCenterlineId;
            item["geom"]               = polylineToJson(el.geom, "LineString");
            item["quality_flags"]      = el.qualityFlags;
            elArr.push_back(item);
        }
        j["edgelines"] = elArr;

        // 路口面
        {
            json poly;
            poly["id"]           = out.intersectionPolygon.id;
            poly["geom"]         = polylineToJson(out.intersectionPolygon.geom, "Polygon");
            poly["quality_flags"]= out.intersectionPolygon.qualityFlags;
            poly["area"]         = out.intersectionPolygon.area;
            j["intersection_polygon"] = poly;
        }

        std::ofstream f(path);
        if(pretty)
            f << j.dump(2) << "\n";
        else
            f << j.dump() << "\n";

        if(f.good())
            std::cout << "[INFO]  JSON written: " << path << "\n";
        else
            std::cerr << "[ERROR] Failed to write JSON: " << path << "\n";
    }

private:
    static json polylineToJson(const Polyline& pts, const std::string& type){
        json geom;
        geom["type"] = type;
        json coords = json::array();

        if(type=="Polygon"){
            json ring = json::array();
            for(auto& p : pts) ring.push_back({p.x, p.y});
            coords.push_back(ring);
        } else {
            for(auto& p : pts) coords.push_back({p.x, p.y});
        }
        geom["coordinates"] = coords;
        return geom;
    }
};
