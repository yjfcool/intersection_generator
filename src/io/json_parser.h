#pragma once
#include "../core/data_types.h"
#include "../../third_party/nlohmann/json.hpp"
#include <fstream>
#include <stdexcept>
#include <string>

using json = nlohmann::json;

class JSONParser {
public:
    static IntersectionInput parse(const std::string& filepath){
        std::ifstream f(filepath);
        if(!f.is_open())
            throw std::runtime_error("Cannot open input file: " + filepath);

        json j;
        try { f >> j; }
        catch(const std::exception& e){
            throw std::runtime_error("JSON parse error: " + std::string(e.what()));
        }

        return parseJson(j);
    }

    static IntersectionInput parseJson(const json& j){
        IntersectionInput inp;

        // 元数据
        inp.scenarioId   = jstr(j,"scenario_id","scenario_001");
        inp.inputIsWGS84 = (jstr(j,"coord_type","projected") == "wgs84");

        if(j.contains("ref_point")){
            auto& rp = j["ref_point"];
            inp.refLon = rp.value("lon", 0.0);
            inp.refLat = rp.value("lat", 0.0);
        }

        // 中心线
        if(j.contains("centerlines")){
            for(auto& item : j["centerlines"]){
                LaneCenterline cl;
                cl.id   = jstr(item,"id","");
                cl.geom = parsePolyline(item["geom"]);
                cl.attrs= parseAttrs(item);
                inp.centerlines[cl.id] = cl;
            }
        }

        // 边线
        if(j.contains("edgelines")){
            for(auto& item : j["edgelines"]){
                LaneEdgeLine el;
                el.id                 = jstr(item,"id","");
                el.geom               = parsePolyline(item["geom"]);
                el.attrs              = parseAttrs(item);
                inp.edgelines[el.id] = el;
            }
        }

        // 车道组
        if(j.contains("lane_groups")){
            for(auto& item : j["lane_groups"]){
                LaneGroup grp;
                grp.id   = jstr(item,"id","");
                grp.type = (jstr(item,"type","enter")=="enter")
                           ? GroupType::ENTER : GroupType::EXIT;
                if(item.contains("centerline_ids")){
                    for(auto& id : item["centerline_ids"])
                        grp.centerlineIds.push_back(id.get<std::string>());
                }
                if(item.contains("edgeline_ids")){
                    for(auto& id : item["edgeline_ids"])
                        grp.edgelineIds.push_back(id.get<std::string>());
                }
                grp.attrs = parseAttrs(item);
                inp.laneGroups[grp.id] = grp;
            }
        }

        // 连通关系
        if(j.contains("connections")){
            for(auto& item : j["connections"]){
                Connection conn;
                conn.id           = jstr(item,"id","");
                conn.enterGroupId = jstr(item,"enter_group_id","");
                conn.exitGroupId  = jstr(item,"exit_group_id","");
                conn.enterLineId  = jstr(item,"enter_line_id","");
                conn.exitLineId   = jstr(item,"exit_line_id","");
                conn.turnType     = parseTurnType(jstr(item,"turn_type","straight"));
                conn.attrs        = parseAttrs(item);
                inp.connections.push_back(conn);
            }
        }

        // 障碍物
        if(j.contains("obstacles")){
            for(auto& item : j["obstacles"]){
                Obstacle obs;
                obs.id    = jstr(item,"id","");
                std::string gt = jstr(item,"geom_type","polygon");
                if(gt=="point")   obs.geomType = Obstacle::GeomType::POINT;
                else if(gt=="line") obs.geomType = Obstacle::GeomType::LINE;
                else              obs.geomType = Obstacle::GeomType::POLYGON;
                obs.geom  = parsePolyline(item["geom"]);
                obs.attrs = parseAttrs(item);
                inp.obstacles.push_back(obs);
            }
        }

        // 停止线
        if(j.contains("stop_lines")){
            for(auto& item : j["stop_lines"]){
                StopLine sl;
                sl.id           = jstr(item,"id","");
                sl.geom         = parsePolyline(item["geom"]);
                sl.enterGroupId = jstr(item,"enter_group_id","");
                sl.attrs        = parseAttrs(item);
                inp.stopLines.push_back(sl);
            }
        }

        // 道路边缘线
        if(j.contains("road_edges")){
            for(auto& item : j["road_edges"]){
                RoadEdgeLine re;
                re.id    = jstr(item,"id","");
                re.geom  = parsePolyline(item["geom"]);
                re.attrs = parseAttrs(item);
                inp.roadEdges.push_back(re);
            }
        }

        // 粗略路口面
        if(j.contains("rough_area")){
            auto& ra = j["rough_area"];
            IntersectionRoughArea roughArea;
            roughArea.id    = jstr(ra,"id","rough_area");
            roughArea.geom  = parsePolyline(ra["geom"]);
            roughArea.attrs = parseAttrs(ra);
            inp.roughArea   = roughArea;
        }

        return inp;
    }

private:
    static std::string jstr(const json& j, const std::string& key,
                             const std::string& def=""){
        if(j.contains(key) && j[key].is_string()) return j[key].get<std::string>();
        return def;
    }

    static Polyline parsePolyline(const json& geom){
        Polyline pts;
        if(!geom.is_object()) return pts;

        std::string type = jstr(geom,"type","LineString");

        if(type=="Point"){
            auto& c = geom["coordinates"];
            if(c.is_array() && c.size()>=2)
                pts.push_back({c[0].get<double>(), c[1].get<double>()});
        }
        else if(type=="LineString"){
            auto& coords = geom["coordinates"];
            for(auto& c : coords){
                if(c.is_array() && c.size()>=2)
                    pts.push_back({c[0].get<double>(), c[1].get<double>()});
            }
        }
        else if(type=="Polygon"){
            auto& rings = geom["coordinates"];
            if(!rings.empty()){
                for(auto& c : rings[0]){
                    if(c.is_array() && c.size()>=2)
                        pts.push_back({c[0].get<double>(), c[1].get<double>()});
                }
            }
        }
        return pts;
    }

    static AttrMap parseAttrs(const json& item){
        AttrMap attrs;
        if(item.contains("attrs") && item["attrs"].is_object()){
            for(auto& [k,v] : item["attrs"].items()){
                attrs[k] = v.is_string() ? v.get<std::string>() : v.dump();
            }
        }
        return attrs;
    }
};
