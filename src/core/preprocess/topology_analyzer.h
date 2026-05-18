#pragma once
#include "../data_types.h"
#include "../../utils/geom_utils.h"
#include "../config.h"
#include "../../utils/logger.h"
#include <unordered_map>
#include <set>

/**
 * 拓扑分析模块
 * - 预处理组内车道排序
 * - 标记中间调头车道
 * - 分析1:N / N:1关系
 * - 计算生成优先级
 */
class TopologyAnalyzer {
public:
    static void analyze(IntersectionInput& inp, const Config& cfg){
        // 1. 标记每条中心线和边线的组类型、组ID、组内排序
        tagGroupInfo(inp);
        // 2. 标记中间调头车道
        tagMidUturn(inp, cfg);
        // 3. 计算生成优先级
        assignPriority(inp);
    }

private:
    // 根据LaneGroup，为每条中心线/边线填充groupType、groupId、laneOrder
    static void tagGroupInfo(IntersectionInput& inp){
        for(auto& [gid, grp] : inp.laneGroups){
            // 中心线：组内排序按 centerlineIds 顺序
            for(int i=0;i<(int)grp.centerlineIds.size();++i){
                auto it = inp.centerlines.find(grp.centerlineIds[i]);
                if(it==inp.centerlines.end()) continue;
                it->second.groupType  = grp.type;
                it->second.groupId    = gid;
                it->second.laneOrder  = i; // 0=最内侧
            }
            // 边线：组内排序按 edgelineIds 顺序（0=最内侧）
            for(int i=0;i<(int)grp.edgelineIds.size();++i){
                auto it = inp.edgelines.find(grp.edgelineIds[i]);
                if(it==inp.edgelines.end()) continue;
                it->second.groupType = grp.type;
                it->second.groupId   = gid;
                it->second.lineOrder = i; // 0=最内侧
            }
            // 中心线左右边线：相邻边线 id（沿行进方向：内侧=左侧，外侧=右侧）
            // 按车道组的常规约定：edgelineIds[i] 是 centerlineIds[i] 左侧边线，
            // edgelineIds[i+1] 是右侧边线（边线数应等于车道数+1）
            int nCl = (int)grp.centerlineIds.size();
            int nEl = (int)grp.edgelineIds.size();
            for(int i=0;i<nCl;++i){
                auto clit = inp.centerlines.find(grp.centerlineIds[i]);
                if(clit==inp.centerlines.end()) continue;
                if(nEl >= nCl + 1){
                    clit->second.leftEdgelineId  = grp.edgelineIds[i];
                    clit->second.rightEdgelineId = grp.edgelineIds[i+1];
                } else if(nEl == nCl){
                    // 退化：边线数与车道数一致，仅赋一侧
                    clit->second.leftEdgelineId  = grp.edgelineIds[i];
                    clit->second.rightEdgelineId = (i+1<nEl) ? grp.edgelineIds[i+1] : "";
                } else if(nEl > 0){
                    clit->second.leftEdgelineId  = (i<nEl) ? grp.edgelineIds[i] : "";
                    clit->second.rightEdgelineId = (i+1<nEl) ? grp.edgelineIds[i+1] : "";
                }
            }
        }
    }

    // 标记中间调头：同一进入组中，非最内/最外侧的调头车道
    static void tagMidUturn(IntersectionInput& inp, const Config& cfg){
        if(!cfg.nonIntersect.enableMidUturnExclude) return;

        // 按进入组分组统计调头
        // 对每个进入组：找出该组所有连通关系
        std::unordered_map<std::string, std::vector<int>> groupConnMap;
        for(int i=0;i<(int)inp.connections.size();++i){
            groupConnMap[inp.connections[i].enterGroupId].push_back(i);
        }

        for(auto& [gid, idxList] : groupConnMap){
            auto git = inp.laneGroups.find(gid);
            if(git==inp.laneGroups.end()) continue;
            const LaneGroup& grp = git->second;
            if(grp.type != GroupType::ENTER) continue;

            int total = (int)grp.centerlineIds.size();

            for(int ci : idxList){
                Connection& conn = inp.connections[ci];
                bool isUturn = (conn.turnType==TurnType::U_TURN_LEFT ||
                                conn.turnType==TurnType::U_TURN_RIGHT);
                if(!isUturn) continue;

                // 找该进入线在组内的排序
                auto clit = inp.centerlines.find(conn.enterLineId);
                if(clit==inp.centerlines.end()) continue;
                int order = clit->second.laneOrder;

                // 中间调头：不是第0条也不是最后一条
                if(order > 0 && order < total-1){
                    conn.isMidUturn = true;
                    Logger::info("MidUturn detected: conn=" + conn.id + " order=" + std::to_string(order));
                }
            }
        }
    }

    // 计算生成优先级：直行=0, 右转内→外=1..N, 左转内→外, 调头
    static void assignPriority(IntersectionInput& inp){
        // 按进入组分组
        std::unordered_map<std::string, std::vector<int>> groupConnMap;
        for(int i=0;i<(int)inp.connections.size();++i){
            groupConnMap[inp.connections[i].enterGroupId].push_back(i);
        }

        for(auto& [gid, idxList] : groupConnMap){
            auto git = inp.laneGroups.find(gid);
            if(git==inp.laneGroups.end()) continue;

            // 对同一组内的连通关系按 转向类型+车道序 排序优先级
            // 直行(0) < 右转按order升序(1..) < 左转按order升序 < 调头
            auto getPriBase = [](TurnType t, bool isMid) -> int {
                if(isMid) return 1000; // 中间调头放最后
                switch(t){
                    case TurnType::STRAIGHT:     return 0;
                    case TurnType::RIGHT:        return 100;
                    case TurnType::U_TURN_RIGHT: return 300;
                    case TurnType::LEFT:         return 200;
                    case TurnType::U_TURN_LEFT:  return 400;
                    default:                     return 500;
                }
            };

            for(int ci : idxList){
                Connection& conn = inp.connections[ci];
                auto clit = inp.centerlines.find(conn.enterLineId);
                int order = (clit!=inp.centerlines.end()) ? clit->second.laneOrder : 0;
                conn.lateralPriority = getPriBase(conn.turnType, conn.isMidUturn) + order;
            }
        }
    }
};
