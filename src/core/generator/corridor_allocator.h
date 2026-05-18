#pragma once
#include "../data_types.h"
#include "../../utils/geom_utils.h"
#include "../../utils/logger.h"
#include <map>
#include <vector>
#include <algorithm>

/**
 * 走廊分配器
 * 基于直行车道骨架，为每条连通关系分配一个走廊（左右边界）
 * 生成顺序: STRAIGHT → RIGHT(内→外) → LEFT(内→外) → U_TURN_R → U_TURN_L → MidUturn
 */
class CorridorAllocator {
    const IntersectionInput& inp_;

public:
    explicit CorridorAllocator(const IntersectionInput& inp) : inp_(inp) {}

    // 为所有连通关系分配初始走廊
    std::map<std::string, Corridor> allocate(double minHalfWidth=0.3){
        std::map<std::string, Corridor> corridors;

        // 按进入组分别处理
        // 对每个进入组，找到该组的所有连通关系，建立横向顺序
        for(auto& [gid, grp] : inp_.laneGroups){
            if(grp.type != GroupType::ENTER) continue;

            // 获取该进入组的所有连通关系，按 laneOrder 排列
            std::vector<const Connection*> conns;
            for(auto& conn : inp_.connections){
                if(conn.enterGroupId == gid) conns.push_back(&conn);
            }
            if(conns.empty()) continue;

            // 按车道序（laneOrder）和转向类型排列
            // 这里简化：直接使用 lateralPriority 排序
            std::sort(conns.begin(), conns.end(),
                [](const Connection* a, const Connection* b){
                    return a->lateralPriority < b->lateralPriority;
                });

            // 为每条连通关系创建初始走廊（边界初始为空，后续动态更新）
            for(auto* conn : conns){
                Corridor c;
                c.connectionId   = conn->id;
                c.minHalfWidth   = minHalfWidth;
                corridors[conn->id] = c;
            }
        }

        return corridors;
    }

    /**
     * 在生成完一条曲线后，更新相邻走廊的边界
     * @param conn 刚生成完的连通关系
     * @param generatedPoly 生成的折线（已采样）
     * @param corridors 走廊映射（原地修改）
     */
    void updateCorridorBounds(
        const Connection& conn,
        const Polyline& generatedPoly,
        std::map<std::string, Corridor>& corridors)
    {
        // 找到同一进入组内、在 conn 左侧和右侧的邻居连通关系
        // "左/右"由进入组的车道排序决定
        auto& groupConns = getGroupConnections(conn.enterGroupId);

        // 找当前 conn 在排序中的位置
        int myIdx = -1;
        for(int i=0;i<(int)groupConns.size();++i){
            if(groupConns[i]->id == conn.id){ myIdx=i; break; }
        }
        if(myIdx < 0) return;

        // 右侧邻居（laneOrder更大方向）的左边界 = 本线生成的折线
        if(myIdx+1 < (int)groupConns.size()){
            const std::string& rightId = groupConns[myIdx+1]->id;
            if(corridors.count(rightId)){
                corridors[rightId].leftBoundary = generatedPoly;
            }
        }
        // 左侧邻居的右边界
        if(myIdx-1 >= 0){
            const std::string& leftId = groupConns[myIdx-1]->id;
            if(corridors.count(leftId)){
                corridors[leftId].rightBoundary = generatedPoly;
            }
        }
    }

    // 返回生成顺序排好的所有连通关系
    std::vector<Connection> sortedConnections() const {
        auto conns = inp_.connections;
        // 先按进入组的地理位置排（不影响逻辑），再按 lateralPriority 全局排序
        std::stable_sort(conns.begin(), conns.end(),
            [](const Connection& a, const Connection& b){
                // 先按进入组ID（保证同组在一起），再按优先级
                if(a.enterGroupId != b.enterGroupId)
                    return a.enterGroupId < b.enterGroupId;
                return a.lateralPriority < b.lateralPriority;
            });
        return conns;
    }

private:
    // 缓存：进入组 → 排序后的连通关系列表
    mutable std::map<std::string, std::vector<const Connection*>> groupConnCache_;

    const std::vector<const Connection*>& getGroupConnections(const std::string& gid) const {
        auto it = groupConnCache_.find(gid);
        if(it != groupConnCache_.end()) return it->second;

        auto& vec = groupConnCache_[gid];
        for(auto& conn : inp_.connections){
            if(conn.enterGroupId == gid) vec.push_back(&conn);
        }
        std::sort(vec.begin(), vec.end(),
            [](const Connection* a, const Connection* b){
                return a->lateralPriority < b->lateralPriority;
            });
        return vec;
    }
};
