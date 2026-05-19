#pragma once
#include "../data_types.h"
#include "../config.h"
#include "../bezier/cubic_bezier.h"
#include "../../utils/geom_utils.h"
#include "../../utils/logger.h"
#include <map>
#include <set>
#include <optional>

/**
 * Edge line generator (optimized shared-edge version)
 *
 * Strategy:
 *  1. Group generated centerlines by (enterGroupId, exitGroupId) pairs
 *  2. Sort within each group by enter lane order (inner to outer)
 *  3. Detect "special" connections that cross other lanes in the same group
 *  4. For adjacent non-special lanes that share edges on BOTH enter and exit sides,
 *     generate a single shared midline (cubic Bezier) instead of two independent edges
 *  5. Outermost right, innermost left, and special lane edges use independent generation
 *
 * Shared midline properties:
 *  - G1 smooth at both endpoints
 *  - Endpoints exactly coincide with shared edge connection points
 *  - Uses CubicBezier::fromAlphaBeta with alpha=0.38, beta=0.38
 */
class EdgeLineGenerator {
    const Config& cfg_;

public:
    explicit EdgeLineGenerator(const Config& cfg) : cfg_(cfg) {}

    std::vector<GeneratedEdgeLine> generate(
        std::vector<GeneratedCenterline>& centerlines,
        const IntersectionInput& inp)
    {
        std::vector<GeneratedEdgeLine> result;
        std::map<std::string, std::pair<std::string,std::string>> edgeIdMap;

        // Build connectionId -> GeneratedCenterline mapping
        std::map<std::string, const GeneratedCenterline*> clMap;
        for(auto& gcl : centerlines) clMap[gcl.connectionId] = &gcl;

        // Build connection id -> Connection mapping
        std::map<std::string, const Connection*> connMap;
        for(auto& conn : inp.connections) connMap[conn.id] = &conn;

        // Group generated centerlines by (enterGroupId, exitGroupId)
        using GroupKey = std::pair<std::string, std::string>;
        std::map<GroupKey, std::vector<const GeneratedCenterline*>> pairGroups;

        for(auto& gcl : centerlines) {
            auto cIt = connMap.find(gcl.connectionId);
            if(cIt == connMap.end()) continue;
            const Connection* conn = cIt->second;
            GroupKey key = {conn->enterGroupId, conn->exitGroupId};
            pairGroups[key].push_back(&gcl);
        }

        // Process each (enterGroupId, exitGroupId) group
        for(auto& [key, groupCls] : pairGroups) {
            // Sort by enter lane order (inner to outer)
            std::vector<const GeneratedCenterline*> sorted = groupCls;
            std::sort(sorted.begin(), sorted.end(),
                [&](const GeneratedCenterline* a, const GeneratedCenterline* b) {
                    auto aIt = inp.centerlines.find(a->enterLineId);
                    auto bIt = inp.centerlines.find(b->enterLineId);
                    int aOrder = (aIt != inp.centerlines.end()) ? aIt->second.laneOrder : 0;
                    int bOrder = (bIt != inp.centerlines.end()) ? bIt->second.laneOrder : 0;
                    return aOrder < bOrder;
                });

            // Detect special connections (those that cross other lanes in the group)
            std::set<std::string> specialIds;
            if(sorted.size() > 1) {
                for(size_t i = 0; i < sorted.size(); ++i) {
                    for(size_t j = i+1; j < sorted.size(); ++j) {
                        if(sorted[i]->geom.size() < 2 || sorted[j]->geom.size() < 2) continue;
                        if(polylinesIntersectExcludeEndpoints(sorted[i]->geom, sorted[j]->geom)) {
                            specialIds.insert(sorted[i]->id);
                            specialIds.insert(sorted[j]->id);
                        }
                    }
                }
            }

            // Determine which adjacent pairs share edges on both sides
            // sharedBetween[i] = true means lanes i and i+1 share an edge
            std::vector<bool> sharedBetween(sorted.size() > 0 ? sorted.size()-1 : 0, false);
            // Store shared edge IDs for each pair
            struct SharedEdgeInfo {
                std::string enterEdgeId;
                std::string exitEdgeId;
            };
            std::vector<SharedEdgeInfo> sharedEdgeInfos(sharedBetween.size());

            for(size_t i = 0; i+1 < sorted.size(); ++i) {
                if(specialIds.count(sorted[i]->id) || specialIds.count(sorted[i+1]->id))
                    continue;

                const std::string& enterLineI = sorted[i]->enterLineId;
                const std::string& enterLineI1 = sorted[i+1]->enterLineId;
                const std::string& exitLineI = sorted[i]->exitLineId;
                const std::string& exitLineI1 = sorted[i+1]->exitLineId;

                // Enter side: lane_i's right edge == lane_{i+1}'s left edge
                // Compare by geometry (connectionPt) since different IDs may represent the same physical edge
                auto enterI = inp.centerlines.find(enterLineI);
                auto enterI1 = inp.centerlines.find(enterLineI1);
                bool enterShared = false;
                std::string sharedEnterEdgeId;
                if(enterI != inp.centerlines.end() && enterI1 != inp.centerlines.end()) {
                    const std::string& rightOfI = enterI->second.rightEdgelineId;
                    const std::string& leftOfI1 = enterI1->second.leftEdgelineId;
                    if(!rightOfI.empty() && !leftOfI1.empty()) {
                        // Check if IDs match directly (nCL+1 convention)
                        if(rightOfI == leftOfI1) {
                            enterShared = true;
                            sharedEnterEdgeId = rightOfI;
                        } else {
                            // Check if connection points match (2*nCL convention with identical geometry)
                            auto rightEdgeIt = inp.edgelines.find(rightOfI);
                            auto leftEdgeIt = inp.edgelines.find(leftOfI1);
                            if(rightEdgeIt != inp.edgelines.end() && leftEdgeIt != inp.edgelines.end()) {
                                double ptDist = dist(rightEdgeIt->second.connectionPt, leftEdgeIt->second.connectionPt);
                                if(ptDist < 0.01) {
                                    enterShared = true;
                                    sharedEnterEdgeId = rightOfI;
                                }
                            }
                        }
                    }
                }

                // Exit side: lane_i's right edge == lane_{i+1}'s left edge
                // (same pattern as enter side - shared edge is between adjacent lanes)
                // Compare by geometry (connectionPt) since different IDs may represent the same physical edge
                auto exitI = inp.centerlines.find(exitLineI);
                auto exitI1 = inp.centerlines.find(exitLineI1);
                bool exitShared = false;
                std::string sharedExitEdgeId;
                if(exitI != inp.centerlines.end() && exitI1 != inp.centerlines.end()) {
                    const std::string& rightOfI = exitI->second.rightEdgelineId;
                    const std::string& leftOfI1 = exitI1->second.leftEdgelineId;
                    if(!rightOfI.empty() && !leftOfI1.empty()) {
                        // Check if IDs match directly (nCL+1 convention)
                        if(rightOfI == leftOfI1) {
                            exitShared = true;
                            sharedExitEdgeId = rightOfI;
                        } else {
                            // Check if connection points match (2*nCL convention with identical geometry)
                            auto rightEdgeIt = inp.edgelines.find(rightOfI);
                            auto leftEdgeIt = inp.edgelines.find(leftOfI1);
                            if(rightEdgeIt != inp.edgelines.end() && leftEdgeIt != inp.edgelines.end()) {
                                double ptDist = dist(rightEdgeIt->second.connectionPt, leftEdgeIt->second.connectionPt);
                                if(ptDist < 0.01) {
                                    exitShared = true;
                                    sharedExitEdgeId = rightOfI;
                                }
                            }
                        }
                    }
                }

                if(enterShared && exitShared) {
                    sharedBetween[i] = true;
                    sharedEdgeInfos[i] = {sharedEnterEdgeId, sharedExitEdgeId};
                }
            }

            // Generate edges for each lane in the group
            for(size_t i = 0; i < sorted.size(); ++i) {
                const GeneratedCenterline& gcl = *sorted[i];
                if(gcl.geom.size() < 2) continue;

                auto connIt = connMap.find(gcl.connectionId);
                if(connIt == connMap.end()) continue;
                const Connection* conn = connIt->second;

                bool isSpecial = specialIds.count(gcl.id) > 0;

                // Determine if left edge is shared (from lane i-1's right)
                bool leftShared = (!isSpecial && i > 0 && sharedBetween[i-1]);
                // Determine if right edge is shared (with lane i+1's left)
                bool rightShared = (!isSpecial && i+1 < sorted.size() && sharedBetween[i]);

                // Generate left edge
                std::string leftElId;
                if(leftShared) {
                    // Left edge was already generated as the shared midline for pair (i-1, i)
                    leftElId = "gen_el_shared_" + sorted[i-1]->id + "_" + gcl.id;
                } else {
                    // Generate independent left edge
                    leftElId = "gen_el_left_" + conn->id;
                    Polyline leftPts = generateIndependentEdge(gcl, conn, true, inp);
                    GeneratedEdgeLine el;
                    el.id = leftElId;
                    el.geom = leftPts;
                    el.centerlineId = gcl.id;
                    el.side = "left";
                    el.qualityFlags = 0;
                    result.push_back(el);
                }

                // Generate right edge
                std::string rightElId;
                if(rightShared) {
                    // Generate a shared midline between lane i and lane i+1
                    rightElId = "gen_el_shared_" + gcl.id + "_" + sorted[i+1]->id;
                    Polyline sharedPts = generateSharedMidline(
                        gcl, *sorted[i+1],
                        sharedEdgeInfos[i].enterEdgeId,
                        sharedEdgeInfos[i].exitEdgeId,
                        inp);
                    GeneratedEdgeLine el;
                    el.id = rightElId;
                    el.geom = sharedPts;
                    // centerlineId records the inner lane (lane I). The outer lane (lane I+1)
                    // references this shared edge via its backfilled leftEdgelineId string,
                    // so consumers using leftEdgelineId/rightEdgelineId will find it correctly.
                    el.centerlineId = gcl.id;
                    el.side = "right";
                    el.qualityFlags = checkSharedMidlineCurvature(
                        gcl, *sorted[i+1],
                        sharedEdgeInfos[i].enterEdgeId,
                        sharedEdgeInfos[i].exitEdgeId,
                        inp);
                    result.push_back(el);
                } else {
                    // Generate independent right edge
                    rightElId = "gen_el_right_" + conn->id;
                    Polyline rightPts = generateIndependentEdge(gcl, conn, false, inp);
                    GeneratedEdgeLine el;
                    el.id = rightElId;
                    el.geom = rightPts;
                    el.centerlineId = gcl.id;
                    el.side = "right";
                    el.qualityFlags = 0;
                    result.push_back(el);
                }

                edgeIdMap[gcl.id] = {leftElId, rightElId};
            }
        }

        // Backfill GeneratedCenterline leftEdgelineId / rightEdgelineId
        for(auto& gcl : centerlines) {
            auto it = edgeIdMap.find(gcl.id);
            if(it != edgeIdMap.end()) {
                gcl.leftEdgelineId  = it->second.first;
                gcl.rightEdgelineId = it->second.second;
            }
        }

        return result;
    }

private:
    // Generate a shared midline between two adjacent lanes
    Polyline generateSharedMidline(
        const GeneratedCenterline& gclI,
        const GeneratedCenterline& gclI1,
        const std::string& sharedEnterEdgeId,
        const std::string& sharedExitEdgeId,
        const IntersectionInput& inp) const
    {
        // Start point = shared enter edge connection point
        Point2D startPt{0,0};
        auto enterEdgeIt = inp.edgelines.find(sharedEnterEdgeId);
        if(enterEdgeIt != inp.edgelines.end()) {
            startPt = enterEdgeIt->second.connectionPt;
        }

        // End point = shared exit edge connection point
        Point2D endPt{0,0};
        auto exitEdgeIt = inp.edgelines.find(sharedExitEdgeId);
        if(exitEdgeIt != inp.edgelines.end()) {
            endPt = exitEdgeIt->second.connectionPt;
        }

        // Start tangent = average of enter tangent directions from both lanes
        auto enterClI = inp.centerlines.find(gclI.enterLineId);
        auto enterClI1 = inp.centerlines.find(gclI1.enterLineId);
        Point2D startTangI = (enterClI != inp.centerlines.end())
            ? enterClI->second.tangentDir : Point2D{0,1};
        Point2D startTangI1 = (enterClI1 != inp.centerlines.end())
            ? enterClI1->second.tangentDir : Point2D{0,1};
        Point2D startTangSum = startTangI + startTangI1;
        Point2D startTang = (startTangSum.norm() > EPS)
            ? startTangSum.normalized() : startTangI;

        // End tangent = average of exit tangent directions from both lanes (points inward)
        auto exitClI = inp.centerlines.find(gclI.exitLineId);
        auto exitClI1 = inp.centerlines.find(gclI1.exitLineId);
        Point2D endTangI = (exitClI != inp.centerlines.end())
            ? exitClI->second.tangentDir : Point2D{0,1};
        Point2D endTangI1 = (exitClI1 != inp.centerlines.end())
            ? exitClI1->second.tangentDir : Point2D{0,1};
        Point2D endTangSum = endTangI + endTangI1;
        Point2D endTang = (endTangSum.norm() > EPS)
            ? endTangSum.normalized() : endTangI;

        // Create cubic Bezier with alpha=0.38, beta=0.38
        double d = dist(startPt, endPt);
        if(d < EPS) {
            // Degenerate case: return a single-point polyline to avoid zero-length segments
            return {startPt};
        }

        CubicBezier cb = CubicBezier::fromAlphaBeta(startPt, startTang, endPt, endTang, 0.38, 0.38);

        // Sample adaptively for quality matching existing edges
        Polyline pts = cb.sampleAdaptive(3.0, 2.0, 0.1);
        if(pts.empty()) {
            pts = cb.sampleCount(30);
        }

        // Ensure exact endpoints
        if(!pts.empty()) {
            pts.front() = startPt;
            pts.back() = endPt;
        }

        return pts;
    }

    // Check shared midline curvature and return quality flags if too high
    int checkSharedMidlineCurvature(
        const GeneratedCenterline& gclI,
        const GeneratedCenterline& gclI1,
        const std::string& sharedEnterEdgeId,
        const std::string& sharedExitEdgeId,
        const IntersectionInput& inp) const
    {
        // Reconstruct the Bezier to check curvature (same logic as generateSharedMidline)
        Point2D startPt{0,0};
        auto enterEdgeIt = inp.edgelines.find(sharedEnterEdgeId);
        if(enterEdgeIt != inp.edgelines.end()) {
            startPt = enterEdgeIt->second.connectionPt;
        }

        Point2D endPt{0,0};
        auto exitEdgeIt = inp.edgelines.find(sharedExitEdgeId);
        if(exitEdgeIt != inp.edgelines.end()) {
            endPt = exitEdgeIt->second.connectionPt;
        }

        double d = dist(startPt, endPt);
        if(d < EPS) return 0;

        auto enterClI = inp.centerlines.find(gclI.enterLineId);
        auto enterClI1 = inp.centerlines.find(gclI1.enterLineId);
        Point2D startTangI = (enterClI != inp.centerlines.end())
            ? enterClI->second.tangentDir : Point2D{0,1};
        Point2D startTangI1 = (enterClI1 != inp.centerlines.end())
            ? enterClI1->second.tangentDir : Point2D{0,1};
        Point2D startTangSum = startTangI + startTangI1;
        Point2D startTang = (startTangSum.norm() > EPS)
            ? startTangSum.normalized() : startTangI;

        auto exitClI = inp.centerlines.find(gclI.exitLineId);
        auto exitClI1 = inp.centerlines.find(gclI1.exitLineId);
        Point2D endTangI = (exitClI != inp.centerlines.end())
            ? exitClI->second.tangentDir : Point2D{0,1};
        Point2D endTangI1 = (exitClI1 != inp.centerlines.end())
            ? exitClI1->second.tangentDir : Point2D{0,1};
        Point2D endTangSum = endTangI + endTangI1;
        Point2D endTang = (endTangSum.norm() > EPS)
            ? endTangSum.normalized() : endTangI;

        CubicBezier cb = CubicBezier::fromAlphaBeta(startPt, startTang, endPt, endTang, 0.38, 0.38);
        double maxK = cb.maxCurvature(30);
        if(maxK > cfg_.bezier.maxCurvature * 2.0) {
            return QF_WARN_CURVATURE_HIGH;
        }
        return 0;
    }

    // Generate an independent edge (left or right) using offset+smooth+bezierAlignEnds
    Polyline generateIndependentEdge(
        const GeneratedCenterline& gcl,
        const Connection* conn,
        bool isLeft,
        const IntersectionInput& inp) const
    {
        // Find the enter group
        auto grpIt = inp.laneGroups.find(conn->enterGroupId);
        if(grpIt == inp.laneGroups.end()) {
            // Fallback: simple offset
            double hw = cfg_.edgeLine.defaultLaneWidth * 0.5;
            return offsetPolyline(gcl.geom, isLeft ? hw : -hw);
        }
        const LaneGroup& grp = grpIt->second;

        // Estimate half-width
        double hw = estimateHalfWidth(conn->enterLineId, isLeft, grp, inp);

        // Generate offset polyline
        Polyline pts = offsetPolyline(gcl.geom, isLeft ? hw : -hw);

        // Remove duplicates
        removeDuplicates(pts, 0.02);

        if(pts.size() < 2) return pts;

        // Find edge endpoint and tangent at enter/exit
        std::string enterGrpId = conn->enterGroupId;
        std::string exitGrpId  = conn->exitGroupId;

        Point2D startPt, endPt;
        if(isLeft) {
            startPt = findEdgePtInGroup(conn->enterLineId, true,  enterGrpId, inp, hw);
            // Exit: exit tangent points inward, so left from exit perspective = right from curve perspective (flipped)
            endPt   = findEdgePtInGroup(conn->exitLineId,  false, exitGrpId,  inp, hw);
        } else {
            startPt = findEdgePtInGroup(conn->enterLineId, false, enterGrpId, inp, hw);
            endPt   = findEdgePtInGroup(conn->exitLineId,  true,  exitGrpId,  inp, hw);
        }

        Point2D enterTang = getEdgeTangent(conn->enterLineId, inp);
        Point2D exitTang  = getEdgeTangent(conn->exitLineId,  inp);

        // Smooth middle
        smoothMiddle(pts, 5);

        // Bezier align ends
        int smoothPts = std::max(6, (int)pts.size() / 4);
        bezierAlignEnds(pts, startPt, enterTang, endPt, exitTang, smoothPts);

        return pts;
    }

    // -----------------------------------------------
    // Remove near-duplicate consecutive points
    // -----------------------------------------------
    void removeDuplicates(Polyline& pts, double minDist) const {
        if(pts.size() < 3) return;
        Polyline out;
        out.push_back(pts.front());
        for(size_t i=1; i<pts.size()-1; ++i){
            if(dist(pts[i], out.back()) >= minDist){
                out.push_back(pts[i]);
            }
        }
        out.push_back(pts.back());
        pts = out;
    }

    // -----------------------------------------------
    // Offset polyline using bisector normals
    // offset > 0 = left, < 0 = right
    // -----------------------------------------------
    Polyline offsetPolyline(const Polyline& pts, double offset) const {
        if(pts.size()<2) return pts;
        int n = (int)pts.size();
        Polyline out;
        out.reserve(n);

        for(int i=0;i<n;++i){
            Point2D normal;
            if(i==0){
                Point2D dir = (pts[1]-pts[0]).normalized();
                normal = dir.rotLeft();
            } else if(i==n-1){
                Point2D dir = (pts[n-1]-pts[n-2]).normalized();
                normal = dir.rotLeft();
            } else {
                Point2D d1 = (pts[i]-pts[i-1]).normalized();
                Point2D d2 = (pts[i+1]-pts[i]).normalized();
                Point2D n1 = d1.rotLeft();
                Point2D n2 = d2.rotLeft();
                Point2D avg = n1 + n2;
                if(avg.norm() < EPS){
                    normal = n1;
                } else {
                    normal = avg.normalized();
                    double cosHalf = n1.dot(normal);
                    if(cosHalf > 0.3){
                        double miter = std::min(1.0/cosHalf, 3.0);
                        out.push_back(pts[i] + normal * (offset * miter));
                        continue;
                    }
                }
            }
            out.push_back(pts[i] + normal * offset);
        }
        return out;
    }

    // -----------------------------------------------
    // Moving average smoothing (preserve head/tail margins)
    // -----------------------------------------------
    void smoothMiddle(Polyline& pts, int passes) const {
        int n = (int)pts.size();
        if(n < 5) return;
        int margin = std::max(2, n/8);

        for(int pass=0; pass<passes; ++pass){
            Polyline tmp = pts;
            for(int i=margin; i<n-margin; ++i){
                Point2D sum = pts[i]*4.0;
                int cnt = 4;
                if(i-1>=0)   { sum += pts[i-1]*2.0; cnt+=2; }
                if(i+1<n)    { sum += pts[i+1]*2.0; cnt+=2; }
                if(i-2>=0)   { sum += pts[i-2]*1.0; cnt+=1; }
                if(i+2<n)    { sum += pts[i+2]*1.0; cnt+=1; }
                tmp[i] = sum / (double)cnt;
            }
            pts = tmp;
        }
    }

    // -----------------------------------------------
    // G1 Bezier refit at both ends + endpoint alignment
    // -----------------------------------------------
    void bezierAlignEnds(Polyline& pts,
        const Point2D& startPt, const Point2D& startTang,
        const Point2D& endPt,   const Point2D& endTang,
        int K) const
    {
        int n = (int)pts.size();
        if(n < 6) return;

        K = std::max(4, std::min(K, n/3));

        pts.front() = startPt;
        pts.back()  = endPt;

        // Head refit [0..K]
        {
            Point2D p0 = startPt;
            Point2D p3 = pts[K];
            double d = dist(p0, p3);
            if(d > EPS){
                Point2D t3;
                if(K+1 < n){
                    t3 = (pts[K+1] - pts[K-1]).normalized();
                } else {
                    t3 = (pts[K] - pts[K-1]).normalized();
                }
                double alpha = 0.38;
                Point2D p1 = p0 + startTang * (alpha * d);
                Point2D p2 = p3 - t3 * (alpha * d);
                CubicBezier cb(p0, p1, p2, p3);
                for(int i=1; i<K; ++i){
                    double t = (double)i / K;
                    pts[i] = cb.eval(t);
                }
            }
        }

        // Tail refit [n-1-K..n-1]
        {
            int startIdx = n-1-K;
            if(startIdx < K) startIdx = K;
            Point2D p0 = pts[startIdx];
            Point2D p3 = endPt;
            double d = dist(p0, p3);
            if(d > EPS){
                Point2D t0;
                if(startIdx > 0 && startIdx+1 < n){
                    t0 = (pts[startIdx] - pts[startIdx-1]).normalized();
                } else {
                    t0 = (p3 - p0).normalized();
                }
                double alpha = 0.38;
                Point2D p1 = p0 + t0 * (alpha * d);
                Point2D p2 = p3 + endTang * (alpha * d);
                CubicBezier cb(p0, p1, p2, p3);
                int count = n-1 - startIdx;
                for(int i=1; i<count; ++i){
                    double t = (double)i / count;
                    pts[startIdx + i] = cb.eval(t);
                }
            }
        }

        pts.front() = startPt;
        pts.back()  = endPt;
    }

    // -----------------------------------------------
    // Estimate half-width from edge endpoints
    // -----------------------------------------------
    double estimateHalfWidth(const std::string& enterLineId, bool isLeft,
                              const LaneGroup& grp, const IntersectionInput& inp) const
    {
        auto clit = inp.centerlines.find(enterLineId);
        if(clit==inp.centerlines.end()) return cfg_.edgeLine.defaultLaneWidth * 0.5;

        const Point2D& clPt = clit->second.connectionPt;
        Point2D tangent = clit->second.tangentDir;
        Point2D normal  = tangent.rotLeft();

        double bestDist = -1;
        for(auto& eid : grp.edgelineIds){
            auto elit = inp.edgelines.find(eid);
            if(elit==inp.edgelines.end()) continue;
            if(elit->second.geom.empty()) continue;
            const Point2D& ep = elit->second.connectionPt;
            double lateral = (ep-clPt).dot(normal);
            if(isLeft && lateral > 0.01 && lateral < 10.0){
                if(bestDist<0 || lateral<bestDist) bestDist=lateral;
            } else if(!isLeft && lateral < -0.01 && lateral > -10.0){
                if(bestDist<0 || (-lateral)<bestDist) bestDist=-lateral;
            }
        }

        if(bestDist > 0) return bestDist;
        return cfg_.edgeLine.defaultLaneWidth * 0.5;
    }

    // -----------------------------------------------
    // Find edge endpoint within a specific group
    // -----------------------------------------------
    Point2D findEdgePtInGroup(const std::string& lineId, bool isLeft,
                              const std::string& groupId,
                              const IntersectionInput& inp, double hw) const
    {
        auto clit = inp.centerlines.find(lineId);
        if(clit==inp.centerlines.end()) return {0,0};

        const Point2D& clPt  = clit->second.connectionPt;
        const Point2D& tang  = clit->second.tangentDir;
        Point2D normal = tang.rotLeft();

        auto git = inp.laneGroups.find(groupId);
        if(git != inp.laneGroups.end()){
            for(auto& eid : git->second.edgelineIds){
                auto elit = inp.edgelines.find(eid);
                if(elit == inp.edgelines.end()) continue;
                if(elit->second.geom.empty()) continue;
                const Point2D& ep = elit->second.connectionPt;
                double lat = (ep-clPt).dot(normal);
                if(isLeft && lat > 0.01 && std::abs(lat-hw) < hw*0.8) return ep;
                if(!isLeft && lat < -0.01 && std::abs(-lat-hw) < hw*0.8) return ep;
            }
        }
        return clPt + normal*(isLeft?hw:-hw);
    }

    // -----------------------------------------------
    // Get tangent direction for a centerline
    // -----------------------------------------------
    Point2D getEdgeTangent(const std::string& lineId, const IntersectionInput& inp) const {
        auto clit = inp.centerlines.find(lineId);
        if(clit==inp.centerlines.end()) return {0,1};
        return clit->second.tangentDir;
    }
};
