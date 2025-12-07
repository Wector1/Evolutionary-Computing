#include "NearestNeighbour2Regret.h"
#include "TSPAInstance.h"
#include "Solution.h"
#include <vector>
#include <limits>
#include <queue>
#include <map>
#include <tuple>

#define INF 1000000000

using namespace std;

Solution NearestNeighbour2Regret::solve(const TSPAInstance& instance, int startingPoint) {
    const auto& distances = instance.getDistanceMatrix();
    const int n = instance.getTotalNodes();
    const int nodesToSelect = (n + 1) / 2;
    
    vector<bool> visited(n, false);
    vector<int> next_node(n, -1);
    priority_queue<tuple<int, int, int>> q_before;
    priority_queue<tuple<int, int, int>> q_end;
    map<pair<int, int>, priority_queue<tuple<int, int, int, int>>> q_middle;
    int first = startingPoint, last = startingPoint;
    
    // Initialize queues for before and end positions
    for (int j = 0; j < n; j++) {
        if (j != startingPoint) {
            int cost = distances[startingPoint][j] + instance.getNodeCost(j);
            q_before.push({-cost, j, startingPoint});
            q_end.push({-cost, j, startingPoint});
        }
    }
    visited[startingPoint] = true;
    
    for (int i = 0; i < nodesToSelect - 1; i++) {
        // For each unvisited node, find best and second-best insertion costs
        int bestNode = -1;
        int maxRegret = -INF;
        int bestCost = INF;
        int bestType = -1;  // 0=before, 1=end, 2=middle
        int bestSrc = -1, bestDst = -1;
        
        for (int j = 0; j < n; j++) {
            if (visited[j]) continue;
            
            vector<int> costs;
            vector<int> types;
            vector<pair<int, int>> edges;
            
            // Check cost to insert before first
            int cost_b = distances[first][j] + instance.getNodeCost(j);
            costs.push_back(cost_b);
            types.push_back(0);
            edges.push_back({-1, first});
            
            // Check cost to insert at end
            int cost_e = distances[last][j] + instance.getNodeCost(j);
            costs.push_back(cost_e);
            types.push_back(1);
            edges.push_back({last, -1});
            
            // Check cost to insert in middle (between existing edges)
            for (const auto& [key, pq] : q_middle) {
                auto [src, dst] = key;
                if (next_node[src] != dst) continue;  // Edge no longer exists
                
                int cost_m = distances[src][j] + distances[j][dst] + instance.getNodeCost(j) - distances[src][dst];
                costs.push_back(cost_m);
                types.push_back(2);
                edges.push_back({src, dst});
            }
            
            // Find best and second-best costs
            if (costs.size() < 2) continue;
            
            int minIdx = 0, secondMinIdx = 1;
            if (costs[1] < costs[0]) {
                minIdx = 1;
                secondMinIdx = 0;
            }
            
            for (size_t k = 2; k < costs.size(); k++) {
                if (costs[k] < costs[minIdx]) {
                    secondMinIdx = minIdx;
                    minIdx = k;
                } else if (costs[k] < costs[secondMinIdx]) {
                    secondMinIdx = k;
                }
            }
            
            int regret = costs[secondMinIdx] - costs[minIdx];
            
            // Select node with max regret (break ties by best cost)
            if (regret > maxRegret || (regret == maxRegret && costs[minIdx] < bestCost)) {
                maxRegret = regret;
                bestCost = costs[minIdx];
                bestNode = j;
                bestType = types[minIdx];
                bestSrc = edges[minIdx].first;
                bestDst = edges[minIdx].second;
            }
        }
        
        if (bestNode == -1) break;
        
        // Insert the selected node based on best position type
        if (bestType == 0) {  // Insert before first
            next_node[bestNode] = first;
            first = bestNode;
            visited[bestNode] = true;
            
            // Clear and rebuild q_before
            while (!q_before.empty()) q_before.pop();
            for (int j = 0; j < n; j++) {
                if (!visited[j]) {
                    int cost = distances[bestNode][j] + instance.getNodeCost(j);
                    q_before.push({-cost, j, bestNode});
                }
            }
            
            // Add middle candidates for new edge (bestNode -> next_node[bestNode])
            for (int j = 0; j < n; j++) {
                if (!visited[j]) {
                    int cost = distances[bestNode][j] + distances[j][next_node[bestNode]] + instance.getNodeCost(j) - distances[bestNode][next_node[bestNode]];
                    q_middle[{bestNode, next_node[bestNode]}].push({-cost, j, bestNode, next_node[bestNode]});
                }
            }
        } else if (bestType == 1) {  // Insert at end
            next_node[last] = bestNode;
            last = bestNode;
            visited[bestNode] = true;
            
            // Clear and rebuild q_end
            while (!q_end.empty()) q_end.pop();
            for (int j = 0; j < n; j++) {
                if (!visited[j]) {
                    int cost = distances[bestNode][j] + instance.getNodeCost(j);
                    q_end.push({-cost, j, bestNode});
                }
            }
            
            // Add middle candidates for new edge (prev_last -> bestNode)
            int prev_last = bestSrc;
            for (int j = 0; j < n; j++) {
                if (!visited[j]) {
                    int cost = distances[prev_last][j] + distances[j][bestNode] + instance.getNodeCost(j) - distances[prev_last][bestNode];
                    q_middle[{prev_last, bestNode}].push({-cost, j, prev_last, bestNode});
                }
            }
        } else {  // Insert in middle
            int src = bestSrc, dst = bestDst;
            next_node[src] = bestNode;
            next_node[bestNode] = dst;
            visited[bestNode] = true;
            
            q_middle.erase({src, dst});
            
            // Add middle candidates for two new edges
            for (int j = 0; j < n; j++) {
                if (!visited[j]) {
                    int cost1 = distances[src][j] + distances[j][bestNode] + instance.getNodeCost(j) - distances[src][bestNode];
                    q_middle[{src, bestNode}].push({-cost1, j, src, bestNode});
                    
                    int cost2 = distances[bestNode][j] + distances[j][dst] + instance.getNodeCost(j) - distances[bestNode][dst];
                    q_middle[{bestNode, dst}].push({-cost2, j, bestNode, dst});
                }
            }
        }
    }
    
    // Build path from first following next_node
    vector<int> path;
    path.reserve(nodesToSelect);
    path.push_back(first);
    for (int i = 1; i < nodesToSelect; i++) {
        first = next_node[first];
        path.push_back(first);
    }
    
    return Solution(path, instance);
}
