#include "NearestNeighbourAny.h"
#include "TSPAInstance.h"
#include "Solution.h"
#include <vector>
#include <limits>
#include <map>

#define INF 1000000000

using namespace std;

Solution NearestNeighbourAny::solve(const TSPAInstance& instance, int startingPoint) {
    const auto& distances = instance.getDistanceMatrix();
    const int n = instance.getTotalNodes();
    const int nodesToSelect = (n + 1) / 2;
    
    vector<bool> visited(n, false);
    vector<int> next_node(n, -1);
    int first = startingPoint, last = startingPoint;
    
    visited[startingPoint] = true;
    
    for (int i = 0; i < nodesToSelect - 1; i++) {
        // For each unvisited node, find the best insertion cost (objective function only)
        int bestNode = -1;
        int bestCost = INF;
        int bestType = -1;  // 0=before, 1=end, 2=middle
        int bestSrc = -1, bestDst = -1;
        
        for (int j = 0; j < n; j++) {
            if (visited[j]) continue;
            
            int minCost = INF;
            int minType = -1;
            int minSrc = -1, minDst = -1;
            
            // Check cost to insert before first
            int cost_b = distances[first][j] + instance.getNodeCost(j);
            if (cost_b < minCost) {
                minCost = cost_b;
                minType = 0;
                minSrc = -1;
                minDst = first;
            }
            
            // Check cost to insert at end
            int cost_e = distances[last][j] + instance.getNodeCost(j);
            if (cost_e < minCost) {
                minCost = cost_e;
                minType = 1;
                minSrc = last;
                minDst = -1;
            }
            
            // Check cost to insert in middle (between existing edges)
            int curr = first;
            while (next_node[curr] != -1) {
                int next = next_node[curr];
                int cost_m = distances[curr][j] + distances[j][next] + instance.getNodeCost(j) - distances[curr][next];
                
                if (cost_m < minCost) {
                    minCost = cost_m;
                    minType = 2;
                    minSrc = curr;
                    minDst = next;
                }
                
                curr = next;
            }
            
            // Select node with minimum insertion cost (best objective function)
            if (minCost < bestCost) {
                bestCost = minCost;
                bestNode = j;
                bestType = minType;
                bestSrc = minSrc;
                bestDst = minDst;
            }
        }
        
        if (bestNode == -1) break;
        
        // Insert the selected node based on best position type
        if (bestType == 0) {  // Insert before first
            next_node[bestNode] = first;
            first = bestNode;
            visited[bestNode] = true;
            
        } else if (bestType == 1) {  // Insert at end
            next_node[last] = bestNode;
            last = bestNode;
            visited[bestNode] = true;
            
        } else {  // Insert in middle
            int src = bestSrc, dst = bestDst;
            next_node[src] = bestNode;
            next_node[bestNode] = dst;
            visited[bestNode] = true;
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
