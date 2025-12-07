#include "NearestNeighbour.h"
#include "TSPAInstance.h"
#include "Solution.h"
#include <vector>
#include <limits>
#include <random>

#define INF 1000000000

using namespace std;

Solution NearestNeighbour::solve(const TSPAInstance& instance, int startingPoint) {
    int totalNodes = instance.getTotalNodes();
    const vector<vector<int>>& distances = instance.getDistanceMatrix();
    vector<int> path;
    vector<bool> visited(totalNodes, false);
    int nodesToSelect = (totalNodes + 1) / 2;

    int point = startingPoint;

    for (int i = 0; i < nodesToSelect; i++) {
        visited[point] = true;
        path.push_back(point);
        
        int lowest_cost_point = 0;
        int lowest_cost = INF;
        
        for (int j = 0; j < totalNodes; j++) {
            if (!visited[j]) {
                int distance = distances[point][j];
                int nodeCost = instance.getNodeCost(j);
                int totalCost = distance + nodeCost;
                
                if (totalCost < lowest_cost) {
                    lowest_cost = totalCost;
                    lowest_cost_point = j;
                }
            }
        }
        
        point = lowest_cost_point;
    }

    return Solution(path, instance);
}
