#include "Solution.h"
#include "TSPAInstance.h"
#include <cmath>
#include <numeric>

Solution::Solution() : totalCost(std::numeric_limits<double>::max()), pathLength(0.0) {
}

Solution::Solution(const std::vector<int>& selectedNodes, const TSPAInstance& instance)
    : selectedNodes(selectedNodes), totalCost(0.0), pathLength(0.0) {
    calculateCostAndLength(instance);
}

void Solution::calculateCostAndLength(const TSPAInstance& instance) {
    totalCost = 0.0;
    pathLength = 0.0;
    
    const auto& distanceMatrix = instance.getDistanceMatrix();

    for (int node : selectedNodes) {
        totalCost += instance.getNodeCost(node);
    }

    // Calculate path length (Hamiltonian cycle)
    for (size_t i = 0; i < selectedNodes.size(); ++i) {
        int from = selectedNodes[i];
        int to = selectedNodes[(i + 1) % selectedNodes.size()]; // Wrap around to form a cycle
        pathLength += distanceMatrix[from][to];
    }
    
    // Total cost is path length + node costs
    totalCost += pathLength;
}

double Solution::getTotalCost() const {
    return totalCost;
}

double Solution::getPathLength() const {
    return pathLength;
}

const std::vector<int>& Solution::getSelectedNodes() const {
    return selectedNodes;
}

bool Solution::isEmpty() const {
    return selectedNodes.empty();
}