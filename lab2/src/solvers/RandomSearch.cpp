#include "RandomSearch.h"
#include "TSPAInstance.h"
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <numeric>
#include <random>

RandomSearch::RandomSearch() {
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
}

Solution RandomSearch::solve(const TSPAInstance& instance, int startingPoint) {
    int totalNodes = instance.getTotalNodes();
    int nodesToSelect = (totalNodes + 1) / 2; // Select 50% of nodes, rounding up if odd
    Solution bestSolution;
    
    std::random_device rd;
    std::mt19937 gen(rd());

    // for (int i = 0; i < 200; ++i) {
    std::vector<int> allNodes(totalNodes);
    std::iota(allNodes.begin(), allNodes.end(), 0); // 0..N-1
    // Ensure startingPoint is included
    std::shuffle(allNodes.begin(), allNodes.end(), gen);
    // Move startingPoint to front
    auto it = std::find(allNodes.begin(), allNodes.end(), startingPoint % totalNodes);
    if (it != allNodes.end()) std::iter_swap(allNodes.begin(), it);

    std::vector<int> currentSelection;
    currentSelection.reserve(nodesToSelect);
    for (int idx = 0; idx < totalNodes && static_cast<int>(currentSelection.size()) < nodesToSelect; ++idx) {
        int node = allNodes[idx];
        if (std::find(currentSelection.begin(), currentSelection.end(), node) == currentSelection.end()) {
            currentSelection.push_back(node);
        }
    }
    Solution currentSolution(currentSelection, instance);
        
        // if (bestSolution.isEmpty() || currentSolution.getTotalCost() < bestSolution.getTotalCost()) {
        //     bestSolution = currentSolution;
        // }
    // }

    // return bestSolution;
    return currentSolution;
}