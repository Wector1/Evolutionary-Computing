#ifndef SOLUTION_H
#define SOLUTION_H

#include <vector>
#include <limits>

class TSPAInstance;

class Solution {
public:
    Solution(); // Default constructor for empty solution
    Solution(const std::vector<int>& selectedNodes, const TSPAInstance& instance);
    
    double getTotalCost() const;
    double getPathLength() const;
    const std::vector<int>& getSelectedNodes() const;
    bool isEmpty() const;
    
private:
    std::vector<int> selectedNodes;
    double totalCost;
    double pathLength;
    
    void calculateCostAndLength(const TSPAInstance& instance);
};

#endif