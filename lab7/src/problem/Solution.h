#ifndef SOLUTION_H
#define SOLUTION_H

#include <limits>
#include <vector>

class TSPAInstance;

class Solution {
public:
  Solution(); // Default constructor for empty solution
  Solution(const std::vector<int> &selectedNodes, const TSPAInstance &instance);

  double getTotalCost() const;
  double getPathLength() const;
  const std::vector<int> &getSelectedNodes() const;
  bool isEmpty() const;
  void printPath() const;
  void addStep();
  void setSteps(long long step);
  long long getSteps() const;

private:
  std::vector<int> selectedNodes;
  double totalCost;
  double pathLength;
  long long steps = 0;

  void calculateCostAndLength(const TSPAInstance &instance);
};

#endif
