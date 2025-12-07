#include "LNS.h"
#include "LocalSteepest.h"
#include "RandomSearch.h"
#include "Solution.h"
#include "SolverRunner.h"
#include "TSPAInstance.h"
#include "kRegretCycleWeighted.h"
#include <algorithm>
#include <bitset>
#include <cmath>
#include <iostream>
#include <random>
#include <set>
#include <vector>

Solution LNS::solve(const TSPAInstance &instance, int startingPoint) {
  StartingStrategy s = getStartingStrategy();
  IntraMode m = getIntraMode();

  RandomSearch randomSolver;
  LocalSteepest LSStRandomEdgeSolver(LocalSteepest::StartingStrategy::Random,
                                     LocalSteepest::IntraMode::EdgeExchange);
  Solution bestSolution = randomSolver.solve(instance, startingPoint);
  LSStRandomEdgeSolver.initialPath = bestSolution.getSelectedNodes();
  bestSolution = LSStRandomEdgeSolver.solve(instance, 0);
  int bestCost = 1'000'000'001;

  auto removeNodes = [](const std::vector<int> &path,
                        const TSPAInstance instance,
                        double p) -> std::vector<int> {
    if (path.empty() || p <= 0.0) {
      return path;
    }
    // std::cout << "p: " << p << std::endl;

    int n = path.size();
    int m = static_cast<int>(n * p); // Number of nodes to remove

    // std::cout << "n: " << n << ", m: " << m << std::endl;
    if (m <= 0) {
      return path;
    }

    // Generate random uniform values and compute keys
    std::vector<std::pair<double, int>> keys; // {key, node_index}
    keys.reserve(n);

    for (int i = 0; i < n; i++) {
      double cost = instance.getNodeCost(path[i]);
      double u = static_cast<double>(rand()) / RAND_MAX; // Uniform[0,1]

      // For inverse linear probability, we want higher probability for lower
      // cost So we use 1/cost as the weight, but we need to handle 0 cost
      double p_i = (cost > 0) ? cost : 1.0; // Avoid division by zero
      double key = std::pow(u, 1.0 / p_i);

      keys.emplace_back(key, path[i]);
    }

    // Sort by keys in descending order to get the m largest keys
    std::sort(keys.begin(), keys.end(),
              [](const auto &a, const auto &b) { return a.first > b.first; });

    // Create a set of nodes to remove
    std::vector<int> nodesToRemove;
    nodesToRemove.reserve(m);
    for (int i = 0; i < m; i++) {
      nodesToRemove.push_back(keys[i].second);
    }

    // Filter out the nodes to remove
    std::vector<int> result;
    result.reserve(n - m);

    for (int nodeIdx : path) {
      if (std::find(nodesToRemove.begin(), nodesToRemove.end(), nodeIdx) ==
          nodesToRemove.end()) {
        result.push_back(nodeIdx);
      }
    }

    return result;
  };

  auto runStart = std::chrono::high_resolution_clock::now();
  long long step = 0;
  for (;; step++) {
    // while (true) {
    // Solution currentSolution = randomSolver.solve(instance, startingPoint);
    Solution currentSolution = bestSolution;

    const auto &distanceMatrix = instance.getDistanceMatrix();
    std::random_device rd;
    std::mt19937 gen(rd());

    std::vector<int> selectedNodes = currentSolution.getSelectedNodes();

    std::vector<int> smallerVector = removeNodes(
        currentSolution.getSelectedNodes(), instance, removePercent_);

    std::vector<bool> isSelected(instance.getTotalNodes(), false);
    kRegretCycleWeighted k;
    currentSolution = k.solve(instance, 0, smallerVector);
    if (doLocalSearch_) {
      LSStRandomEdgeSolver.initialPath = currentSolution.getSelectedNodes();
      currentSolution = LSStRandomEdgeSolver.solve(instance, 0);
    }

    int currentSolutionCost = currentSolution.getTotalCost();
    if (currentSolutionCost < bestCost) {
      bestCost = currentSolutionCost;
      bestSolution = currentSolution;
      // std::cout << "step: " << step << std::endl;
    }
    auto runEnd = std::chrono::high_resolution_clock::now();
    double elapsedMs =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            runEnd - runStart)
            .count();
    if (elapsedMs >= maxTime_) {
      // std::cout << step << ",";
      break;
    }
    // if (step > 2) {
    //   break;
    // }
  }
  bestSolution.setSteps(step);
  return bestSolution;
}
