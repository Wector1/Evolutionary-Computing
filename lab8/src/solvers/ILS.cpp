#include "ILS.h"
#include "RandomSearch.h"
#include "Solution.h"
#include "SolverRunner.h"
#include "TSPAInstance.h"
#include "kRegretCycleWeighted.h"
#include <algorithm>
#include <bitset>
#include <iostream>
#include <random>
#include <set>
#include <vector>
#include <chrono>

Solution ILS::solve(const TSPAInstance &instance, int startingPoint) {
  StartingStrategy s = getStartingStrategy();
  IntraMode m = getIntraMode();

  // Assuming you have these variables defined:
  // std::vector<int> selectedNodes;
  // std::set<int> unselectedNodes;
  // int k; // number of operations
  // double p; // probability for swap operation

  // Lambda function 1: k random node exchanges with unselectedNodes
  auto exchangeWithUnselected = [](std::vector<int> &selectedNodes,
                                   std::set<int> &unselectedNodes, int k) {
    if (selectedNodes.empty() || unselectedNodes.empty() || k <= 0)
      return;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> disSelected(0, selectedNodes.size() - 1);
    std::uniform_int_distribution<> disUnselected(0,
                                                  unselectedNodes.size() - 1);

    for (int i = 0; i < k; ++i) {
      // Pick random node from selectedNodes
      int selectedIdx = disSelected(gen);
      int oldNode = selectedNodes[selectedIdx];

      // Pick random node from unselectedNodes
      auto it = unselectedNodes.begin();
      std::advance(it, disUnselected(gen) % unselectedNodes.size());
      int newNode = *it;

      // Exchange
      selectedNodes[selectedIdx] = newNode;

      // Update sets
      unselectedNodes.erase(newNode);
      unselectedNodes.insert(oldNode);
    }
  };

  // Lambda function 2: k random swaps of 2 nodes from selectedNodes
  auto swapNodes = [](std::vector<int> &selectedNodes, int k) {
    if (selectedNodes.size() < 2 || k <= 0)
      return;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, selectedNodes.size() - 1);

    for (int i = 0; i < k; ++i) {
      int idx1 = dis(gen);
      int idx2 = dis(gen);
      while (idx1 == idx2) {
        idx2 = dis(gen);
      }

      // Make sure we don't swap with itself
      if (idx1 != idx2) {
        std::swap(selectedNodes[idx1], selectedNodes[idx2]);
      }
    }
  };

  // Lambda function 3: swap each pair of consecutive nodes with probability p
  auto swapConsecutivePairs = [](std::vector<int> &selectedNodes, double p) {
    if (selectedNodes.size() < 2)
      return;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::bernoulli_distribution dis(p);

    // Process pairs of consecutive nodes
    for (size_t i = 0; i + 1 < selectedNodes.size(); i += 2) {
      if (dis(gen)) {
        std::swap(selectedNodes[i], selectedNodes[i + 1]);
      }
    }
  };

  // Lambda function 4: rearrange nodes based on positions a and b
  auto rearrangeNodes = [](std::vector<int> &selectedNodes, int a, int b) {
    if (selectedNodes.empty() || a < 0 || b < 0 || a >= selectedNodes.size() ||
        b >= selectedNodes.size() || a >= b) {
      return;
    }

    std::vector<int> newOrder;

    // Add nodes from positions [a, b] first
    for (int i = a; i <= b; ++i) {
      newOrder.push_back(selectedNodes[i]);
    }

    // Add nodes from positions [0, a-1]
    for (int i = 0; i < a; ++i) {
      newOrder.push_back(selectedNodes[i]);
    }

    // Add nodes from positions [b+1, end]
    for (int i = b + 1; i < selectedNodes.size(); ++i) {
      newOrder.push_back(selectedNodes[i]);
    }

    selectedNodes = newOrder;
  };

  // Lambda function that selects and applies ONE random perturbation from the
  // set bits
  auto applyPerturbation = [&](Solution &currentSolution,
                               const std::bitset<4> &perturbationBitset, int k,
                               double p) {
    std::vector<int> selectedNodes = currentSolution.getSelectedNodes();
    std::vector<bool> isSelected(instance.getTotalNodes(), false);
    for (int node : selectedNodes) {
      isSelected[node] = true;
    }
    std::set<int> unselectedNodes;
    for (int i = 0; i < instance.getTotalNodes(); ++i) {
      if (!isSelected[i]) {
        unselectedNodes.insert(i);
      }
    }
    // Collect all enabled perturbation types
    std::vector<int> enabledPerturbations;

    for (int i = 0; i < 4; ++i) {
      if (perturbationBitset[i]) {
        enabledPerturbations.push_back(i);
      }
    }

    // If no perturbations are enabled, do nothing
    if (enabledPerturbations.empty()) {
      return;
    }

    // Randomly select one perturbation from the enabled ones
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, enabledPerturbations.size() - 1);
    int selectedPerturbation = enabledPerturbations[dis(gen)];

    // currentSolution.printPath();
    // Apply the selected perturbation
    switch (selectedPerturbation) {
    case 0: // exchange with unselected nodes
      // std::cout << "case 0" << std::endl;
      exchangeWithUnselected(selectedNodes, unselectedNodes, k);
      break;
    case 1: // swap nodes
      // std::cout << "case 1" << std::endl;
      swapNodes(selectedNodes, k);
      break;
    case 2: // consecutive pairs swap
      // std::cout << "case 2" << std::endl;
      swapConsecutivePairs(selectedNodes, p);
      break;
    case 3: // rearrange nodes
      // std::cout << "case 3" << std::endl;
      if (selectedNodes.size() >= 2) {
        std::uniform_int_distribution<> posDis(0, selectedNodes.size() - 1);
        int a = posDis(gen);
        int b = posDis(gen);

        // Ensure a <= b and both are valid indices
        if (a > b)
          std::swap(a, b);

        // Make sure we have at least 2 elements in range
        if (b - a >= 1) {
          rearrangeNodes(selectedNodes, a, b);
        }
      }
      break;
    }
    currentSolution = Solution(selectedNodes, instance);
    // currentSolution.printPath();
  };

  // Example usage:
  /*
  std::vector<int> selectedNodes = {1, 2, 3, 4, 5};
  std::set<int> unselectedNodes = {6, 7, 8, 9, 10};
  std::bitset<4> bitset(15); // All bits set (0b1111) - allows all perturbations
  int k = 2;
  double p = 0.5;

  applyPerturbation(selectedNodes, unselectedNodes, bitset, k, p);
  */

  RandomSearch randomSolver;
  Solution bestSolution = randomSolver.solve(instance, startingPoint);
  int bestCost = 1'000'000'001;

  auto runStart = std::chrono::high_resolution_clock::now();
  for (long long step = 0;; step++) {
    // while (true) {
    // Solution currentSolution = randomSolver.solve(instance, startingPoint);
    Solution currentSolution = bestSolution;
    // std::cout << "======" << std::endl;
    // currentSolution.printPath();
    applyPerturbation(currentSolution, bs_, 5, 0.15);
    // currentSolution.printPath();
    // std::cout << "======" << std::endl << std::endl;

    const auto &distanceMatrix = instance.getDistanceMatrix();
    std::random_device rd;
    std::mt19937 gen(rd());

    std::vector<int> selectedNodes = currentSolution.getSelectedNodes();
    std::vector<bool> isSelected(instance.getTotalNodes(), false);
    for (int node : selectedNodes) {
      isSelected[node] = true;
    }
    std::vector<int> unselectedNodes;
    for (int i = 0; i < instance.getTotalNodes(); ++i) {
      if (!isSelected[i]) {
        unselectedNodes.push_back(i);
      }
    }
    bool improvement = true;

    while (improvement) {
      improvement = false;
      double bestDelta = 0.0;

      // Track the best move found
      int bestMoveCategory = -1;
      int bestMoveType = -1;
      int bestI = -1;
      int bestJ = -1;

      std::vector<int> selectedNodes = currentSolution.getSelectedNodes();
      int n = selectedNodes.size();

      // Build list of unselected nodes (needed for Inter moves)
      std::vector<bool> isSelected(instance.getTotalNodes(), false);
      for (int node : selectedNodes) {
        isSelected[node] = true;
      }

      std::vector<int> unselectedNodes;
      for (int i = 0; i < instance.getTotalNodes(); ++i) {
        if (!isSelected[i]) {
          unselectedNodes.push_back(i);
        }
      }

      // Generate all moves: Intra (node-exchange OR edge-exchange) and Inter
      // Format: {moveCategory, type, i, j}
      // moveCategory: 0=Intra, 1=Inter
      // For Intra: type: 0=node-exchange, 1=edge-exchange
      // For Inter: type is unused
      std::vector<std::tuple<int, int, int, int>> allMoves;

      // Add Intra-route moves based on IntraMode
      if (m == IntraMode::NodeExchange) {
        // Add two-node exchange moves (Intra)
        for (int i = 0; i < n; ++i) {
          for (int j = i + 1; j < n; ++j) {
            allMoves.push_back(
                {0, 0, i, j}); // category=0 (Intra), type=0 (node)
          }
        }
      } else if (m == IntraMode::EdgeExchange) {
        // Add two-edge exchange moves (Intra)
        for (int i = 0; i < n; ++i) {
          int edge1_end = (i + 1) % n;
          for (int j = i + 2; j < n; ++j) {
            if (j == edge1_end)
              continue;
            allMoves.push_back(
                {0, 1, i, j}); // category=0 (Intra), type=1 (edge)
          }
        }
      }

      // Add inter-route moves
      for (int i = 0; i < n; ++i) {
        for (int j = 0; j < unselectedNodes.size(); ++j) {
          allMoves.push_back({1, 0, i, j}); // category=1 (Inter)
        }
      }

      // Evaluate ALL moves to find the best one (steepest descent)
      for (const auto &move : allMoves) {
        int moveCategory = std::get<0>(move);
        int moveType = std::get<1>(move);
        int i = std::get<2>(move);
        int j = std::get<3>(move);

        double delta = 0.0;

        if (moveCategory == 0) {
          // INTRA-ROUTE MOVES
          if (moveType == 0) {
            // Two-node exchange
            int node_i = selectedNodes[i];
            int node_j = selectedNodes[j];
            int prev_i = selectedNodes[(i - 1 + n) % n];
            int next_i = selectedNodes[(i + 1) % n];
            int prev_j = selectedNodes[(j - 1 + n) % n];
            int next_j = selectedNodes[(j + 1) % n];

            // Handle adjacent nodes case
            if (j == i + 1) {
              delta -= distanceMatrix[prev_i][node_i];
              delta -= distanceMatrix[node_i][node_j];
              delta -= distanceMatrix[node_j][next_j];

              delta += distanceMatrix[prev_i][node_j];
              delta += distanceMatrix[node_j][node_i];
              delta += distanceMatrix[node_i][next_j];
            } else if (i == 0 && j == n - 1) {
              delta -= distanceMatrix[prev_j][node_j];
              delta -= distanceMatrix[node_j][node_i];
              delta -= distanceMatrix[node_i][next_i];

              delta += distanceMatrix[prev_j][node_i];
              delta += distanceMatrix[node_i][node_j];
              delta += distanceMatrix[node_j][next_i];
            } else {
              delta -= distanceMatrix[prev_i][node_i];
              delta -= distanceMatrix[node_i][next_i];
              delta -= distanceMatrix[prev_j][node_j];
              delta -= distanceMatrix[node_j][next_j];

              delta += distanceMatrix[prev_i][node_j];
              delta += distanceMatrix[node_j][next_i];
              delta += distanceMatrix[prev_j][node_i];
              delta += distanceMatrix[node_i][next_j];
            }

          } else {
            // Two-edge exchange
            int edge1_start = i;
            int edge1_end = (i + 1) % n;
            int edge2_start = j;
            int edge2_end = (j + 1) % n;

            int u1 = selectedNodes[edge1_start];
            int v1 = selectedNodes[edge1_end];
            int u2 = selectedNodes[edge2_start];
            int v2 = selectedNodes[edge2_end];

            delta -= distanceMatrix[u1][v1];
            delta -= distanceMatrix[u2][v2];

            delta += distanceMatrix[u1][u2];
            delta += distanceMatrix[v1][v2];
          }

        } else {
          // INTER-ROUTE MOVES
          int nodeInIdx = i;
          int nodeOutIdx = j;

          int nodeIn = selectedNodes[nodeInIdx];
          int nodeOut = unselectedNodes[nodeOutIdx];
          int prevNode = selectedNodes[(nodeInIdx - 1 + n) % n];
          int nextNode = selectedNodes[(nodeInIdx + 1) % n];

          delta -= distanceMatrix[prevNode][nodeIn];
          delta -= distanceMatrix[nodeIn][nextNode];

          delta += distanceMatrix[prevNode][nodeOut];
          delta += distanceMatrix[nodeOut][nextNode];

          delta -= instance.getNodeCost(nodeIn);
          delta += instance.getNodeCost(nodeOut);
        }

        if (delta < bestDelta) {
          bestDelta = delta;
          bestMoveCategory = moveCategory;
          bestMoveType = moveType;
          bestI = i;
          bestJ = j;
        }
      }

      if (bestDelta < 0) {
        improvement = true;

        if (bestMoveCategory == 0) {
          // Apply INTRA-ROUTE move
          if (bestMoveType == 0) {
            // Two-node exchange
            std::swap(selectedNodes[bestI], selectedNodes[bestJ]);
          } else {
            // Two-edge exchange
            int edge1_end = (bestI + 1) % n;
            int edge2_start = bestJ;
            std::reverse(selectedNodes.begin() + edge1_end,
                         selectedNodes.begin() + edge2_start + 1);
          }
        } else {
          // Apply INTER-ROUTE move
          selectedNodes[bestI] = unselectedNodes[bestJ];
        }

        currentSolution = Solution(selectedNodes, instance);
      }
    }
    int currentSolutionCost = currentSolution.getTotalCost();
    if (currentSolutionCost < bestCost) {
      bestCost = currentSolutionCost;
      bestSolution = currentSolution;
      bestSolution.setSteps(step);
    }
    auto runEnd = std::chrono::high_resolution_clock::now();
    double elapsedMs =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            runEnd - runStart)
            .count();
    if (elapsedMs >= maxTime_) {
      std::cout << step << ",";
      break;
    }
  }
  return bestSolution;
}
