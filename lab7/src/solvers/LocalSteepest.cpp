#include "LocalSteepest.h"
#include "RandomSearch.h"
#include "Solution.h"
#include "SolverRunner.h"
#include "TSPAInstance.h"
#include "kRegretCycleWeighted.h"
#include <algorithm>
#include <random>
#include <vector>

Solution LocalSteepest::solve(const TSPAInstance &instance, int startingPoint) {
  StartingStrategy s = getStartingStrategy();
  IntraMode m = getIntraMode();
  Solution currentSolution;
  if (initialPath.size() != 0) {
    currentSolution = Solution(initialPath, instance);
    // currentSolution.printPath();
  } else {
    // currentSolution = Solution(selectedNodes, instance);
    if (s == StartingStrategy::Random) {
      RandomSearch randomSolver;
      currentSolution = randomSolver.solve(instance, startingPoint);
    } else {
      kRegretCycleWeighted kRegretSolver;
      currentSolution = kRegretSolver.solve(instance, startingPoint);
    }
  }

  const auto &distanceMatrix = instance.getDistanceMatrix();
  std::random_device rd;
  std::mt19937 gen(rd());

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
          allMoves.push_back({0, 0, i, j}); // category=0 (Intra), type=0 (node)
        }
      }
    } else if (m == IntraMode::EdgeExchange) {
      // Add two-edge exchange moves (Intra)
      for (int i = 0; i < n; ++i) {
        int edge1_end = (i + 1) % n;
        for (int j = i + 2; j < n; ++j) {
          if (j == edge1_end)
            continue;
          allMoves.push_back({0, 1, i, j}); // category=0 (Intra), type=1 (edge)
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
  return currentSolution;
}
