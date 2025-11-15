#include "NeighbourSteepest.h"
#include "RandomSearch.h"
#include "Solution.h"
#include "SolverRunner.h"
#include "TSPAInstance.h"
#include "kRegretCycleWeighted.h"
#include <algorithm>
#include <random>
#include <vector>
#include <iostream>
#include <set>

Solution NeighbourSteepest::solve(const TSPAInstance &instance,
                                  int startingPoint) {
  StartingStrategy s = getStartingStrategy();
  IntraMode m = getIntraMode();

  Solution currentSolution;
  if (s == StartingStrategy::Random) {
    RandomSearch randomSolver;
    currentSolution = randomSolver.solve(instance, startingPoint);
  } else {
    kRegretCycleWeighted kRegretSolver;
    currentSolution = kRegretSolver.solve(instance, startingPoint);
  }

  const auto &distanceMatrix = instance.getDistanceMatrix();
  // Number of candidate neighbours to consider per vertex (per task ~10)
  constexpr int K_CANDIDATES = 10;
  std::vector<std::vector<std::pair<int, int>>> distances_and_costs(
      (int)distanceMatrix.size(),
      std::vector<std::pair<int, int>>((int)distanceMatrix.size(), {0, 0}));
  for (int i = 0; i < distanceMatrix.size(); i++) {
      for (int j = 0; j < distanceMatrix.size(); j++) {
          if (j != i)
            distances_and_costs[i][j] = {distanceMatrix[i][j] + instance.getNodeCost(j), j};
          else
              distances_and_costs[i][j] = {0, j};
      }
  }

  for (int i = 0; i < distanceMatrix.size(); i++) {
      std::sort(distances_and_costs[i].begin(), distances_and_costs[i].end());
  }
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
  std::vector<int> nodesIndices(distanceMatrix.size(), -1);
  for (int i = 0; i < (int)selectedNodes.size(); i++) {
        nodesIndices[selectedNodes[i]] = i;
    }
    int n = selectedNodes.size();

    // Build list of unselected nodes (needed for Inter moves)
    std::vector<bool> isSelected(instance.getTotalNodes(), false);
    for (int node : selectedNodes) {
      isSelected[node] = true;
    }

    std::vector<int> unselectedNodes;
    std::set<int> unselectedNodesSet;
    for (int i = 0; i < instance.getTotalNodes(); ++i) {
      if (!isSelected[i]) {
        unselectedNodes.push_back(i);
        unselectedNodesSet.insert(i);
      }
    }

    // Generate all moves: Intra (node-exchange OR edge-exchange) and Inter
    // Format: {moveCategory, type, i, j}
    // moveCategory: 0=Intra, 1=Inter
    // For Intra: type: 0=node-exchange, 1=edge-exchange
    // For Inter: type is unused
    std::vector<std::tuple<int, int, int, int>> allMoves;

    // Build candidate moves only (do not iterate all moves)
    // For each selected node u, consider its K nearest neighbours v (by dist + node cost)
    for (int u : selectedNodes) {
      int posU = nodesIndices[u];
      if (posU < 0) continue;
      int limit = std::min(K_CANDIDATES, (int)distanceMatrix.size() - 1);
      for (int k = 1; k <= limit; ++k) { // k=0 is self
        int v = distances_and_costs[u][k].second;
        if (unselectedNodesSet.count(v)) {
          // Inter-route candidate moves that introduce candidate edge (u,v)
          int posAfterU = (posU + 1) % n;
          int posBeforeU = (posU - 1 + n) % n;
          // Replace successor of u with v => introduces edge (u,v)
          allMoves.push_back({1, 0, posAfterU, v});
          // Replace predecessor of u with v => introduces edge (v,u)
          allMoves.push_back({1, 0, posBeforeU, v});
        } else {
          // Intra-route 2-opt candidate introducing edge (u,v)
          int posV = nodesIndices[v];
          if (posV < 0) continue;
          int a = std::min(posU, posV);
          int b = std::max(posU, posV);
          // skip adjacent or wraparound (invalid 2-opt)
          if (b == (a + 1) % n) continue;
          if (a == 0 && b == n - 1) continue;
          allMoves.push_back({0, 1, a, b}); // Intra 2-opt between a and b
        }
      }
    }

    // Add inter-route moves
    // for (int i = 0; i < n; ++i) {
    //   for (int j = 0; j < unselectedNodes.size(); ++j) {
    //     allMoves.push_back({1, 0, i, j}); // category=1 (Inter)
    //   }
    // }

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
        int nodeOut = nodeOutIdx; // here j is a node id (unselected)
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
        selectedNodes[bestI] = bestJ; // bestJ holds unselected node id
      }

      currentSolution = Solution(selectedNodes, instance);
    }
  }
  return currentSolution;
}