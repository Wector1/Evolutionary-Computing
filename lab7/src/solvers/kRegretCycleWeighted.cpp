#include "kRegretCycleWeighted.h"
#include "Solution.h"
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <queue>
#include <tuple>
#include <vector>

using std::pair;
using std::priority_queue;
using std::tuple;
using std::vector;

static constexpr int INF = 1000000000;
static int WEIGHT_REGRET = 1;
static int WEIGHT_OBJECTIVE_FUNC_CHANGE = 1;

enum class Place : uint8_t {
  Between = 0
}; // Only one type: insert between two nodes in cycle

// Candidate insertion descriptor
struct Cand {
  int cost = INF;
  int after = -1; // Insert new node after this node in the cycle

  bool operator<(const Cand &other) const { return cost < other.cost; }
};

// Merge a new candidate into top-2 best for a node
static inline void merge_top2(const Cand &c, Cand &b1, Cand &b2) {
  if (c.cost < b1.cost) {
    b2 = b1;
    b1 = c;
  } else if (c.cost < b2.cost) {
    b2 = c;
  }
}

static inline int regret_of(const Cand &b1, const Cand &b2) {
  return (b2.cost >= INF) ? 0 : (b2.cost - b1.cost);
}

// Global node heap entry: (-regret, -best1cost, node)
using NodeKey = tuple<int /*-regret*/, int /*-best1*/, int /*node*/>;

Solution kRegretCycleWeighted::solve(const TSPAInstance &instance, int point) {
  const auto &distances = instance.getDistanceMatrix();
  const int n = instance.getTotalNodes();
  const int nodesToPick = (n + 1) / 2;

  vector<bool> visited(n, false);
  vector<int> next_node(n, -1);
  vector<int> prev_node(n, -1);

  // Per-node current best two candidates
  vector<Cand> best1(n), best2(n);
  for (int j = 0; j < n; ++j) {
    best1[j].cost = best2[j].cost = INF;
  }

  int first = point;
  visited[point] = true;
  next_node[point] = point;
  prev_node[point] = point;

  vector<int> cycle;
  cycle.push_back(point);

  auto push_node_key = [&](priority_queue<NodeKey> &nodes, int j) {
    if (visited[j] || best1[j].cost >= INF)
      return;
    nodes.emplace(WEIGHT_REGRET * regret_of(best1[j], best2[j]) -
                      WEIGHT_OBJECTIVE_FUNC_CHANGE * best1[j].cost,
                  -best1[j].cost, j);
  };

  // Calculate all insertion positions for a given unvisited node
  auto calculate_candidates = [&](int j) {
    if (visited[j])
      return;

    best1[j].cost = best2[j].cost = INF;

    // Try inserting j after each node in the current cycle
    for (int u : cycle) {
      int v = next_node[u];
      // Cost = add edges u->j and j->v, remove edge u->v, add node cost
      int cost = distances[u][j] + distances[j][v] - distances[u][v] +
                 instance.getNodeCost(j);
      Cand c{cost, u};
      merge_top2(c, best1[j], best2[j]);
    }
  };

  // Recalculate candidates for all unvisited nodes
  auto recalculate_all = [&]() {
    for (int j = 0; j < n; ++j) {
      calculate_candidates(j);
    }
  };

  // Insert a node into the cycle after position 'after'
  auto insert_into_cycle = [&](int node, int after) {
    int before = next_node[after];

    next_node[after] = node;
    prev_node[node] = after;
    next_node[node] = before;
    prev_node[before] = node;

    visited[node] = true;
    cycle.push_back(node);
  };

  // Main k-regret loop (k=2)
  for (int picked = 1; picked < nodesToPick; ++picked) {
    // Recalculate all candidates based on current cycle
    recalculate_all();

    // Build priority queue of nodes by regret
    priority_queue<NodeKey> nodes;
    for (int j = 0; j < n; ++j) {
      push_node_key(nodes, j);
    }

    if (nodes.empty())
      break;

    // Select node with maximum regret
    int chosen = -1;
    while (!nodes.empty()) {
      auto [negReg, negBest1, node] = nodes.top();
      nodes.pop();

      if (!visited[node] && best1[node].cost < INF) {
        chosen = node;
        break;
      }
    }

    if (chosen == -1)
      break;

    // Insert at best position
    insert_into_cycle(chosen, best1[chosen].after);
  }

  // Build final path from cycle (starting from first node)
  vector<int> path;
  path.reserve(nodesToPick);
  int cur = first;
  for (int i = 0; i < nodesToPick; ++i) {
    path.push_back(cur);
    cur = next_node[cur];
  }

  return Solution(path, instance);
}

Solution kRegretCycleWeighted::solve(const TSPAInstance &instance, int point,
                                     std::vector<int> initialCycle) {
  const auto &distances = instance.getDistanceMatrix();
  const int n = instance.getTotalNodes();
  const int nodesToPick = (n + 1) / 2;

  vector<bool> visited(n, false);
  vector<int> next_node(n, -1);
  vector<int> prev_node(n, -1);

  // Per-node current best two candidates
  vector<Cand> best1(n), best2(n);
  for (int j = 0; j < n; ++j) {
    best1[j].cost = best2[j].cost = INF;
  }

  int first = initialCycle[0];
  point = first;
  visited[point] = true;
  next_node[point] = point;
  prev_node[point] = point;

  vector<int> cycle;
  cycle.push_back(point);

  auto push_node_key = [&](priority_queue<NodeKey> &nodes, int j) {
    if (visited[j] || best1[j].cost >= INF)
      return;
    nodes.emplace(WEIGHT_REGRET * regret_of(best1[j], best2[j]) -
                      WEIGHT_OBJECTIVE_FUNC_CHANGE * best1[j].cost,
                  -best1[j].cost, j);
  };

  // Calculate all insertion positions for a given unvisited node
  auto calculate_candidates = [&](int j) {
    if (visited[j])
      return;

    best1[j].cost = best2[j].cost = INF;

    // Try inserting j after each node in the current cycle
    for (int u : cycle) {
      int v = next_node[u];
      // Cost = add edges u->j and j->v, remove edge u->v, add node cost
      int cost = distances[u][j] + distances[j][v] - distances[u][v] +
                 instance.getNodeCost(j);
      Cand c{cost, u};
      merge_top2(c, best1[j], best2[j]);
    }
  };

  // Recalculate candidates for all unvisited nodes
  auto recalculate_all = [&]() {
    for (int j = 0; j < n; ++j) {
      calculate_candidates(j);
    }
  };
  auto print = [&]() {
    std::cout << "next nodes:" << std::endl;
    for (auto &it : next_node) {
      std::cout << it << ", ";
    }
    std::cout << std::endl;

    std::cout << "pref nodes:" << std::endl;
    for (auto &it : prev_node) {
      std::cout << it << ", ";
    }
    std::cout << std::endl;

    std::cout << "cycle: " << std::endl;
    for (auto &it : cycle) {
      std::cout << it << ", ";
    }
    std::cout << std::endl;
  };
  // Insert a node into the cycle after position 'after'
  auto insert_into_cycle = [&](int node, int after) {
    int before = next_node[after];

    next_node[after] = node;
    prev_node[node] = after;
    next_node[node] = before;
    prev_node[before] = node;

    visited[node] = true;
    cycle.push_back(node);
  };
  // print();
  // std::cout << "INSERTING" << std::endl;
  for (int i = 1; i < initialCycle.size(); i++) {
    insert_into_cycle(initialCycle[i], initialCycle[i - 1]);
    // std::cout << "INSERTED NODE " << i << std::endl;
    // print();
  }
  // print();

  // Main k-regret loop (k=2)
  for (int picked = cycle.size(); picked < nodesToPick; ++picked) {
    // Recalculate all candidates based on current cycle
    recalculate_all();

    // Build priority queue of nodes by regret
    priority_queue<NodeKey> nodes;
    for (int j = 0; j < n; ++j) {
      push_node_key(nodes, j);
    }

    if (nodes.empty())
      break;

    // Select node with maximum regret
    int chosen = -1;
    while (!nodes.empty()) {
      auto [negReg, negBest1, node] = nodes.top();
      nodes.pop();

      if (!visited[node] && best1[node].cost < INF) {
        chosen = node;
        break;
      }
    }

    if (chosen == -1)
      break;

    // Insert at best position
    insert_into_cycle(chosen, best1[chosen].after);
  }

  // Build final path from cycle (starting from first node)
  vector<int> path;
  path.reserve(nodesToPick);
  int cur = first;
  for (int i = 0; i < nodesToPick; ++i) {
    path.push_back(cur);
    cur = next_node[cur];
  }
  // print();
  // std::cout << "Final path: ";
  // for (auto &node : path) {
  //   std::cout << node << " ";
  // }
  // std::cout << std::endl;

  return Solution(path, instance);
}
