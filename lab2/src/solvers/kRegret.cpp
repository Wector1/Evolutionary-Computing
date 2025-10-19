#include "kRegret.h"
#include "Solution.h"
#include <queue>
#include <tuple>
#include <vector>
#include <cstdint>

using std::pair;
using std::priority_queue;
using std::tuple;
using std::vector;

static constexpr int INF = 1000000000;

enum class Place : uint8_t { Before = 0, End = 1, Middle = 2 };

// Candidate insertion descriptor
struct Cand {
    int cost = INF;
    Place where = Place::Middle;
    int src = -1; // for Middle: existing edge src->dst; for End: src==last; for Before: src unused
    int dst = -1; // for Middle: existing edge src->dst; for Before: dst==first; for End: dst unused
};

// Merge a new candidate into top-2 best for a node
static inline void merge_top2(const Cand& c, Cand& b1, Cand& b2) {
    if (c.cost < b1.cost) { b2 = b1; b1 = c; }
    else if (c.cost < b2.cost) { b2 = c; }
}

static inline bool edge_valid(const vector<int>& next_node, int first, int last, const Cand& c) {
    if (c.cost >= INF) return false;
    switch (c.where) {
        case Place::Before: return c.dst == first;
        case Place::End:    return c.src == last;
        case Place::Middle: return (c.src >= 0 && next_node[c.src] == c.dst);
    }
    return false;
}

static inline int regret_of(const Cand& b1, const Cand& b2) {
    return (b2.cost >= INF) ? 0 : (b2.cost - b1.cost);
}

// Node-position heap entry: negative cost for min-heap behavior via default max PQ
using NodePos = tuple<int /*negCost*/, uint8_t /*where*/, int /*src*/, int /*dst*/>;
// Global node heap entry: (-regret, -best1cost, node)
using NodeKey = tuple<int /*-regret*/, int /*-best1*/, int /*node*/>;

Solution kRegret::solve(const TSPAInstance& instance, int point) {
    const auto& distances = instance.getDistanceMatrix();
    const int n = instance.getTotalNodes();
    const int nodesToPick = (n + 1) / 2;

    vector<bool> visited(n, false);
    vector<int> next_node(n, -1);

    // Per-node candidate heaps
    vector<priority_queue<NodePos>> nodeHeaps(n);
    // Per-node current best two (maintained incrementally)
    vector<Cand> best1(n), best2(n);
    for (int j = 0; j < n; ++j) { best1[j].cost = best2[j].cost = INF; }

    int first = point, last = point;
    visited[point] = true;

    auto push_node_key = [&](priority_queue<NodeKey>& nodes, int j) {
        if (visited[j] || best1[j].cost >= INF) return;
        nodes.emplace(-regret_of(best1[j], best2[j]), -best1[j].cost, j);
    };

    // Add a candidate position (and update best1/best2)
    auto add_candidate = [&](int j, const Cand& c) {
        if (visited[j]) return;
        int neg = -c.cost;
        uint8_t w = static_cast<uint8_t>(c.where);
        nodeHeaps[j].emplace(neg, w, c.src, c.dst);
        merge_top2(c, best1[j], best2[j]);
    };

    // Seed "before first" and "end last" for all unvisited nodes
    auto seed_ends = [&]() {
        for (int j = 0; j < n; ++j) {
            if (visited[j]) continue;
            // Before
            {
                int cost = distances[first][j] + instance.getNodeCost(j);
                add_candidate(j, Cand{cost, Place::Before, -1, first});
            }
            // End
            {
                int cost = distances[last][j] + instance.getNodeCost(j);
                add_candidate(j, Cand{cost, Place::End, last, -1});
            }
        }
    };

    // For a new middle edge (u -> v), add candidates for all unvisited nodes
    auto add_middle_for_edge = [&](int u, int v) {
        for (int j = 0; j < n; ++j) {
            if (visited[j]) continue;
            int cost = distances[u][j] + distances[j][v] + instance.getNodeCost(j) - distances[u][v]; // cost of inserting j between u and v
            add_candidate(j, Cand{cost, Place::Middle, u, v});
        }
    };

    // Repair best1/best2 for a node by popping invalid entries from its heap
    auto repair_node = [&](int j) {
        if (visited[j]) { best1[j].cost = best2[j].cost = INF; return; }
        best1[j].cost = best2[j].cost = INF;
        auto& pq = nodeHeaps[j];
        // Pop invalid until two valid found
        while (!pq.empty() && (!edge_valid(next_node, first, last, Cand{-std::get<0>(pq.top()),
                           static_cast<Place>(std::get<1>(pq.top())), std::get<2>(pq.top()), std::get<3>(pq.top())}))) {
            pq.pop();
        }
        if (!pq.empty()) {
            auto t = pq.top(); pq.pop();
            Cand c1{-std::get<0>(t), static_cast<Place>(std::get<1>(t)), std::get<2>(t), std::get<3>(t)};
            merge_top2(c1, best1[j], best2[j]);
            // Find second valid
            while (!pq.empty()) {
                auto t2 = pq.top();
                Cand c2{-std::get<0>(t2), static_cast<Place>(std::get<1>(t2)), std::get<2>(t2), std::get<3>(t2)};
                if (!edge_valid(next_node, first, last, c2)) { pq.pop(); continue; }
                merge_top2(c2, best1[j], best2[j]);
                break;
            }
            // Push back the popped best candidate
            nodeHeaps[j].emplace(-c1.cost, static_cast<uint8_t>(c1.where), c1.src, c1.dst);
        }
    };

    // Initialize
    seed_ends();
    priority_queue<NodeKey> nodes;
    for (int j = 0; j < n; ++j) push_node_key(nodes, j);

    // Main k-regret loop (k=2)
    for (int picked = 1; picked < nodesToPick; ++picked) {
        int chosen = -1;
        Cand where;

        // Extract a valid top node by lazy invalidation
        // The heap is keyed by (-regret, -best1cost, node), so top has MAX regret
        while (!nodes.empty()) {
            auto [negReg, negBest1, j] = nodes.top(); nodes.pop();
            if (visited[j]) continue;
            
            // Repair if needed - this ensures best1/best2 are current
            repair_node(j);
            
            // After repair, check if this node still has valid insertions
            if (best1[j].cost >= INF) continue;
            
            // Check if the regret/cost in the heap entry matches current state
            int currentRegret = regret_of(best1[j], best2[j]);
            if (-negReg != currentRegret || -negBest1 != best1[j].cost) {
                // Stale entry, re-push with updated values
                push_node_key(nodes, j);
                continue;
            }
            chosen = j;
            where = best1[j];
            break;
        }
        if (chosen == -1) break; // no feasible (shouldn't happen)

        // Apply insertion
        if (where.where == Place::Before) {
            int oldFirst = first;
            next_node[chosen] = oldFirst;
            first = chosen;
            visited[chosen] = true;
            // New middle edge (chosen -> oldFirst)
            add_middle_for_edge(chosen, oldFirst);
            // First changed => seed new "before first" for all unvisited
            for (int j = 0; j < n; ++j) {
                if (visited[j]) continue;
                int cost = distances[first][j] + instance.getNodeCost(j);
                add_candidate(j, Cand{cost, Place::Before, -1, first});
                push_node_key(nodes, j);
            }
        } else if (where.where == Place::End) {
            int oldLast = last;
            next_node[oldLast] = chosen;
            last = chosen;
            visited[chosen] = true;
            // New middle edge (oldLast -> chosen)
            add_middle_for_edge(oldLast, chosen);
            // Last changed => seed new "end last" for all unvisited
            for (int j = 0; j < n; ++j) {
                if (visited[j]) continue;
                int cost = distances[last][j] + instance.getNodeCost(j);
                add_candidate(j, Cand{cost, Place::End, last, -1});
                push_node_key(nodes, j);
            }
        } else { // Middle insertion
            int u = where.src, v = where.dst;
            next_node[u] = chosen;
            next_node[chosen] = v;
            visited[chosen] = true;
            // Replace edge u->v by u->chosen and chosen->v
            add_middle_for_edge(u, chosen);
            add_middle_for_edge(chosen, v);
            // Ends unchanged; before/end seeds remain valid; new middle positions added above
            for (int j = 0; j < n; ++j) {
                if (visited[j]) continue;
                push_node_key(nodes, j);
            }
        }

        // Invalidate chosen node's bests
        best1[chosen].cost = best2[chosen].cost = INF;
    }

    // Build path from first following next_node
    vector<int> path;
    path.reserve(nodesToPick);
    int cur = first;
    path.push_back(cur);
    for (int i = 1; i < nodesToPick; ++i) {
        cur = next_node[cur];
        path.push_back(cur);
    }
    return Solution(path, instance);
}

// #include "kRegret.h"
// #include "Solution.h"
// #include <queue>
// #include <tuple>
// #include <vector>
// #include <cstdint>
// #include <algorithm>

// using std::pair;
// using std::priority_queue;
// using std::tuple;
// using std::vector;

// static constexpr int INF = 1000000000;

// enum class Place : uint8_t { Between = 0 }; // Only one type: insert between two nodes in cycle

// // Candidate insertion descriptor
// struct Cand {
//     int cost = INF;
//     int after = -1;  // Insert new node after this node in the cycle
    
//     bool operator<(const Cand& other) const {
//         return cost < other.cost;
//     }
// };

// // Merge a new candidate into top-2 best for a node
// static inline void merge_top2(const Cand& c, Cand& b1, Cand& b2) {
//     if (c.cost < b1.cost) { 
//         b2 = b1; 
//         b1 = c; 
//     }
//     else if (c.cost < b2.cost) { 
//         b2 = c; 
//     }
// }

// static inline int regret_of(const Cand& b1, const Cand& b2) {
//     return (b2.cost >= INF) ? 0 : (b2.cost - b1.cost);
// }

// // Global node heap entry: (-regret, -best1cost, node)
// using NodeKey = tuple<int /*-regret*/, int /*-best1*/, int /*node*/>;

// Solution kRegret::solve(const TSPAInstance& instance, int point) {
//     const auto& distances = instance.getDistanceMatrix();
//     const int n = instance.getTotalNodes();
//     const int nodesToPick = (n + 1) / 2;

//     vector<bool> visited(n, false);
//     vector<int> next_node(n, -1);
//     vector<int> prev_node(n, -1);

//     // Per-node current best two candidates
//     vector<Cand> best1(n), best2(n);
//     for (int j = 0; j < n; ++j) { 
//         best1[j].cost = best2[j].cost = INF; 
//     }

//     // Initialize with starting node as a self-loop
//     int first = point;
//     visited[point] = true;
//     next_node[point] = point;
//     prev_node[point] = point;

//     vector<int> cycle;
//     cycle.push_back(point);

//     auto push_node_key = [&](priority_queue<NodeKey>& nodes, int j) {
//         if (visited[j] || best1[j].cost >= INF) return;
//         nodes.emplace(-regret_of(best1[j], best2[j]), -best1[j].cost, j);
//     };

//     // Calculate all insertion positions for a given unvisited node
//     auto calculate_candidates = [&](int j) {
//         if (visited[j]) return;
        
//         best1[j].cost = best2[j].cost = INF;
        
//         // Try inserting j after each node in the current cycle
//         for (int u : cycle) {
//             int v = next_node[u];
//             // Cost = add edges u->j and j->v, remove edge u->v, add node cost
//             int cost = distances[u][j] + distances[j][v] - distances[u][v] + instance.getNodeCost(j);
//             Cand c{cost, u};
//             merge_top2(c, best1[j], best2[j]);
//         }
//     };

//     // Recalculate candidates for all unvisited nodes
//     auto recalculate_all = [&]() {
//         for (int j = 0; j < n; ++j) {
//             calculate_candidates(j);
//         }
//     };

//     // Insert a node into the cycle after position 'after'
//     auto insert_into_cycle = [&](int node, int after) {
//         int before = next_node[after];
        
//         // Update links
//         next_node[after] = node;
//         prev_node[node] = after;
//         next_node[node] = before;
//         prev_node[before] = node;
        
//         visited[node] = true;
//         cycle.push_back(node);
//     };

//     // Main k-regret loop (k=2)
//     for (int picked = 1; picked < nodesToPick; ++picked) {
//         // Recalculate all candidates based on current cycle
//         recalculate_all();
        
//         // Build priority queue of nodes by regret
//         priority_queue<NodeKey> nodes;
//         for (int j = 0; j < n; ++j) {
//             push_node_key(nodes, j);
//         }
        
//         if (nodes.empty()) break;
        
//         // Select node with maximum regret
//         auto [negReg, negBest1, chosen] = nodes.top();
        
//         if (visited[chosen] || best1[chosen].cost >= INF) break;
        
//         // Insert at best position
//         insert_into_cycle(chosen, best1[chosen].after);
//     }

//     // Build final path from cycle (starting from first node)
//     vector<int> path;
//     path.reserve(nodesToPick);
//     int cur = first;
//     for (int i = 0; i < nodesToPick; ++i) {
//         path.push_back(cur);
//         cur = next_node[cur];
//     }
    
//     return Solution(path, instance);
// }