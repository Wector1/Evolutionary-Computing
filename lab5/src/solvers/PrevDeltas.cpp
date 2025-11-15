#include "PrevDeltas.h"
#include "TSPAInstance.h"
#include "Solution.h"
#include "RandomSearch.h"
#include "SolverRunner.h"
#include "kRegretCycleWeighted.h"
#include <vector>
#include <random>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <tuple>
#include <numeric>

namespace {

inline int prevIdx(int i, int n) { return (i - 1 + n) % n; }
inline int nextIdx(int i, int n) { return (i + 1) % n; }

enum class MoveType { InterRoute, EdgeExchange };

struct Move {
    MoveType type;
    // For EdgeExchange: u1->v1 and u2->v2 are the two edges (stored as node IDs)
    // For InterRoute: nodeIn is the selected node to remove, nodeOut is unselected node to add
    int u1, v1, u2, v2; // EdgeExchange
    int nodeIn, nodeOut; // InterRoute
    double delta;
    
    Move() : type(MoveType::InterRoute), u1(-1), v1(-1), u2(-1), v2(-1), nodeIn(-1), nodeOut(-1), delta(0.0) {}
};

// Comparator for priority queue (max heap by improvement = most negative delta first)
struct MoveComparator {
    bool operator()(const Move& a, const Move& b) const {
        return a.delta > b.delta; // min heap of deltas (most negative = best)
    }
};

struct MoveHash {
    size_t operator()(const Move& m) const {
        std::string key;
        if (m.type == MoveType::EdgeExchange) {
            int e1a = std::min(m.u1, m.v1), e1b = std::max(m.u1, m.v1);
            int e2a = std::min(m.u2, m.v2), e2b = std::max(m.u2, m.v2);
            if (e1a > e2a || (e1a == e2a && e1b > e2b)) {
                std::swap(e1a, e2a);
                std::swap(e1b, e2b);
            }
            key = "E_" + std::to_string(e1a) + "_" + std::to_string(e1b) + "_" + 
                  std::to_string(e2a) + "_" + std::to_string(e2b);
        } else {
            key = "I_" + std::to_string(m.nodeIn) + "_" + std::to_string(m.nodeOut);
        }
        return std::hash<std::string>{}(key);
    }
};

struct MoveEqual {
    bool operator()(const Move& a, const Move& b) const {
        if (a.type != b.type) return false;
        if (a.type == MoveType::EdgeExchange) {
            auto normalize = [](int u, int v) { return std::make_pair(std::min(u,v), std::max(u,v)); };
            auto e1a = normalize(a.u1, a.v1), e1b = normalize(a.u2, a.v2);
            auto e2a = normalize(b.u1, b.v1), e2b = normalize(b.u2, b.v2);
            return (e1a == e2a && e1b == e2b) || (e1a == e2b && e1b == e2a);
        } else {
            return a.nodeIn == b.nodeIn && a.nodeOut == b.nodeOut;
        }
    }
};

}

Solution PrevDeltas::solve(const TSPAInstance& instance, int startingPoint) {
    StartingStrategy s = getStartingStrategy();
    
    Solution currentSolution;
    if (s == StartingStrategy::Random) {
        RandomSearch randomSolver;
        currentSolution = randomSolver.solve(instance, startingPoint);
    } else {
        kRegretCycleWeighted kRegretSolver;
        currentSolution = kRegretSolver.solve(instance, startingPoint);
    }

    std::vector<int> route = currentSolution.getSelectedNodes();
    const auto& dist = instance.getDistanceMatrix();
    const int nTotal = instance.getTotalNodes();
    int n = route.size();

    std::vector<bool> isSelected(nTotal, false);
    for (int node : route) {
        isSelected[node] = true;
    }
    
    std::vector<int> unselected;
    for (int i = 0; i < nTotal; ++i) {
        if (!isSelected[i]) unselected.push_back(i);
    }

    // Position map: node -> index in route
    std::vector<int> posOf(nTotal, -1);
    auto updatePositions = [&]() {
        for (int i = 0; i < n; ++i) {
            posOf[route[i]] = i;
        }
    };
    updatePositions();

    std::priority_queue<Move, std::vector<Move>, MoveComparator> LM;
    std::unordered_set<Move, MoveHash, MoveEqual> inLM;

    // Check if edge exists in current route with given direction
    // Returns: 0=doesn't exist, 1=exists in same direction, 2=exists in reverse direction
    auto checkEdgeDirection = [&](int u, int v) -> int {
        if (!isSelected[u] || !isSelected[v]) return 0;
        int pu = posOf[u], pv = posOf[v];
        if (nextIdx(pu, n) == pv) return 1; // u->v exists
        if (nextIdx(pv, n) == pu) return 2; // v->u exists (reversed)
        return 0; // edge doesn't exist
    };

    // Evaluate delta for EdgeExchange move
    auto evalEdgeExchange = [&](int u1, int v1, int u2, int v2) -> double {
        return -dist[u1][v1] - dist[u2][v2] + dist[u1][u2] + dist[v1][v2];
    };

    // Evaluate delta for InterRoute move
    auto evalInterRoute = [&](int nodeIn, int nodeOut) -> double {
        if (!isSelected[nodeIn] || isSelected[nodeOut]) return 0.0;
        int pos = posOf[nodeIn];
        int prev = route[prevIdx(pos, n)];
        int next = route[nextIdx(pos, n)];
        return -dist[prev][nodeIn] - dist[nodeIn][next] 
               + dist[prev][nodeOut] + dist[nodeOut][next]
               - instance.getNodeCost(nodeIn) + instance.getNodeCost(nodeOut);
    };

    // Generate ALL edge moves (used for initial population only)
    auto generateAllEdgeMoves = [&]() {
        for (int i = 0; i < n; ++i) {
            int u1 = route[i];
            int v1 = route[nextIdx(i, n)];
            
            for (int j = i + 2; j < n; ++j) {
                if (j == nextIdx(i, n)) continue;
                
                int u2 = route[j];
                int v2 = route[nextIdx(j, n)];
                
                double delta = evalEdgeExchange(u1, v1, u2, v2);
                if (delta < -1e-9) {
                    Move m;
                    m.type = MoveType::EdgeExchange;
                    m.u1 = u1; m.v1 = v1; m.u2 = u2; m.v2 = v2;
                    m.delta = delta;
                    if (inLM.find(m) == inLM.end()) {
                        LM.push(m);
                        inLM.insert(m);
                    }
                }
                
                double deltaInv = evalEdgeExchange(v1, u1, u2, v2);
                if (deltaInv < -1e-9) {
                    Move mInv;
                    mInv.type = MoveType::EdgeExchange;
                    mInv.u1 = v1; mInv.v1 = u1; mInv.u2 = u2; mInv.v2 = v2;
                    mInv.delta = deltaInv;
                    if (inLM.find(mInv) == inLM.end()) {
                        LM.push(mInv);
                        inLM.insert(mInv);
                    }
                }
            }
        }
    };
    
    // Generate edge moves involving specific edges (incremental after a move)
    auto generateEdgeMovesForEdges = [&](const std::vector<std::pair<int,int>>& affectedEdges) {
        for (const auto& [u1, v1] : affectedEdges) {
            if (!isSelected[u1] || !isSelected[v1]) continue;
            
            // Pair this edge with all other edges in the route
            for (int i = 0; i < n; ++i) {
                int u2 = route[i];
                int v2 = route[nextIdx(i, n)];
                
                // Skip if edges are adjacent or overlapping
                if (u2 == u1 || u2 == v1 || v2 == u1 || v2 == v1) continue;
                
                // Evaluate with current direction
                double delta = evalEdgeExchange(u1, v1, u2, v2);
                if (delta < -1e-9) {
                    Move m;
                    m.type = MoveType::EdgeExchange;
                    m.u1 = u1; m.v1 = v1; m.u2 = u2; m.v2 = v2;
                    m.delta = delta;
                    if (inLM.find(m) == inLM.end()) {
                        LM.push(m);
                        inLM.insert(m);
                    }
                }
                
                // Evaluate with inverted edge
                double deltaInv = evalEdgeExchange(v1, u1, u2, v2);
                if (deltaInv < -1e-9) {
                    Move mInv;
                    mInv.type = MoveType::EdgeExchange;
                    mInv.u1 = v1; mInv.v1 = u1; mInv.u2 = u2; mInv.v2 = v2;
                    mInv.delta = deltaInv;
                    if (inLM.find(mInv) == inLM.end()) {
                        LM.push(mInv);
                        inLM.insert(mInv);
                    }
                }
            }
        }
    };

    // Generate ALL inter-route moves (used for initial population only)
    auto generateAllInterMoves = [&]() {
        for (int i = 0; i < n; ++i) {
            int nodeIn = route[i];
            for (int nodeOut : unselected) {
                double delta = evalInterRoute(nodeIn, nodeOut);
                if (delta < -1e-9) {
                    Move m;
                    m.type = MoveType::InterRoute;
                    m.nodeIn = nodeIn; m.nodeOut = nodeOut;
                    m.delta = delta;
                    if (inLM.find(m) == inLM.end()) {
                        LM.push(m);
                        inLM.insert(m);
                    }
                }
            }
        }
    };
    
    // Generate inter-route moves for specific positions (incremental after a move)
    auto generateInterMovesForPositions = [&](const std::vector<int>& positions) {
        for (int pos : positions) {
            if (pos < 0 || pos >= n) continue;
            int nodeIn = route[pos];
            
            // Try replacing this node with all unselected nodes
            for (int nodeOut : unselected) {
                double delta = evalInterRoute(nodeIn, nodeOut);
                if (delta < -1e-9) {
                    Move m;
                    m.type = MoveType::InterRoute;
                    m.nodeIn = nodeIn; m.nodeOut = nodeOut;
                    m.delta = delta;
                    if (inLM.find(m) == inLM.end()) {
                        LM.push(m);
                        inLM.insert(m);
                    }
                }
            }
        }
    };

    generateAllEdgeMoves();
    generateAllInterMoves();

    while (!LM.empty()) {
        Move m = LM.top();
        LM.pop();
        inLM.erase(m);

        bool applicable = false;
        
        if (m.type == MoveType::EdgeExchange) {
            // Check edge directions as per task description
            int dir1 = checkEdgeDirection(m.u1, m.v1);
            int dir2 = checkEdgeDirection(m.u2, m.v2);
            
            if (dir1 == 0 || dir2 == 0) {
                // At least one edge doesn't exist -> remove from LM (already popped)
                continue;
            }
            
            if (dir1 == 2 || dir2 == 2) {
                // Edge(s) exist but in reversed direction -> not applicable now
                // Re-insert with inverted direction for future
                if (dir1 == 2) std::swap(m.u1, m.v1);
                if (dir2 == 2) std::swap(m.u2, m.v2);
                m.delta = evalEdgeExchange(m.u1, m.v1, m.u2, m.v2);
                if (m.delta < -1e-9 && inLM.find(m) == inLM.end()) {
                    LM.push(m);
                    inLM.insert(m);
                }
                continue;
            }
            
            // Both edges exist in same direction -> applicable
            applicable = true;
            
        } else { // InterRoute
            // Check if move is still valid
            if (!isSelected[m.nodeIn] || isSelected[m.nodeOut]) {
                continue;
            }
            
            double newDelta = evalInterRoute(m.nodeIn, m.nodeOut);
            if (newDelta >= -1e-9) {
                continue;
            }
            
            applicable = true;
            m.delta = newDelta;
        }

        if (!applicable) continue;

        // Apply the move and generate only NEW moves
        if (m.type == MoveType::EdgeExchange) {
            // 2-opt: reverse segment between the two edges
            int pos1 = posOf[m.u1];
            int pos2 = posOf[m.u2];
            int start = nextIdx(pos1, n);
            int end = pos2;
            
            // Store old edges that will be removed
            int oldEdge1_u = m.u1, oldEdge1_v = m.v1;
            int oldEdge2_u = m.u2, oldEdge2_v = m.v2;
            
            // Reverse segment
            if (start <= end) {
                std::reverse(route.begin() + start, route.begin() + end + 1);
            } else {
                // Wrap-around case
                std::vector<int> temp;
                for (int i = start; i < n; ++i) temp.push_back(route[i]);
                for (int i = 0; i <= end; ++i) temp.push_back(route[i]);
                std::reverse(temp.begin(), temp.end());
                int idx = start;
                for (int node : temp) {
                    route[idx] = node;
                    idx = nextIdx(idx, n);
                }
            }
            
            updatePositions();
            
            // NEW edges after 2-opt: u1→u2 and v1→v2
            std::vector<std::pair<int,int>> newEdges;
            newEdges.push_back({m.u1, m.u2});
            newEdges.push_back({m.v1, m.v2});
            
            generateEdgeMovesForEdges(newEdges);
            
            // InterRoute: affected positions are endpoints of new edges
            std::vector<int> affectedPositions = {
                posOf[m.u1], posOf[m.u2], posOf[m.v1], posOf[m.v2]
            };
            generateInterMovesForPositions(affectedPositions);
            
        } else { // InterRoute
            int pos = posOf[m.nodeIn];
            int prevNode = route[prevIdx(pos, n)];
            int nextNode = route[nextIdx(pos, n)];
            
            route[pos] = m.nodeOut;
            isSelected[m.nodeIn] = false;
            isSelected[m.nodeOut] = true;
            posOf[m.nodeOut] = pos;
            posOf[m.nodeIn] = -1;
            
            unselected.erase(std::remove(unselected.begin(), unselected.end(), m.nodeOut), unselected.end());
            unselected.push_back(m.nodeIn);
            
            std::vector<std::pair<int,int>> newEdges;
            newEdges.push_back({prevNode, m.nodeOut});
            newEdges.push_back({m.nodeOut, nextNode});
            
            // Generate moves involving only the NEW edges
            generateEdgeMovesForEdges(newEdges);
            
            // InterRoute: affected positions are the 3 nodes in the modified segment
            std::vector<int> affectedPositions = {
                posOf[prevNode], posOf[m.nodeOut], posOf[nextNode]
            };
            generateInterMovesForPositions(affectedPositions);
        }
    }

    return Solution(route, instance);
}