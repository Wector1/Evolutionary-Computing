#include "NearestNeighbourCycle.h"
#include "Solution.h"
#include <queue>
#include <map>
#include <tuple>
#include <limits>

using std::pair; using std::priority_queue; using std::tuple; using std::vector; using std::make_pair;

// Helper alias types
using ThreeTuple = tuple<int,int,int>;               // cost, point, source
using FourTuple  = tuple<int,int,int,int>;           // cost, point, source, dest

static constexpr int INF = 1000000000;

Solution NearestNeighbourCycle::solve(const TSPAInstance& instance, int point) {
    const auto& distances = instance.getDistanceMatrix();
    const int n = instance.getTotalNodes();
    const int nodesToPick = n / 2; // same as algo: cycle of size floor(n/2)

    vector<bool> visited(n, false);
    vector<int> next_node(n, -1);
    priority_queue<ThreeTuple> q_before;
    priority_queue<ThreeTuple> q_end;
    std::map<pair<int,int>, priority_queue<FourTuple>> q_middle;
    int first = point, last = point;

    // initialize q_before and q_end
    for (int j = 0; j < n; j++) {
        if (j != point) {
            int cost = distances[point][j] + instance.getNodeCost(j);
            q_before.push({-cost, j, point});
        }
    }
    for (int j = 0; j < n; j++) {
        if (j != point) {
            int cost = distances[point][j] + instance.getNodeCost(j);
            q_end.push({-cost, j, point});
        }
    }
    visited[point] = true;

    for (int i = 0; i < nodesToPick - 1; i++) {
        int cost_b = INF, cost_m = INF, cost_e = INF;
        int point_b = -1, point_m = -1, point_e = -1;
        int source_b = -1, source_e = -1;
        int source_m = -1, dest_m = -1;
        int type = -1;

        // best candidate before (attach at the beginning)
        while (!q_before.empty()) {
            auto [cost_local, point_local, source_local] = q_before.top();
            if (source_local != first || visited[point_local]) {
                q_before.pop();
                continue;
            }
            cost_b = -cost_local;
            point_b = point_local;
            source_b = source_local;
            break;
        }

        // best candidate end (attach at the end)
        while (!q_end.empty()) {
            auto [cost_local, point_local, source_local] = q_end.top();
            if (source_local != last || visited[point_local]) {
                q_end.pop();
                continue;
            }
            cost_e = -cost_local;
            point_e = point_local;
            source_e = source_local;
            break;
        }

        // best candidate middle (insert between)
        for (auto& kv : q_middle) {
            auto& value = kv.second;
            while (!value.empty()) {
                auto [cost_local, point_local, source_local, dest_local] = value.top();
                if (next_node[source_local] != dest_local || visited[point_local]) {
                    value.pop();
                } else {
                    if (-cost_local < cost_m) {
                        cost_m = -cost_local;
                        source_m = source_local;
                        dest_m = dest_local;
                        point_m = point_local;
                    }
                    break;
                }
            }
        }

        if (cost_b <= cost_e && cost_b <= cost_m) {
            type = 0;
        } else if (cost_e < cost_m) {
            type = 1;
        } else {
            type = 2;
        }

        if (type == 0) {
            next_node[point_b] = source_b;
            visited[point_b] = true;
            first = point_b;
            while (!q_before.empty()) q_before.pop();
            while (!q_end.empty()) q_end.pop();
            for (int j = 0; j < n; j++) {
                if (!visited[j]) {
                    int cost = distances[point_b][j] + distances[j][source_b] + instance.getNodeCost(j) - distances[point_b][source_b];
                    q_middle[{point_b, source_b}].push({-cost, j, point_b, source_b});
                }
            }
        } else if (type == 1) {
            next_node[source_e] = point_e;
            visited[point_e] = true;
            last = point_e;
            while (!q_end.empty()) q_end.pop();
            while (!q_before.empty()) q_before.pop();
            for (int j = 0; j < n; j++) {
                if (!visited[j]) {
                    int cost = distances[source_e][j] + distances[j][point_e] + instance.getNodeCost(j) - distances[source_e][point_e];
                    q_middle[{source_e, point_e}].push({-cost, j, source_e, point_e});
                }
            }
        } else {
            next_node[source_m] = point_m;
            next_node[point_m] = dest_m;
            visited[point_m] = true;
            q_middle.erase(std::make_pair(source_m, dest_m));
            for (int j = 0; j < n; j++) {
                if (!visited[j]) {
                    int cost1 = distances[source_m][j] + distances[j][point_m] + instance.getNodeCost(j) - distances[source_m][point_m];
                    q_middle[{source_m, point_m}].push({-cost1, j, source_m, point_m});
                    int cost2 = distances[point_m][j] + distances[j][dest_m] + instance.getNodeCost(j) - distances[point_m][dest_m];
                    q_middle[{point_m, dest_m}].push({-cost2, j, point_m, dest_m});
                }
            }
        }
    }

    vector<int> path;
    path.reserve(nodesToPick);
    path.push_back(first);
    for (int i = 0; i < nodesToPick - 1; i++) {
        first = next_node[first];
        path.push_back(first);
    }

    return Solution(path, instance);
}
