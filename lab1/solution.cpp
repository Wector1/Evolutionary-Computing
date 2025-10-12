#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <queue>
#include <random>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace std;

#define INF 1000000000

template <class result_t = chrono::milliseconds,
          class clock_t = chrono::steady_clock,
          class duration_t = chrono::milliseconds>
auto since(chrono::time_point<clock_t, duration_t> const &start) {
  return chrono::duration_cast<result_t>(clock_t::now() - start);
}

vector<string> split(const string &str, char delimiter) {
  vector<string> tokens;
  stringstream ss(str);
  string token;

  while (getline(ss, token, delimiter)) {
    tokens.push_back(token);
  }

  return tokens;
}

void read_data(ifstream &file, vector<pair<int, int>> &points,
               vector<int> &costs) {

  string line;
  while (getline(file, line)) {
    vector<string> fields = split(line, ';');

    int x = stoi(fields[0]);
    int y = stoi(fields[1]);
    int cost = stoi(fields[2]);

    points.push_back(make_pair(x, y));
    costs.push_back(cost);
  }
}

int calculate_distance(const pair<int, int> &p1, const pair<int, int> &p2) {
  return int(
      sqrt(((float)p1.first - p2.first) * ((float)p1.first - p2.first) +
           ((float)p1.second - p2.second) * ((float)p1.second - p2.second)) +
      0.5);
}

void calculate_distances(const vector<pair<int, int>> &points,
                         vector<vector<int>> &distanecs) {
  for (int i = 0; i < points.size(); i++) {
    for (int j = 0; j < points.size(); j++) {
      distanecs[i][j] = calculate_distance(points[i], points[j]);
    }
  }
}

long long calculate_path_cost(const vector<vector<int>> &distanecs,
                              const vector<int> &costs,
                              const vector<int> &path) {
  long long path_cost{};

  for (int i = 0; i < path.size(); i++) {
    path_cost +=
        (distanecs[path[i]][path[(i + 1) % path.size()]] + costs[path[i]]);
  }

  return path_cost;
}

vector<int> draw_random_points(int n) {
  vector<int> sequence(n);
  for (int i = 0; i < n; ++i) {
    sequence[i] = i;
  }

  random_device rd;
  mt19937 gen(rd());

  shuffle(sequence.begin(), sequence.end(), gen);

  int pointsToDraw = n / 2;
  vector<int> result(sequence.begin(), sequence.begin() + pointsToDraw);

  return result;
}

void print_path(const vector<int> &path, const vector<vector<int>> &distances,
                const vector<int> &costs) {
  for (int i = 0; i < path.size(); i++) {
    cout << path[i] << endl;
  }
}

vector<int> nearest_neighbour(const vector<vector<int>> &distances,
                              const vector<int> &costs, int point) {
  vector<int> path;
  vector<bool> visited(costs.size());

  for (int i = 0; i < costs.size() / 2; i++) {
    visited[point] = true;
    path.push_back(point);
    int lowest_cost_point = 1;
    int lowest_cost = INF;
    for (int j = 0; j < costs.size(); j++) {
      if (!visited[j] && distances[point][j] + costs[j] < lowest_cost) {
        lowest_cost = distances[point][j] + costs[j];
        lowest_cost_point = j;
      }
    }
    point = lowest_cost_point;
  }

  return path;
}

vector<int> nearest_neighbour_2(const vector<vector<int>> &distances,
                                const vector<int> &costs, int point) {
  int original_point = point;
  vector<bool> visited(costs.size());
  vector<int> next_node(costs.size(), -1);
  priority_queue<tuple<int, int, int>> q_before;
  priority_queue<tuple<int, int, int>> q_end;
  map<pair<int, int>, priority_queue<tuple<int, int, int, int>>> q_middle;
  int first = point, last = point;

  for (int j = 0; j < costs.size(); j++) {
    if (j != point) {
      int cost = distances[point][j] + costs[j];
      q_before.push({-cost, j, point});
    }
  }
  for (int j = 0; j < costs.size(); j++) {
    if (j != point) {
      int cost = distances[point][j] + costs[j];
      q_end.push({-cost, j, point});
    }
  }
  visited[point] = true;

  for (int i = 0; i < costs.size() / 2 - 1; i++) {
    int old_point = point;
    int cost_b = INF, cost_m = INF, cost_e = INF;
    int point_b = -1, point_m = -1, point_e = -1;
    int source_b = -1, source_e = -1;
    int source_m = -1, dest_m = -1;
    int type = -1;
    while (!q_before.empty()) {
      auto [cost_local, point_local, source_local] = q_before.top();
      if (source_local != first or visited[point_local]) {
        q_before.pop();
        continue;
      }
      cost_b = -cost_local;
      point_b = point_local;
      source_b = source_local;
      break;
    }
    while (!q_end.empty()) {
      auto [cost_local, point_local, source_local] = q_end.top();
      if (source_local != last or visited[point_local]) {
        q_end.pop();
        continue;
      }
      cost_e = -cost_local;
      point_e = point_local;
      source_e = source_local;
      break;
    }
    for (auto &[key, value] : q_middle) {
      if (last == 113) {
        auto &[p1, p2] = key;
        auto [cost_local, point_local, source_local, dest_local] = value.top();
      }
      while (!value.empty()) {
        auto [cost_local, point_local, source_local, dest_local] = value.top();
        if (next_node[source_local] != dest_local or visited[point_local]) {
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
      while (!q_before.empty()) {
        q_before.pop();
      }
      for (int j = 0; j < costs.size(); j++) {
        if (!visited[j]) {
          int cost = distances[point_b][j] + costs[j];
          q_before.push({-cost, j, point_b});
        }
      }
      for (int j = 0; j < costs.size(); j++) {
        if (!visited[j]) {
          int cost = distances[point_b][j] + distances[j][source_b] + costs[j] -
                     distances[point_b][source_b];
          q_middle[{point_b, source_b}].push({-cost, j, point_b, source_b});
        }
      }
    } else if (type == 1) {
      next_node[source_e] = point_e;
      visited[point_e] = true;
      last = point_e;
      while (!q_end.empty()) {
        q_end.pop();
      }
      for (int j = 0; j < costs.size(); j++) {
        if (!visited[j]) {
          int cost = distances[point_e][j] + costs[j];
          q_end.push({-cost, j, point_e});
        }
      }
      for (int j = 0; j < costs.size(); j++) {
        if (!visited[j]) {
          int cost = distances[source_e][j] + distances[j][point_e] + costs[j] -
                     distances[source_e][point_e];
          q_middle[{source_e, point_e}].push({-cost, j, source_e, point_e});
        }
      }
    } else {
      next_node[source_m] = point_m;
      next_node[point_m] = dest_m;
      visited[point_m] = true;
      q_middle.erase(make_pair(source_m, dest_m));
      for (int j = 0; j < costs.size(); j++) {
        if (!visited[j]) {
          int cost = distances[source_m][j] + distances[j][point_m] + costs[j] -
                     distances[source_m][point_m];
          q_middle[{source_m, point_m}].push({-cost, j, source_m, point_m});
          cost = distances[point_m][j] + distances[j][dest_m] + costs[j] -
                 distances[point_m][dest_m];
          q_middle[{point_m, dest_m}].push({-cost, j, point_m, dest_m});
        }
      }
    }
  }
  vector<int> path;
  path.push_back(first);
  for (int i = 0; i < costs.size() / 2 - 1; i++) {
    path.push_back(first = next_node[first]);
  }

  return path;
}

vector<int> nearest_neighbour_cycle(const vector<vector<int>> &distances,
                                    const vector<int> &costs, int point) {
  int original_point = point;
  vector<bool> visited(costs.size());
  vector<int> next_node(costs.size(), -1);
  priority_queue<tuple<int, int, int>> q_before;
  priority_queue<tuple<int, int, int>> q_end;
  map<pair<int, int>, priority_queue<tuple<int, int, int, int>>> q_middle;
  int first = point, last = point;

  for (int j = 0; j < costs.size(); j++) {
    if (j != point) {
      int cost = distances[point][j] + costs[j];
      q_before.push({-cost, j, point});
    }
  }
  for (int j = 0; j < costs.size(); j++) {
    if (j != point) {
      int cost = distances[point][j] + costs[j];
      q_end.push({-cost, j, point});
    }
  }
  visited[point] = true;

  for (int i = 0; i < costs.size() / 2 - 1; i++) {
    int old_point = point;
    int cost_b = INF, cost_m = INF, cost_e = INF;
    int point_b = -1, point_m = -1, point_e = -1;
    int source_b = -1, source_e = -1;
    int source_m = -1, dest_m = -1;
    int type = -1;
    while (!q_before.empty()) {
      auto [cost_local, point_local, source_local] = q_before.top();
      if (source_local != first or visited[point_local]) {
        q_before.pop();
        continue;
      }
      cost_b = -cost_local;
      point_b = point_local;
      source_b = source_local;
      break;
    }
    while (!q_end.empty()) {
      auto [cost_local, point_local, source_local] = q_end.top();
      if (source_local != last or visited[point_local]) {
        q_end.pop();
        continue;
      }
      cost_e = -cost_local;
      point_e = point_local;
      source_e = source_local;
      break;
    }
    for (auto &[key, value] : q_middle) {
      if (last == 113) {
        auto &[p1, p2] = key;
        auto [cost_local, point_local, source_local, dest_local] = value.top();
      }
      while (!value.empty()) {
        auto [cost_local, point_local, source_local, dest_local] = value.top();
        if (next_node[source_local] != dest_local or visited[point_local]) {
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
      if (point_m == 133) {
      }
      type = 2;
    }

    if (type == 0) {
      next_node[point_b] = source_b;
      visited[point_b] = true;
      first = point_b;
      while (!q_before.empty()) {
        q_before.pop();
      }
      while (!q_end.empty()) {
        q_end.pop();
      }
      for (int j = 0; j < costs.size(); j++) {
        if (!visited[j]) {
          int cost = distances[point_b][j] + distances[j][source_b] + costs[j] -
                     distances[point_b][source_b];
          q_middle[{point_b, source_b}].push({-cost, j, point_b, source_b});
        }
      }
    } else if (type == 1) {
      next_node[source_e] = point_e;
      visited[point_e] = true;
      last = point_e;
      while (!q_end.empty()) {
        q_end.pop();
      }
      while (!q_before.empty()) {
        q_before.pop();
      }
      for (int j = 0; j < costs.size(); j++) {
        if (!visited[j]) {
          int cost = distances[source_e][j] + distances[j][point_e] + costs[j] -
                     distances[source_e][point_e];
          q_middle[{source_e, point_e}].push({-cost, j, source_e, point_e});
        }
      }
    } else {
      next_node[source_m] = point_m;
      next_node[point_m] = dest_m;
      visited[point_m] = true;
      q_middle.erase(make_pair(source_m, dest_m));
      for (int j = 0; j < costs.size(); j++) {
        if (!visited[j]) {
          int cost = distances[source_m][j] + distances[j][point_m] + costs[j] -
                     distances[source_m][point_m];
          q_middle[{source_m, point_m}].push({-cost, j, source_m, point_m});
          cost = distances[point_m][j] + distances[j][dest_m] + costs[j] -
                 distances[point_m][dest_m];
          q_middle[{point_m, dest_m}].push({-cost, j, point_m, dest_m});
        }
      }
    }
  }
  vector<int> path;
  path.push_back(first);
  for (int i = 0; i < costs.size() / 2 - 1; i++) {
    path.push_back(first = next_node[first]);
  }

  return path;
}

vector<int> check_all_possible_nn(const vector<vector<int>> &distances,
                                  const vector<int> &costs) {
  vector<int> best_path;
  int best_cost = INF;
  long long path_costs_sum{};
  int worst_cost = 0;
  auto start = chrono::steady_clock::now();
  for (int i = 0; i < costs.size(); i++) {
    auto path = nearest_neighbour(distances, costs, i);
    auto cost = calculate_path_cost(distances, costs, path);
    if (cost < best_cost) {
      best_cost = cost;
      best_path = path;
    }
    path_costs_sum += cost;
    if (cost > worst_cost) {
      worst_cost = cost;
    }
  }
  cout << "Nearest neighbor, at the end only:" << endl;
  cout << "Min: " << best_cost
       << ", average: " << (double)path_costs_sum / costs.size()
       << ", max: " << worst_cost << endl;
  cout << "Elapsed(ms)=" << since(start).count() << endl;
  return best_path;
}

vector<int> check_all_possible_nn_2(const vector<vector<int>> &distances,
                                    const vector<int> &costs) {
  vector<int> best_path;
  int best_cost = INF;
  long long path_costs_sum{};
  int worst_cost = 0;
  auto start = chrono::steady_clock::now();
  for (int i = 0; i < costs.size(); i++) {
    auto path = nearest_neighbour_2(distances, costs, i);
    auto cost = calculate_path_cost(distances, costs, path);
    if (cost < best_cost) {
      best_cost = cost;
      best_path = path;
    }
    path_costs_sum += cost;
    if (cost > worst_cost) {
      worst_cost = cost;
    }
  }
  cout << "Nearest neighbor, at all possible position:" << endl;
  cout << "Min: " << best_cost
       << ", average: " << (double)path_costs_sum / costs.size()
       << ", max: " << worst_cost << endl;
  cout << "Elapsed(ms)=" << since(start).count() << endl;
  return best_path;
}

vector<int> check_all_possible_cycle(const vector<vector<int>> &distances,
                                     const vector<int> &costs) {
  vector<int> best_path;
  int best_cost = INF;
  long long path_costs_sum{};
  int worst_cost = 0;
  auto start = chrono::steady_clock::now();
  for (int i = 0; i < costs.size(); i++) {
    auto path = nearest_neighbour_cycle(distances, costs, i);
    auto cost = calculate_path_cost(distances, costs, path);
    if (cost < best_cost) {
      best_cost = cost;
      best_path = path;
    }
    path_costs_sum += cost;
    if (cost > worst_cost) {
      worst_cost = cost;
    }
  }
  cout << "Greedy cycle:" << endl;
  cout << "Min: " << best_cost
       << ", average: " << (double)path_costs_sum / costs.size()
       << ", max: " << worst_cost << endl;
  cout << "Elapsed(ms)=" << since(start).count() << endl;

  return best_path;
}

vector<int> check_all_random(const vector<vector<int>> &distances,
                             const vector<int> &costs) {

  vector<int> best_path;
  int best_cost = INF;
  long long path_costs_sum{};
  int worst_cost = 0;
  auto start = chrono::steady_clock::now();
  for (int i = 0; i < costs.size(); i++) {
    vector<int> random_path = draw_random_points(costs.size());
    long long path_cost = calculate_path_cost(distances, costs, random_path);
    path_costs_sum += path_cost;
    if (path_cost < best_cost) {
      best_cost = path_cost;
      best_path = random_path;
    }
    if (path_cost > worst_cost) {
      worst_cost = path_cost;
    }
  }
  cout << "Random:" << endl;
  cout << "Min: " << best_cost
       << ", average: " << (double)path_costs_sum / costs.size()
       << ", max: " << worst_cost << endl;
  cout << "Elapsed(ms)=" << since(start).count() << endl;
  return best_path;
}

void solve(string filename) {
  cout << "=== " << filename << " ===" << endl;
  ifstream file(filename);

  vector<pair<int, int>> points;
  vector<int> costs;

  read_data(file, points, costs);

  vector<vector<int>> distances(points.size(), vector<int>(points.size()));
  calculate_distances(points, distances);

  check_all_random(distances, costs);
  check_all_possible_nn(distances, costs);
  check_all_possible_nn_2(distances, costs);
  check_all_possible_cycle(distances, costs);
}

int main() {
  solve("../problem/TSPA.csv");
  solve("../problem/TSPB.csv");
  return 0;
}
