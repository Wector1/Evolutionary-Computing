#ifndef TSPAINSTANCE_H
#define TSPAINSTANCE_H

#include <vector>
#include <string>
#include <utility>
#include <cmath>
#include <algorithm>
#include <stdexcept>

struct Node {
    int x, y, cost;
    Node(int x, int y, int cost) : x(x), y(y), cost(cost) {}
};

class TSPAInstance {
public:
    TSPAInstance(const std::string& filename);
    const std::vector<std::vector<int>>& getDistanceMatrix() const;
    const std::vector<Node>& getNodes() const;
    int getTotalNodes() const;
    int getNodeCost(int nodeIndex) const;
    int getNodeX(int nodeIndex) const;
    int getNodeY(int nodeIndex) const;

private:
    std::vector<Node> nodes;
    std::vector<std::vector<int>> distanceMatrix;

    void readTSPAFile(const std::string& filename);
    void calculateDistanceMatrix();
};

#endif