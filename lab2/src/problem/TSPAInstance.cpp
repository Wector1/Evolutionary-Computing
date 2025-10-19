#include "TSPAInstance.h"
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <iostream>

TSPAInstance::TSPAInstance(const std::string& filename) {
    readTSPAFile(filename);
    calculateDistanceMatrix();
}

void TSPAInstance::readTSPAFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open TSPA file: " + filename);
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string token;
        int x, y, cost;
        if (std::getline(iss, token, ';') && !token.empty()) {
            x = std::stoi(token);
            if (std::getline(iss, token, ';') && !token.empty()) {
                y = std::stoi(token);
                if (std::getline(iss, token, ';') && !token.empty()) {
                    cost = std::stoi(token);
                    nodes.emplace_back(x, y, cost);
                }
            }
        }
    }
    file.close();
    
    if (nodes.empty()) {
        throw std::runtime_error("No nodes read from file: " + filename);
    }
}

void TSPAInstance::calculateDistanceMatrix() {
    int n = nodes.size();
    distanceMatrix.resize(n, std::vector<int>(n, 0));

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i != j) {
                double dx = nodes[i].x - nodes[j].x;
                double dy = nodes[i].y - nodes[j].y;
                distanceMatrix[i][j] = static_cast<int>(std::round(std::sqrt(dx * dx + dy * dy)));
            }
        }
    }
}

const std::vector<Node>& TSPAInstance::getNodes() const {
    return nodes;
}

const std::vector<std::vector<int>>& TSPAInstance::getDistanceMatrix() const {
    return distanceMatrix;
}

int TSPAInstance::getTotalNodes() const {
    return nodes.size();
}

int TSPAInstance::getNodeCost(int nodeIndex) const {
    if (nodeIndex >= 0 && nodeIndex < static_cast<int>(nodes.size())) {
        return nodes[nodeIndex].cost;
    }
    return 0;
}