#ifndef TSP_PARSER_H
#define TSP_PARSER_H

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>

struct TSPNode {
    double x;
    double y;
    double cost;
    int id;
};

class TSPParser {
public:
    static std::vector<TSPNode> parseFromCSV(const std::string& filename) {
        std::vector<TSPNode> nodes;
        std::ifstream file(filename);
        
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << filename << std::endl;
            return nodes;
        }

        std::string line;
        int nodeId = 0;
        
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string token;
            TSPNode node;
            
            if (std::getline(ss, token, ';')) {
                node.x = std::stod(token);
            } else {
                continue;
            }
            
            if (std::getline(ss, token, ';')) {
                node.y = std::stod(token);
            } else {
                continue;
            }
            
            if (std::getline(ss, token, ';')) {
                node.cost = std::stod(token);
            } else {
                continue;
            }
            
            node.id = 0; // temp
            
            nodes.push_back(node);
        }
        
        return nodes;
    }
    
    static double calculateTotalDistance(const std::vector<TSPNode>& nodes) {
        double totalDistance = 0.0;
        
        for (size_t i = 0; i < nodes.size(); i++) {
            size_t nextIndex = (i + 1) % nodes.size();
            double dx = nodes[i].x - nodes[nextIndex].x;
            double dy = nodes[i].y - nodes[nextIndex].y;
            totalDistance += int(std::sqrt(dx*dx + dy*dy) + 0.5);
            totalDistance += nodes[i].cost;
        }
        
        return totalDistance;
    }
    
    static std::vector<int> parseSolutionIndices(const std::string& filename) {
        std::vector<int> indices;
        std::ifstream file(filename);
        
        if (!file.is_open()) {
            std::cerr << "Error: Could not open solution file " << filename << std::endl;
            return indices;
        }
        
        std::string line;
        
        while (std::getline(file, line)) {  
            try {
                int index = std::stoi(line);
                indices.push_back(index);
            } catch (const std::exception& e) {
                std::cerr << "Error parsing index from line: " << line << std::endl;
                continue;
            }
        }
        
        return indices;
    }
    
    static std::vector<TSPNode> createOrderedSolution(
            const std::vector<TSPNode>& allNodes, 
            const std::vector<int>& solutionIndices) {
        
        std::vector<TSPNode> orderedNodes;
        
        for (int idx : solutionIndices) {
            // Check if the index is valid
            if (idx >= 0 && idx < static_cast<int>(allNodes.size())) {
                TSPNode node = allNodes[idx];
                node.id = orderedNodes.size();
                orderedNodes.push_back(node);
            } else {
                std::cerr << "Warning: Index " << idx << " out of range. Skipping." << std::endl;
            }
        }
        
        return orderedNodes;
    }
};

#endif