#include "tsp_parser.h"
#include "tsp_visualizer.h"
#include <iostream>
#include <string>

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <instance_file.csv> <solution_indices.csv> [output_image.png]" << std::endl;
        return 1;
    }
    
    std::string instanceFile = argv[1];
    std::string solutionFile = argv[2];
    std::string outputFile = (argc > 3) ? argv[3] : "";
    
    std::vector<TSPNode> allNodes = TSPParser::parseFromCSV(instanceFile);
    
    if (allNodes.empty()) {
        std::cerr << "No valid data found in the instance file." << std::endl;
        return 1;
    }
    
    std::cout << "Loaded " << allNodes.size() << " nodes from instance " << instanceFile << std::endl;
    
    std::vector<int> solutionIndices = TSPParser::parseSolutionIndices(solutionFile);
    
    if (solutionIndices.empty()) {
        std::cerr << "No valid solution indices found in " << solutionFile << std::endl;
        return 1;
    }
    
    std::cout << "Loaded " << solutionIndices.size() << " solution indices from " << solutionFile << std::endl;
    
    std::vector<TSPNode> solutionNodes = TSPParser::createOrderedSolution(allNodes, solutionIndices);
    
    if (solutionNodes.empty()) {
        std::cerr << "Failed to create solution from indices." << std::endl;
        return 1;
    }
    
    std::cout << "Created solution with " << solutionNodes.size() << " nodes" << std::endl;
    std::cout << "Total tour distance: " << TSPParser::calculateTotalDistance(solutionNodes) << std::endl;
    
    // Create title with both instance and solution file
    std::string instanceName = instanceFile.substr(instanceFile.find_last_of("/") + 1);
    std::string solutionName = solutionFile.substr(solutionFile.find_last_of("/") + 1);
    std::string title = "TSP: " + instanceName + " (Solution: " + solutionName + ")";
    
    // Visualize the solution, showing all nodes with solution highlighted
    TSPVisualizer visualizer(solutionNodes, allNodes, title);
    visualizer.visualize(outputFile);
    
    return 0;
}