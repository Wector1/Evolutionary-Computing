#include <algorithm>
#include <iterator>
#include <iostream>
#include <fstream>
#include <chrono>
#include <filesystem>
#include "problem/TSPAInstance.h"
#include "solvers/RandomSearch.h"
#include "solvers/NearestNeighbour.h"
#include "solvers/NearestNeighbourAny.h"
#include "solvers/NearestNeighbour2Regret.h"
#include "solvers/NearestNeighbourCycle.h"
#include "solvers/kRegretCycle.h"
#include "solvers/kRegret.h"
#include "solvers/kRegretCycleWeighted.h"
#include "solvers/NearestNeighbour2RegretWeighted.h"
#include "solvers/LocalGreedy.h"
#include "solvers/LocalSteepest.h"
#include "solvers/NeighbourSteepest.h"
#include "solvers/PrevDeltas.h"
#include "solvers/ILS.h"
#include "utils/SolverRunner.h"
#include "utils/TSPVisualizer.h"
#include <set>
#include <numeric>
#include <cmath>

struct SolutionRepresentation {
    std::vector<bool> nodes;
    std::set<std::pair<int, int>> edges; // Store edges as pairs (u, v) with u < v for uniqueness

    SolutionRepresentation(const Solution& sol, int totalNodes) {
        nodes.resize(totalNodes, false);

        const auto& path = sol.getSelectedNodes();
        if (path.empty()) return;

        for (size_t i = 0; i < path.size(); ++i) {
            int u = path[i];
            int v = path[(i + 1) % path.size()];
            
            nodes[u] = true;
            
            edges.insert(u < v ? std::make_pair(u, v) : std::make_pair(v, u));
        }
    }
};

double calculateCorrelation(const std::vector<double>& x, const std::vector<double>& y) {
    if (x.size() != y.size() || x.empty()) return 0.0;
    double n = static_cast<double>(x.size());
    double sum_x = std::accumulate(x.begin(), x.end(), 0.0);
    double sum_y = std::accumulate(y.begin(), y.end(), 0.0);
    double sum_xy = 0.0;
    double sum_x2 = 0.0;
    double sum_y2 = 0.0;
    for (size_t i = 0; i < n; ++i) {
        sum_xy += x[i] * y[i];
        sum_x2 += x[i] * x[i];
        sum_y2 += y[i] * y[i];
    }
    double numerator = n * sum_xy - sum_x * sum_y;
    double denominator = std::sqrt((n * sum_x2 - sum_x * sum_x) * (n * sum_y2 - sum_y * sum_y));
    return (denominator == 0) ? 0.0 : numerator / denominator;
}

void saveScatterPlot(const std::vector<double>& x, const std::vector<double>& y, 
                     const std::string& title, const std::string& filename, 
                     const std::string& xlabel_text, const std::string& ylabel_text) {
    auto f = matplot::figure(true);
    f->size(800, 600);
    matplot::scatter(x, y);
    matplot::title(title);
    matplot::xlabel(xlabel_text);
    matplot::ylabel(ylabel_text);
    matplot::save("solutions/images/" + filename);
    std::cout << "Saved plot: " << filename << std::endl;
}

std::pair<double, double> calculateSimilarity(const SolutionRepresentation& a, const SolutionRepresentation& b) {
    // 1. Node Similarity: Intersection / Union
    int commonNodes = 0;
    int unionNodes = 0;
    for (size_t i = 0; i < a.nodes.size(); ++i) {
        bool inA = a.nodes[i];
        bool inB = b.nodes[i];
        if (inA && inB) commonNodes++;
        if (inA || inB) unionNodes++;
    }
    double nodeSim = (unionNodes == 0) ? 0.0 : static_cast<double>(commonNodes) / unionNodes;

    // 2. Edge Similarity: Intersection / Union
    std::vector<std::pair<int, int>> intersection;
    std::set_intersection(
        a.edges.begin(), a.edges.end(),
        b.edges.begin(), b.edges.end(),
        std::back_inserter(intersection)
    );

    int commonEdgesCount = intersection.size();
    // Union size = |A| + |B| - |Intersection|
    int unionEdgesCount = a.edges.size() + b.edges.size() - commonEdgesCount;
    double edgeSim = (unionEdgesCount == 0) ? 0.0 : static_cast<double>(commonEdgesCount) / unionEdgesCount;

    return {nodeSim, edgeSim};
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_tspa_file> [--visualize]" << std::endl;
        return 1;
    }

    try {
        std::string tspaFilePath = argv[1];
        bool enableVisualization = (argc > 2 && std::string(argv[2]) == "--visualize");
        
        // Extract instance name from path
        std::string instanceName = std::filesystem::path(tspaFilePath).stem().string();
        
        std::filesystem::create_directories("solutions/paths");
        if (enableVisualization) {
            std::filesystem::create_directories("solutions/images");
        }
        
        TSPAInstance tspaInstance(tspaFilePath);
        
        std::cout << "Loaded " << tspaInstance.getTotalNodes() << " nodes from " << tspaFilePath << std::endl;
        std::cout << std::endl;

        // Helper function to print stats and optionally visualize
        auto printAndVisualize = [&](const std::string& solverName, const SolverStatistics& stats) {
            std::cout << "=== " << solverName << " ===" << std::endl;
            std::cout << "Total execution time (ms): " << stats.totalRuntimeMs << std::endl;
            std::cout << "Min run time (ms): " << stats.minRuntimeMs << std::endl;
            std::cout << "Max run time (ms): " << stats.maxRuntimeMs << std::endl;
            std::cout << "Average run time (ms): " << stats.averageRuntimeMs << std::endl;
            std::cout << "Number of runs: " << stats.numRuns << std::endl;
            std::cout << "Best cost: " << stats.minCost << std::endl;
            std::cout << "Worst cost: " << stats.maxCost << std::endl;
            std::cout << "Average cost: " << stats.averageCost << std::endl;
            std::cout << "Best path length: " << stats.bestSolution.getPathLength() << std::endl;
            std::cout << "Best path: " << std::endl;
            for (int node : stats.bestSolution.getSelectedNodes()) {
                std::cout << node << " ";
            }
            std::cout << std::endl;
            
            std::string cleanName = solverName;
            std::replace(cleanName.begin(), cleanName.end(), ' ', '_');
            std::replace(cleanName.begin(), cleanName.end(), '(', '_');
            std::replace(cleanName.begin(), cleanName.end(), ')', '_');
            
            // Save best path to text file (always)
            std::string pathFile = "solutions/paths/" + cleanName + "_" + instanceName + ".txt";
            std::ofstream pathOut(pathFile);
            if (pathOut.is_open()) {
                pathOut << "Solver: " << solverName << std::endl;
                pathOut << "Instance: " << instanceName << std::endl;
                pathOut << "Total Cost: " << stats.minCost << std::endl;
                pathOut << "Path Length: " << stats.bestSolution.getPathLength() << std::endl;
                pathOut << "Total Runtime (ms): " << stats.totalRuntimeMs << std::endl;
                pathOut << "Average Runtime (ms): " << stats.averageRuntimeMs << std::endl;
                pathOut << "Number of Runs: " << stats.numRuns << std::endl;
                pathOut << std::endl;
                pathOut << "Best Path:" << std::endl;
                for (int node : stats.bestSolution.getSelectedNodes()) {
                    pathOut << node << " ";
                }
                pathOut << std::endl;
                pathOut.close();
                std::cout << "Path saved to: " << pathFile << std::endl;
            }
            
            if (enableVisualization) {
                std::string outputFile = "solutions/images/" + cleanName + "_" + instanceName + ".png";
                TSPVisualizer::visualize(stats.bestSolution, tspaInstance, 
                    solverName + " - " + instanceName, outputFile);
                }
            };
            
        LocalGreedy LGRandomSolver(LocalGreedy::StartingStrategy::Random, LocalGreedy::IntraMode::EdgeExchange);
        SolverStatistics lgrStats = SolverRunner::runMultipleTimes(LGRandomSolver, tspaInstance, 1000);
        // printAndVisualize("Local Greedy Random", lgrStats);
        Solution goodSol = lgrStats.bestSolution;
        std::vector<Solution> allLGRandom = lgrStats.allSolutions;
        
        // LocalSteepest LSStRandomEdgeSolver(LocalSteepest::StartingStrategy::Heuristic, LocalSteepest::IntraMode::EdgeExchange);
        // SolverStatistics lstatsEdge = SolverRunner::runAllStartingPoints(LSStRandomEdgeSolver, tspaInstance);
        // printAndVisualize("Local Steepest Edge", lstatsEdge);
        // Solution veryGoodSol = lstatsEdge.bestSolution;
        ILS ILSSolver(ILS::StartingStrategy::Random, ILS::IntraMode::EdgeExchange, 17000, 15);
        SolverStatistics lstatsILSA = SolverRunner::runMultipleTimes(ILSSolver, tspaInstance, 1);
        Solution veryGoodSol = lstatsILSA.bestSolution;
        std::cout << "Good Solution Cost (from Local Greedy Random): " << goodSol.getTotalCost() << std::endl;
        std::cout << "Very Good Solution Cost (from ILS): " << veryGoodSol.getTotalCost() << std::endl;

        std::vector<SolutionRepresentation> representations;
        for (const auto& sol : allLGRandom) {
            representations.emplace_back(sol, tspaInstance.getTotalNodes());
        }
        SolutionRepresentation veryGoodRep(veryGoodSol, tspaInstance.getTotalNodes());
        SolutionRepresentation goodRep(goodSol, tspaInstance.getTotalNodes());

        // Prepare data vectors
        std::vector<double> objectives;
        std::vector<double> avg_sim_nodes, avg_sim_edges;
        std::vector<double> good_sim_nodes, good_sim_edges; // For goodSol
        std::vector<double> good_obj_filtered; // For goodSol (filtered)
        std::vector<double> very_good_sim_nodes, very_good_sim_edges;

        // Find index of goodSol in allLGRandom to exclude it strictly
        int bestIdx = -1;
        double minCost = std::numeric_limits<double>::max();
        for(size_t i=0; i<allLGRandom.size(); ++i) {
            if(allLGRandom[i].getTotalCost() < minCost) {
                minCost = allLGRandom[i].getTotalCost();
                bestIdx = i;
            }
        }

        // Calculate Average Similarity to all others
        std::cout << "Calculating average similarities..." << std::endl;
        for (size_t i = 0; i < representations.size(); ++i) {
            double sumNodeSim = 0.0;
            double sumEdgeSim = 0.0;
            int count = 0;
            
            for (size_t j = 0; j < representations.size(); ++j) {
                if (i == j) continue;
                
                auto [nodeSim, edgeSim] = calculateSimilarity(representations[i], representations[j]);
                sumNodeSim += nodeSim;
                sumEdgeSim += edgeSim;
                count++;
            }
            
            double avgNode = (count > 0) ? sumNodeSim / count : 0.0;
            double avgEdge = (count > 0) ? sumEdgeSim / count : 0.0;
            
            avg_sim_nodes.push_back(avgNode);
            avg_sim_edges.push_back(avgEdge);
        }

        // Collect all data
        for (size_t i = 0; i < representations.size(); ++i) {
            double cost = allLGRandom[i].getTotalCost();
            objectives.push_back(cost);

            // 2. Similarity to Very Good
            auto [vgNode, vgEdge] = calculateSimilarity(representations[i], veryGoodRep);
            very_good_sim_nodes.push_back(vgNode);
            very_good_sim_edges.push_back(vgEdge);

            // 3. Similarity to Good (goodSol)
            if (static_cast<int>(i) != bestIdx) {
                auto [gNode, gEdge] = calculateSimilarity(representations[i], goodRep);
                good_sim_nodes.push_back(gNode);
                good_sim_edges.push_back(gEdge);
                good_obj_filtered.push_back(cost);
            }
        }

        // Calculate Correlations
        double corr_avg_node = calculateCorrelation(objectives, avg_sim_nodes);
        double corr_avg_edge = calculateCorrelation(objectives, avg_sim_edges);
        double corr_good_node = calculateCorrelation(good_obj_filtered, good_sim_nodes);
        double corr_good_edge = calculateCorrelation(good_obj_filtered, good_sim_edges);
        double corr_vg_node = calculateCorrelation(objectives, very_good_sim_nodes);
        double corr_vg_edge = calculateCorrelation(objectives, very_good_sim_edges);

        std::cout << "\nCorrelation Coefficients (Objective vs Similarity):" << std::endl;
        std::cout << "Avg Node Sim: " << corr_avg_node << std::endl;
        std::cout << "Avg Edge Sim: " << corr_avg_edge << std::endl;
        std::cout << "Good Node Sim: " << corr_good_node << std::endl;
        std::cout << "Good Edge Sim: " << corr_good_edge << std::endl;
        std::cout << "Very Good Node Sim: " << corr_vg_node << std::endl;
        std::cout << "Very Good Edge Sim: " << corr_vg_edge << std::endl;

        // Generate Plots
        if (enableVisualization) {
            saveScatterPlot(objectives, avg_sim_nodes, 
                "Avg Node Similarity vs Cost (Corr: " + std::to_string(corr_avg_node) + ")", 
                "avg_node_sim_" + instanceName + ".png", "Cost", "Similarity");
                
            saveScatterPlot(objectives, avg_sim_edges, 
                "Avg Edge Similarity vs Cost (Corr: " + std::to_string(corr_avg_edge) + ")", 
                "avg_edge_sim_" + instanceName + ".png", "Cost", "Similarity");

            saveScatterPlot(good_obj_filtered, good_sim_nodes, 
                "Good Node Similarity vs Cost (Corr: " + std::to_string(corr_good_node) + ")", 
                "good_node_sim_" + instanceName + ".png", "Cost", "Similarity");

            saveScatterPlot(good_obj_filtered, good_sim_edges, 
                "Good Edge Similarity vs Cost (Corr: " + std::to_string(corr_good_edge) + ")", 
                "good_edge_sim_" + instanceName + ".png", "Cost", "Similarity");

            saveScatterPlot(objectives, very_good_sim_nodes, 
                "Very Good Node Similarity vs Cost (Corr: " + std::to_string(corr_vg_node) + ")", 
                "vg_node_sim_" + instanceName + ".png", "Cost", "Similarity");

            saveScatterPlot(objectives, very_good_sim_edges, 
                "Very Good Edge Similarity vs Cost (Corr: " + std::to_string(corr_vg_edge) + ")", 
                "vg_edge_sim_" + instanceName + ".png", "Cost", "Similarity");
        }


    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}