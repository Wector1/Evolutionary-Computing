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
#include "utils/SolverRunner.h"
#include "utils/TSPVisualizer.h"

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

        RandomSearch randomSearchSolver;
        NearestNeighbour nearestNeighbour;
        NearestNeighbourAny nearestNeighbourAny;
        NearestNeighbourCycle nearestNeighbourCycle;
        NearestNeighbour2Regret nearestNeighbour2Regret;
        kRegretCycle kRegretCycleSolver;
        kRegret kRegretSolver;
        // NeighbourSteepest neighbourSteepestSolver(NeighbourSteepest::StartingStrategy::Random, NeighbourSteepest::IntraMode::EdgeExchange);
        // SolverStatistics nsStats = SolverRunner::runAllStartingPoints(neighbourSteepestSolver, tspaInstance);
        // printAndVisualize("Neighbour Steepest", nsStats);

        // SolverStatistics rsStats = SolverRunner::runAllStartingPoints(randomSearchSolver, tspaInstance);
        // printAndVisualize("Random Search", rsStats);

        // SolverStatistics nnaStats = SolverRunner::runAllStartingPoints(nearestNeighbourAny, tspaInstance);
        // printAndVisualize("Nearest Neighbour Any", nnaStats);

        // SolverStatistics nnStats = SolverRunner::runAllStartingPoints(nearestNeighbour, tspaInstance);
        // printAndVisualize("Nearest Neighbour Simple", nnStats);

        // SolverStatistics nncCycleStats = SolverRunner::runAllStartingPoints(nearestNeighbourCycle, tspaInstance);
        // printAndVisualize("Nearest Neighbour Cycle", nncCycleStats);

        // SolverStatistics nncStats = SolverRunner::runAllStartingPoints(kRegretCycleSolver, tspaInstance);
        // printAndVisualize("Cycle K-Regret", nncStats);

        // SolverStatistics nn2rStats = SolverRunner::runAllStartingPoints(nearestNeighbour2Regret, tspaInstance);
        // printAndVisualize("Nearest Neighbour 2-Regret", nn2rStats);

        // kRegretCycleWeighted kRegretCycleWeightedSolver;
        // SolverStatistics krcwStats = SolverRunner::runAllStartingPoints(kRegretCycleWeightedSolver, tspaInstance);
        // printAndVisualize("Cycle K-Regret Weighted", krcwStats);

        // NearestNeighbour2RegretWeighted nearestNeighbour2RegretWeighted;
        // SolverStatistics nn2rwStats = SolverRunner::runAllStartingPoints(nearestNeighbour2RegretWeighted, tspaInstance);
        // printAndVisualize("Nearest Neighbour 2-Regret Weighted", nn2rwStats);

        // LocalGreedy LSRandomNodeSolver(LocalGreedy::StartingStrategy::Random, LocalGreedy::IntraMode::NodeExchange);
        // SolverStatistics lgStats = SolverRunner::runAllStartingPoints(LSRandomNodeSolver, tspaInstance);
        // printAndVisualize("Local Greedy Node", lgStats);

        // LocalGreedy LSRandomEdgeSolver(LocalGreedy::StartingStrategy::Random, LocalGreedy::IntraMode::EdgeExchange);
        // SolverStatistics lgEdgeStats = SolverRunner::runAllStartingPoints(LSRandomEdgeSolver, tspaInstance);
        // printAndVisualize("Local Greedy Edge", lgEdgeStats);

        // LocalGreedy LSHeuristicNodeSolver(LocalGreedy::StartingStrategy::Heuristic, LocalGreedy::IntraMode::NodeExchange);
        // SolverStatistics lgHeuNodeStats = SolverRunner::runAllStartingPoints(LSHeuristicNodeSolver, tspaInstance);
        // printAndVisualize("Local Greedy Heuristic Node", lgHeuNodeStats);

        // LocalGreedy LSHeuristicEdgeSolver(LocalGreedy::StartingStrategy::Heuristic, LocalGreedy::IntraMode::EdgeExchange);
        // SolverStatistics lgHeuEdgeStats = SolverRunner::runAllStartingPoints(LSHeuristicEdgeSolver, tspaInstance);
        // printAndVisualize("Local Greedy Heuristic Edge", lgHeuEdgeStats);

        // LocalSteepest LSStRandomNodeSolver(LocalSteepest::StartingStrategy::Random, LocalSteepest::IntraMode::NodeExchange);
        // SolverStatistics lstats = SolverRunner::runAllStartingPoints(LSStRandomNodeSolver, tspaInstance);
        // printAndVisualize("Local Steepest Node", lstats);

        LocalSteepest LSStRandomEdgeSolver(LocalSteepest::StartingStrategy::Random, LocalSteepest::IntraMode::EdgeExchange);
        SolverStatistics lstatsEdge = SolverRunner::runAllStartingPoints(LSStRandomEdgeSolver, tspaInstance);
        printAndVisualize("Local Steepest Edge", lstatsEdge);

        // LocalSteepest LSStHeuristicNodeSolver(LocalSteepest::StartingStrategy::Heuristic, LocalSteepest::IntraMode::NodeExchange);
        // SolverStatistics lstatsHeuNode = SolverRunner::runAllStartingPoints(LSStHeuristicNodeSolver, tspaInstance);
        // printAndVisualize("Local Steepest Heuristic Node", lstatsHeuNode);

        // LocalSteepest LSStHeuristicEdgeSolver(LocalSteepest::StartingStrategy::Heuristic, LocalSteepest::IntraMode::EdgeExchange);
        // SolverStatistics lstatsHeuEdge = SolverRunner::runAllStartingPoints(LSStHeuristicEdgeSolver, tspaInstance);
        // printAndVisualize("Local Steepest Heuristic Edge", lstatsHeuEdge);

        PrevDeltas prevDeltasSolver(PrevDeltas::StartingStrategy::Random);
        SolverStatistics pdStats = SolverRunner::runAllStartingPoints(prevDeltasSolver, tspaInstance);
        printAndVisualize("Prev Deltas", pdStats);

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}