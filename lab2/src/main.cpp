#include <iostream>
#include <chrono>
#include "problem/TSPAInstance.h"
#include "solvers/RandomSearch.h"
#include "solvers/NearestNeighbour.h"
#include "solvers/NearestNeighbour2Regret.h"
#include "solvers/NearestNeighbourCycle.h"
#include "solvers/kRegretCycle.h"
#include "solvers/kRegret.h"
#include "solvers/kRegretCycleWeighted.h"
#include "solvers/NearestNeighbour2RegretWeighted.h"
#include "utils/SolverRunner.h"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_tspa_file>" << std::endl;
        return 1;
    }

    try {
        std::string tspaFilePath = argv[1];
        
        TSPAInstance tspaInstance(tspaFilePath);
        
        std::cout << "Loaded " << tspaInstance.getTotalNodes() << " nodes from " << tspaFilePath << std::endl;
        std::cout << std::endl;

        NearestNeighbour nearestNeighbour;
        NearestNeighbour2Regret nearestNeighbour2Regret;
        RandomSearch randomSearchSolver;
        kRegretCycle kRegretCycleSolver;
        kRegret kRegretSolver;
        
        std::cout << "\n=== Cycle K-Regret (All Starting Points) ===" << std::endl;
        auto start = std::chrono::high_resolution_clock::now();
        SolverStatistics nncStats = SolverRunner::runAllStartingPoints(kRegretCycleSolver, tspaInstance);
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        std::cout << "Execution time: " << duration.count() << " ms" << std::endl;
        std::cout << "Number of runs: " << nncStats.numRuns << std::endl;
        std::cout << "Best cost: " << nncStats.minCost << std::endl;
        std::cout << "Worst cost: " << nncStats.maxCost << std::endl;
        std::cout << "Average cost: " << nncStats.averageCost << std::endl;
        std::cout << "Best path length: " << nncStats.bestSolution.getPathLength() << std::endl;
        std::cout << "Best path: " << std::endl;
        for (int node : nncStats.bestSolution.getSelectedNodes()) {
            std::cout << node << " ";
        }
        std::cout << std::endl;

        std::cout << "=== Nearest Neighbour (All Starting Points) ===" << std::endl;
        start = std::chrono::high_resolution_clock::now();
        SolverStatistics nnStats = SolverRunner::runAllStartingPoints(nearestNeighbour, tspaInstance);
        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        std::cout << "Execution time: " << duration.count() << " ms" << std::endl;
        std::cout << "Number of runs: " << nnStats.numRuns << std::endl;
        std::cout << "Best cost: " << nnStats.minCost << std::endl;
        std::cout << "Worst cost: " << nnStats.maxCost << std::endl;
        std::cout << "Average cost: " << nnStats.averageCost << std::endl;
        std::cout << "Best path length: " << nnStats.bestSolution.getPathLength() << std::endl;
        std::cout << "Best path: " << std::endl;
        for (int node : nnStats.bestSolution.getSelectedNodes()) {
            std::cout << node << " ";
        }
        std::cout << std::endl;

        std::cout << "=== Nearest Neighbour 2-Regret (All Starting Points) ===" << std::endl;
        start = std::chrono::high_resolution_clock::now();
        SolverStatistics nn2rStats = SolverRunner::runAllStartingPoints(nearestNeighbour2Regret, tspaInstance);
        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        std::cout << "Execution time: " << duration.count() << " ms" << std::endl;
        std::cout << "Number of runs: " << nn2rStats.numRuns << std::endl;
        std::cout << "Best cost: " << nn2rStats.minCost << std::endl;
        std::cout << "Worst cost: " << nn2rStats.maxCost << std::endl;
        std::cout << "Average cost: " << nn2rStats.averageCost << std::endl;
        std::cout << "Best path length: " << nn2rStats.bestSolution.getPathLength() << std::endl;
        std::cout << "Best path: " << std::endl;
        for (int node : nn2rStats.bestSolution.getSelectedNodes()) {
            std::cout << node << " ";
        }
        std::cout << std::endl;

        kRegretCycleWeighted kRegretCycleWeightedSolver;
        std::cout << "\n=== Cycle K-Regret Weighted (All Starting Points) ===" << std::endl;
        start = std::chrono::high_resolution_clock::now();
        SolverStatistics krcwStats = SolverRunner::runAllStartingPoints(kRegretCycleWeightedSolver, tspaInstance);
        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        std::cout << "Execution time: " << duration.count() << " ms" << std::endl;
        std::cout << "Number of runs: " << krcwStats.numRuns << std::endl;
        std::cout << "Best cost: " << krcwStats.minCost << std::endl;
        std::cout << "Worst cost: " << krcwStats.maxCost << std::endl;
        std::cout << "Average cost: " << krcwStats.averageCost << std::endl;
        std::cout << "Best path length: " << krcwStats.bestSolution.getPathLength() << std::endl;
        std::cout << "Best path: " << std::endl;
        for (int node : krcwStats.bestSolution.getSelectedNodes()) {
            std::cout << node << " ";
        }
        std::cout << std::endl;

        NearestNeighbour2RegretWeighted nearestNeighbour2RegretWeighted;
        std::cout << "=== Nearest Neighbour 2-Regret Weighted (All Starting Points) ===" << std::endl;
        start = std::chrono::high_resolution_clock::now();
        SolverStatistics nn2rwStats = SolverRunner::runAllStartingPoints(nearestNeighbour2RegretWeighted, tspaInstance);
        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        std::cout << "Execution time: " << duration.count() << " ms" << std::endl;
        std::cout << "Number of runs: " << nn2rwStats.numRuns << std::endl;
        std::cout << "Best cost: " << nn2rwStats.minCost << std::endl;
        std::cout << "Worst cost: " << nn2rwStats.maxCost << std::endl;
        std::cout << "Average cost: " << nn2rwStats.averageCost << std::endl;
        std::cout << "Best path length: " << nn2rwStats.bestSolution.getPathLength() << std::endl;
        std::cout << "Best path: " << std::endl;
        for (int node : nn2rwStats.bestSolution.getSelectedNodes()) {
            std::cout << node << " ";
        }
        std::cout << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}