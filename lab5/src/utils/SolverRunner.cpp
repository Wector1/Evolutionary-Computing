#include "SolverRunner.h"
#include <limits>
#include <numeric>
#include <chrono>

SolverStatistics SolverRunner::runMultipleTimes(Solver& solver, const TSPAInstance& instance, int numRuns) {
    SolverStatistics stats;
    stats.numRuns = numRuns;
    stats.minCost = std::numeric_limits<double>::max();
    stats.maxCost = 0.0;
    double totalCost = 0.0;
    stats.totalRuntimeMs = 0.0;
    stats.minRuntimeMs = std::numeric_limits<double>::max();
    stats.maxRuntimeMs = 0.0;

    for (int i = 0; i < numRuns; i++) {
        auto runStart = std::chrono::high_resolution_clock::now();
        Solution solution = solver.solve(instance, i % instance.getTotalNodes());
        auto runEnd = std::chrono::high_resolution_clock::now();

        double cost = solution.getTotalCost();
        double elapsedMs = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(runEnd - runStart).count();

        totalCost += cost;

        if (cost < stats.minCost) {
            stats.minCost = cost;
            stats.bestSolution = solution;
        }

        if (cost > stats.maxCost) {
            stats.maxCost = cost;
            stats.worstSolution = solution;
        }

        stats.totalRuntimeMs += elapsedMs;
        if (elapsedMs < stats.minRuntimeMs) stats.minRuntimeMs = elapsedMs;
        if (elapsedMs > stats.maxRuntimeMs) stats.maxRuntimeMs = elapsedMs;
    }

    if (numRuns > 0) {
        stats.averageCost = totalCost / numRuns;
        stats.averageRuntimeMs = stats.totalRuntimeMs / numRuns;
    }
    return stats;
}

SolverStatistics SolverRunner::runAllStartingPoints(Solver& solver, const TSPAInstance& instance) {
    int totalNodes = instance.getTotalNodes();
    return runMultipleTimes(solver, instance, totalNodes);
}
