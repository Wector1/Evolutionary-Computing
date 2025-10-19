#include "SolverRunner.h"
#include <limits>
#include <numeric>

SolverStatistics SolverRunner::runMultipleTimes(Solver& solver, const TSPAInstance& instance, int numRuns) {
    SolverStatistics stats;
    stats.numRuns = numRuns;
    stats.minCost = std::numeric_limits<double>::max();
    stats.maxCost = 0.0;
    double totalCost = 0.0;
    
    
    for (int i = 0; i < numRuns; i++) {
        Solution solution = solver.solve(instance, i % instance.getTotalNodes());
        double cost = solution.getTotalCost();
        
        totalCost += cost;
        
        if (cost < stats.minCost) {
            stats.minCost = cost;
            stats.bestSolution = solution;
        }
        
        if (cost > stats.maxCost) {
            stats.maxCost = cost;
            stats.worstSolution = solution;
        }
    }
    
    stats.averageCost = totalCost / numRuns;
    
    return stats;
}

SolverStatistics SolverRunner::runAllStartingPoints(Solver& solver, const TSPAInstance& instance) {
    int totalNodes = instance.getTotalNodes();
    return runMultipleTimes(solver, instance, totalNodes);
}
