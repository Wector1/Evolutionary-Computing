#ifndef SOLVERRUNNER_H
#define SOLVERRUNNER_H

#include "Solver.h"
#include "TSPAInstance.h"
#include "Solution.h"
#include <vector>

struct SolverStatistics {
    Solution bestSolution;
    Solution worstSolution;
    std::vector<Solution> allSolutions;
    double averageCost;
    double minCost;
    double maxCost;
    int numRuns;
    double totalRuntimeMs = 0.0;
    double averageRuntimeMs = 0.0;
    double minRuntimeMs = 0.0;
    double maxRuntimeMs = 0.0;
    
};

class SolverRunner {
public:
    static SolverStatistics runMultipleTimes(Solver& solver, const TSPAInstance& instance, int numRuns);
    
    static SolverStatistics runAllStartingPoints(Solver& solver, const TSPAInstance& instance);
};

#endif
