#ifndef SOLVERRUNNER_H
#define SOLVERRUNNER_H

#include "Solver.h"
#include "TSPAInstance.h"
#include "Solution.h"
#include <vector>

struct SolverStatistics {
    Solution bestSolution;
    Solution worstSolution;
    double averageCost;
    double minCost;
    double maxCost;
    int numRuns;
};

class SolverRunner {
public:
    static SolverStatistics runMultipleTimes(Solver& solver, const TSPAInstance& instance, int numRuns);
    
    static SolverStatistics runAllStartingPoints(Solver& solver, const TSPAInstance& instance);
};

#endif
