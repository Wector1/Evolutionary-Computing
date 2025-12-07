#ifndef SOLVERRUNNER_H
#define SOLVERRUNNER_H

#include "Solution.h"
#include "Solver.h"
#include "TSPAInstance.h"
#include <vector>

struct SolverStatistics {
  Solution bestSolution;
  Solution worstSolution;
  double averageCost;
  double minCost;
  double averageSteps;
  double maxCost;
  int numRuns;
  double totalRuntimeMs = 0.0;
  double averageRuntimeMs = 0.0;
  double minRuntimeMs = 0.0;
  double maxRuntimeMs = 0.0;
  long long minSteps = 1'000'000'000'000'000;
  long long maxSteps = 0;
};

class SolverRunner {
public:
  static SolverStatistics
  runMultipleTimes(Solver &solver, const TSPAInstance &instance, int numRuns);

  static SolverStatistics runAllStartingPoints(Solver &solver,
                                               const TSPAInstance &instance,
                                               int numberOfRuns = -1);
};

#endif
