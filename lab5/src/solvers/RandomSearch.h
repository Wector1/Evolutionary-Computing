#ifndef RANDOMSEARCH_H
#define RANDOMSEARCH_H

#include "Solver.h"
#include "TSPAInstance.h"
#include <vector>
#include <cstdlib>
#include <ctime>

class RandomSearch : public Solver {
public:
    RandomSearch();
    Solution solve(const TSPAInstance& instance, int startingPoint) override;

private:
    Solution generateRandomSolution();
    double evaluateSolution(const Solution& solution);
};

#endif