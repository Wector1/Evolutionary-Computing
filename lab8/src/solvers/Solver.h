#ifndef SOLVER_H
#define SOLVER_H

#include "TSPAInstance.h"
#include "Solution.h"

class Solver {
public:
    virtual Solution solve(const TSPAInstance& instance, int startingPoint) = 0;
    virtual ~Solver() {}
};

#endif 