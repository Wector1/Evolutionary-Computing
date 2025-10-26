#ifndef NEARESTNEIGHBOUR2REGRET_H
#define NEARESTNEIGHBOUR2REGRET_H

#include "Solver.h"
#include "TSPAInstance.h"

class NearestNeighbour2Regret : public Solver {
public:
    NearestNeighbour2Regret() = default;
    ~NearestNeighbour2Regret() override = default;

    Solution solve(const TSPAInstance& instance, int startingPoint) override;
};

#endif
