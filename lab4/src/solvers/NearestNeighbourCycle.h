#ifndef NEARESTNEIGHBOURCYCLE_H
#define NEARESTNEIGHBOURCYCLE_H

#include "Solver.h"
#include "TSPAInstance.h"

class NearestNeighbourCycle : public Solver {
public:
    NearestNeighbourCycle() = default;
    ~NearestNeighbourCycle() override = default;

    Solution solve(const TSPAInstance& instance, int startingPoint) override;
};

#endif
