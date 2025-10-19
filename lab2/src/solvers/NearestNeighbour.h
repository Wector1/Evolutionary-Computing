#ifndef NEARESTNEIGHBOUR_H
#define NEARESTNEIGHBOUR_H

#include "Solver.h"
#include "TSPAInstance.h"
#include <random>

class NearestNeighbour : public Solver {
public:
    NearestNeighbour() = default;
    ~NearestNeighbour() override = default;

    Solution solve(const TSPAInstance& instance, int startingPoint) override;
};

#endif