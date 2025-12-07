#ifndef NEARESTNEIGHBOURANY_H
#define NEARESTNEIGHBOURANY_H

#include "Solver.h"
#include "TSPAInstance.h"

class NearestNeighbourAny : public Solver {
public:
    NearestNeighbourAny() = default;
    ~NearestNeighbourAny() override = default;

    Solution solve(const TSPAInstance& instance, int startingPoint) override;
};

#endif
