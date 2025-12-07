#ifndef NEARESTNEIGHBOUR2REGRETWEIGHTED_H
#define NEARESTNEIGHBOUR2REGRETWEIGHTED_H

#include "Solver.h"
#include "TSPAInstance.h"

class NearestNeighbour2RegretWeighted : public Solver {
public:
    NearestNeighbour2RegretWeighted() = default;
    ~NearestNeighbour2RegretWeighted() override = default;

    Solution solve(const TSPAInstance& instance, int startingPoint) override;
};

#endif
