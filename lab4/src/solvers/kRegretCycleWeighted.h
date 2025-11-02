#ifndef KREGRETCYCLEWEIGHTED_H
#define KREGRETCYCLEWEIGHTED_H

#include "Solver.h"
#include "TSPAInstance.h"

class kRegretCycleWeighted : public Solver {
public:
    kRegretCycleWeighted() = default;
    ~kRegretCycleWeighted() override = default;

    Solution solve(const TSPAInstance& instance, int startingPoint) override;
};

#endif
