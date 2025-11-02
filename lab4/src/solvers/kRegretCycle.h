#ifndef KREGRETCYCLE_H
#define KREGRETCYCLE_H

#include "Solver.h"
#include "TSPAInstance.h"

class kRegretCycle : public Solver {
public:
    kRegretCycle() = default;
    ~kRegretCycle() override = default;

    Solution solve(const TSPAInstance& instance, int startingPoint) override;
};

#endif
