#ifndef KREGRETCYCLEWEIGHTED_H
#define KREGRETCYCLEWEIGHTED_H

#include "Solver.h"
#include "TSPAInstance.h"
#include <vector>

class kRegretCycleWeighted : public Solver {
public:
  kRegretCycleWeighted() = default;
  ~kRegretCycleWeighted() override = default;
  using Solver::solve;

  Solution solve(const TSPAInstance &instance, int startingPoint,
                 std::vector<int> initialCycle);

  Solution solve(const TSPAInstance &instance, int startingPoint) override;
};

#endif
