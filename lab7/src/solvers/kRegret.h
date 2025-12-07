#ifndef KREGRET_H
#define KREGRET_H

#include "Solver.h"
#include "TSPAInstance.h"

class kRegret : public Solver {
public:
  kRegret() = default;
  ~kRegret() override = default;

  Solution solve(const TSPAInstance &instance, int startingPoint) override;
};

#endif
