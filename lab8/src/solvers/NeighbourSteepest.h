#ifndef NEIGHBOURSTEEPEST_H
#define NEIGHBOURSTEEPEST_H

#include "Solver.h"
#include "TSPAInstance.h"

class NeighbourSteepest : public Solver {
public:
  enum class StartingStrategy { Random, Heuristic };

  enum class IntraMode { NodeExchange, EdgeExchange };

  NeighbourSteepest() = default;
  explicit NeighbourSteepest(StartingStrategy s, IntraMode m)
      : startingStrategy_(s), intraMode_(m) {}

  void setStartingStrategy(StartingStrategy s) { startingStrategy_ = s; }
  StartingStrategy getStartingStrategy() const { return startingStrategy_; }

  void setIntraMode(IntraMode m) { intraMode_ = m; }
  IntraMode getIntraMode() const { return intraMode_; }

  Solution solve(const TSPAInstance &instance, int startingPoint) override;
  ~NeighbourSteepest() override = default;

private:
  IntraMode intraMode_{IntraMode::NodeExchange};
  StartingStrategy startingStrategy_{StartingStrategy::Random};
};

#endif