#ifndef LOCALSTEEPEST_H
#define LOCALSTEEPEST_H

#include "Solver.h"
#include "TSPAInstance.h"
#include <vector>

class LocalSteepest : public Solver {
public:
  enum class StartingStrategy { Random, Heuristic };

  enum class IntraMode { NodeExchange, EdgeExchange };

  LocalSteepest() = default;
  explicit LocalSteepest(StartingStrategy s, IntraMode m)
      : startingStrategy_(s), intraMode_(m) {}
  std::vector<int> initialPath;

  void setStartingStrategy(StartingStrategy s) { startingStrategy_ = s; }
  StartingStrategy getStartingStrategy() const { return startingStrategy_; }

  void setIntraMode(IntraMode m) { intraMode_ = m; }
  IntraMode getIntraMode() const { return intraMode_; }

  Solution solve(const TSPAInstance &instance, int startingPoint) override;
  ~LocalSteepest() override = default;

private:
  IntraMode intraMode_{IntraMode::NodeExchange};
  StartingStrategy startingStrategy_{StartingStrategy::Random};
};

#endif
