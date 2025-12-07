#ifndef LNS_H
#define LNS_H

#include "Solver.h"
#include "TSPAInstance.h"
#include <bitset>

class LNS : public Solver {
public:
  enum class StartingStrategy { Random, Heuristic };

  enum class IntraMode { NodeExchange, EdgeExchange };

  LNS() = default;
  explicit LNS(StartingStrategy s, IntraMode m, double t, bool doLocalSearch,
               double removePercent)
      : startingStrategy_(s), intraMode_(m), maxTime_(t),
        doLocalSearch_(doLocalSearch), removePercent_(removePercent) {}

  void setStartingStrategy(StartingStrategy s) { startingStrategy_ = s; }
  StartingStrategy getStartingStrategy() const { return startingStrategy_; }

  void setIntraMode(IntraMode m) { intraMode_ = m; }
  IntraMode getIntraMode() const { return intraMode_; }

  Solution solve(const TSPAInstance &instance, int startingPoint) override;
  ~LNS() override = default;

private:
  IntraMode intraMode_{IntraMode::NodeExchange};
  StartingStrategy startingStrategy_{StartingStrategy::Random};
  double maxTime_;
  bool doLocalSearch_;
  double removePercent_;
  std::bitset<4> bs_; // All bits set (0b1111) - allows all perturbations
};

#endif
