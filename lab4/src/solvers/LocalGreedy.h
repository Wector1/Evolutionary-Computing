#ifndef LOCAL_GREEDY_H
#define LOCAL_GREEDY_H

#include "Solver.h"
#include "TSPAInstance.h"


class LocalGreedy : public Solver {
public:
    enum class StartingStrategy {
        Random,
        Heuristic
    };

    enum class IntraMode {
        NodeExchange,
        EdgeExchange
    };

    LocalGreedy() = default;
    explicit LocalGreedy(StartingStrategy s, IntraMode m) : startingStrategy_(s), intraMode_(m) {}

    void setStartingStrategy(StartingStrategy s) { startingStrategy_ = s; }
    StartingStrategy getStartingStrategy() const { return startingStrategy_; }

    void setIntraMode(IntraMode m) { intraMode_ = m; }
    IntraMode getIntraMode() const { return intraMode_; }

    Solution solve(const TSPAInstance& instance, int startingPoint) override;
    ~LocalGreedy() override = default;

private:
    IntraMode intraMode_{IntraMode::NodeExchange};
    StartingStrategy startingStrategy_{StartingStrategy::Random};
};


#endif