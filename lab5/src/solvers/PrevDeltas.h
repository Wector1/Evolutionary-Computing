#ifndef PREVDELTAS_H
#define PREVDELTAS_H

#include "Solver.h"
#include "TSPAInstance.h"


class PrevDeltas : public Solver {
public:
    enum class StartingStrategy {
        Random,
        Heuristic
    };

    PrevDeltas() = default;
    explicit PrevDeltas(StartingStrategy s) : startingStrategy_(s) {}

    void setStartingStrategy(StartingStrategy s) { startingStrategy_ = s; }
    StartingStrategy getStartingStrategy() const { return startingStrategy_; }

    Solution solve(const TSPAInstance& instance, int startingPoint) override;
    ~PrevDeltas() override = default;

private:
    StartingStrategy startingStrategy_{StartingStrategy::Random};
};


#endif