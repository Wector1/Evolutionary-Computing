#ifndef ILS_H
#define ILS_H

#include "Solver.h"
#include "TSPAInstance.h"
#include <bitset>


class ILS : public Solver {
public:
    enum class StartingStrategy {
        Random,
        Heuristic
    };

    enum class IntraMode {
        NodeExchange,
        EdgeExchange
    };

    ILS() = default;
    explicit ILS(StartingStrategy s, IntraMode m, double t, int bs) : startingStrategy_(s), intraMode_(m), maxTime_(t), bs_(bs) {}

    void setStartingStrategy(StartingStrategy s) { startingStrategy_ = s; }
    StartingStrategy getStartingStrategy() const { return startingStrategy_; }

    void setIntraMode(IntraMode m) { intraMode_ = m; }
    IntraMode getIntraMode() const { return intraMode_; }

    Solution solve(const TSPAInstance& instance, int startingPoint) override;
    ~ILS() override = default;

private:
    IntraMode intraMode_{IntraMode::NodeExchange};
    StartingStrategy startingStrategy_{StartingStrategy::Random};
    double maxTime_;
    std::bitset<4> bs_; // All bits set (0b1111) - allows all perturbations
};


#endif
