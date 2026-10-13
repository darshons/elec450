///////////////////////////////////////
// COMP/ELEC/MECH 450/550 — Project 3
// Authors: Darshon Singh (ds161), Winson Lin (wl85)
///////////////////////////////////////
#pragma once

#include <ompl/base/Planner.h>
#include <ompl/base/StateSampler.h>
#include <ompl/util/RandomNumbers.h>

#include <vector>

namespace ompl::geometric
{
class RTP : public base::Planner
{
public:
    explicit RTP(const base::SpaceInformationPtr &si);
    ~RTP() override;

    base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
    void clear() override;
    void getPlannerData(base::PlannerData &data) const override;

    void setGoalBias(double goalBias) { goalBias_ = goalBias; }
    double getGoalBias() const { return goalBias_; }

protected:
    // One vertex of the tree. The planner owns both the Motion and its state.
    struct Motion
    {
        base::State *state{nullptr};
        Motion *parent{nullptr};
    };

    // Frees every state and motion in the tree.
    void freeMemory();

    double goalBias_ = 0.05;
    RNG rng_;

    base::StateSamplerPtr sampler_;

    // Every tree vertex. A plain vector works since RTP picks vertices
    // uniformly at random, so no nearest neighbor structure is needed.
    std::vector<Motion *> tree_;

    // Last vertex that reached the goal (used to label the goal in getPlannerData)
    Motion *lastGoalMotion_{nullptr};
};
}  // namespace ompl::geometric
