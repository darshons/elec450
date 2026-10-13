///////////////////////////////////////
// COMP/ELEC/MECH 450/550 — Project 3
// Authors: Darshon Singh (ds161), Winson Lin (wl85)
///////////////////////////////////////
#include "RTP.h"

#include <ompl/base/PlannerData.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/geometric/PathGeometric.h>

#include <limits>

namespace ompl::geometric
{
RTP::RTP(const base::SpaceInformationPtr &si) : base::Planner(si, "RTP")
{
    specs_.approximateSolutions = true;
    declareParam<double>("goal_bias", this, &RTP::setGoalBias, &RTP::getGoalBias, "0.:.05:1.");
}

RTP::~RTP()
{
    freeMemory();
}

void RTP::freeMemory()
{
    for (Motion *motion : tree_)
    {
        if (motion->state != nullptr)
            si_->freeState(motion->state);
        delete motion;
    }
    tree_.clear();
    lastGoalMotion_ = nullptr;
}

base::PlannerStatus RTP::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goalSampler = dynamic_cast<base::GoalSampleableRegion *>(goal);

    // Add the start state(s) as roots of the tree
    while (const base::State *start = pis_.nextStart())
    {
        auto *motion = new Motion;
        motion->state = si_->cloneState(start);
        tree_.push_back(motion);
    }
    if (tree_.empty())
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in the tree", getName().c_str(),
                static_cast<unsigned int>(tree_.size()));

    Motion *solution = nullptr;
    Motion *approxSolution = nullptr;
    double approxDist = std::numeric_limits<double>::infinity();

    // A start state might already satisfy the goal, and on later calls to
    // solve() the old tree is still here, so check what we already have.
    for (Motion *motion : tree_)
    {
        double dist = 0.0;
        if (goal->isSatisfied(motion->state, &dist))
        {
            solution = motion;
            approxDist = dist;
            break;
        }
        if (dist < approxDist)
        {
            approxDist = dist;
            approxSolution = motion;
        }
    }

    base::State *sample = si_->allocState();

    while (solution == nullptr && !ptc)
    {
        // 1. pick an existing tree vertex uniformly at random
        Motion *from = tree_[rng_.uniformInt(0, static_cast<int>(tree_.size()) - 1)];

        // 2. sample a target, using the goal with probability goalBias_
        if (goalSampler != nullptr && rng_.uniform01() < goalBias_ && goalSampler->canSample())
            goalSampler->sampleGoal(sample);
        else
            sampler_->sampleUniform(sample);

        // 3. check the whole straight line motion (this also checks the end state)
        if (!si_->checkMotion(from->state, sample))
            continue;

        auto *motion = new Motion;
        motion->state = si_->cloneState(sample);
        motion->parent = from;
        tree_.push_back(motion);

        // 4. stop if the new vertex reaches the goal, otherwise track the closest one
        double dist = 0.0;
        if (goal->isSatisfied(motion->state, &dist))
        {
            solution = motion;
            approxDist = dist;
        }
        else if (dist < approxDist)
        {
            approxDist = dist;
            approxSolution = motion;
        }
    }

    si_->freeState(sample);

    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxSolution;
        approximate = true;
    }

    if (solution != nullptr)
    {
        if (!approximate)
            lastGoalMotion_ = solution;

        // Walk back to the root through parent pointers, then reverse
        std::vector<Motion *> chain;
        for (Motion *m = solution; m != nullptr; m = m->parent)
            chain.push_back(m);

        auto path = std::make_shared<PathGeometric>(si_);
        for (auto it = chain.rbegin(); it != chain.rend(); ++it)
            path->append((*it)->state);
        pdef_->addSolutionPath(path, approximate, approxDist, getName());

        OMPL_INFORM("%s: Created %u states, %s solution with %u waypoints", getName().c_str(),
                    static_cast<unsigned int>(tree_.size()), approximate ? "approximate" : "exact",
                    static_cast<unsigned int>(chain.size()));
        return {true, approximate};
    }

    // Only possible if the tree has just the start and the goal distance was infinite
    return base::PlannerStatus::TIMEOUT;
}

void RTP::clear()
{
    base::Planner::clear();
    sampler_.reset();
    freeMemory();
}

void RTP::getPlannerData(base::PlannerData &data) const
{
    base::Planner::getPlannerData(data);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (const Motion *motion : tree_)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state),
                         base::PlannerDataVertex(motion->state));
    }
}
}  // namespace ompl::geometric
