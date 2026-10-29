///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Darshon Singh (ds161), Winson Lin (wl85)
//////////////////////////////////////

#include "RG-RRT.h"

#include <limits>

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/tools/config/SelfConfig.h>

ompl::control::RGRRT::RGRRT(const SpaceInformationPtr &si) : base::Planner(si, "RG-RRT")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();

    Planner::declareParam<double>("goal_bias", this, &RGRRT::setGoalBias, &RGRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<unsigned int>("reachable_controls", this, &RGRRT::setReachableControlCount,
                                        &RGRRT::getReachableControlCount, "2:1:50");
    Planner::declareParam<unsigned int>("reachable_steps", this, &RGRRT::setReachableSteps,
                                        &RGRRT::getReachableSteps, "1:1:100");
}

ompl::control::RGRRT::~RGRRT()
{
    freeMemory();
}

void ompl::control::RGRRT::setup()
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });

    auto *space = dynamic_cast<const RealVectorControlSpace *>(siC_->getControlSpace().get());
    if (space == nullptr)
        throw Exception(getName(), "requires a RealVectorControlSpace");
    if (reachableControls_ < 2 || reachableSteps_ == 0)
        throw Exception(getName(), "needs at least 2 reachable controls and 1 reachable step");

    for (auto *c : reachableControlSet_)
        siC_->freeControl(c);
    reachableControlSet_.clear();

    // Evenly spaced values of u0 over its bounds, e.g. {-10, -8, ..., 10} for 11 controls and |tau| <= 10.
    const auto &bounds = space->getBounds();
    for (unsigned int i = 0; i < reachableControls_; ++i)
    {
        Control *c = siC_->allocControl();
        auto *values = c->as<RealVectorControlSpace::ControlType>()->values;
        for (unsigned int d = 0; d < space->getDimension(); ++d)
            values[d] = (bounds.low[d] + bounds.high[d]) / 2.0;
        values[0] = bounds.low[0] + (bounds.high[0] - bounds.low[0]) * i / (reachableControls_ - 1);
        reachableControlSet_.push_back(c);
    }
}

void ompl::control::RGRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::control::RGRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            if (motion->control)
                siC_->freeControl(motion->control);
            for (auto *r : motion->reachable)
                si_->freeState(r);
            delete motion;
        }
    }
}

void ompl::control::RGRRT::computeReachable(Motion *motion)
{
    for (const auto *c : reachableControlSet_)
    {
        base::State *r = si_->allocState();
        // propagateWhileValid stops at the first invalid state, so a state is kept only if the whole
        // segment is valid: an invalid state is certainly not reachable.
        if (siC_->propagateWhileValid(motion->state, c, reachableSteps_, r) == reachableSteps_)
            motion->reachable.push_back(r);
        else
            si_->freeState(r);
    }
}

bool ompl::control::RGRRT::reachableTowards(const Motion *motion, const base::State *sample) const
{
    const double nearDistance = si_->distance(motion->state, sample);
    for (const auto *r : motion->reachable)
        if (si_->distance(r, sample) < nearDistance)
            return true;
    return false;
}

ompl::base::PlannerStatus ompl::control::RGRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        computeReachable(motion);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    unsigned int discarded = 0;

    auto *rmotion = new Motion(siC_);
    base::State *rstate = rmotion->state;
    Control *rctrl = rmotion->control;

    while (!ptc)
    {
        // Sample a random state, with goal biasing.
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        // Reachability-guided selection: keep the sample only if R(q_near) leads towards it.
        Motion *nmotion = nn_->nearest(rmotion);
        if (!reachableTowards(nmotion, rstate))
        {
            ++discarded;
            continue;
        }

        // Expand q_near towards the sample, as in control RRT.
        unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);
        if (cd < siC_->getMinControlDuration())
            continue;

        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, rmotion->state);
        siC_->copyControl(motion->control, rctrl);
        motion->steps = cd;
        motion->parent = nmotion;
        computeReachable(motion);
        nn_->add(motion);

        double dist = 0.0;
        bool solv = goal->isSatisfied(motion->state, &dist);
        if (solv)
        {
            approxdif = dist;
            solution = motion;
            break;
        }
        if (dist < approxdif)
        {
            approxdif = dist;
            approxsol = motion;
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        auto path(std::make_shared<PathControl>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        solved = true;
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
    }

    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states, discarded %u samples", getName().c_str(), nn_->size(), discarded);

    return {solved, approximate};
}

void ompl::control::RGRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    double delta = siC_->getPropagationStepSize();

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto m : motions)
    {
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state),
                             PlannerDataEdgeControl(m->control, m->steps * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state));
    }
}
