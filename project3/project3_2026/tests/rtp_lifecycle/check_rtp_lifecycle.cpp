// Exercise the actual RTP without altering its implementation or arm binding.
#include "RTP.h"
#include <ompl/base/PlannerData.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace ob = ompl::base;
namespace og = ompl::geometric;

void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

class CountingSpace : public ob::RealVectorStateSpace
{
public:
    CountingSpace() : ob::RealVectorStateSpace(1) { setBounds(-1.0, 1.0); }
    ob::State *allocState() const override
    {
        auto *state = ob::RealVectorStateSpace::allocState();
        ++liveStates;
        return state;
    }
    void freeState(ob::State *state) const override
    {
        ob::RealVectorStateSpace::freeState(state);
        --liveStates;
    }
    mutable long liveStates = 0;
};

class InspectableRTP : public og::RTP
{
public:
    using og::RTP::RTP;
    std::size_t size() const { return tree_.size(); }
    bool reset() const { return tree_.empty() && !sampler_ && !lastGoalMotion_; }
    double closest(double goal) const
    {
        double result = std::numeric_limits<double>::infinity();
        for (const auto *motion : tree_)
            result = std::min(result, std::abs(value(motion->state) - goal));
        return result;
    }
    static double value(const ob::State *state)
    {
        return state->as<ob::RealVectorStateSpace::StateType>()->values[0];
    }
};

void checkTree(const InspectableRTP &planner, const ob::SpaceInformationPtr &si, bool exact)
{
    ob::PlannerData data(si);
    planner.getPlannerData(data);
    require(data.numVertices() == planner.size(), "missing planner vertices");
    require(data.numStartVertices() == 1, "incorrect start label");
    require(data.numGoalVertices() == (exact ? 1u : 0u), "incorrect goal label");
    require(data.numEdges() + 1 == data.numVertices(), "incorrect tree edge count");
    std::vector<unsigned int> incoming(data.numVertices(), 0);
    for (unsigned int i = 0; i < data.numVertices(); ++i)
    {
        std::vector<unsigned int> children;
        data.getEdges(i, children);
        for (auto j : children)
        {
            ++incoming[j];
            require(si->checkMotion(data.getVertex(i).getState(), data.getVertex(j).getState()),
                    "invalid exported tree edge");
        }
    }
    for (unsigned int i = 0; i < data.numVertices(); ++i)
        require(incoming[i] == (data.isStartVertex(i) ? 0u : 1u), "incorrect parent relationship");
}

int main()
{
    try
    {
        ompl::RNG::setSeed(450);
        auto space = std::make_shared<CountingSpace>();
        {
            og::SimpleSetup setup(space);
            setup.setStateValidityChecker([](const ob::State *) { return true; });
            ob::ScopedState<> start(space), goal(space);
            start[0] = -0.9;
            goal[0] = 0.9;
            setup.setStartAndGoalStates(start, goal);
            auto planner = std::make_shared<InspectableRTP>(setup.getSpaceInformation());
            planner->setGoalBias(1.0);
            setup.setPlanner(planner);
            setup.setup();
            for (int trial = 0; trial < 25; ++trial)
            {
                require(setup.solve(0.1) == ob::PlannerStatus::EXACT_SOLUTION, "exact solve failed");
                require(planner->size() == 2, "clear retained old tree vertices");
                auto &path = setup.getSolutionPath();
                require(path.getStateCount() == 2 && path.check(), "incorrect exact path");
                require(std::abs(InspectableRTP::value(path.getState(0)) + 0.9) < 1e-12,
                        "incorrect exact path start");
                require(std::abs(InspectableRTP::value(path.getState(1)) - 0.9) < 1e-12,
                        "incorrect exact path goal");
                checkTree(*planner, setup.getSpaceInformation(), true);
                require(planner->solve(ob::plannerAlwaysTerminatingCondition()) ==
                            ob::PlannerStatus::EXACT_SOLUTION, "repeat solve lost exact solution");
                require(planner->size() == 2, "repeat solve duplicated start states");
                const long before = space->liveStates;
                planner->clear();
                require(planner->reset(), "clear did not reset planner storage");
                require(space->liveStates == before - 2, "clear did not free both tree states");
                require(path.check(), "clear invalidated the independent solution path");
                setup.getProblemDefinition()->clearSolutionPaths();
            }
            // A valid start already at the goal must be exact, even with no planning time.
            setup.clear();
            setup.setStartAndGoalStates(start, start);
            require(planner->solve(ob::plannerAlwaysTerminatingCondition()) ==
                        ob::PlannerStatus::EXACT_SOLUTION, "start-equals-goal failed");
            require(setup.getSolutionPath().getStateCount() == 1, "incorrect trivial path");
            checkTree(*planner, setup.getSpaceInformation(), true);
        }
        require(space->liveStates == 0, "exact planner/destructor leaked allocated states");
        {
            og::SimpleSetup setup(space);
            setup.setStateValidityChecker([](const ob::State *s) {
                return std::abs(InspectableRTP::value(s)) > 0.2;
            });
            setup.getSpaceInformation()->setStateValidityCheckingResolution(0.001);
            ob::ScopedState<> start(space), goal(space);
            start[0] = -0.9;
            goal[0] = 0.9;
            setup.setStartAndGoalStates(start, goal);
            auto planner = std::make_shared<InspectableRTP>(setup.getSpaceInformation());
            planner->setGoalBias(0.0);
            setup.setPlanner(planner);
            setup.setup();
            // Immediate termination should return the start as the best approximation.
            require(planner->solve(ob::plannerAlwaysTerminatingCondition()) ==
                        ob::PlannerStatus::APPROXIMATE_SOLUTION, "zero-time approximation failed");
            require(setup.getSolutionPath().getStateCount() == 1, "incorrect zero-time path");
            for (int call = 0; call < 2; ++call)
            {
                setup.getProblemDefinition()->clearSolutionPaths();
                auto previousSize = planner->size();
                require(setup.solve(0.01) == ob::PlannerStatus::APPROXIMATE_SOLUTION,
                        "disconnected query should be approximate");
                auto &path = setup.getSolutionPath();
                require(path.check() && planner->size() > previousSize, "failed continuation");
                double endpoint = InspectableRTP::value(path.getState(path.getStateCount() - 1));
                require(std::abs(std::abs(endpoint - 0.9) - planner->closest(0.9)) < 1e-12,
                        "approximation does not end at closest vertex");
                checkTree(*planner, setup.getSpaceInformation(), false);
            }
            setup.clear();
            start[0] = 0.0;
            setup.setStartAndGoalStates(start, goal);
            require(setup.solve(0.01) == ob::PlannerStatus::INVALID_START, "invalid start not rejected");
            require(planner->size() == 0, "invalid start inserted into tree");
        }
        require(space->liveStates == 0, "approximate planner/destructor leaked allocated states");
        std::cout << "PASS: 25 clear/reuse cycles, exact/approximate/continuation/trivial/invalid-start, "
                     "tree labels/edges, independent path ownership, balanced state allocation\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
