// Provided Python interface to the C++ planner and collision model.
#include "ArmPlanning.h"
#include "RTP.h"

#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include <ompl/base/PlannerData.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/util/RandomNumbers.h>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <limits>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace nb::literals;

namespace
{
std::vector<double> configuration(const ob::State *state, unsigned int dimension)
{
    const auto *values = state->as<ob::RealVectorStateSpace::StateType>()->values;
    return {values, values + dimension};
}

class ArmValidityChecker : public ob::StateValidityChecker
{
public:
    ArmValidityChecker(const ob::SpaceInformationPtr &si, const ArmWorld &world,
                       unsigned int dimension)
        : ob::StateValidityChecker(si), world_(world), dimension_(dimension)
    {
        specs_.clearanceComputationType = ob::StateValidityCheckerSpecs::APPROXIMATE;
    }

    bool isValid(const ob::State *state) const override
    {
        return world_.isValid(configuration(state, dimension_));
    }

    double clearance(const ob::State *state) const override
    {
        // Workspace clearance of the sphere approximation, measured in meters.
        return world_.clearance(configuration(state, dimension_));
    }

private:
    const ArmWorld &world_;
    unsigned int dimension_;
};

std::shared_ptr<ob::RealVectorStateSpace> makeSpace(const ArmModel &model)
{
    const unsigned int dimension = static_cast<unsigned int>(model.lower.size());
    auto space = std::make_shared<ob::RealVectorStateSpace>(dimension);
    ob::RealVectorBounds bounds(dimension);
    bounds.low = model.lower;
    bounds.high = model.upper;
    space->setBounds(bounds);
    return space;
}

struct PlanningProblem
{
    PlanningProblem(const ArmModel &model, const std::vector<SphereSpec> &spheres,
                    const std::vector<BoxSpec> &boxes, const std::vector<double> &start,
                    const std::vector<double> &goal, double resolution)
        : dimension(static_cast<unsigned int>(model.lower.size())), world(model, spheres, boxes),
          space(makeSpace(model)), setup(space)
    {
        const auto &si = setup.getSpaceInformation();
        setup.setStateValidityChecker(std::make_shared<ArmValidityChecker>(si, world, dimension));
        si->setStateValidityCheckingResolution(resolution);
        ob::ScopedState<> startState(space), goalState(space);
        for (unsigned int i = 0; i < dimension; ++i)
        {
            startState[i] = start[i];
            goalState[i] = goal[i];
        }
        setup.setStartAndGoalStates(startState, goalState);
    }

    unsigned int dimension;
    ArmWorld world;
    std::shared_ptr<ob::RealVectorStateSpace> space;
    og::SimpleSetup setup;
};

ob::PlannerPtr makePlanner(const ob::SpaceInformationPtr &si, const std::string &planner,
                           double goalBias, double plannerRange, unsigned int maxNeighbors)
{
    if (planner == "RTP")
    {
        auto instance = std::make_shared<og::RTP>(si);
        instance->setGoalBias(goalBias);
        return instance;
    }
    else if (planner == "RRT")
    {
        auto instance = std::make_shared<og::RRT>(si);
        instance->setGoalBias(goalBias);
        instance->setRange(plannerRange);
        return instance;
    }
    else if (planner == "RRTConnect")
    {
        auto instance = std::make_shared<og::RRTConnect>(si);
        instance->setRange(plannerRange);
        return instance;
    }
    else if (planner == "PRM")
    {
        auto instance = std::make_shared<og::PRM>(si);
        instance->setMaxNearestNeighbors(maxNeighbors);
        return instance;
    }
    throw std::invalid_argument("Unknown planner: " + planner);
}

std::string statusName(ob::PlannerStatus solved)
{
    switch (static_cast<ob::PlannerStatus::StatusType>(solved))
    {
        case ob::PlannerStatus::EXACT_SOLUTION: return "exact";
        case ob::PlannerStatus::APPROXIMATE_SOLUTION: return "approximate";
        case ob::PlannerStatus::TIMEOUT: return "timeout";
        case ob::PlannerStatus::INVALID_START: return "invalid_start";
        case ob::PlannerStatus::INVALID_GOAL: return "invalid_goal";
        case ob::PlannerStatus::UNRECOGNIZED_GOAL_TYPE: return "unrecognized_goal_type";
        case ob::PlannerStatus::CRASH: return "crash";
        case ob::PlannerStatus::ABORT: return "abort";
        default:
        {
            std::string name = solved.asString();
            std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c)
            {
                return c == ' ' ? '_' : static_cast<char>(std::tolower(c));
            });
            return name;
        }
    }
}

double minimumClearance(const PlanningProblem &problem, const og::PathGeometric &solution)
{
    const auto &space = problem.space;
    double clearance = problem.world.clearance(configuration(solution.getState(0), problem.dimension));
    ob::ScopedState<> sample(space);
    for (std::size_t i = 1; i < solution.getStateCount(); ++i)
    {
        const ob::State *from = solution.getState(i - 1);
        const ob::State *to = solution.getState(i);
        const unsigned int segments = std::max(1u, space->validSegmentCount(from, to));
        for (unsigned int j = 1; j <= segments; ++j)
        {
            space->interpolate(from, to, static_cast<double>(j) / segments, sample.get());
            clearance = std::min(clearance,
                problem.world.clearance(configuration(sample.get(), problem.dimension)));
        }
    }
    return clearance;
}

nb::dict collectResult(PlanningProblem &problem, const ob::PlannerPtr &planner,
                       ob::PlannerStatus solved, double planningTime)
{
    const bool exact = solved == ob::PlannerStatus::EXACT_SOLUTION;
    const bool approximate = solved == ob::PlannerStatus::APPROXIMATE_SOLUTION;
    nb::dict result;
    result["status"] = statusName(solved);
    result["planning_time"] = planningTime;
    result["length"] = nb::none();
    result["smoothness"] = nb::none();
    result["min_clearance"] = nb::none();
    std::vector<std::vector<double>> path;
    if (exact || approximate)
    {
        const og::PathGeometric &solution = problem.setup.getSolutionPath();
        for (std::size_t i = 0; i < solution.getStateCount(); ++i)
            path.push_back(configuration(solution.getState(i), problem.dimension));
        if (exact)
        {
            result["length"] = solution.length();
            result["smoothness"] = solution.smoothness();
            const double clearance = minimumClearance(problem, solution);
            // An empty environment has no finite obstacle-clearance metric.
            if (std::isfinite(clearance)) result["min_clearance"] = clearance;
        }
    }
    result["path"] = path;

    ob::PlannerData data(problem.setup.getSpaceInformation());
    planner->getPlannerData(data);
    std::vector<std::vector<double>> vertices;
    std::vector<std::array<unsigned int, 2>> edges;
    for (unsigned int i = 0; i < data.numVertices(); ++i)
    {
        vertices.push_back(configuration(data.getVertex(i).getState(), problem.dimension));
        std::vector<unsigned int> neighbors;
        data.getEdges(i, neighbors);
        for (unsigned int j : neighbors) edges.push_back({i, j});
    }
    result["vertices"] = vertices;
    result["edges"] = edges;
    return result;
}

nb::dict plan(const ArmModel &model, const std::vector<SphereSpec> &spheres,
              const std::vector<BoxSpec> &boxes, const std::vector<double> &start,
              const std::vector<double> &goal, const std::string &planner,
              double timeLimit, double goalBias, double plannerRange,
              unsigned int maxNeighbors, double resolution, unsigned int seed)
{
    ompl::RNG::setSeed(seed);
    PlanningProblem problem(model, spheres, boxes, start, goal, resolution);
    auto instance = makePlanner(problem.setup.getSpaceInformation(), planner,
                                goalBias, plannerRange, maxNeighbors);
    problem.setup.setPlanner(instance);
    problem.setup.setup();
    const auto begin = std::chrono::steady_clock::now();
    const ob::PlannerStatus solved = problem.setup.solve(timeLimit);
    const double planningTime = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - begin).count();
    return collectResult(problem, instance, solved, planningTime);
}

void benchmark(const ArmModel &model, const std::vector<SphereSpec> &spheres,
                   const std::vector<BoxSpec> &boxes, const std::vector<double> &start,
                   const std::vector<double> &goal, const nb::list &configs,
                   const std::string &logPath, const std::string &experimentName,
                   double timeLimit, unsigned int runCount, double resolution,
                   double memoryLimit)
{
    PlanningProblem problem(model, spheres, boxes, start, goal, resolution);
    ompl::tools::Benchmark experiment(problem.setup, experimentName);
    for (nb::handle item : configs)
    {
        const auto config = nb::cast<nb::dict>(item);
        const auto settings = nb::cast<nb::dict>(config["settings"]);
        const auto name = nb::cast<std::string>(config["name"]);
        const auto planner = nb::cast<std::string>(config["planner"]);
        auto instance = makePlanner(problem.setup.getSpaceInformation(), planner,
            nb::cast<double>(settings.get("goal_bias", nb::float_(0.05))),
            nb::cast<double>(settings.get("planner_range", nb::float_(0.4))),
            nb::cast<unsigned int>(settings.get("max_neighbors", nb::int_(10))));
        instance->setName(name);
        experiment.addPlanner(instance);
    }

    experiment.setPostRunEvent([&](const ob::PlannerPtr &,
                                   ompl::tools::Benchmark::RunProperties &properties)
    {
        if (problem.setup.haveExactSolutionPath())
        {
            const double clearance = minimumClearance(problem, problem.setup.getSolutionPath());
            if (std::isfinite(clearance))
            {
                std::ostringstream value;
                value << std::setprecision(17) << clearance;
                properties["minimum clearance REAL"] = value.str();
            }
        }
    });

    ompl::tools::Benchmark::Request request;
    request.maxTime = timeLimit;
    request.maxMem = memoryLimit;
    request.runCount = runCount;
    request.displayProgress = false;
    request.saveConsoleOutput = false;
    request.simplify = false;
    experiment.benchmark(request);
    if (!experiment.saveResultsToFile(logPath.c_str()))
        throw std::runtime_error("Could not write OMPL benchmark log: " + logPath);
}
}  // namespace

NB_MODULE(project3_cpp, m)
{
    m.doc() = "Project 3: C++ OMPL planning and arm collision checking.";
    nb::class_<ArmModel>(m, "ArmModel")
        .def(nb::init<const std::vector<JointSpec> &, const std::vector<LinkSphereSpec> &,
                      std::vector<double>, std::vector<double>>(),
             "joints"_a, "spheres"_a, "lower"_a, "upper"_a);
    m.def("fk_spheres", [](const ArmModel &model, const std::vector<double> &q)
    {
        std::vector<SphereSpec> spheres;
        for (const Sphere3D &sphere : model.worldSpheres(q))
            spheres.emplace_back(Vec3{sphere.x, sphere.y, sphere.z}, sphere.r);
        return spheres;
    }, "model"_a, "q"_a);
    m.def("is_valid", [](const ArmModel &model, const std::vector<SphereSpec> &spheres,
                          const std::vector<BoxSpec> &boxes, const std::vector<double> &q)
    {
        return ArmWorld(model, spheres, boxes).isValid(q);
    }, "model"_a, "spheres"_a, "boxes"_a, "q"_a);
    m.def("clearance", [](const ArmModel &model, const std::vector<SphereSpec> &spheres,
                           const std::vector<BoxSpec> &boxes, const std::vector<double> &q)
    {
        return ArmWorld(model, spheres, boxes).clearance(q);
    }, "model"_a, "spheres"_a, "boxes"_a, "q"_a);
    m.def("plan", &plan, "model"_a, "spheres"_a, "boxes"_a, "start"_a, "goal"_a,
          "planner"_a = "RRTConnect", "time_limit"_a = 2.0, "goal_bias"_a = 0.05,
          "planner_range"_a = 0.4, "max_neighbors"_a = 10, "resolution"_a = 0.005,
          "seed"_a = 1);
    m.def("benchmark", &benchmark, "model"_a, "spheres"_a, "boxes"_a, "start"_a, "goal"_a,
          "configs"_a, "log_path"_a, "experiment_name"_a = "Project 3 arm planning",
          "time_limit"_a = 2.0, "run_count"_a = 5, "resolution"_a = 0.005,
          "memory_limit"_a = 1024.0);
    m.def("set_seed", &ompl::RNG::setSeed, "seed"_a);
}
