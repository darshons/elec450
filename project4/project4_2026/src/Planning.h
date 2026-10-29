// Winson Lin (wl85), Darshon Singh (ds161). Shared planning and benchmarking utilities.
#pragma once
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/util/RandomNumbers.h>
#include "RG-RRT.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>
#include <cstdio>
#include <cstdlib>
namespace ob = ompl::base;
namespace oc = ompl::control;
constexpr double PI = 3.14159265358979323846;
// A planner request: "RRT", "KPIECE1", "RG-RRT" or "RG-RRT:<controls>:<steps>".
struct Options {
    std::string planner = "RRT", output = "path.json", log = "";
    std::vector<std::string> planners = {"RRT", "KPIECE1", "RG-RRT"};
    double torque = 3, seconds = 30;
    unsigned seed = 450, runs = 20, rgControls = 11, rgSteps = 5;
    bool benchmark = false;
};
// Template-style menu choice (1 RRT, 2 KPIECE1, 3 RG-RRT) to planner name.
inline std::string plannerName(int choice) { return choice == 1 ? "RRT" : choice == 2 ? "KPIECE1" : "RG-RRT"; }
inline std::vector<std::string> split(const std::string &s) {
    std::vector<std::string> out; std::stringstream in(s); std::string item;
    while (std::getline(in, item, ',')) if (!item.empty()) out.push_back(item);
    return out;
}
inline bool knownPlanner(const std::string &p) { return p == "RRT" || p == "KPIECE1" || p.rfind("RG-RRT", 0) == 0; }
inline Options options(int argc, char **argv) {
    Options o;
    for (int i=1;i<argc;++i) {
        std::string key=argv[i];
        if (key == "--help") {
            std::cout << "--planner RRT|KPIECE1|RG-RRT --torque 3|5|10 --seed N --time seconds --output path.json\n"
                         "--benchmark --runs N --planners RRT,KPIECE1,RG-RRT[:controls:steps] --log file.log\n"
                         "--rg-controls N --rg-steps N   (RG-RRT reachable set; default 11 controls, 5 steps)\n"
                         "With no arguments, the template's interactive menu is shown.\n";
            std::exit(0);
        }
        if (key == "--benchmark") { o.benchmark = true; continue; }
        if (i+1 == argc) throw std::runtime_error("Missing value for " + key);
        std::string value=argv[++i];
        if (key=="--planner") o.planner=value;
        else if (key=="--planners") o.planners=split(value);
        else if (key=="--torque") o.torque=std::stod(value);
        else if (key=="--time") o.seconds=std::stod(value);
        else if (key=="--runs") o.runs=std::stoul(value);
        else if (key=="--rg-controls") o.rgControls=std::stoul(value);
        else if (key=="--rg-steps") o.rgSteps=std::stoul(value);
        else if (key=="--log") o.log=value;
        else if (key=="--seed") { auto n=std::stoul(value); if (!n || n>0xffffffffUL) throw std::runtime_error("Seed out of range"); o.seed=static_cast<unsigned>(n); }
        else if (key=="--output") o.output=value;
        else throw std::runtime_error("Unknown option " + key);
    }
    if (!knownPlanner(o.planner)) throw std::runtime_error("Planner must be RRT, KPIECE1 or RG-RRT");
    for (const auto &p : o.planners) if (!knownPlanner(p)) throw std::runtime_error("Unknown planner " + p);
    if (o.torque!=3 && o.torque!=5 && o.torque!=10) throw std::runtime_error("Torque must be 3, 5, or 10");
    if (!std::isfinite(o.seconds) || o.seconds<=0) throw std::runtime_error("Time must be positive and finite");
    if (o.runs == 0 || o.rgControls < 2 || o.rgSteps == 0) throw std::runtime_error("Need runs >= 1, rg-controls >= 2, rg-steps >= 1");
    return o;
}
// Read a template menu choice in [low, high], re-prompting as the starter does.
inline int menu(const std::string &prompt, int low, int high) {
    int choice;
    do {
        std::cout << prompt;
        if (!(std::cin >> choice)) throw std::runtime_error("Invalid input");
    } while (choice < low || choice > high);
    return choice;
}
inline void configure(oc::SimpleSetupPtr &ss, const oc::ODESolver::ODE &ode,
                      const oc::ODESolver::PostPropagationEvent &post) {
    auto si=ss->getSpaceInformation();
    auto solver=std::make_shared<oc::ODEBasicSolver<>>(si, ode, 0.001);
    ss->setStatePropagator(oc::ODESolver::getStatePropagator(solver, post));
    si->setPropagationStepSize(0.02);
    si->setMinMaxControlDuration(1,25); // 0.02--0.50 seconds, forward propagation
}
inline ob::PlannerPtr makePlanner(const oc::SpaceInformationPtr &si, const std::string &name, const Options &o) {
    if (name=="RRT") { auto p=std::make_shared<oc::RRT>(si); p->setGoalBias(0.1); return p; }
    if (name=="KPIECE1") { auto p=std::make_shared<oc::KPIECE1>(si); p->setGoalBias(0.1); return p; }
    auto p=std::make_shared<oc::RGRRT>(si); p->setGoalBias(0.1);
    unsigned controls=o.rgControls, steps=o.rgSteps;
    if (name!="RG-RRT") { // RG-RRT:<controls>:<steps>, named distinctly in benchmark logs
        if (std::sscanf(name.c_str(), "RG-RRT:%u:%u", &controls, &steps) != 2) throw std::runtime_error("Bad planner " + name);
        p->setName("RG-RRT_" + std::to_string(controls) + "x" + std::to_string(steps));
    }
    p->setReachableControlCount(controls); p->setReachableSteps(steps);
    return p;
}
inline int plan(oc::SimpleSetupPtr &ss, const Options &o, const std::string &system) {
    ss->setPlanner(makePlanner(ss->getSpaceInformation(), o.planner, o));
    ss->setup();
    auto status=ss->solve(o.seconds);
    if (status != ob::PlannerStatus::EXACT_SOLUTION) {
        std::cerr << "No exact solution: " << status.asString() << '\n'; return 2;
    }
    auto &path=ss->getSolutionPath();
    if (!path.check()) throw std::runtime_error("PathControl::check failed");
    double distance;
    if (!ss->getGoal()->isSatisfied(path.getStates().back(), &distance)) throw std::runtime_error("Goal check failed");
    // Export original controls and durations, before making a dense geometric copy.
    std::ofstream f(o.output);
    if (!f) throw std::runtime_error("Cannot open " + o.output);
    f << std::setprecision(17) << "{\n\"system\":\"" << system << "\",\"planner\":\"" << o.planner
      << "\",\"seed\":" << o.seed << ",\"torque\":" << o.torque << ",\"exact\":true,\"time\":"
      << ss->getLastPlanComputationTime() << ",\"goal_distance\":" << distance << ",\"states\":[";
    auto writeState=[&](const ob::State *s) { std::vector<double> v; ss->getStateSpace()->copyToReals(v,s); f << '['; for (size_t j=0;j<v.size();++j) { if(j) f << ','; f << v[j]; } f << ']'; };
    for(size_t i=0;i<path.getStateCount();++i) { if(i) f<<','; writeState(path.getState(i)); }
    f << "],\"controls\":[";
    for(size_t i=0;i<path.getControlCount();++i) {
        if(i) f<<','; f<<'[';
        auto u=path.getControl(i)->as<oc::RealVectorControlSpace::ControlType>();
        for(unsigned j=0;j<ss->getControlSpace()->getDimension();++j) { if(j) f<<','; f<<u->values[j]; }
        f<<']';
    }
    f << "],\"durations\":[";
    for(size_t i=0;i<path.getControlCount();++i) { if(i) f<<','; f<<path.getControlDuration(i); }
    f << "],\"dense\":[";
    auto dense=path.asGeometric();
    for(size_t i=0;i<dense.getStateCount();++i) { if(i) f<<','; writeState(dense.getState(i)); }
    f << "]}\n";
    f.close(); if(!f) throw std::runtime_error("Failed to write path");
    std::cout << "Saved exact solution to " << o.output << '\n';
    return 0;
}
// OMPL Benchmark over the requested planners; writes a native log for ompl_benchmark_statistics.py.
inline int benchmark(oc::SimpleSetupPtr &ss, const Options &o, const std::string &name) {
    ompl::tools::Benchmark b(*ss, name);
    for (const auto &p : o.planners) b.addPlanner(makePlanner(ss->getSpaceInformation(), p, o));
    ompl::tools::Benchmark::Request request(o.seconds, 4096.0, o.runs);
    request.displayProgress = true;
    request.saveConsoleOutput = false;
    b.benchmark(request);
    const std::string log = o.log.empty() ? name + ".log" : o.log;
    if (!b.saveResultsToFile(log.c_str())) throw std::runtime_error("Cannot write " + log);
    std::cout << "Saved benchmark log to " << log << '\n';
    return 0;
}
