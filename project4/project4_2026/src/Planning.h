// Winson Lin (wl85), Darshon Singh (ds161). Shared checkpoint utilities.
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
#include <ompl/util/RandomNumbers.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <cmath>
#include <cstdlib>
namespace ob = ompl::base;
namespace oc = ompl::control;
constexpr double PI = 3.14159265358979323846;
struct Options {
    std::string planner = "RRT", output = "path.json";
    double torque = 3, seconds = 30;
    unsigned seed = 450;
};
inline Options options(int argc, char **argv, bool pendulum) {
    Options o;
    if (argc == 1) {
        int p;
        std::cout << "Planner (1 RRT, 2 KPIECE1): ";
        if (!(std::cin >> p) || (p != 1 && p != 2)) throw std::runtime_error("Invalid planner");
        o.planner = p == 1 ? "RRT" : "KPIECE1";
        if (pendulum) { std::cout << "Torque bound (3, 5, 10): "; if (!(std::cin >> o.torque)) throw std::runtime_error("Invalid torque"); }
    }
    for (int i=1;i<argc;++i) {
        std::string key=argv[i];
        if (key == "--help") { std::cout << "--planner RRT|KPIECE1 --torque 3|5|10 --seed N --time seconds --output path.json\n"; std::exit(0); }
        if (i+1 == argc) throw std::runtime_error("Missing value for " + key);
        std::string value=argv[++i];
        if (key=="--planner") o.planner=value;
        else if (key=="--torque") o.torque=std::stod(value);
        else if (key=="--time") o.seconds=std::stod(value);
        else if (key=="--seed") { auto n=std::stoul(value); if (!n || n>0xffffffffUL) throw std::runtime_error("Seed out of range"); o.seed=static_cast<unsigned>(n); }
        else if (key=="--output") o.output=value;
        else throw std::runtime_error("Unknown option " + key);
    }
    if (o.planner!="RRT" && o.planner!="KPIECE1") throw std::runtime_error("Checkpoint supports RRT and KPIECE1");
    if (o.torque!=3 && o.torque!=5 && o.torque!=10) throw std::runtime_error("Torque must be 3, 5, or 10");
    if (!std::isfinite(o.seconds) || o.seconds<=0) throw std::runtime_error("Time must be positive and finite");
    return o;
}
inline void configure(oc::SimpleSetupPtr &ss, const oc::ODESolver::ODE &ode,
                      const oc::ODESolver::PostPropagationEvent &post) {
    auto si=ss->getSpaceInformation();
    auto solver=std::make_shared<oc::ODEBasicSolver<>>(si, ode, 0.001);
    ss->setStatePropagator(oc::ODESolver::getStatePropagator(solver, post));
    si->setPropagationStepSize(0.02);
    si->setMinMaxControlDuration(1,25); // 0.02--0.50 seconds, forward propagation
}
inline int plan(oc::SimpleSetupPtr &ss, const Options &o, const std::string &system) {
    if (o.planner=="RRT") {
        auto p=std::make_shared<oc::RRT>(ss->getSpaceInformation()); p->setGoalBias(0.1); ss->setPlanner(p);
    } else {
        auto p=std::make_shared<oc::KPIECE1>(ss->getSpaceInformation()); p->setGoalBias(0.1); ss->setPlanner(p);
    }
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
    return 0;
}
