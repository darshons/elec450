// Winson Lin (wl85), Darshon Singh (ds161).
#include "Planning.h"
#include "CollisionChecking.h"
#include <ompl/base/ProjectionEvaluator.h>
class CarProjection : public ob::ProjectionEvaluator {
public:
    explicit CarProjection(const ob::StateSpace *space): ProjectionEvaluator(space) {}
    unsigned getDimension() const override { return 3; }
    void defaultCellSizes() override { cellSizes_={0.5,0.5,0.5}; }
    void project(const ob::State *s,Eigen::Ref<Eigen::VectorXd> p) const override {
        const auto *q=s->as<ob::CompoundState>(); const auto *pose=q->as<ob::SE2StateSpace::StateType>(0);
        p[0]=pose->getX(); p[1]=pose->getY(); p[2]=q->as<ob::RealVectorStateSpace::StateType>(1)->values[0];
    }
};
void carODE(const oc::ODESolver::StateType &q,const oc::Control *control,oc::ODESolver::StateType &dq) {
    const auto *u=control->as<oc::RealVectorControlSpace::ControlType>();
    dq.resize(4); dq[0]=q[3]*std::cos(q[2]); dq[1]=q[3]*std::sin(q[2]); dq[2]=u->values[0]; dq[3]=u->values[1];
}
void makeStreet(std::vector<Rectangle> &obstacles) {
    // Three building blocks with alternating north/south passages; closed boundaries collide.
    obstacles={{3,0,1,6.5},{6,3.5,1,6.5},{9,0,1,6.5}};
}
oc::SimpleSetupPtr createCar(const std::vector<Rectangle> &obstacles) {
    auto pose=std::make_shared<ob::SE2StateSpace>();
    ob::RealVectorBounds xy(2); xy.setLow(0); xy.setHigh(0,12); xy.setHigh(1,10); pose->setBounds(xy);
    auto speed=std::make_shared<ob::RealVectorStateSpace>(1);
    ob::RealVectorBounds vb(1); vb.setLow(-2); vb.setHigh(2); speed->setBounds(vb);
    auto space=std::make_shared<ob::CompoundStateSpace>(); space->addSubspace(pose,1); space->addSubspace(speed,1);
    space->registerDefaultProjection(std::make_shared<CarProjection>(space.get()));
    auto controls=std::make_shared<oc::RealVectorControlSpace>(space,2);
    ob::RealVectorBounds ub(2); ub.setLow(-1); ub.setHigh(1); controls->setBounds(ub);
    auto ss=std::make_shared<oc::SimpleSetup>(controls);
    ss->setStateValidityChecker([space,obstacles](const ob::State *s){
        auto *p=s->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0);
        return space->satisfiesBounds(s) && isValidPoint(p->getX(),p->getY(),obstacles);
    });
    configure(ss,carODE,[](const ob::State *,const oc::Control *,double,ob::State *s){
        auto *pose=s->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0);
        ob::SO2StateSpace angle; angle.enforceBounds(pose->as<ob::SO2StateSpace::StateType>(1));
    });
    ob::ScopedState<> start(space),goal(space);
    start[0]=1; start[1]=1; start[2]=PI/2; start[3]=0;
    goal[0]=11; goal[1]=9; goal[2]=PI/2; goal[3]=0;
    ss->setStartAndGoalStates(start,goal,0.5);
    return ss;
}
void planCar(oc::SimpleSetupPtr &ss, int choice) {
    Options o; o.planner=plannerName(choice); o.seconds=120; o.output="car_path.json";
    plan(ss,o,"car");
}
void benchmarkCar(oc::SimpleSetupPtr &ss) {
    Options o; o.seconds=60; benchmark(ss,o,"car");
}
#ifndef PROJECT4_NO_MAIN
int main(int argc,char **argv) {
    try {
        std::vector<Rectangle> obs; makeStreet(obs);
        if (argc > 1) { // scripted runs: see --help
            auto o=options(argc,argv); ompl::RNG::setSeed(o.seed); auto ss=createCar(obs);
            return o.benchmark ? benchmark(ss,o,"car") : plan(ss,o,"car");
        }
        int choice=menu("Plan or Benchmark? \n (1) Plan\n (2) Benchmark\n",1,2);
        auto ss=createCar(obs);
        if (choice==1) planCar(ss,menu("What Planner? \n (1) RRT\n (2) KPIECE1\n (3) RG-RRT\n",1,3));
        else benchmarkCar(ss);
        return 0;
    }
    catch(const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
#endif
