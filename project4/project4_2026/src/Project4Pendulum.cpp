// Darshon Singh (ds161), Winson Lin (wl85).
#include "Planning.h"
#include <ompl/base/ProjectionEvaluator.h>
class PendulumProjection : public ob::ProjectionEvaluator {
public:
    explicit PendulumProjection(const ob::StateSpace *space): ProjectionEvaluator(space) {}
    unsigned getDimension() const override { return 2; }
    void defaultCellSizes() override { cellSizes_={0.25,0.5}; }
    void project(const ob::State *s, Eigen::Ref<Eigen::VectorXd> p) const override {
        const auto *q=s->as<ob::CompoundState>();
        p[0]=q->as<ob::SO2StateSpace::StateType>(0)->value;
        p[1]=q->as<ob::RealVectorStateSpace::StateType>(1)->values[0];
    }
};
void pendulumODE(const oc::ODESolver::StateType &q, const oc::Control *control, oc::ODESolver::StateType &dq) {
    const double torque=control->as<oc::RealVectorControlSpace::ControlType>()->values[0];
    dq.resize(2); dq[0]=q[1]; dq[1]=-9.81*std::cos(q[0])+torque;
}
oc::SimpleSetupPtr createPendulum(double torque) {
    auto angle=std::make_shared<ob::SO2StateSpace>();
    auto velocity=std::make_shared<ob::RealVectorStateSpace>(1);
    ob::RealVectorBounds vb(1); vb.setLow(-10); vb.setHigh(10); velocity->setBounds(vb);
    auto space=std::make_shared<ob::CompoundStateSpace>();
    space->addSubspace(angle,1.0); space->addSubspace(velocity,1.0);
    space->registerDefaultProjection(std::make_shared<PendulumProjection>(space.get()));
    auto controls=std::make_shared<oc::RealVectorControlSpace>(space,1);
    ob::RealVectorBounds ub(1); ub.setLow(-torque); ub.setHigh(torque); controls->setBounds(ub);
    auto ss=std::make_shared<oc::SimpleSetup>(controls);
    ss->setStateValidityChecker([space](const ob::State *s){ return space->satisfiesBounds(s); });
    configure(ss,pendulumODE,[angle](const ob::State *,const oc::Control *,double,ob::State *s){
        angle->enforceBounds(s->as<ob::CompoundState>()->components[0]); // wrap angle only; never clamp speed
    });
    ob::ScopedState<> start(space),goal(space);
    start[0]=-PI/2; start[1]=0; goal[0]=PI/2; goal[1]=0;
    ss->setStartAndGoalStates(start,goal,0.15);
    return ss;
}
#ifndef PROJECT4_NO_MAIN
int main(int argc,char **argv) {
    try { auto o=options(argc,argv,true); ompl::RNG::setSeed(o.seed); auto ss=createPendulum(o.torque); return plan(ss,o,"pendulum"); }
    catch(const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
#endif
