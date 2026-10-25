// Model regression checks: analytical cases, validity, projections and propagation.
#define PROJECT4_NO_MAIN
#include "Project4Pendulum.cpp"
#include "Project4Car.cpp"
#include <functional>
void require(bool ok,const char *message) { if(!ok) throw std::runtime_error(message); }
int main() {
    auto pend=createPendulum(3); pend->setup(); auto si=pend->getSpaceInformation();
    auto *u=si->allocControl(); auto *uv=u->as<oc::RealVectorControlSpace::ControlType>();
    uv->values[0]=0; oc::ODESolver::StateType dq;
    pendulumODE({0,2},u,dq); require(std::abs(dq[0]-2)<1e-12 && std::abs(dq[1]+9.81)<1e-12,"Pendulum horizontal gravity");
    pendulumODE({-PI/2,0},u,dq); require(std::abs(dq[1])<1e-12,"Hanging equilibrium");
    ob::ScopedState<> q(pend->getStateSpace()),r(pend->getStateSpace()); q[0]=-PI/2; q[1]=10.01;
    require(!si->isValid(q.get()),"Reject excessive pendulum velocity");
    q[1]=-10.01; require(!si->isValid(q.get()),"Reject negative excessive velocity");
    q[1]=0; si->getStatePropagator()->propagate(q.get(),u,0.5,r.get());
    require(si->distance(q.get(),r.get())<1e-10,"Zero torque equilibrium propagation");
    Eigen::VectorXd p(2); PendulumProjection pp(pend->getStateSpace().get()); pp.project(q.get(),p);
    require(std::abs(p[0]+PI/2)<1e-12 && p[1]==0,"Pendulum projection"); si->freeControl(u);
    std::vector<Rectangle> obs; makeStreet(obs); auto car=createCar(obs); car->setup(); auto cs=car->getSpaceInformation();
    auto *cu=cs->allocControl(); auto *cv=cu->as<oc::RealVectorControlSpace::ControlType>(); cv->values[0]=0;cv->values[1]=1;
    carODE({1,2,PI/2,-2},cu,dq); require(std::abs(dq[0])<1e-12 && std::abs(dq[1]+2)<1e-12 && dq[2]==0 && dq[3]==1,"Car reverse dynamics/control order");
    ob::ScopedState<> a(car->getStateSpace()),b(car->getStateSpace()); a[0]=1; a[1]=1;a[2]=0;a[3]=0;
    cs->getStatePropagator()->propagate(a.get(),cu,0.5,b.get());
    require(std::abs(b[0]-1.125)<1e-9 && std::abs(b[3]-0.5)<1e-9,"Car constant acceleration propagation");
    a[3]=2.01;require(!cs->isValid(a.get()),"Reject car speed"); a[3]=-2.01;require(!cs->isValid(a.get()),"Reject reverse speed");
    a[3]=0;a[0]=3;a[1]=1;require(!cs->isValid(a.get()),"Obstacle boundary collides");
    a[0]=-0.1;require(!cs->isValid(a.get()),"Reject workspace bounds");
    a[0]=1; a[1]=2; a[3]=-1; Eigen::VectorXd cp(3); CarProjection cproj(car->getStateSpace().get()); cproj.project(a.get(),cp);
    require(cp[0]==1 && cp[1]==2 && cp[2]==-1,"Car dynamic projection"); cs->freeControl(cu);
    std::vector<Rectangle> box={{0,0,1,1}};
    require(!isValidPoint(0,0,box) && isValidPoint(-0.01,0,box),"Point contacts");
    require(!isValidCircle(2,0.5,1,box) && isValidCircle(2.01,0.5,1,box),"Circle tangency");
    require(!isValidSquare(1.5,0.5,0,1,box) && isValidSquare(1.51,0.5,0,1,box),"Square tangency");
    require(!isValidSquare(1.6,0.5,PI/4,1,box),"Rotated square collision");
    require(isValidSquare(2,2,PI/4,1,box),"Rotated square separated");
    std::cout << "All model checks passed\n";
}
