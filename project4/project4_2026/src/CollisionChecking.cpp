// Winson Lin (wl85), Darshon Singh (ds161).
#include "CollisionChecking.h"
#include <algorithm>
#include <array>
#include <cmath>
bool isValidPoint(double x,double y,const std::vector<Rectangle> &obstacles) {
    if (!std::isfinite(x) || !std::isfinite(y)) return false;
    for(const auto &r:obstacles) if(x>=r.x && x<=r.x+r.width && y>=r.y && y<=r.y+r.height) return false;
    return true;
}
bool isValidCircle(double x,double y,double radius,const std::vector<Rectangle> &obstacles) {
    if(!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(radius) || radius<0) return false;
    for(const auto &r:obstacles) {
        double dx=x-std::clamp(x,r.x,r.x+r.width),dy=y-std::clamp(y,r.y,r.y+r.height);
        if(dx*dx+dy*dy<=radius*radius) return false;
    }
    return true;
}
bool isValidSquare(double x,double y,double theta,double side,const std::vector<Rectangle> &obstacles) {
    if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(theta)||!std::isfinite(side)||side<0) return false;
    const double c=std::cos(theta),s=std::sin(theta),h=side/2;
    const std::array<std::array<double,2>,4> axes={{{1,0},{0,1},{c,s},{-s,c}}};
    for(const auto &r:obstacles) {
        double dx=x-(r.x+r.width/2),dy=y-(r.y+r.height/2); bool separated=false;
        for(const auto &a:axes) {
            double rs=h*(std::abs(a[0]*c+a[1]*s)+std::abs(-a[0]*s+a[1]*c));
            double rr=std::abs(a[0])*r.width/2+std::abs(a[1])*r.height/2;
            if(std::abs(dx*a[0]+dy*a[1])>rs+rr) { separated=true; break; }
        }
        if(!separated) return false;
    }
    return true;
}
