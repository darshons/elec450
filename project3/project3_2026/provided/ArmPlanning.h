// Provided infrastructure. Students implement src/RTP.h and src/RTP.cpp.
#pragma once

#include "CollisionChecking3D.h"

#include <Eigen/Geometry>
#include <array>
#include <tuple>
#include <vector>

using Vec3 = std::array<double, 3>;
using JointSpec = std::tuple<int, int, int, Vec3, Vec3, Vec3>;
using LinkSphereSpec = std::tuple<int, Vec3, double>;
using SphereSpec = std::tuple<Vec3, double>;
using BoxSpec = std::tuple<Vec3, Vec3, std::array<double, 4>>;

class ArmModel
{
public:
    ArmModel(const std::vector<JointSpec> &joints,
             const std::vector<LinkSphereSpec> &spheres,
             std::vector<double> lower, std::vector<double> upper);

    std::vector<Sphere3D> worldSpheres(const std::vector<double> &q) const;
    bool withinBounds(const std::vector<double> &q) const;

    const std::vector<double> lower;
    const std::vector<double> upper;

private:
    struct Joint
    {
        int parent, child, qIndex;
        Eigen::Isometry3d origin;
        Eigen::Vector3d axis;
    };
    std::vector<Joint> joints_;
    std::vector<LinkSphereSpec> spheres_;
    std::size_t linkCount_ = 1;
};

class ArmWorld
{
public:
    ArmWorld(const ArmModel &model, const std::vector<SphereSpec> &spheres,
             const std::vector<BoxSpec> &boxes);
    bool isValid(const std::vector<double> &q) const;
    double clearance(const std::vector<double> &q) const;

private:
    const ArmModel &model_;
    std::vector<Sphere3D> spheres_;
    std::vector<Box3D> boxes_;
};
