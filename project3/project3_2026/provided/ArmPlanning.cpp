#include "ArmPlanning.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace
{
Eigen::Vector3d vector(const Vec3 &v) { return {v[0], v[1], v[2]}; }
}

ArmModel::ArmModel(const std::vector<JointSpec> &joints,
                   const std::vector<LinkSphereSpec> &spheres,
                   std::vector<double> lowerLimits, std::vector<double> upperLimits)
    : lower(std::move(lowerLimits)), upper(std::move(upperLimits)), spheres_(spheres)
{
    for (const auto &[parent, child, qIndex, xyz, rpy, axis] : joints)
    {
        Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
        origin.translation() = vector(xyz);
        origin.linear() = (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
        joints_.push_back({parent, child, qIndex, origin, vector(axis)});
        linkCount_ = std::max(linkCount_, static_cast<std::size_t>(child + 1));
    }
}

std::vector<Sphere3D> ArmModel::worldSpheres(const std::vector<double> &q) const
{
    std::vector<Eigen::Isometry3d> transforms(linkCount_, Eigen::Isometry3d::Identity());
    for (const Joint &joint : joints_)
    {
        transforms[joint.child] = transforms[joint.parent] * joint.origin;
        if (joint.qIndex >= 0)
            transforms[joint.child].rotate(Eigen::AngleAxisd(q[joint.qIndex], joint.axis));
    }
    std::vector<Sphere3D> result;
    result.reserve(spheres_.size());
    for (const auto &[link, center, radius] : spheres_)
    {
        const Eigen::Vector3d position = transforms[link] * vector(center);
        result.push_back({position.x(), position.y(), position.z(), radius});
    }
    return result;
}

bool ArmModel::withinBounds(const std::vector<double> &q) const
{
    for (std::size_t i = 0; i < lower.size(); ++i)
        if (q[i] < lower[i] || q[i] > upper[i])
            return false;
    return true;
}

ArmWorld::ArmWorld(const ArmModel &model, const std::vector<SphereSpec> &spheres,
                   const std::vector<BoxSpec> &boxes) : model_(model)
{
    for (const auto &[center, radius] : spheres)
        spheres_.push_back({center[0], center[1], center[2], radius});
    for (const auto &[center, half, xyzw] : boxes)
    {
        const Eigen::Matrix3d rotation =
            Eigen::Quaterniond(xyzw[3], xyzw[0], xyzw[1], xyzw[2]).normalized().toRotationMatrix();
        boxes_.push_back({center[0], center[1], center[2], half[0], half[1], half[2],
                          rotation(0, 0), rotation(0, 1), rotation(0, 2),
                          rotation(1, 0), rotation(1, 1), rotation(1, 2),
                          rotation(2, 0), rotation(2, 1), rotation(2, 2)});
    }
}

bool ArmWorld::isValid(const std::vector<double> &q) const
{
    return model_.withinBounds(q) && is_valid(model_.worldSpheres(q), spheres_, boxes_);
}

double ArmWorld::clearance(const std::vector<double> &q) const
{
    double distance = std::numeric_limits<double>::infinity();
    for (const Sphere3D &sphere : model_.worldSpheres(q))
    {
        for (const Sphere3D &obstacle : spheres_)
            distance = std::min(distance,
                std::hypot(sphere.x - obstacle.x, sphere.y - obstacle.y, sphere.z - obstacle.z)
                    - sphere.r - obstacle.r);
        for (const Box3D &box : boxes_)
        {
            const double dx = sphere.x - box.x, dy = sphere.y - box.y, dz = sphere.z - box.z;
            const double x = box.r00 * dx + box.r10 * dy + box.r20 * dz;
            const double y = box.r01 * dx + box.r11 * dy + box.r21 * dz;
            const double z = box.r02 * dx + box.r12 * dy + box.r22 * dz;
            const double ox = std::abs(x) - box.hx;
            const double oy = std::abs(y) - box.hy;
            const double oz = std::abs(z) - box.hz;
            const double outside = std::hypot(std::max(ox, 0.0), std::max(oy, 0.0), std::max(oz, 0.0));
            const double inside = std::min(std::max({ox, oy, oz}), 0.0);
            distance = std::min(distance, outside + inside - sphere.r);
        }
    }
    return distance;
}
