///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2 - 3D Collision Checking
//////////////////////////////////////

#include "CollisionChecking3D.h"

#include <algorithm>

// Intersect two spheres. Returns true if they collide.
bool spheresCollide(const Sphere3D &a, const Sphere3D &b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    const double radius = a.r + b.r;
    return dx * dx + dy * dy + dz * dz <= radius * radius;
}

// Intersect a sphere with an oriented box. Returns true if they collide.
bool sphereBoxCollide(const Sphere3D &sphere, const Box3D &box)
{
    const double dx = sphere.x - box.x;
    const double dy = sphere.y - box.y;
    const double dz = sphere.z - box.z;
    const double x = box.r00 * dx + box.r10 * dy + box.r20 * dz;
    const double y = box.r01 * dx + box.r11 * dy + box.r21 * dz;
    const double z = box.r02 * dx + box.r12 * dy + box.r22 * dz;
    const double closestDx = x - std::clamp(x, -box.hx, box.hx);
    const double closestDy = y - std::clamp(y, -box.hy, box.hy);
    const double closestDz = z - std::clamp(z, -box.hz, box.hz);
    return closestDx * closestDx + closestDy * closestDy + closestDz * closestDz
        <= sphere.r * sphere.r;
}

// Check a robot (represented as a list of collision spheres) against an
// environment of sphere and box obstacles. Returns true if the robot does
// not collide with anything.
bool is_valid(
    const std::vector<Sphere3D> &spheres,
    const std::vector<Sphere3D> &envSpheres,
    const std::vector<Box3D> &envBoxes)
{
    for (const Sphere3D &sphere : spheres)
    {
        for (const Sphere3D &obstacle : envSpheres)
        {
            if (spheresCollide(sphere, obstacle))
                return false;
        }
        for (const Box3D &obstacle : envBoxes)
        {
            if (sphereBoxCollide(sphere, obstacle))
                return false;
        }
    }
    return true;
}
