///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2 - 3D extension
//////////////////////////////////////
#ifndef COLLISION_CHECKING_3D_H_
#define COLLISION_CHECKING_3D_H_

#include <vector>

// A sphere in world coordinates: either a robot collision sphere or a
// spherical obstacle.
struct Sphere3D
{
    double x, y, z, r;
};

// An oriented box (cuboid) obstacle in world coordinates.
struct Box3D
{
    // Center of the box, in world coordinates.
    double x, y, z;
    // Half-extents along the box's own local axes.
    double hx, hy, hz;
    // Rotation matrix mapping the box's local axes to world axes
    // (row-major: world_vector = R * local_vector). Identity for an
    // axis-aligned box.
    double r00 = 1.0, r01 = 0.0, r02 = 0.0;
    double r10 = 0.0, r11 = 1.0, r12 = 0.0;
    double r20 = 0.0, r21 = 0.0, r22 = 1.0;
};

// Intersect two spheres. Returns true if they overlap.
bool spheresCollide(const Sphere3D &a, const Sphere3D &b);

// Intersect a sphere with an (arbitrarily oriented) box. Returns true if they overlap.
bool sphereBoxCollide(const Sphere3D &sphere, const Box3D &box);

// Check a robot (represented as a list of collision spheres) against an
// environment of sphere and box obstacles. Returns true if the robot does
// not collide with anything.
bool is_valid(
    const std::vector<Sphere3D> &spheres,
    const std::vector<Sphere3D> &envSpheres,
    const std::vector<Box3D> &envBoxes);

#endif
