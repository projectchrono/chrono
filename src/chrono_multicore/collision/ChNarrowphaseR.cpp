// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Implementation file for ChCNarrowphaseR.
//
// =============================================================================

#include "chrono/collision/ChCollisionModel.h"

#include "chrono_multicore/math/ChMulticoreMath.h"
#include "chrono_multicore/collision/ChNarrowphaseR.h"
#include "chrono_multicore/collision/ChNarrowphaseRUtils.h"

namespace chrono {
namespace collision {

// Fictitious radius of curvature for collision with a corner or an edge.
static real edge_radius = 0.1;

void SetDefaultEdgeRadius(real radius) {
    edge_radius = radius;
}

real GetDefaultEdgeRadius() {
    return edge_radius;
}

// This is the main worker function for narrow phase check of the collision
// between two candidate shapes.  Each candidate pair of shapes can result in
// 0, 1, or more contacts.  For each actual contact, we calculate various
// geometrical quantities and load them in the output arguments (starting from
// the given addresses)
//   - ct_pt1:      contact point on first shape (in global frame)
//   - ct_pt2:      contact point on second shape (in global frame)
//   - ct_depth:    penetration distance (negative if overlap exists)
//   - ct_norm:     contact normal, from ct_pt2 to ct_pt1 (in global frame)
//   - ct_eff_rad:  effective contact radius
// Note that we also report collisions for which the distance between the two
// shapes is at most 'separation' (typically twice the collision envelope).
// In these cases, the corresponding ct_depth is a positive value.
// This function returns true if it was able to determine the collision state
// for the given pair of shapes and false if the shape types are not supported.

bool RCollision(const ConvexBase* shapeA,  // first candidate shape
                const ConvexBase* shapeB,  // second candidate shape
                real separation,           // maximum separation
                real3* ct_norm,            // [output] contact normal (per contact pair)
                real3* ct_pt1,             // [output] point on shape1 (per contact pair)
                real3* ct_pt2,             // [output] point on shape2 (per contact pair)
                real* ct_depth,            // [output] penetration depth (per contact pair)
                real* ct_eff_rad,          // [output] effective contact radius (per contact pair)
                int& nC)                   // [output] number of contacts found
{
    // Special-case the collision detection based on the types of the
    // two potentially colliding shapes.

    nC = 0;

    if (shapeA->Type() == ChCollisionShape::Type::SPHERE && shapeB->Type() == ChCollisionShape::Type::SPHERE) {
        if (sphere_sphere(shapeA->A(), shapeA->Radius(), shapeB->A(), shapeB->Radius(), separation, *ct_norm, *ct_depth,
                          *ct_pt1, *ct_pt2, *ct_eff_rad)) {
            nC = 1;
        }
        return true;
    }

    if (shapeA->Type() == ChCollisionShape::Type::CAPSULE && shapeB->Type() == ChCollisionShape::Type::SPHERE) {
        if (capsule_sphere(shapeA->A(), shapeA->R(), shapeA->Capsule().x, shapeA->Capsule().y, shapeB->A(),
                           shapeB->Radius(), separation, *ct_norm, *ct_depth, *ct_pt1, *ct_pt2, *ct_eff_rad)) {
            nC = 1;
        }
        return true;
    }

    if (shapeA->Type() == ChCollisionShape::Type::SPHERE && shapeB->Type() == ChCollisionShape::Type::CAPSULE) {
        if (capsule_sphere(shapeB->A(), shapeB->R(), shapeB->Capsule().x, shapeB->Capsule().y, shapeA->A(),
                           shapeA->Radius(), separation, *ct_norm, *ct_depth, *ct_pt2, *ct_pt1, *ct_eff_rad)) {
            *ct_norm = -(*ct_norm);
            nC = 1;
        }
        return true;
    }

    if (shapeA->Type() == ChCollisionShape::Type::CYLINDER && shapeB->Type() == ChCollisionShape::Type::SPHERE) {
        if (cylinder_sphere(shapeA->A(), shapeA->R(), shapeA->Box().x, shapeA->Box().y, shapeB->A(), shapeB->Radius(),
                            separation, *ct_norm, *ct_depth, *ct_pt1, *ct_pt2, *ct_eff_rad)) {
            nC = 1;
        }
        return true;
    }

    if (shapeA->Type() == ChCollisionShape::Type::SPHERE && shapeB->Type() == ChCollisionShape::Type::CYLINDER) {
        if (cylinder_sphere(shapeB->A(), shapeB->R(), shapeB->Box().x, shapeB->Box().y, shapeA->A(), shapeA->Radius(),
                            separation, *ct_norm, *ct_depth, *ct_pt2, *ct_pt1, *ct_eff_rad)) {
            *ct_norm = -(*ct_norm);
            nC = 1;
        }
        return true;
    }

    if (shapeA->Type() == ChCollisionShape::Type::ROUNDEDCYL && shapeB->Type() == ChCollisionShape::Type::SPHERE) {
        if (roundedcyl_sphere(shapeA->A(), shapeA->R(), shapeA->Rbox().x, shapeA->Rbox().y, shapeA->Rbox().w,
                              shapeB->A(), shapeB->Radius(), separation, *ct_norm, *ct_depth, *ct_pt1, *ct_pt2,
                              *ct_eff_rad)) {
            nC = 1;
        }
        return true;
    }

    if (shapeA->Type() == ChCollisionShape::Type::SPHERE && shapeB->Type() == ChCollisionShape::Type::ROUNDEDCYL) {
        if (roundedcyl_sphere(shapeB->A(), shapeB->R(), shapeB->Rbox().x, shapeB->Rbox().y, shapeB->Rbox().w,
                              shapeA->A(), shapeA->Radius(), separation, *ct_norm, *ct_depth, *ct_pt2, *ct_pt1,
                              *ct_eff_rad)) {
            *ct_norm = -(*ct_norm);
            nC = 1;
        }
        return true;
    }

    if (shapeA->Type() == ChCollisionShape::Type::BOX && shapeB->Type() == ChCollisionShape::Type::SPHERE) {
        if (box_sphere(shapeA->A(), shapeA->R(), shapeA->Box(), shapeB->A(), shapeB->Radius(), separation, *ct_norm,
                       *ct_depth, *ct_pt1, *ct_pt2, *ct_eff_rad)) {
            nC = 1;
        }
        return true;
    }

    if (shapeA->Type() == ChCollisionShape::Type::SPHERE && shapeB->Type() == ChCollisionShape::Type::BOX) {
        if (box_sphere(shapeB->A(), shapeB->R(), shapeB->Box(), shapeA->A(), shapeA->Radius(), separation, *ct_norm,
                       *ct_depth, *ct_pt2, *ct_pt1, *ct_eff_rad)) {
            *ct_norm = -(*ct_norm);
            nC = 1;
        }
        return true;
    }

    if (shapeA->Type() == ChCollisionShape::Type::ROUNDEDBOX && shapeB->Type() == ChCollisionShape::Type::SPHERE) {
        if (roundedbox_sphere(shapeA->A(), shapeA->R(), shapeA->Rbox(), shapeA->Rbox().w, shapeB->A(), shapeB->Radius(),
                              separation, *ct_norm, *ct_depth, *ct_pt1, *ct_pt2, *ct_eff_rad)) {
            nC = 1;
        }
        return true;
    }

    if (shapeA->Type() == ChCollisionShape::Type::SPHERE && shapeB->Type() == ChCollisionShape::Type::ROUNDEDBOX) {
        if (roundedbox_sphere(shapeB->A(), shapeB->R(), shapeB->Rbox(), shapeB->Rbox().w, shapeA->A(), shapeA->Radius(),
                              separation, *ct_norm, *ct_depth, *ct_pt2, *ct_pt1, *ct_eff_rad)) {
            *ct_norm = -(*ct_norm);
            nC = 1;
        }
        return true;
    }

    if (shapeA->Type() == ChCollisionShape::Type::TRIANGLE && shapeB->Type() == ChCollisionShape::Type::SPHERE) {
        if (triangle_sphere(shapeA->Triangles()[0], shapeA->Triangles()[1], shapeA->Triangles()[2], shapeB->A(),
                            shapeB->Radius(), separation, *ct_norm, *ct_depth, *ct_pt1, *ct_pt2, *ct_eff_rad)) {
            nC = 1;
        }
        return true;
    }

    if (shapeA->Type() == ChCollisionShape::Type::SPHERE && shapeB->Type() == ChCollisionShape::Type::TRIANGLE) {
        if (triangle_sphere(shapeB->Triangles()[0], shapeB->Triangles()[1], shapeB->Triangles()[2], shapeA->A(),
                            shapeA->Radius(), separation, *ct_norm, *ct_depth, *ct_pt2, *ct_pt1, *ct_eff_rad)) {
            *ct_norm = -(*ct_norm);
            nC = 1;
        }
        return true;
    }

    if (shapeA->Type() == ChCollisionShape::Type::CAPSULE && shapeB->Type() == ChCollisionShape::Type::CAPSULE) {
        nC = capsule_capsule(shapeA->A(), shapeA->R(), shapeA->Capsule().x, shapeA->Capsule().y, shapeB->A(),
                             shapeB->R(), shapeB->Capsule().x, shapeB->Capsule().y, separation, ct_norm, ct_depth,
                             ct_pt1, ct_pt2, ct_eff_rad);
        return true;
    }

    if (shapeA->Type() == ChCollisionShape::Type::BOX && shapeB->Type() == ChCollisionShape::Type::CAPSULE) {
        nC = box_capsule(shapeA->A(), shapeA->R(), shapeA->Box(), shapeB->A(), shapeB->R(), shapeB->Capsule().x,
                         shapeB->Capsule().y, separation, ct_norm, ct_depth, ct_pt1, ct_pt2, ct_eff_rad);
        return true;
    }

    if (shapeA->Type() == ChCollisionShape::Type::CAPSULE && shapeB->Type() == ChCollisionShape::Type::BOX) {
        nC = box_capsule(shapeB->A(), shapeB->R(), shapeB->Box(), shapeA->A(), shapeA->R(), shapeA->Capsule().x,
                         shapeA->Capsule().y, separation, ct_norm, ct_depth, ct_pt2, ct_pt1, ct_eff_rad);
        for (int i = 0; i < nC; i++) {
            *(ct_norm + i) = -(*(ct_norm + i));
        }
        return true;
    }

    if (shapeA->Type() == ChCollisionShape::Type::BOX && shapeB->Type() == ChCollisionShape::Type::CYLSHELL) {
        nC = box_cylshell(shapeA->A(), shapeA->R(), shapeA->Box(), shapeB->A(), shapeB->R(), shapeB->Cylshell().x,
                          shapeB->Cylshell().y, separation, ct_norm, ct_depth, ct_pt1, ct_pt2, ct_eff_rad);
        return true;
    }

    if (shapeA->Type() == ChCollisionShape::Type::CYLSHELL && shapeB->Type() == ChCollisionShape::Type::BOX) {
        nC = box_cylshell(shapeB->A(), shapeB->R(), shapeB->Box(), shapeA->A(), shapeA->R(), shapeA->Cylshell().x,
                          shapeA->Cylshell().y, separation, ct_norm, ct_depth, ct_pt2, ct_pt1, ct_eff_rad);
        for (int i = 0; i < nC; i++) {
            *(ct_norm + i) = -(*(ct_norm + i));
        }
        return true;
    }

    if (shapeA->Type() == ChCollisionShape::Type::BOX && shapeB->Type() == ChCollisionShape::Type::BOX) {
        nC = box_box(shapeA->A(), shapeA->R(), shapeA->Box(), shapeB->A(), shapeB->R(), shapeB->Box(), separation,
                     ct_norm, ct_depth, ct_pt1, ct_pt2, ct_eff_rad);

        ////std::cout << nC << std::endl;
        ////for (int j = 0; j < nC; j++) {
        ////    real3 ptT = *(ct_pt1 + j);
        ////    real3 ptO = *(ct_pt2 + j);
        ////    real3 nrm = *(ct_norm + j);
        ////    real depth = *(ct_depth + j);
        ////    real er = *(ct_eff_rad + j);
        ////    std::cout << "  " << ptT.x << "  " << ptT.y << "  " << ptT.z << std::endl;
        ////    std::cout << "  " << ptO.x << "  " << ptO.y << "  " << ptO.z << std::endl;
        ////    std::cout << "  " << nrm.x << "  " << nrm.y << "  " << nrm.z << std::endl;
        ////    std::cout << "  " << depth << std::endl;
        ////    std::cout << "  " << er << std::endl;
        ////}

        return true;
    }

    // Contact could not be checked using this CD algorithm
    return false;
}

// =============================================================================
//              SPHERE - SPHERE

// Sphere-sphere narrow phase collision detection.
// In:  sphere centered at pos1 with radius1
//      sphere centered at pos2 with radius2

bool sphere_sphere(const real3& pos1,
                   const real& radius1,
                   const real3& pos2,
                   const real& radius2,
                   const real& separation,
                   real3& norm,
                   real& depth,
                   real3& pt1,
                   real3& pt2,
                   real& eff_radius) {
    real3 delta = pos2 - pos1;
    real dist2 = Dot(delta, delta);
    real radSum = radius1 + radius2;
    real radSum_s = radSum + separation;

    // If the two sphere centers are separated by more than the sum of their
    // radii (plus the separation value), there is no contact. Also ignore
    // contact if the two centers almost coincide, in which case we cannot
    // decide on the direction.
    if (dist2 >= radSum_s * radSum_s || dist2 < 1e-12)
        return false;

    // Generate contact information.
    real dist = Sqrt(dist2);
    norm = delta / dist;
    pt1 = pos1 + norm * radius1;
    pt2 = pos2 - norm * radius2;
    depth = dist - radSum;
    eff_radius = radius1 * radius2 / radSum;

    return true;
}

// =============================================================================
//              CAPSULE - SPHERE

// Capsule-sphere narrow phase collision detection.
// In:  capsule at pos1, with orientation rot1
//              capsule has radius1 and half-length hlen1 (in Y direction)
//      sphere centered at pos2 with radius2

bool capsule_sphere(const real3& pos1,
                    const quaternion& rot1,
                    const real& radius1,
                    const real& hlen1,
                    const real3& pos2,
                    const real& radius2,
                    const real& separation,
                    real3& norm,
                    real& depth,
                    real3& pt1,
                    real3& pt2,
                    real& eff_radius) {
    // Working in the global frame, project the sphere center onto the
    // capsule's centerline and clamp the resulting location to the extent
    // of the capsule length.
    real3 V = AMatV(rot1);
    real alpha = Dot(pos2 - pos1, V);
    alpha = Clamp(alpha, -hlen1, hlen1);

    real3 loc = pos1 + alpha * V;

    // Treat the capsule as a sphere centered at the above location. If the
    // sphere center is farther away than the sum of radii plus the separation
    // value, there is no contact. Also, ignore contact if the two centers
    // almost coincide, in which case we couldn't decide on the proper contact
    // direction.
    real radSum = radius1 + radius2;
    real radSum_s = radSum + separation;
    real3 delta = pos2 - loc;
    real dist2 = Dot(delta, delta);

    if (dist2 >= radSum_s * radSum_s || dist2 <= 1e-12f)
        return false;

    // Generate contact information.
    real dist = Sqrt(dist2);
    norm = delta / dist;
    pt1 = loc + norm * radius1;
    pt2 = pos2 - norm * radius2;
    depth = dist - radSum;
    eff_radius = radius1 * radius2 / radSum;

    return true;
}

// =============================================================================
//              CYLINDER - SPHERE

// Cylinder-sphere narrow phase collision detection.
// In:  cylinder at pos1, with orientation rot1
//              cylinder has radius1 and half-length hlen1 (in Y direction)
//      sphere centered at pos2 with radius2

bool cylinder_sphere(const real3& pos1,
                     const quaternion& rot1,
                     const real& radius1,
                     const real& hlen1,
                     const real3& pos2,
                     const real& radius2,
                     const real& separation,
                     real3& norm,
                     real& depth,
                     real3& pt1,
                     real3& pt2,
                     real& eff_radius) {
    // Express the sphere position in the frame of the cylinder.
    real3 spherePos = TransformParentToLocal(pos1, rot1, pos2);

    // Snap the sphere position to the surface of the cylinder.
    real3 cylPos = spherePos;
    uint code = snap_to_cylinder(radius1, hlen1, cylPos);

    // Quick return: no contact if the sphere center is inside the cylinder.
    if (code == 0)
        return false;

    // If the distance from the sphere center to the closest point is larger
    // than the sphere radius plus the separation value, there is no contact.
    // Also, ignore contact if the sphere center (almost) coincides with the
    // closest point, in which case we couldn't decide on the proper contact
    // direction.
    real3 delta = spherePos - cylPos;
    real dist2 = Dot(delta, delta);
    real radius2_s = radius2 + separation;

    if (dist2 >= radius2_s * radius2_s || dist2 <= 1e-12f)
        return false;

    // Generate contact information
    real dist = Sqrt(dist2);
    depth = dist - radius2;
    norm = Rotate(delta / dist, rot1);
    pt1 = TransformLocalToParent(pos1, rot1, cylPos);
    pt2 = pos2 - norm * radius2;

    switch (code) {
        case 1:
            eff_radius = radius2;
            break;
        case 2:
            eff_radius = radius1 * radius2 / (radius1 + radius2);
            break;
        case 3:
            eff_radius = radius2 * edge_radius / (radius2 + edge_radius);
            break;
    }

    return true;
}

// =============================================================================
//              ROUNDEDCYL - SPHERE

// RoundedCylinder-sphere narrow phase collision detection.
// In:  roundedcyl at pos1, with orientation rot1
//              roundedcyl has radius1 and half-length hlen1 (in Y direction)
//              radius of the sweeping sphere is srad1
//      sphere centered at pos2 with radius2

bool roundedcyl_sphere(const real3& pos1,
                       const quaternion& rot1,
                       const real& radius1,
                       const real& hlen1,
                       const real& srad1,
                       const real3& pos2,
                       const real& radius2,
                       const real& separation,
                       real3& norm,
                       real& depth,
                       real3& pt1,
                       real3& pt2,
                       real& eff_radius) {
    // Express the sphere position in the frame of the rounded cylinder.
    real3 spherePos = TransformParentToLocal(pos1, rot1, pos2);

    // Snap the sphere position to the surface of the skeleton cylinder.
    real3 cylPos = spherePos;
    uint code = snap_to_cylinder(radius1, hlen1, cylPos);

    // Quick return: no contact if the sphere center is inside the skeleton
    // cylinder.
    if (code == 0)
        return false;

    // Reduce the problem to the interaction between two spheres:
    //    (a) a sphere with radius srad1, centered at cylPos
    //    (b) a sphere with radius radius2, centered at spherePos
    // If the two sphere centers are farther away that the radii sum plus the
    // separation distance, there is no contact. Also, ignore contact if the
    // two centers almost coincide, in which case we couldn't decide on the
    // proper contact direction.
    real radSum = srad1 + radius2;
    real radSum_s = radSum + separation;
    real3 delta = spherePos - cylPos;
    real dist2 = Dot(delta, delta);

    if (dist2 >= radSum_s * radSum_s || dist2 <= 1e-12f)
        return false;

    // Generate contact information.
    real dist = Sqrt(dist2);
    depth = dist - radSum;
    norm = Rotate(delta / dist, rot1);
    pt2 = pos2 - norm * radius2;
    pt1 = pt2 - depth * norm;

    switch (code) {
        case 1:
            eff_radius = radius2;
            break;
        case 2:
            eff_radius = radius1 * radius2 / (radius1 + radius2);
            break;
        case 3:
            eff_radius = srad1 * radius2 / (srad1 + radius2);
            break;
    }

    return true;
}

// =============================================================================
//              BOX - SPHERE

// Box-sphere narrow phase collision detection.
// In:  box at position pos1, with orientation rot1, and half-dimensions hdims1
//      sphere centered at pos2 and with radius2

bool box_sphere(const real3& pos1,
                const quaternion& rot1,
                const real3& hdims1,
                const real3& pos2,
                const real& radius2,
                const real& separation,
                real3& norm,
                real& depth,
                real3& pt1,
                real3& pt2,
                real& eff_radius) {
    // Express the sphere position in the frame of the box.
    real3 spherePos = TransformParentToLocal(pos1, rot1, pos2);

    // Snap the sphere position to the surface of the box.
    real3 boxPos = spherePos;
    uint code = snap_to_box(hdims1, boxPos);

    // If the distance from the sphere center to the closest point is larger
    // than the sphere radius plus the separation value, there is no contact.
    // Also, ignore contact if the sphere center (almost) coincides with the
    // closest point, in which case we couldn't decide on the proper contact
    // direction.
    real3 delta = spherePos - boxPos;
    real dist2 = Dot(delta, delta);
    real radius2_s = radius2 + separation;

    if (dist2 >= radius2_s * radius2_s || dist2 <= 1e-12f)
        return false;

    // Generate contact information
    real dist = Sqrt(dist2);
    depth = dist - radius2;
    norm = Rotate(delta / dist, rot1);
    pt1 = TransformLocalToParent(pos1, rot1, boxPos);
    pt2 = pos2 - norm * radius2;

    if ((code != 1) && (code != 2) && (code != 4))
        eff_radius = radius2 * edge_radius / (radius2 + edge_radius);
    else
        eff_radius = radius2;

    return true;
}

// =============================================================================
//              ROUNDEDBOX - SPHERE

// RoundedBox-sphere narrow phase collision detection.
// In:  roundedbox at position pos1, with orientation rot1
//              roundedbox has half-dimensions hdims1
//              radius of the sweeping sphere is srad1
//      sphere centered at pos2 and with radius2

bool roundedbox_sphere(const real3& pos1,
                       const quaternion& rot1,
                       const real3& hdims1,
                       const real& srad1,
                       const real3& pos2,
                       const real& radius2,
                       const real& separation,
                       real3& norm,
                       real& depth,
                       real3& pt1,
                       real3& pt2,
                       real& eff_radius) {
    // Express the sphere position in the frame of the rounded box.
    real3 spherePos = TransformParentToLocal(pos1, rot1, pos2);

    // Snap the sphere position to the surface of the skeleton box.
    real3 boxPos = spherePos;
    uint code = snap_to_box(hdims1, boxPos);

    // Reduce the problem to the interaction between two spheres:
    //    (a) a sphere with radius srad1, centered at boxPos
    //    (b) a sphere with radius radius2, centered at spherePos
    // If the two sphere centers are farther away that the radii sum plus the
    // separation value, there is no contact. Also, ignore contact if the two
    // centers almost coincide, in which case we couldn't decide on the proper
    // contact direction.
    real radSum = srad1 + radius2;
    real radSum_s = radSum + separation;
    real3 delta = spherePos - boxPos;
    real dist2 = Dot(delta, delta);

    if (dist2 >= radSum_s * radSum_s || dist2 <= 1e-12f)
        return false;

    // Generate contact information.
    real dist = Sqrt(dist2);
    depth = dist - radSum;
    norm = Rotate(delta / dist, rot1);
    pt2 = pos2 - norm * radius2;
    pt1 = pt2 - depth * norm;

    if ((code != 1) && (code != 2) && (code != 4))
        eff_radius = radius2 * srad1 / (radius2 + srad1);
    else
        eff_radius = radius2;

    return true;
}

// =============================================================================
//              TRIANGLE - SPHERE

// Triangle-sphere narrow phase collision detection.
// In: triangular face defined by points A1, B1, C1
//     sphere sphere centered at pos2 and with radius2

bool triangle_sphere(const real3& A1,
                     const real3& B1,
                     const real3& C1,
                     const real3& pos2,
                     const real& radius2,
                     const real& separation,
                     real3& norm,
                     real& depth,
                     real3& pt1,
                     real3& pt2,
                     real& eff_radius) {
    real radius2_s = radius2 + separation;

    // Calculate face normal.
    real3 nrm1 = triangle_normal(A1, B1, C1);

    // Calculate signed height of sphere center above face plane. If the
    // height is larger than the sphere radius plus the separation value
    // or if the sphere center is below the plane, there is no contact.
    real h = Dot(pos2 - A1, nrm1);

    if (h >= radius2_s || h <= 0)
        return false;

    // Find the closest point on the face to the sphere center and determine
    // whether or not this location is inside the face or on an edge.
    real3 faceLoc;

    if (snap_to_triangle(A1, B1, C1, pos2, faceLoc)) {
        // Closest face feature is an edge. If the distance between the sphere
        // center and the closest point is more than the radius plus the
        // separation value, then there is no contact. Also, ignore contact if
        // the sphere center (almost) coincides with the closest point, in
        // which case we couldn't decide on the proper contact direction.
        real3 delta = pos2 - faceLoc;
        real dist2 = Dot(delta, delta);

        if (dist2 >= radius2_s * radius2_s || dist2 <= 1e-12f)
            return false;

        real dist = Sqrt(dist2);
        norm = delta / dist;
        depth = dist - radius2;
        eff_radius = radius2 * edge_radius / (radius2 + edge_radius);
    } else {
        // Closest point on face is inside the face.
        norm = nrm1;
        depth = h - radius2;
        eff_radius = radius2;
    }

    pt1 = faceLoc;
    pt2 = pos2 - norm * radius2;

    return true;
}

// =============================================================================
//              CAPSULE - CAPSULE

// Capsule-capsule narrow phase collision detection.
// In:  capsule at pos1, with orientation rot1
//              capsule has radius1 and half-length hlen1 (in Y direction)
//      capsule at pos2, with orientation rot2
//              capsule has radius2 and half-length hlen2 (in Y direction)
// Note: a capsule-capsule collision may return 0, 1, or 2 contacts

int capsule_capsule(const real3& pos1,
                    const quaternion& rot1,
                    const real& radius1,
                    const real& hlen1,
                    const real3& pos2,
                    const quaternion& rot2,
                    const real& radius2,
                    const real& hlen2,
                    const real& separation,
                    real3* norm,
                    real* depth,
                    real3* pt1,
                    real3* pt2,
                    real* eff_radius) {
    // Express the second capule in the frame of the first one.
    real3 pos = RotateT(pos2 - pos1, rot1);
    quaternion rot = Mult(Inv(rot1), rot2);

    // Unit vectors along capsule axes.
    real3 V1 = AMatV(rot1);  // capsule1 in the global frame
    real3 V2 = AMatV(rot2);  // capsule2 in the global frame
    real3 V = AMatV(rot);    // capsule2 in the frame of capsule1

    // Sum of radii
    real radSum = radius1 + radius2;
    real radSum_s = radSum + separation;
    real radSum_s2 = radSum_s * radSum_s;

    // If the two capsules intersect, there may be 1 or 2 contacts. Note that 2
    // contacts are possible only if the two capsules are parallel. Calculate
    // the pairs of potential contact points, expressed in the global frame.
    int numLocs = 0;
    real3 locs1[2];
    real3 locs2[2];
    real denom = 1 - V.y * V.y;

    if (denom < 1e-4f) {
        // The two capsules are parallel. If the distance between their axes is
        // more than the sum of radii plus the separation value, there is no contact.
        if (pos.x * pos.x + pos.z * pos.z >= radSum_s2)
            return 0;

        // Find overlap of the two axes (as signed distances along the axis of
        // the first capsule).
        real locs[2] = {Min(hlen1, pos.y + hlen2), Max(-hlen1, pos.y - hlen2)};

        if (locs[0] > locs[1]) {
            // The two axes overlap. Both ends of the overlapping segment represent
            // potential contacts.
            numLocs = 2;
            locs1[0] = pos1 + locs[0] * V1;
            locs2[0] = TransformLocalToParent(pos1, rot1, real3(pos.x, locs[0], pos.z));
            locs1[1] = pos1 + locs[1] * V1;
            locs2[1] = TransformLocalToParent(pos1, rot1, real3(pos.x, locs[1], pos.z));
        } else {
            // There is no overlap between axes. The two closest ends represent
            // a single potential contact.
            numLocs = 1;
            locs1[0] = pos1 + locs[pos.y < 0] * V1;
            locs2[0] = TransformLocalToParent(pos1, rot1, real3(pos.x, locs[pos.y > 0], pos.z));
        }
    } else {
        // The two capsule axes are not parallel. Find the closest points on the
        // two axes and clamp them to the extents of the their respective capsule.
        // This pair of points represents a single potential contact.
        real alpha2 = (V.y * pos.y - Dot(V, pos)) / denom;
        real alpha1 = V.y * alpha2 + pos.y;

        if (alpha1 < -hlen1) {
            alpha1 = -hlen1;
            alpha2 = -Dot(pos, V) - hlen1 * V.y;
        } else if (alpha1 > hlen1) {
            alpha1 = hlen1;
            alpha2 = -Dot(pos, V) + hlen1 * V.y;
        }

        if (alpha2 < -hlen2) {
            alpha2 = -hlen2;
            alpha1 = Clamp(pos.y - hlen2 * V.y, -hlen1, hlen1);
        } else if (alpha2 > hlen2) {
            alpha2 = hlen2;
            alpha1 = Clamp(pos.y + hlen2 * V.y, -hlen1, hlen1);
        }

        numLocs = 1;
        locs1[0] = pos1 + alpha1 * V1;
        locs2[0] = pos2 + alpha2 * V2;
    }

    // Check the pairs of locations for actual contact and generate contact
    // information. Keep track of the actual number of contacts.
    real effRad = radius1 * radius2 / radSum;
    int j = 0;

    for (int i = 0; i < numLocs; i++) {
        real3 delta = locs2[i] - locs1[i];
        real dist2 = Dot(delta, delta);

        // If the two sphere centers are separated by more than the sum of their
        // radii plus the separation value, there is no contact. Also ignore
        // contact if the two centers almost coincide, in which case we cannot
        // decide on the direction.
        if (dist2 >= radSum_s2 || dist2 < 1e-12)
            continue;

        // Generate contact information.
        real dist = Sqrt(dist2);
        *(norm + j) = delta / dist;
        *(pt1 + j) = locs1[i] + (*(norm + j)) * radius1;
        *(pt2 + j) = locs2[i] - (*(norm + j)) * radius2;
        *(depth + j) = dist - radSum;
        *(eff_radius + j) = effRad;

        j++;
    }

    // Return the number of actual contacts
    return j;
}

// =============================================================================
//              BOX - CAPSULE

// Box-capsule narrow phase collision detection.
// In:  box at position pos1, with orientation rot1, and half-dimensions hdims1
//      capsule at pos2, with orientation rot2
//              capsule has radius2 and half-length hlen2 (in Y direction)
// Note: a box-capsule collision may return 0, 1, or 2 contacts

int box_capsule(const real3& pos1,
                const quaternion& rot1,
                const real3& hdims1,
                const real3& pos2,
                const quaternion& rot2,
                const real& radius2,
                const real& hlen2,
                const real& separation,
                real3* norm,
                real* depth,
                real3* pt1,
                real3* pt2,
                real* eff_radius) {
    real radius2_s = radius2 + separation;

    // Express the capsule in the frame of the box.
    // (this is a bit cryptic with the functions we have available)
    real3 pos = RotateT(pos2 - pos1, rot1);
    quaternion rot = Mult(Inv(rot1), rot2);
    real3 V = AMatV(rot);

    // Inflate the box by the radius of the capsule plus the separation value
    // and check if the capsule centerline intersects the expanded box. We do
    // this by clamping the capsule axis to the volume between two parallel
    // faces of the box, considering in turn the x, y, and z faces.
    real3 hdims1_exp = hdims1 + radius2_s;
    real tMin = -C_LARGE_REAL;
    real tMax = C_LARGE_REAL;

    if (Abs(V.x) < 1e-5) {
        // Capsule axis parallel to the box x-faces
        if (Abs(pos.x) > hdims1_exp.x)
            return 0;
    } else {
        real t1 = (-hdims1_exp.x - pos.x) / V.x;
        real t2 = (hdims1_exp.x - pos.x) / V.x;

        tMin = Max(tMin, Min(t1, t2));
        tMax = Min(tMax, Max(t1, t2));

        if (tMin > tMax)
            return 0;
    }

    if (Abs(V.y) < 1e-5) {
        // Capsule axis parallel to the box y-faces
        if (Abs(pos.y) > hdims1_exp.y)
            return 0;
    } else {
        real t1 = (-hdims1_exp.y - pos.y) / V.y;
        real t2 = (hdims1_exp.y - pos.y) / V.y;

        tMin = Max(tMin, Min(t1, t2));
        tMax = Min(tMax, Max(t1, t2));

        if (tMin > tMax)
            return 0;
    }

    if (Abs(V.z) < 1e-5) {
        // Capsule axis parallel to the box z-faces
        if (Abs(pos.z) > hdims1_exp.z)
            return 0;
    } else {
        real t1 = (-hdims1_exp.z - pos.z) / V.z;
        real t2 = (hdims1_exp.z - pos.z) / V.z;

        tMin = Max(tMin, Min(t1, t2));
        tMax = Min(tMax, Max(t1, t2));

        if (tMin > tMax)
            return 0;
    }

    // Generate the two points where the capsule centerline intersects
    // the exapanded box (still expressed in the box frame). Snap these
    // locations onto the original box, then snap back onto the capsule
    // axis. This reduces the collision problem to 1 or 2 box-sphere
    // collisions.
    real3 locs[2] = {pos + tMin * V, pos + tMax * V};
    real t[2];

    for (int i = 0; i < 2; i++) {
        /*uint code =*/snap_to_box(hdims1, locs[i]);
        t[i] = Clamp(Dot(locs[i] - pos, V), -hlen2, hlen2);
    }

    // Check if the two sphere centers coincide (i.e. if we should
    // consider 1 or 2 box-sphere potential contacts)
    int numSpheres = IsEqual(t[0], t[1]) ? 1 : 2;

    // Perform box-sphere tests, and keep track of actual number of contacts.
    int j = 0;

    for (int i = 0; i < numSpheres; i++) {
        // Calculate the center of the corresponding sphere on the capsule
        // centerline (expressed in the box frame).
        real3 spherePos = pos + V * t[i];

        // Snap the sphere position to the surface of the box.
        real3 boxPos = spherePos;
        uint code = snap_to_box(hdims1, boxPos);

        // If the distance from the sphere center to the closest point is larger
        // than the radius plus the separation value, then there is no contact.
        // Also, ignore contact if the sphere center (almost) coincides with the
        // closest point, in which case we couldn't decide on the proper contact
        // direction.
        real3 delta = spherePos - boxPos;
        real dist2 = Dot(delta, delta);

        if (dist2 >= radius2_s * radius2_s || dist2 <= 1e-12)
            continue;

        // Generate contact information.
        real dist = Sqrt(dist2);

        *(depth + j) = dist - radius2;
        *(norm + j) = Rotate(delta / dist, rot1);
        *(pt1 + j) = TransformLocalToParent(pos1, rot1, boxPos);
        *(pt2 + j) = TransformLocalToParent(pos1, rot1, spherePos) - (*(norm + j)) * radius2;

        if ((code != 1) && (code != 2) && (code != 4))
            *(eff_radius + j) = radius2 * edge_radius / (radius2 + edge_radius);
        else
            *(eff_radius + j) = radius2;

        j++;
    }

    // Return the number of actual contacts
    return j;
}

// =============================================================================
//              BOX - CYLSHELL

// Box-cylshell narrow phase collision detection.
// In:  box at position pos1, with orientation rot1, and half-dimensions hdims
//      cylshell at pos2, with orientation rot2, radius and half-length hlen (in Y direction)
// Notes:
// - collisions with the caps of the cylshell are ignored!
// - a box-cylshell collision may return 0, 1, or more contacts

int box_cylshell(const real3& pos1,
                 const quaternion& rot1,
                 const real3& hdims,
                 const real3& pos2,
                 const quaternion& rot2,
                 const real& radius,
                 const real& hlen,
                 const real& separation,
                 real3* norm,
                 real* depth,
                 real3* pt1,
                 real3* pt2,
                 real* eff_radius) {
    real radius_s = radius + separation;

    // Express cylinder in the box frame
    real3 c = RotateT(pos2 - pos1, rot1);    // cylinder center (expressed in box frame)
    quaternion rot = Mult(Inv(rot1), rot2);  // cylinder orientation (w.r.t box frame)
    real3 a = AMatV(rot);                    // cylinder axis (expressed in box frame)

    // Inflate the box by the radius of the capsule plus the separation value and check if the capsule centerline
    // intersects the expanded box. We do this by clamping the capsule axis to the volume between two parallel faces
    // of the box, considering in turn the x, y, and z faces.
    real3 hdims_exp = hdims + radius_s;
    real tMin = -C_LARGE_REAL;
    real tMax = C_LARGE_REAL;

    real threshold = real(1e-5);  // threshold for line parallel to face tests

    if (Abs(a.x) < threshold) {
        // Capsule axis parallel to the box x-faces
        if (Abs(c.x) > hdims_exp.x)
            return 0;
    } else {
        real t1 = (-hdims_exp.x - c.x) / a.x;
        real t2 = (hdims_exp.x - c.x) / a.x;

        tMin = Max(tMin, Min(t1, t2));
        tMax = Min(tMax, Max(t1, t2));

        if (tMin > tMax)
            return 0;
    }

    if (Abs(a.y) < threshold) {
        // Capsule axis parallel to the box y-faces
        if (Abs(c.y) > hdims_exp.y)
            return 0;
    } else {
        real t1 = (-hdims_exp.y - c.y) / a.y;
        real t2 = (hdims_exp.y - c.y) / a.y;

        tMin = Max(tMin, Min(t1, t2));
        tMax = Min(tMax, Max(t1, t2));

        if (tMin > tMax)
            return 0;
    }

    if (Abs(a.z) < threshold) {
        // Capsule axis parallel to the box z-faces
        if (Abs(c.z) > hdims_exp.z)
            return 0;
    } else {
        real t1 = (-hdims_exp.z - c.z) / a.z;
        real t2 = (hdims_exp.z - c.z) / a.z;

        tMin = Max(tMin, Min(t1, t2));
        tMax = Min(tMax, Max(t1, t2));

        if (tMin > tMax)
            return 0;
    }

    // Generate the two points where the cylinder centerline intersects the exapanded box (still expressed in the
    // box frame). Snap these locations to the original box, then snap back onto the cylinder axis. This reduces
    // the collision problem to 1 or 2 collisions.
    real3 locs[2] = {c + tMin * a, c + tMax * a};
    real t[2];

    for (int i = 0; i < 2; i++) {
        /*uint code =*/snap_to_box(hdims, locs[i]);
        t[i] = Clamp(Dot(locs[i] - c, a), -hlen, hlen);
    }

    // Check if the two points almost coincide (in which case consider only one of them)
    int numPoints = IsEqual(t[0], t[1]) ? 1 : 2;

    // Perform collision tests and keep track of actual number of contacts.
    int j = 0;
    for (int i = 0; i < numPoints; i++) {
        // Point on the cylinder axis (expressed in the box frame).
        real3 axisPoint = c + a * t[i];

        // Snap to box. If axis point inside box, no contact.
        real3 boxPoint = axisPoint;
        uint code = snap_to_box(hdims, boxPoint);
        if (code == 0)
            continue;

        // Find closest point on cylinder to the box. If this point is on the cylinder centerline, no contact.
        real3 u = axisPoint - boxPoint;
        real u_length = Sqrt(Dot(u, u));
        if (u_length < threshold)
            continue;
        u = u / u_length;
        real3 w = Cross(u, a);
        real w_length = Sqrt(Dot(w, w));
        if (w_length < threshold)
            continue;
        real3 v = Cross(w, a);
        real v_length = Sqrt(Dot(v, v));
        v = v / v_length;
        real3 cylPoint = axisPoint + radius * v;

        // If cylinder point outside box, no contact.
        code = snap_to_box(hdims, cylPoint);
        if (code != 0)
            continue;

        // Moving in the u direction, project cylinder point onto box surface.
        real step = C_LARGE_REAL;
        if (Abs(u.x) > threshold)
            step = Min((Sign(u.x) * hdims.x - cylPoint.x) / u.x, step);
        if (Abs(u.y) > threshold)
            step = Min((Sign(u.y) * hdims.y - cylPoint.y) / u.y, step);
        if (Abs(u.z) > threshold)
            step = Min((Sign(u.z) * hdims.z - cylPoint.z) / u.z, step);
        boxPoint = cylPoint + step * u;

        // Debug check: boxPoint inside cylinder
        assert(Abs(Dot(a, boxPoint - c)) <= real(1.01) * hlen);

        // Calculate penetration
        real3 delta = boxPoint - cylPoint;
        real dist = Sqrt(Dot(delta, delta));
        if (dist < 1e-10)
            continue;

        *(depth + j) = -dist;
        *(norm + j) = Rotate(delta / dist, rot1);
        *(pt1 + j) = TransformLocalToParent(pos1, rot1, boxPoint);
        *(pt2 + j) = TransformLocalToParent(pos1, rot1, cylPoint);
        *(eff_radius + j) = 0.1;
        /// radius;

        j++;
    }

    // Return the number of actual contacts
    return j;
}

// =============================================================================
//              BOX - BOX

// Box-box narrow phase collision detection.
// In:  "this" box at position posT, with orientation rotT, and half-dimensions hdimsT
//      "other" box at position posO, with orientation rotO, and half-dimensions hdimsO

int box_box(const real3& posT,
            const quaternion& rotT,
            const real3& hdimsT,
            const real3& posO,
            const quaternion& rotO,
            const real3& hdimsO,
            const real& separation,
            real3* norm,
            real* depth,
            real3* ptT,
            real3* ptO,
            real* eff_radius) {
    // Express the other box into the frame of this box.
    // (this is a bit cryptic with the functions we have available)
    real3 pos = RotateT(posO - posT, rotT);
    quaternion rot = Mult(Inv(rotT), rotO);

    // Find the direction of smallest overlap between boxes. Note that dirT is calculated so that it points
    // from boxO to boxT and is expressed in the frame of boxT. If the two boxes don't overlap, we're done.
    // Otherwise, penetrated = -1 if the boxes overlap or +1 if they are separated.
    real3 dirT;
    int penetrated = box_intersects_box(hdimsT, hdimsO, pos, rot, separation, dirT);
    if (penetrated == 0)
        return 0;
    if (Dot(pos, dirT) > 0)
        dirT = -dirT;

    // If separation = 0, then penetrated must be -1.
    assert(separation > 0 || penetrated == -1);

    // Determine the features of the boxes that are interacting.
    // A feature is defined by a box corner and a code (7 for corner; 3,5,6 for an edge; 1,2,4 for a face).
    real3 dirO = RotateT(-dirT, rot);
    real3 cornerT = box_farthest_corner(hdimsT, dirT);  // corner on this box (in this box frame)
    real3 cornerO = box_farthest_corner(hdimsO, dirO);  // corner on other box (in other box frame)
    uint codeT = box_closest_feature(dirT, hdimsT);
    uint codeO = box_closest_feature(dirO, hdimsO);
    assert(codeT > 0);
    assert(codeO > 0);

    // Generate contacts (9 possible configuration, depending on interacting box features)
    bool Tcorner = (codeT == 7);
    bool Tedge = (codeT == 3) || (codeT == 5) || (codeT == 6);
    bool Tface = (codeT == 1) || (codeT == 2) || (codeT == 4);
    bool Ocorner = (codeO == 7);
    bool Oedge = (codeO == 3) || (codeO == 5) || (codeO == 6);
    bool Oface = (codeO == 1) || (codeO == 2) || (codeO == 4);

    // (1) A box corner is involved [5 configurations] => 1 contact
    if (Tcorner || Ocorner) {
        if (Tcorner) {
            if (Oedge) {
                // cornerT to edgeO
                cornerO = snap_to_box_edge(hdimsO, cornerO, codeO, RotateT(cornerT - pos, rot));
            } else if (Oface) {
                // cornerT to faceO
                cornerO = snap_to_box_face(hdimsO, cornerO, codeO, RotateT(cornerT - pos, rot));
            }
        } else {
            if (Tedge) {
                // cornerO to edgeT
                cornerT = snap_to_box_edge(hdimsT, cornerT, codeT, Rotate(cornerO, rot) + pos);
            } else if (Tface) {
                // cornerO to faceT
                cornerT = snap_to_box_face(hdimsT, cornerT, codeT, Rotate(cornerO, rot) + pos);
            }
        }

        *(ptT) = Rotate(cornerT, rotT) + posT;
        *(ptO) = Rotate(cornerO, rotO) + posO;
        real3 delta = penetrated * (*(ptO) - *(ptT));
        real dist = Sqrt(Dot(delta, delta));
        *(norm) = delta / dist;
        *(depth) = penetrated * dist;
        *(eff_radius) = edge_radius / 2;

        return 1;
    }

    // (2) Edge against edge [1 configuration] => 0 or 1 contact
    if (Tedge && Oedge) {
        // Get the corners of the other edge and express in the frame of this box
        real3 cornersO[2];
        get_edge_corners(cornerO, codeO, cornersO);
        real3 corner0 = Rotate(cornersO[0], rot) + pos;
        real3 corner1 = Rotate(cornersO[1], rot) + pos;

        // Check for contact between edgeT and edgeO
        real3 locT;
        real3 locO;
        if (segment_vs_edge(hdimsT, cornerT, codeT, corner0, corner1, locT, locO)) {
            locT = Rotate(locT, rotT) + posT;
            locO = Rotate(locO, rotT) + posT;

            real3 delta = penetrated * (locO - locT);
            real dist = Sqrt(Dot(delta, delta));

            if (penetrated == 1 && dist > separation)
                return 0;

            *(ptT) = locT;
            *(ptO) = locO;
            *(norm) = delta / dist;
            *(depth) = penetrated * dist;
            *(eff_radius) = edge_radius / 2;
            return 1;
        }

        return 0;
    }

    // (3) Face against face [1 configuration] => up to 8 contacts
    if (Tface && Oface) {
        // Get the corners of the two faces, expressed in their respective box frames
        real3 cornersT[4];
        real3 cornersO[4];
        get_face_corners(cornerT, codeT, cornersT);
        get_face_corners(cornerO, codeO, cornersO);

        // Keep track of all added contacts
        int j = 0;

        // Check corners of one box against the other face
        real3 loc;
        real3 nrm;
        real dist;
        for (uint i = 0; i < 4; i++) {
            // Check corners of faceO against faceT
            if (point_vs_face(hdimsT, cornerT, codeT, Rotate(cornersO[i], rot) + pos, separation, loc, nrm, dist)) {
                *(ptT + j) = Rotate(loc, rotT) + posT;
                *(ptO + j) = Rotate(cornersO[i], rotO) + posO;
                *(norm + j) = Rotate(nrm, rotT);
                *(depth + j) = dist;
                *(eff_radius + j) = edge_radius;
                j++;
            }

            // Check corners of faceT against faceO
            if (point_vs_face(hdimsO, cornerO, codeO, RotateT(cornersT[i] - pos, rot), separation, loc, nrm, dist)) {
                *(ptT + j) = Rotate(cornersT[i], rotT) + posT;
                *(ptO + j) = Rotate(loc, rotO) + posO;
                *(norm + j) = Rotate(-nrm, rotO);
                *(depth + j) = dist;
                *(eff_radius + j) = edge_radius;
                j++;
            }
        }

        // Check the edges of faceT against the edges of faceO.
        uint codeTN = (codeT | (codeT >> 1) | (codeT << 2)) & 7;
        uint codeTP = (codeT | (codeT << 1) | (codeT >> 2)) & 7;
        real3 locT;
        real3 locO;
        for (uint i = 0; i < 4; i++) {
            uint codeE = i & 1 ? codeTP : codeTN;

            for (uint j1 = 0, j2 = 3; j1 < 4; j2 = j1++) {
                real3 cornerO1 = Rotate(cornersO[j1], rot) + pos;
                real3 cornerO2 = Rotate(cornersO[j2], rot) + pos;

                if (segment_vs_edge(hdimsT, cornersT[i], codeE, cornerO1, cornerO2, locT, locO)) {
                    locT = Rotate(locT, rotT) + posT;
                    locO = Rotate(locO, rotT) + posT;

                    real3 delta = penetrated * (locO - locT);
                    dist = Sqrt(Dot(delta, delta));

                    if (penetrated == 1 && dist > separation)
                        continue;

                    *(ptT + j) = locT;
                    *(ptO + j) = locO;
                    *(norm + j) = delta / dist;
                    *(depth + j) = penetrated * dist;
                    *(eff_radius + j) = edge_radius;
                    j++;
                }
            }
        }

        return j;
    }

    // (4) Face of this box against edge on other box [1 configuration] => up to 2 contacts
    if (Tface) {
        assert(Oedge);

        real3 cornersT[4];
        real3 cornersO[4];
        get_face_corners(cornerT, codeT, cornersT);
        get_edge_corners(cornerO, codeO, cornersO);
        dirT = Rotate(dirT, rotT);

        // Keep track of added contacts
        int j = 0;

        // Check corners of edgeO against faceT
        real3 loc;
        real3 nrm;
        real dist;
        for (uint i = 0; i < 2; i++) {
            if (point_vs_face(hdimsT, cornerT, codeT, Rotate(cornersO[i], rot) + pos, separation, loc, nrm, dist)) {
                *(ptT + j) = Rotate(loc, rotT) + posT;
                *(ptO + j) = Rotate(cornersO[i], rotO) + posO;
                *(norm + j) = Rotate(nrm, rotT);
                *(depth + j) = dist;
                *(eff_radius + j) = edge_radius;
                j++;
            }
        }

        // Check edgeO against the edges of faceT
        real3 locT;
        real3 locO;
        for (uint i1 = 0, i2 = 3; i1 < 4; i2 = i1++) {
            real3 cornerT1 = RotateT(cornersT[i1] - pos, rot);
            real3 cornerT2 = RotateT(cornersT[i2] - pos, rot);

            if (segment_vs_edge(hdimsO, cornerO, codeO, cornerT1, cornerT2, locO, locT)) {
                locT = Rotate(locT, rotO) + posO;
                locO = Rotate(locO, rotO) + posO;

                real3 delta = penetrated * (locO - locT);
                dist = Sqrt(Dot(delta, delta));

                if (penetrated == 1 && dist > separation)
                    continue;

                *(ptT + j) = locT;
                *(ptO + j) = locO;
                *(norm + j) = delta / dist;
                *(depth + j) = penetrated * dist;
                *(eff_radius + j) = edge_radius;
                j++;
            }
        }

        return j;
    }

    // (5) Face of other box against edge on this box [1 configuration] => up to 2 contacts
    if (Oface) {
        assert(Tedge);

        real3 cornersT[4];
        real3 cornersO[4];
        get_edge_corners(cornerT, codeT, cornersT);
        get_face_corners(cornerO, codeO, cornersO);

        // Keep track of added contacts
        int j = 0;

        // Check corners of edgeT against faceO
        real3 loc;
        real3 nrm;
        real dist;
        for (uint i = 0; i < 2; i++) {
            if (point_vs_face(hdimsO, cornerO, codeO, RotateT(cornersT[i] - pos, rot), separation, loc, nrm, dist)) {
                *(ptT + j) = Rotate(cornersT[i], rotT) + posT;
                *(ptO + j) = Rotate(loc, rotO) + posO;
                *(norm + j) = Rotate(-nrm, rotO);
                *(depth + j) = dist;
                *(eff_radius + j) = edge_radius;
                j++;
            }
        }

        // Check edgeT against the edges of faceO
        real3 locT;
        real3 locO;
        for (uint i1 = 0, i2 = 3; i1 < 4; i2 = i1++) {
            real3 cornerO1 = Rotate(cornersO[i1], rot) + pos;
            real3 cornerO2 = Rotate(cornersO[i2], rot) + pos;

            if (segment_vs_edge(hdimsT, cornerT, codeT, cornerO1, cornerO2, locT, locO)) {
                locT = Rotate(locT, rotT) + posT;
                locO = Rotate(locO, rotT) + posT;

                real3 delta = penetrated * (locO - locT);
                dist = Sqrt(Dot(delta, delta));

                if (penetrated == 1 && dist > separation)
                    continue;

                *(ptT + j) = locT;
                *(ptO + j) = locO;
                *(norm + j) = delta / dist;
                *(depth + j) = penetrated * dist;
                *(eff_radius + j) = edge_radius;
                j++;
            }
        }

        return j;
    }

    // All configurations treated.
    assert(false);

    return 0;
}

}  // end namespace collision
}  // namespace chrono
