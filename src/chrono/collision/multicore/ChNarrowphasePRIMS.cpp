// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
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
// Implementation file for ChCNarrowphasePRIMS.
//
// =============================================================================

#include "chrono/collision/multicore/ChNarrowphase.h"
#include "chrono/collision/multicore/ChCollisionUtils.h"

namespace chrono {

using namespace chrono::ch_utils;

// Fictitious radius of curvature for collision with a corner or an edge.
static real edge_radius = 0.1;

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
//              capsule has radius1 and half-length hlen1 (in Z direction)
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
    real3 W = AMatW(rot1);
    real alpha = Dot(pos2 - pos1, W);
    alpha = Clamp(alpha, -hlen1, hlen1);

    real3 loc = pos1 + alpha * W;

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
//              cylinder has radius1 and half-length hlen1 (in Z direction)
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
//              roundedcyl has radius1 and half-length hlen1 (in Z direction)
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
//              capsule has radius1 and half-length hlen1 (in Z direction)
//      capsule at pos2, with orientation rot2
//              capsule has radius2 and half-length hlen2 (in Z direction)
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
    real3 W1 = AMatW(rot1);  // capsule1 in the global frame
    real3 W2 = AMatW(rot2);  // capsule2 in the global frame
    real3 W = AMatW(rot);    // capsule2 in the frame of capsule1

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
    real denom = 1 - W.z * W.z;

    if (denom < 1e-4f) {
        // The two capsules are parallel. If the distance between their axes is
        // more than the sum of radii plus the separation value, there is no contact.
        if (pos.x * pos.x + pos.y * pos.y >= radSum_s2)
            return 0;

        // Find overlap of the two axes (as signed distances along the axis of
        // the first capsule).
        real locs[2] = {Min(hlen1, pos.z + hlen2), Max(-hlen1, pos.z - hlen2)};

        if (locs[0] > locs[1]) {
            // The two axes overlap. Both ends of the overlapping segment represent
            // potential contacts.
            numLocs = 2;
            locs1[0] = pos1 + locs[0] * W1;
            locs2[0] = TransformLocalToParent(pos1, rot1, real3(pos.x, pos.y, locs[0]));
            locs1[1] = pos1 + locs[1] * W1;
            locs2[1] = TransformLocalToParent(pos1, rot1, real3(pos.x, pos.y, locs[1]));
        } else {
            // There is no overlap between axes. The two closest ends represent
            // a single potential contact.
            numLocs = 1;
            locs1[0] = pos1 + locs[pos.z < 0] * W1;
            locs2[0] = TransformLocalToParent(pos1, rot1, real3(pos.x, pos.y, locs[pos.y > 0]));
        }
    } else {
        // The two capsule axes are not parallel. Find the closest points on the
        // two axes and clamp them to the extents of the their respective capsule.
        // This pair of points represents a single potential contact.
        real alpha2 = (W.z * pos.z - Dot(W, pos)) / denom;
        real alpha1 = W.z * alpha2 + pos.z;

        if (alpha1 < -hlen1) {
            alpha1 = -hlen1;
            alpha2 = -Dot(pos, W) - hlen1 * W.z;
        } else if (alpha1 > hlen1) {
            alpha1 = hlen1;
            alpha2 = -Dot(pos, W) + hlen1 * W.z;
        }

        if (alpha2 < -hlen2) {
            alpha2 = -hlen2;
            alpha1 = Clamp(pos.z - hlen2 * W.z, -hlen1, hlen1);
        } else if (alpha2 > hlen2) {
            alpha2 = hlen2;
            alpha1 = Clamp(pos.z + hlen2 * W.z, -hlen1, hlen1);
        }

        numLocs = 1;
        locs1[0] = pos1 + alpha1 * W1;
        locs2[0] = pos2 + alpha2 * W2;
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
//              capsule has radius2 and half-length hlen2 (in Z direction)
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
    real3 W = AMatW(rot);

    // Inflate the box by the radius of the capsule plus the separation value
    // and check if the capsule centerline intersects the expanded box. We do
    // this by clamping the capsule axis to the volume between two parallel
    // faces of the box, considering in turn the x, y, and z faces.
    real3 hdims1_exp = hdims1 + radius2_s;
    real tMin = -C_REAL_MAX;
    real tMax = C_REAL_MAX;

    if (Abs(W.x) < 1e-5) {
        // Capsule axis parallel to the box x-faces
        if (Abs(pos.x) > hdims1_exp.x)
            return 0;
    } else {
        real t1 = (-hdims1_exp.x - pos.x) / W.x;
        real t2 = (hdims1_exp.x - pos.x) / W.x;

        tMin = Max(tMin, Min(t1, t2));
        tMax = Min(tMax, Max(t1, t2));

        if (tMin > tMax)
            return 0;
    }

    if (Abs(W.y) < 1e-5) {
        // Capsule axis parallel to the box y-faces
        if (Abs(pos.y) > hdims1_exp.y)
            return 0;
    } else {
        real t1 = (-hdims1_exp.y - pos.y) / W.y;
        real t2 = (hdims1_exp.y - pos.y) / W.y;

        tMin = Max(tMin, Min(t1, t2));
        tMax = Min(tMax, Max(t1, t2));

        if (tMin > tMax)
            return 0;
    }

    if (Abs(W.z) < 1e-5) {
        // Capsule axis parallel to the box z-faces
        if (Abs(pos.z) > hdims1_exp.z)
            return 0;
    } else {
        real t1 = (-hdims1_exp.z - pos.z) / W.z;
        real t2 = (hdims1_exp.z - pos.z) / W.z;

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
    real3 locs[2] = {pos + tMin * W, pos + tMax * W};
    real t[2];

    for (int i = 0; i < 2; i++) {
        /*uint code =*/snap_to_box(hdims1, locs[i]);
        t[i] = Clamp(Dot(locs[i] - pos, W), -hlen2, hlen2);
    }

    // Check if the two sphere centers coincide (i.e. if we should
    // consider 1 or 2 box-sphere potential contacts)
    int numSpheres = IsEqual(t[0], t[1]) ? 1 : 2;

    // Perform box-sphere tests, and keep track of actual number of contacts.
    int j = 0;

    for (int i = 0; i < numSpheres; i++) {
        // Calculate the center of the corresponding sphere on the capsule
        // centerline (expressed in the box frame).
        real3 spherePos = pos + W * t[i];

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
//      cylshell at pos2, with orientation rot2, radius and half-length hlen (in Z direction)
// Notes:
// - only treat interactions when the cylinder axis is parallel to or perpendicular on a box face.
// - for any other relative configuration report -1 contacts (which will trigger a fall-back onto MPR).
// - a box-cylshell collision may return up to 8 contacts.
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
    // Express cylinder in the box frame
    real3 c = RotateT(pos2 - pos1, rot1);    // cylinder center (expressed in box frame)
    quaternion rot = Mult(Inv(rot1), rot2);  // cylinder orientation (w.r.t box frame)
    real3 a = AMatW(rot);                    // cylinder axis (expressed in box frame)

    real3 c_abs = Abs(c);
    real3 a_abs = Abs(a);

    static const real threshold_par = real(1e-4);   // threshold for axis parallel to face test
    static const real threshold_perp = real(1e-4);  // threshold for axis perpendicular to face test

    // Loop over the 3 box directions. Treat only the cases where the cylinder axis is almost parallel or almost
    // perpendicular to a box face.
    for (uint i1 = 0, i2 = 1, i3 = 2; i1 < 3; i2 = i3, i3 = i1++) {
        // (1) Check if cylinder axis is parallel to the 'i1' box face
        if (a_abs[i1] < threshold_par) {
            // if cylinder too far from face, no collision
            if (c_abs[i1] > hdims[i1] + radius + separation)
                return 0;

            // if cylinder too far into box, do nothing
            if (c_abs[i1] < hdims[i1])
                continue;

            // clamp cylinder centerline to [i2,i3] box slabs
            real tMin = -C_REAL_MAX;
            real tMax = C_REAL_MAX;
            if (a_abs[i2] > threshold_par) {
                real t1 = (-hdims[i2] - c[i2]) / a[i2];
                real t2 = (+hdims[i2] - c[i2]) / a[i2];
                tMin = Max(tMin, Min(t1, t2));
                tMax = Min(tMax, Max(t1, t2));
                if (tMin > tMax)
                    return 0;
            }
            if (a_abs[i3] > threshold_par) {
                real t1 = (-hdims[i3] - c[i3]) / a[i3];
                real t2 = (+hdims[i3] - c[i3]) / a[i3];
                tMin = Max(tMin, Min(t1, t2));
                tMax = Min(tMax, Max(t1, t2));
                if (tMin > tMax)
                    return 0;
            }

            // clamp tMin and tMax to cylinder axis
            ClampValue(tMin, -hlen, +hlen);
            ClampValue(tMax, -hlen, +hlen);

            // generate two collisions (points on cylinder axis)
            real3 locs[2] = {c + tMin * a, c + tMax * a};

            for (int i = 0; i < 2; i++) {
                // snap point to box
                real3 boxPoint = locs[i];
                uint code = snap_to_box(hdims, boxPoint);  // point on box (in box frame)
                assert(code != 0);                         // point cannot be inside box
                real3 u = locs[i] - boxPoint;              // collision direction (in box frame)
                real u_nrm = Sqrt(Dot(u, u));              // distance between point on box and cylinder axis
                assert(u_nrm > 0);                         // cylinder axis must be outside box
                u = u / u_nrm;                             // collision normal (in box frame)
                real3 cylPoint = locs[i] - radius * u;     // point on cylinder (in box frame)

                *(depth + i) = u_nrm - radius;                              // depth (negative for penetration)
                *(norm + i) = Rotate(u, rot1);                              // collision normal (in global frame)
                *(pt1 + i) = TransformLocalToParent(pos1, rot1, boxPoint);  // point on box (in global frame)
                *(pt2 + i) = TransformLocalToParent(pos1, rot1, cylPoint);  // point on cylinder (in global frame)
                *(eff_radius + i) = radius;                                 // effective radius

                ////std::cout << u_nrm - radius << std::endl;
            }

            return 2;
        }

        // (2) Check if cylinder axis is perpendicular to the 'i1' box face
        if (a_abs[i1] > 1 - threshold_perp) {
            // if cylinder too far from box, no collision
            if (c_abs[i1] > hdims[i1] + hlen + separation)
                return 0;

            // if cylinder too far into box, do nothing
            if (c_abs[i1] < hdims[i1])
                continue;

            // if cylinder too far to the "side", do nothing
            if (c_abs[i2] > hdims[i2] || c_abs[i3] > hdims[i3]) {
                continue;
            }

            // decide on "sign" of box face and set the normal direction for any resulting collisions
            int sign = (c[i1] > 0) ? +1 : -1;
            real3 u(0);
            u[i1] = sign;

            // working in the plane fo the 'i1' face, the circle center is at (c[i2], c[i3]).
            // check circle intersection with each face edge (and clamp to edge length if needed).
            // if circle does not intersect edge, create collision point on circle.
            real discr;
            real3 locs[8];  // collision points (expressed in box frame)
            int nc = 0;     // keep track of number of circle-rectangle intersection points (at most 8)

            // negative 'i2' edge
            discr = radius * radius - (c[i2] + hdims[i2]) * (c[i2] + hdims[i2]);
            if (discr > 0) {
                real sqrt_discr = Sqrt(discr);
                locs[nc][i2] = -hdims[i2];
                locs[nc][i3] = Clamp(c[i2] + sqrt_discr, -hdims[i3], +hdims[i3]);
                nc++;
                locs[nc][i2] = -hdims[i2];
                locs[nc][i3] = Clamp(c[i2] - sqrt_discr, -hdims[i3], +hdims[i3]);
                nc++;
            } else {
                locs[nc][i2] = c[i2] - radius;
                locs[nc][i3] = c[i3];
                nc++;
            }

            // positive 'i2' edge
            discr = radius * radius - (c[i2] - hdims[i2]) * (c[i2] - hdims[i2]);
            if (discr > 0) {
                real sqrt_discr = Sqrt(discr);
                locs[nc][i2] = +hdims[i2];
                locs[nc][i3] = Clamp(c[i2] + sqrt_discr, -hdims[i3], +hdims[i3]);
                nc++;
                locs[nc][i2] = +hdims[i2];
                locs[nc][i3] = Clamp(c[i2] - sqrt_discr, -hdims[i3], +hdims[i3]);
                nc++;
            } else {
                locs[nc][i2] = c[i2] + radius;
                locs[nc][i3] = c[i3];
                nc++;
            }

            // negative 'i3' edge
            discr = radius * radius - (c[i3] + hdims[i3]) * (c[i3] + hdims[i3]);
            if (discr > 0) {
                real sqrt_discr = Sqrt(discr);
                locs[nc][i3] = -hdims[i3];
                locs[nc][i2] = Clamp(c[i3] + sqrt_discr, -hdims[i2], +hdims[i2]);
                nc++;
                locs[nc][i3] = -hdims[i3];
                locs[nc][i2] = Clamp(c[i3] - sqrt_discr, -hdims[i2], +hdims[i2]);
                nc++;
            } else {
                locs[nc][i3] = c[i3] - radius;
                locs[nc][i2] = c[i2];
                nc++;
            }

            // positive 'i3' edge
            discr = radius * radius - (c[i3] - hdims[i3]) * (c[i3] - hdims[i3]);
            if (discr > 0) {
                real sqrt_discr = Sqrt(discr);
                locs[nc][i3] = +hdims[i3];
                locs[nc][i2] = Clamp(c[i3] + sqrt_discr, -hdims[i2], +hdims[i2]);
                nc++;
                locs[nc][i3] = +hdims[i3];
                locs[nc][i2] = Clamp(c[i3] - sqrt_discr, -hdims[i2], +hdims[i2]);
                nc++;
            } else {
                locs[nc][i3] = c[i3] + radius;
                locs[nc][i2] = c[i2];
                nc++;
            }

            // Generate collision geometric information for all interactions
            for (int i = 0; i < nc; i++) {
                locs[i][i1] = sign * hdims[i1];                            // point on box (in box frame)
                *(pt1 + i) = TransformLocalToParent(pos1, rot1, locs[i]);  // point on box (in global frame)
                locs[i][i1] = c[i1] - sign * hlen;                         // point on cylinder (in box frame)
                *(pt2 + i) = TransformLocalToParent(pos1, rot1, locs[i]);  // point on cylinder (in global frame)
                *(depth + i) = c[i1] - sign * hlen - sign * hdims[i1];     // depth (negative for penetration)
                *(norm + i) = Rotate(u, rot1);                             // collision normal (in global frame)
                *(eff_radius + i) = radius;                                // questionable as this is face-face contact
            }

            ////std::cout << "Axis perpendicular to face " << i1 << "    nc = " << nc << std::endl;
            return nc;
        }
    }

    // We were unable to compute collision analytically - signal fall-back to MPR
    return -1;
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
    // from boxO to boxT and is expressed in the frame of boxT. If the two boxes are too far apart, we're done.
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

// =============================================================================
//              TRIANGLE - BOX

// Triangle-box narrow phase collision detection.
// In: box at position pos1, with orientation rot1, and half-dimensions hdims1
//     triangular face defined by points A2, B2, C2
// Note: a triangle-box collision may return up to 6 contacts

int triangle_box(const real3& pos1,
                 const quaternion& rot1,
                 const real3& hdims1,
                 const real3* v2,
                 const real& separation,
                 real3* norm,
                 real* depth,
                 real3* pt1,
                 real3* pt2,
                 real* eff_radius) {
    // Express the triangle vertices in the box frame.
    real3 v[] = {RotateT(v2[0] - pos1, rot1), RotateT(v2[1] - pos1, rot1), RotateT(v2[2] - pos1, rot1)};

    // Test intersection. If the two shapes are too far apart, we're done. Otherwise, penetrated = -1 if the boxes
    // overlap or +1 if they are separated.
    int penetrated = box_intersects_triangle(hdims1, v[0], v[1], v[2], separation);
    if (penetrated == 0)
        return 0;

    // If separation = 0, then penetrated must be -1.
    assert(separation > 0 || penetrated == -1);

    // Calculate face normal.
    real3 nrm2 = triangle_normal(v2[0], v2[1], v2[2]);  // expressed in global frame
    real3 nrm2b = RotateT(nrm2, rot1);                  // expressed in box frame

    // Find the box face closest to being parallel to the triangle.
    real3 nrmAbs = Abs(nrm2b);
    uint codeF = nrmAbs[0] > nrmAbs[1] ? (nrmAbs[0] > nrmAbs[2] ? 1 : 4) : (nrmAbs[1] > nrmAbs[2] ? 2 : 4);
    real3 corner = box_farthest_corner(hdims1, nrm2b);

    // Keep track of all added contacts
    int nc = 0;

    // Working in the global frame, check the face vertices against the triangle.
    real3 face_corners[4];
    get_face_corners(corner, codeF, face_corners);

    for (uint i = 0; i < 4; i++) {
        // Express face corner in global frame
        real3 box_point = pos1 + Rotate(face_corners[i], rot1);
        if (point_in_triangle(v2[0], v2[1], v2[2], box_point)) {
            real h = Dot(box_point - v2[0], nrm2);
            if (h < separation) {
                *(pt1 + nc) = box_point;
                *(pt2 + nc) = box_point - h * nrm2;
                *(norm + nc) = nrm2;
                *(depth + nc) = h;
                *(eff_radius + nc) = edge_radius;
                nc++;
            }
        }
    }

    // Working in the box frame, check the triangle edges against the box face.
    static const real threshold_par = real(1e-4);    // threshold for axis parallel to face test
    static const real threshold_close = real(1e-4);  // threshold for distance to vertex
    bool v_added[] = {false, false, false};          // keep track of triangle vertices (do not include twice)

    int i1 = codeF >> 1;    // index of face normal direction
    int i2 = (i1 + 1) % 3;  // next direction (0->1->2->0)
    int i3 = (i2 + 1) % 3;  // previous direction (0->2->1->0)

    real3 hdims1s = hdims1 + separation;  // size of expanded box

    for (uint j = 0; j < 3; j++) {
        const auto& A = v[j];            // first point on triangle edge
        const auto& B = v[(j + 1) % 3];  // second point on triangle edge
        real3 AB = B - A;
        real AB_len2 = Length2(AB);

        // Clamp triangle edge to extended box slabs i2 and i3
        real tRange[2] = {-C_REAL_MAX, +C_REAL_MAX};
        if (abs(AB[i2]) > threshold_par) {
            real tA = (-hdims1s[i2] - A[i2]) / AB[i2];
            real tB = (+hdims1s[i2] - A[i2]) / AB[i2];
            tRange[0] = Max(tRange[0], Min(tA, tB));
            tRange[1] = Min(tRange[1], Max(tA, tB));
        }
        if (abs(AB[i3]) > threshold_par) {
            real tA = (-hdims1s[i3] - A[i3]) / AB[i3];
            real tB = (+hdims1s[i3] - A[i3]) / AB[i3];
            tRange[0] = Max(tRange[0], Min(tA, tB));
            tRange[1] = Min(tRange[1], Max(tA, tB));
        }
        if (tRange[0] > tRange[1])
            continue;

        // Clamp tRange to triangle edge
        ClampValue(tRange[0], 0, 1);
        ClampValue(tRange[1], 0, 1);

        // Check the two points on the triangle edge
        for (uint i = 0; i < 2; i++) {
            // Point on triangle edge
            real3 tri_point = A + tRange[i] * AB;
            // Snap to original box slabs
            tri_point[i1] = corner[i1];
            ClampValue(tri_point[i2], -hdims1[i2], +hdims1[i2]);
            ClampValue(tri_point[i3], -hdims1[i3], +hdims1[i3]);
            // Snap back to triangle edge
            real t = Clamp(Dot(tri_point - A, AB) / AB_len2, 0, 1);
            tri_point = A + t * AB;

            if (abs(tri_point[i1]) > hdims1s[i1] || abs(tri_point[i2]) > hdims1s[i2] ||
                abs(tri_point[i3]) > hdims1s[i3])
                continue;

            // Point on box
            real3 box_point = tri_point;
            box_point[i1] = corner[i1];
            ClampValue(box_point[i2], -hdims1[i2], +hdims1[i2]);
            ClampValue(box_point[i3], -hdims1[i3], +hdims1[i3]);

            // Check distance smaller than separation
            real3 delta = box_point - tri_point;
            real dist = penetrated * Length(delta);
            if (dist > separation)
                continue;

            // Do not add triangle vertices twice
            if (t < threshold_close) {
                if (v_added[j])
                    continue;
                else
                    v_added[j] = true;
            }
            if (t > 1 - threshold_close) {
                if (v_added[(j + 1) % 3])
                    continue;
                else
                    v_added[(j + 1) % 3] = true;
            }

            // Add collision pair
            *(pt1 + nc) = Rotate(box_point, rot1) + pos1;
            *(pt2 + nc) = Rotate(tri_point, rot1) + pos1;
            *(norm + nc) = Rotate(delta / dist, rot1);
            *(depth + nc) = dist;
            *(eff_radius + nc) = edge_radius;
            nc++;
        }
    }

    return nc;
}

// =============================================================================

void ChNarrowphase::SetDefaultEdgeRadius(real radius) {
    edge_radius = radius;
}

real ChNarrowphase::GetDefaultEdgeRadius() {
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
bool ChNarrowphase::PRIMSCollision(const ConvexBase* shapeA,  // first candidate shape
                                   const ConvexBase* shapeB,  // second candidate shape
                                   real separation,           // maximum separation
                                   real3* ct_norm,            // [output] contact normal (per contact pair)
                                   real3* ct_pt1,             // [output] point on shape1 (per contact pair)
                                   real3* ct_pt2,             // [output] point on shape2 (per contact pair)
                                   real* ct_depth,            // [output] penetration depth (per contact pair)
                                   real* ct_eff_rad,          // [output] effective contact radius (per contact pair)
                                   int& nC)                   // [output] number of contacts found
{
    // Special-case the collision detection based on the types of the two potentially colliding shapes.

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
        return (nC >= 0);
    }

    if (shapeA->Type() == ChCollisionShape::Type::CYLSHELL && shapeB->Type() == ChCollisionShape::Type::BOX) {
        nC = box_cylshell(shapeB->A(), shapeB->R(), shapeB->Box(), shapeA->A(), shapeA->R(), shapeA->Cylshell().x,
                          shapeA->Cylshell().y, separation, ct_norm, ct_depth, ct_pt2, ct_pt1, ct_eff_rad);
        for (int i = 0; i < nC; i++) {
            *(ct_norm + i) = -(*(ct_norm + i));
        }
        return (nC >= 0);
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

    if (shapeA->Type() == ChCollisionShape::Type::BOX && shapeB->Type() == ChCollisionShape::Type::TRIANGLE) {
        nC = triangle_box(shapeA->A(), shapeA->R(), shapeA->Box(), shapeB->Triangles(), separation, ct_norm, ct_depth,
                          ct_pt1, ct_pt2, ct_eff_rad);
        return false;
    }

    if (shapeA->Type() == ChCollisionShape::Type::TRIANGLE && shapeB->Type() == ChCollisionShape::Type::BOX) {
        nC = triangle_box(shapeB->A(), shapeB->R(), shapeB->Box(), shapeA->Triangles(), separation, ct_norm, ct_depth,
                          ct_pt2, ct_pt1, ct_eff_rad);
        for (int i = 0; i < nC; i++) {
            *(ct_norm + i) = -(*(ct_norm + i));
        }
        return false;
    }

    // Contact could not be checked using this CD algorithm
    return false;
}

}  // namespace chrono
