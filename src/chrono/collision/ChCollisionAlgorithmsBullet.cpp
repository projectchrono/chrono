// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/core/ChMathematics.h"
#include "chrono/collision/ChCollisionAlgorithmsBullet.h"
#include "chrono/collision/ChCollisionModelBullet.h"
#include "chrono/collision/ChCollisionUtils.h"

#include "chrono/collision/bullet/BulletCollision/CollisionShapes/btSphereShape.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/btCylinderShape.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/btBoxShape.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/btCylindricalShellShape.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/btCapsuleShape.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/bt2DShape.h"
#include "chrono/collision/bullet/BulletCollision/CollisionShapes/btCEtriangleShape.h"

namespace chrono {
namespace collision {

// ================================================================================================

// Snap the specified location to a point on a box with given half-dimensions.
// The in/out location is assumed to be specified in the frame of the box (which is therefore assumed to be an AABB
// centered at the origin).  The return code indicates the box axes that caused snapping:
//   - first bit (least significant) corresponds to x-axis
//   - second bit corresponds to y-axis
//   - third bit corresponds to z-axis
//
// Therefore:
//   code = 0 indicates an interior point
//   code = 1 or code = 2 or code = 4  indicates snapping to a face
//   code = 3 or code = 5 or code = 6  indicates snapping to an edge
//   code = 7 indicates snapping to a corner
int snap_point_to_box(const btVector3& hdims,  // box half-dimensions
                      btVector3& loc           // point (in/out)
) {
    int code = 0;

    if (std::abs(loc[0]) > hdims[0]) {
        code |= 1;
        loc[0] = (loc[0] > 0) ? hdims[0] : -hdims[0];
    }
    if (std::abs(loc[1]) > hdims[1]) {
        code |= 2;
        loc[1] = (loc[1] > 0) ? hdims[1] : -hdims[1];
    }
    if (std::abs(loc[2]) > hdims[2]) {
        code |= 4;
        loc[2] = (loc[2] > 0) ? hdims[2] : -hdims[2];
    }

    return code;
}

// ================================================================================================

// Check if given point is inside box (point expressed in box frame).
bool is_point_inside_box(const btVector3& hdims,  // box half-dimensions
                         const btVector3& loc     // point
) {
    for (int i = 0; i < 3; i++) {
        if (loc[i] > hdims[i] || loc[i] < -hdims[i])
            return false;
    }
    return true;
}

// ================================================================================================

// Find the closest box face to the given point (expressed in box frame).
// Returns +1, +2, +3 (for a "positive" face in x, y, z, respectively) or -1, -2, -3 (for a "negative" face).
int find_closest_box_face(const btVector3& hdims,  // box half-dimensions
                          const btVector3& loc     // point
) {
    int code = 0;
    btScalar dist = +BT_LARGE_FLOAT;
    for (int i = 0; i < 3; i++) {
        btScalar p_dist = std::abs(loc[i] - hdims[i]);
        btScalar n_dist = std::abs(loc[i] + hdims[i]);
        if (p_dist < dist) {
            code = i + 1;
            dist = p_dist;
        }
        if (n_dist < dist) {
            code = -(i + 1);
            dist = n_dist;
        }
    }

    return code;
}

// ================================================================================================

// Utility function for intersecting a box with a line segment.
// It is assumed that the box is centered at the origin and the segment is expressed in the box frame.
// The function returns false if the segment does not intersect the box.
// Algorithm:
//  - check intersection of the supporting line with the box slabs in the 3 directions
//  - if segment parallel to a slab, no intersection if segment far from box
//  - otherwise, update line parameters with points where line intersects box faces
//    (keep track of the most "inner" points at all times)
//  - finally, clamp to segment length
bool intersect_segment_box(const btVector3& hdims,  // box half-dimensions
                           const btVector3& c,      // segment center point
                           const btVector3& a,      // segment direction (unit vector)
                           const btScalar hlen,     // segment half-length
                           const btScalar tol,      // tolerance for parallelism test
                           btScalar& tMin,          // segment parameter of first intersection point
                           btScalar& tMax           // segment parameter of second intersection point
) {
    tMin = -BT_LARGE_FLOAT;
    tMax = +BT_LARGE_FLOAT;

    if (std::abs(a.x()) < tol) {
        // Segment parallel to the box x-faces
        if (std::abs(c.x()) > hdims.x())
            return false;
    } else {
        btScalar t1 = (-hdims.x() - c.x()) / a.x();
        btScalar t2 = (+hdims.x() - c.x()) / a.x();

        tMin = btMax(tMin, btMin(t1, t2));
        tMax = btMin(tMax, btMax(t1, t2));

        if (tMin > tMax)
            return false;
    }

    if (std::abs(a.y()) < tol) {
        // Segment parallel to the box y-faces
        if (std::abs(c.y()) > hdims.y())
            return false;
    } else {
        btScalar t1 = (-hdims.y() - c.y()) / a.y();
        btScalar t2 = (+hdims.y() - c.y()) / a.y();

        tMin = btMax(tMin, btMin(t1, t2));
        tMax = btMin(tMax, btMax(t1, t2));

        if (tMin > tMax)
            return false;

    }

    if (std::abs(a.z()) < tol) {
        // Capsule axis parallel to the box z-faces
        if (std::abs(c.z()) > hdims.z())
            return false;
    } else {
        btScalar t1 = (-hdims.z() - c.z()) / a.z();
        btScalar t2 = (+hdims.z() - c.z()) / a.z();

        tMin = btMax(tMin, btMin(t1, t2));
        tMax = btMin(tMax, btMax(t1, t2));

        if (tMin > tMax)
            return false;

    }

    // Clamp intersection points to segment length
    ChClampValue(tMin, -hlen, +hlen);
    ChClampValue(tMax, -hlen, +hlen);

    return true;
}

// ================================================================================================

btCapsuleBoxCollisionAlgorithm::btCapsuleBoxCollisionAlgorithm(btPersistentManifold* mf,
                                                               const btCollisionAlgorithmConstructionInfo& ci,
                                                               const btCollisionObjectWrapper* col0,
                                                               const btCollisionObjectWrapper* col1,
                                                               bool isSwapped)
    : btActivatingCollisionAlgorithm(ci, col0, col1), m_ownManifold(false), m_manifoldPtr(mf), m_isSwapped(isSwapped) {
    const btCollisionObjectWrapper* capsuleObjWrap = m_isSwapped ? col1 : col0;
    const btCollisionObjectWrapper* boxObjWrap = m_isSwapped ? col0 : col1;

    if (!m_manifoldPtr &&
        m_dispatcher->needsCollision(capsuleObjWrap->getCollisionObject(), boxObjWrap->getCollisionObject())) {
        m_manifoldPtr =
            m_dispatcher->getNewManifold(capsuleObjWrap->getCollisionObject(), boxObjWrap->getCollisionObject());
        m_ownManifold = true;
    }
}

btCapsuleBoxCollisionAlgorithm::btCapsuleBoxCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci)
    : btActivatingCollisionAlgorithm(ci) {}

btCapsuleBoxCollisionAlgorithm ::~btCapsuleBoxCollisionAlgorithm() {
    if (m_ownManifold) {
        if (m_manifoldPtr)
            m_dispatcher->releaseManifold(m_manifoldPtr);
    }
}

// Capsule-box intersection test.
void btCapsuleBoxCollisionAlgorithm::processCollision(const btCollisionObjectWrapper* body0,
                                                      const btCollisionObjectWrapper* body1,
                                                      const btDispatcherInfo& dispatchInfo,
                                                      btManifoldResult* resultOut) {
    (void)dispatchInfo;
    (void)resultOut;
    if (!m_manifoldPtr)
        return;

    const btCollisionObjectWrapper* capObjWrap = m_isSwapped ? body1 : body0;
    const btCollisionObjectWrapper* boxObjWrap = m_isSwapped ? body0 : body1;

    resultOut->setPersistentManifold(m_manifoldPtr);

    const btCapsuleShape* cap = (btCapsuleShape*)capObjWrap->getCollisionShape();
    const btBoxShape* box = (btBoxShape*)boxObjWrap->getCollisionShape();

    // Express capsule in the box frame
    const btTransform& abs_X_cap = capObjWrap->getWorldTransform();
    const btTransform& abs_X_box = boxObjWrap->getWorldTransform();
    btTransform box_X_cap = abs_X_box.inverseTimes(abs_X_cap);

    btVector3 a = box_X_cap.getBasis().getColumn(1);  // capsule axis (expressed in box frame)
    btVector3 c = box_X_cap.getOrigin();              // capsule center (expressed in box frame)

    // Box dimensions
    btVector3 hdims = box->getHalfExtentsWithMargin();

    // Cylinder dimensions
    btScalar radius = cap->getRadius();    // capsule radius
    btScalar hlen = cap->getHalfHeight();  // capsule half-length

    // Loop over each direction of the box frame (i.e., each of the 3 face normals).
    // In each case, consider two segments on the cylindrical surface that are on a plane defined by the axis and the
    // face normal. (Note that, in principle, we could only consider the segment "closest" to the box, but that is not
    // trivial to define in all configurations). Such segments are parameterized by t in [-H,H].
    //
    // Consider (at most) 4 candidate points on each segment: the 2 ends of the segment and the intersections of the
    // segment with the box (if such an intersection exists).
    //
    // For a capsule, projects these points back onto the capsule axis and check intersection between spheres centered
    // at the points on the axis and the box.  In this case, the projection is done orthogonal to the capsule axis.

    const btScalar parallel_tol = btScalar(1e-5);           // tolearance for parallelism tests
    const btScalar near_tol = btScalar(1e-4) * (2 * hlen);  // tolerance for line parameters of near duplicate points

    std::vector<btScalar> t_points = {-hlen, +hlen};  // initialize list of candidates with segment ends

    for (int i = 0; i < 3; i++) {
        // "positive" face normal
        btVector3 n(0, 0, 0);
        n[i] = 1;

        // If the axis is parallel to the face normal, no additional candidate point.
        btVector3 v = n.cross(a);
        if (std::abs(a[i] - 1) < parallel_tol)
            continue;

        // Direction perpendicular to axis
        btVector3 r = v.cross(a);

        // Construct center points of the two segments on cylindrical surface
        btVector3 c1 = c + radius * r;
        btVector3 c2 = c - radius * r;

        // Check if either segment intersects box. 
        // If it does, append line parameters for intersection points (clamped to segment limits).
        btScalar tMin;
        btScalar tMax;
        if (intersect_segment_box(hdims, c1, a, hlen, parallel_tol, tMin, tMax)) {
            t_points.push_back(ChClamp(tMin, -hlen, +hlen));
            t_points.push_back(ChClamp(tMax, -hlen, +hlen));
        }
        if (intersect_segment_box(hdims, c2, a, hlen, parallel_tol, tMin, tMax)) {
            t_points.push_back(ChClamp(tMin, -hlen, +hlen));
            t_points.push_back(ChClamp(tMax, -hlen, +hlen));
        }
    }

    // Contact distance
    btScalar contactDist = radius;
    ////btScalar contactDist = radius + m_manifoldPtr->getContactBreakingThreshold();

    // Loop over all candidate points (points on the capsule axis) and perform a sphere-box collision test.
    // In order to eliminate near duplicate points, use a sorted list and keep track of last processed point.
    std::sort(t_points.begin(), t_points.end());
    btScalar t_last = -2 * hlen;
    int n_contacts = 0;
    for (auto t : t_points) {
        if (t - t_last < near_tol)
            continue;

        // Update last processed point
        t_last = t;

        // Calculate sphere center (point on axis, expressed in box frame) and snap it to the surface of the box
        btVector3 sphPos = c + a * t;
        btVector3 boxPos = sphPos;
        snap_point_to_box(hdims, boxPos);

        // If the distance from the sphere center to the closest point is larger than the radius plus the separation
        // value, then there is no contact. Also, ignore contact if the sphere center (almost) coincides with the
        // closest point, in which case we couldn't decide on the proper contact direction.
        btVector3 delta = sphPos - boxPos;
        btScalar dist2 = delta.length2();
        if (dist2 >= contactDist * contactDist || dist2 <= 1e-12)
            continue;

        // Generate contact information (transform to absolute frame)
        btScalar dist = btSqrt(dist2);
        btScalar penetration = dist - radius;
        btVector3 normal = abs_X_box.getBasis() * (delta / dist);
        btVector3 point = abs_X_box(boxPos);

        // A new contact point must specify:
        //   normal, pointing from B towards A
        //   point, located on surface of B
        //   distance, negative for penetration
        resultOut->addContactPoint(normal, point, penetration);
        n_contacts++;
    }

    if (m_ownManifold && m_manifoldPtr->getNumContacts()) {
        resultOut->refreshContactPoints();
    }
}

btScalar btCapsuleBoxCollisionAlgorithm::calculateTimeOfImpact(btCollisionObject* body0,
                                                               btCollisionObject* body1,
                                                               const btDispatcherInfo& dispatchInfo,
                                                               btManifoldResult* resultOut) {
    // not yet
    return btScalar(1.);
}

void btCapsuleBoxCollisionAlgorithm::getAllContactManifolds(btManifoldArray& manifoldArray) {
    if (m_manifoldPtr && m_ownManifold) {
        manifoldArray.push_back(m_manifoldPtr);
    }
}

btCollisionAlgorithm* btCapsuleBoxCollisionAlgorithm::CreateFunc::CreateCollisionAlgorithm(
    btCollisionAlgorithmConstructionInfo& ci,
    const btCollisionObjectWrapper* body0Wrap,
    const btCollisionObjectWrapper* body1Wrap) {
    void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btCapsuleBoxCollisionAlgorithm));
    if (!m_swapped) {
        return new (mem) btCapsuleBoxCollisionAlgorithm(0, ci, body0Wrap, body1Wrap, false);
    } else {
        return new (mem) btCapsuleBoxCollisionAlgorithm(0, ci, body0Wrap, body1Wrap, true);
    }
}

// ================================================================================================

btCylshellBoxCollisionAlgorithm::btCylshellBoxCollisionAlgorithm(btPersistentManifold* mf,
                                                                 const btCollisionAlgorithmConstructionInfo& ci,
                                                                 const btCollisionObjectWrapper* col0,
                                                                 const btCollisionObjectWrapper* col1,
                                                                 bool isSwapped)
    : btActivatingCollisionAlgorithm(ci, col0, col1), m_ownManifold(false), m_manifoldPtr(mf), m_isSwapped(isSwapped) {
    const btCollisionObjectWrapper* cylshellObjWrap = m_isSwapped ? col1 : col0;
    const btCollisionObjectWrapper* boxObjWrap = m_isSwapped ? col0 : col1;

    if (!m_manifoldPtr &&
        m_dispatcher->needsCollision(cylshellObjWrap->getCollisionObject(), boxObjWrap->getCollisionObject())) {
        m_manifoldPtr =
            m_dispatcher->getNewManifold(cylshellObjWrap->getCollisionObject(), boxObjWrap->getCollisionObject());
        m_ownManifold = true;
    }
}

btCylshellBoxCollisionAlgorithm::btCylshellBoxCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci)
    : btActivatingCollisionAlgorithm(ci) {}

btCylshellBoxCollisionAlgorithm::~btCylshellBoxCollisionAlgorithm() {
    if (m_ownManifold) {
        if (m_manifoldPtr)
            m_dispatcher->releaseManifold(m_manifoldPtr);
    }
}

// Check and add contact between the given cylshell point and the specified box face.
// 'iface' is +1, +2, +3 for the "positive" x, y, or z box face, respectively.
// 'iface' is -1, -2, -3 for the "negative" x, y, or z box face, respectively.
int btCylshellBoxCollisionAlgorithm::addContactPoint(const btVector3& pc,
                                                     int iface,
                                                     const btTransform& X_box,
                                                     const btVector3& hdims,
                                                     btManifoldResult* resultOut) {
    assert(iface >= -3 && iface <= +3 && iface != 0);

    // No contact if point outside box
    if (!is_point_inside_box(hdims, pc))
        return 0;  // no contacts added

    // Find point projection on box face and calculate normal and penetration
    // (still working in the box frame)
    btVector3 p = pc;
    btVector3 n(0, 0, 0);
    btScalar penetration;
    if (iface > 0) {
        // "positive" box face
        int i = iface - 1;
        p[i] = hdims[i];
        n[i] = 1;
        penetration = pc[i] - hdims[i];
    } else {
        // "negative" box face
        int i = -iface - 1;
        p[i] = -hdims[i];
        n[i] = -1;
        penetration = -pc[i] - hdims[i];
    }

    // A new contact point must specify (in absolute frame):
    //   normal, pointing from B towards A
    //   point, located on surface of B
    //   distance, negative for penetration
    btVector3 normal = X_box.getBasis() * n;
    btVector3 point = X_box(p);
    resultOut->addContactPoint(normal, point, penetration);

    ////std::cout << "add contact     nrm = " << normal.x() << " " << normal.y() << " " << normal.z() << std::endl;
    ////std::cout << "                pnt = " << point.x() << " " << point.y() << " " << point.z() << std::endl;
    ////std::cout << "                pen = " << penetration << std::endl;

    return 1;  // one contact added
}

//// Different option (1, 2, 3, 4, 5)
#define CYLSHELL_BOX_ALG 5

// Cylshell-box intersection test:
//   - cylinder caps are ignored
//   - the cylshell is replaced with a capsule on the surface of the cylshell
//   - capsule-box intersection is then reduced to a segment-box intersection
//   - a replacement capsule (one for each direction of the box) may generate 0, 1, or 2 contacts
void btCylshellBoxCollisionAlgorithm::processCollision(const btCollisionObjectWrapper* body0,
                                                       const btCollisionObjectWrapper* body1,
                                                       const btDispatcherInfo& dispatchInfo,
                                                       btManifoldResult* resultOut) {
    (void)dispatchInfo;
    (void)resultOut;
    if (!m_manifoldPtr)
        return;

    const btCollisionObjectWrapper* cylObjWrap = m_isSwapped ? body1 : body0;
    const btCollisionObjectWrapper* boxObjWrap = m_isSwapped ? body0 : body1;

    resultOut->setPersistentManifold(m_manifoldPtr);

    const btCylindricalShellShape* cyl = (btCylindricalShellShape*)cylObjWrap->getCollisionShape();
    const btBoxShape* box = (btBoxShape*)boxObjWrap->getCollisionShape();

    // Express cylinder in the box frame
    const btTransform& abs_X_cyl = cylObjWrap->getWorldTransform();
    const btTransform& abs_X_box = boxObjWrap->getWorldTransform();
    btTransform box_X_cyl = abs_X_box.inverseTimes(abs_X_cyl);

    btVector3 a = box_X_cyl.getBasis().getColumn(1);  // cylinder axis (expressed in box frame)
    btVector3 c = box_X_cyl.getOrigin();              // cylinder center (expressed in box frame)

    // Box dimensions
    btVector3 hdims = box->getHalfExtentsWithMargin();

    // Cylinder dimensions
    btScalar radius = cyl->getRadius();    // cylinder radius
    btScalar hlen = cyl->getHalfLength();  // cylinder half-length

#if CYLSHELL_BOX_ALG == 5
    // - Loop over each direction of the box frame (i.e., each of the 3 face normals).
    // - For each direction, consider two segments on the cylindrical surface that are on a plane defined by the axis
    //   and the face normal. (Note that, in principle, we could only consider the segment "closest" to the box, but
    //   that is not trivial to define in all configurations). All segments are parameterized by t in [-H,H].
    // - For each segment, if the segment intersects the box, consider 3 candidate contact points: the 2 segment ends
    //   and the midpoint between the intersection points. A contact is added if the segment point is inside the box.
    //   Furthermore, the corresponding box point is located on the box face that is closest to the intersection
    //   midpoint candidate.

    const btScalar parallel_tol = btScalar(1e-5);           // tolearance for parallelism tests
    const btScalar near_tol = btScalar(1e-4) * (2 * hlen);  // tolerance for line parameters of near duplicate points

    int num_contacts = 0;
    for (int idir = 0; idir < 3; idir++) {
        // current box direction
        btVector3 ndir(0, 0, 0);
        ndir[idir] = 1;

        // If the axis is parallel to the current direction, no contact.
        if (std::abs(a[idir] - 1) < parallel_tol || std::abs(a[idir] + 1) < parallel_tol)
            continue;

        // Direction perpendicular to cylinder axis (in direction opposite to ndir)
        btVector3 v = ndir.cross(a);
        btVector3 r = v.cross(a);
        assert(r.length() > parallel_tol);
        r.normalize();

        // Consider segments in both "negative" and "positive" r direction
        btScalar dir[2] = {-1, 1};
        for (int jdir = 0; jdir < 2; jdir++) {
            // Calculate current segment center
            btVector3 cs = c + dir[jdir] * radius * r;
            // Check for intersection with box
            btScalar tMin, tMax;
            if (intersect_segment_box(hdims, cs, a, hlen, parallel_tol, tMin, tMax)) {
                // Consider the segment end points and the midpoint between intersection points as candidates
                btVector3 eMin = cs - a * hlen;                 // 1st segment end
                btVector3 eMax = cs + a * hlen;                 // 2nd segment end
                btVector3 pMid = cs + a * ((tMin + tMax) / 2);  // intersection midpoint

                // Pick box face that is closest to midpoint
                int iface = find_closest_box_face(hdims, pMid);

                // Add a contact for any of the candidate points that is inside the box
                num_contacts += addContactPoint(eMin, iface, abs_X_box, hdims, resultOut);  // 1st segment end
                num_contacts += addContactPoint(eMax, iface, abs_X_box, hdims, resultOut);  // 2nd segment end
                num_contacts += addContactPoint(pMid, iface, abs_X_box, hdims, resultOut);  // intersection midpoint
            }
        }
    }

    ////std::cout << num_contacts << std::endl;

#endif


#if CYLSHELL_BOX_ALG == 4
    // Loop over each direction of the box frame (i.e., each of the 3 face normals).
    // In each case, consider two segments on the cylindrical surface that are on a plane defined by the axis and the
    // face normal. (Note that, in principle, we could only consider the segment "closest" to the box, but that is not
    // trivial to define in all configurations). Such segments are parameterized by t in [-H,H].
    //
    // Consider (at most) 4 candidate points on each segment: the 2 ends and the intersections of the segment with the
    // box (if such an intersection exists).

    const btScalar parallel_tol = btScalar(1e-5);           // tolearance for parallelism tests
    const btScalar near_tol = btScalar(1e-4) * (2 * hlen);  // tolerance for line parameters of near duplicate points

    ////
    //// TODO:  MUST collect all candidate segments over all box faces and only add contacts for one combination at any given time.
    ////

    int num_contacts = 0;
    for (int i = 0; i < 3; i++) {
        // "positive" face normal
        btVector3 n(0, 0, 0);
        n[i] = 1;

        // If the axis is parallel to the face normal, no contact.
        if (std::abs(a[i] - 1) < parallel_tol || std::abs(a[idir] + 1) < parallel_tol)
            continue;

        // Direction perpendicular to cylinder axis (in direction opposite to n)
        btVector3 v = n.cross(a);
        btVector3 r = v.cross(a);
        r.normalize();

        // Consider segments in both "negative" and "positive" r direction
        double dir[2] = {-1, 1};
        for (int idir = 0; idir < 2; idir++) {
            // Calculate current segment center
            btVector3 cs = c + dir[idir] * radius * r;
            // Check for intersection with box
            btScalar tMin, tMax;
            if (intersect_segment_box(hdims, cs, a, hlen, parallel_tol, tMin, tMax)) {
                // Consider the segment end points and the midpoint between intersection points as candidates
                btVector3 pMid = cs + a * ((tMin + tMax) / 2);  // intersection midpoint
                btVector3 eMin = cs - a * hlen;                 // 1st segment end
                btVector3 eMax = cs + a * hlen;                 // 2nd segment end

                // Pick "positive" or "negative" box face in current direction (closest to midpoint)
                int iface = (pMid[i] > 0) ? i : -i;

                // Add a contact for any of the candidate points that is inside the box
                num_contacts += addContactPoint(eMin, iface, abs_X_box, hdims, resultOut);  // 1st segment end
                num_contacts += addContactPoint(eMax, iface, abs_X_box, hdims, resultOut);  // 2nd segment end
                num_contacts += addContactPoint(pMid, iface, abs_X_box, hdims, resultOut);  // intersection midpoint
            }
        }
    }

    ////std::cout << num_contacts << std::endl;

#endif

#if CYLSHELL_BOX_ALG == 3
    btVector3 dummy = c;
    int code = snap_point_to_box(hdims, dummy);
    int i;
    switch (code) {
        case 1:
            i = 0;
            break;
        case 2:
            i = 1;
            break;
        case 4:
            i = 2;
            break;
        default:
            return;
    }

    int j = (i + 1) % 3;
    int k = (i + 2) % 3;
    int sign = btSign(c[i]);

    btScalar tMin = -BT_LARGE_FLOAT;
    btScalar tMax = +BT_LARGE_FLOAT;

    const btScalar threshold = btScalar(1e-5);  // threshold for line parallel to face tests

    if (std::abs(a[j]) > threshold) {
        btScalar t1 = (-hdims[j] - c[j]) / a[j];
        btScalar t2 = (+hdims[j] - c[j]) / a[j];

        tMin = btMax(tMin, btMin(t1, t2));
        tMax = btMin(tMax, btMax(t1, t2));

        if (tMin > tMax)
            return;
    }

    if (std::abs(a[k]) > threshold) {
        btScalar t1 = (-hdims[k] - c[k]) / a[k];
        btScalar t2 = (+hdims[k] - c[k]) / a[k];

        tMin = btMax(tMin, btMin(t1, t2));
        tMax = btMin(tMax, btMax(t1, t2));

        if (tMin > tMax)
            return;
    }

    btVector3 locs[2] = {c + tMin * a, c + tMax * a};
    btScalar t[2];
    for (int i = 0; i < 2; i++) {
        t[i] = btClamped(a.dot(locs[i] - c), -hlen, hlen);
    }

    // Check if the two points almost coincide (in which case consider only one of them)
    int numPoints = std::abs(t[0] - t[1]) < 1e-4 ? 1 : 2;

    for (int ip = 0; ip < numPoints; ip++) {
        // Calculate the center of the corresponding sphere on the capsule centerline (expressed in the box
        // frame).
        btVector3 spherePos = c + a * t[ip];

        // Snap the sphere position to the surface of the box.
        btVector3 boxPos = spherePos;
        snap_point_to_box(hdims, boxPos);

        // If the distance from the sphere center to the closest point is larger than the radius plus the
        // separation value, then there is no contact. Also, ignore contact if the sphere center (almost)
        // coincides with the closest point, in which case we couldn't decide on the proper contact direction.
        btVector3 delta = spherePos - boxPos;
        btScalar dist2 = delta.length2();

        if (dist2 >= radius * radius || dist2 <= 1e-12)
            continue;

        // Generate contact information.
        btScalar dist = btSqrt(dist2);
        btScalar penetration = dist - radius;
        // Transform to absolute frame
        btVector3 normal = abs_X_box.getBasis() * (delta / dist);
        btVector3 point = abs_X_box(boxPos);

        // A new contact point must specify:
        //   normal, pointing from B towards A
        //   point, located on surface of B
        //   distance, negative for penetration
        resultOut->addContactPoint(normal, point, penetration);

        ////std::cout << "add --  t= " << t[ip] << "  dist= " << dist << "  depth= " << penetration << std::endl;
    }
#endif

#if CYLSHELL_BOX_ALG == 2
    // Inflate the box by the radius of the capsule plus the separation value and check if the capsule centerline
    // intersects the expanded box. We do this by clamping the capsule axis to the volume between two parallel faces
    // of the box, considering in turn the x, y, and z faces.
    btVector3 hdims_exp = hdims + btVector3(radius, radius, radius);
    btScalar tMin = -BT_LARGE_FLOAT;
    btScalar tMax = +BT_LARGE_FLOAT;

    const btScalar threshold = btScalar(1e-5);  // threshold for line parallel to face tests

    if (std::abs(a.x()) < threshold) {
        // Capsule axis parallel to the box x-faces
        if (std::abs(c.x()) > hdims_exp.x())
            return;
    } else {
        btScalar t1 = (-hdims_exp.x() - c.x()) / a.x();
        btScalar t2 = (+hdims_exp.x() - c.x()) / a.x();

        tMin = btMax(tMin, btMin(t1, t2));
        tMax = btMin(tMax, btMax(t1, t2));

        if (tMin > tMax)
            return;
    }

    if (std::abs(a.y()) < threshold) {
        // Capsule axis parallel to the box y-faces
        if (std::abs(c.y()) > hdims_exp.y())
            return;
    } else {
        btScalar t1 = (-hdims_exp.y() - c.y()) / a.y();
        btScalar t2 = (+hdims_exp.y() - c.y()) / a.y();

        tMin = btMax(tMin, btMin(t1, t2));
        tMax = btMin(tMax, btMax(t1, t2));

        if (tMin > tMax)
            return;
    }

    if (std::abs(a.z()) < threshold) {
        // Capsule axis parallel to the box z-faces
        if (std::abs(c.z()) > hdims_exp.z())
            return;
    } else {
        btScalar t1 = (-hdims_exp.z() - c.z()) / a.z();
        btScalar t2 = (+hdims_exp.z() - c.z()) / a.z();

        tMin = btMax(tMin, btMin(t1, t2));
        tMax = btMin(tMax, btMax(t1, t2));

        if (tMin > tMax)
            return;
    }

    // Generate the two points where the cylinder centerline intersects the exapanded box (still expressed in the
    // box frame). Snap these locations to the original box, then snap back onto the cylinder axis. This reduces
    // the collision problem to 1 or 2 collisions.
    btVector3 locs[2] = {c + tMin * a, c + tMax * a};
    btScalar t[2];

    for (int i = 0; i < 2; i++) {
        snap_point_to_box(hdims, locs[i]);
        t[i] = btClamped(a.dot(locs[i] - c), -hlen, hlen);
    }

    // Check if the two points almost coincide (in which case consider only one of them)
    int numPoints = std::abs(t[0] - t[1]) < 1e-4 ? 1 : 2;

    // Perform collision tests.
    for (int i = 0; i < numPoints; i++) {
        // Point on the cylinder axis (expressed in the box frame).
        btVector3 axisPoint = c + a * t[i];

        // Snap to box. If axis point inside box, no contact.
        btVector3 boxPoint = axisPoint;
        int code = snap_point_to_box(hdims, boxPoint);
        if (code == 0)
            continue;

        // Find closest point on cylinder to the box. If this point is on the cylinder centerline, no contact.
        btVector3 u = axisPoint - boxPoint;
        btScalar u_length = u.length();
        if (u_length < threshold)
            continue;
        u /= u_length;
        btVector3 w = u.cross(a);
        if (w.length() < threshold)
            continue;
        btVector3 v = w.cross(a);
        v.normalize();
        btVector3 cylPoint = axisPoint + radius * v;

        // If cylinder point outside box, no contact.
        code = snap_point_to_box(hdims, cylPoint);
        if (code != 0)
            continue;

        // Moving in the u direction, project cylinder point onto box surface.
        btScalar step = BT_LARGE_FLOAT;
        if (std::abs(u.x()) > threshold)
            step = btMin((btSign(u.x()) * hdims.x() - cylPoint.x()) / u.x(), step);
        if (std::abs(u.y()) > threshold)
            step = btMin((btSign(u.y()) * hdims.y() - cylPoint.y()) / u.y(), step);
        if (std::abs(u.z()) > threshold)
            step = btMin((btSign(u.z()) * hdims.z() - cylPoint.z()) / u.z(), step);
        boxPoint = cylPoint + step * u;

        // Debug check: boxPoint inside cylinder
        assert(std::abs(a.dot(boxPoint - c)) <= btScalar(1.01) * hlen);

        // Calculate penetration
        btVector3 delta = boxPoint - cylPoint;
        btScalar dist = delta.length();
        if (dist < 1e-10)
            continue;

        // Generate contact information (transform to absolute frame).
        btScalar penetration = -dist;                              // distance, negative for penetration
        btVector3 normal = abs_X_box.getBasis() * (delta / dist);  // normal, pointing from B to A
        btVector3 point = abs_X_box(boxPoint);                     // point, located on surface of B

        resultOut->addContactPoint(normal, point, penetration);

        ////std::cout << "add contact --  t= " << t[i] << "  face " << i << "  dist= " << dist << "  depth= " << penetration << std::endl;
    }
#endif

#if CYLSHELL_BOX_ALG == 1
    //==  USE CAPSULE ==//

    // Contact distance
    btScalar contactDist = radius;
    ////btScalar contactDist = radius + m_manifoldPtr->getContactBreakingThreshold();

    // Inflate the box by the radius of the capsule plus the separation value
    // and check if the capsule centerline intersects the expanded box. We do
    // this by clamping the capsule axis to the volume between two parallel
    // faces of the box, considering in turn the x, y, and z faces.
    btVector3 hdims_exp = hdims + btVector3(radius, radius, radius);
    btScalar tMin = -BT_LARGE_FLOAT;
    btScalar tMax = +BT_LARGE_FLOAT;

    const btScalar threshold = btScalar(1e-5);  // threshold for line parallel to face tests

    if (std::abs(a.x()) < threshold) {
        // Capsule axis parallel to the box x-faces
        if (std::abs(c.x()) > hdims_exp.x())
            return;
    } else {
        btScalar t1 = (-hdims_exp.x() - c.x()) / a.x();
        btScalar t2 = (+hdims_exp.x() - c.x()) / a.x();

        tMin = btMax(tMin, btMin(t1, t2));
        tMax = btMin(tMax, btMax(t1, t2));

        if (tMin > tMax)
            return;
    }

    if (std::abs(a.y()) < threshold) {
        // Capsule axis parallel to the box y-faces
        if (std::abs(c.y()) > hdims_exp.y())
            return;
    } else {
        btScalar t1 = (-hdims_exp.y() - c.y()) / a.y();
        btScalar t2 = (+hdims_exp.y() - c.y()) / a.y();

        tMin = btMax(tMin, btMin(t1, t2));
        tMax = btMin(tMax, btMax(t1, t2));

        if (tMin > tMax)
            return;
    }

    if (std::abs(a.z()) < threshold) {
        // Capsule axis parallel to the box z-faces
        if (std::abs(c.z()) > hdims_exp.z())
            return;
    } else {
        btScalar t1 = (-hdims_exp.z() - c.z()) / a.z();
        btScalar t2 = (+hdims_exp.z() - c.z()) / a.z();

        tMin = btMax(tMin, btMin(t1, t2));
        tMax = btMin(tMax, btMax(t1, t2));

        if (tMin > tMax)
            return;
    }

    // Generate the two points where the capsule centerline intersects
    // the exapanded box (still expressed in the box frame). Snap these
    // locations onto the original box, then snap back onto the capsule
    // axis. This reduces the collision problem to 1 or 2 box-sphere
    // collisions.
    btVector3 locs[2] = {c + tMin * a, c + tMax * a};
    btScalar t[2];

    for (int i = 0; i < 2; i++) {
        snap_point_to_box(hdims, locs[i]);
        t[i] = btClamped(a.dot(locs[i] - c), -hlen, hlen);
    }

    // Check if the two points almost coincide (in which case consider only one of them)
    int numSpheres = std::abs(t[0] - t[1]) < 1e-4 ? 1 : 2;

    // Perform box-sphere tests, and keep track of actual number of contacts.
    int j = 0;

    for (int i = 0; i < numSpheres; i++) {
        // Calculate the center of the corresponding sphere on the capsule centerline (expressed in the box frame).
        btVector3 spherePos = c + a * t[i];

        // Snap the sphere position to the surface of the box.
        btVector3 boxPos = spherePos;
        snap_point_to_box(hdims, boxPos);

        // If the distance from the sphere center to the closest point is larger than the radius plus the separation
        // value, then there is no contact. Also, ignore contact if the sphere center (almost) coincides with the
        // closest point, in which case we couldn't decide on the proper contact direction.
        btVector3 delta = spherePos - boxPos;
        btScalar dist2 = delta.length2();

        if (dist2 >= contactDist * contactDist || dist2 <= 1e-12)
            continue;

        // Generate contact information.
        btScalar dist = btSqrt(dist2);
        btScalar penetration = dist - radius;
        // Transform to absolute frame
        btVector3 normal = abs_X_box.getBasis() * (delta / dist);
        btVector3 point = abs_X_box(boxPos);

        // A new contact point must specify:
        //   normal, pointing from B towards A
        //   point, located on surface of B
        //   distance, negative for penetration
        resultOut->addContactPoint(normal, point, penetration);

        ////std::cout << "add contact --  t= " << t[i] << "  face " << i << "  dist= " << dist << "  depth= " << penetration << std::endl;
    }
#endif

    if (m_ownManifold && m_manifoldPtr->getNumContacts()) {
        resultOut->refreshContactPoints();
    }
}

btScalar btCylshellBoxCollisionAlgorithm::calculateTimeOfImpact(btCollisionObject* body0,
                                                                btCollisionObject* body1,
                                                                const btDispatcherInfo& dispatchInfo,
                                                                btManifoldResult* resultOut) {
    // not yet
    return btScalar(1.);
}

void btCylshellBoxCollisionAlgorithm::getAllContactManifolds(btManifoldArray& manifoldArray) {
    if (m_manifoldPtr && m_ownManifold) {
        manifoldArray.push_back(m_manifoldPtr);
    }
}

btCollisionAlgorithm* btCylshellBoxCollisionAlgorithm::CreateFunc::CreateCollisionAlgorithm(
    btCollisionAlgorithmConstructionInfo& ci,
    const btCollisionObjectWrapper* body0Wrap,
    const btCollisionObjectWrapper* body1Wrap) {
    void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btCylshellBoxCollisionAlgorithm));
    if (!m_swapped) {
        return new (mem) btCylshellBoxCollisionAlgorithm(0, ci, body0Wrap, body1Wrap, false);
    } else {
        return new (mem) btCylshellBoxCollisionAlgorithm(0, ci, body0Wrap, body1Wrap, true);
    }
}

// ================================================================================================

btSphereCylinderCollisionAlgorithm::btSphereCylinderCollisionAlgorithm(btPersistentManifold* mf,
                                                                       const btCollisionAlgorithmConstructionInfo& ci,
                                                                       const btCollisionObjectWrapper* col0,
                                                                       const btCollisionObjectWrapper* col1,
                                                                       bool isSwapped)
    : btActivatingCollisionAlgorithm(ci, col0, col1), m_ownManifold(false), m_manifoldPtr(mf), m_isSwapped(isSwapped) {
    const btCollisionObjectWrapper* sphereObj = m_isSwapped ? col1 : col0;
    const btCollisionObjectWrapper* cylObj = m_isSwapped ? col0 : col1;

    if (!m_manifoldPtr && m_dispatcher->needsCollision(sphereObj->getCollisionObject(), cylObj->getCollisionObject())) {
        m_manifoldPtr = m_dispatcher->getNewManifold(sphereObj->getCollisionObject(), cylObj->getCollisionObject());
        m_ownManifold = true;
    }
}

btSphereCylinderCollisionAlgorithm::btSphereCylinderCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci)
    : btActivatingCollisionAlgorithm(ci) {}

btSphereCylinderCollisionAlgorithm ::~btSphereCylinderCollisionAlgorithm() {
    if (m_ownManifold) {
        if (m_manifoldPtr)
            m_dispatcher->releaseManifold(m_manifoldPtr);
    }
}

void btSphereCylinderCollisionAlgorithm::processCollision(const btCollisionObjectWrapper* body0,
                                                          const btCollisionObjectWrapper* body1,
                                                          const btDispatcherInfo& dispatchInfo,
                                                          btManifoldResult* resultOut) {
    (void)dispatchInfo;
    (void)resultOut;
    if (!m_manifoldPtr)
        return;

    const btCollisionObjectWrapper* sphereObjWrap = m_isSwapped ? body1 : body0;
    const btCollisionObjectWrapper* cylObjWrap = m_isSwapped ? body0 : body1;

    resultOut->setPersistentManifold(m_manifoldPtr);

    const btSphereShape* sphere0 = (btSphereShape*)sphereObjWrap->getCollisionShape();
    const btCylinderShape* cylinder = (btCylinderShape*)cylObjWrap->getCollisionShape();

    const btTransform& m44T = cylObjWrap->getCollisionObject()->getWorldTransform();
    btVector3 diff = m44T.invXform(
        sphereObjWrap->getCollisionObject()
            ->getWorldTransform()
            .getOrigin());  // col0->getWorldTransform().getOrigin()-  col1->getWorldTransform().getOrigin();
    btScalar radius0 = sphere0->getRadius();
    btScalar radius1 = cylinder->getHalfExtentsWithMargin().getX();  // cylinder->getRadius();
    btScalar H1 = cylinder->getHalfExtentsWithMargin().getY();

    btVector3 r1 = diff;
    r1.setY(0);

    btScalar y1 = diff.y();

    btScalar r1_len = r1.length();

    btVector3 pos1;
    btVector3 normalOnSurfaceB(1, 0, 0);
    btScalar dist;

    // Case A
    if ((y1 <= H1) && (y1 >= -H1)) {
        /// iff distance positive, don't generate a new contact
        if (r1_len > (radius0 + radius1)) {
            resultOut->refreshContactPoints();
            return;
        }
        /// distance (negative means penetration)
        dist = r1_len - (radius0 + radius1);

        btVector3 localnormalOnSurfaceB;
        if (r1_len > SIMD_EPSILON) {
            localnormalOnSurfaceB = r1 / r1_len;
            normalOnSurfaceB = m44T.getBasis() * localnormalOnSurfaceB;
        }
        /// point on B (worldspace)
        pos1 = m44T(btVector3(0, y1, 0)) + radius1 * normalOnSurfaceB;
    } else {
        btScalar side = 1;
        if (y1 < -H1)
            side = -1;

        if (r1_len > radius1) {
            // case B
            btVector3 pos_loc = r1.normalized() * radius1 + btVector3(0, H1 * side, 0);
            pos1 = m44T(pos_loc);
            btVector3 d = sphereObjWrap->getCollisionObject()->getWorldTransform().getOrigin() - pos1;
            normalOnSurfaceB = d.normalized();
            dist = d.length() - radius0;
        } else {
            // case C
            normalOnSurfaceB = m44T.getBasis() * btVector3(0, 1 * side, 0);
            btVector3 pos_loc = r1 + btVector3(0, H1 * side, 0);
            pos1 = m44T(pos_loc);
            dist = side * (y1 - H1) - radius0;
        }
    }
    /// report a contact. internally this will be kept persistent, and contact reduction is done
    resultOut->addContactPoint(normalOnSurfaceB, pos1, dist);

    resultOut->refreshContactPoints();
}

btScalar btSphereCylinderCollisionAlgorithm::calculateTimeOfImpact(btCollisionObject* body0,
                                                                   btCollisionObject* body1,
                                                                   const btDispatcherInfo& dispatchInfo,
                                                                   btManifoldResult* resultOut) {
    // not yet
    return btScalar(1.);
}

void btSphereCylinderCollisionAlgorithm::getAllContactManifolds(btManifoldArray& manifoldArray) {
    if (m_manifoldPtr && m_ownManifold) {
        manifoldArray.push_back(m_manifoldPtr);
    }
}

btCollisionAlgorithm* btSphereCylinderCollisionAlgorithm::CreateFunc::CreateCollisionAlgorithm(
    btCollisionAlgorithmConstructionInfo& ci,
    const btCollisionObjectWrapper* body0Wrap,
    const btCollisionObjectWrapper* body1Wrap) {
    void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btSphereCylinderCollisionAlgorithm));
    if (!m_swapped) {
        return new (mem) btSphereCylinderCollisionAlgorithm(0, ci, body0Wrap, body1Wrap, false);
    } else {
        return new (mem) btSphereCylinderCollisionAlgorithm(0, ci, body0Wrap, body1Wrap, true);
    }
}

// ================================================================================================

btArcSegmentCollisionAlgorithm::btArcSegmentCollisionAlgorithm(btPersistentManifold* mf,
                                                               const btCollisionAlgorithmConstructionInfo& ci,
                                                               const btCollisionObjectWrapper* col0,
                                                               const btCollisionObjectWrapper* col1,
                                                               bool isSwapped)
    : btActivatingCollisionAlgorithm(ci, col0, col1), m_ownManifold(false), m_manifoldPtr(mf), m_isSwapped(isSwapped) {
    const btCollisionObjectWrapper* arcObjWrap = m_isSwapped ? col1 : col0;
    const btCollisionObjectWrapper* segmentObjWrap = m_isSwapped ? col0 : col1;

    if (!m_manifoldPtr &&
        m_dispatcher->needsCollision(arcObjWrap->getCollisionObject(), segmentObjWrap->getCollisionObject())) {
        m_manifoldPtr =
            m_dispatcher->getNewManifold(arcObjWrap->getCollisionObject(), segmentObjWrap->getCollisionObject());
        m_ownManifold = true;
    }
}

btArcSegmentCollisionAlgorithm::btArcSegmentCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci)
    : btActivatingCollisionAlgorithm(ci) {}

btArcSegmentCollisionAlgorithm ::~btArcSegmentCollisionAlgorithm() {
    if (m_ownManifold) {
        if (m_manifoldPtr)
            m_dispatcher->releaseManifold(m_manifoldPtr);
    }
}

void btArcSegmentCollisionAlgorithm::processCollision(const btCollisionObjectWrapper* body0,
                                                      const btCollisionObjectWrapper* body1,
                                                      const btDispatcherInfo& dispatchInfo,
                                                      btManifoldResult* resultOut) {
    (void)dispatchInfo;
    (void)resultOut;
    if (!m_manifoldPtr)
        return;

    const btCollisionObjectWrapper* arcObjWrap = m_isSwapped ? body1 : body0;
    const btCollisionObjectWrapper* segmentObjWrap = m_isSwapped ? body0 : body1;

    resultOut->setPersistentManifold(m_manifoldPtr);

    // only 1 contact per pair, avoid persistence
    resultOut->getPersistentManifold()->clearManifold();

    const bt2DarcShape* arc = (bt2DarcShape*)arcObjWrap->getCollisionShape();
    const bt2DsegmentShape* segment = (bt2DsegmentShape*)segmentObjWrap->getCollisionShape();

    // A concave arc (i.e.with outward volume, counterclockwise abscissa) will never collide with segments
    if (arc->get_counterclock())
        return;

    const btTransform& m44Tarc = arcObjWrap->getCollisionObject()->getWorldTransform();
    const btTransform& m44Tsegment = segmentObjWrap->getCollisionObject()->getWorldTransform();

    // Shapes on two planes that are not so parallel? no collisions!
    btVector3 Zarc = m44Tarc.getBasis().getColumn(2);
    btVector3 Zsegment = m44Tsegment.getBasis().getColumn(2);
    if (fabs(Zarc.dot(Zsegment)) < 0.99)  //***TODO*** threshold as setting
        return;

    // Shapes on two planes that are too far? no collisions!
    btVector3 diff = m44Tsegment.invXform(m44Tarc.getOrigin());
    if (fabs(diff.getZ()) > (arc->get_zthickness() + segment->get_zthickness()))
        return;

    // vectors of body 1 in body 2 csys:
    btVector3 local_arc_center = m44Tsegment.invXform(m44Tarc * btVector3(arc->get_X(), arc->get_Y(), 0));
    btVector3 local_arc_X = m44Tsegment.getBasis().transpose() * (m44Tarc.getBasis() * btVector3(1, 0, 0));
    double local_arc_rot = atan2(local_arc_X.getY(), local_arc_X.getX());
    double arc1_angle1 = local_arc_rot + arc->get_angle1();
    double arc1_angle2 = local_arc_rot + arc->get_angle2();

    btVector3 local_CS1 = local_arc_center - segment->get_P1();
    btVector3 local_seg_S2S1 = (segment->get_P2() - segment->get_P1());
    btScalar seg_length = local_seg_S2S1.length();
    if (seg_length < 1e-30)
        return;
    btVector3 local_seg_D = local_seg_S2S1 / seg_length;
    btScalar param = local_CS1.dot(local_seg_D);

    // contact out of segment extrema?
    if (param < 0)
        return;
    if (param > seg_length)
        return;

    btVector3 local_P2 = segment->get_P1() + local_seg_D * param;
    btVector3 local_CP2 = local_arc_center - local_P2;
    local_CP2.setZ(0);
    btVector3 local_R = local_CP2.normalized() * arc->get_radius();
    btVector3 local_P1;
    btVector3 local_N2;
    if (local_seg_S2S1.cross(local_CP2).getZ() > 0) {
        local_P1 = local_arc_center - local_R;
        local_N2 = local_CP2.normalized();
    } else {
        local_P1 = local_arc_center + local_R;
        local_N2 = -local_CP2.normalized();
    }
    btVector3 local_P1P2 = local_P1 - local_P2;

    double alpha = atan2(-local_N2.getY(), -local_N2.getX());

    // Discard points out of min-max angles

    // to always positive angles:
    arc1_angle1 = fmod(arc1_angle1 + 1e-30, CH_C_2PI);
    if (arc1_angle1 < 0)
        arc1_angle1 += CH_C_2PI;
    arc1_angle2 = fmod(arc1_angle2 + 1e-30, CH_C_2PI);
    if (arc1_angle2 < 0)
        arc1_angle2 += CH_C_2PI;
    alpha = fmod(alpha, CH_C_2PI);
    if (alpha < 0)
        alpha += CH_C_2PI;

    arc1_angle1 = fmod(arc1_angle1, CH_C_2PI);
    arc1_angle2 = fmod(arc1_angle2, CH_C_2PI);

    alpha = fmod(alpha, CH_C_2PI);

    bool inangle1 = false;

    if (arc1_angle1 < arc1_angle2) {
        if (alpha >= arc1_angle2 || alpha <= arc1_angle1)
            inangle1 = true;
    } else {
        if (alpha >= arc1_angle2 && alpha <= arc1_angle1)
            inangle1 = true;
    }

    if (!inangle1)
        return;

    // transform in absolute coords:
    // btVector3 pos1 = m44Tsegment * local_P1; // not needed
    btVector3 pos2 = m44Tsegment * local_P2;
    btVector3 normal_on_2 = m44Tsegment.getBasis() * local_N2;
    btScalar dist = local_N2.dot(local_P1 - local_P2);

    // too far or too interpenetrate? discard.
    if (fabs(dist) > (arc->getMargin() + segment->getMargin()))
        return;

    /// report a contact. internally this will be kept persistent, and contact reduction is done
    resultOut->addContactPoint(normal_on_2, pos2, dist);

    resultOut->refreshContactPoints();
}

btScalar btArcSegmentCollisionAlgorithm::calculateTimeOfImpact(btCollisionObject* body0,
                                                               btCollisionObject* body1,
                                                               const btDispatcherInfo& dispatchInfo,
                                                               btManifoldResult* resultOut) {
    // not yet
    return btScalar(1.);
}

void btArcSegmentCollisionAlgorithm::getAllContactManifolds(btManifoldArray& manifoldArray) {
    if (m_manifoldPtr && m_ownManifold) {
        manifoldArray.push_back(m_manifoldPtr);
    }
}

btCollisionAlgorithm* btArcSegmentCollisionAlgorithm::CreateFunc::CreateCollisionAlgorithm(
    btCollisionAlgorithmConstructionInfo& ci,
    const btCollisionObjectWrapper* body0Wrap,
    const btCollisionObjectWrapper* body1Wrap) {
    void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btArcSegmentCollisionAlgorithm));
    if (!m_swapped) {
        return new (mem) btArcSegmentCollisionAlgorithm(0, ci, body0Wrap, body1Wrap, false);
    } else {
        return new (mem) btArcSegmentCollisionAlgorithm(0, ci, body0Wrap, body1Wrap, true);
    }
}

// ================================================================================================

btArcArcCollisionAlgorithm::btArcArcCollisionAlgorithm(btPersistentManifold* mf,
                                                       const btCollisionAlgorithmConstructionInfo& ci,
                                                       const btCollisionObjectWrapper* col0,
                                                       const btCollisionObjectWrapper* col1,
                                                       bool isSwapped)
    : btActivatingCollisionAlgorithm(ci, col0, col1), m_ownManifold(false), m_manifoldPtr(mf), m_isSwapped(isSwapped) {
    const btCollisionObjectWrapper* arcObj1Wrap = m_isSwapped ? col1 : col0;
    const btCollisionObjectWrapper* arcObj2Wrap = m_isSwapped ? col0 : col1;

    if (!m_manifoldPtr &&
        m_dispatcher->needsCollision(arcObj1Wrap->getCollisionObject(), arcObj2Wrap->getCollisionObject())) {
        m_manifoldPtr =
            m_dispatcher->getNewManifold(arcObj1Wrap->getCollisionObject(), arcObj2Wrap->getCollisionObject());
        m_ownManifold = true;
    }
}

btArcArcCollisionAlgorithm::btArcArcCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci)
    : btActivatingCollisionAlgorithm(ci) {}

btArcArcCollisionAlgorithm ::~btArcArcCollisionAlgorithm() {
    if (m_ownManifold) {
        if (m_manifoldPtr)
            m_dispatcher->releaseManifold(m_manifoldPtr);
    }
}

void btArcArcCollisionAlgorithm::processCollision(const btCollisionObjectWrapper* body0,
                                                  const btCollisionObjectWrapper* body1,
                                                  const btDispatcherInfo& dispatchInfo,
                                                  btManifoldResult* resultOut) {
    (void)dispatchInfo;
    (void)resultOut;
    if (!m_manifoldPtr)
        return;

    const btCollisionObjectWrapper* arcObj1Wrap = m_isSwapped ? body1 : body0;
    const btCollisionObjectWrapper* arcObj2Wrap = m_isSwapped ? body0 : body1;

    resultOut->setPersistentManifold(m_manifoldPtr);

    // only 1 contact per pair, avoid persistence
    resultOut->getPersistentManifold()->clearManifold();

    const bt2DarcShape* arc1 = (bt2DarcShape*)arcObj1Wrap->getCollisionShape();
    const bt2DarcShape* arc2 = (bt2DarcShape*)arcObj2Wrap->getCollisionShape();

    const btTransform& m44Tarc1 = arcObj1Wrap->getCollisionObject()->getWorldTransform();
    const btTransform& m44Tarc2 = arcObj2Wrap->getCollisionObject()->getWorldTransform();

    // Shapes on two planes that are not so parallel? no collisions!
    btVector3 Zarc1 = m44Tarc1.getBasis().getColumn(2);
    btVector3 Zarc2 = m44Tarc2.getBasis().getColumn(2);
    if (fabs(Zarc1.dot(Zarc2)) < 0.99)  //***TODO*** threshold as setting
        return;

    // Shapes on two planes that are too far? no collisions!
    btVector3 diff = m44Tarc2.invXform(m44Tarc1.getOrigin());
    if (fabs(diff.getZ()) > (arc1->get_zthickness() + arc2->get_zthickness()))
        return;

    // vectors and angles of arc 1 in arc 2 csys:
    btVector3 local_arc1_center = m44Tarc2.invXform(m44Tarc1 * btVector3(arc1->get_X(), arc1->get_Y(), 0));
    btVector3 local_arc1_X = m44Tarc2.getBasis().transpose() * (m44Tarc1.getBasis() * btVector3(1, 0, 0));
    double local_arc1_rot = atan2(local_arc1_X.getY(), local_arc1_X.getX());
    double arc1_angle1 = local_arc1_rot + arc1->get_angle1();
    double arc1_angle2 = local_arc1_rot + arc1->get_angle2();

    btVector3 local_arc2_center = btVector3(arc2->get_X(), arc2->get_Y(), 0);
    double arc2_angle1 = arc2->get_angle1();
    double arc2_angle2 = arc2->get_angle2();

    btVector3 local_C1C2 = local_arc1_center - local_arc2_center;
    btVector3 local_D12 = local_C1C2.normalized();

    btVector3 local_P1;
    btVector3 local_P2;
    btVector3 local_N2;
    double dist = 0;
    bool paired = false;
    double alpha = atan2(local_C1C2.getY(), local_C1C2.getX());
    double alpha1, alpha2;

    // convex-convex
    if (arc1->get_counterclock() == false && arc2->get_counterclock() == false) {
        local_P1 = local_arc1_center - local_D12 * arc1->get_radius();
        local_P2 = local_arc2_center + local_D12 * arc2->get_radius();
        local_N2 = local_D12;
        dist = local_C1C2.length() - arc1->get_radius() - arc2->get_radius();
        alpha1 = alpha + CH_C_PI;
        alpha2 = alpha;
        paired = true;
    }
    // convex-concave
    if (arc1->get_counterclock() == false && arc2->get_counterclock() == true)
        if (arc1->get_radius() <= arc2->get_radius()) {
            local_P1 = local_arc1_center + local_D12 * arc1->get_radius();
            local_P2 = local_arc2_center + local_D12 * arc2->get_radius();
            local_N2 = -local_D12;
            dist = -local_C1C2.length() - arc1->get_radius() + arc2->get_radius();
            alpha1 = alpha;
            alpha2 = alpha;
            paired = true;
        }
    // concave-convex
    if (arc1->get_counterclock() == true && arc2->get_counterclock() == false)
        if (arc1->get_radius() >= arc2->get_radius()) {
            local_P1 = local_arc1_center - local_D12 * arc1->get_radius();
            local_P2 = local_arc2_center - local_D12 * arc2->get_radius();
            local_N2 = -local_D12;
            dist = -local_C1C2.length() + arc1->get_radius() - arc2->get_radius();
            alpha1 = alpha + CH_C_PI;
            alpha2 = alpha + CH_C_PI;
            paired = true;
        }

    if (!paired)
        return;

    // Discard points out of min-max angles

    // to always positive angles:
    arc1_angle1 = fmod(arc1_angle1, CH_C_2PI);
    if (arc1_angle1 < 0)
        arc1_angle1 += CH_C_2PI;
    arc1_angle2 = fmod(arc1_angle2, CH_C_2PI);
    if (arc1_angle2 < 0)
        arc1_angle2 += CH_C_2PI;
    arc2_angle1 = fmod(arc2_angle1, CH_C_2PI);
    if (arc2_angle1 < 0)
        arc2_angle1 += CH_C_2PI;
    arc2_angle2 = fmod(arc2_angle2, CH_C_2PI);
    if (arc2_angle2 < 0)
        arc2_angle2 += CH_C_2PI;
    alpha1 = fmod(alpha1, CH_C_2PI);
    if (alpha1 < 0)
        alpha1 += CH_C_2PI;
    alpha2 = fmod(alpha2, CH_C_2PI);
    if (alpha2 < 0)
        alpha2 += CH_C_2PI;

    arc1_angle1 = fmod(arc1_angle1, CH_C_2PI);
    arc1_angle2 = fmod(arc1_angle2, CH_C_2PI);
    arc2_angle1 = fmod(arc2_angle1, CH_C_2PI);
    arc2_angle2 = fmod(arc2_angle2, CH_C_2PI);
    alpha1 = fmod(alpha1, CH_C_2PI);
    alpha2 = fmod(alpha2, CH_C_2PI);

    bool inangle1 = false;
    bool inangle2 = false;

    if (arc1->get_counterclock() == true) {
        if (arc1_angle1 < arc1_angle2) {
            if (alpha1 >= arc1_angle1 && alpha1 <= arc1_angle2)
                inangle1 = true;
        } else {
            if (alpha1 >= arc1_angle1 || alpha1 <= arc1_angle2)
                inangle1 = true;
        }
    } else {
        if (arc1_angle1 < arc1_angle2) {
            if (alpha1 >= arc1_angle2 || alpha1 <= arc1_angle1)
                inangle1 = true;
        } else {
            if (alpha1 >= arc1_angle2 && alpha1 <= arc1_angle1)
                inangle1 = true;
        }
    }

    if (arc2->get_counterclock() == true) {
        if (arc2_angle1 < arc2_angle2) {
            if (alpha2 >= arc2_angle1 && alpha2 <= arc2_angle2)
                inangle2 = true;
        } else {
            if (alpha2 >= arc2_angle1 || alpha2 <= arc2_angle2)
                inangle2 = true;
        }
    } else {
        if (arc2_angle1 < arc2_angle2) {
            if (alpha2 >= arc2_angle2 || alpha2 <= arc2_angle1)
                inangle2 = true;
        } else {
            if (alpha2 >= arc2_angle2 && alpha2 <= arc2_angle1)
                inangle2 = true;
        }
    }

    if (!(inangle1 && inangle2))
        return;

    // transform in absolute coords:
    btVector3 pos2 = m44Tarc2 * local_P2;
    btVector3 normal_on_2 = m44Tarc2.getBasis() * local_N2;

    // too far or too interpenetrate? discard.
    if (fabs(dist) > (arc1->getMargin() + arc2->getMargin()))
        return;

    /// report a contact.
    resultOut->addContactPoint(normal_on_2, pos2, (btScalar)dist);

    resultOut->refreshContactPoints();
}

btScalar btArcArcCollisionAlgorithm::calculateTimeOfImpact(btCollisionObject* body0,
                                                           btCollisionObject* body1,
                                                           const btDispatcherInfo& dispatchInfo,
                                                           btManifoldResult* resultOut) {
    // not yet
    return btScalar(1.);
}

void btArcArcCollisionAlgorithm::getAllContactManifolds(btManifoldArray& manifoldArray) {
    if (m_manifoldPtr && m_ownManifold) {
        manifoldArray.push_back(m_manifoldPtr);
    }
}

btCollisionAlgorithm* btArcArcCollisionAlgorithm::CreateFunc::CreateCollisionAlgorithm(
    btCollisionAlgorithmConstructionInfo& ci,
    const btCollisionObjectWrapper* body0Wrap,
    const btCollisionObjectWrapper* body1Wrap) {
    void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btArcArcCollisionAlgorithm));
    if (!m_swapped) {
        return new (mem) btArcArcCollisionAlgorithm(0, ci, body0Wrap, body1Wrap, false);
    } else {
        return new (mem) btArcArcCollisionAlgorithm(0, ci, body0Wrap, body1Wrap, true);
    }
}

// ================================================================================================

btCEtriangleShapeCollisionAlgorithm::btCEtriangleShapeCollisionAlgorithm(btPersistentManifold* mf,
                                                                         const btCollisionAlgorithmConstructionInfo& ci,
                                                                         const btCollisionObjectWrapper* col0,
                                                                         const btCollisionObjectWrapper* col1,
                                                                         bool isSwapped)
    : btActivatingCollisionAlgorithm(ci, col0, col1), m_ownManifold(false), m_manifoldPtr(mf), m_isSwapped(isSwapped) {
    const btCollisionObjectWrapper* triObj1Wrap = m_isSwapped ? col1 : col0;
    const btCollisionObjectWrapper* triObj2Wrap = m_isSwapped ? col0 : col1;

    if (!m_manifoldPtr &&
        m_dispatcher->needsCollision(triObj1Wrap->getCollisionObject(), triObj2Wrap->getCollisionObject())) {
        m_manifoldPtr =
            m_dispatcher->getNewManifold(triObj1Wrap->getCollisionObject(), triObj2Wrap->getCollisionObject());
        m_ownManifold = true;
    }
}

btCEtriangleShapeCollisionAlgorithm::btCEtriangleShapeCollisionAlgorithm(const btCollisionAlgorithmConstructionInfo& ci)
    : btActivatingCollisionAlgorithm(ci) {}

btCEtriangleShapeCollisionAlgorithm ::~btCEtriangleShapeCollisionAlgorithm() {
    if (m_ownManifold) {
        if (m_manifoldPtr)
            m_dispatcher->releaseManifold(m_manifoldPtr);
    }
}

void btCEtriangleShapeCollisionAlgorithm::processCollision(const btCollisionObjectWrapper* body0,
                                                           const btCollisionObjectWrapper* body1,
                                                           const btDispatcherInfo& dispatchInfo,
                                                           btManifoldResult* resultOut) {
    (void)dispatchInfo;
    (void)resultOut;
    if (!m_manifoldPtr)
        return;

    const btCollisionObjectWrapper* triObj1Wrap = m_isSwapped ? body1 : body0;
    const btCollisionObjectWrapper* triObj2Wrap = m_isSwapped ? body0 : body1;

    resultOut->setPersistentManifold(m_manifoldPtr);

    // avoid persistence of contacts in manifold:
    resultOut->getPersistentManifold()->clearManifold();

    const btCEtriangleShape* triA = (btCEtriangleShape*)triObj1Wrap->getCollisionShape();
    const btCEtriangleShape* triB = (btCEtriangleShape*)triObj2Wrap->getCollisionShape();
    ChCollisionModelBullet* triModelA = (ChCollisionModelBullet*)triA->getUserPointer();
    ChCollisionModelBullet* triModelB = (ChCollisionModelBullet*)triB->getUserPointer();

    // brute force discard of connected triangles
    // ***TODO*** faster approach based on collision families that can bypass the
    // check at the broadphase level?
    if (triA->get_p1() == triB->get_p1() || triA->get_p1() == triB->get_p2() || triA->get_p1() == triB->get_p3())
        return;
    if (triA->get_p2() == triB->get_p1() || triA->get_p2() == triB->get_p2() || triA->get_p2() == triB->get_p3())
        return;
    if (triA->get_p3() == triB->get_p1() || triA->get_p3() == triB->get_p2() || triA->get_p3() == triB->get_p3())
        return;

    double max_allowed_dist =
        triModelA->GetEnvelope() + triModelB->GetEnvelope() + triA->sphereswept_r() + triB->sphereswept_r();
    double min_allowed_dist = -(triModelA->GetSafeMargin() + triModelB->GetSafeMargin());

    double offset_A = triA->sphereswept_r();
    double offset_B = triB->sphereswept_r();

    // Trick!! offset also by outward 'envelope' because during ReportContacts()
    // contact points are offset inward by envelope, to cope with GJK method.
    offset_A += triModelA->GetEnvelope();
    offset_B += triModelB->GetEnvelope();

    const btTransform& m44Ta = triObj1Wrap->getCollisionObject()->getWorldTransform();
    const btTransform& m44Tb = triObj2Wrap->getCollisionObject()->getWorldTransform();
    const btMatrix3x3& mbtRa = m44Ta.getBasis();
    const btMatrix3x3& mbtRb = m44Tb.getBasis();
    ChMatrix33<> mRa;
    mRa(0, 0) = mbtRa[0][0];
    mRa(0, 1) = mbtRa[0][1];
    mRa(0, 2) = mbtRa[0][2];
    mRa(1, 0) = mbtRa[1][0];
    mRa(1, 1) = mbtRa[1][1];
    mRa(1, 2) = mbtRa[1][2];
    mRa(2, 0) = mbtRa[2][0];
    mRa(2, 1) = mbtRa[2][1];
    mRa(2, 2) = mbtRa[2][2];
    ChVector<> mOa(m44Ta.getOrigin().x(), m44Ta.getOrigin().y(), m44Ta.getOrigin().z());

    ChMatrix33<> mRb;
    mRb(0, 0) = mbtRb[0][0];
    mRb(0, 1) = mbtRb[0][1];
    mRb(0, 2) = mbtRb[0][2];
    mRb(1, 0) = mbtRb[1][0];
    mRb(1, 1) = mbtRb[1][1];
    mRb(1, 2) = mbtRb[1][2];
    mRb(2, 0) = mbtRb[2][0];
    mRb(2, 1) = mbtRb[2][1];
    mRb(2, 2) = mbtRb[2][2];
    ChVector<> mOb(m44Tb.getOrigin().x(), m44Tb.getOrigin().y(), m44Tb.getOrigin().z());

    // transform points to absolute coords, since models might be roto-translated
    ChVector<> pA1 = mOa + mRa * (*triA->get_p1());
    ChVector<> pA2 = mOa + mRa * (*triA->get_p2());
    ChVector<> pA3 = mOa + mRa * (*triA->get_p3());
    ChVector<> pB1 = mOb + mRb * (*triB->get_p1());
    ChVector<> pB2 = mOb + mRb * (*triB->get_p2());
    ChVector<> pB3 = mOb + mRb * (*triB->get_p3());

    // edges
    ChVector<> eA1 = pA2 - pA1;
    ChVector<> eA2 = pA3 - pA2;
    ChVector<> eA3 = pA1 - pA3;
    ChVector<> eB1 = pB2 - pB1;
    ChVector<> eB2 = pB3 - pB2;
    ChVector<> eB3 = pB1 - pB3;

    // normals
    ChVector<> nA = Vcross(eA1, eA2).GetNormalized();
    ChVector<> nB = Vcross(eB1, eB2).GetNormalized();

    double min_dist = 1e20;
    ChVector<> candid_pA;
    ChVector<> candid_pB;
    double dist = 1e20;
    bool is_into;
    ChVector<> p_projected;
    double mu, mv;
    double candid_mu, candid_mv;

    // Shortcut: if two degenerate 'skinny' triangles with points 2&3 coincident (ex. used to
    // represent chunks of beams) just do an edge-edge test (as capsule-capsule) and return:
    if ((pA2 == pA3) && (pB2 == pB3) && triA->owns_e1() && triB->owns_e1()) {
        ChVector<> cA, cB, D;
        if (utils::LineLineIntersect(pA1, pA2, pB1, pB2, &cA, &cB, &mu, &mv)) {
            D = cB - cA;
            dist = D.Length();
            if (dist < max_allowed_dist && dist > min_allowed_dist && mu > 0 && mu < 1 && mv > 0 && mv < 1) {
                _add_contact(cA, cB, dist, resultOut, offset_A, offset_B);
                resultOut->refreshContactPoints();
                return;
            }
        }
    }

    // vertex-face tests:

    if (triA->owns_v1()) {
        dist = utils::PointTriangleDistance(pA1, pB1, pB2, pB3, mu, mv, is_into, p_projected);
        if (is_into) {
            if (dist < max_allowed_dist && dist > min_allowed_dist) {
                _add_contact(pA1, p_projected, dist, resultOut, offset_A, offset_B);
            }
            if (dist < min_dist) {
                min_dist = dist;
                candid_pA = pA1;
                candid_pB = p_projected;
                candid_mu = mu;
                candid_mv = mv;
            }
        }
    }
    if (triA->owns_v2()) {
        dist = utils::PointTriangleDistance(pA2, pB1, pB2, pB3, mu, mv, is_into, p_projected);
        if (is_into) {
            if (dist < max_allowed_dist && dist > min_allowed_dist) {
                _add_contact(pA2, p_projected, dist, resultOut, offset_A, offset_B);
            }
            if (dist < min_dist) {
                min_dist = dist;
                candid_pA = pA2;
                candid_pB = p_projected;
            }
        }
    }
    if (triA->owns_v3()) {
        dist = utils::PointTriangleDistance(pA3, pB1, pB2, pB3, mu, mv, is_into, p_projected);
        if (is_into) {
            if (dist < max_allowed_dist && dist > min_allowed_dist) {
                _add_contact(pA3, p_projected, dist, resultOut, offset_A, offset_B);
            }
            if (dist < min_dist) {
                min_dist = dist;
                candid_pA = pA3;
                candid_pB = p_projected;
            }
        }
    }

    if (triB->owns_v1()) {
        dist = utils::PointTriangleDistance(pB1, pA1, pA2, pA3, mu, mv, is_into, p_projected);
        if (is_into) {
            if (dist < max_allowed_dist && dist > min_allowed_dist) {
                _add_contact(p_projected, pB1, dist, resultOut, offset_A, offset_B);
            }
            if (dist < min_dist) {
                min_dist = dist;
                candid_pB = pB1;
                candid_pA = p_projected;
            }
        }
    }
    if (triB->owns_v2()) {
        dist = utils::PointTriangleDistance(pB2, pA1, pA2, pA3, mu, mv, is_into, p_projected);
        if (is_into) {
            if (dist < max_allowed_dist && dist > min_allowed_dist) {
                _add_contact(p_projected, pB2, dist, resultOut, offset_A, offset_B);
            }
            if (dist < min_dist) {
                min_dist = dist;
                candid_pB = pB2;
                candid_pA = p_projected;
            }
        }
    }
    if (triB->owns_v3()) {
        dist = utils::PointTriangleDistance(pB3, pA1, pA2, pA3, mu, mv, is_into, p_projected);
        if (is_into) {
            if (dist < max_allowed_dist && dist > min_allowed_dist) {
                _add_contact(p_projected, pB3, dist, resultOut, offset_A, offset_B);
            }
            if (dist < min_dist) {
                min_dist = dist;
                candid_pB = pB3;
                candid_pA = p_projected;
            }
        }
    }
    double beta_A1, beta_A2, beta_A3, beta_B1, beta_B2, beta_B3;  // defaults for free edge
    ChVector<> tA1, tA2, tA3, tB1, tB2, tB3;
    ChVector<> lA1, lA2, lA3, lB1, lB2, lB3;

    //  edges-edges tests

    if (triA->owns_e1()) {
        tA1 = Vcross(eA1, nA).GetNormalized();
        if (triA->get_e1())
            lA1 = (mOa + mRa * (*triA->get_e1())) - pA1;
        else
            lA1 = -tA1;
        beta_A1 = atan2(Vdot(lA1, tA1), Vdot(lA1, nA));
        if (beta_A1 < 0)
            beta_A1 += CH_C_2PI;
    }
    if (triA->owns_e2()) {
        tA2 = Vcross(eA2, nA).GetNormalized();
        if (triA->get_e2())
            lA2 = (mOa + mRa * (*triA->get_e2())) - pA2;
        else
            lA2 = -tA2;
        beta_A2 = atan2(Vdot(lA2, tA2), Vdot(lA2, nA));
        if (beta_A2 < 0)
            beta_A2 += CH_C_2PI;
    }
    if (triA->owns_e3()) {
        tA3 = Vcross(eA3, nA).GetNormalized();
        if (triA->get_e3())
            lA3 = (mOa + mRa * (*triA->get_e3())) - pA3;
        else
            lA3 = -tA3;
        beta_A3 = atan2(Vdot(lA3, tA3), Vdot(lA3, nA));
        if (beta_A3 < 0)
            beta_A3 += CH_C_2PI;
    }
    if (triB->owns_e1()) {
        tB1 = Vcross(eB1, nB).GetNormalized();
        if (triB->get_e1())
            lB1 = (mOb + mRb * (*triB->get_e1())) - pB1;
        else
            lB1 = -tB1;
        beta_B1 = atan2(Vdot(lB1, tB1), Vdot(lB1, nB));
        if (beta_B1 < 0)
            beta_B1 += CH_C_2PI;
    }
    if (triB->owns_e2()) {
        tB2 = Vcross(eB2, nB).GetNormalized();
        if (triB->get_e2())
            lB2 = (mOb + mRb * (*triB->get_e2())) - pB2;
        else
            lB2 = -tB2;
        beta_B2 = atan2(Vdot(lB2, tB2), Vdot(lB2, nB));
        if (beta_B2 < 0)
            beta_B2 += CH_C_2PI;
    }
    if (triB->owns_e3()) {
        tB3 = Vcross(eB3, nB).GetNormalized();
        if (triB->get_e3())
            lB3 = (mOb + mRb * (*triB->get_e3())) - pB3;
        else
            lB3 = -tB3;
        beta_B3 = atan2(Vdot(lB3, tB3), Vdot(lB3, nB));
        if (beta_B3 < 0)
            beta_B3 += CH_C_2PI;
    }

    ChVector<> cA, cB, D;

    double edge_tol = 1e-3;
    //  + edge_tol to discard flat edges with some tolerance:
    double beta_convex_limit = CH_C_PI_2 + edge_tol;
    //  +/- edge_tol to inflate arc of acceptance of edge vs edge, to cope with singular cases (ex. flat cube vs
    //  flat cube):
    double alpha_lo_limit = -edge_tol;
    double CH_C_PI_mtol = CH_C_PI - edge_tol;
    double CH_C_PI_2_ptol = CH_C_PI_2 + edge_tol;

    // edge A1 vs edge B1
    if (triA->owns_e1() && triB->owns_e1())
        if (beta_A1 > beta_convex_limit && beta_B1 > beta_convex_limit) {
            if (utils::LineLineIntersect(pA1, pA2, pB1, pB2, &cA, &cB, &mu, &mv)) {
                D = cB - cA;
                dist = D.Length();
                if (dist < max_allowed_dist && dist > min_allowed_dist && mu > 0 && mu < 1 && mv > 0 && mv < 1) {
                    double alpha_A = atan2(Vdot(D, tA1), Vdot(D, nA));
                    double alpha_B = atan2(Vdot(-D, tB1), Vdot(-D, nB));
                    if (alpha_A < alpha_lo_limit)
                        alpha_A += CH_C_2PI;
                    if (alpha_B < alpha_lo_limit)
                        alpha_B += CH_C_2PI;
                    if ((alpha_A < beta_A1 - CH_C_PI_2_ptol) && (alpha_B < beta_B1 - CH_C_PI_2_ptol))
                        _add_contact(cA, cB, dist, resultOut, offset_A, offset_B);
                    else if (alpha_A > CH_C_PI_mtol && (alpha_A < beta_A1 + CH_C_PI_2) && alpha_B > CH_C_PI_mtol &&
                             (alpha_B < beta_B1 + CH_C_PI_2_ptol))
                        _add_contact(cA, cB, -dist, resultOut, offset_A, offset_B);
                }
            }
        }
    // edge A1 vs edge B2
    if (triA->owns_e1() && triB->owns_e2())
        if (beta_A1 > beta_convex_limit && beta_B2 > beta_convex_limit) {
            if (utils::LineLineIntersect(pA1, pA2, pB2, pB3, &cA, &cB, &mu, &mv)) {
                D = cB - cA;
                dist = D.Length();
                if (dist < max_allowed_dist && dist > min_allowed_dist && mu > 0 && mu < 1 && mv > 0 && mv < 1) {
                    D = cB - cA;
                    double alpha_A = atan2(Vdot(D, tA1), Vdot(D, nA));
                    double alpha_B = atan2(Vdot(-D, tB2), Vdot(-D, nB));
                    if (alpha_A < alpha_lo_limit)
                        alpha_A += CH_C_2PI;
                    if (alpha_B < alpha_lo_limit)
                        alpha_B += CH_C_2PI;
                    if ((alpha_A < beta_A1 - CH_C_PI_2_ptol) && (alpha_B < beta_B2 - CH_C_PI_2_ptol))
                        _add_contact(cA, cB, dist, resultOut, offset_A, offset_B);
                    else if (alpha_A > CH_C_PI_mtol && (alpha_A < beta_A1 + CH_C_PI_2) && alpha_B > CH_C_PI_mtol &&
                             (alpha_B < beta_B2 + CH_C_PI_2_ptol))
                        _add_contact(cA, cB, -dist, resultOut, offset_A, offset_B);
                }
            }
        }
    // edge A1 vs edge B3
    if (triA->owns_e1() && triB->owns_e3())
        if (beta_A1 > beta_convex_limit && beta_B3 > beta_convex_limit) {
            if (utils::LineLineIntersect(pA1, pA2, pB3, pB1, &cA, &cB, &mu, &mv)) {
                D = cB - cA;
                dist = D.Length();
                if (dist < max_allowed_dist && dist > min_allowed_dist && mu > 0 && mu < 1 && mv > 0 && mv < 1) {
                    D = cB - cA;
                    double alpha_A = atan2(Vdot(D, tA1), Vdot(D, nA));
                    double alpha_B = atan2(Vdot(-D, tB3), Vdot(-D, nB));
                    if (alpha_A < alpha_lo_limit)
                        alpha_A += CH_C_2PI;
                    if (alpha_B < alpha_lo_limit)
                        alpha_B += CH_C_2PI;
                    if ((alpha_A < beta_A1 - CH_C_PI_2_ptol) && (alpha_B < beta_B3 - CH_C_PI_2_ptol))
                        _add_contact(cA, cB, dist, resultOut, offset_A, offset_B);
                    else if (alpha_A > CH_C_PI_mtol && (alpha_A < beta_A1 + CH_C_PI_2) && alpha_B > CH_C_PI_mtol &&
                             (alpha_B < beta_B3 + CH_C_PI_2_ptol))
                        _add_contact(cA, cB, -dist, resultOut, offset_A, offset_B);
                }
            }
        }
    // edge A2 vs edge B1
    if (triA->owns_e2() && triB->owns_e1())
        if (beta_A2 > beta_convex_limit && beta_B1 > beta_convex_limit) {
            if (utils::LineLineIntersect(pA2, pA3, pB1, pB2, &cA, &cB, &mu, &mv)) {
                D = cB - cA;
                dist = D.Length();
                if (dist < max_allowed_dist && dist > min_allowed_dist && mu > 0 && mu < 1 && mv > 0 && mv < 1) {
                    D = cB - cA;
                    double alpha_A = atan2(Vdot(D, tA2), Vdot(D, nA));
                    double alpha_B = atan2(Vdot(-D, tB1), Vdot(-D, nB));
                    if (alpha_A < alpha_lo_limit)
                        alpha_A += CH_C_2PI;
                    if (alpha_B < alpha_lo_limit)
                        alpha_B += CH_C_2PI;
                    if ((alpha_A < beta_A2 - CH_C_PI_2_ptol) && (alpha_B < beta_B1 - CH_C_PI_2_ptol))
                        _add_contact(cA, cB, dist, resultOut, offset_A, offset_B);
                    else if (alpha_A > CH_C_PI_mtol && (alpha_A < beta_A2 + CH_C_PI_2) && alpha_B > CH_C_PI_mtol &&
                             (alpha_B < beta_B1 + CH_C_PI_2_ptol))
                        _add_contact(cA, cB, -dist, resultOut, offset_A, offset_B);
                }
            }
        }
    // edge A2 vs edge B2
    if (triA->owns_e2() && triB->owns_e2())
        if (beta_A2 > beta_convex_limit && beta_B2 > beta_convex_limit) {
            if (utils::LineLineIntersect(pA2, pA3, pB2, pB3, &cA, &cB, &mu, &mv)) {
                D = cB - cA;
                dist = D.Length();
                if (dist < max_allowed_dist && dist > min_allowed_dist && mu > 0 && mu < 1 && mv > 0 && mv < 1) {
                    D = cB - cA;
                    double alpha_A = atan2(Vdot(D, tA2), Vdot(D, nA));
                    double alpha_B = atan2(Vdot(-D, tB2), Vdot(-D, nB));
                    if (alpha_A < alpha_lo_limit)
                        alpha_A += CH_C_2PI;
                    if (alpha_B < alpha_lo_limit)
                        alpha_B += CH_C_2PI;
                    if ((alpha_A < beta_A2 - CH_C_PI_2_ptol) && (alpha_B < beta_B2 - CH_C_PI_2_ptol))
                        _add_contact(cA, cB, dist, resultOut, offset_A, offset_B);
                    else if (alpha_A > CH_C_PI_mtol && (alpha_A < beta_A2 + CH_C_PI_2) && alpha_B > CH_C_PI_mtol &&
                             (alpha_B < beta_B2 + CH_C_PI_2_ptol))
                        _add_contact(cA, cB, -dist, resultOut, offset_A, offset_B);
                }
            }
        }
    // edge A2 vs edge B3
    if (triA->owns_e2() && triB->owns_e3())
        if (beta_A2 > beta_convex_limit && beta_B3 > beta_convex_limit) {
            if (utils::LineLineIntersect(pA2, pA3, pB3, pB1, &cA, &cB, &mu, &mv)) {
                D = cB - cA;
                dist = D.Length();
                if (dist < max_allowed_dist && dist > min_allowed_dist && mu > 0 && mu < 1 && mv > 0 && mv < 1) {
                    D = cB - cA;
                    double alpha_A = atan2(Vdot(D, tA2), Vdot(D, nA));
                    double alpha_B = atan2(Vdot(-D, tB3), Vdot(-D, nB));
                    if (alpha_A < alpha_lo_limit)
                        alpha_A += CH_C_2PI;
                    if (alpha_B < alpha_lo_limit)
                        alpha_B += CH_C_2PI;
                    if ((alpha_A < beta_A2 - CH_C_PI_2_ptol) && (alpha_B < beta_B3 - CH_C_PI_2_ptol))
                        _add_contact(cA, cB, dist, resultOut, offset_A, offset_B);
                    else if (alpha_A > CH_C_PI_mtol && (alpha_A < beta_A2 + CH_C_PI_2) && alpha_B > CH_C_PI_mtol &&
                             (alpha_B < beta_B3 + CH_C_PI_2_ptol))
                        _add_contact(cA, cB, -dist, resultOut, offset_A, offset_B);
                }
            }
        }
    // edge A3 vs edge B1
    if (triA->owns_e3() && triB->owns_e1())
        if (beta_A3 > beta_convex_limit && beta_B1 > beta_convex_limit) {
            if (utils::LineLineIntersect(pA3, pA1, pB1, pB2, &cA, &cB, &mu, &mv)) {
                D = cB - cA;
                dist = D.Length();
                if (dist < max_allowed_dist && dist > min_allowed_dist && mu > 0 && mu < 1 && mv > 0 && mv < 1) {
                    D = cB - cA;
                    double alpha_A = atan2(Vdot(D, tA3), Vdot(D, nA));
                    double alpha_B = atan2(Vdot(-D, tB1), Vdot(-D, nB));
                    if (alpha_A < alpha_lo_limit)
                        alpha_A += CH_C_2PI;
                    if (alpha_B < alpha_lo_limit)
                        alpha_B += CH_C_2PI;
                    if ((alpha_A < beta_A3 - CH_C_PI_2_ptol) && (alpha_B < beta_B1 - CH_C_PI_2_ptol))
                        _add_contact(cA, cB, dist, resultOut, offset_A, offset_B);
                    else if (alpha_A > CH_C_PI_mtol && (alpha_A < beta_A3 + CH_C_PI_2) && alpha_B > CH_C_PI_mtol &&
                             (alpha_B < beta_B1 + CH_C_PI_2_ptol))
                        _add_contact(cA, cB, -dist, resultOut, offset_A, offset_B);
                }
            }
        }
    // edge A3 vs edge B2
    if (triA->owns_e3() && triB->owns_e2())
        if (beta_A3 > beta_convex_limit && beta_B2 > beta_convex_limit) {
            if (utils::LineLineIntersect(pA3, pA1, pB2, pB3, &cA, &cB, &mu, &mv)) {
                D = cB - cA;
                dist = D.Length();
                if (dist < max_allowed_dist && dist > min_allowed_dist && mu > 0 && mu < 1 && mv > 0 && mv < 1) {
                    D = cB - cA;
                    double alpha_A = atan2(Vdot(D, tA3), Vdot(D, nA));
                    double alpha_B = atan2(Vdot(-D, tB2), Vdot(-D, nB));
                    if (alpha_A < alpha_lo_limit)
                        alpha_A += CH_C_2PI;
                    if (alpha_B < alpha_lo_limit)
                        alpha_B += CH_C_2PI;
                    if ((alpha_A < beta_A3 - CH_C_PI_2_ptol) && (alpha_B < beta_B2 - CH_C_PI_2_ptol))
                        _add_contact(cA, cB, dist, resultOut, offset_A, offset_B);
                    else if (alpha_A > CH_C_PI_mtol && (alpha_A < beta_A3 + CH_C_PI_2) && alpha_B > CH_C_PI_mtol &&
                             (alpha_B < beta_B2 + CH_C_PI_2_ptol))
                        _add_contact(cA, cB, -dist, resultOut, offset_A, offset_B);
                }
            }
        }
    // edge A3 vs edge B3
    if (triA->owns_e3() && triB->owns_e3())
        if (beta_A3 > beta_convex_limit && beta_B3 > beta_convex_limit) {
            if (utils::LineLineIntersect(pA3, pA1, pB3, pB1, &cA, &cB, &mu, &mv)) {
                D = cB - cA;
                dist = D.Length();
                if (dist < max_allowed_dist && dist > min_allowed_dist && mu > 0 && mu < 1 && mv > 0 && mv < 1) {
                    D = cB - cA;
                    double alpha_A = atan2(Vdot(D, tA3), Vdot(D, nA));
                    double alpha_B = atan2(Vdot(-D, tB3), Vdot(-D, nB));
                    if (alpha_A < alpha_lo_limit)
                        alpha_A += CH_C_2PI;
                    if (alpha_B < alpha_lo_limit)
                        alpha_B += CH_C_2PI;
                    if ((alpha_A < beta_A3 - CH_C_PI_2_ptol) && (alpha_B < beta_B3 - CH_C_PI_2_ptol))
                        _add_contact(cA, cB, dist, resultOut, offset_A, offset_B);
                    else if (alpha_A > CH_C_PI_mtol && (alpha_A < beta_A3 + CH_C_PI_2) && alpha_B > CH_C_PI_mtol &&
                             (alpha_B < beta_B3 + CH_C_PI_2_ptol))
                        _add_contact(cA, cB, -dist, resultOut, offset_A, offset_B);
                }
            }
        }

    resultOut->refreshContactPoints();
}

btScalar btCEtriangleShapeCollisionAlgorithm::calculateTimeOfImpact(btCollisionObject* body0,
                                                                    btCollisionObject* body1,
                                                                    const btDispatcherInfo& dispatchInfo,
                                                                    btManifoldResult* resultOut) {
    // not yet
    return btScalar(1.);
}

void btCEtriangleShapeCollisionAlgorithm::getAllContactManifolds(btManifoldArray& manifoldArray) {
    if (m_manifoldPtr && m_ownManifold) {
        manifoldArray.push_back(m_manifoldPtr);
    }
}

void btCEtriangleShapeCollisionAlgorithm::_add_contact(const ChVector<>& candid_pA,
                                                       const ChVector<>& candid_pB,
                                                       const double dist,
                                                       btManifoldResult* resultOut,
                                                       const double offsetA,
                                                       const double offsetB) {
    // convert to Bullet vectors. Note: in absolute csys.
    btVector3 absA((btScalar)candid_pA.x(), (btScalar)candid_pA.y(), (btScalar)candid_pA.z());
    btVector3 absB((btScalar)candid_pB.x(), (btScalar)candid_pB.y(), (btScalar)candid_pB.z());
    ChVector<> dabsN_onB((candid_pA - candid_pB).GetNormalized());
    btVector3 absN_onB((btScalar)dabsN_onB.x(), (btScalar)dabsN_onB.y(), (btScalar)dabsN_onB.z());
    if (dist < 0)
        absN_onB = -absN_onB;  // flip norm to be coherent with dist sign
    resultOut->addContactPoint(absN_onB, absB + absN_onB * (btScalar)offsetB, (btScalar)(dist - (offsetA + offsetB)));
}

btCollisionAlgorithm* btCEtriangleShapeCollisionAlgorithm::CreateFunc::CreateCollisionAlgorithm(
    btCollisionAlgorithmConstructionInfo& ci,
    const btCollisionObjectWrapper* body0Wrap,
    const btCollisionObjectWrapper* body1Wrap) {
    void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btCEtriangleShapeCollisionAlgorithm));
    if (!m_swapped) {
        return new (mem) btCEtriangleShapeCollisionAlgorithm(0, ci, body0Wrap, body1Wrap, false);
    } else {
        return new (mem) btCEtriangleShapeCollisionAlgorithm(0, ci, body0Wrap, body1Wrap, true);
    }
}

}  // namespace collision
}  // namespace chrono
