//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHC_COLLISIONPAIR_H
#define CHC_COLLISIONPAIR_H

#include "chrono/geometry/ChGeometry.h"

namespace chrono {
namespace collision {

///
/// Class for storing information about a collision point.
///

class ChCollisionPair {
  public:
    /// Basic constructor
    ChCollisionPair() {
        geo1 = geo2 = 0;
        p1 = p2 = VNULL;
        normal.Set(1, 0, 0);
        norm_dist = 0.;
        just_intersection = false;
        reactions_cache = 0;
    }

    /// Constructor for case of contact point correctly estimated
    ChCollisionPair(geometry::ChGeometry* mgeo1,
                    geometry::ChGeometry* mgeo2,
                    const ChVector<>& mp1,
                    const ChVector<>& mp2,
                    const ChVector<float>& mnormal,
                    float* mreaction_cache = 0)

    {
        this->Set(mgeo1, mgeo2, mp1, mp2, mnormal, mreaction_cache);
    }

    /// Constructor for case of just intersection
    ChCollisionPair(geometry::ChGeometry* mgeo1, geometry::ChGeometry* mgeo2) {
        geo1 = mgeo1;
        geo2 = mgeo2;
        p1 = p2 = VNULL;
        normal.Set(1, 0, 0);
        norm_dist = 0.;
        just_intersection = true;
    }

    /// Set all data at once (better: use the custom constructor)
    void Set(geometry::ChGeometry* mgeo1,
             geometry::ChGeometry* mgeo2,
             const ChVector<>& mp1,
             const ChVector<>& mp2,
             const ChVector<float>& mnormal,
             float* mreaction_cache = 0) {
        geo1 = mgeo1;
        geo2 = mgeo2;
        p1 = mp1;
        p2 = mp2;
        normal = mnormal;
        just_intersection = false;
        reactions_cache = mreaction_cache;

        norm_dist = Vdot(mnormal, mp2 - mp1);
    }

    /// Swap geometries, that is
    /// geo1 becomes geo2 and viceversa; normal and so on are updates as well.
    void SwapGeometries() {
        geometry::ChGeometry* gtemp;
        gtemp = geo1;
        geo1 = geo2;
        geo2 = gtemp;
        ChVector<> vtemp;
        vtemp = p1;
        p1 = p2;
        p2 = vtemp;
        normal = Vmul(normal, -1.0);
    }

    /// Fetches normal and U,V impulsive reactions, as previously stored in a
    /// persistent contact manifold maintained by the collision engine. If no cache, set as 0,0,0
    void CacheFetchSpeedSolutionFromManifold(float& mN, float& mU, float& mV) {
        if (reactions_cache) {
            mN = reactions_cache[0];
            mU = reactions_cache[1];
            mV = reactions_cache[2];
        }
    }
    /// Fetches normal and U,V 'positional' reactions, as previously stored in a
    /// persistent contact manifold maintained by the collision engine. If no cache, set as 0,0,0
    void CacheFetchPositionSolutionFromManifold(float& mN, float& mU, float& mV) {
        if (reactions_cache) {
            mN = reactions_cache[3];
            mU = reactions_cache[4];
            mV = reactions_cache[5];
        }
    }
    /// Stores normal and U,V reactions into a persistent contact manifold
    /// maintained by the collision engine (if any)
    void CacheStoreSpeedSolutionIntoManifold(const float mN, const float mU, const float mV) {
        if (reactions_cache) {
            reactions_cache[0] = mN;
            reactions_cache[1] = mU;
            reactions_cache[2] = mV;
        }
    }
    /// Stores normal and U,V 'positional' reactions into a persistent contact manifold
    /// maintained by the collision engine (if any)
    void CacheStorePositionSolutionIntoManifold(const float mN, const float mU, const float mV) {
        if (reactions_cache) {
            reactions_cache[3] = mN;
            reactions_cache[4] = mU;
            reactions_cache[5] = mV;
        }
    }

    // DATA

    geometry::ChGeometry* geo1;  ///< pointer to 1st geometry which generated this collision pair
    geometry::ChGeometry* geo2;  ///< pointer to 2nd geometry which generated this collision pair

    ChVector<> p1;  ///< max penetration point on geo1, after refining, in abs space
    ChVector<> p2;  ///< max penetration point on geo2, after refining, in abs space

    ChVector<float> normal;  ///< normal, on surface of master reference (geo1)
    double norm_dist;        ///< penetration distance (negative if going inside) after refining

    bool just_intersection;  ///< if true, only reports that two geometries are intersection, but no info is reliable
    /// about normal, p1 or p2.

    float* reactions_cache;  ///< points to an array[3] of N,U,V reactions which might be stored in a persistent contact
                             /// manifold in the collision engine
};

}  // end namespace collision
}  // end namespace chrono

#endif
