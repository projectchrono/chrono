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

#ifndef CHCCOLLISIONINFO_H
#define CHCCOLLISIONINFO_H

#include "chrono/core/ChVector.h"
#include "chrono/collision/ChCCollisionModel.h"

namespace chrono {
namespace collision {

/// @addtogroup chrono_collision
/// @{

///   Class for passing basic data about contact pairs
class ChCollisionInfo {
  public:
    ChCollisionModel* modelA;  ///<  model A
    ChCollisionModel* modelB;  ///<  model B
    ChVector<> vpA;            ///<  coll.point on A, in abs coords
    ChVector<> vpB;            ///<  coll.point on B, in abs coords
    ChVector<> vN;             ///<  coll.normal, respect to A, in abs coords
    double distance;           ///<  distance (negative for penetration)
    float* reaction_cache;     ///<  pointer to some persistent user cache of reactions

    /// Basic default constructor
    ChCollisionInfo() {
        modelA = modelB = 0;
        vpA = vpB = VNULL;
        vN.Set(1, 0, 0);
        distance = 0.;
        reaction_cache = 0;
    }

    /// Copy from other 
    ChCollisionInfo(const ChCollisionInfo& other, const bool swap=false) {
        if (!swap) {
            modelA = other.modelA;
            modelB = other.modelB;
            vpA = other.vpA;
            vpB = other.vpB;
            vN  = other.vN;
        } 
        else {
            // copy by swapping models !
            modelA = other.modelB;
            modelB = other.modelA;
            vpA = other.vpB;
            vpB = other.vpA;
            vN  = -other.vN;
        }
        distance = other.distance;
        reaction_cache = other.reaction_cache;
    }

    /// Swap models, that is modelA becomes modelB and viceversa;
    /// normal and so on are updates as well.
    void SwapModels() {
        ChCollisionModel* modeltemp;
        modeltemp = modelA;
        modelA = modelB;
        modelB = modeltemp;
        ChVector<> vtemp;
        vtemp = vpA;
        vpA = vpB;
        vpB = vtemp;
        vN = Vmul(vN, -1.0);
    }
};

/// @} chrono_collision

}  // end namespace collision
}  // end namespace chrono

#endif
