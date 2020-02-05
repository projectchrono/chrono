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

#include "chrono/collision/ChCCollisionModel.h"
#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChVector.h"

namespace chrono {
namespace collision {

/// @addtogroup chrono_collision
/// @{

///   Class for passing basic data about contact pairs
class ChApi ChCollisionInfo {
  public:
    ChCollisionModel* modelA;  ///< model A
    ChCollisionModel* modelB;  ///< model B
    ChVector<> vpA;            ///< coll.point on A, in abs coords
    ChVector<> vpB;            ///< coll.point on B, in abs coords
    ChVector<> vN;             ///< coll.normal, respect to A, in abs coords
    double distance;           ///< distance (negative for penetration)
    double eff_radius;         ///< effective radius of curvature at contact (SMC only)
    float* reaction_cache;     ///< pointer to some persistent user cache of reactions

    /// Basic default constructor.
    ChCollisionInfo();

    /// Copy from other.
    ChCollisionInfo(const ChCollisionInfo& other, const bool swap = false);

    /// Swap models, that is modelA becomes modelB and viceversa.
    void SwapModels();

    /// Set the default effective radius of curvature (for SMC contact).
    /// <pre>
    /// A collision system should evaluate this value for each collision using
    ///     1/r_eff = 1/rA + 1/rB
    /// where rA and rB are the radii of curvature of the two surfaces at the contact point.
    /// </pre>
    /// If a collision system does not set this quantity, all collisions use this default value.
    static void SetDefaultEffectiveCurvatureRadius(double eff_radius);

    /// Return the current value of the default effective radius of curvature.
    static double GetDefaultEffectiveCurvatureRadius();
};

/// @} chrono_collision

}  // end namespace collision
}  // end namespace chrono

#endif
