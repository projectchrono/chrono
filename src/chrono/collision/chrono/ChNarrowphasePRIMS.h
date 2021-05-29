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
// Header file for ChCNarrowphaseR.
// This narrow-phase collision detection relies on specialized functions for
// each pair of collision shapes. Only a subset of collision shapes and of
// pair-wise interactions are currently supported:
//
//          |  sphere   box   rbox   capsule   cylinder   rcyl   trimesh
// ---------+----------------------------------------------------------
// sphere   |    Y       Y      Y       Y         Y        Y        Y
// box      |            Y      N       Y         N        N        N
// rbox     |                   N       N         N        N        N
// capsule  |                           Y         N        N        N
// cylinder |                                     N        N        N
// rcyl     |                                              N        N
// trimesh  |                                                       N
//
// Note that some pairs may return more than one contact (e.g., box-box).
//
// =============================================================================

#pragma once

#include "chrono/collision/chrono/ChConvexShape.h"

namespace chrono {
namespace collision {

/// @addtogroup collision_mc
/// @{

/// Dispatcher for analytic collision functions between a pair of candidate shapes.
/// Each candidate pair of shapes can result in 0, 1, or more contacts.  For each actual contact, we calculate various
/// geometrical quantities and load them in the output arguments (starting from the given addresses)
/// @remark
///   - ct_pt1:      contact point on first shape (in global frame)
/// @remark
///   - ct_pt2:      contact point on second shape (in global frame)
/// @remark
///   - ct_depth:    penetration distance (negative if overlap exists)
/// @remark
///   - ct_norm:     contact normal, from ct_pt2 to ct_pt1 (in global frame)
/// @remark
///   - ct_eff_rad:  effective contact radius
///
/// Note that we also report collisions for which the distance between the two shapes is at most 'separation' (typically
/// twice the collision envelope). In these cases, the corresponding ct_depth is a positive value. This function returns
/// true if it was able to determine the collision state for the given pair of shapes and false if the shape types are
/// not supported.
///
/// Currently supported pair-wise interactions:
/// <pre>
///          |  sphere   box   rbox   capsule   cylinder   rcyl   trimesh
/// ---------+----------------------------------------------------------
/// sphere   |    Y       Y      Y       Y         Y        Y        Y
/// box      |            Y      N       Y         N        N        N
/// rbox     |                   N       N         N        N        N
/// capsule  |                           Y         N        N        N
/// cylinder |                                     N        N        N
/// rcyl     |                                              N        N
/// trimesh  |                                                       N
/// </pre>
ChApi bool PRIMSCollision(const ConvexBase* shapeA,  ///< first candidate shape
                          const ConvexBase* shapeB,  ///< second candidate shape
                          real separation,           ///< maximum separation
                          real3* ct_norm,            ///< [output] contact normal (per contact pair)
                          real3* ct_pt1,             ///< [output] point on shape1 (per contact pair)
                          real3* ct_pt2,             ///< [output] point on shape2 (per contact pair)
                          real* ct_depth,            ///< [output] penetration depth (per contact pair)
                          real* ct_eff_rad,          ///< [output] effective contact radius (per contact pair)
                          int& nC                    ///< [output] number of contacts found
);

/// Set the fictitious radius of curvature used for collision with a corner or an edge.
ChApi void SetDefaultEdgeRadius(real radius);

/// Return the fictitious radius of curvature used for collisions with a corner or an edge.
ChApi real GetDefaultEdgeRadius();

/// @} collision_mc

}  // end namespace collision
}  // end namespace chrono
