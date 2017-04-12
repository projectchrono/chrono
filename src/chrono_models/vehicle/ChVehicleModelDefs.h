// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Definitions for vehicle models.
//
// =============================================================================

#ifndef CH_VEHICLE_MODEL_DEFS_H
#define CH_VEHICLE_MODEL_DEFS_H


namespace chrono {
namespace vehicle {

enum class ChassisCollisionType {
    NONE,        ///< no contact shapes
    PRIMITIVES,  ///< contact model composed of primitives
    MESH         ///< contact model composed of convex hulls
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
