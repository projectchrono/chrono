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

/// Enumeration of collision shape types.
enum class ChassisCollisionType {
    NONE,        ///< no contact shapes
    PRIMITIVES,  ///< contact model composed of primitives
    MESH         ///< contact model composed of convex hulls
};

/// @addtogroup vehicle_models
/// @{

/// Namespace for the HMMWV vehicle model
namespace hmmwv {}

/// Namespace for the passenger vehicle model
namespace sedan {}

/// Namespace for the UAZ vehicle model
namespace uaz {}

/// Namespace for the bus vehicle model
namespace citybus {}

/// Namespace for the generic wheeled vehicle model
namespace generic {}

/// Namespace for the M113 segmented track vehicle
namespace m113 {}

/// @} vehicle_models

}  // end namespace vehicle
}  // end namespace chrono

#endif
