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

/// Enumeration of collision model types.
enum class CollisionType {
    NONE,        ///< no contact shapes
    PRIMITIVES,  ///< contact model composed of primitives
    HULLS,       ///< contact model composed of convex hulls
    MESH         ///< contact model composed of trimeshes
};

/// @addtogroup vehicle_models
/// @{

/// Namespace for the HMMWV vehicle model
namespace hmmwv {}

/// Namespace for the FED-alpha vehicle model
namespace feda {}

/// Namespace for the passenger vehicle model
namespace sedan {}

/// Namespace for the UAZ vehicle model
namespace uaz {}

/// Namespace for the bus vehicle model
namespace citybus {}

/// Namespace for the BMW 330i vehicle model
namespace bmw {}

/// Namespace for the generic wheeled vehicle model
namespace generic {}

/// Namespace for the M113 track vehicle model
namespace m113 {}

/// Namespace for the MAN truck vehicle models
namespace man {}

/// Namespace for the Marder track vehicle model
namespace marder {}

/// Namespace for the MROLE multi-purpose wheeled vehicle model
namespace mrole {}

/// Namespace for the artcar vehicle model
namespace artcar {}

/// Namespace for the Gator vehicle model
namespace gator {}

/// Namespace for the FMTV vehicle models
namespace fmtv {}

/// Namespace for the ARTcar vehicle model
namespace artcar {}

/// Namespace for the Kraz truck vehicle model
namespace kraz {}

/// Namespace for the Mercedes G500 vehicle model
namespace gclass {}

/// Namespace for the Duro vehicle model
namespace duro {}

/// Namespace for the Unimog 401 vehicle model
namespace unimog {}

/// Namespace for the Jeep Cherokee vehicle model
namespace jeep {}

/// @} vehicle_models

}  // end namespace vehicle
}  // end namespace chrono

#endif
