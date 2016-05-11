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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Base class for a vehicle wheel.
// A wheel subsystem does not own a body. Instead, when attached to a suspension
// subsystem, the wheel's mass properties are used to update those of the
// spindle body owned by the suspension.
// A concrete wheel subsystem can optionally carry its own visualization assets
// (which are associated with the suspension's spindle body).
//
// =============================================================================

#ifndef CH_WHEEL_H
#define CH_WHEEL_H

#include <vector>

#include "chrono/physics/ChBody.h"

#include "chrono_vehicle/ChApiVehicle.h"

/**
    @addtogroup vehicle_wheeled
    @{
        @defgroup vehicle_wheeled_wheel Wheel subsystem
    @}
*/

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_wheel
/// @{

/// Base class for a vehicle wheel subsystem.
/// A wheel subsystem does not own a body. Instead, when attached to a suspension
/// subsystem, the wheel's mass properties are used to update those of the
/// spindle body owned by the suspension.
/// A concrete wheel subsystem can optionally carry its own visualization assets
/// (which are associated with the suspension's spindle body).
class CH_VEHICLE_API ChWheel {
  public:
    ChWheel() {}
    virtual ~ChWheel() {}

    /// Get the wheel mass.
    virtual double GetMass() const = 0;

    /// Get the wheel moments of inertia.
    virtual ChVector<> GetInertia() const = 0;

    /// Get the wheel radius
    virtual double GetRadius() const { return 0; }

    /// Get the wheel width
    virtual double GetWidth() const { return 0; }

    /// Initialize this wheel subsystem.
    /// The wheel mass and inertia are used to increment those of the spindle.
    virtual void Initialize(std::shared_ptr<ChBody> spindle  ///< handle to the associated spindle body
                            );
};

/// Vector of handles to wheel subsystems.
typedef std::vector<std::shared_ptr<ChWheel> > ChWheelList;

/// @} vehicle_wheeled_wheel

}  // end namespace vehicle
}  // end namespace chrono

#endif
