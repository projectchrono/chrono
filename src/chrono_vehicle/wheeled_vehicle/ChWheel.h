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
#include "chrono/assets/ChCylinderShape.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPart.h"

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
class CH_VEHICLE_API ChWheel : public ChPart {
  public:
    ChWheel(const std::string& name  ///< [in] name of the subsystem
            );

    virtual ~ChWheel() {}

    /// Get the wheel mass.
    virtual double GetMass() const = 0;

    /// Get the wheel moments of inertia.
    virtual ChVector<> GetInertia() const = 0;

    /// Get the wheel radius (for visualization only).
    virtual double GetRadius() const { return 0; }

    /// Get the wheel width (for visualization only).
    virtual double GetWidth() const { return 0; }

    /// Initialize this wheel subsystem.
    /// The wheel mass and inertia are used to increment those of the spindle.
    virtual void Initialize(std::shared_ptr<ChBody> spindle  ///< handle to the associated spindle body
                            );

    /// Add visualization assets for the wheel subsystem.
    /// This default implementation uses primitives.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the wheel subsystem.
    virtual void RemoveVisualizationAssets() override;

  protected:
    std::shared_ptr<ChBody> m_spindle;             ///< associated spindle body
    std::shared_ptr<ChCylinderShape> m_cyl_shape;  ///< visualization cylinder asset
};

/// Vector of handles to wheel subsystems.
typedef std::vector<std::shared_ptr<ChWheel> > ChWheelList;

/// @} vehicle_wheeled_wheel

}  // end namespace vehicle
}  // end namespace chrono

#endif
