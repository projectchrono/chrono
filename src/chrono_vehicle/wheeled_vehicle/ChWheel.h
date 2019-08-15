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
#include "chrono_vehicle/wheeled_vehicle/ChSuspension.h"

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

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "Wheel"; }

    /// Get the wheel mass.
    virtual double GetMass() const = 0;

    /// Get the wheel moments of inertia.
    virtual ChVector<> GetInertia() const = 0;

    /// Get the wheel radius (for visualization only).
    virtual double GetRadius() const { return 0; }

    /// Get the wheel width (for visualization only).
    virtual double GetWidth() const { return 0; }

    /// Initialize this wheel subsystem by associating it to an existing suspension subsystem.
    /// The optional 'offset' argument allows models with double wheels(tires).
    /// The default value offset=0 corresponds to an axle with a single tire.
    /// The wheel mass and inertia are used to increment those of the associated spindle body.
    virtual void Initialize(std::shared_ptr<ChSuspension> suspension,  ///< associated suspension subsystem
                            VehicleSide side,                          ///< wheel mounted on left/right side
                            double offset = 0                          ///< offset from associated spindle offset
    );

    /// Get the associated spindle body.
    std::shared_ptr<ChBody> GetSpindle() const { return m_suspension->GetSpindle(m_side); }

    /// Get the vehicle side on which this wheel is mounted.
    VehicleSide GetSide() const { return m_side; }

    /// Get wheel position (expressed in absolute frame).
    ChVector<> GetPos() const;

    /// Get the current state for this wheel.
    /// This includes the location, orientation, linear and angular velocities,
    /// all expressed in the global reference frame, as well as the wheel angular
    /// speed about its rotation axis.
    WheelState GetState() const;

    /// Add visualization assets for the wheel subsystem.
    /// This default implementation uses primitives.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the wheel subsystem.
    virtual void RemoveVisualizationAssets() override;

  protected:
    std::shared_ptr<ChSuspension> m_suspension;    ///< associated suspension subsystem
    VehicleSide m_side;                            ///< wheel mounted on left/right side
    double m_offset;                               ///< offset from spindle center
    std::shared_ptr<ChCylinderShape> m_cyl_shape;  ///< visualization cylinder asset
};

/// Vector of handles to wheel subsystems.
typedef std::vector<std::shared_ptr<ChWheel> > ChWheelList;

/// @} vehicle_wheeled_wheel

}  // end namespace vehicle
}  // end namespace chrono

#endif
