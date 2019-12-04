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

// Forward declaration
class ChTire;

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

    /// Initialize this wheel subsystem by associating it to an existing spindle of a suspension subsystem.
    /// The optional 'offset' argument allows models with double wheels(tires). The default value offset=0 corresponds
    /// to an axle with a single tire. A positive offset corresponds to an "outer" wheel, while a negative offset
    /// corresponds to an "inner" wheel. The wheel mass and inertia are used to increment those of the associated
    /// spindle body.
    virtual void Initialize(std::shared_ptr<ChBody> spindle,  ///< associated suspension spindle body
                            VehicleSide side,                 ///< wheel mounted on left/right side
                            double offset = 0                 ///< offset from associated spindle center
    );

    /// Synchronize the wheel subsystem. 
    /// This queries the forces from the attached tire and passes it to the associated suspension.
    void Synchronize();

    /// Get the tire attached to this wheel.
    std::shared_ptr<ChTire> GetTire() const { return m_tire; }

    /// Associate the given tire with this wheel.
    void SetTire(std::shared_ptr<ChTire> tire) { m_tire = tire; }

    /// Get the associated spindle body.
    std::shared_ptr<ChBody> GetSpindle() const { return m_spindle; }

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
    std::shared_ptr<ChBody> m_spindle;             ///< associated suspension spindle body
    std::shared_ptr<ChTire> m_tire;                ///< attached tire subsystem
    VehicleSide m_side;                            ///< wheel mounted on left/right side
    double m_offset;                               ///< offset from spindle center
    std::shared_ptr<ChCylinderShape> m_cyl_shape;  ///< visualization cylinder asset

    friend class ChTire;
    friend class ChWheeledVehicle;
};

/// Vector of handles to wheel subsystems.
typedef std::vector<std::shared_ptr<ChWheel> > ChWheelList;

/// @} vehicle_wheeled_wheel

}  // end namespace vehicle
}  // end namespace chrono

#endif
