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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz
// =============================================================================
//
// Generic 2-axle vehicle model.
// Can be constructed either with solid-axle or with multi-link suspensions.
// Always uses a generic rack-pinion steering and a 2WD driveline model.
//
// =============================================================================

#ifndef GENERIC_VEHICLE_H
#define GENERIC_VEHICLE_H

#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChMaterialSurface.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace generic {

/// @addtogroup vehicle_models_generic
/// @{

/// Definition of a Generic wheeled vehicle.
/// This 2-axle vehicle is used as a sandbox for testing the various templates for wheeled vehicle
/// subsystems (suspensions, steering, tires, etc.)
class CH_MODELS_API Generic_Vehicle : public ChWheeledVehicle {
  public:
    /// Create a generic vehicle with the specified types of subsystems.
    Generic_Vehicle(const bool fixed,
                    SuspensionTypeWV suspension_type,
                    SteeringTypeWV steering_type,
                    DrivelineTypeWV driveline_type,
                    BrakeType brake_type,
                    ChContactMethod contactMethod = ChContactMethod::NSC);

    ~Generic_Vehicle() {}

    virtual int GetNumberAxles() const override { return 2; }

    virtual double GetWheelbase() const override { return 3.378; }
    virtual double GetMinTurningRadius() const override { return 8.0; }
    virtual double GetMaxSteeringAngle() const override { return 25 * CH_C_DEG_TO_RAD; }

    /// Initialize the vehicle at the specified location and with specified orientation.
    virtual void Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;

    /// Utility function to create and initialize the 4 vehicle tires to the specified type.
    void CreateAndInitializeTires(TireModelType tire_type, VisualizationType vis_type);

    /// Utility function to create and initialize the powertrain system using the specified types.
    void CreateAndInitializePowertrain(EngineModelType engine_type, TransmissionModelType transmission_type);

    // Log debugging information
    void LogHardpointLocations();  /// suspension hardpoints at design
    void DebugLog(int what);       /// shock forces and lengths, constraints, etc.

  private:
    SuspensionTypeWV m_suspension_type;
    SteeringTypeWV m_steering_type;
    DrivelineTypeWV m_driveline_type;
    BrakeType m_brake_type;
};

/// @} vehicle_models_generic

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono

#endif
