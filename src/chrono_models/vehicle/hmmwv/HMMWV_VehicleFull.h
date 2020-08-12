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
// HMMWV full vehicle model...
//
// =============================================================================

#ifndef HMMWV_VEHICLE_FULL_H
#define HMMWV_VEHICLE_FULL_H

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Vehicle.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

/// @addtogroup vehicle_models_hmmwv
/// @{

/// HMMWV vehicle system using full double wishbone suspension (control arms modeled using rigid bodies)
/// and Pitman arm steering mechanism.
class CH_MODELS_API HMMWV_VehicleFull : public HMMWV_Vehicle {
  public:
    HMMWV_VehicleFull(const bool fixed,
                      DrivelineType drive_type,
                      BrakeType brake_type,
                      SteeringType steering_type,
                      bool rigid_steering_column,
                      ChContactMethod contact_method,
                      ChassisCollisionType chassis_collision_type);

    HMMWV_VehicleFull(ChSystem* system,
                      const bool fixed,
                      DrivelineType drive_type,
                      BrakeType brake_type,
                      SteeringType steering_type,
                      bool rigid_steering_column,
                      ChassisCollisionType chassis_collision_type);

    ~HMMWV_VehicleFull();

    double GetSpringForce(int axle, VehicleSide side) const;
    double GetSpringLength(int axle, VehicleSide side) const;
    double GetSpringDeformation(int axle, VehicleSide side) const;

    double GetShockForce(int axle, VehicleSide side) const;
    double GetShockLength(int axle, VehicleSide side) const;
    double GetShockVelocity(int axle, VehicleSide side) const;

    virtual void Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;

    // Log debugging information
    void LogHardpointLocations();  /// suspension hardpoints at design
    void DebugLog(int what);       /// shock forces and lengths, constraints, etc.

  private:
    void Create(bool fixed,
                BrakeType brake_type,
                SteeringType steering_type,
                bool rigid_steering_column,
                ChassisCollisionType chassis_collision_type);
};

/// @} vehicle_models_hmmwv

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

#endif
