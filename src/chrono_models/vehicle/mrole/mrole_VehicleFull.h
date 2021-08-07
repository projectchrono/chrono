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
// mrole full vehicle model...
//
// =============================================================================

#ifndef MROLE_VEHICLE_FULL_H
#define MROLE_VEHICLE_FULL_H

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"
#include "chrono_models/vehicle/mrole/mrole_Vehicle.h"

namespace chrono {
namespace vehicle {
namespace mrole {

/// @addtogroup vehicle_models_mrole
/// @{

/// mrole vehicle system using full double wishbone suspension (control arms modeled using rigid bodies)
/// and Pitman arm steering mechanism.
class CH_MODELS_API mrole_VehicleFull : public mrole_Vehicle {
  public:
    mrole_VehicleFull(const bool fixed,
                      DrivelineTypeWV drive_type,
                      BrakeType brake_type,
                      SteeringTypeWV steering_type,
                      bool rigid_steering_column,
                      ChContactMethod contact_method,
                      CollisionType chassis_collision_type);

    mrole_VehicleFull(ChSystem* system,
                      const bool fixed,
                      DrivelineTypeWV drive_type,
                      BrakeType brake_type,
                      SteeringTypeWV steering_type,
                      bool rigid_steering_column,
                      CollisionType chassis_collision_type);

    ~mrole_VehicleFull();

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
                SteeringTypeWV steering_type,
                bool rigid_steering_column,
                CollisionType chassis_collision_type);
};

/// @} vehicle_models_mrole

}  // namespace mrole
}  // end namespace vehicle
}  // end namespace chrono

#endif
