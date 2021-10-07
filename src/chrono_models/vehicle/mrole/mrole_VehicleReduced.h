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
// mrole 9-body vehicle model...
//
// =============================================================================

#ifndef MROLE_VEHICLE_REDUCED_H
#define MROLE_VEHICLE_REDUCED_H

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"
#include "chrono_models/vehicle/mrole/mrole_Vehicle.h"

namespace chrono {
namespace vehicle {
namespace mrole {

/// @addtogroup vehicle_models_mrole
/// @{

/// mrole vehicle system using reduced double wishbone suspension (control arms modeled using distance constraints)
/// and rack-pinion steering mechanism.
class CH_MODELS_API mrole_VehicleReduced : public mrole_Vehicle {
  public:
    mrole_VehicleReduced(const bool fixed,
                         DrivelineTypeWV drive_type,
                         BrakeType brake_type,
                         ChContactMethod contact_method,
                         CollisionType chassis_collision_type);

    mrole_VehicleReduced(ChSystem* system,
                         const bool fixed,
                         DrivelineTypeWV drive_type,
                         BrakeType brake_type,
                         CollisionType chassis_collision_type);

    ~mrole_VehicleReduced();

    virtual void Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;

  private:
    void Create(bool fixed, BrakeType brake_type, CollisionType chassis_collision_type);
};

/// @} vehicle_models_mrole

}  // namespace mrole
}  // end namespace vehicle
}  // end namespace chrono

#endif
