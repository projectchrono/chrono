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
// HMMWV 9-body vehicle model...
//
// =============================================================================

#ifndef HMMWV_VEHICLE_REDUCED_H
#define HMMWV_VEHICLE_REDUCED_H

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

#include "chrono_models/vehicle/hmmwv/HMMWV_Vehicle.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Chassis.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_BrakeSimple.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_DoubleWishboneReduced.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Driveline2WD.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Driveline4WD.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_SimpleDriveline.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_RackPinion.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Wheel.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

/// @addtogroup vehicle_models_hmmwv
/// @{

/// HMMWV vehicle system using reduced double wishbone suspension (control arms modeled using distance constraints)
/// and rack-pinion steering mechanism.
class CH_MODELS_API HMMWV_VehicleReduced : public HMMWV_Vehicle {
  public:
    HMMWV_VehicleReduced(const bool fixed,
                         DrivelineType drive_type,
                         ChMaterialSurface::ContactMethod contact_method,
                         ChassisCollisionType chassis_collision_type);

    HMMWV_VehicleReduced(ChSystem* system,
                         const bool fixed,
                         DrivelineType drive_type,
                         ChassisCollisionType chassis_collision_type);

    ~HMMWV_VehicleReduced();

    virtual void Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;

  private:
    void Create(bool fixed, ChassisCollisionType chassis_collision_type);
};

/// @} vehicle_models_hmmwv

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

#endif
