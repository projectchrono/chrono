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
// Authors: Radu Serban
// =============================================================================
//
// M113 vehicle model.
//
// =============================================================================

#ifndef M113_VEHICLE_H
#define M113_VEHICLE_H

#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

namespace chrono {
namespace vehicle {
namespace m113 {

class CH_MODELS_API M113_Vehicle : public ChTrackedVehicle {
  public:
    M113_Vehicle(bool fixed,
                 TrackShoeType shoe_type,
                 ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DVI,
                 ChassisCollisionType chassis_collision_type = ChassisCollisionType::NONE);

    M113_Vehicle(bool fixed,
                 TrackShoeType shoe_type,
                 ChSystem* system,
                 ChassisCollisionType chassis_collision_type = ChassisCollisionType::NONE);

    ~M113_Vehicle() {}

    virtual void Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;

  private:
    void Create(bool fixed, ChassisCollisionType chassis_collision_type);

    TrackShoeType m_type;  ///< type of track assembly (SINGLE_PIN or DOUBLE_PIN)
};

}  // end namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
