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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Semitractor for the long haul vehicle model based on Kraz 64431 data
//
// =============================================================================

#ifndef KRAZ_TRACTOR_H
#define KRAZ_TRACTOR_H

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

namespace chrono {
namespace vehicle {
namespace kraz {

/// @addtogroup vehicle_models_kraz
/// @{

/// Kraz tractor system.
class CH_MODELS_API Kraz_tractor : public ChWheeledVehicle {
  public:
    Kraz_tractor(bool fixed,
                 CollisionType chassis_collision_type = CollisionType::NONE,
                 ChContactMethod contactMethod = ChContactMethod::NSC);
    Kraz_tractor(ChSystem* system, bool fixed, CollisionType chassis_collision_type = CollisionType::NONE);
    ~Kraz_tractor() {}

    virtual unsigned int GetNumberAxles() const override { return 3; }

    virtual double GetWheelbase() const override { return 4.78; }
    virtual double GetMinTurningRadius() const override { return 7.7; }
    virtual double GetMaxSteeringAngle() const override { return 30 * chrono::CH_DEG_TO_RAD; }

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
    void Create(bool fixed, CollisionType chassis_collision_type);
};

/// @} vehicle_models_kraz

}  // end namespace kraz
}  // end namespace vehicle
}  // end namespace chrono

#endif
