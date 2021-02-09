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
// Base class for the Gator vehicle
//
// =============================================================================

#ifndef GATOR_VEHICLE_H
#define GATOR_VEHICLE_H

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

namespace chrono {
namespace vehicle {
namespace gator {

/// @addtogroup vehicle_models_gator
/// @{

/// Gator vehicle system.
class CH_MODELS_API Gator_Vehicle : public ChWheeledVehicle {
  public:
    Gator_Vehicle(const bool fixed,
                  DrivelineType driveline_type,
                  BrakeType brake_type,
                  ChContactMethod contact_method,
                  CollisionType chassis_collision_type);

    Gator_Vehicle(ChSystem* system,
                  const bool fixed,
                  DrivelineType driveline_type,
                  BrakeType brake_type,
                  CollisionType chassis_collision_type);

    ~Gator_Vehicle();

    virtual int GetNumberAxles() const override { return 2; }

    virtual double GetWheelbase() const override { return 2.776; }
    virtual double GetMinTurningRadius() const override { return 7.6; }
    virtual double GetMaxSteeringAngle() const override { return 25.0 * CH_C_DEG_TO_RAD; }

    void SetInitWheelAngVel(const std::vector<double>& omega) {
        assert(omega.size() == 4);
        m_omega = omega;
    }

    double GetShockForce(int axle, VehicleSide side) const;
    double GetShockLength(int axle, VehicleSide side) const;
    double GetShockVelocity(int axle, VehicleSide side) const;

    virtual void Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;

    // Log debugging information
    void LogHardpointLocations();  /// suspension hardpoints at design
    void DebugLog(int what);       /// shock forces and lengths, constraints, etc.

  private:
    void Create(bool fixed,
                DrivelineType driveline_type,
                BrakeType brake_type,
                CollisionType chassis_collision_type);

    std::vector<double> m_omega;
};

/// @} vehicle_models_gator

}  // end namespace gator
}  // end namespace vehicle
}  // end namespace chrono

#endif
