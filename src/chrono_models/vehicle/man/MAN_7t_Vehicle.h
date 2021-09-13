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
// Authors: Radu Serban, Asher Elmquist, Evan Hoerl, Rainer Gericke
// =============================================================================
//
// Base class for the MAN 7t vehicle models
//
// =============================================================================

#ifndef MAN7T_VEHICLE_H
#define MAN7T_VEHICLE_H

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

namespace chrono {
namespace vehicle {
namespace man {

/// @addtogroup vehicle_models_man
/// @{

class CH_MODELS_API MAN_7t_Vehicle : public ChWheeledVehicle {
  public:
    MAN_7t_Vehicle(const bool fixed,
                   BrakeType brake_type,
                   ChContactMethod contact_method = ChContactMethod::NSC,
                   CollisionType chassis_collision_type = CollisionType::NONE,
                   bool use_6WD_drivetrain = false);

    MAN_7t_Vehicle(ChSystem* system,
                   const bool fixed,
                   BrakeType brake_type,
                   CollisionType chassis_collision_type = CollisionType::NONE,
                   bool use_6WD_drivetrain = false);

    ~MAN_7t_Vehicle();

    virtual int GetNumberAxles() const override { return 3; }
    virtual double GetWheelbase() const override { return 4.5; }  // average wheelbase
    virtual double GetMinTurningRadius() const override { return 13.1; }
    virtual double GetMaxSteeringAngle() const override { return 39.0 * CH_C_DEG_TO_RAD; }

    void SetInitWheelAngVel(const std::vector<double>& omega) {
        assert(omega.size() == 6);
        m_omega = omega;
    }

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
    void Create(bool fixed, BrakeType brake_type, CollisionType chassis_collision_type);

    std::vector<double> m_omega;

    bool m_use_6WD_drivetrain;
};

/// @} vehicle_models_man

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono

#endif
