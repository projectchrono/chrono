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
// Authors: Radu Serban, Asher Elmquist, Rainer Gericke
// =============================================================================
//
// Base class for modeling an entire Jeep Cherokee 1997 vehicle assembly
// Vehicle Parameters taken from SAE Paper 1999-01-0121
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#ifndef CHRONO_CHEROKEE_VEHICLE_H
#define CHRONO_CHEROKEE_VEHICLE_H

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

namespace chrono {
namespace vehicle {
namespace jeep {

/// @addtogroup vehicle_models_cherokee
/// @{

/// Sedan vehicle system.
class CH_MODELS_API Cherokee_Vehicle : public ChWheeledVehicle {
  public:
    Cherokee_Vehicle(const bool fixed,
                    BrakeType brake_type,
                    ChContactMethod contact_method = ChContactMethod::NSC,
                    CollisionType chassis_collision_type = CollisionType::NONE);

    Cherokee_Vehicle(ChSystem* system,
                    const bool fixed,
                    BrakeType brake_type,
                    CollisionType chassis_collision_type = CollisionType::NONE);

    ~Cherokee_Vehicle();

    virtual unsigned int GetNumberAxles() const override { return 2; }

    virtual double GetWheelbase() const override { return 2.578; }
    virtual double GetMinTurningRadius() const override { return 4.39775; }
    virtual double GetMaxSteeringAngle() const override { return 0.656422; }

    void SetInitWheelAngVel(const std::vector<double>& omega) {
        assert(omega.size() == 4);
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
};

/// @} vehicle_models_cherokee

}  // namespace bmw
}  // end namespace vehicle
}  // end namespace chrono

#endif  // CHRONO_CHEROKEE_VEHICLE_H
