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
// Base class for the BMW E90 vehicle models
// Vehicle Parameters taken from SAE Paper 2007-01-0818
//
// Suspension/ARB parameters fit to test results presented in SAE Paper 2007-01-0817
// The chassis roll angle = 2.0 deg at steady state lateral acceleration = 0.6 g
//
// Steady state cornering simulation shows that the vehicle is understeering, there
// is no real test available, but this result was expected for a passenger car.
//
// The steering geometry has been changed for chrono, because the original data
// lead to a situation where the inner wheel turn angle is smaller than the outer
// one at cornering. The steering trapez has been recalculated to standard behavior.
//
// The tire parameters where calculated from data with a Matlab script, both presented
// in SAE Paper 2007-01-0818. Actually no alignment torque, camber influence and
// relaxation is considered.
//
// SAE 2007-01-0817 shows a test called  'Slowly Increasing Steer', where the vehicle
// runs with constant speed of 50 mph (80 km/h) and the steering wheel angle is increased
// until the friction limit is reached. To reproduce a similar result with the
// cornering test the road friction coefficient must be set to 1.0.
//
// The wall-to-wall test shows a minimal distance of 11.4m.
// =============================================================================

#ifndef BMW_E90_VEHICLE_H
#define BMW_E90_VEHICLE_H

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

namespace chrono {
namespace vehicle {
namespace bmw {

/// @addtogroup vehicle_models_bmw
/// @{

/// Sedan vehicle system.
class CH_MODELS_API BMW_E90_Vehicle : public ChWheeledVehicle {
  public:
    BMW_E90_Vehicle(const bool fixed,
                    BrakeType brake_type,
                    ChContactMethod contact_method = ChContactMethod::NSC,
                    CollisionType chassis_collision_type = CollisionType::NONE);

    BMW_E90_Vehicle(ChSystem* system,
                    const bool fixed,
                    BrakeType brake_type,
                    CollisionType chassis_collision_type = CollisionType::NONE);

    ~BMW_E90_Vehicle();

    virtual unsigned int GetNumberAxles() const override { return 2; }

    virtual double GetWheelbase() const override { return 2.776; }
    virtual double GetMinTurningRadius() const override { return 4.702; }
    virtual double GetMaxSteeringAngle() const override { return 0.626671; }

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

/// @} vehicle_models_bmw

}  // namespace bmw
}  // end namespace vehicle
}  // end namespace chrono

#endif
