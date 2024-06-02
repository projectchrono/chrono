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
// Authors: Radu Serban, Asher Elmquist
// =============================================================================
//
// Base class for the Sedan vehicle models
//
// =============================================================================

#ifndef FEDA_VEHICLE_H
#define FEDA_VEHICLE_H

#include <vector>

#include "chrono/core/ChCoordsys.h"
#include "chrono/utils/ChUtils.h"
#include "chrono/physics/ChContactMaterial.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

namespace chrono {
namespace vehicle {
namespace feda {

/// @addtogroup vehicle_models_feda
/// @{

/// FEDA vehicle system.
class CH_MODELS_API FEDA_Vehicle : public ChWheeledVehicle {
  public:
    FEDA_Vehicle(const bool fixed = false,
                 BrakeType brake_type = BrakeType::SIMPLE,
                 ChContactMethod contact_method = ChContactMethod::NSC,
                 CollisionType chassis_collision_type = CollisionType::NONE,
                 int ride_height = 1,
                 int damperMode = 2);

    FEDA_Vehicle(ChSystem* system,
                 const bool fixed = false,
                 BrakeType brake_type = BrakeType::SIMPLE,
                 CollisionType chassis_collision_type = CollisionType::NONE,
                 int ride_height = 1,
                 int damperMode = 2);

    ~FEDA_Vehicle();

    virtual unsigned int GetNumberAxles() const override { return 2; }

    virtual double GetWheelbase() const override { return 3.302; }
    virtual double GetMinTurningRadius() const override { return 6.52332; } // Center trace avg.
    virtual double GetMaxSteeringAngle() const override { return 0.53237; } // L/R avg.

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

    void SetRideHeight(int theConfig) { m_ride_height = ChClamp(theConfig, 0, 2); }

    virtual void Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;

    // Log debugging information
    void LogHardpointLocations();  /// suspension hardpoints at design
    void DebugLog(int what);       /// shock forces and lengths, constraints, etc.

  private:
    void Create(bool fixed, BrakeType brake_type, CollisionType chassis_collision_type);

    std::vector<double> m_omega;

    int m_ride_height;
    int m_damper_mode;
};

/// @} vehicle_models_feda

}  // namespace feda
}  // end namespace vehicle
}  // end namespace chrono

#endif
