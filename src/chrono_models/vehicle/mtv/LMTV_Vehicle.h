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
// Base class for the LMTV vehicle models
//
// =============================================================================

#ifndef LMTV_VEHICLE_H
#define LMTV_VEHICLE_H

#include <vector>

#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChMaterialSurface.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

#include "chrono_models/vehicle/mtv/LMTV_BrakeSimple.h"
#include "chrono_models/vehicle/mtv/LMTV_Chassis.h"
#include "chrono_models/vehicle/mtv/LMTV_Driveline4WD.h"
#include "chrono_models/vehicle/mtv/LMTV_LeafspringAxle.h"
#include "chrono_models/vehicle/mtv/LMTV_AntiRollBar.h"
#include "chrono_models/vehicle/mtv/LMTV_RotaryArm.h"
#include "chrono_models/vehicle/mtv/LMTV_SimpleMapPowertrain.h"
#include "chrono_models/vehicle/mtv/LMTV_ToebarLeafspringAxle.h"
#include "chrono_models/vehicle/mtv/LMTV_Wheel.h"

namespace chrono {
namespace vehicle {
namespace mtv {

/// @addtogroup vehicle_models_mtv
/// @{

/// LMTV vehicle system.
class CH_MODELS_API LMTV_Vehicle : public ChWheeledVehicle {
  public:
    LMTV_Vehicle(const bool fixed = false,
                 SteeringType steering_model = SteeringType::PITMAN_ARM,
                 ChContactMethod contact_method = ChContactMethod::NSC,
                 ChassisCollisionType chassis_collision_type = ChassisCollisionType::NONE);

    LMTV_Vehicle(ChSystem* system,
                 const bool fixed = false,
                 SteeringType steering_model = SteeringType::PITMAN_ARM,
                 ChassisCollisionType chassis_collision_type = ChassisCollisionType::NONE);

    ~LMTV_Vehicle();

    virtual int GetNumberAxles() const override { return 2; }

    virtual double GetWheelbase() const override { return 3.8; }
    virtual double GetMinTurningRadius() const override { return 11.0; }
    virtual double GetMaxSteeringAngle() const override { return 24.6 * CH_C_DEG_TO_RAD; }

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

    /// Get the vehicle total mass.
    /// This includes the mass of the chassis and all vehicle subsystems, but not the mass of tires.
    /// ChTorsionChassis needs special attention
    virtual double GetVehicleMass() const override;

    virtual void Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;

    // Log debugging information
    void LogHardpointLocations();  /// suspension hardpoints at design
    void DebugLog(int what);       /// shock forces and lengths, constraints, etc.

  private:
    void Create(bool fixed, SteeringType steering_model, ChassisCollisionType chassis_collision_type);

    std::vector<double> m_omega;
};

/// @} vehicle_models_mtv

}  // namespace mtv
}  // end namespace vehicle
}  // end namespace chrono

#endif
