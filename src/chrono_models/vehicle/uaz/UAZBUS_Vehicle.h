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
// Base class for the UAZBUS vehicle models
//
// =============================================================================

#ifndef UAZBUS_VEHICLE_H
#define UAZBUS_VEHICLE_H

#include <vector>

#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChMaterialSurface.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

#include "chrono_models/vehicle/uaz/UAZBUS_BrakeSimple.h"
#include "chrono_models/vehicle/uaz/UAZBUS_Chassis.h"
#include "chrono_models/vehicle/uaz/UAZBUS_Driveline4WD.h"
#include "chrono_models/vehicle/uaz/UAZBUS_LeafspringAxle.h"
#include "chrono_models/vehicle/uaz/UAZBUS_RotaryArm.h"
#include "chrono_models/vehicle/uaz/UAZBUS_SimpleMapPowertrain.h"
#include "chrono_models/vehicle/uaz/UAZBUS_ToeBarLeafspringAxle.h"
#include "chrono_models/vehicle/uaz/UAZBUS_Wheel.h"

namespace chrono {
namespace vehicle {
namespace uaz {

/// @addtogroup vehicle_models_uaz
/// @{

/// UAZ vehicle system.
class CH_MODELS_API UAZBUS_Vehicle : public ChWheeledVehicle {
  public:
    UAZBUS_Vehicle(const bool fixed = false,
                   SteeringType steering_model = SteeringType::PITMAN_ARM,
                   ChContactMethod contact_method = ChContactMethod::NSC,
                   ChassisCollisionType chassis_collision_type = ChassisCollisionType::NONE);

    UAZBUS_Vehicle(ChSystem* system,
                   const bool fixed = false,
                   SteeringType steering_model = SteeringType::PITMAN_ARM,
                   ChassisCollisionType chassis_collision_type = ChassisCollisionType::NONE);

    ~UAZBUS_Vehicle();

    virtual int GetNumberAxles() const override { return 2; }

    virtual double GetWheelbase() const override { return 2.3; }
    virtual double GetMinTurningRadius() const override { return 5.8; }
    virtual double GetMaxSteeringAngle() const override { return 27 * CH_C_DEG_TO_RAD; }

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
    void Create(bool fixed, SteeringType steering_model, ChassisCollisionType chassis_collision_type);

    std::vector<double> m_omega;
};

/// @} vehicle_models_uaz

}  // end namespace uaz
}  // end namespace vehicle
}  // end namespace chrono

#endif
