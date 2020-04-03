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
// Base class for the MAN 10t vehicle models
//
// =============================================================================

#ifndef MAN10T_VEHICLE_H
#define MAN10T_VEHICLE_H

#include <vector>

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChMaterialSurface.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

#include "chrono_models/vehicle/man/MAN_10t_Chassis.h"
#include "chrono_models/vehicle/man/MAN_5t_BrakeSimple.h"
#include "chrono_models/vehicle/man/MAN_7t_Solid3LinkAxle.h"
#include "chrono_models/vehicle/man/MAN_10t_Front1Axle.h"
#include "chrono_models/vehicle/man/MAN_10t_Front2Axle.h"
#include "chrono_models/vehicle/man/MAN_5t_RotaryArm.h"
#include "chrono_models/vehicle/man/MAN_10t_RotaryArm2.h"
#include "chrono_models/vehicle/man/MAN_5t_Driveline4WD.h"
#include "chrono_models/vehicle/man/MAN_5t_SimpleDrivelineXWD.h"
#include "chrono_models/vehicle/man/MAN_5t_Wheel.h"

namespace chrono {
namespace vehicle {
namespace man {

/// @addtogroup vehicle_models_man
/// @{

class CH_MODELS_API MAN_10t_Vehicle : public ChWheeledVehicle {
  public:
    MAN_10t_Vehicle(const bool fixed = false,
                    ChContactMethod contact_method = ChContactMethod::NSC,
                    ChassisCollisionType chassis_collision_type = ChassisCollisionType::NONE,
                    bool useShaftDrivetrain = true);

    MAN_10t_Vehicle(ChSystem* system,
                    const bool fixed = false,
                    ChassisCollisionType chassis_collision_type = ChassisCollisionType::NONE,
                    bool useShaftDrivetrain = true);

    ~MAN_10t_Vehicle();

    virtual int GetNumberAxles() const override { return 4; }
    virtual double GetWheelbase() const override { return 6.3; }  // average wheelbase
    virtual double GetMinTurningRadius() const override { return 13.1; }
    virtual double GetMaxSteeringAngle() const override { return 39.0 * CH_C_DEG_TO_RAD; }

    void SetInitWheelAngVel(const std::vector<double>& omega) {
        assert(omega.size() == 8);
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
    void Create(bool fixed, ChassisCollisionType chassis_collision_type);

    std::vector<double> m_omega;

    bool m_use_shafts_drivetrain;
};

/// @} vehicle_models_man

}  // namespace man
}  // end namespace vehicle
}  // end namespace chrono

#endif
