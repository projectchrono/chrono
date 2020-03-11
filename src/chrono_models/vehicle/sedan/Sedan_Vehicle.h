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

#ifndef SEDAN_VEHICLE_H
#define SEDAN_VEHICLE_H

#include <vector>

#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChMaterialSurface.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/ChVehicleModelDefs.h"

#include "chrono_models/vehicle/sedan/Sedan_BrakeSimple.h"
#include "chrono_models/vehicle/sedan/Sedan_Chassis.h"
#include "chrono_models/vehicle/sedan/Sedan_DoubleWishbone.h"
#include "chrono_models/vehicle/sedan/Sedan_Driveline2WD.h"
#include "chrono_models/vehicle/sedan/Sedan_MultiLink.h"
#include "chrono_models/vehicle/sedan/Sedan_RackPinion.h"
#include "chrono_models/vehicle/sedan/Sedan_Wheel.h"

namespace chrono {
namespace vehicle {
namespace sedan {

/// @addtogroup vehicle_models_sedan
/// @{

/// Sedan vehicle system.
class CH_MODELS_API Sedan_Vehicle : public ChWheeledVehicle {
  public:
    Sedan_Vehicle(const bool fixed = false,
                  ChContactMethod contact_method = ChContactMethod::NSC,
                  ChassisCollisionType chassis_collision_type = ChassisCollisionType::NONE);

    Sedan_Vehicle(ChSystem* system,
                  const bool fixed = false,
                  ChassisCollisionType chassis_collision_type = ChassisCollisionType::NONE);

    ~Sedan_Vehicle();

    virtual int GetNumberAxles() const override { return 2; }

    virtual double GetWheelbase() const override { return 2.776; }
    virtual double GetMinTurningRadius() const override { return 7.6; }
    virtual double GetMaxSteeringAngle() const override { return 25.0 * CH_C_DEG_TO_RAD; }

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
    void Create(bool fixed, ChassisCollisionType chassis_collision_type);

    std::vector<double> m_omega;
};

/// @} vehicle_models_sedan

}  // end namespace sedan
}  // end namespace vehicle
}  // end namespace chrono

#endif
