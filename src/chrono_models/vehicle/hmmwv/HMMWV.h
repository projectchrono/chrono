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
// Authors: Radu Serban
// =============================================================================
//
// Wrapper classes for modeling an entire HMMWV vehicle assembly
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#ifndef HMMWV_H
#define HMMWV_H

#include <array>
#include <string>

#include "chrono_vehicle/ChPowertrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_VehicleFull.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_VehicleReduced.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

/// @addtogroup vehicle_models_hmmwv
/// @{

/// Definition of the HMMWV assembly.
/// This class encapsulates a concrete wheeled vehicle model with parameters corresponding to
/// a HMMWV, the powertrain model, and the 4 tires. It provides wrappers to access the different
/// systems and subsystems, functions for specifying the driveline, powertrain, and tire types,
/// as well as functions for controlling the visualization mode of each component.
/// Note that this is an abstract class which cannot be instantiated.  Instead, use one of the
/// concrete classes HMMWV_Full or HMMWV_Reduced.
class CH_MODELS_API HMMWV {
  public:
    virtual ~HMMWV();

    void SetContactMethod(ChMaterialSurface::ContactMethod val) { m_contactMethod = val; }

    void SetChassisFixed(bool val) { m_fixed = val; }
    void SetChassisCollisionType(ChassisCollisionType val) { m_chassisCollisionType = val; }

    void SetDriveType(DrivelineType val) { m_driveType = val; }
    void SetPowertrainType(PowertrainModelType val) { m_powertrainType = val; }
    void SetTireType(TireModelType val) { m_tireType = val; }

    void SetTireCollisionType(ChTire::CollisionType collision_type) { m_tire_collision_type = collision_type; }

    void SetInitPosition(const ChCoordsys<>& pos) { m_initPos = pos; }
    void SetInitFwdVel(double fwdVel) { m_initFwdVel = fwdVel; }
    void SetInitWheelAngVel(const std::vector<double>& omega) { m_initOmega = omega; }

    void SetVehicleStepSize(double step_size) { m_vehicle_step_size = step_size; }
    void SetTireStepSize(double step_size) { m_tire_step_size = step_size; }

    ChSystem* GetSystem() const { return m_vehicle->GetSystem(); }
    ChWheeledVehicle& GetVehicle() const { return *m_vehicle; }
    std::shared_ptr<ChChassis> GetChassis() const { return m_vehicle->GetChassis(); }
    std::shared_ptr<ChBodyAuxRef> GetChassisBody() const { return m_vehicle->GetChassisBody(); }
    ChPowertrain& GetPowertrain() const { return *m_powertrain; }
    ChTire* GetTire(WheelID which) const { return m_tires[which.id()]; }
    double GetTotalMass() const;

    void Initialize();

    void LockAxleDifferential(int axle, bool lock) { m_vehicle->LockAxleDifferential(axle, lock); }
    void LockCentralDifferential(int which, bool lock) { m_vehicle->LockCentralDifferential(which, lock); }

    void SetAerodynamicDrag(double Cd, double area, double air_density);

    void SetChassisVisualizationType(VisualizationType vis) { m_vehicle->SetChassisVisualizationType(vis); }
    void SetSuspensionVisualizationType(VisualizationType vis) { m_vehicle->SetSuspensionVisualizationType(vis); }
    void SetSteeringVisualizationType(VisualizationType vis) { m_vehicle->SetSteeringVisualizationType(vis); }
    void SetWheelVisualizationType(VisualizationType vis) { m_vehicle->SetWheelVisualizationType(vis); }
    void SetTireVisualizationType(VisualizationType vis);

    void Synchronize(double time,
                     double steering_input,
                     double braking_input,
                     double throttle_input,
                     const ChTerrain& terrain);

    void Advance(double step);

  protected:
    // Protected constructors -- this class cannot be instantiated by itself.
    HMMWV();
    HMMWV(ChSystem* system);

    virtual HMMWV_Vehicle* CreateVehicle() = 0;

    ChMaterialSurface::ContactMethod m_contactMethod;
    ChassisCollisionType m_chassisCollisionType;
    bool m_fixed;

    DrivelineType m_driveType;
    PowertrainModelType m_powertrainType;
    TireModelType m_tireType;
    ChTire::CollisionType m_tire_collision_type;

    double m_vehicle_step_size;
    double m_tire_step_size;

    ChCoordsys<> m_initPos;
    double m_initFwdVel;
    std::vector<double> m_initOmega;

    bool m_apply_drag;
    double m_Cd;
    double m_area;
    double m_air_density;

    ChSystem* m_system;
    HMMWV_Vehicle* m_vehicle;
    ChPowertrain* m_powertrain;
    std::array<ChTire*, 4> m_tires;

    double m_tire_mass;
};

/// Definition of a HMMWV vehicle assembly (vehicle, powertrain, and tires), using full
/// double wishbone suspensions (i.e., suspensions that include rigid bodies for the upper
/// and lower control arms) and a Pitman arm steering mechanism.
class CH_MODELS_API HMMWV_Full : public HMMWV {
  public:
    HMMWV_Full() : m_steeringType(SteeringType::PITMAN_ARM), m_rigidColumn(false) {}
    HMMWV_Full(ChSystem* system) : HMMWV(system), m_steeringType(SteeringType::PITMAN_ARM), m_rigidColumn(false) {}

    /// Set the type of steering mechanism (PITMAN_ARM or PITMAN_ARM_SHAFTS.
    /// Default: PITMAN_ARM
    void SetSteeringType(SteeringType val) { m_steeringType = val; }

    /// Force a rigid steering column (PITMAN_ARM_SHAFTS only).
    /// Default: false (compliant column).
    void SetRigidSteeringColumn(bool val) { m_rigidColumn = val; }

    void LogHardpointLocations() { ((HMMWV_VehicleFull*)m_vehicle)->LogHardpointLocations(); }
    void DebugLog(int what) { ((HMMWV_VehicleFull*)m_vehicle)->DebugLog(what); }

  private:
    virtual HMMWV_Vehicle* CreateVehicle() override;

    SteeringType m_steeringType;  ///< type of steering mechanism
    bool m_rigidColumn;           ///< only used with PITMAN_ARM_SHAFT
};

/// Definition of a HMMWV vehicle assembly (vehicle, powertrain, and tires), using reduced
/// double wishbone suspensions (i.e., suspensions that replace the upper and lower control
/// arms with distance constraints) and a rack-pinion steering mechanism.
class CH_MODELS_API HMMWV_Reduced : public HMMWV {
  public:
    HMMWV_Reduced() {}
    HMMWV_Reduced(ChSystem* system) : HMMWV(system) {}

  private:
    virtual HMMWV_Vehicle* CreateVehicle() override;
};

/// @} vehicle_models_hmmwv

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

#endif
