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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Wrapper classes for modeling an entire mrole vehicle assembly,
// a concept demonstrator for a multirole land vehicle,
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#ifndef MROLE_H
#define MROLE_H

#include <array>
#include <string>

#include "chrono_vehicle/ChPowertrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChTire.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/mrole/mrole_VehicleFull.h"
#include "chrono_models/vehicle/mrole/mrole_VehicleReduced.h"

namespace chrono {
namespace vehicle {
namespace mrole {

/// @addtogroup vehicle_models_mrole
/// @{

/// Definition of the mrole assembly.
/// This class encapsulates a concrete wheeled vehicle model with parameters corresponding to
/// a mrole, the powertrain model, and the 8 tires. It provides wrappers to access the different
/// systems and subsystems, functions for specifying the driveline, powertrain, and tire types,
/// as well as functions for controlling the visualization mode of each component.
/// Note that this is an abstract class which cannot be instantiated.  Instead, use one of the
/// concrete classes mrole_Full or mrole_Reduced.
class CH_MODELS_API mrole {
  public:
    // simple mimic of central tire inflation system (CTIS), lets the user select use cases
    typedef enum { ROAD, OFFROAD_SOIL, OFFROAD_SAND } CTIS;

  public:
    virtual ~mrole();

    void SetContactMethod(ChContactMethod val) { m_contactMethod = val; }

    void SetChassisFixed(bool val) { m_fixed = val; }
    void SetChassisCollisionType(CollisionType val) { m_chassisCollisionType = val; }

    void SetDriveType(DrivelineTypeWV val) { m_driveType = val; }
    void SetBrakeType(BrakeType brake_type) { m_brake_type = brake_type; }
    void SetPowertrainType(PowertrainModelType val) { m_powertrainType = val; }
    void SetTireType(TireModelType val) { m_tireType = val; }

    void SetTireCollisionType(ChTire::CollisionType collision_type) { m_tire_collision_type = collision_type; }

    void SetInitPosition(const ChCoordsys<>& pos) { m_initPos = pos; }
    void SetInitFwdVel(double fwdVel) { m_initFwdVel = fwdVel; }
    void SetInitWheelAngVel(const std::vector<double>& omega) { m_initOmega = omega; }

    void SetTireStepSize(double step_size) { m_tire_step_size = step_size; }

    void EnableBrakeLocking(bool lock) { m_brake_locking = lock; }

    ChSystem* GetSystem() const { return m_vehicle->GetSystem(); }
    ChWheeledVehicle& GetVehicle() const { return *m_vehicle; }
    std::shared_ptr<ChChassis> GetChassis() const { return m_vehicle->GetChassis(); }
    std::shared_ptr<ChBodyAuxRef> GetChassisBody() const { return m_vehicle->GetChassisBody(); }
    std::shared_ptr<ChPowertrain> GetPowertrain() const { return m_vehicle->GetPowertrain(); }

    void Initialize();

    void LockAxleDifferential(int axle, bool lock) { m_vehicle->LockAxleDifferential(axle, lock); }
    void LockCentralDifferential(int which, bool lock) { m_vehicle->LockCentralDifferential(which, lock); }

    void SetAerodynamicDrag(double Cd, double area, double air_density);

    void SetChassisVisualizationType(VisualizationType vis) { m_vehicle->SetChassisVisualizationType(vis); }
    void SetSuspensionVisualizationType(VisualizationType vis) { m_vehicle->SetSuspensionVisualizationType(vis); }
    void SetSteeringVisualizationType(VisualizationType vis) { m_vehicle->SetSteeringVisualizationType(vis); }
    void SetWheelVisualizationType(VisualizationType vis) { m_vehicle->SetWheelVisualizationType(vis); }
    void SetTireVisualizationType(VisualizationType vis) { m_vehicle->SetTireVisualizationType(vis); }

    void Synchronize(double time, const ChDriver::Inputs& driver_inputs, const ChTerrain& terrain);
    void Advance(double step);
    void SelectRoadOperation() { m_ctis = CTIS::ROAD; }  // set by default
    void SelectOffroadSoilOperation() { m_ctis = CTIS::OFFROAD_SOIL; }
    void SelectOffroadSandOperation() { m_ctis = CTIS::OFFROAD_SAND; }
    double GetMaxTireSpeed();

  protected:
    // Protected constructors -- this class cannot be instantiated by itself.
    mrole();
    mrole(ChSystem* system);

    virtual mrole_Vehicle* CreateVehicle() = 0;

    CTIS m_ctis;

    ChContactMethod m_contactMethod;
    CollisionType m_chassisCollisionType;
    bool m_fixed;
    bool m_brake_locking;

    DrivelineTypeWV m_driveType;
    PowertrainModelType m_powertrainType;
    BrakeType m_brake_type;
    TireModelType m_tireType;
    ChTire::CollisionType m_tire_collision_type;

    double m_tire_step_size;

    ChCoordsys<> m_initPos;
    double m_initFwdVel;
    std::vector<double> m_initOmega;

    bool m_apply_drag;
    double m_Cd;
    double m_area;
    double m_air_density;

    ChSystem* m_system;
    mrole_Vehicle* m_vehicle;

    double m_tire_mass;
};

/// Definition of a mrole vehicle assembly (vehicle, powertrain, and tires), using full
/// double wishbone suspensions (i.e., suspensions that include rigid bodies for the upper
/// and lower control arms) and a Pitman arm steering mechanism.
class CH_MODELS_API mrole_Full : public mrole {
  public:
    mrole_Full() : m_steeringType(SteeringTypeWV::PITMAN_ARM), m_rigidColumn(false) {}
    mrole_Full(ChSystem* system) : mrole(system), m_steeringType(SteeringTypeWV::PITMAN_ARM), m_rigidColumn(false) {}

    /// Set the type of steering mechanism (PITMAN_ARM or PITMAN_ARM_SHAFTS.
    /// Default: PITMAN_ARM
    void SetSteeringType(SteeringTypeWV val) { m_steeringType = val; }

    /// Force a rigid steering column (PITMAN_ARM_SHAFTS only).
    /// Default: false (compliant column).
    void SetRigidSteeringColumn(bool val) { m_rigidColumn = val; }

    void LogHardpointLocations() { ((mrole_VehicleFull*)m_vehicle)->LogHardpointLocations(); }
    void DebugLog(int what) { ((mrole_VehicleFull*)m_vehicle)->DebugLog(what); }

  private:
    virtual mrole_Vehicle* CreateVehicle() override;

    SteeringTypeWV m_steeringType;  ///< type of steering mechanism
    bool m_rigidColumn;             ///< only used with PITMAN_ARM_SHAFT
};

/// Definition of a mrole vehicle assembly (vehicle, powertrain, and tires), using reduced
/// double wishbone suspensions (i.e., suspensions that replace the upper and lower control
/// arms with distance constraints) and a rack-pinion steering mechanism.
class CH_MODELS_API mrole_Reduced : public mrole {
  public:
    mrole_Reduced() {}
    mrole_Reduced(ChSystem* system) : mrole(system) {}

  private:
    virtual mrole_Vehicle* CreateVehicle() override;
};

/// @} vehicle_models_hmmwv

}  // namespace mrole
}  // end namespace vehicle
}  // end namespace chrono

#endif
