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
// Wrapper classes for modeling an entire Sedan vehicle assembly
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#ifndef FEDA_H
#define FEDA_H

#include <array>
#include <string>

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/feda/FEDA_Vehicle.h"
#include "chrono_models/vehicle/feda/FEDA_EngineSimpleMap.h"
#include "chrono_models/vehicle/feda/FEDA_AutomaticTransmissionSimpleMap.h"
#include "chrono_models/vehicle/feda/FEDA_RigidTire.h"
#include "chrono_models/vehicle/feda/FEDA_Pac02Tire.h"
#include "chrono_models/vehicle/feda/FEDA_TMsimpleTire.h"
#include "chrono_models/vehicle/feda/FEDA_TMeasyTire.h"

namespace chrono {
namespace vehicle {
namespace feda {

/// @addtogroup vehicle_models_feda
/// @{

/// Definition of the FED-alpha assembly.
/// This class encapsulates a concrete wheeled vehicle model with parameters corresponding to
/// a FED-alpha, the powertrain model, and the 4 tires. It provides wrappers to access the different
/// systems and subsystems, functions for specifying the driveline, powertrain, and tire types,
/// as well as functions for controlling the visualization mode of each component.
class CH_MODELS_API FEDA {
  public:
    enum class DamperMode { FSD, PASSIVE_LOW, PASSIVE_HIGH };
    FEDA();
    FEDA(ChSystem* system);

    ~FEDA();

    void SetContactMethod(ChContactMethod val) { m_contactMethod = val; }

    void SetChassisFixed(bool val) { m_fixed = val; }
    void SetChassisCollisionType(CollisionType val) { m_chassisCollisionType = val; }

    void SetBrakeType(BrakeType brake_type) { m_brake_type = brake_type; }
    void SetEngineType(EngineModelType val) { m_engineType = val; }
    void SetTransmissionType(TransmissionModelType val) { m_transmissionType = val; }

    void SetTireType(TireModelType val) { m_tireType = val; }
    void SetTireCollisionType(ChTire::CollisionType collType) { m_tire_collision_type = collType; }
    void SetTirePressure(double pressure) { m_tire_pressure = pressure; }

    void SetInitPosition(const ChCoordsys<>& pos) { m_initPos = pos; }
    void SetInitFwdVel(double fwdVel) { m_initFwdVel = fwdVel; }
    void SetInitWheelAngVel(const std::vector<double>& omega) { m_initOmega = omega; }

    void SetTireStepSize(double step_size) { m_tire_step_size = step_size; }

    void SetRideHeight_Low() { m_ride_height_config = 0; }
    void SetRideHeight_OnRoad() { m_ride_height_config = 1; }
    void SetRideHeight_ObstacleCrossing() { m_ride_height_config = 2; }

    void SetDamperMode(DamperMode theDamperMode = DamperMode::FSD);

    ChSystem* GetSystem() const { return m_vehicle->GetSystem(); }
    ChWheeledVehicle& GetVehicle() const { return *m_vehicle; }
    std::shared_ptr<ChChassis> GetChassis() const { return m_vehicle->GetChassis(); }
    std::shared_ptr<ChBodyAuxRef> GetChassisBody() const { return m_vehicle->GetChassisBody(); }
    std::shared_ptr<ChTransmission> GetTransmission() const { return m_vehicle->GetTransmission(); }

    void Initialize();

    void LockAxleDifferential(int axle, bool lock) { m_vehicle->LockAxleDifferential(axle, lock); }

    void SetAerodynamicDrag(double Cd, double area, double air_density);

    void SetChassisVisualizationType(VisualizationType vis) { m_vehicle->SetChassisVisualizationType(vis); }
    void SetSuspensionVisualizationType(VisualizationType vis) { m_vehicle->SetSuspensionVisualizationType(vis); }
    void SetSteeringVisualizationType(VisualizationType vis) { m_vehicle->SetSteeringVisualizationType(vis); }
    void SetWheelVisualizationType(VisualizationType vis) { m_vehicle->SetWheelVisualizationType(vis); }
    void SetTireVisualizationType(VisualizationType vis) { m_vehicle->SetTireVisualizationType(vis); }

    void Synchronize(double time, const DriverInputs& driver_inputs, const ChTerrain& terrain);
    void Advance(double step);

    void LogHardpointLocations() { m_vehicle->LogHardpointLocations(); }
    void DebugLog(int what) { m_vehicle->DebugLog(what); }

  protected:
    ChContactMethod m_contactMethod;
    CollisionType m_chassisCollisionType;
    bool m_fixed;

    TireModelType m_tireType;
    BrakeType m_brake_type;
    EngineModelType m_engineType;
    TransmissionModelType m_transmissionType;

    double m_tire_step_size;

    ChCoordsys<> m_initPos;
    double m_initFwdVel;
    std::vector<double> m_initOmega;

    bool m_apply_drag;
    double m_Cd;
    double m_area;
    double m_air_density;

    ChSystem* m_system;
    FEDA_Vehicle* m_vehicle;

    double m_tire_mass;
    double m_tire_pressure;

    int m_ride_height_config;
    int m_damper_mode;

    ChTire::CollisionType m_tire_collision_type;
};

/// @} vehicle_models_feda

}  // namespace feda
}  // end namespace vehicle
}  // end namespace chrono

#endif
