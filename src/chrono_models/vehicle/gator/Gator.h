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
// Authors: Radu Serban
// =============================================================================
//
// Wrapper classes for modeling an entire Gator vehicle assembly
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#ifndef GATOR_H
#define GATOR_H

#include <array>
#include <string>

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/gator/Gator_Vehicle.h"

namespace chrono {
namespace vehicle {
namespace gator {

/// @addtogroup vehicle_models_gator
/// @{

/// Definition of the Gator assembly.
/// This class encapsulates a concrete wheeled vehicle model with parameters corresponding to
/// a Gator model E4X2, the powertrain model, and the 4 tires.
class CH_MODELS_API Gator {
  public:
    Gator();
    Gator(ChSystem* system);

    ~Gator();

    void SetContactMethod(ChContactMethod contact_method) { m_contact_method = contact_method; }

    void SetChassisFixed(bool val) { m_fixed = val; }
    void SetChassisCollisionType(CollisionType chassis_collision_type) {
        m_chassis_collision_type = chassis_collision_type;
    }

    void SetBrakeType(BrakeType brake_type) { m_brake_type = brake_type; }
    void SetDrivelineType(DrivelineTypeWV driveline_type) { m_driveline_type = driveline_type; }
    void SetTireType(TireModelType tire_type) { m_tire_type = tire_type; }

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

    void SetAerodynamicDrag(double Cd, double area, double air_density);

    void SetChassisVisualizationType(VisualizationType vis) { m_vehicle->SetChassisVisualizationType(vis); }
    void SetSuspensionVisualizationType(VisualizationType vis) { m_vehicle->SetSuspensionVisualizationType(vis); }
    void SetSteeringVisualizationType(VisualizationType vis) { m_vehicle->SetSteeringVisualizationType(vis); }
    void SetWheelVisualizationType(VisualizationType vis);
    void SetTireVisualizationType(VisualizationType vis) { m_vehicle->SetTireVisualizationType(vis); }

    void Synchronize(double time, const ChDriver::Inputs& driver_inputs, const ChTerrain& terrain);
    void Advance(double step);

    void LogHardpointLocations() { m_vehicle->LogHardpointLocations(); }
    void DebugLog(int what) { m_vehicle->DebugLog(what); }

  protected:
    ChContactMethod m_contact_method;
    CollisionType m_chassis_collision_type;
    bool m_fixed;
    bool m_brake_locking;

    DrivelineTypeWV m_driveline_type;
    BrakeType m_brake_type;
    TireModelType m_tire_type;
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
    Gator_Vehicle* m_vehicle;

    double m_tire_mass;
};

/// @} vehicle_models_gator

}  // end namespace gator
}  // end namespace vehicle
}  // end namespace chrono

#endif
