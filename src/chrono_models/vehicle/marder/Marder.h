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
// Authors: Rainer Gericke
// =============================================================================
//
// Wrapper classes for modeling an entire Marder vehicle assembly
// (including the vehicle itself and the powertrain).
//
// =============================================================================

#ifndef MARDER_H
#define MARDER_H

#include <array>
#include <string>

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/marder/Marder_Vehicle.h"

namespace chrono {
namespace vehicle {
namespace marder {

/// @addtogroup vehicle_models_marder
/// @{

/// Definition of the marder assembly.
/// This class encapsulates a concrete tracked vehicle model with parameters corresponding to
/// a typical marder and the powertrain model.
class CH_MODELS_API Marder {
  public:
    Marder();
    Marder(ChSystem* system);

    ~Marder();

    void SetContactMethod(ChContactMethod val) { m_contactMethod = val; }

    void SetChassisFixed(bool val) { m_fixed = val; }
    void SetChassisCollisionType(CollisionType val) { m_chassisCollisionType = val; }
    void SetWheelCollisionType(bool roadwheel_as_cylinder, bool idler_as_cylinder, bool roller_as_cylinder) {
        m_wheel_cyl = roadwheel_as_cylinder;
        m_idler_cyl = idler_as_cylinder;
        m_roller_cyl = roller_as_cylinder;
    }

    void SetBrakeType(BrakeType brake_type) { m_brake_type = brake_type; }
    ////void SetTrackShoeType(TrackShoeType shoe_type) { m_shoe_type = shoe_type; }
    ////void SetDrivelineType(DrivelineTypeTV driveline_type) { m_driveline_type = driveline_type; }
    void SetPowertrainType(PowertrainModelType powertrain_type) { m_powertrain_type = powertrain_type; }

    void SetInitPosition(const ChCoordsys<>& pos) { m_initPos = pos; }
    void SetInitFwdVel(double fwdVel) { m_initFwdVel = fwdVel; }

    void SetCollisionSystemType(collision::ChCollisionSystemType collsys_type) { m_collsys_type = collsys_type; }

    void CreateTrack(bool val) { m_create_track = val; }

    ChSystem* GetSystem() const { return m_vehicle->GetSystem(); }
    ChTrackedVehicle& GetVehicle() const { return *m_vehicle; }
    std::shared_ptr<ChChassis> GetChassis() const { return m_vehicle->GetChassis(); }
    std::shared_ptr<ChBodyAuxRef> GetChassisBody() const { return m_vehicle->GetChassisBody(); }
    std::shared_ptr<ChDrivelineTV> GetDriveline() const { return m_vehicle->GetDriveline(); }
    std::shared_ptr<ChPowertrain> GetPowertrain() const { return m_vehicle->GetPowertrain(); }

    void Initialize();

    void SetAerodynamicDrag(double Cd, double area, double air_density);

    void SetChassisVisualizationType(VisualizationType vis) { m_vehicle->SetChassisVisualizationType(vis); }
    void SetSprocketVisualizationType(VisualizationType vis) { m_vehicle->SetSprocketVisualizationType(vis); }
    void SetIdlerVisualizationType(VisualizationType vis) { m_vehicle->SetIdlerVisualizationType(vis); }
    void SetRollerVisualizationType(VisualizationType vis) { m_vehicle->SetRollerVisualizationType(vis); }
    void SetRoadWheelAssemblyVisualizationType(VisualizationType vis) {
        m_vehicle->SetRoadWheelAssemblyVisualizationType(vis);
    }
    void SetRoadWheelVisualizationType(VisualizationType vis) { m_vehicle->SetRoadWheelVisualizationType(vis); }
    void SetTrackShoeVisualizationType(VisualizationType vis) { m_vehicle->SetTrackShoeVisualizationType(vis); }

    void Synchronize(double time,
                     const ChDriver::Inputs& driver_inputs,
                     const TerrainForces& shoe_forces_left,
                     const TerrainForces& shoe_forces_right);
    void Advance(double step);

    void LogConstraintViolations() { m_vehicle->LogConstraintViolations(); }

  protected:
    ChContactMethod m_contactMethod;
    CollisionType m_chassisCollisionType;
    bool m_fixed;
    bool m_create_track;
    bool m_wheel_cyl;
    bool m_idler_cyl;
    bool m_roller_cyl;

    collision::ChCollisionSystemType m_collsys_type;

    BrakeType m_brake_type;
    TrackShoeType m_shoe_type;
    DrivelineTypeTV m_driveline_type;
    PowertrainModelType m_powertrain_type;

    ChCoordsys<> m_initPos;
    double m_initFwdVel;

    bool m_apply_drag;
    double m_Cd;
    double m_area;
    double m_air_density;

    ChSystem* m_system;
    Marder_Vehicle* m_vehicle;
};

/// @} vehicle_models_marder

}  // namespace marder
}  // end namespace vehicle
}  // end namespace chrono

#endif
