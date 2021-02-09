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
// Wrapper classes for modeling an entire M113 vehicle assembly
// (including the vehicle itself and the powertrain).
//
// =============================================================================

#ifndef M113_H
#define M113_H

#include <array>
#include <string>

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/m113/M113_Vehicle.h"

namespace chrono {
namespace vehicle {
namespace m113 {

/// @addtogroup vehicle_models_m113
/// @{

/// Definition of the m113 assembly.
/// This class encapsulates a concrete tracked vehicle model with parameters corresponding to
/// a typical m113 and the powertrain model.
class CH_MODELS_API M113 {
  public:
    M113();
    M113(ChSystem* system);

    ~M113();

    void SetContactMethod(ChContactMethod val) { m_contactMethod = val; }

    void SetChassisFixed(bool val) { m_fixed = val; }
    void SetChassisCollisionType(CollisionType val) { m_chassisCollisionType = val; }

    void SetBrakeType(BrakeType brake_type) { m_brake_type = brake_type; }
    void SetTrackShoeType(TrackShoeType shoe_type) { m_shoe_type = shoe_type; }

    void SetInitPosition(const ChCoordsys<>& pos) { m_initPos = pos; }
    void SetInitFwdVel(double fwdVel) { m_initFwdVel = fwdVel; }

    ChSystem* GetSystem() const { return m_vehicle->GetSystem(); }
    ChTrackedVehicle& GetVehicle() const { return *m_vehicle; }
    std::shared_ptr<ChChassis> GetChassis() const { return m_vehicle->GetChassis(); }
    std::shared_ptr<ChBodyAuxRef> GetChassisBody() const { return m_vehicle->GetChassisBody(); }
    std::shared_ptr<ChPowertrain> GetPowertrain() const { return m_vehicle->GetPowertrain(); }
    double GetTotalMass() const;

    void Initialize();

    void SetAerodynamicDrag(double Cd, double area, double air_density);

    void SetChassisVisualizationType(VisualizationType vis) { m_vehicle->SetChassisVisualizationType(vis); }
    void SetSprocketVisualizationType(VisualizationType vis) { m_vehicle->SetSprocketVisualizationType(vis); }
    void SetIdlerVisualizationType(VisualizationType vis) { m_vehicle->SetIdlerVisualizationType(vis); }
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

    BrakeType m_brake_type;
    TrackShoeType m_shoe_type;

    ChCoordsys<> m_initPos;
    double m_initFwdVel;

    bool m_apply_drag;
    double m_Cd;
    double m_area;
    double m_air_density;

    ChSystem* m_system;
    M113_Vehicle* m_vehicle;
};

/// @} vehicle_models_sedan

}  // namespace m113
}  // end namespace vehicle
}  // end namespace chrono

#endif
