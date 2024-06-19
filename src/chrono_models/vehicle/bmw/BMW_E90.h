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
// Wrapper classes for modeling an entire BMW_E90 vehicle assembly
// (including the vehicle itself, the powertrain, and the tires).
// Vehicle Parameters taken from SAE Paper 2007-01-0818
//
// =============================================================================

#ifndef BMW_E90_H
#define BMW_E90_H

#include <array>
#include <string>

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/bmw/BMW_E90_Vehicle.h"
#include "chrono_models/vehicle/bmw/BMW_E90_AutomaticTransmissionSimpleMap.h"
#include "chrono_models/vehicle/bmw/BMW_E90_EngineSimpleMap.h"
#include "chrono_models/vehicle/bmw/BMW_E90_TMsimpleTireFront.h"
#include "chrono_models/vehicle/bmw/BMW_E90_TMsimpleTireRear.h"
#include "chrono_models/vehicle/bmw/BMW_E90_TMeasyTireFront.h"
#include "chrono_models/vehicle/bmw/BMW_E90_TMeasyTireRear.h"

namespace chrono {
namespace vehicle {
namespace bmw {

/// @addtogroup vehicle_models_bmw
/// @{

/// Definition of the BMW_E90 assembly.
/// This class encapsulates a concrete wheeled vehicle model with parameters corresponding to
/// a typical passenger, the powertrain model, and the 4 tires.
class CH_MODELS_API BMW_E90 {
  public:
    BMW_E90();
    BMW_E90(ChSystem* system);

    ~BMW_E90();

    void SetContactMethod(ChContactMethod val) { m_contactMethod = val; }
    void SetCollisionSystemType(ChCollisionSystem::Type collsys_type) { m_collsysType = collsys_type; }

    void SetChassisFixed(bool val) { m_fixed = val; }
    void SetChassisCollisionType(CollisionType val) { m_chassisCollisionType = val; }

    void SetBrakeType(BrakeType brake_type) { m_brake_type = brake_type; }
    void SetTireType(TireModelType val) { m_tireType = val; }

    void SetInitPosition(const ChCoordsys<>& pos) { m_initPos = pos; }
    void SetInitFwdVel(double fwdVel) { m_initFwdVel = fwdVel; }
    void SetInitWheelAngVel(const std::vector<double>& omega) { m_initOmega = omega; }

    void SetTireStepSize(double step_size) { m_tire_step_size = step_size; }

    void EnableBrakeLocking(bool lock) { m_brake_locking = lock; }

    ChSystem* GetSystem() const { return m_vehicle->GetSystem(); }
    ChWheeledVehicle& GetVehicle() const { return *m_vehicle; }
    std::shared_ptr<ChChassis> GetChassis() const { return m_vehicle->GetChassis(); }
    std::shared_ptr<ChBodyAuxRef> GetChassisBody() const { return m_vehicle->GetChassisBody(); }

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
    ChSystem* m_system;
    BMW_E90_Vehicle* m_vehicle;

    ChContactMethod m_contactMethod;
    ChCollisionSystem::Type m_collsysType;
    CollisionType m_chassisCollisionType;
    bool m_fixed;
    bool m_brake_locking;

    BrakeType m_brake_type;
    TireModelType m_tireType;

    double m_tire_step_size;

    ChCoordsys<> m_initPos;
    double m_initFwdVel;
    std::vector<double> m_initOmega;

    bool m_apply_drag;
    double m_Cd;
    double m_area;
    double m_air_density;

    double m_tire_mass;
};

/// @} vehicle_models_bmw

}  // namespace bmw
}  // end namespace vehicle
}  // end namespace chrono

#endif
