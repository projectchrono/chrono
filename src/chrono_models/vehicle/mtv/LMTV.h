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
// Wrapper classes for modeling an entire LMTV vehicle assembly
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#ifndef LMTV_H
#define LMTV_H

#include <array>
#include <string>

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/mtv/LMTV_Vehicle.h"
#include "chrono_models/vehicle/mtv/FMTV_SimpleCVTPowertrain.h"
#include "chrono_models/vehicle/mtv/FMTV_SimpleMapPowertrain.h"
#include "chrono_models/vehicle/mtv/FMTV_SimplePowertrain.h"
#include "chrono_models/vehicle/mtv/FMTV_Powertrain.h"
#include "chrono_models/vehicle/mtv/FMTV_RigidTire.h"
#include "chrono_models/vehicle/mtv/FMTV_TMeasyTire.h"

namespace chrono {
namespace vehicle {
namespace fmtv {

/// @addtogroup vehicle_models_fmtv
/// @{

/// Definition of the LMTV assembly.
/// This class encapsulates a concrete wheeled vehicle model with parameters corresponding to
/// an LMTV vehicle, the powertrain model, and the 4 tires.
class CH_MODELS_API LMTV {
  public:
    LMTV();
    LMTV(ChSystem* system);

    ~LMTV();

    void SetContactMethod(ChContactMethod val) { m_contactMethod = val; }

    void SetChassisFixed(bool val) { m_fixed = val; }
    void SetChassisCollisionType(CollisionType val) { m_chassisCollisionType = val; }

     void SetBrakeType(BrakeType brake_type) { m_brake_type = brake_type; }
    void SetTireType(TireModelType val) { m_tireType = val; }
    void SetPowertrainType(PowertrainModelType val) { m_powertrainType = val; }

    // void setSteeringType(SteeringTypeWV val) { m_steeringType = val; }

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
    void SetChassisRearVisualizationType(VisualizationType vis) { m_vehicle->SetChassisRearVisualizationType(vis); }
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
    bool m_brake_locking;

    BrakeType m_brake_type;
    TireModelType m_tireType;
    PowertrainModelType m_powertrainType;

    double m_tire_step_size;

    SteeringTypeWV m_steeringType;

    ChCoordsys<> m_initPos;
    double m_initFwdVel;
    std::vector<double> m_initOmega;

    bool m_apply_drag;
    double m_Cd;
    double m_area;
    double m_air_density;

    ChSystem* m_system;
    LMTV_Vehicle* m_vehicle;

    double m_tire_mass;
};

/// @} vehicle_models_fmtv

}  // namespace fmtv
}  // end namespace vehicle
}  // end namespace chrono

#endif
