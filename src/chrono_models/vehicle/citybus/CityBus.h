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
// Authors: Radu Serban, Asher Elmquist, Evan Hoerl, Shuo He
// =============================================================================
//
// Wrapper classes for modeling an entire CityBus vehicle assembly
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#ifndef CITYBUS_H
#define CITYBUS_H

#include <array>
#include <string>

#include "chrono_vehicle/wheeled_vehicle/tire/ChPacejkaTire.h"

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/citybus/CityBus_Vehicle.h"
#include "chrono_models/vehicle/citybus/CityBus_SimpleMapPowertrain.h"
#include "chrono_models/vehicle/citybus/CityBus_RigidTire.h"
#include "chrono_models/vehicle/citybus/CityBus_TMeasyTire.h"

namespace chrono {
namespace vehicle {
namespace citybus {

class CH_MODELS_API CityBus {
  public:
    CityBus();
    CityBus(ChSystem* system);

    ~CityBus();

    void SetContactMethod(ChMaterialSurface::ContactMethod val) { m_contactMethod = val; }

    void SetChassisFixed(bool val) { m_fixed = val; }
    void SetChassisCollisionType(ChassisCollisionType val) { m_chassisCollisionType = val; }

    void SetTireType(TireModelType val) { m_tireType = val; }

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

    void LogHardpointLocations() { m_vehicle->LogHardpointLocations(); }
    void DebugLog(int what) { m_vehicle->DebugLog(what); }

  protected:
    ChMaterialSurface::ContactMethod m_contactMethod;
    ChassisCollisionType m_chassisCollisionType;
    bool m_fixed;

    TireModelType m_tireType;

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
    CityBus_Vehicle* m_vehicle;
    ChPowertrain* m_powertrain;
    std::array<ChTire*, 4> m_tires;

    double m_tire_mass;
};

}  // end namespace citybus
}  // end namespace vehicle
}  // end namespace chrono

#endif
