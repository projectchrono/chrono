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
// Wrapper classes for modeling an entire HMMWV vehicle assembly
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#ifndef HMMWV_H
#define HMMWV_H

#include <array>
#include <string>

#include "chrono_vehicle/wheeled_vehicle/tire/ChPacejkaTire.h"

#include "models/ChApiModels.h"
#include "models/vehicle/hmmwv/HMMWV_FialaTire.h"
#include "models/vehicle/hmmwv/HMMWV_LugreTire.h"
#include "models/vehicle/hmmwv/HMMWV_Powertrain.h"
#include "models/vehicle/hmmwv/HMMWV_RigidTire.h"
#include "models/vehicle/hmmwv/HMMWV_SimplePowertrain.h"
#include "models/vehicle/hmmwv/HMMWV_Vehicle.h"
#include "models/vehicle/hmmwv/HMMWV_VehicleReduced.h"

namespace chrono {
namespace vehicle {
namespace hmmwv {

class CH_MODELS_API HMMWV {
  public:
    virtual ~HMMWV();

    void SetContactMethod(ChMaterialSurfaceBase::ContactMethod val) { m_contactMethod = val; }

    void SetChassisFixed(bool val) { m_fixed = val; }

    void SetDriveType(DrivelineType val) { m_driveType = val; }
    void SetPowertrainType(PowertrainModelType val) { m_powertrainType = val; }
    void SetTireType(TireModelType val) { m_tireType = val; }

    void SetChassisVis(VisualizationType val) { m_chassisVis = val; }
    void SetWheelVis(VisualizationType val) { m_wheelVis = val; }

    void SetInitPosition(const ChCoordsys<>& pos) { m_initPos = pos; }

    void SetTireStepSize(double step_size) { m_tire_step_size = step_size; }
    void SetPacejkaParamfile(const std::string& filename) { m_pacejkaParamFile = filename; }

    ChSystem* GetSystem() const { return m_vehicle->GetSystem(); }
    ChWheeledVehicle& GetVehicle() const { return *m_vehicle; }
    std::shared_ptr<ChBodyAuxRef> GetChassis() const { return m_vehicle->GetChassis(); }
    ChPowertrain& GetPowertrain() const { return *m_powertrain; }
    ChTire& GetTire(WheelID which) { return *m_tires[which.id()]; }

    void Initialize();

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

    virtual ChWheeledVehicle* CreateVehicle() = 0;

    ChMaterialSurfaceBase::ContactMethod m_contactMethod;
    bool m_fixed;
    VisualizationType m_chassisVis;
    VisualizationType m_wheelVis;

    DrivelineType m_driveType;
    PowertrainModelType m_powertrainType;
    TireModelType m_tireType;

    double m_tire_step_size;
    std::string m_pacejkaParamFile;

    ChCoordsys<> m_initPos;

    ChSystem* m_system;
    ChWheeledVehicle* m_vehicle;
    ChPowertrain* m_powertrain;
    std::array<ChTire*, 4> m_tires;
};

class CH_MODELS_API HMMWV_Full : public HMMWV {
  public:
    HMMWV_Full() {}
    HMMWV_Full(ChSystem* system) : HMMWV(system) {}

    void ExportMeshPovray(const std::string& out_dir) { ((HMMWV_Vehicle*)m_vehicle)->ExportMeshPovray(out_dir); }
    void LogHardpointLocations() { ((HMMWV_Vehicle*)m_vehicle)->LogHardpointLocations(); }
    void DebugLog(int what) { ((HMMWV_Vehicle*)m_vehicle)->DebugLog(what); }

  private:
    virtual ChWheeledVehicle* CreateVehicle() override {
        return m_system ? new HMMWV_Vehicle(m_system, m_fixed, m_driveType, m_chassisVis, m_wheelVis)
                        : new HMMWV_Vehicle(m_fixed, m_driveType, m_chassisVis, m_wheelVis, m_contactMethod);
    }
};

class CH_MODELS_API HMMWV_Reduced : public HMMWV {
  public:
    HMMWV_Reduced() {}
    HMMWV_Reduced(ChSystem* system) : HMMWV(system) {}

    void ExportMeshPovray(const std::string& out_dir) { ((HMMWV_VehicleReduced*)m_vehicle)->ExportMeshPovray(out_dir); }

  private:
    virtual ChWheeledVehicle* CreateVehicle() override {
        return m_system ? new HMMWV_VehicleReduced(m_system, m_fixed, m_driveType, m_chassisVis, m_wheelVis)
                        : new HMMWV_VehicleReduced(m_fixed, m_driveType, m_chassisVis, m_wheelVis, m_contactMethod);
    }
};

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono

#endif
