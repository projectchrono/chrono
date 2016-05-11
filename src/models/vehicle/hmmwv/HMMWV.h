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

#include <string>

#include "chrono_vehicle/wheeled_vehicle/tire/ChPacejkaTire.h"

#include "models/ChApiModels.h"
#include "models/vehicle/hmmwv/HMMWV_Vehicle.h"
#include "models/vehicle/hmmwv/HMMWV_VehicleReduced.h"
#include "models/vehicle/hmmwv/HMMWV_Powertrain.h"
#include "models/vehicle/hmmwv/HMMWV_SimplePowertrain.h"
#include "models/vehicle/hmmwv/HMMWV_RigidTire.h"
#include "models/vehicle/hmmwv/HMMWV_LugreTire.h"
#include "models/vehicle/hmmwv/HMMWV_FialaTire.h"

namespace hmmwv {

class CH_MODELS_API HMMWV {
  public:
    virtual ~HMMWV();

    void SetContactMethod(chrono::ChMaterialSurfaceBase::ContactMethod val) { m_contactMethod = val; }

    void SetChassisFixed(bool val) { m_fixed = val; }

    void SetDriveType(chrono::vehicle::DrivelineType val) { m_driveType = val; }
    void SetPowertrainType(chrono::vehicle::PowertrainModelType val) { m_powertrainType = val; }
    void SetTireType(chrono::vehicle::TireModelType val) { m_tireType = val; }

    void SetChassisVis(chrono::vehicle::VisualizationType val) { m_chassisVis = val; }
    void SetWheelVis(chrono::vehicle::VisualizationType val) { m_wheelVis = val; }

    void SetInitPosition(const chrono::ChCoordsys<>& pos) { m_initPos = pos; }

    void SetTireStepSize(double step_size) { m_tire_step_size = step_size; }
    void SetPacejkaParamfile(const std::string& filename) { m_pacejkaParamFile = filename; }

    chrono::ChSystem* GetSystem() const { return m_vehicle->GetSystem(); }
    chrono::vehicle::ChWheeledVehicle& GetVehicle() const { return *m_vehicle; }
    chrono::vehicle::ChPowertrain& GetPowertrain() const { return *m_powertrain; }

    void Initialize();

    void Synchronize(double time,
                     double steering_input,
                     double braking_input,
                     double throttle_input,
                     const chrono::vehicle::ChTerrain& terrain);

    void Advance(double step);

  protected:
    // Protected constructors -- this class cannot be instantiated by itself.
    HMMWV();
    HMMWV(chrono::ChSystem* system);

    virtual chrono::vehicle::ChWheeledVehicle* CreateVehicle() = 0;

    chrono::ChMaterialSurfaceBase::ContactMethod m_contactMethod;
    bool m_fixed;
    chrono::vehicle::VisualizationType m_chassisVis;
    chrono::vehicle::VisualizationType m_wheelVis;

    chrono::vehicle::DrivelineType m_driveType;
    chrono::vehicle::PowertrainModelType m_powertrainType;
    chrono::vehicle::TireModelType m_tireType;

    double m_tire_step_size;
    std::string m_pacejkaParamFile;

    chrono::ChCoordsys<> m_initPos;

    chrono::ChSystem* m_system;
    chrono::vehicle::ChWheeledVehicle* m_vehicle;
    chrono::vehicle::ChPowertrain* m_powertrain;
    chrono::vehicle::ChTire* m_tireFL;
    chrono::vehicle::ChTire* m_tireFR;
    chrono::vehicle::ChTire* m_tireRR;
    chrono::vehicle::ChTire* m_tireRL;
};

class CH_MODELS_API HMMWV_Full : public HMMWV {
  public:
    HMMWV_Full() {}
    HMMWV_Full(chrono::ChSystem* system) : HMMWV(system) {}

    void ExportMeshPovray(const std::string& out_dir) { ((HMMWV_Vehicle*)m_vehicle)->ExportMeshPovray(out_dir); }
    void LogHardpointLocations() { ((HMMWV_Vehicle*)m_vehicle)->LogHardpointLocations(); }
    void DebugLog(int what) { ((HMMWV_Vehicle*)m_vehicle)->DebugLog(what); }

  private:
    virtual chrono::vehicle::ChWheeledVehicle* CreateVehicle() override {
        return m_system ? new HMMWV_Vehicle(m_system, m_fixed, m_driveType, m_chassisVis, m_wheelVis)
                        : new HMMWV_Vehicle(m_fixed, m_driveType, m_chassisVis, m_wheelVis, m_contactMethod);
    }
};

class CH_MODELS_API HMMWV_Reduced : public HMMWV {
  public:
    HMMWV_Reduced() {}
    HMMWV_Reduced(chrono::ChSystem* system) : HMMWV(system) {}

    void ExportMeshPovray(const std::string& out_dir) { ((HMMWV_VehicleReduced*)m_vehicle)->ExportMeshPovray(out_dir); }

  private:
    virtual chrono::vehicle::ChWheeledVehicle* CreateVehicle() override {
        return m_system ? new HMMWV_VehicleReduced(m_system, m_fixed, m_driveType, m_chassisVis, m_wheelVis)
                        : new HMMWV_VehicleReduced(m_fixed, m_driveType, m_chassisVis, m_wheelVis, m_contactMethod);
    }
};

}  // end namespace hmmwv

#endif
