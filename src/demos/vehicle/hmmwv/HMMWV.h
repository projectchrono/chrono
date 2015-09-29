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

#include "chrono_vehicle/tire/ChPacejkaTire.h"

#include "ModelDefs.h"
#include "hmmwv/vehicle/HMMWV_Vehicle.h"
#include "hmmwv/vehicle/HMMWV_VehicleReduced.h"
#include "hmmwv/powertrain/HMMWV_Powertrain.h"
#include "hmmwv/powertrain/HMMWV_SimplePowertrain.h"
#include "hmmwv/tire/HMMWV_RigidTire.h"
#include "hmmwv/tire/HMMWV_LugreTire.h"
#include "hmmwv/tire/HMMWV_FialaTire.h"

namespace hmmwv {

class HMMWV {
  public:
    virtual ~HMMWV();

    void SetChassisFixed(bool val) { m_fixed = val; }

    void SetDriveType(DrivelineType val) { m_driveType = val; }
    void SetPowertrainType(PowertrainModelType val) { m_powertrainType = val; }
    void SetTireType(TireModelType val) { m_tireType = val; }

    void SetChassisVis(VisualizationType val) { m_chassisVis = val; }
    void SetwheelVis(VisualizationType val) { m_wheelVis = val; }

    void SetInitPosition(const chrono::ChCoordsys<>& pos) { m_initPos = pos; }

    void SetTireStepSize(double step_size) { m_tire_step_size = step_size; }
    void SetPacejkaParamfile(const std::string& filename) { m_pacejkaParamFile = filename; }

    chrono::ChSystem* GetSystem() const { return m_vehicle->GetSystem(); }
    chrono::ChVehicle& GetVehicle() const { return *m_vehicle; }
    chrono::ChPowertrain& GetPowertrain() const { return *m_powertrain; }

    void Initialize();

    void Update(double time,
                double steering_input,
                double braking_input,
                double throttle_input,
                const chrono::ChTerrain& terrain);

    void Advance(double step);

  protected:
    // Protected constructor -- this class cannot be instantiated by itself.
    HMMWV();

    virtual chrono::ChVehicle* CreateVehicle() = 0;

    bool m_fixed;
    VisualizationType m_chassisVis;
    VisualizationType m_wheelVis;

    DrivelineType m_driveType;
    PowertrainModelType m_powertrainType;
    TireModelType m_tireType;

    double m_tire_step_size;
    std::string m_pacejkaParamFile;

    chrono::ChCoordsys<> m_initPos;

    chrono::ChVehicle* m_vehicle;
    chrono::ChPowertrain* m_powertrain;
    chrono::ChTire* m_tireFL;
    chrono::ChTire* m_tireFR;
    chrono::ChTire* m_tireRR;
    chrono::ChTire* m_tireRL;
};

class HMMWV_Full : public HMMWV {
  public:
    HMMWV_Full() {}
    ~HMMWV_Full() {}

    void ExportMeshPovray(const std::string& out_dir) { ((HMMWV_Vehicle*)m_vehicle)->ExportMeshPovray(out_dir); }
    void LogHardpointLocations() { ((HMMWV_Vehicle*)m_vehicle)->LogHardpointLocations(); }
    void DebugLog(int what) { ((HMMWV_Vehicle*)m_vehicle)->DebugLog(what); }

  private:
    virtual chrono::ChVehicle* CreateVehicle() override {
        return new HMMWV_Vehicle(m_fixed, m_driveType, m_chassisVis, m_wheelVis);
    }
};

class HMMWV_Reduced : public HMMWV {
  public:
    HMMWV_Reduced() {}
    ~HMMWV_Reduced() {}

    void ExportMeshPovray(const std::string& out_dir) { ((HMMWV_VehicleReduced*)m_vehicle)->ExportMeshPovray(out_dir); }

  private:
    virtual chrono::ChVehicle* CreateVehicle() override {
        return new HMMWV_VehicleReduced(m_fixed, m_driveType, m_chassisVis, m_wheelVis);
    }
};

}  // end namespace hmmwv

#endif