// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2022 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke
// =============================================================================
//
// VSG-based visualization wrapper for vehicles.  This class is a derived
// from ChVisualSystemVSG and provides the following functionality:
//   - rendering of the entire Irrlicht scene
//   - custom chase-camera (which can be controlled with keyboard)
//   - optional rendering of links, springs, stats, etc.
//
// =============================================================================

#ifndef CH_VEHICLE_VISUAL_SYSTEM_VSG_H
#define CH_VEHICLE_VISUAL_SYSTEM_VSG_H

#include <string>

#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtilsChaseCamera.h"

#include "chrono_vsg/ChVisualSystemVSG.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/ChVehicleVisualSystem.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/powertrain/ChShaftsPowertrain.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"

namespace chrono {
namespace vehicle {

class ChVSGGuiDriver;

class CH_VEHICLE_API ChVehicleVisualSystemVSG : public ChVehicleVisualSystem, public vsg3d::ChVisualSystemVSG {
  public:
    /// Construct a vehicle VSG visualization system
    ChVehicleVisualSystemVSG();

    virtual ~ChVehicleVisualSystemVSG();

    virtual void Initialize() override;

    virtual void UpdateFromMBS() override;

    /// Attach a vehicle to this VSG vehicle visualization system.
    virtual void AttachVehicle(vehicle::ChVehicle* vehicle) override;

    /// Advance the dynamics of the chase camera.
    /// The integration of the underlying ODEs is performed using as many steps as needed to advance
    /// by the specified duration.
    virtual void Advance(double step) override;

    /// Update information related to driver inputs.
    virtual void Synchronize(const std::string& msg, const DriverInputs& driver_inputs) override;

    void SetTargetSymbol(double size, ChColor col);
    void SetTargetSymbolPosition(ChVector<> pos);
    void SetSentinelSymbol(double size, ChColor col);
    void SetSentinelSymbolPosition(ChVector<> pos);

    void IncreaseVehicleSpeed();
    void DecreaseVehicleSpeed();
    void SteeringLeft();
    void SteeringRight();
    void SteeringCenter();
    void ReleasePedals();

    void CameraZoom(int how);
    void CameraTurn(int how);
    void CameraRaise(int how);
    void CameraState(utils::ChChaseCamera::State state);

    void LogContraintViolations();

    virtual int GetGearPosition() override;
    virtual double GetEngineSpeedRPM() override;
    virtual double GetEngineTorque() override;
    virtual char GetTransmissionMode() override;
    virtual char GetDriveMode() override;
    virtual double GetTconvSlip() override;
    virtual double GetTconvTorqueInput() override;
    virtual double GetTconvTorqueOutput() override;
    virtual double GetTconvSpeedOutput() override;

  protected:
    void AttachGuiDriver(ChVSGGuiDriver* driver);

    ChVSGGuiDriver* m_guiDriver;

    vsg::dvec3 m_target_symbol_position = vsg::dvec3(0.0, 0.0, 0.0);
    vsg::dvec3 m_target_symbol_size = vsg::dvec3(1.0, 1.0, 1.0);
    vsg::dvec3 m_sentinel_symbol_position = vsg::dvec3(0.0, 0.0, 0.0);
    vsg::dvec3 m_sentinel_symbol_size = vsg::dvec3(1.0, 1.0, 1.0);

    friend class ChVSGGuiDriver;
};

}  // namespace vehicle
}  // namespace chrono
#endif