// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Irrlicht-based visualization wrapper for vehicles.  This class is a derived
// from ChVisualSystemIrrlicht and provides the following functionality:
//   - rendering of the entire Irrlicht scene
//   - custom chase-camera (which can be controlled with keyboard)
//   - optional rendering of links, springs, stats, etc.
//
// =============================================================================

#include "ChVehicleVisualSystemVSG.h"

namespace chrono {
namespace vehicle {

ChVehicleVisualSystemVSG::ChVehicleVisualSystemVSG() {
    m_params->showVehicleState = true;
}

void ChVehicleVisualSystemVSG::Initialize() {
    ChVisualSystemVSG::Initialize();
}

void ChVehicleVisualSystemVSG::Synchronize(const std::string& msg, const DriverInputs& driver_inputs) {
    ChVehicleVisualSystem::Synchronize(msg, driver_inputs);
    if (m_vehicle) {
        m_params->vehicleSpeed = m_vehicle->GetSpeed();
        m_params->steering = driver_inputs.m_steering;
        m_params->throttle = driver_inputs.m_throttle;
        m_params->braking = driver_inputs.m_braking;
    }
}

// -----------------------------------------------------------------------------
// Advance the dynamics of the chase camera.
// The integration of the underlying ODEs is performed using as many steps as
// needed to advance by the specified duration.
// -----------------------------------------------------------------------------
void ChVehicleVisualSystemVSG::Advance(double step) {
    // Update the ChChaseCamera: take as many integration steps as needed to
    // exactly reach the value 'step'
    double t = 0;
    while (t < step) {
        double h = std::min<>(m_stepsize, step - t);
        m_camera->Update(h);
        t += h;
    }

    // Update the Irrlicht camera
    ChVector<> cam_pos = m_camera->GetCameraPos();
    ChVector<> cam_target = m_camera->GetTargetPos();

    // apply camera setting to VSG app
    // GetActiveCamera()->setPosition(core::vector3dfCH(cam_pos));
    // GetActiveCamera()->setTarget(core::vector3dfCH(cam_target));
    m_vsg_cameraEye.set(cam_pos.x(), cam_pos.y(), cam_pos.z());
    m_vsg_cameraTarget.set(cam_target.x(), cam_target.y(), cam_target.z());
    m_lookAt->eye = m_vsg_cameraEye;
    m_lookAt->center = m_vsg_cameraTarget;
}

}  // end namespace vehicle
}  // end namespace chrono
