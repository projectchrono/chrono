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
// Authors: Radu Serban
// =============================================================================
//
// Base class for a vehicle visualization system
//
// =============================================================================

#include "chrono_vehicle/ChVehicleVisualSystem.h"
#include "chrono_vehicle/ChWorldFrame.h"

namespace chrono {
namespace vehicle {

ChVehicleVisualSystem::ChVehicleVisualSystem()
    : m_vehicle(nullptr),
      m_stepsize(1e-3),
      m_camera_point(ChVector<>(1, 0, 0)),
      m_camera_dist(5.0),
      m_camera_height(0.5),
      m_camera_state(utils::ChChaseCamera::State::Chase),
      m_camera_pos(VNULL),
      m_camera_angle(0),
      m_camera_minMult(0.5),
      m_camera_maxMult(10),
      m_steering(0),
      m_throttle(0),
      m_braking(0) {}

ChVehicleVisualSystem ::~ChVehicleVisualSystem() {}

void ChVehicleVisualSystem::AttachVehicle(ChVehicle* vehicle) {
    m_vehicle = vehicle;

    // Attach the vehicle's Chrono system to the visualization system
    //// RADU TOOD
    //// Is this good enough for cases where the vehicle does NOT own the system?
    if (m_systems.empty())
        AttachSystem(vehicle->GetSystem());

    // Create a vehicle chase-cam and associate it with the vehicle
    // Attention: order of calls is important here!
    m_camera = chrono_types::make_unique<utils::ChChaseCamera>(m_vehicle->GetChassisBody());
    m_camera->SetCameraPos(m_camera_pos);
    m_camera->SetCameraAngle(m_camera_angle);
    m_camera->SetState(m_camera_state);
    m_camera->Initialize(m_camera_point, m_vehicle->GetChassis()->GetLocalDriverCoordsys(), m_camera_dist,
                         m_camera_height, ChWorldFrame::Vertical(), ChWorldFrame::Forward());
    m_camera->SetMultLimits(m_camera_minMult, m_camera_maxMult);
}

double ChVehicleVisualSystem::GetSimulationRTF() const {
    if (!m_vehicle)
        return 0;
    return m_vehicle->GetRTF();
}

void ChVehicleVisualSystem::Synchronize(double time, const DriverInputs& driver_inputs) {
    m_steering = driver_inputs.m_steering;
    m_throttle = driver_inputs.m_throttle;
    m_braking = driver_inputs.m_braking;
    m_clutch = driver_inputs.m_clutch;
}

void ChVehicleVisualSystem::SetChaseCamera(const ChVector<>& ptOnChassis, double chaseDist, double chaseHeight) {
    m_camera_point = ptOnChassis;
    m_camera_dist = chaseDist;
    m_camera_height = chaseHeight;
    if (m_camera) {
        m_camera->SetTargetPoint(m_camera_point);
        m_camera->SetChaseDistance(m_camera_dist);
        m_camera->SetChaseHeight(m_camera_height);
    }
}
void ChVehicleVisualSystem::SetStepsize(double val) {
    m_stepsize = val;
}
void ChVehicleVisualSystem::SetChaseCameraState(utils::ChChaseCamera::State state) {
    m_camera_state = state;
    if (m_camera)
        m_camera->SetState(m_camera_state);
}
void ChVehicleVisualSystem::SetChaseCameraPosition(const ChVector<>& pos) {
    m_camera_pos = pos;
    if (m_camera)
        m_camera->SetCameraPos(m_camera_pos);
}
void ChVehicleVisualSystem::SetChaseCameraAngle(double angle) {
    m_camera_angle = angle;
    if (m_camera)
        m_camera->SetCameraAngle(m_camera_angle);
}
void ChVehicleVisualSystem::SetChaseCameraMultipliers(double minMult, double maxMult) {
    m_camera_minMult = minMult;
    m_camera_maxMult = maxMult;
    if (m_camera)
        m_camera->SetMultLimits(m_camera_minMult, m_camera_maxMult);
}

}  // namespace vehicle
}  // namespace chrono
