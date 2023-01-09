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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// VSG-based GUI driver for the a vehicle. This class implements the
// functionality required by its base ChDriver class using keyboard or joystick
// inputs.
// =============================================================================

#include "ChVSGGuiDriver.h"

namespace chrono {
namespace vehicle {
ChVSGGuiDriver::ChVSGGuiDriver(ChVehicleVisualSystemVSG& vsys)
    : ChDriver(*vsys.m_vehicle),
      m_mode(InputMode::KEYBOARD),
      m_steering_target(0),
      m_throttle_target(0),
      m_braking_target(0),
      m_stepsize(1e-3),
      m_steering_delta(1.0 / 50),
      m_throttle_delta(1.0 / 50),
      m_braking_delta(1.0 / 50),
      m_steering_gain(4.0),
      m_throttle_gain(4.0),
      m_braking_gain(4.0) {
    vsys.AttachGuiDriver(this);
}

ChVSGGuiDriver::~ChVSGGuiDriver() {}

void ChVSGGuiDriver::Initialize() {}

void ChVSGGuiDriver::Synchronize(double time) {
    // Do nothing if no embedded DataDriver.
    if (m_mode != InputMode::DATAFILE || !m_data_driver)
        return;

    // Call the update function of the embedded DataDriver, with shifted time.
    m_data_driver->Synchronize(time - m_time_shift);

    // Use inputs from embedded DataDriver
    m_throttle = m_data_driver->GetThrottle();
    m_steering = m_data_driver->GetSteering();
    m_braking = m_data_driver->GetBraking();
}

void ChVSGGuiDriver::Advance(double step) {
    // Do nothing if not in KEYBOARD mode.
    if (m_mode != InputMode::KEYBOARD)
        return;

    // Integrate dynamics, taking as many steps as required to reach the value 'step'
    double t = 0;
    while (t < step) {
        double h = std::min<>(m_stepsize, step - t);

        double throttle_deriv = m_throttle_gain * (m_throttle_target - m_throttle);
        double steering_deriv = m_steering_gain * (m_steering_target - m_steering);
        double braking_deriv = m_braking_gain * (m_braking_target - m_braking);

        SetThrottle(m_throttle + h * throttle_deriv);
        SetSteering(m_steering + h * steering_deriv);
        SetBraking(m_braking + h * braking_deriv);

        t += h;
    }
}

void ChVSGGuiDriver::SetInputMode(InputMode mode) {
    switch (mode) {
        case InputMode::KEYBOARD:
            m_throttle_target = 0;
            m_steering_target = 0;
            m_braking_target = 0;
            m_mode = mode;
            break;
        case InputMode::DATAFILE:
            if (m_data_driver) {
                m_time_shift = m_vehicle.GetSystem()->GetChTime();
                m_mode = mode;
            }
            break;
        case InputMode::LOCK:
            m_mode = mode;
            break;
        default:
            break;
    }
}

std::string ChVSGGuiDriver::GetInputModeAsString() const {
    switch (m_mode) {
        case InputMode::LOCK:
            return std::string("Input mode: LOCK");
        case InputMode::KEYBOARD:
            return std::string("Input mode: KEY");
        case InputMode::DATAFILE:
            return std::string("Input mode: FILE");
    }
    return std::string("");
}

void ChVSGGuiDriver::SetThrottleDelta(double delta) {
    m_throttle_delta = delta;
}

void ChVSGGuiDriver::SetSteeringDelta(double delta) {
    m_steering_delta = delta;
}

void ChVSGGuiDriver::SetBrakingDelta(double delta) {
    m_braking_delta = delta;
}

void ChVSGGuiDriver::SetGains(double steering_gain, double throttle_gain, double braking_gain) {
    m_steering_gain = steering_gain;
    m_throttle_gain = throttle_gain;
    m_braking_gain = braking_gain;
}

void ChVSGGuiDriver::SetInputDataFile(const std::string& filename) {
    // Embed a DataDriver.
    m_data_driver = chrono_types::make_shared<ChDataDriver>(m_vehicle, filename, false);
}

void ChVSGGuiDriver::IncreaseThrottle() {
    m_throttle_target = ChClamp(m_throttle_target + m_throttle_delta, 0.0, +1.0);
    if (m_throttle_target > 0)
        m_braking_target = ChClamp(m_braking_target - m_braking_delta * 3, 0.0, +1.0);
}

void ChVSGGuiDriver::DecreaseThrottle() {
    m_throttle_target = ChClamp(m_throttle_target - m_throttle_delta * 3, 0.0, +1.0);
    if (m_throttle_target <= 0)
        m_braking_target = ChClamp(m_braking_target + m_braking_delta, 0.0, +1.0);
}

void ChVSGGuiDriver::SteeringLeft() {
    m_steering_target = ChClamp(m_steering_target + m_steering_delta, -1.0, +1.0);
}

void ChVSGGuiDriver::SteeringRight() {
    m_steering_target = ChClamp(m_steering_target - m_steering_delta, -1.0, +1.0);
}

void ChVSGGuiDriver::SteeringCenter() {
    m_steering_target = 0.0;
}

void ChVSGGuiDriver::ReleasePedals() {
    m_throttle_target = 0.0;
    m_braking_target = 0.0;
}

}  // namespace vehicle
}  // namespace chrono