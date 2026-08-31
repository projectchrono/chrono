// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Base class and utilities for interactive vehicle drivers. This class
// implements the common functionality for a driver that accepts user inputs
// from keyboard or a joystick.
//
// =============================================================================

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <climits>
#include <bitset>

#include "chrono/utils/ChUtils.h"

#include "chrono_vehicle/driver/ChInteractiveDriver.h"
#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"

namespace chrono {
namespace vehicle {

ChInteractiveDriver::ChInteractiveDriver(ChVehicle& vehicle)
    : ChDriver(vehicle),
      m_mode(InputMode::KEYBOARD),
      m_keyboard_mode(KeyboardMode::CUMULATIVE),
      m_keyboard_mode_user_set(false),
      m_key_throttle(false),
      m_key_brake(false),
      m_key_steer_left(false),
      m_key_steer_right(false),
      m_key_clutch(false),
      m_steering_target(0),
      m_throttle_target(0),
      m_braking_target(0),
      m_clutch_target(0),
      m_stepsize(1e-3),
      m_steering_delta(1.0 / 50),
      m_throttle_delta(1.0 / 50),
      m_braking_delta(1.0 / 50),
      m_clutch_delta(1.0 / 50),
      m_steering_gain(4.0),
      m_throttle_gain(4.0),
      m_braking_gain(4.0),
      m_clutch_gain(4.0) {}

// -----------------------------------------------------------------------------

void ChInteractiveDriver::SetInputMode(InputMode mode) {
    switch (mode) {
        case InputMode::KEYBOARD:
            ReleaseAllKeys();
            m_throttle_target = 0;
            m_steering_target = 0;
            m_braking_target = 0;
            m_clutch_target = 0;
            m_mode = mode;
            break;
        case InputMode::JOYSTICK:
            if (HasJoystick())
                m_mode = mode;
            else
                std::cerr << "No joysticks available. Input mode unchanged" << std::endl;
            break;
        case InputMode::LOCK:
            m_mode = mode;
            break;
    }
}

void ChInteractiveDriver::SetKeyboardMode(KeyboardMode mode) {
    m_keyboard_mode_user_set = true;
    if (mode == m_keyboard_mode)
        return;
    ReleaseAllKeys();
    m_steering_target = 0;
    m_throttle_target = 0;
    m_braking_target = 0;
    m_clutch_target = 0;
    m_keyboard_mode = mode;
}

void ChInteractiveDriver::SetDefaultKeyboardMode(KeyboardMode mode) {
    if (m_keyboard_mode_user_set)
        return;
    bool user_set = m_keyboard_mode_user_set;
    SetKeyboardMode(mode);
    m_keyboard_mode_user_set = user_set;
}

// -----------------------------------------------------------------------------

void ChInteractiveDriver::SetKeyState(InputKey key, bool pressed) {
    switch (key) {
        case InputKey::THROTTLE:
            m_key_throttle = pressed;
            break;
        case InputKey::BRAKE:
            m_key_brake = pressed;
            break;
        case InputKey::STEER_LEFT:
            m_key_steer_left = pressed;
            break;
        case InputKey::STEER_RIGHT:
            m_key_steer_right = pressed;
            break;
        case InputKey::CLUTCH:
            m_key_clutch = pressed;
            break;
    }
}

void ChInteractiveDriver::ReleaseAllKeys() {
    m_key_throttle = false;
    m_key_brake = false;
    m_key_steer_left = false;
    m_key_steer_right = false;
    m_key_clutch = false;
}

void ChInteractiveDriver::UpdateTargetsFromHeldKeys() {
    // The brake pedal takes precedence over the accelerator pedal.
    m_throttle_target = (m_key_throttle && !m_key_brake) ? 1.0 : 0.0;
    m_braking_target = m_key_brake ? 1.0 : 0.0;

    // Steering self-centers when neither (or both) steering keys are held.
    m_steering_target = (m_key_steer_left ? 1.0 : 0.0) - (m_key_steer_right ? 1.0 : 0.0);

    m_clutch_target = m_key_clutch ? 1.0 : 0.0;
}

// -----------------------------------------------------------------------------

void ChInteractiveDriver::SetGains(double steering_gain,
                                   double throttle_gain,
                                   double braking_gain,
                                   double clutch_gain) {
    m_steering_gain = steering_gain;
    m_throttle_gain = throttle_gain;
    m_braking_gain = braking_gain;
    m_clutch_gain = clutch_gain;
}

// -----------------------------------------------------------------------------

void ChInteractiveDriver::IncreaseThrottle() {
    m_throttle_target = ChClamp(m_throttle_target + m_throttle_delta, 0.0, +1.0);
    if (m_throttle_target > 0)
        m_braking_target = ChClamp(m_braking_target - m_braking_delta * 3, 0.0, +1.0);
}

void ChInteractiveDriver::DecreaseThrottle() {
    m_throttle_target = ChClamp(m_throttle_target - m_throttle_delta * 3, 0.0, +1.0);
    if (m_throttle_target <= 0)
        m_braking_target = ChClamp(m_braking_target + m_braking_delta, 0.0, +1.0);
}

void ChInteractiveDriver::SteeringLeft() {
    m_steering_target = ChClamp(m_steering_target + m_steering_delta, -1.0, +1.0);
}

void ChInteractiveDriver::SteeringRight() {
    m_steering_target = ChClamp(m_steering_target - m_steering_delta, -1.0, +1.0);
}

void ChInteractiveDriver::IncreaseClutch() {
    m_clutch_target = ChClamp(m_clutch_target + m_clutch_delta, 0.0, +1.0);
}

void ChInteractiveDriver::DecreaseClutch() {
    m_clutch_target = ChClamp(m_clutch_target - m_clutch_delta, 0.0, +1.0);
}

void ChInteractiveDriver::SteeringCenter() {
    m_key_steer_left = false;
    m_key_steer_right = false;
    m_steering_target = 0.0;
}

void ChInteractiveDriver::ReleasePedals() {
    m_key_throttle = false;
    m_key_brake = false;
    m_key_clutch = false;
    m_throttle_target = 0.0;
    m_braking_target = 0.0;
    m_clutch_target = 0.0;
}

// -----------------------------------------------------------------------------

void ChInteractiveDriver::Advance(double step) {
    // Do nothing if not in KEYBOARD mode.
    if (m_mode != InputMode::KEYBOARD)
        return;

    // With held-key semantics, the targets are a pure function of the keys currently down.
    if (m_keyboard_mode == KeyboardMode::HELD)
        UpdateTargetsFromHeldKeys();

    // Integrate dynamics, taking as many steps as required to reach the value 'step'
    double t = 0;
    while (t < step) {
        double h = std::min<>(m_stepsize, step - t);

        double throttle_deriv = m_throttle_gain * (m_throttle_target - m_throttle);
        double steering_deriv = m_steering_gain * (m_steering_target - m_steering);
        double braking_deriv = m_braking_gain * (m_braking_target - m_braking);
        double clutch_deriv = m_clutch_gain * (m_clutch_target - m_clutch);

        SetThrottle(m_throttle + h * throttle_deriv);
        SetSteering(m_steering + h * steering_deriv);
        SetBraking(m_braking + h * braking_deriv);
        SetClutch(m_clutch + h * clutch_deriv);

        t += h;
    }
}

}  // end namespace vehicle
}  // end namespace chrono
