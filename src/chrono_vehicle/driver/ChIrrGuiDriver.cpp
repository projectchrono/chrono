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
// Authors: Radu Serban, Justin Madsen, Conlain Kelly
// =============================================================================
//
// Irrlicht-based GUI driver for the a vehicle. This class implements the
// functionality required by its base ChDriver class using keyboard or joystick
// inputs. If a joystick is present it will use that as an input; it will
// otherwise default to a keyboard input.
// As an Irrlicht event receiver, its OnEvent() callback is used to keep track
// and update the current driver inputs. As such it does not need to override
// the default no-op Advance() virtual method.
//
// =============================================================================

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <climits>

#include "chrono_vehicle/driver/ChIrrGuiDriver.h"

using namespace irr;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChIrrGuiDriver::ChIrrGuiDriver(ChVehicleVisualSystemIrrlicht& vsys)
    : ChDriver(*vsys.m_vehicle),
      m_vsys(vsys),
      m_steering_target(0),
      m_throttle_target(0),
      m_braking_target(0),
      m_stepsize(1e-3),
      m_steering_delta(1.0 / 50),
      m_throttle_delta(1.0 / 50),
      m_braking_delta(1.0 / 50),
      m_steering_gain(4.0),
      m_throttle_gain(4.0),
      m_braking_gain(4.0),
      m_mode(KEYBOARD) {
    vsys.AddUserEventReceiver(this);
    m_dT = 0;
    core::array<SJoystickInfo> joystickInfo = core::array<SJoystickInfo>();

    /// Activates joysticks, if available
    vsys.GetDevice()->activateJoysticks(joystickInfo);
    if (joystickInfo.size() > 0) {
        GetLog() << "Joystick support is enabled and " << joystickInfo.size() << " joystick(s) are present.\n";
        m_mode = JOYSTICK;
        for (u32 joystick = 0; joystick < joystickInfo.size(); ++joystick) {
            GetLog() << "Joystick " << joystick << ":\n";
            GetLog() << "\tName: '" << joystickInfo[joystick].Name.c_str() << "'\n";
            GetLog() << "\tAxes: " << joystickInfo[joystick].Axes << "\n";
            GetLog() << "\tButtons: " << joystickInfo[joystick].Buttons << "\n";

            GetLog() << "\tHat is: ";

            switch (joystickInfo[joystick].PovHat) {
                case SJoystickInfo::POV_HAT_PRESENT:
                    GetLog() << "present\n";
                    break;

                case SJoystickInfo::POV_HAT_ABSENT:
                    GetLog() << "absent\n";
                    break;

                case SJoystickInfo::POV_HAT_UNKNOWN:
                default:
                    GetLog() << "unknown\n";
                    break;
            }
        }
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
bool ChIrrGuiDriver::OnEvent(const SEvent& event) {
    
    if (m_mode == JOYSTICK) {
        /// Only handles joystick events
        if (event.EventType != EET_JOYSTICK_INPUT_EVENT)
            return false;
        /// Driver only handles input every 16 ticks (~60 Hz)
        if (m_dT < 16) {
            m_dT++;
            return true;
        }

        m_dT = 0;

        double th = event.JoystickEvent.Axis[throttle_axis] + SHRT_MAX;
        double br = event.JoystickEvent.Axis[brake_axis] + SHRT_MAX;
        double new_steering = (double)event.JoystickEvent.Axis[steer_axis] / SHRT_MAX;
        double new_throttle = (2 * SHRT_MAX - th) / (2 * SHRT_MAX);
        double new_braking = (2 * SHRT_MAX - br) / (2 * SHRT_MAX);

        if (m_steering != new_steering)
            SetSteering(new_steering);
        if (m_throttle != new_throttle)
            SetThrottle(new_throttle);
        if (m_braking != new_braking)
            SetBraking(new_braking);

        if (event.JoystickEvent.Axis[clutch_axis] != SHRT_MAX && clutch_axis!=NONE) {
            SetThrottle(0);
            if (m_vsys.m_vehicle->GetPowertrain()) {
                if (event.JoystickEvent.IsButtonPressed(22)) {
                    /// Gear is set to reverse
                    m_vsys.m_vehicle->GetPowertrain()->SetDriveMode(ChPowertrain::DriveMode::REVERSE);
                } else if (event.JoystickEvent.IsButtonPressed(12) || event.JoystickEvent.IsButtonPressed(13) ||
                           event.JoystickEvent.IsButtonPressed(14) || event.JoystickEvent.IsButtonPressed(15) ||
                           event.JoystickEvent.IsButtonPressed(16) || event.JoystickEvent.IsButtonPressed(17)) {
                    // All 'forward' gears set drive mode to forward, regardless of gear
                    m_vsys.m_vehicle->GetPowertrain()->SetDriveMode(ChPowertrain::DriveMode::FORWARD);
                } else {
                    m_vsys.m_vehicle->GetPowertrain()->SetDriveMode(ChPowertrain::DriveMode::NEUTRAL);
                }
            }
        }
        // joystick callback, outside of condition because it might be used to switch to joystick
        if (callbackButton > -1 && cb_fun != nullptr && event.JoystickEvent.IsButtonPressed(callbackButton) ) {
                cb_fun();
            }

        return true;
    }

    /// Only interpret keyboard inputs.
    if (event.EventType != EET_KEY_INPUT_EVENT)
        return false;

    if (event.KeyInput.PressedDown) {
        switch (event.KeyInput.Key) {
            case KEY_KEY_A:
                if (m_mode == KEYBOARD)
                    m_steering_target = ChClamp(m_steering_target + m_steering_delta, -1.0, +1.0);
                return true;
            case KEY_KEY_D:
                if (m_mode == KEYBOARD)
                    m_steering_target = ChClamp(m_steering_target - m_steering_delta, -1.0, +1.0);
                return true;
            case KEY_KEY_W:
                if (m_mode == KEYBOARD) {
                    m_throttle_target = ChClamp(m_throttle_target + m_throttle_delta, 0.0, +1.0);
                    if (m_throttle_target > 0)
                        m_braking_target = ChClamp(m_braking_target - m_braking_delta * 3, 0.0, +1.0);
                }
                return true;
            case KEY_KEY_S:
                if (m_mode == KEYBOARD) {
                    m_throttle_target = ChClamp(m_throttle_target - m_throttle_delta * 3, 0.0, +1.0);
                    if (m_throttle_target <= 0)
                        m_braking_target = ChClamp(m_braking_target + m_braking_delta, 0.0, +1.0);
                }
                return true;
            default:
                break;
        }
    } else {
        switch (event.KeyInput.Key) {
            case KEY_KEY_L:
                m_mode = LOCK;
                return true;

            case KEY_KEY_K:
                m_throttle = 0;
                m_steering = 0;
                m_braking = 0;
                m_mode = KEYBOARD;
                return true;

            case KEY_KEY_J:
                if (m_data_driver) {
                    m_mode = DATAFILE;
                    m_time_shift = m_vsys.m_vehicle->GetSystem()->GetChTime();
                }
                return true;

            case KEY_KEY_Z:
                if (m_mode == KEYBOARD && m_vsys.m_vehicle->GetPowertrain())
                    m_vsys.m_vehicle->GetPowertrain()->SetDriveMode(ChPowertrain::DriveMode::FORWARD);
                return true;
            case KEY_KEY_X:
                if (m_mode == KEYBOARD && m_vsys.m_vehicle->GetPowertrain())
                    m_vsys.m_vehicle->GetPowertrain()->SetDriveMode(ChPowertrain::DriveMode::NEUTRAL);
                return true;
            case KEY_KEY_C:
                if (m_mode == KEYBOARD && m_vsys.m_vehicle->GetPowertrain())
                    m_vsys.m_vehicle->GetPowertrain()->SetDriveMode(ChPowertrain::DriveMode::REVERSE);
                return true;

            case KEY_KEY_T:
                if (m_mode == KEYBOARD && m_vsys.m_vehicle->GetPowertrain()) {
                    switch (m_vsys.m_vehicle->GetPowertrain()->GetTransmissionMode()) {
                        case ChPowertrain::TransmissionMode::MANUAL:
                            m_vsys.m_vehicle->GetPowertrain()->SetTransmissionMode(
                                ChPowertrain::TransmissionMode::AUTOMATIC);
                            break;
                        case ChPowertrain::TransmissionMode::AUTOMATIC:
                            m_vsys.m_vehicle->GetPowertrain()->SetTransmissionMode(
                                ChPowertrain::TransmissionMode::MANUAL);
                            break;
                    }
                }
                return true;
            case KEY_PERIOD:
                if (m_mode == KEYBOARD && m_vsys.m_vehicle->GetPowertrain())
                    m_vsys.m_vehicle->GetPowertrain()->ShiftUp();
                return true;
            case KEY_COMMA:
                if (m_mode == KEYBOARD && m_vsys.m_vehicle->GetPowertrain())
                    m_vsys.m_vehicle->GetPowertrain()->ShiftDown();
                return true;

            default:
                break;
        }
    }

    return false;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIrrGuiDriver::SetThrottleDelta(double delta) {
    m_throttle_delta = delta;
}

void ChIrrGuiDriver::SetSteeringDelta(double delta) {
    m_steering_delta = delta;
}

void ChIrrGuiDriver::SetBrakingDelta(double delta) {
    m_braking_delta = delta;
}

void ChIrrGuiDriver::SetGains(double steering_gain, double throttle_gain, double braking_gain) {
    m_steering_gain = steering_gain;
    m_throttle_gain = throttle_gain;
    m_braking_gain = braking_gain;
}

void ChIrrGuiDriver::SetInputDataFile(const std::string& filename) {
    // Embed a DataDriver.
    m_data_driver = chrono_types::make_shared<ChDataDriver>(m_vehicle, filename, false);
}

void ChIrrGuiDriver::SetInputMode(InputMode mode) {
    switch (mode) {
        case LOCK:
            m_mode = LOCK;
            break;
        case KEYBOARD:
            m_throttle_target = 0;
            m_steering_target = 0;
            m_braking_target = 0;
            m_mode = KEYBOARD;
            break;
        case DATAFILE:
            if (m_data_driver) {
                m_mode = DATAFILE;
                m_time_shift = m_vsys.m_vehicle->GetSystem()->GetChTime();
            }
            break;
        case JOYSTICK:
            m_mode = JOYSTICK;
            break;
    }
}

void ChIrrGuiDriver::SetJoystickAxes(JoystickAxes tr_ax, JoystickAxes br_ax, JoystickAxes st_ax, JoystickAxes cl_ax){

    throttle_axis = tr_ax;
    brake_axis = br_ax; 
    steer_axis = st_ax;
    clutch_axis = cl_ax;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIrrGuiDriver::Synchronize(double time) {
    // Do nothing if no embedded DataDriver.
    if (m_mode != DATAFILE || !m_data_driver)
        return;

    // Call the update function of the embedded DataDriver, with shifted time.
    m_data_driver->Synchronize(time - m_time_shift);

    // Use inputs from embedded DataDriver
    m_throttle = m_data_driver->GetThrottle();
    m_steering = m_data_driver->GetSteering();
    m_braking = m_data_driver->GetBraking();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIrrGuiDriver::Advance(double step) {
    // Do nothing if not in KEYBOARD mode.
    if (m_mode != KEYBOARD)
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

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
std::string ChIrrGuiDriver::GetInputModeAsString() const {
    switch (m_mode) {
        case LOCK:
            return std::string("Input mode: LOCK");
        case KEYBOARD:
            return std::string("Input mode: KEY");
        case DATAFILE:
            return std::string("Input mode: FILE");
        case JOYSTICK:
            return std::string("Input mode: JOYSTICK");
    }
    return std::string("");
}

}  // end namespace vehicle
}  // end namespace chrono
