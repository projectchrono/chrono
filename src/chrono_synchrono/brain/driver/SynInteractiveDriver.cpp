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
// Authors: Radu Serban, Justin Madsen, Conlain Kelly, Aaron Young
// =============================================================================
//
// Interactive driver for a vehicle. This class implements the
// functionality required by its base ChDriver class using keyboard or joystick
// inputs. If a joystick is present it will use that as an input; it will
// otherwise default to a keyboard input.
//
// =============================================================================

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <climits>

#include "chrono_synchrono/brain/driver/SynInteractiveDriver.h"

#include "chrono_synchrono/simulation/SynSimulationConfig.h"

namespace chrono {
namespace synchrono {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
SynInteractiveDriver::SynInteractiveDriver(ChVehicle& vehicle)
    : ChDriver(vehicle),
      m_steering_target(0),
      m_throttle_target(0),
      m_braking_target(0),
      m_stepsize(STEP_SIZE),
      m_steering_delta(1.0 / 50),
      m_throttle_delta(1.0 / 50),
      m_braking_delta(1.0 / 50),
      m_steering_gain(4.0),
      m_throttle_gain(4.0),
      m_braking_gain(4.0),
      m_mode(KEYBOARD) {
    m_dT = 0;

    /// Activates joysticks, if available
    if (m_joystick.isFound()) {
        GetLog() << "Joystick support is enabled and a joystick is present.\n";
        m_mode = JOYSTICK;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
bool SynInteractiveDriver::Sample() {
    if (m_mode == JOYSTICK) {
        // Attempt to sample an event from the joystick
        JoystickEvent event;

        if (m_joystick.sample(&event)) {
            if (event.isButton()) {
                // printf("Button %u is %s\n", event.number, event.value == 0 ? "up" : "down");
                return true;
            } else if (event.isAxis()) {
                if (VERBOSE && event.number != 1)
                    printf("Axis %u is at position %d\n", event.number, event.value);

                if (event.number == 2) {
                    // Set new throttle value
                    double th = event.value + SHRT_MAX;
                    double new_throttle = (2 * SHRT_MAX - th) / (2 * SHRT_MAX);
                    SetThrottle(new_throttle);
                } else if (event.number == 3) {
                    // Set new braking value
                    double br = event.value + SHRT_MAX;
                    double new_braking = (2 * SHRT_MAX - br) / (2 * SHRT_MAX);
                    SetBraking(new_braking);
                } else if (event.number == 0) {
                    // Set new steering value
                    double new_steering = -(double)event.value / SHRT_MAX;
                    SetSteering(new_steering);
                }

                return true;
            }
            return false;
        }
    }

    // /// Only interpret keyboard inputs.
    // if (event.EventType != EET_KEY_INPUT_EVENT)
    //     return false;

    // if (event.KeyInput.PressedDown) {
    //     switch (event.KeyInput.Key) {
    //         case KEY_KEY_A:
    //             if (m_mode == KEYBOARD)
    //                 m_steering_target = ChClamp(m_steering_target - m_steering_delta, -1.0, +1.0);
    //             return true;
    //         case KEY_KEY_D:
    //             if (m_mode == KEYBOARD)
    //                 m_steering_target = ChClamp(m_steering_target + m_steering_delta, -1.0, +1.0);
    //             return true;
    //         case KEY_KEY_W:
    //             if (m_mode == KEYBOARD) {
    //                 m_throttle_target = ChClamp(m_throttle_target + m_throttle_delta, 0.0, +1.0);
    //                 if (m_throttle_target > 0)
    //                     m_braking_target = ChClamp(m_braking_target - m_braking_delta * 3, 0.0, +1.0);
    //             }
    //             return true;
    //         case KEY_KEY_S:
    //             if (m_mode == KEYBOARD) {
    //                 m_throttle_target = ChClamp(m_throttle_target - m_throttle_delta * 3, 0.0, +1.0);
    //                 if (m_throttle_target <= 0)
    //                     m_braking_target = ChClamp(m_braking_target + m_braking_delta, 0.0, +1.0);
    //             }
    //             return true;
    //         default:
    //             break;
    //     }
    // } else {
    //     switch (event.KeyInput.Key) {
    //         case KEY_KEY_L:
    //             m_mode = LOCK;
    //             return true;

    //         case KEY_KEY_K:
    //             m_throttle = 0;
    //             m_steering = 0;
    //             m_braking = 0;
    //             m_mode = KEYBOARD;
    //             return true;

    //         case KEY_KEY_J:
    //             if (m_data_driver) {
    //                 m_mode = DATAFILE;
    //                 m_time_shift = m_app.m_vehicle->GetSystem()->GetChTime();
    //             }
    //             return true;

    //         case KEY_KEY_Z:
    //             if (m_mode == KEYBOARD && m_app.m_vehicle->GetPowertrain())
    //                 m_app.m_vehicle->GetPowertrain()->SetDriveMode(ChPowertrain::FORWARD);
    //             return true;
    //         case KEY_KEY_X:
    //             if (m_mode == KEYBOARD && m_app.m_vehicle->GetPowertrain())
    //                 m_app.m_vehicle->GetPowertrain()->SetDriveMode(ChPowertrain::NEUTRAL);
    //             return true;
    //         case KEY_KEY_C:
    //             if (m_mode == KEYBOARD && m_app.m_vehicle->GetPowertrain())
    //                 m_app.m_vehicle->GetPowertrain()->SetDriveMode(ChPowertrain::REVERSE);
    //             return true;
    //         default:
    //             break;
    //     }
    // }

    return false;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void SynInteractiveDriver::SetThrottleDelta(double delta) {
    m_throttle_delta = delta;
}

void SynInteractiveDriver::SetSteeringDelta(double delta) {
    m_steering_delta = delta;
}

void SynInteractiveDriver::SetBrakingDelta(double delta) {
    m_braking_delta = delta;
}

void SynInteractiveDriver::SetGains(double steering_gain, double throttle_gain, double braking_gain) {
    m_steering_gain = steering_gain;
    m_throttle_gain = throttle_gain;
    m_braking_gain = braking_gain;
}

void SynInteractiveDriver::SetInputDataFile(const std::string& filename) {
    // Embed a DataDriver.
    m_data_driver = chrono_types::make_shared<ChDataDriver>(m_vehicle, filename, false);
}

void SynInteractiveDriver::SetInputMode(InputMode mode) {
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
                m_time_shift = m_vehicle.GetSystem()->GetChTime();
            }
            break;
        case JOYSTICK:
            m_mode = JOYSTICK;
            break;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void SynInteractiveDriver::Synchronize(double time) {
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
void SynInteractiveDriver::Advance(double step) {
    // Update driver inputs with either joystick or keyboard values
    Sample();

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
std::string SynInteractiveDriver::GetInputModeAsString() const {
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

}  // namespace synchrono
}  // end namespace chrono
