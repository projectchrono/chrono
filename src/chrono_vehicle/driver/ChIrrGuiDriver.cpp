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
// Authors: Radu Serban, Justin Madsen, Conlain Kelly
// =============================================================================
//
// Irrlicht-based GUI driver for the a vehicle. This class implements the
// functionality required by its base ChDriver class using keyboard or joystick
// inputs. If a joystick is present it will use use that as an input; it will
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
ChIrrGuiDriver::ChIrrGuiDriver(ChVehicleIrrApp& app)
    : ChDriver(*app.m_vehicle),
      m_app(app),
      m_throttleDelta(1.0 / 50),
      m_steeringDelta(1.0 / 50),
      m_brakingDelta(1.0 / 50),
      m_mode(KEYBOARD) {
    app.SetUserEventReceiver(this);
    m_dT = 0;
    core::array<SJoystickInfo> joystickInfo = core::array<SJoystickInfo>();

    /// Activates joysticks, if available
    app.GetDevice()->activateJoysticks(joystickInfo);
    if (joystickInfo.size() > 0) {
        std::cout << "Joystick support is enabled and " << joystickInfo.size() << " joystick(s) are present."
                  << std::endl;
        m_mode = JOYSTICK;
        for (u32 joystick = 0; joystick < joystickInfo.size(); ++joystick) {
            std::cout << "Joystick " << joystick << ":" << std::endl;
            std::cout << "\tName: '" << joystickInfo[joystick].Name.c_str() << "'" << std::endl;
            std::cout << "\tAxes: " << joystickInfo[joystick].Axes << std::endl;
            std::cout << "\tButtons: " << joystickInfo[joystick].Buttons << std::endl;

            std::cout << "\tHat is: ";

            switch (joystickInfo[joystick].PovHat) {
                case SJoystickInfo::POV_HAT_PRESENT:
                    std::cout << "present" << std::endl;
                    break;

                case SJoystickInfo::POV_HAT_ABSENT:
                    std::cout << "absent" << std::endl;
                    break;

                case SJoystickInfo::POV_HAT_UNKNOWN:
                default:
                    std::cout << "unknown" << std::endl;
                    break;
            }
        }
    } else {
        std::cout << "Joystick support is not enabled." << std::endl;
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

        double th = event.JoystickEvent.Axis[SEvent::SJoystickEvent::AXIS_Z] + SHRT_MAX;
        double br = event.JoystickEvent.Axis[SEvent::SJoystickEvent::AXIS_R] + SHRT_MAX;
        double new_steering = (double)event.JoystickEvent.Axis[SEvent::SJoystickEvent::AXIS_X] / SHRT_MAX;
        double new_throttle = (2 * SHRT_MAX - th) / (2 * SHRT_MAX);
        double new_braking = (2 * SHRT_MAX - br) / (2 * SHRT_MAX);

        if (m_steering != new_steering)
            SetSteering(new_steering);
        if (m_throttle != new_throttle)
            SetThrottle(new_throttle);
        if (m_braking != new_braking)
            SetBraking(new_braking);

        if (event.JoystickEvent.Axis[SEvent::SJoystickEvent::AXIS_Y] != SHRT_MAX) {
            SetThrottle(0);
            /// Gear is set to reverse
            if (event.JoystickEvent.IsButtonPressed(22)) {
                m_app.m_powertrain->SetDriveMode(ChPowertrain::REVERSE);

            } else if (event.JoystickEvent.IsButtonPressed(12) || event.JoystickEvent.IsButtonPressed(13) ||
                       event.JoystickEvent.IsButtonPressed(14) || event.JoystickEvent.IsButtonPressed(15) ||
                       event.JoystickEvent.IsButtonPressed(16) || event.JoystickEvent.IsButtonPressed(17)) {
                // All 'forward' gears set drive mode to forward, regardless of gear
                m_app.m_powertrain->SetDriveMode(ChPowertrain::FORWARD);
            } else {
                m_app.m_powertrain->SetDriveMode(ChPowertrain::NEUTRAL);
            }
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
                    SetSteering(m_steering - m_steeringDelta);
                return true;
            case KEY_KEY_D:
                if (m_mode == KEYBOARD)
                    SetSteering(m_steering + m_steeringDelta);
                return true;
            case KEY_KEY_W:
                if (m_mode == KEYBOARD) {
                    SetThrottle(m_throttle + m_throttleDelta);
                    if (m_throttle > 0)
                        SetBraking(m_braking - m_brakingDelta * 3.0);
                }
                return true;
            case KEY_KEY_S:
                if (m_mode == KEYBOARD) {
                    SetThrottle(m_throttle - m_throttleDelta * 3.0);
                    if (m_throttle <= 0)
                        SetBraking(m_braking + m_brakingDelta);
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
                    m_time_shift = m_app.m_vehicle->GetSystem()->GetChTime();
                }
                return true;

            case KEY_KEY_Z:
                if (m_mode == KEYBOARD)
                    m_app.m_powertrain->SetDriveMode(ChPowertrain::FORWARD);
                return true;
            case KEY_KEY_X:
                if (m_mode == KEYBOARD)
                    m_app.m_powertrain->SetDriveMode(ChPowertrain::NEUTRAL);
                return true;
            case KEY_KEY_C:
                if (m_mode == KEYBOARD)
                    m_app.m_powertrain->SetDriveMode(ChPowertrain::REVERSE);
                return true;
            default:
                break;
        }
    }

    return false;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIrrGuiDriver::SetInputDataFile(const std::string& filename) {
    // Embed a DataDriver.
    m_data_driver = std::make_shared<ChDataDriver>(m_vehicle, filename, false);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIrrGuiDriver::SetInputMode(InputMode mode) {
    switch (mode) {
        case LOCK:
            m_mode = LOCK;
            break;
        case KEYBOARD:
            m_throttle = 0;
            m_steering = 0;
            m_braking = 0;
            m_mode = KEYBOARD;
            break;
        case DATAFILE:
            if (m_data_driver) {
                m_mode = DATAFILE;
                m_time_shift = m_app.m_vehicle->GetSystem()->GetChTime();
            }
            break;
        case JOYSTICK:
            m_mode = JOYSTICK;
            break;
    }
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
