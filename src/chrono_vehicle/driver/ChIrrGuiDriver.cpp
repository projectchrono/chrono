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
#include <bitset>

#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

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
    m_shiftDelay = 0;
    m_debugPrintDelay = 0;
    core::array<SJoystickInfo> joystickInfo = core::array<SJoystickInfo>();

    // Load joystick configuration...
    rapidjson::Document d;
    ReadFileJSON("../data/joystick.json", d);
    if (!d.IsNull()) {
        steerAxis.read(d, "steering");
        throttleAxis.read(d, "throttle");
        brakeAxis.read(d, "brake");
        clutchAxis.read(d, "clutch");
        shiftUpButton.read(d, "shiftUp");
        shiftDownButton.read(d, "shiftDown");
        gearReverseButton.read(d, "gearReverse");
        gear1Button.read(d, "gear1");
        gear2Button.read(d, "gear2");
        gear3Button.read(d, "gear3");
        gear4Button.read(d, "gear4");
        gear5Button.read(d, "gear5");
        gear6Button.read(d, "gear6");
        gear7Button.read(d, "gear7");
        gear8Button.read(d, "gear8");
        gear9Button.read(d, "gear9");
    } else {
        GetLog() << "Could not load joystick settings from ../data/joystick.json\n.";
    }

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

            // Now try to assign any axis and buttons to this joystick...
            if (strcmp(throttleAxis.name.c_str(), joystickInfo[joystick].Name.c_str()) == 0) {
                throttleAxis.id = joystick;
            }
            if (strcmp(brakeAxis.name.c_str(), joystickInfo[joystick].Name.c_str()) == 0) {
                brakeAxis.id = joystick;
            }
            if (strcmp(clutchAxis.name.c_str(), joystickInfo[joystick].Name.c_str()) == 0) {
                clutchAxis.id = joystick;
            }
            if (strcmp(steerAxis.name.c_str(), joystickInfo[joystick].Name.c_str()) == 0) {
                steerAxis.id = joystick;
            }
            if (strcmp(shiftUpButton.name.c_str(), joystickInfo[joystick].Name.c_str()) == 0) {
                shiftUpButton.id = joystick;
            }
            if (strcmp(shiftDownButton.name.c_str(), joystickInfo[joystick].Name.c_str()) == 0) {
                shiftDownButton.id = joystick;
            }
            if (strcmp(gear1Button.name.c_str(), joystickInfo[joystick].Name.c_str()) == 0) {
                gear1Button.id = joystick;
            }
            if (strcmp(gear2Button.name.c_str(), joystickInfo[joystick].Name.c_str()) == 0) {
                gear2Button.id = joystick;
            }
            if (strcmp(gear3Button.name.c_str(), joystickInfo[joystick].Name.c_str()) == 0) {
                gear3Button.id = joystick;
            }
            if (strcmp(gear4Button.name.c_str(), joystickInfo[joystick].Name.c_str()) == 0) {
                gear4Button.id = joystick;
            }
            if (strcmp(gear5Button.name.c_str(), joystickInfo[joystick].Name.c_str()) == 0) {
                gear5Button.id = joystick;
            }
            if (strcmp(gear6Button.name.c_str(), joystickInfo[joystick].Name.c_str()) == 0) {
                gear6Button.id = joystick;
            }
            if (strcmp(gear7Button.name.c_str(), joystickInfo[joystick].Name.c_str()) == 0) {
                gear7Button.id = joystick;
            }
            if (strcmp(gear8Button.name.c_str(), joystickInfo[joystick].Name.c_str()) == 0) {
                gear8Button.id = joystick;
            }
            if (strcmp(gear9Button.name.c_str(), joystickInfo[joystick].Name.c_str()) == 0) {
                gear9Button.id = joystick;
            }
            if (strcmp(gearReverseButton.name.c_str(), joystickInfo[joystick].Name.c_str()) == 0) {
                gearReverseButton.id = joystick;
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

        // ...
        if (event.JoystickEvent.Joystick == steerAxis.id) {
            SetSteering(steerAxis.value((double)event.JoystickEvent.Axis[steerAxis.axis]));
        }
        if (event.JoystickEvent.Joystick == throttleAxis.id) {
            SetThrottle(throttleAxis.value((double)event.JoystickEvent.Axis[throttleAxis.axis]));
        }
        if (event.JoystickEvent.Joystick == brakeAxis.id) {
            SetBraking(brakeAxis.value((double) event.JoystickEvent.Axis[brakeAxis.axis]));
        }

        // Sequential shifter code...
        if (m_vehicle.GetPowertrain()) {
            // To prevent people from "double shifting" we add a shift delay here and ignore any further
            // button presses for a while.
            if (m_shiftDelay > 10) {
                if (shiftUpButton.isPressed(event.JoystickEvent)) {
                    m_vehicle.GetPowertrain()->ShiftUp();
                    m_shiftDelay = 0;
                } else if (shiftDownButton.isPressed(event.JoystickEvent)) {
                    m_vehicle.GetPowertrain()->ShiftDown();
                    m_shiftDelay = 0;
                }
            }
            m_shiftDelay++;
        }

        // H-shifter code...
        if (clutchAxis.axis != NONE && event.JoystickEvent.Joystick == clutchAxis.id) {
            double rawClutchPosition = (double)event.JoystickEvent.Axis[clutchAxis.axis];
            double clutchPosition = clutchAxis.value(rawClutchPosition);
            // Check if that clutch is pressed...
            if ((clutchAxis.scaled_max - clutchPosition) < 0.1) {
                SetThrottle(0);
                bool reverseGearEngaged = gearReverseButton.isPressed(event.JoystickEvent);
                int forwardGearEngaged = 0;
                if (gear1Button.isPressed(event.JoystickEvent)) {
                    forwardGearEngaged = 1;
                } else if (gear2Button.isPressed(event.JoystickEvent)) {
                    forwardGearEngaged = 2;
                } else if (gear3Button.isPressed(event.JoystickEvent)) {
                    forwardGearEngaged = 3;
                } else if (gear4Button.isPressed(event.JoystickEvent)) {
                    forwardGearEngaged = 4;
                } else if (gear5Button.isPressed(event.JoystickEvent)) {
                    forwardGearEngaged = 5;
                } else if (gear6Button.isPressed(event.JoystickEvent)) {
                    forwardGearEngaged = 6;
                } else if (gear7Button.isPressed(event.JoystickEvent)) {
                    forwardGearEngaged = 7;
                } else if (gear8Button.isPressed(event.JoystickEvent)) {
                    forwardGearEngaged = 8;
                } else if (gear9Button.isPressed(event.JoystickEvent)) {
                    forwardGearEngaged = 9;
                }

                if (m_vsys.m_vehicle->GetPowertrain() && m_vsys.m_vehicle->GetPowertrain()->GetTransmissionMode() == ChPowertrain::TransmissionMode::MANUAL) {
                    if (reverseGearEngaged) {
                        /// Gear is set to reverse
                        m_vsys.m_vehicle->GetPowertrain()->SetDriveMode(ChPowertrain::DriveMode::REVERSE);
                        m_vsys.m_vehicle->GetPowertrain()->SetGear(0);
                    } else if (forwardGearEngaged > 0) {
                        // All 'forward' gears set drive mode to forward, regardless of gear
                        m_vsys.m_vehicle->GetPowertrain()->SetDriveMode(ChPowertrain::DriveMode::FORWARD);
                        m_vsys.m_vehicle->GetPowertrain()->SetGear(forwardGearEngaged);
                    } else {
                        m_vsys.m_vehicle->GetPowertrain()->SetDriveMode(ChPowertrain::DriveMode::NEUTRAL);
                        // Here you see it would be beneficial to have a gear selection for 'neutral' in the model.
                    }
                }
            }
        }
        // joystick callback, outside of condition because it might be used to switch to joystick
        if (callbackButton > -1 && cb_fun != nullptr && event.JoystickEvent.IsButtonPressed(callbackButton) ) {
            cb_fun();
        }

        // Output some debug information
        if (m_debugJoystickData) {
            if (m_debugPrintDelay < 30) {
                m_debugPrintDelay++;
            } else {
                m_debugPrintDelay = 0;
                std::string joystickLine = "Joystick " + std::to_string(event.JoystickEvent.Joystick) + " A:";
                for (int i = 0; i < 6; i++) {
                    joystickLine += " " + std::to_string(event.JoystickEvent.Axis[i]);
                }
                joystickLine += " B: " + std::bitset<32>(event.JoystickEvent.ButtonStates).to_string() + "\n";
                GetLog() << joystickLine;
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

//void ChIrrGuiDriver::SetJoystickAxes(JoystickAxes tr_ax, JoystickAxes br_ax, JoystickAxes st_ax, JoystickAxes cl_ax){
//
//    throttle_axis = tr_ax;
//    brake_axis = br_ax; 
//    steer_axis = st_ax;
//    clutch_axis = cl_ax;
//}

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
