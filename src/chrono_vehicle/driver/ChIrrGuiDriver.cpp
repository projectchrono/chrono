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
// Authors: Radu Serban, Justin Madsen, Conlain Kelly, Marcel Offermans
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

#include "chrono_thirdparty/filesystem/path.h"

using namespace irr;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

// Process joystick events every 16 ticks (~60 Hz)
const int JoystickProcessFrequency = 16;

// Output joystick debug info every 30 ticks
const int JoystickOutputFrequency = 32;

// -----------------------------------------------------------------------------

ChIrrGuiDriver::ChIrrGuiDriver(ChVehicleVisualSystemIrrlicht& vsys)
    : ChDriver(*vsys.m_vehicle),
      m_vsys(vsys),
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
      m_braking_gain(4.0),
      m_joystick_debug(false),
      m_joystick_debug_frame(0),
      m_joystick_proccess_frame(0),
      m_callback_button(-1),
      m_callback_function(nullptr),
      m_joystick_file(GetDataFile("joystick/controller_Default.json")) {
    // Add this driver as an Irrlicht event receiver
    vsys.AddUserEventReceiver(this);

    // Activate joysticks, is any
    m_joystick_info = core::array<SJoystickInfo>();
    vsys.GetDevice()->activateJoysticks(m_joystick_info);
    if (m_joystick_info.size() > 0) {
        m_mode = InputMode::JOYSTICK;
    }
}

// -----------------------------------------------------------------------------

bool ChIrrGuiDriver::OnEvent(const SEvent& event) {
    if (event.EventType == EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown) {
        switch (event.KeyInput.Key) {
            case KEY_KEY_L:
                if (m_mode != InputMode::LOCK) {
                    m_mode = InputMode::LOCK;
                }
                return true;

            case KEY_KEY_K:
                if (m_mode != InputMode::KEYBOARD) {
                    m_throttle = 0;
                    m_steering = 0;
                    m_braking = 0;
                    m_mode = InputMode::KEYBOARD;
                }
                return true;

            case KEY_KEY_J:
                if (m_mode != InputMode::JOYSTICK && m_joystick_info.size() > 0) {
                    m_mode = InputMode::JOYSTICK;
                }
                return true;

            case KEY_KEY_F:
                if (m_mode != InputMode::DATAFILE && m_data_driver) {
                    m_mode = InputMode::DATAFILE;
                    m_time_shift = m_vsys.m_vehicle->GetSystem()->GetChTime();
                }
                return true;

            default:
                return false;
        }
    }

    if (m_mode == InputMode::JOYSTICK && event.EventType == EET_JOYSTICK_INPUT_EVENT) {
        return ProcessJoystickEvents(event);
    }

    if (m_mode == InputMode::KEYBOARD && event.EventType == EET_KEY_INPUT_EVENT) {
        return ProcessKeyboardEvents(event);
    }

    return false;
}

bool ChIrrGuiDriver::ProcessJoystickEvents(const SEvent& event) {
    // Driver only handles input every 16 ticks (~60 Hz)
    if (m_joystick_proccess_frame < JoystickProcessFrequency) {
        m_joystick_proccess_frame++;
        return true;
    }

    m_joystick_proccess_frame = 0;

    // Update steering, throttle and brake axes...
    SetSteering(steerAxis.GetValue(event.JoystickEvent));
    SetThrottle(throttleAxis.GetValue(event.JoystickEvent));
    SetBraking(brakeAxis.GetValue(event.JoystickEvent));

    // Sequential shifter code...
    if (m_vehicle.GetPowertrain()) {
        // To prevent people from "double shifting" we add a shift delay here and ignore any further
        // button presses for a while. Also we make sure that after pressing the shifter, you need
        // to release it again before you can shift again.
        if (shiftUpButton.IsPressed(event.JoystickEvent)) {
            m_vehicle.GetPowertrain()->ShiftUp();
        } else if (shiftDownButton.IsPressed(event.JoystickEvent)) {
            m_vehicle.GetPowertrain()->ShiftDown();
        }
    }

    // H-shifter code...
    if (clutchAxis.axis != ChIrrJoystickAxis::NONE) {
        // double rawClutchPosition = (double)event.JoystickEvent.Axis[clutchAxis.axis];
        double clutchPosition = clutchAxis.GetValue(event.JoystickEvent);
        // Check if that clutch is pressed...
        if ((clutchAxis.scaled_max - clutchPosition) < 0.1) {
            SetThrottle(0);
            bool reverseGearEngaged = gearReverseButton.IsPressed(event.JoystickEvent);
            int forwardGearEngaged = 0;
            if (gear1Button.IsPressed(event.JoystickEvent, true)) {
                forwardGearEngaged = 1;
            } else if (gear2Button.IsPressed(event.JoystickEvent, true)) {
                forwardGearEngaged = 2;
            } else if (gear3Button.IsPressed(event.JoystickEvent, true)) {
                forwardGearEngaged = 3;
            } else if (gear4Button.IsPressed(event.JoystickEvent, true)) {
                forwardGearEngaged = 4;
            } else if (gear5Button.IsPressed(event.JoystickEvent, true)) {
                forwardGearEngaged = 5;
            } else if (gear6Button.IsPressed(event.JoystickEvent, true)) {
                forwardGearEngaged = 6;
            } else if (gear7Button.IsPressed(event.JoystickEvent, true)) {
                forwardGearEngaged = 7;
            } else if (gear8Button.IsPressed(event.JoystickEvent, true)) {
                forwardGearEngaged = 8;
            } else if (gear9Button.IsPressed(event.JoystickEvent, true)) {
                forwardGearEngaged = 9;
            }

            if (m_vsys.m_vehicle->GetPowertrain() &&
                m_vsys.m_vehicle->GetPowertrain()->GetTransmissionMode() == ChPowertrain::TransmissionMode::MANUAL) {
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
    if (m_callback_button > -1 && m_callback_function != nullptr &&
        event.JoystickEvent.IsButtonPressed(m_callback_button)) {
        m_callback_function();
    }

    // Toggle between a manual and automatic gearbox
    if (toggleManualGearboxButton.IsPressed(event.JoystickEvent)) {
        if (m_vsys.m_vehicle->GetPowertrain()->GetTransmissionMode() == ChPowertrain::TransmissionMode::AUTOMATIC) {
            m_vsys.m_vehicle->GetPowertrain()->SetTransmissionMode(ChPowertrain::TransmissionMode::MANUAL);
        } else {
            m_vsys.m_vehicle->GetPowertrain()->SetTransmissionMode(ChPowertrain::TransmissionMode::AUTOMATIC);
        }
    }

    // Output some debug information (can be very useful when setting up or calibrating joysticks)
    if (m_joystick_debug) {
        if (m_joystick_debug_frame < JoystickOutputFrequency) {
            m_joystick_debug_frame++;
        } else {
            m_joystick_debug_frame = 0;
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

bool ChIrrGuiDriver::ProcessKeyboardEvents(const SEvent& event) {
    if (event.KeyInput.PressedDown) {
        switch (event.KeyInput.Key) {
            case KEY_KEY_A:
                m_steering_target = ChClamp(m_steering_target + m_steering_delta, -1.0, +1.0);
                return true;
            case KEY_KEY_D:
                m_steering_target = ChClamp(m_steering_target - m_steering_delta, -1.0, +1.0);
                return true;
            case KEY_KEY_W:
                m_throttle_target = ChClamp(m_throttle_target + m_throttle_delta, 0.0, +1.0);
                if (m_throttle_target > 0)
                    m_braking_target = ChClamp(m_braking_target - m_braking_delta * 3, 0.0, +1.0);
                return true;
            case KEY_KEY_S:
                m_throttle_target = ChClamp(m_throttle_target - m_throttle_delta * 3, 0.0, +1.0);
                if (m_throttle_target <= 0)
                    m_braking_target = ChClamp(m_braking_target + m_braking_delta, 0.0, +1.0);
                return true;
            default:
                return false;
        }
    } else {
        switch (event.KeyInput.Key) {
            case KEY_KEY_Z:
                if (m_vsys.m_vehicle->GetPowertrain())
                    m_vsys.m_vehicle->GetPowertrain()->SetDriveMode(ChPowertrain::DriveMode::FORWARD);
                return true;
            case KEY_KEY_X:
                if (m_vsys.m_vehicle->GetPowertrain())
                    m_vsys.m_vehicle->GetPowertrain()->SetDriveMode(ChPowertrain::DriveMode::NEUTRAL);
                return true;
            case KEY_KEY_C:
                if (m_vsys.m_vehicle->GetPowertrain())
                    m_vsys.m_vehicle->GetPowertrain()->SetDriveMode(ChPowertrain::DriveMode::REVERSE);
                return true;

            case KEY_KEY_T:
                if (m_vsys.m_vehicle->GetPowertrain()) {
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
                if (m_vsys.m_vehicle->GetPowertrain())
                    m_vsys.m_vehicle->GetPowertrain()->ShiftUp();
                return true;
            case KEY_COMMA:
                if (m_vsys.m_vehicle->GetPowertrain())
                    m_vsys.m_vehicle->GetPowertrain()->ShiftDown();
                return true;

            default:
                break;
        }
    }

    return false;
}

// -----------------------------------------------------------------------------

void ChIrrGuiDriver::SetInputMode(InputMode mode) {
    switch (mode) {
        case InputMode::KEYBOARD:
            m_throttle_target = 0;
            m_steering_target = 0;
            m_braking_target = 0;
            m_mode = mode;
            break;
        case InputMode::DATAFILE:
            if (m_data_driver) {
                m_time_shift = m_vsys.m_vehicle->GetSystem()->GetChTime();
                m_mode = mode;
            }
            break;
        case InputMode::JOYSTICK:
            if (m_joystick_info.size() > 0)
                m_mode = mode;
            else
                std::cerr << "No joysticks available. Input mode unchanged" << std::endl;
            break;
        case InputMode::LOCK:
            m_mode = mode;
        default:
            break;
    }
}

std::string ChIrrGuiDriver::GetInputModeAsString() const {
    switch (m_mode) {
        case InputMode::LOCK:
            return std::string("Input mode: LOCK");
        case InputMode::KEYBOARD:
            return std::string("Input mode: KEY");
        case InputMode::DATAFILE:
            return std::string("Input mode: FILE");
        case InputMode::JOYSTICK:
            return std::string("Input mode: JOYSTICK");
    }
    return std::string("");
}

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

void ChIrrGuiDriver::SetButtonCallback(int button, void (*cbfun)()) {
    m_callback_function = cbfun;
    m_callback_button = button;
}

void ChIrrGuiDriver::SetJoystickConfigFile(const std::string& filename) {
    if (!filesystem::path(filename).exists()) {
        std::cerr << "Error: the specified joystick configuration file " << filename << " does not exist.\n"
                  << "Using default configuration." << std::endl;
        return;
    }
    m_joystick_file = filename;
}

// -----------------------------------------------------------------------------

void ChIrrGuiDriver::Initialize() {
    if (m_joystick_info.size() > 0) {
        rapidjson::Document d;
        ReadFileJSON(m_joystick_file, d);

        if (d.IsNull()) {
            std::cerr << "Error reading joystick configuration file " << m_joystick_file << "\n"
                      << "Reverting to KEYBOARD mode" << std::endl;
            m_mode = InputMode::KEYBOARD;
            return;
        }

        // Read specification for individual axes and buttons.
        // For any axis/button listed in the specification file, default to first joystick
        steerAxis.Read(d, "steering", m_joystick_debug);
        throttleAxis.Read(d, "throttle", m_joystick_debug);
        brakeAxis.Read(d, "brake", m_joystick_debug);
        clutchAxis.Read(d, "clutch", m_joystick_debug);
        shiftUpButton.Read(d, "shiftUp", m_joystick_debug);
        shiftDownButton.Read(d, "shiftDown", m_joystick_debug);
        gearReverseButton.Read(d, "gearReverse", m_joystick_debug);
        gear1Button.Read(d, "gear1", m_joystick_debug);
        gear2Button.Read(d, "gear2", m_joystick_debug);
        gear3Button.Read(d, "gear3", m_joystick_debug);
        gear4Button.Read(d, "gear4", m_joystick_debug);
        gear5Button.Read(d, "gear5", m_joystick_debug);
        gear6Button.Read(d, "gear6", m_joystick_debug);
        gear7Button.Read(d, "gear7", m_joystick_debug);
        gear8Button.Read(d, "gear8", m_joystick_debug);
        gear9Button.Read(d, "gear9", m_joystick_debug);
        toggleManualGearboxButton.Read(d, "toggleManualGearbox", m_joystick_debug);

        // Loop over available controllers and distribute axes per controller if specified
        for (u32 id = 0; id < m_joystick_info.size(); ++id) {
            const auto& jinfo = m_joystick_info[id];
            GetLog() << "Joystick " << id << ":\n";
            GetLog() << "\tName: '" << jinfo.Name.c_str() << "'\n";
            GetLog() << "\tAxes: " << jinfo.Axes << "\n";
            GetLog() << "\tButtons: " << jinfo.Buttons << "\n";
            GetLog() << "\tHat is: ";
            switch (jinfo.PovHat) {
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
            if (strcmp(throttleAxis.name.c_str(), jinfo.Name.c_str()) == 0) {
                throttleAxis.id = id;
            }
            if (strcmp(brakeAxis.name.c_str(), jinfo.Name.c_str()) == 0) {
                brakeAxis.id = id;
            }
            if (strcmp(clutchAxis.name.c_str(), jinfo.Name.c_str()) == 0) {
                clutchAxis.id = id;
            }
            if (strcmp(steerAxis.name.c_str(), jinfo.Name.c_str()) == 0) {
                steerAxis.id = id;
            }
            if (strcmp(shiftUpButton.name.c_str(), jinfo.Name.c_str()) == 0) {
                shiftUpButton.id = id;
            }
            if (strcmp(shiftDownButton.name.c_str(), jinfo.Name.c_str()) == 0) {
                shiftDownButton.id = id;
            }
            if (strcmp(gear1Button.name.c_str(), jinfo.Name.c_str()) == 0) {
                gear1Button.id = id;
            }
            if (strcmp(gear2Button.name.c_str(), jinfo.Name.c_str()) == 0) {
                gear2Button.id = id;
            }
            if (strcmp(gear3Button.name.c_str(), jinfo.Name.c_str()) == 0) {
                gear3Button.id = id;
            }
            if (strcmp(gear4Button.name.c_str(), jinfo.Name.c_str()) == 0) {
                gear4Button.id = id;
            }
            if (strcmp(gear5Button.name.c_str(), jinfo.Name.c_str()) == 0) {
                gear5Button.id = id;
            }
            if (strcmp(gear6Button.name.c_str(), jinfo.Name.c_str()) == 0) {
                gear6Button.id = id;
            }
            if (strcmp(gear7Button.name.c_str(), jinfo.Name.c_str()) == 0) {
                gear7Button.id = id;
            }
            if (strcmp(gear8Button.name.c_str(), jinfo.Name.c_str()) == 0) {
                gear8Button.id = id;
            }
            if (strcmp(gear9Button.name.c_str(), jinfo.Name.c_str()) == 0) {
                gear9Button.id = id;
            }
            if (strcmp(gearReverseButton.name.c_str(), jinfo.Name.c_str()) == 0) {
                gearReverseButton.id = id;
            }
            if (strcmp(toggleManualGearboxButton.name.c_str(), jinfo.Name.c_str()) == 0) {
                toggleManualGearboxButton.id = id;
            }
        }
    }
}

void ChIrrGuiDriver::Synchronize(double time) {
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

void ChIrrGuiDriver::Advance(double step) {
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

// -----------------------------------------------------------------------------

ChIrrJoystickAxis::ChIrrJoystickAxis()
    : id(-1), name("Unknown"), axis(Axis::NONE), min(0), max(1), scaled_min(0), scaled_max(1), value(0) {}

double ChIrrJoystickAxis::GetValue(const irr::SEvent::SJoystickEvent& joystickEvent) {
    if (joystickEvent.Joystick == id) {
        // Scale raw_value fromm [scaled_min, scaled_max] to [min, max] range
        value = (joystickEvent.Axis[axis] - max) * (scaled_max - scaled_min) / (max - min) + scaled_max;
    }
    return value;
}

void ChIrrJoystickAxis::Read(rapidjson::Document& d, const std::string& elementName, bool dbg_print) {
    auto name = elementName.c_str();
    if (d.HasMember(name) && d[name].IsObject()) {
        id = 0;  // default to first attached controller
        name = d[name]["name"].GetString();
        axis = (Axis)d[name]["axis"].GetInt();
        min = d[name]["min"].GetDouble();
        max = d[name]["max"].GetDouble();
        scaled_min = d[name]["scaled_min"].GetDouble();
        scaled_max = d[name]["scaled_max"].GetDouble();
        value = min;
    } else if (dbg_print) {
        GetLog() << "Expected a joystick axis definition for " << elementName << " but did not find one.\n";
    }
}

ChIrrJoystickButton::ChIrrJoystickButton()
    : id(-1), name("Unknown"), button(-1), buttonPressed(false), buttonPressedCount(0) {}

bool ChIrrJoystickButton::IsPressed(const irr::SEvent::SJoystickEvent& joystickEvent, bool continuous) {
    if (joystickEvent.Joystick == id) {
        buttonPressed = joystickEvent.IsButtonPressed(button);
    }
    if (buttonPressed) {
        buttonPressedCount++;
    } else {
        buttonPressedCount = 0;
    }
    return continuous ? buttonPressedCount > 0 : buttonPressedCount == 1;
}

void ChIrrJoystickButton::Read(rapidjson::Document& d, const std::string& elementName, bool dbg_print) {
    auto name = elementName.c_str();
    if (d.HasMember(name) && d[name].IsObject()) {
        id = 0;  // default to first attached controller
        name = d[name]["name"].GetString();
        button = d[name]["button"].GetInt();
    } else if (dbg_print) {
        GetLog() << "Expected a joystick button definition for " << elementName << " but did not find one.\n";
    }
}

}  // end namespace vehicle
}  // end namespace chrono
