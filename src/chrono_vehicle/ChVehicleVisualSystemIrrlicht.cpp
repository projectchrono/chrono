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

#include <algorithm>
#include <bitset>

#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/driver/ChInteractiveDriver.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_vehicle/ChVehicleVisualSystemIrrlicht.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace irr;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Irrlicht joystick support
// -----------------------------------------------------------------------------

// Process joystick events every 16 ticks (~60 Hz)
const int JoystickProcessFrequency = 16;

// Output joystick debug info every 30 ticks
const int JoystickOutputFrequency = 32;

// Irrlicht interface to a specific joystick axis and its calibration data.
struct ChJoystickAxisIRR {
    ChJoystickAxisIRR();

    // Exposes the unnamed enum of irrlicht axes to enforce right values in API usage.
    enum Axis {
        AXIS_X = irr::SEvent::SJoystickEvent::AXIS_X,
        AXIS_Y = irr::SEvent::SJoystickEvent::AXIS_Y,
        AXIS_Z = irr::SEvent::SJoystickEvent::AXIS_Z,
        AXIS_R = irr::SEvent::SJoystickEvent::AXIS_R,
        AXIS_U = irr::SEvent::SJoystickEvent::AXIS_U,
        AXIS_V = irr::SEvent::SJoystickEvent::AXIS_V,
        NONE
    };

    // Get value from joystick event (if triggered by this axis).
    double GetValue(const irr::SEvent::SJoystickEvent& joystickEvent);

    // Read axis configuration from JSON file.
    void Read(rapidjson::Document& d, const std::string& elementName, bool dbg_print);

    int id;                         // controller ID
    Axis axis;                      // controller axis
    std::string name;               // axis name
    double min, max;                // input range
    double scaled_min, scaled_max;  // output range
    double value;                   // current output value
};

// Irrlicht interface to a specific joystick button.
struct ChJoystickButtonIRR {
    ChJoystickButtonIRR();

    // Return press state for this joystick event (if triggered by this button).
    bool IsPressed(const irr::SEvent::SJoystickEvent& joystickEvent, bool continuous = false);

    // Read button configuration from JSON file.
    void Read(rapidjson::Document& d, const std::string& elementName, bool dbg_print);

    int id;                  // controller ID
    int button;              // controller button
    std::string name;        // buttoin name
    int buttonPressedCount;  // counter to identify continuous press
    bool buttonPressed;      // current output value
};

// Irrlicht joystick settings.
class ChJoystickIRR {
  public:
    // Construct an Irrlicht GUI driver.
    ChJoystickIRR(ChVehicleVisualSystemIrrlicht* vsys);

    virtual ~ChJoystickIRR() {}

    // Check if joystick is supported.
    bool Active() const { return joystick_info.size() > 0; }

    // Initialize this driver system.
    void Initialize();

  private:
    ChVehicleVisualSystemIrrlicht* m_vsys;

    // Variables for mode=JOYSTICK
    irr::core::array<irr::SJoystickInfo> joystick_info;  // Irrlicht joystick information
    std::string joystick_file;                           // JSON specification file
    int m_joystick_proccess_frame;                       // counter for successive event processing frames
    bool joystick_debug;                                 // enable/disable debug output
    int joystick_debug_frame;                            // counter for successive output frames

    ChJoystickAxisIRR steerAxis;
    ChJoystickAxisIRR throttleAxis;
    ChJoystickAxisIRR brakeAxis;
    ChJoystickAxisIRR clutchAxis;
    ChJoystickButtonIRR shiftUpButton;
    ChJoystickButtonIRR shiftDownButton;
    ChJoystickButtonIRR gearReverseButton;
    ChJoystickButtonIRR gear1Button;
    ChJoystickButtonIRR gear2Button;
    ChJoystickButtonIRR gear3Button;
    ChJoystickButtonIRR gear4Button;
    ChJoystickButtonIRR gear5Button;
    ChJoystickButtonIRR gear6Button;
    ChJoystickButtonIRR gear7Button;
    ChJoystickButtonIRR gear8Button;
    ChJoystickButtonIRR gear9Button;
    ChJoystickButtonIRR toggleManualGearboxButton;

    int callback_button;          // joystick button associated to the custom callback
    void (*callback_function)();  // custom callback, can be implemented in the application

    friend class ChVehicleVisualSystemIrrlicht;
    friend class ChVehicleEventReceiver;
};

// -----------------------------------------------------------------------------
// Implementation of the custom Irrlicht event receiver for chase camera control
// -----------------------------------------------------------------------------

class ChChaseCameraEventReceiver : public irr::IEventReceiver {
  public:
    ChChaseCameraEventReceiver(ChVehicleVisualSystemIrrlicht* vsys) : m_vsys(vsys) {}

    virtual bool OnEvent(const irr::SEvent& event);

  private:
    ChVehicleVisualSystemIrrlicht* m_vsys;  // associated vehicle Irrlicht visualization
};

bool ChChaseCameraEventReceiver::OnEvent(const SEvent& event) {
    // Only interpret keyboard inputs.
    if (event.EventType != EET_KEY_INPUT_EVENT)
        return false;

    if (event.KeyInput.PressedDown) {
        switch (event.KeyInput.Key) {
            case KEY_DOWN:
                m_vsys->m_camera->Zoom(1);
                return true;
            case KEY_UP:
                m_vsys->m_camera->Zoom(-1);
                return true;
            case KEY_LEFT:
                m_vsys->m_camera->Turn(-1);
                return true;
            case KEY_RIGHT:
                m_vsys->m_camera->Turn(1);
                return true;
            case KEY_NEXT:
                m_vsys->m_camera->Raise(1);
                return true;
            case KEY_PRIOR:
                m_vsys->m_camera->Raise(-1);
                return true;
            default:
                break;
        }
    } else {
        switch (event.KeyInput.Key) {
            case KEY_KEY_1:
                m_vsys->m_camera->SetState(utils::ChChaseCamera::Chase);
                return true;
            case KEY_KEY_2:
                m_vsys->m_camera->SetState(utils::ChChaseCamera::Follow);
                return true;
            case KEY_KEY_3:
                m_vsys->m_camera->SetState(utils::ChChaseCamera::Track);
                return true;
            case KEY_KEY_4:
                m_vsys->m_camera->SetState(utils::ChChaseCamera::Inside);
                return true;
            case KEY_KEY_5:
                m_vsys->m_camera->SetState(utils::ChChaseCamera::Free);
                return true;
            case KEY_KEY_V:
                m_vsys->m_vehicle->LogConstraintViolations();
                return true;
            default:
                break;
        }
    }

    return false;
}

// -----------------------------------------------------------------------------
// Implementation of the custom Irrlicht event receiver for vehicle control
// -----------------------------------------------------------------------------

class ChVehicleEventReceiver : public irr::IEventReceiver {
  public:
    ChVehicleEventReceiver(ChVehicleVisualSystemIrrlicht* vsys) : m_vsys(vsys) {}

    virtual bool OnEvent(const irr::SEvent& event);

  private:
    bool ProcessKeyboardEvents(const irr::SEvent& event, ChVehicle* vehicle, ChInteractiveDriver* driver);
    bool ProcessJoystickEvents(const irr::SEvent& event,
                               ChJoystickIRR* joystick,
                               ChVehicle* vehicle,
                               ChInteractiveDriver* driver);

    ChVehicleVisualSystemIrrlicht* m_vsys;  // associated vehicle Irrlicht visualization
};

bool ChVehicleEventReceiver::OnEvent(const irr::SEvent& event) {
    if (!m_vsys->m_vehicle)
        return false;
    auto driver = dynamic_cast<ChInteractiveDriver*>(m_vsys->GetDriver());
    if (!driver)
        return false;
    auto mode = driver->GetInputMode();
    auto joystick = m_vsys->m_joystick;
    if (!joystick->Active() && mode == ChInteractiveDriver::InputMode::JOYSTICK) {
        driver->SetInputMode(ChInteractiveDriver::InputMode::KEYBOARD);
    }

    if (event.EventType == EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown) {
        switch (event.KeyInput.Key) {
            case KEY_KEY_L:
                if (mode != ChInteractiveDriver::InputMode::LOCK) {
                    driver->SetInputMode(ChInteractiveDriver::InputMode::LOCK);
                }
                return true;

            case KEY_KEY_K:
                if (mode != ChInteractiveDriver::InputMode::KEYBOARD) {
                    driver->SteeringCenter();
                    driver->ReleasePedals();
                    driver->SetInputMode(ChInteractiveDriver::InputMode::KEYBOARD);
                }
                return true;

            case KEY_KEY_J:
                if (mode != ChInteractiveDriver::InputMode::JOYSTICK && joystick->Active()) {
                    driver->SetInputMode(ChInteractiveDriver::InputMode::JOYSTICK);
                }
                return true;

            default:
                break;
        }
    }

    if (mode == ChInteractiveDriver::InputMode::JOYSTICK && event.EventType == EET_JOYSTICK_INPUT_EVENT) {
        assert(joystick->Active());
        return ProcessJoystickEvents(event, joystick, m_vsys->m_vehicle, driver);
    }

    if (mode == ChInteractiveDriver::InputMode::KEYBOARD && event.EventType == EET_KEY_INPUT_EVENT) {
        return ProcessKeyboardEvents(event, m_vsys->m_vehicle, driver);
    }

    return false;
}

bool ChVehicleEventReceiver::ProcessKeyboardEvents(const SEvent& event,
                                                   ChVehicle* vehicle,
                                                   ChInteractiveDriver* driver) {
    ChAutomaticTransmission* transmission_auto = nullptr;
    ChManualTransmission* transmission_manual = nullptr;
    if (vehicle->GetTransmission()) {
        transmission_auto = vehicle->GetTransmission()->asAutomatic();  // nullptr for a manual transmission
        transmission_manual = vehicle->GetTransmission()->asManual();   // nullptr for an automatic transmission
    }

    if (event.KeyInput.PressedDown) {
        switch (event.KeyInput.Key) {
            case KEY_KEY_A:
                driver->SteeringLeft();
                return true;
            case KEY_KEY_D:
                driver->SteeringRight();
                return true;
            case KEY_KEY_W:
                driver->IncreaseThrottle();
                return true;
            case KEY_KEY_S:
                driver->DecreaseThrottle();
                return true;
            default:
                break;
        }
        if (transmission_manual) {
            switch (event.KeyInput.Key) {
                case KEY_KEY_E:
                    driver->IncreaseClutch();
                    return true;
                case KEY_KEY_Q:
                    driver->DecreaseClutch();
                    return true;
                default:
                    break;
            }
        }
    } else {
        switch (event.KeyInput.Key) {
            case KEY_KEY_C:
                driver->SteeringCenter();
                return true;
            case KEY_KEY_R:
                driver->ReleasePedals();
                return true;
            default:
                break;
        }
        if (transmission_auto) {
            switch (event.KeyInput.Key) {
                case KEY_KEY_Z:
                    if (transmission_auto->GetDriveMode() != ChAutomaticTransmission::DriveMode::FORWARD)
                        transmission_auto->SetDriveMode(ChAutomaticTransmission::DriveMode::FORWARD);
                    else
                        transmission_auto->SetDriveMode(ChAutomaticTransmission::DriveMode::REVERSE);
                    return true;
                case KEY_KEY_X:
                    transmission_auto->SetDriveMode(ChAutomaticTransmission::DriveMode::NEUTRAL);
                    return true;
                case KEY_KEY_T:
                    if (transmission_auto->GetShiftMode() == ChAutomaticTransmission::ShiftMode::MANUAL)
                        transmission_auto->SetShiftMode(ChAutomaticTransmission::ShiftMode::AUTOMATIC);
                    else
                        transmission_auto->SetShiftMode(ChAutomaticTransmission::ShiftMode::MANUAL);
                    return true;
                case KEY_OEM_6:  // ']'
                    transmission_auto->ShiftUp();
                    return true;
                case KEY_OEM_4:  // '['
                    transmission_auto->ShiftDown();
                    return true;
                default:
                    break;
            }
        } else if (transmission_manual) {
            switch (event.KeyInput.Key) {
                case KEY_OEM_6:  // ']'
                    transmission_manual->ShiftUp();
                    return true;
                case KEY_OEM_4:  // '['
                    transmission_manual->ShiftDown();
                    return true;
                default:
                    break;
            }
        }
    }

    return false;
}

bool ChVehicleEventReceiver::ProcessJoystickEvents(const SEvent& event,
                                                   ChJoystickIRR* joystick,
                                                   ChVehicle* vehicle,
                                                   ChInteractiveDriver* driver) {
    // Driver only handles input every 16 ticks (~60 Hz)
    if (joystick->m_joystick_proccess_frame < JoystickProcessFrequency) {
        joystick->m_joystick_proccess_frame++;
        return true;
    }

    joystick->m_joystick_proccess_frame = 0;

    // Update steering, throttle and brake axes...
    driver->SetSteering(joystick->steerAxis.GetValue(event.JoystickEvent));
    driver->SetThrottle(joystick->throttleAxis.GetValue(event.JoystickEvent));
    driver->SetBraking(joystick->brakeAxis.GetValue(event.JoystickEvent));
    driver->SetClutch(joystick->clutchAxis.GetValue(event.JoystickEvent));

    // joystick callback
    if (joystick->callback_button > -1 && joystick->callback_function != nullptr &&
        event.JoystickEvent.IsButtonPressed(joystick->callback_button)) {
        joystick->callback_function();
    }

    auto transmission = vehicle->GetTransmission();
    if (!transmission)
        return true;

    auto transmission_auto = transmission->asAutomatic();  // nullptr for a manual transmission
    auto transmission_manual = transmission->asManual();   // nullptr for an automatic transmission

    // Automatic transmission: check shift to manumatic and gear shift
    if (transmission->IsAutomatic()) {
        // Toggle between a automatic and manumatic shift modes
        if (joystick->toggleManualGearboxButton.IsPressed(event.JoystickEvent)) {
            if (transmission_auto->GetShiftMode() == ChAutomaticTransmission::ShiftMode::AUTOMATIC) {
                transmission_auto->SetShiftMode(ChAutomaticTransmission::ShiftMode::MANUAL);
            } else {
                transmission_auto->SetShiftMode(ChAutomaticTransmission::ShiftMode::AUTOMATIC);
            }
        }

        // Shift up or down
        if (joystick->shiftUpButton.IsPressed(event.JoystickEvent)) {
            transmission_auto->ShiftUp();
        } else if (joystick->shiftDownButton.IsPressed(event.JoystickEvent)) {
            transmission_auto->ShiftDown();
        }
    }

    // Manual transmission
    if (transmission->IsManual()) {
        // Sequential gear shifts: up or down
        if (joystick->shiftUpButton.IsPressed(event.JoystickEvent)) {
            transmission_manual->ShiftUp();
        } else if (joystick->shiftDownButton.IsPressed(event.JoystickEvent)) {
            transmission_manual->ShiftDown();
        }
        // Support an H-shifter if present and change gears if you press
        // the clutch and shift the car into a specific gear.
        if (joystick->clutchAxis.axis != ChJoystickAxisIRR::NONE) {
            double clutchPosition = joystick->clutchAxis.GetValue(event.JoystickEvent);
            // Check if the clutch is pressed...
            if ((joystick->clutchAxis.scaled_max - clutchPosition) < 0.1) {
                bool reverseGearEngaged = joystick->gearReverseButton.IsPressed(event.JoystickEvent);
                int forwardGearEngaged = 0;
                if (joystick->gear1Button.IsPressed(event.JoystickEvent, true))
                    forwardGearEngaged = 1;
                else if (joystick->gear2Button.IsPressed(event.JoystickEvent, true))
                    forwardGearEngaged = 2;
                else if (joystick->gear3Button.IsPressed(event.JoystickEvent, true))
                    forwardGearEngaged = 3;
                else if (joystick->gear4Button.IsPressed(event.JoystickEvent, true))
                    forwardGearEngaged = 4;
                else if (joystick->gear5Button.IsPressed(event.JoystickEvent, true))
                    forwardGearEngaged = 5;
                else if (joystick->gear6Button.IsPressed(event.JoystickEvent, true))
                    forwardGearEngaged = 6;
                else if (joystick->gear7Button.IsPressed(event.JoystickEvent, true))
                    forwardGearEngaged = 7;
                else if (joystick->gear8Button.IsPressed(event.JoystickEvent, true))
                    forwardGearEngaged = 8;
                else if (joystick->gear9Button.IsPressed(event.JoystickEvent, true))
                    forwardGearEngaged = 9;

                if (reverseGearEngaged) {
                    // Gear is set to reverse
                    vehicle->GetTransmission()->SetGear(-1);
                } else if (forwardGearEngaged > 0) {
                    // All 'forward' gears set drive mode to forward, regardless of gear
                    vehicle->GetTransmission()->SetGear(forwardGearEngaged);
                }
            }
        }
    }

    // Output some debug information (can be very useful when setting up or calibrating joysticks)
    if (joystick->joystick_debug) {
        if (joystick->joystick_debug_frame < JoystickOutputFrequency) {
            joystick->joystick_debug_frame++;
        } else {
            joystick->joystick_debug_frame = 0;
            std::string joystickLine = "Joystick " + std::to_string(event.JoystickEvent.Joystick) + " A:";
            for (int i = 0; i < 6; i++) {
                joystickLine += " " + std::to_string(event.JoystickEvent.Axis[i]);
            }
            joystickLine += " B: " + std::bitset<32>(event.JoystickEvent.ButtonStates).to_string() + "\n";
            std::cout << joystickLine;
        }
    }

    return true;
}

// -----------------------------------------------------------------------------
// Construct a vehicle Irrlicht application.
// -----------------------------------------------------------------------------

ChVehicleVisualSystemIrrlicht::ChVehicleVisualSystemIrrlicht()
    : ChVisualSystemIrrlicht(), m_renderStats(true), m_HUD_x(600), m_HUD_y(20) {
    // Set default window size and title
    SetWindowSize(1000, 800);
    SetWindowTitle("Chrono::Vehicle");

    // Default camera uses Z up
    SetCameraVertical(CameraVerticalDir::Z);

    // Create the joystick setup
    m_joystick = new ChJoystickIRR(this);

    // Create the event receivers for camera and vehicle control
    m_camera_control = new ChChaseCameraEventReceiver(this);
    m_vehicle_control = new ChVehicleEventReceiver(this);

#ifdef CHRONO_IRRKLANG
    m_sound_engine = 0;
    m_car_sound = 0;
#endif
}

ChVehicleVisualSystemIrrlicht::~ChVehicleVisualSystemIrrlicht() {
    delete m_joystick;
    delete m_camera_control;
    delete m_vehicle_control;
}

void ChVehicleVisualSystemIrrlicht::SetJoystickConfigFile(const std::string& filename) {
    if (!filesystem::path(filename).exists()) {
        std::cerr << "Error: the specified joystick configuration file " << filename << " does not exist.\n"
                  << "Using default configuration." << std::endl;
        return;
    }
    m_joystick->joystick_file = filename;
}

void ChVehicleVisualSystemIrrlicht::SetJoystickDebug(bool val) {
    m_joystick->joystick_debug = val;
}

void ChVehicleVisualSystemIrrlicht::SetButtonCallback(int button, void (*cbfun)()) {
    m_joystick->callback_function = cbfun;
    m_joystick->callback_button = button;
}

void ChVehicleVisualSystemIrrlicht::AttachVehicle(ChVehicle* vehicle) {
    ChVehicleVisualSystem::AttachVehicle(vehicle);

    // Add the Irrlicht camera (controlled through the chase-cam) if already initialized
    if (GetDevice()) {
        ChVector3d cam_pos = m_camera->GetCameraPos();
        ChVector3d cam_target = m_camera->GetTargetPos();
        AddCamera(cam_pos, cam_target);
    }
}

void ChVehicleVisualSystemIrrlicht::Initialize() {
    ChVisualSystemIrrlicht::Initialize();

    // Adjust HUD location based on window size
    m_HUD_x = GetVideoDriver()->getScreenSize().Width - 380;

    // Initialize joysticks
    m_joystick->Initialize();

    // Attach the event receivers for chase camera and vehicle control
    AddUserEventReceiver(m_camera_control);
    AddUserEventReceiver(m_vehicle_control);

    // Add the Irrlicht camera (controlled through the chase-cam) if already attach to a vehicle
    if (m_vehicle) {
        ChVector3d cam_pos = m_camera->GetCameraPos();
        ChVector3d cam_target = m_camera->GetTargetPos();
        AddCamera(cam_pos, cam_target);
    }
}

// -----------------------------------------------------------------------------
// Turn on/off Irrklang sound generation.
// Note that this has an effect only if Irrklang support was enabled at
// configuration.
// -----------------------------------------------------------------------------
void ChVehicleVisualSystemIrrlicht::EnableSound(bool sound) {
#ifdef CHRONO_IRRKLANG
    if (sound) {
        // Start the sound engine with default parameters
        m_sound_engine = irrklang::createIrrKlangDevice();

        // To play a sound, call play2D(). The second parameter tells the engine to
        // play it looped.
        if (m_sound_engine) {
            m_car_sound =
                m_sound_engine->play2D(GetChronoDataFile("vehicle/sounds/carsound.ogg").c_str(), true, false, true);
            m_car_sound->setIsPaused(true);
        } else
            std::cerr << "Cannot start sound engine Irrklang" << std::endl;
    } else {
        m_sound_engine = 0;
        m_car_sound = 0;
    }
#endif
}

// -----------------------------------------------------------------------------
// Advance the dynamics of the chase camera.
// The integration of the underlying ODEs is performed using as many steps as
// needed to advance by the specified duration.
// -----------------------------------------------------------------------------
void ChVehicleVisualSystemIrrlicht::Advance(double step) {
    // Update the ChChaseCamera: take as many integration steps as needed to
    // exactly reach the value 'step'
    double t = 0;
    while (t < step) {
        double h = std::min<>(m_stepsize, step - t);
        m_camera->Update(h);
        t += h;
    }

    // Update the Irrlicht camera
    ChVector3d cam_pos = m_camera->GetCameraPos();
    ChVector3d cam_target = m_camera->GetTargetPos();

    GetActiveCamera()->setPosition(core::vector3dfCH(cam_pos));
    GetActiveCamera()->setTarget(core::vector3dfCH(cam_target));

#ifdef CHRONO_IRRKLANG
    static int stepsbetweensound = 0;

    // Update sound pitch
    if (m_car_sound && m_vehicle->GetPowertrainAssembly()) {
        stepsbetweensound++;
        double engine_rpm = m_vehicle->GetPowertrainAssembly()->GetEngine()->GetMotorSpeed() * 60 / CH_2PI;
        double soundspeed = engine_rpm / (4000.);  // denominator: to guess
        if (soundspeed < 0.1)
            soundspeed = 0.1;
        if (stepsbetweensound > 20) {
            stepsbetweensound = 0;
            if (m_car_sound->getIsPaused())
                m_car_sound->setIsPaused(false);
            m_car_sound->setPlaybackSpeed((irrklang::ik_f32)soundspeed);
        }
    }
#endif
}

// -----------------------------------------------------------------------------
// Render the Irrlicht scene and additional visual elements.
// -----------------------------------------------------------------------------
void ChVehicleVisualSystemIrrlicht::Render() {
    ChVisualSystemIrrlicht::Render();

    if (m_renderStats)
        renderStats();

    // Allow derived classes to render additional graphical elements
    renderOtherGraphics();
}

// Render a linear gauge in the HUD.
void ChVehicleVisualSystemIrrlicht::renderLinGauge(const std::string& msg,
                                                   double factor,
                                                   bool sym,
                                                   int xpos,
                                                   int ypos,
                                                   int length,
                                                   int height) {
    irr::core::rect<s32> mclip(xpos, ypos, xpos + length, ypos + height);
    GetVideoDriver()->draw2DRectangle(irr::video::SColor(90, 60, 60, 60),
                                      irr::core::rect<s32>(xpos, ypos, xpos + length, ypos + height), &mclip);

    int left = sym ? (int)((length / 2 - 2) * std::min<>(factor, 0.0) + length / 2) : 2;
    int right = sym ? (int)((length / 2 - 2) * std::max<>(factor, 0.0) + length / 2) : (int)((length - 4) * factor + 2);

    GetVideoDriver()->draw2DRectangle(irr::video::SColor(255, 250, 200, 0),
                                      irr::core::rect<s32>(xpos + left, ypos + 2, xpos + right, ypos + height - 2),
                                      &mclip);
    if (sym) {
        GetVideoDriver()->draw2DLine(irr::core::vector2d<irr::s32>(xpos + length / 2, ypos + 2),
                                     irr::core::vector2d<irr::s32>(xpos + length / 2, ypos + height - 2),
                                     irr::video::SColor(255, 250, 0, 0));
    }

    GetMonospaceFont()->draw(msg.c_str(), irr::core::rect<s32>(xpos + 3, ypos + 3, xpos + length, ypos + height),
                             irr::video::SColor(255, 20, 20, 20));
}

// Render text in a box.
void ChVehicleVisualSystemIrrlicht::renderTextBox(const std::string& msg,
                                                  int xpos,
                                                  int ypos,
                                                  int length,
                                                  int height,
                                                  irr::video::SColor color) {
    irr::core::rect<s32> mclip(xpos, ypos, xpos + length, ypos + height);
    GetVideoDriver()->draw2DRectangle(irr::video::SColor(90, 60, 60, 60),
                                      irr::core::rect<s32>(xpos, ypos, xpos + length, ypos + height), &mclip);

    GetMonospaceFont()->draw(msg.c_str(), irr::core::rect<s32>(xpos + 3, ypos + 3, xpos + length, ypos + height),
                             color);
}

// Render stats for the vehicle and powertrain systems (render the HUD).
void ChVehicleVisualSystemIrrlicht::renderStats() {
    char msg[100];

    renderTextBox("Camera: " + m_camera->GetStateName(), m_HUD_x, m_HUD_y, 170, 15);

    double speed = m_vehicle->GetSpeed();
    snprintf(msg, sizeof(msg), "Speed(m/s): %+.2f", speed);
    renderLinGauge(std::string(msg), speed / 30, false, m_HUD_x, m_HUD_y + 40, 170, 15);

    // Display information from powertrain system
    const auto& powertrain = m_vehicle->GetPowertrainAssembly();

    if (powertrain) {
        const auto& engine = powertrain->GetEngine();
        const auto& transmission = powertrain->GetTransmission();

        double engine_rpm = engine->GetMotorSpeed() * 60 / CH_2PI;
        snprintf(msg, sizeof(msg), "Eng.speed(RPM): %+.2f", engine_rpm);
        renderLinGauge(std::string(msg), engine_rpm / 7000, false, m_HUD_x, m_HUD_y + 60, 170, 15);

        double engine_torque = engine->GetOutputMotorshaftTorque();
        snprintf(msg, sizeof(msg), "Eng.torque(Nm): %+.2f", engine_torque);
        renderLinGauge(std::string(msg), engine_torque / 600, false, m_HUD_x, m_HUD_y + 80, 170, 15);

        char msgT[5];
        int ngear = transmission->GetCurrentGear();
        int maxgear = transmission->GetMaxGear();
        if (transmission->IsAutomatic()) {
            auto transmission_auto = transmission->asAutomatic();

            double tc_slip = transmission_auto->GetTorqueConverterSlippage();
            snprintf(msg, sizeof(msg), "T.conv.slip: %+.2f", tc_slip);
            renderLinGauge(std::string(msg), tc_slip / 1, false, m_HUD_x, m_HUD_y + 100, 170, 15);

            double tc_torquein = transmission_auto->GetTorqueConverterInputTorque();
            snprintf(msg, sizeof(msg), "T.conv.in(Nm): %+.2f", tc_torquein);
            renderLinGauge(std::string(msg), tc_torquein / 600, false, m_HUD_x, m_HUD_y + 120, 170, 15);

            double tc_torqueout = transmission_auto->GetTorqueConverterOutputTorque();
            snprintf(msg, sizeof(msg), "T.conv.out(Nm): %+.2f", tc_torqueout);
            renderLinGauge(std::string(msg), tc_torqueout / 600, false, m_HUD_x, m_HUD_y + 140, 170, 15);

            double tc_rpmout = transmission_auto->GetTorqueConverterOutputSpeed() * 60 / CH_2PI;
            snprintf(msg, sizeof(msg), "T.conv.out(RPM): %+.2f", tc_rpmout);
            renderLinGauge(std::string(msg), tc_rpmout / 7000, false, m_HUD_x, m_HUD_y + 160, 170, 15);
            switch (transmission_auto->GetShiftMode()) {
                case ChAutomaticTransmission::ShiftMode::AUTOMATIC:
                    snprintf(msgT, sizeof(msgT), "[A]");
                    break;
                case ChAutomaticTransmission::ShiftMode::MANUAL:
                    snprintf(msgT, sizeof(msgT), "[M]");
                    break;
                default:
                    snprintf(msgT, sizeof(msgT), "  ");
                    break;
            }
            ChAutomaticTransmission::DriveMode drivemode = transmission_auto->GetDriveMode();
            switch (drivemode) {
                case ChAutomaticTransmission::DriveMode::FORWARD:
                    snprintf(msg, sizeof(msg), "%s Gear: Forward %d", msgT, ngear);
                    break;
                case ChAutomaticTransmission::DriveMode::NEUTRAL:
                    snprintf(msg, sizeof(msg), "%s Gear: Neutral", msgT);
                    break;
                case ChAutomaticTransmission::DriveMode::REVERSE:
                    snprintf(msg, sizeof(msg), "%s Gear: Reverse", msgT);
                    break;
                default:
                    snprintf(msg, sizeof(msg), "Gear:");
                    break;
            }
        } else if (transmission->IsManual()) {
            snprintf(msg, sizeof(msg), "[M] Gear: %d", ngear);
        } else {
            snprintf(msgT, sizeof(msgT), "[?]");
        }
        renderLinGauge(std::string(msg), (double)ngear / (double)maxgear, false, m_HUD_x, m_HUD_y + 180, 170, 15);
    }

    // Display information from driver system.
    int ypos = m_HUD_y;
    snprintf(msg, sizeof(msg), "Steering: %+.2f", m_steering);
    renderLinGauge(std::string(msg), -m_steering, true, m_HUD_x + 190, ypos, 170, 15);
    ypos += 20;

    if (powertrain && powertrain->GetTransmission()->IsManual()) {
        snprintf(msg, sizeof(msg), "Clutch: %+.2f", m_clutch * 100.);
        renderLinGauge(std::string(msg), m_clutch, false, m_HUD_x + 190, ypos, 170, 15);
        ypos += 20;
    }

    snprintf(msg, sizeof(msg), "Throttle: %+.2f", m_throttle * 100.);
    renderLinGauge(std::string(msg), m_throttle, false, m_HUD_x + 190, ypos, 170, 15);
    ypos += 20;

    snprintf(msg, sizeof(msg), "Braking: %+.2f", m_braking * 100.);
    renderLinGauge(std::string(msg), m_braking, false, m_HUD_x + 190, ypos, 170, 15);

    // Display current global location
    auto pos = m_vehicle->GetPos();
    snprintf(msg, sizeof(msg), "x: %+.1f", pos.x());
    renderTextBox(msg, m_HUD_x + 190, m_HUD_y + 80, 170, 15);
    snprintf(msg, sizeof(msg), "y: %+.1f", pos.y());
    renderTextBox(msg, m_HUD_x + 190, m_HUD_y + 95, 170, 15);
    snprintf(msg, sizeof(msg), "z: %+.1f", pos.z());
    renderTextBox(msg, m_HUD_x + 190, m_HUD_y + 110, 170, 15);

    // Display current simulation time.
    snprintf(msg, sizeof(msg), "Simulation time %.2f", m_vehicle->GetChTime());
    renderTextBox(msg, m_HUD_x + 190, m_HUD_y + 140, 170, 15, irr::video::SColor(255, 255, 255, 255));

    // Display estimated RTF values
    snprintf(msg, sizeof(msg), "RTF %3.2f (simulation)", GetRTF());
    renderTextBox(msg, m_HUD_x + 190, m_HUD_y + 160, 170, 15, irr::video::SColor(255, 255, 255, 255));
    snprintf(msg, sizeof(msg), "RTF %3.2f (step)", GetStepRTF());
    renderTextBox(msg, m_HUD_x + 190, m_HUD_y + 180, 170, 15, irr::video::SColor(255, 255, 255, 255));

    // Allow derived classes to display additional information (e.g. driveline)
    renderOtherStats(m_HUD_x, m_HUD_y + 210);
}

// -----------------------------------------------------------------------------

ChJoystickAxisIRR::ChJoystickAxisIRR()
    : id(-1), name("Unknown"), axis(Axis::NONE), min(0), max(1), scaled_min(0), scaled_max(1), value(0) {}

double ChJoystickAxisIRR::GetValue(const irr::SEvent::SJoystickEvent& joystickEvent) {
    if (joystickEvent.Joystick == id) {
        // Scale raw_value fromm [scaled_min, scaled_max] to [min, max] range
        value = (joystickEvent.Axis[axis] - max) * (scaled_max - scaled_min) / (max - min) + scaled_max;
    }
    return value;
}

void ChJoystickAxisIRR::Read(rapidjson::Document& d, const std::string& elementName, bool dbg_print) {
    auto element = elementName.c_str();
    if (d.HasMember(element) && d[element].IsObject()) {
        name = d[element]["name"].GetString();
        id = 0;  // default to first attached controller
        axis = (Axis)d[element]["axis"].GetInt();
        min = d[element]["min"].GetDouble();
        max = d[element]["max"].GetDouble();
        scaled_min = d[element]["scaled_min"].GetDouble();
        scaled_max = d[element]["scaled_max"].GetDouble();
        value = min;
    } else if (dbg_print) {
        std::cerr << "Expected a joystick axis definition for " << elementName << " but did not find one." << std::endl;
    }
}

ChJoystickButtonIRR::ChJoystickButtonIRR()
    : id(-1), name("Unknown"), button(-1), buttonPressed(false), buttonPressedCount(0) {}

bool ChJoystickButtonIRR::IsPressed(const irr::SEvent::SJoystickEvent& joystickEvent, bool continuous) {
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

void ChJoystickButtonIRR::Read(rapidjson::Document& d, const std::string& elementName, bool dbg_print) {
    auto element = elementName.c_str();
    if (d.HasMember(element) && d[element].IsObject()) {
        id = 0;  // default to first attached controller
        name = d[element]["name"].GetString();
        button = d[element]["button"].GetInt();
    } else if (dbg_print) {
        std::cerr << "Expected a joystick button definition for " << elementName << " but did not find one."
                  << std::endl;
    }
}

ChJoystickIRR::ChJoystickIRR(ChVehicleVisualSystemIrrlicht* vsys)
    : m_vsys(vsys),
      joystick_debug(false),
      joystick_debug_frame(0),
      m_joystick_proccess_frame(0),
      callback_button(-1),
      callback_function(nullptr),
      joystick_file(GetDataFile("joystick/controller_Default.json")) {}

void ChJoystickIRR::Initialize() {
    // Activate joysticks, is any
    joystick_info = core::array<SJoystickInfo>();
    m_vsys->GetDevice()->activateJoysticks(joystick_info);

    if (joystick_info.size() == 0)
        return;

    rapidjson::Document d;
    ReadFileJSON(joystick_file, d);

    if (d.IsNull()) {
        std::cerr << "Error reading joystick configuration file " << joystick_file << "\n"
                  << "Joystick support disabled" << std::endl;
        return;
    }

    // Read specification for individual axes and buttons.
    // For any axis/button listed in the specification file, default to first joystick
    steerAxis.Read(d, "steering", joystick_debug);
    throttleAxis.Read(d, "throttle", joystick_debug);
    brakeAxis.Read(d, "brake", joystick_debug);
    clutchAxis.Read(d, "clutch", joystick_debug);
    shiftUpButton.Read(d, "shiftUp", joystick_debug);
    shiftDownButton.Read(d, "shiftDown", joystick_debug);
    gearReverseButton.Read(d, "gearReverse", joystick_debug);
    gear1Button.Read(d, "gear1", joystick_debug);
    gear2Button.Read(d, "gear2", joystick_debug);
    gear3Button.Read(d, "gear3", joystick_debug);
    gear4Button.Read(d, "gear4", joystick_debug);
    gear5Button.Read(d, "gear5", joystick_debug);
    gear6Button.Read(d, "gear6", joystick_debug);
    gear7Button.Read(d, "gear7", joystick_debug);
    gear8Button.Read(d, "gear8", joystick_debug);
    gear9Button.Read(d, "gear9", joystick_debug);
    toggleManualGearboxButton.Read(d, "toggleManualGearbox", joystick_debug);

    // Loop over available controllers and distribute axes per controller if specified
    for (u32 id = 0; id < joystick_info.size(); ++id) {
        const auto& jinfo = joystick_info[id];
        std::cout << "Joystick " << id << ":\n";
        std::cout << "\tName: '" << jinfo.Name.c_str() << "'\n";
        std::cout << "\tAxes: " << jinfo.Axes << "\n";
        std::cout << "\tButtons: " << jinfo.Buttons << "\n";
        std::cout << "\tHat is: ";
        switch (jinfo.PovHat) {
            case SJoystickInfo::POV_HAT_PRESENT:
                std::cout << "present\n";
                break;
            case SJoystickInfo::POV_HAT_ABSENT:
                std::cout << "absent\n";
                break;
            case SJoystickInfo::POV_HAT_UNKNOWN:
            default:
                std::cout << "unknown\n";
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

}  // end namespace vehicle
}  // end namespace chrono
