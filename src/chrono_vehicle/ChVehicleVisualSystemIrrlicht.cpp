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

#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/ChVehicleVisualSystemIrrlicht.h"

using namespace irr;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Implementation of the custom Irrlicht event receiver for camera control
// -----------------------------------------------------------------------------
class ChChaseCameraEventReceiver : public irr::IEventReceiver {
  public:
    // Construct a custom event receiver.
    ChChaseCameraEventReceiver(ChVehicleVisualSystemIrrlicht* vsys) : m_vsys(vsys) {}

    // Implementation of the event processing method.
    // This function interprets keyboard inputs for controlling the chase camera in
    // the associated vehicle Irrlicht application.
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
// Construct a vehicle Irrlicht application.
// -----------------------------------------------------------------------------
ChVehicleVisualSystemIrrlicht::ChVehicleVisualSystemIrrlicht()
    : ChVisualSystemIrrlicht(), m_camera_control(nullptr), m_renderStats(true), m_HUD_x(600), m_HUD_y(20) {
    // Set default window size and title
    SetWindowSize(1000, 800);
    SetWindowTitle("Chrono::Vehicle");

    // Default camera uses Z up
    SetCameraVertical(CameraVerticalDir::Z);

#ifdef CHRONO_IRRKLANG
    m_sound_engine = 0;
    m_car_sound = 0;
#endif
}

ChVehicleVisualSystemIrrlicht::~ChVehicleVisualSystemIrrlicht() {
    delete m_camera_control;
}

void ChVehicleVisualSystemIrrlicht::AttachVehicle(ChVehicle* vehicle) {
    ChVehicleVisualSystem::AttachVehicle(vehicle);

    // Add the Irrlicht camera (controlled through the chase-cam) if already initialized
    if (GetDevice()) {
        ChVector<> cam_pos = m_camera->GetCameraPos();
        ChVector<> cam_target = m_camera->GetTargetPos();
        AddCamera(cam_pos, cam_target);
    }
}

void ChVehicleVisualSystemIrrlicht::Initialize() {
    ChVisualSystemIrrlicht::Initialize();

    // Adjust HUD location based on window size
    m_HUD_x = GetVideoDriver()->getScreenSize().Width - 380;

    // Create the event receiver for controlling the chase camera
    m_camera_control = new ChChaseCameraEventReceiver(this);
    AddUserEventReceiver(m_camera_control);

    // Add the Irrlicht camera (controlled through the chase-cam) if already attach to a vehicle
    if (m_vehicle) {
        ChVector<> cam_pos = m_camera->GetCameraPos();
        ChVector<> cam_target = m_camera->GetTargetPos();
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
            GetLog() << "Cannot start sound engine Irrklang \n";
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
    ChVector<> cam_pos = m_camera->GetCameraPos();
    ChVector<> cam_target = m_camera->GetTargetPos();

    GetActiveCamera()->setPosition(core::vector3dfCH(cam_pos));
    GetActiveCamera()->setTarget(core::vector3dfCH(cam_target));

#ifdef CHRONO_IRRKLANG
    static int stepsbetweensound = 0;

    // Update sound pitch
    if (m_car_sound && m_vehicle->GetPowertrain()) {
        stepsbetweensound++;
        double engine_rpm = m_vehicle->GetPowertrain()->GetMotorSpeed() * 60 / CH_C_2PI;
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

    sprintf(msg, "Camera:%s", m_camera->GetStateName().c_str());
    renderTextBox(std::string(msg), m_HUD_x, m_HUD_y, 170, 15);

    double speed = m_vehicle->GetSpeed();
    sprintf(msg, "Speed(m/s): %+.2f", speed);
    renderLinGauge(std::string(msg), speed / 30, false, m_HUD_x, m_HUD_y + 30, 170, 15);

    // Display information from powertrain system.

    auto powertrain = m_vehicle->GetPowertrain();
    if (powertrain) {
        double engine_rpm = powertrain->GetMotorSpeed() * 60 / CH_C_2PI;
        sprintf(msg, "Eng.speed(RPM): %+.2f", engine_rpm);
        renderLinGauge(std::string(msg), engine_rpm / 7000, false, m_HUD_x, m_HUD_y + 50, 170, 15);

        double engine_torque = powertrain->GetMotorTorque();
        sprintf(msg, "Eng.torque(Nm): %+.2f", engine_torque);
        renderLinGauge(std::string(msg), engine_torque / 600, false, m_HUD_x, m_HUD_y + 70, 170, 15);

        double tc_slip = powertrain->GetTorqueConverterSlippage();
        sprintf(msg, "T.conv.slip: %+.2f", tc_slip);
        renderLinGauge(std::string(msg), tc_slip / 1, false, m_HUD_x, m_HUD_y + 90, 170, 15);

        double tc_torquein = powertrain->GetTorqueConverterInputTorque();
        sprintf(msg, "T.conv.in(Nm): %+.2f", tc_torquein);
        renderLinGauge(std::string(msg), tc_torquein / 600, false, m_HUD_x, m_HUD_y + 110, 170, 15);

        double tc_torqueout = powertrain->GetTorqueConverterOutputTorque();
        sprintf(msg, "T.conv.out(Nm): %+.2f", tc_torqueout);
        renderLinGauge(std::string(msg), tc_torqueout / 600, false, m_HUD_x, m_HUD_y + 130, 170, 15);

        double tc_rpmout = powertrain->GetTorqueConverterOutputSpeed() * 60 / CH_C_2PI;
        sprintf(msg, "T.conv.out(RPM): %+.2f", tc_rpmout);
        renderLinGauge(std::string(msg), tc_rpmout / 7000, false, m_HUD_x, m_HUD_y + 150, 170, 15);

        char msgT[5];
        switch (powertrain->GetTransmissionMode()) {
            case ChPowertrain::TransmissionMode::AUTOMATIC:
                sprintf(msgT, "[A]");
                break;
            case ChPowertrain::TransmissionMode::MANUAL:
                sprintf(msgT, "[M]");
                break;
            default:
                sprintf(msgT, "   ");
                break;
        }

        int ngear = powertrain->GetCurrentTransmissionGear();
        ChPowertrain::DriveMode drivemode = powertrain->GetDriveMode();
        switch (drivemode) {
            case ChPowertrain::DriveMode::FORWARD:
                sprintf(msg, "%s Gear: Forward %d", msgT, ngear);
                break;
            case ChPowertrain::DriveMode::NEUTRAL:
                sprintf(msg, "%s Gear: Neutral", msgT);
                break;
            case ChPowertrain::DriveMode::REVERSE:
                sprintf(msg, "%s Gear: Reverse", msgT);
                break;
            default:
                sprintf(msg, "Gear:");
                break;
        }
        renderLinGauge(std::string(msg), (double)ngear / 4.0, false, m_HUD_x, m_HUD_y + 170, 170, 15);
    }

    // Display information from driver system.
    sprintf(msg, "Steering: %+.2f", m_steering);
    renderLinGauge(std::string(msg), m_steering, true, m_HUD_x + 190, m_HUD_y + 30, 170, 15);

    sprintf(msg, "Throttle: %+.2f", m_throttle * 100.);
    renderLinGauge(std::string(msg), m_throttle, false, m_HUD_x + 190, m_HUD_y + 50, 170, 15);

    sprintf(msg, "Braking: %+.2f", m_braking * 100.);
    renderLinGauge(std::string(msg), m_braking, false, m_HUD_x + 190, m_HUD_y + 70, 170, 15);

    // Display current global location
    auto pos = m_vehicle->GetPos();
    sprintf(msg, "  x: %+.1f", pos.x());
    renderTextBox(msg, m_HUD_x + 190, m_HUD_y + 95, 170, 15);
    sprintf(msg, "  y: %+.1f", pos.y());
    renderTextBox(msg, m_HUD_x + 190, m_HUD_y + 110, 170, 15);
    sprintf(msg, "  z: %+.1f", pos.z());
    renderTextBox(msg, m_HUD_x + 190, m_HUD_y + 125, 170, 15);

    // Display current simulation time.
    sprintf(msg, "Time %.2f", m_vehicle->GetChTime());
    renderTextBox(msg, m_HUD_x + 190, m_HUD_y + 150, 170, 15, irr::video::SColor(255, 250, 0, 0));

    // Display estimated RTF
    if (m_vehicle->GetRTF() > 0) {
        sprintf(msg, "RTF %3.2f", GetSimulationRTF());
        renderTextBox(msg, m_HUD_x + 190, m_HUD_y + 170, 170, 15, irr::video::SColor(255, 250, 0, 0));
    }

    // Allow derived classes to display additional information (e.g. driveline)
    renderOtherStats(m_HUD_x, m_HUD_y + 200);
}

}  // end namespace vehicle
}  // end namespace chrono
