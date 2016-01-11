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
// Authors: Justin Madsen, Radu Serban
// =============================================================================
//
// Irrlicht-based GUI driver for the a vehicle. This class implements the
// functionality required by its base ChDriver class using keyboard inputs.
// As an Irrlicht event receiver, its OnEvent() callback is used to keep track
// and update the current driver inputs. As such it does not need to override
// the default no-op Update() virtual method.
//
// In addition, this class provides additional Irrlicht support for rendering:
//  - implements a custom camera (which follows the vehicle)
//  - provides support for rendering links, force elements, displaying stats,
//    etc.  In order to render these elements, call the its DrawAll() method
//    instead of ChIrrAppInterface::DrawAll().
//
// =============================================================================

#include <algorithm>

#include "subsys/driver/ChIrrGuiTrack.h"
#include "subsys/powertrain/TrackPowertrain.h"

using namespace irr;

namespace chrono {

// -----------------------------------------------------------------------------
ChIrrGuiTrack::ChIrrGuiTrack(irrlicht::ChIrrApp& app,
                             ChTrackVehicle& vehicle,
                             const ChVector<>& ptOnChassis,
                             double chaseDist,
                             double chaseHeight,
                             int HUD_x,
                             int HUD_y)
    : m_app(app),
      m_vehicle(vehicle),
      m_powertrain(vehicle.GetPowertrain(0)),
      m_HUD_x(HUD_x),
      m_HUD_y(HUD_y),
      m_terrainHeight(0),
      m_throttleDelta(1.0 / 50),
      m_steeringDelta(1.0 / 50),
      m_brakingDelta(1.0 / 50),
      m_dampingDelta(0),
      m_dampingVal(vehicle.GetShoePinDamping()),
      m_camera(vehicle.GetChassis()),
      m_stepsize(1e-3),
      ChDriverTrack(2) {
    app.SetUserEventReceiver(this);

    // Initialize the ChChaseCamera
    m_camera.Initialize(ptOnChassis, vehicle.GetLocalDriverCoordsys(), chaseDist, chaseHeight);
    ChVector<> cam_pos = m_camera.GetCameraPos();
    ChVector<> cam_target = m_camera.GetTargetPos();

    // Create and initialize the Irrlicht camera
    scene::ICameraSceneNode* camera = m_app.GetSceneManager()->addCameraSceneNode(
        m_app.GetSceneManager()->getRootSceneNode(), core::vector3df(0, 0, 0), core::vector3df(0, 0, 0));

    camera->setUpVector(core::vector3df(0, 1, 0));
    camera->setPosition(core::vector3df((f32)cam_pos.x, (f32)cam_pos.y, (f32)cam_pos.z));
    camera->setTarget(core::vector3df((f32)cam_target.x, (f32)cam_target.y, (f32)cam_target.z));
}

// set a damping value, clamped between
void ChIrrGuiTrack::SetDamping(double delta, double min_val, double max_val) {
    double val = m_dampingVal + delta;
    if (val <= min_val)
        val = min_val;
    if (val >= max_val)
        val = max_val;

    // update the Irr driver value for pin friction damping
    m_dampingVal = val;
    // update the chain system with the val
    m_vehicle.SetShoePinDamping(m_dampingVal);
}

// -----------------------------------------------------------------------------
bool ChIrrGuiTrack::OnEvent(const SEvent& event) {
    // Only interpret keyboard inputs.
    if (event.EventType != EET_KEY_INPUT_EVENT)
        return false;
    // Control one of two ways: using the FPS type camera:
    // A/D = left / right, corresponds to simultaneous increase & decrease in L/R throttle
    //     to achieve the desired turn.
    // W/S = forward/backward, so both L/R gear throttles increase, or decrease
    if (event.KeyInput.PressedDown) {
        switch (event.KeyInput.Key) {
            case KEY_KEY_A:
                // turn left, negative steer
                SetSteering(-m_steeringDelta);
                return true;
            case KEY_KEY_D:
                // turn right, pos. steer
                SetSteering(m_steeringDelta);
                return true;
            case KEY_KEY_W:
                // both throttles increase
                SetAllThrottle(m_throttleDelta);
                return true;
            case KEY_KEY_S:
                // both throttles decrease
                SetAllThrottle(-m_throttleDelta);
                return true;

            case KEY_KEY_E:
                // increase damping
                SetDamping(m_dampingDelta);
                return true;
            case KEY_KEY_Q:
                // reduce damping
                SetDamping(-m_dampingDelta);
                return true;

            // control each track throttle/brake individually
            // T/G = Left, Y/H = right
            case KEY_KEY_T:
                // throttle left up
                SetThrottle(0, m_throttle[0] + m_throttleDelta);
                return true;
            case KEY_KEY_G:
                // throttle left down
                SetThrottle(0, m_throttle[0] - m_throttleDelta);
                return true;
            case KEY_KEY_Y:
                // throttle right up
                SetThrottle(1, m_throttle[1] + m_throttleDelta);
                return true;
            case KEY_KEY_H:
                // throttle right down
                SetThrottle(1, m_throttle[1] - m_throttleDelta);

            case KEY_DOWN:
                m_camera.Zoom(1);
                return true;
            case KEY_UP:
                m_camera.Zoom(-1);
                return true;
            case KEY_LEFT:
                m_camera.Turn(1);
                return true;
            case KEY_RIGHT:
                m_camera.Turn(-1);
                return true;
        }

    } else {
        switch (event.KeyInput.Key) {
            case KEY_KEY_1:
                m_camera.SetState(utils::ChChaseCamera::Chase);
                return true;
            case KEY_KEY_2:
                m_camera.SetState(utils::ChChaseCamera::Follow);
                return true;
            case KEY_KEY_3:
                m_camera.SetState(utils::ChChaseCamera::Track);
                return true;
            case KEY_KEY_4:
                m_camera.SetState(utils::ChChaseCamera::Inside);
                return true;

            case KEY_KEY_Z:
                m_powertrain->SetDriveMode(TrackPowertrain::FORWARD);
                return true;
            case KEY_KEY_X:
                m_powertrain->SetDriveMode(TrackPowertrain::NEUTRAL);
                return true;
            case KEY_KEY_C:
                m_powertrain->SetDriveMode(TrackPowertrain::REVERSE);
                return true;

                // TODO: no LogConstraintViolation() func., yet
                // case KEY_KEY_V:
                //  m_vehicle->LogConstraintViolations();
                // return true;
        }
    }

    return false;
}

// -----------------------------------------------------------------------------
void ChIrrGuiTrack::Advance(double step) {
    // Update the ChChaseCamera: take as many integration steps as needed to
    // exactly reach the value 'step'
    double t = 0;
    while (t < step) {
        double h = std::min<>(m_stepsize, step - t);
        m_camera.Update(step);
        t += h;
    }

    // Update the Irrlicht camera
    ChVector<> cam_pos = m_camera.GetCameraPos();
    ChVector<> cam_target = m_camera.GetTargetPos();

    scene::ICameraSceneNode* camera = m_app.GetSceneManager()->getActiveCamera();

    camera->setPosition(core::vector3df((f32)cam_pos.x, (f32)cam_pos.y, (f32)cam_pos.z));
    camera->setTarget(core::vector3df((f32)cam_target.x, (f32)cam_target.y, (f32)cam_target.z));
}

// -----------------------------------------------------------------------------
void ChIrrGuiTrack::Advance(double step, const ChVector<>& cam_pos) {
    // Update the ChChaseCamera: take as many integration steps as needed to
    // exactly reach the value 'step'
    double t = 0;
    while (t < step) {
        double h = std::min<>(m_stepsize, step - t);
        m_camera.Update(step);
        t += h;
    }

    // in this case, just manually set the camera pos
    ChVector<> cam_target = m_camera.GetTargetPos();

    scene::ICameraSceneNode* camera = m_app.GetSceneManager()->getActiveCamera();

    camera->setPosition(core::vector3df((f32)cam_pos.x, (f32)cam_pos.y, (f32)cam_pos.z));
    camera->setTarget(core::vector3df((f32)cam_target.x, (f32)cam_target.y, (f32)cam_target.z));
}

// set the Irrlicht camera, and also the have the chaseCamera loc match.
void ChIrrGuiTrack::SetCameraPos(const ChVector<>& pos) {
    m_app.GetSceneManager()->getActiveCamera()->setPosition(core::vector3df((f32)pos.x, (f32)pos.y, (f32)pos.z));
    m_camera.SetCameraPos(pos);
}

// -----------------------------------------------------------------------------
void ChIrrGuiTrack::DrawAll(bool draw_normal) {
    // renderGrid();

    m_app.DrawAll();

    renderSprings();
    // renderLinks();
    if (draw_normal)
        renderContactShoeGear();

    renderStats();
}

void ChIrrGuiTrack::renderSprings() {
    auto ilink = m_app.GetSystem()->Get_linklist()->begin();
    for (; ilink != m_app.GetSystem()->Get_linklist()->end(); ++ilink) {
        if (ChLinkSpring* link = dynamic_cast<ChLinkSpring*>((*ilink).get_ptr())) {
            irrlicht::ChIrrTools::drawSpring(m_app.GetVideoDriver(), 0.05, link->GetEndPoint1Abs(),
                                             link->GetEndPoint2Abs(), video::SColor(255, 150, 20, 20), 80, 15, true);
        } else if (ChLinkSpringCB* link = dynamic_cast<ChLinkSpringCB*>((*ilink).get_ptr())) {
            irrlicht::ChIrrTools::drawSpring(m_app.GetVideoDriver(), 0.05, link->GetEndPoint1Abs(),
                                             link->GetEndPoint2Abs(), video::SColor(255, 150, 20, 20), 80, 15, true);
        }
    }
}

void ChIrrGuiTrack::renderLinks() {
    auto ilink = m_app.GetSystem()->Get_linklist()->begin();
    for (; ilink != m_app.GetSystem()->Get_linklist()->end(); ++ilink) {
        if (ChLinkDistance* link = dynamic_cast<ChLinkDistance*>((*ilink).get_ptr())) {
            irrlicht::ChIrrTools::drawSegment(m_app.GetVideoDriver(), link->GetEndPoint1Abs(), link->GetEndPoint2Abs(),
                                              video::SColor(255, 0, 20, 0), true);
        } else if (ChLinkRevoluteSpherical* link = dynamic_cast<ChLinkRevoluteSpherical*>((*ilink).get_ptr())) {
            irrlicht::ChIrrTools::drawSegment(m_app.GetVideoDriver(), link->GetPoint1Abs(), link->GetPoint2Abs(),
                                              video::SColor(255, 180, 0, 0), true);
        }
    }
}

void ChIrrGuiTrack::renderGrid() {
    ChCoordsys<> gridCsys(m_vehicle.GetChassisPos());
    // ,chrono::Q_from_AngAxis(-CH_C_PI_2, VECT_Z));

    irrlicht::ChIrrTools::drawGrid(m_app.GetVideoDriver(), 0.1, 0.1, 30, 30, gridCsys, video::SColor(70, 125, 125, 125), true);
}

// data gets appended when the specified shoe pin and gear are in contact,
//  and reportShoeGearContact() was called
void ChIrrGuiTrack::renderContactShoeGear(double lenScale, int chain_id) {
        // two perisistent contacts for the driveChain system
        for (int pc = 0; pc < 2; pc++) {
            // only plot if the force magitude is non-zero
            if (m_vehicle.Get_SG_Persistent_Fn(pc, chain_id).Length() > 0) {
                // draw persistent contact point red
                irrlicht::ChIrrTools::drawSegment(m_app.GetVideoDriver(),
                                                  m_vehicle.Get_SG_Persistent_PosAbs(pc, chain_id),
                                                  m_vehicle.Get_SG_Persistent_PosAbs(pc, chain_id) +
                                                      m_vehicle.Get_SG_Persistent_Fn(pc, chain_id) * lenScale,
                                                  video::SColor(200, 255, 60, 60), true);  // red
            }

            /*
            // all other contact points that aren't the tracked collision are green
            std::vector<ChVector<> >::const_iterator pos_iter;
            std::vector<ChVector<> >::const_iterator Fn_iter;
            for (pos_iter = m_vehicle.Get_SG_PosAbs_all(chain_id).begin(), Fn_iter =
            m_vehicle.Get_SG_Fn_all(chain_id).begin();
                pos_iter < m_vehicle.Get_SG_PosAbs_all(chain_id).end() && Fn_iter <
            m_vehicle.Get_SG_Fn_all(chain_id).end();
                 pos_iter++, Fn_iter++) {
                ChIrrTools::drawSegment(m_app.GetVideoDriver(), (*pos_iter), (*pos_iter) + (*Fn_iter) * lenScale,
                                        video::SColor(200, 20, 255, 20), true);  // green
            }
            */
    }
}

void ChIrrGuiTrack::renderLinGauge(const std::string& msg,
                                   double factor,
                                   bool sym,
                                   int xpos,
                                   int ypos,
                                   int length,
                                   int height) {
    irr::core::rect<s32> mclip(xpos, ypos, xpos + length, ypos + height);
    m_app.GetVideoDriver()->draw2DRectangle(irr::video::SColor(90, 60, 60, 60),
                                            irr::core::rect<s32>(xpos, ypos, xpos + length, ypos + height), &mclip);

    int left = sym ? (int)((length / 2 - 2) * std::min<>(factor, 0.0) + length / 2) : 2;
    int right = sym ? (int)((length / 2 - 2) * std::max<>(factor, 0.0) + length / 2) : (int)((length - 4) * factor + 2);

    m_app.GetVideoDriver()->draw2DRectangle(
        irr::video::SColor(255, 250, 200, 00),
        irr::core::rect<s32>(xpos + left, ypos + 2, xpos + right, ypos + height - 2), &mclip);

    irr::gui::IGUIFont* font = m_app.GetIGUIEnvironment()->getBuiltInFont();
    font->draw(msg.c_str(), irr::core::rect<s32>(xpos + 3, ypos + 3, xpos + length, ypos + height),
               irr::video::SColor(255, 20, 20, 20));
}

void ChIrrGuiTrack::renderTextBox(const std::string& msg, int xpos, int ypos, int length, int height) {
    irr::core::rect<s32> mclip(xpos, ypos, xpos + length, ypos + height);
    m_app.GetVideoDriver()->draw2DRectangle(irr::video::SColor(90, 60, 60, 60),
                                            irr::core::rect<s32>(xpos, ypos, xpos + length, ypos + height), &mclip);

    irr::gui::IGUIFont* font = m_app.GetIGUIEnvironment()->getBuiltInFont();
    font->draw(msg.c_str(), irr::core::rect<s32>(xpos + 3, ypos + 3, xpos + length, ypos + height),
               irr::video::SColor(255, 20, 20, 20));
}

void ChIrrGuiTrack::renderStats() {
    char msg[100];
    int y_pos = m_HUD_y + 10;
    sprintf(msg, "Camera mode: %s", m_camera.GetStateName().c_str());
    renderTextBox(std::string(msg), m_HUD_x, y_pos, 120, 15);

    // Left [0] and right [1] throttle
    sprintf(msg, "Throttle Right: %+.2f", m_throttle[1] * 100.);
    renderLinGauge(std::string(msg), m_throttle[1], false, m_HUD_x, y_pos += 20, 120, 15);

    sprintf(msg, "Throttle Left: %+.2f", m_throttle[0] * 100.);
    renderLinGauge(std::string(msg), m_throttle[0], false, m_HUD_x, y_pos += 20, 120, 15);

    // TODO: idler is not part of ChTrackVehicle, so can't be used here, yet.
    // idler spring displacement
    // sprintf(msg, "idler Right: %+.2f", m_vehicle.GetIdlerForce(0) );
    //  renderLinGauge(std::string(msg), m_vehicle.GetIdlerForce(0), false, m_HUD_x, y_pos += 20, 120, 15);
    // sprintf(msg, "idler Left: %+.2f", m_vehicle->GetIdlerForce(1) );
    // renderLinGauge(std::string(msg),  m_vehicle->GetIdlerForce(1), false, m_HUD_x, y_pos += 20, 120, 15);

    /*
    // TODO: Left and Right braking
    sprintf(msg, "Braking Left: %+.2f", m_braking[0]*100.);
    renderLinGauge(std::string(msg), m_braking[0], false, m_HUD_x, m_HUD_y + 80, 120, 15);

    sprintf(msg, "Braking Right: %+.2f", m_braking[1]*100.);
    renderLinGauge(std::string(msg), m_braking[1], false, m_HUD_x, m_HUD_y + 80, 120, 15);
    */

    double speed = m_vehicle.GetVehicleSpeed();
    sprintf(msg, "Speed: %+.2f", speed);
    renderLinGauge(std::string(msg), speed / 30, false, m_HUD_x, y_pos += 20, 120, 15);

    if (m_vehicle.GetNum_Engines() > 0) {
        double engine_rpm = m_powertrain->GetMotorSpeed() * 60 / chrono::CH_C_2PI;
        sprintf(msg, "Eng. RPM: %+.2f", engine_rpm);
        renderLinGauge(std::string(msg), engine_rpm / 7000, false, m_HUD_x, y_pos += 20, 120, 15);

        double engine_torque = m_powertrain->GetMotorTorque();
        sprintf(msg, "Eng. Nm: %+.2f", engine_torque);
        renderLinGauge(std::string(msg), engine_torque / 600, false, m_HUD_x, y_pos += 20, 120, 15);

        double tc_slip = m_powertrain->GetTorqueConverterSlippage();
        sprintf(msg, "T.conv. slip: %+.2f", tc_slip);
        renderLinGauge(std::string(msg), tc_slip / 1, false, m_HUD_x, y_pos += 20, 120, 15);

        double tc_torquein = m_powertrain->GetTorqueConverterInputTorque();
        sprintf(msg, "T.conv. in  Nm: %+.2f", tc_torquein);
        renderLinGauge(std::string(msg), tc_torquein / 600, false, m_HUD_x, y_pos += 20, 120, 15);

        double tc_torqueout = m_powertrain->GetTorqueConverterOutputTorque();
        sprintf(msg, "T.conv. out Nm: %+.2f", tc_torqueout);
        renderLinGauge(std::string(msg), tc_torqueout / 600, false, m_HUD_x, y_pos += 20, 120, 15);

        int ngear = m_powertrain->GetCurrentTransmissionGear();
        TrackPowertrain::DriveMode drivemode = m_powertrain->GetDriveMode();
        switch (drivemode) {
            case TrackPowertrain::FORWARD:
                sprintf(msg, "Gear: forward, n.gear: %d", ngear);
                break;
            case TrackPowertrain::NEUTRAL:
                sprintf(msg, "Gear: neutral");
                break;
            case TrackPowertrain::REVERSE:
                sprintf(msg, "Gear: reverse");
                break;
            default:
                sprintf(msg, "Gear:");
                break;
        }
        renderLinGauge(std::string(msg), (double)ngear / 4.0, false, m_HUD_x, y_pos += 20, 120, 15);
    }

    double torque = m_powertrain->GetOutputTorque();
    sprintf(msg, "Powertrain Output torque: %+.2f", torque);
    renderLinGauge(std::string(msg), torque / 5000., false, m_HUD_x, y_pos += 20, 120, 15);

    double gearSpeed = m_vehicle.GetSprocketSpeed(0);
    sprintf(msg, "Gear rot. vel, rad/sec: %+.2f", gearSpeed);
    renderLinGauge(std::string(msg), gearSpeed / 1000, false, m_HUD_x, y_pos += 20, 120, 15);

    sprintf(msg, "Damping coef: %+.2f", m_dampingVal);
    renderLinGauge(std::string(msg), m_dampingVal / 2.0, false, m_HUD_x, y_pos += 20, 120, 15);
}

}  // end namespace chrono
