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
#include "subsys/driveline/TrackDriveline.h"
#include "subsys/powertrain/TrackPowertrain.h"

using namespace irr;

namespace chrono {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChIrrGuiTrack::ChIrrGuiTrack(ChIrrApp&           app,
                            TrackVehicle&       vehicle,
                            TrackPowertrain&    powertrain,
                            const ChVector<>&   ptOnChassis,
                            double              chaseDist,
                            double              chaseHeight,
                            int                 HUD_x,
                            int                 HUD_y)
: m_app(app),
  m_vehicle(vehicle),
  m_powertrain(powertrain),
  m_HUD_x(HUD_x),
  m_HUD_y(HUD_y),
  m_terrainHeight(0),
  m_throttleDelta(1.0/50),
  m_steeringDelta(1.0/50),
  m_brakingDelta(1.0/50),
  m_camera(vehicle.GetChassis()),
  m_stepsize(1e-3),
  ChDriverTrack(2)
{
  app.SetUserEventReceiver(this);

  // Initialize the ChChaseCamera
  m_camera.Initialize(ptOnChassis, vehicle.GetLocalDriverCoordsys(), chaseDist, chaseHeight);
  ChVector<> cam_pos = m_camera.GetCameraPos();
  ChVector<> cam_target = m_camera.GetTargetPos();

  // Create and initialize the Irrlicht camera
  scene::ICameraSceneNode *camera = m_app.GetSceneManager()->addCameraSceneNode(
    m_app.GetSceneManager()->getRootSceneNode(),
    core::vector3df(0, 0, 0), core::vector3df(0, 0, 0));

  camera->setUpVector(core::vector3df(0, 0, 1));
  camera->setPosition(core::vector3df((f32)cam_pos.x, (f32)cam_pos.y, (f32)cam_pos.z));
  camera->setTarget(core::vector3df((f32)cam_target.x, (f32)cam_target.y, (f32)cam_target.z));

}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
bool ChIrrGuiTrack::OnEvent(const SEvent& event)
{
  // Only interpret keyboard inputs.
  if (event.EventType != EET_KEY_INPUT_EVENT)
    return false;

  if (event.KeyInput.PressedDown) {

    switch (event.KeyInput.Key) {
    case KEY_KEY_A:
      SetSteering(m_steering - m_steeringDelta);
      return true;
    case KEY_KEY_D:
      SetSteering(m_steering + m_steeringDelta);
      return true;
    case KEY_KEY_W:
      SetThrottle(m_throttle + m_throttleDelta);
      if (m_throttle > 0)
        SetBraking(m_braking - m_brakingDelta*3.0);
      return true;
    case KEY_KEY_S:
      SetThrottle(m_throttle - m_throttleDelta*3.0);
      if (m_throttle <= 0)
        SetBraking(m_braking + m_brakingDelta);
      return true;

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
      m_powertrain.SetDriveMode(TrackPowertrain::FORWARD);
      return true;
    case KEY_KEY_X:
      m_powertrain.SetDriveMode(TrackPowertrain::NEUTRAL);
      return true;
    case KEY_KEY_C:
      m_powertrain.SetDriveMode(TrackPowertrain::REVERSE);
      return true;

    case KEY_KEY_V:
      m_car.LogConstraintViolations();
      return true;
    }

  }

  return false;
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIrrGuiTrack::Advance(double step)
{
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

  scene::ICameraSceneNode *camera = m_app.GetSceneManager()->getActiveCamera();

  camera->setPosition(core::vector3df((f32)cam_pos.x, (f32)cam_pos.y, (f32)cam_pos.z));
  camera->setTarget(core::vector3df((f32)cam_target.x, (f32)cam_target.y, (f32)cam_target.z));

}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIrrGuiTrack::DrawAll()
{
  renderGrid();

  m_app.DrawAll();

  renderSprings();
  renderLinks();
  renderStats();
}

void ChIrrGuiTrack::renderSprings()
{
  std::vector<chrono::ChLink*>::iterator ilink = m_app.GetSystem()->Get_linklist()->begin();
  for (; ilink != m_app.GetSystem()->Get_linklist()->end(); ++ilink) {
    if (ChLinkSpring* link = dynamic_cast<ChLinkSpring*>(*ilink)) {
      ChIrrTools::drawSpring(m_app.GetVideoDriver(), 0.05,
        link->GetEndPoint1Abs(),
        link->GetEndPoint2Abs(),
        video::SColor(255, 150, 20, 20), 80, 15, true);
    }
    else if (ChLinkSpringCB* link = dynamic_cast<ChLinkSpringCB*>(*ilink)) {
      ChIrrTools::drawSpring(m_app.GetVideoDriver(), 0.05,
        link->GetEndPoint1Abs(),
        link->GetEndPoint2Abs(),
        video::SColor(255, 150, 20, 20), 80, 15, true);
    }
  }
}

void ChIrrGuiTrack::renderLinks()
{
  std::vector<chrono::ChLink*>::iterator ilink = m_app.GetSystem()->Get_linklist()->begin();
  for (; ilink != m_app.GetSystem()->Get_linklist()->end(); ++ilink) {
    if (ChLinkDistance* link = dynamic_cast<ChLinkDistance*>(*ilink)) {
      ChIrrTools::drawSegment(m_app.GetVideoDriver(),
                              link->GetEndPoint1Abs(),
                              link->GetEndPoint2Abs(),
                              video::SColor(255, 0, 20, 0), true);
    }
    else if (ChLinkRevoluteSpherical* link = dynamic_cast<ChLinkRevoluteSpherical*>(*ilink)) {
      ChIrrTools::drawSegment(m_app.GetVideoDriver(),
                              link->GetPoint1Abs(),
                              link->GetPoint2Abs(),
                              video::SColor(255, 180, 0, 0), true);
    }
  }
}

void ChIrrGuiTrack::renderGrid()
{
  ChCoordsys<> gridCsys(ChVector<>(0, 0, m_terrainHeight + 0.02),
                        chrono::Q_from_AngAxis(-CH_C_PI_2, VECT_Z));

  ChIrrTools::drawGrid(m_app.GetVideoDriver(),
                       0.5, 0.5, 100, 100,
                       gridCsys,
                       video::SColor(255, 80, 130, 255),
                       true);
}

void ChIrrGuiTrack::renderLinGauge(const std::string& msg,
                                    double factor, bool sym,
                                    int xpos, int ypos,
                                    int length, int height)
{
  irr::core::rect<s32> mclip(xpos,ypos, xpos+length, ypos+height);
  m_app.GetVideoDriver()->draw2DRectangle(
            irr::video::SColor(90,60,60,60),
            irr::core::rect<s32>(xpos,ypos, xpos+length, ypos+height),
            &mclip);

  int left  = sym ? (int)((length/2 - 2) * std::min<>(factor,0.0) + length/2) : 2;
  int right = sym ? (int)((length/2 - 2) * std::max<>(factor,0.0) + length/2) : (int)((length - 4)*factor + 2);

  m_app.GetVideoDriver()->draw2DRectangle(
            irr::video::SColor(255,250,200,00),
            irr::core::rect<s32>(xpos+left, ypos+2, xpos+right, ypos+height-2),
            &mclip);

  irr::gui::IGUIFont* font = m_app.GetIGUIEnvironment()->getBuiltInFont();
  font->draw(msg.c_str(),
             irr::core::rect<s32>(xpos+3,ypos+3, xpos+length, ypos+height),
             irr::video::SColor(255,20,20,20));
}

void ChIrrGuiTrack::renderTextBox(const std::string& msg,
                                   int xpos, int ypos,
                                   int length, int height)
{
  irr::core::rect<s32> mclip(xpos, ypos, xpos + length, ypos + height);
  m_app.GetVideoDriver()->draw2DRectangle(
    irr::video::SColor(90, 60, 60, 60),
    irr::core::rect<s32>(xpos, ypos, xpos + length, ypos + height),
    &mclip);

  irr::gui::IGUIFont* font = m_app.GetIGUIEnvironment()->getBuiltInFont();
  font->draw(msg.c_str(),
    irr::core::rect<s32>(xpos + 3, ypos + 3, xpos + length, ypos + height),
    irr::video::SColor(255, 20, 20, 20));
}

void ChIrrGuiTrack::renderStats()
{
  char msg[100];

  sprintf(msg, "Camera mode: %s", m_camera.GetStateName().c_str());
  renderTextBox(std::string(msg), m_HUD_x, m_HUD_y + 10, 120, 15);

  sprintf(msg, "Steering: %+.2f", m_steering);
  renderLinGauge(std::string(msg), m_steering, true, m_HUD_x, m_HUD_y + 40, 120, 15);

  sprintf(msg, "Throttle: %+.2f", m_throttle*100.);
  renderLinGauge(std::string(msg), m_throttle, false, m_HUD_x, m_HUD_y + 60, 120, 15);

  sprintf(msg, "Braking: %+.2f", m_braking*100.);
  renderLinGauge(std::string(msg), m_braking, false, m_HUD_x, m_HUD_y + 80, 120, 15);

  double speed = m_car.GetVehicleSpeed();
  sprintf(msg, "Speed: %+.2f", speed);
  renderLinGauge(std::string(msg), speed/30, false, m_HUD_x, m_HUD_y + 100, 120, 15);


  double engine_rpm = m_powertrain.GetMotorSpeed() * 60 / chrono::CH_C_2PI;
  sprintf(msg, "Eng. RPM: %+.2f", engine_rpm);
  renderLinGauge(std::string(msg), engine_rpm / 7000, false, m_HUD_x, m_HUD_y + 120, 120, 15);

  double engine_torque = m_powertrain.GetMotorTorque();
  sprintf(msg, "Eng. Nm: %+.2f", engine_torque);
  renderLinGauge(std::string(msg), engine_torque / 600, false, m_HUD_x, m_HUD_y + 140, 120, 15);

  double tc_slip = m_powertrain.GetTorqueConverterSlippage();
  sprintf(msg, "T.conv. slip: %+.2f", tc_slip);
  renderLinGauge(std::string(msg), tc_slip / 1, false, m_HUD_x, m_HUD_y + 160, 120, 15);

  double tc_torquein = m_powertrain.GetTorqueConverterInputTorque();
  sprintf(msg, "T.conv. in  Nm: %+.2f", tc_torquein);
  renderLinGauge(std::string(msg), tc_torquein / 600, false, m_HUD_x, m_HUD_y + 180, 120, 15);

  double tc_torqueout = m_powertrain.GetTorqueConverterOutputTorque();
  sprintf(msg, "T.conv. out Nm: %+.2f", tc_torqueout);
  renderLinGauge(std::string(msg), tc_torqueout / 600, false, m_HUD_x, m_HUD_y + 200, 120, 15);

  int ngear = m_powertrain.GetCurrentTransmissionGear();
  TrackPowertrain::DriveMode drivemode = m_powertrain.GetDriveMode();
  switch (drivemode)
  {
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
  renderLinGauge(std::string(msg), (double)ngear / 4.0, false, m_HUD_x, m_HUD_y + 220, 120, 15);

  // driveline data
  double torque;
  int axle = driveline->GetDrivenAxleIndexes()[0];

  torque = driveline->GetWheelTorque(ChWheelID(axle, LEFT));
  sprintf(msg, "Torque wheel L: %+.2f", torque);
  renderLinGauge(std::string(msg), torque / 5000, false, m_HUD_x, m_HUD_y + 260, 120, 15);

  torque = driveline->GetWheelTorque(ChWheelID(axle, RIGHT));
  sprintf(msg, "Torque wheel R: %+.2f", torque);
  renderLinGauge(std::string(msg), torque / 5000, false, m_HUD_x, m_HUD_y + 280, 120, 15);
 

}


} // end namespace chrono
