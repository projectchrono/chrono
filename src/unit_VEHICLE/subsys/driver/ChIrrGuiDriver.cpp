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
// Authors: Radu Serban, Justin Madsen
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

#include "subsys/driver/ChIrrGuiDriver.h"
#include "subsys/powertrain/ChShaftsPowertrain.h"

using namespace irr;

namespace chrono {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChIrrGuiDriver::ChIrrGuiDriver(ChIrrApp&         app,
                               const ChVehicle&  car,
                               const ChVector<>& ptOnChassis,
                               double            chaseDist,
                               double            chaseHeight,
                               int               HUD_x,
                               int               HUD_y)
: m_app(app),
  m_car(car),
  m_HUD_x(HUD_x),
  m_HUD_y(HUD_y),
  m_terrainHeight(0),
  m_throttleDelta(1.0/50),
  m_steeringDelta(1.0/50),
  m_camera(car.GetChassis())
{
  app.SetUserEventReceiver(this);

  // Initialize the ChChaseCamera
  m_camera.Initialize(ptOnChassis, chaseDist, chaseHeight);
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
bool ChIrrGuiDriver::OnEvent(const SEvent& event)
{
  // Only interpret keyboard inputs.
  if (event.EventType != EET_KEY_INPUT_EVENT)
    return false;

  if (event.KeyInput.PressedDown) {

    switch (event.KeyInput.Key) {
    case KEY_KEY_A:
      setSteering(m_steering - m_steeringDelta);
      return true;
    case KEY_KEY_D:
      setSteering(m_steering + m_steeringDelta);
      return true;
    case KEY_KEY_W:
      setThrottle(m_throttle + m_throttleDelta);
      return true;
    case KEY_KEY_S:
      setThrottle(m_throttle - m_throttleDelta);
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

	case KEY_KEY_Z:
      if (ChShaftsPowertrain* powertrain = dynamic_cast<ChShaftsPowertrain*>(m_car.m_powertrain))
		  powertrain->SetDriveMode(ChShaftsPowertrain::eDriveMode::FORWARD);
      return true;
	case KEY_KEY_X:
      if (ChShaftsPowertrain* powertrain = dynamic_cast<ChShaftsPowertrain*>(m_car.m_powertrain))
		  powertrain->SetDriveMode(ChShaftsPowertrain::eDriveMode::NEUTRAL);
      return true;
	case KEY_KEY_C:
      if (ChShaftsPowertrain* powertrain = dynamic_cast<ChShaftsPowertrain*>(m_car.m_powertrain))
		  powertrain->SetDriveMode(ChShaftsPowertrain::eDriveMode::REVERSE);
      return true;
    }

  }

  return false;
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIrrGuiDriver::Advance(double step)
{
  // Update the ChChaseCamera
  m_camera.Update(step);
  ChVector<> cam_pos = m_camera.GetCameraPos();
  ChVector<> cam_target = m_camera.GetTargetPos();

  // Update the Irrlicht camera
  scene::ICameraSceneNode *camera = m_app.GetSceneManager()->getActiveCamera();

  camera->setPosition(core::vector3df((f32)cam_pos.x, (f32)cam_pos.y, (f32)cam_pos.z));
  camera->setTarget(core::vector3df((f32)cam_target.x, (f32)cam_target.y, (f32)cam_target.z));
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIrrGuiDriver::DrawAll()
{
  renderGrid();

  m_app.DrawAll();

  renderSprings();
  renderLinks();
  renderStats();
}

void ChIrrGuiDriver::renderSprings()
{
  std::list<chrono::ChLink*>::iterator ilink = m_app.GetSystem()->Get_linklist()->begin();
  for (; ilink != m_app.GetSystem()->Get_linklist()->end(); ++ilink) {
    if (ChLinkSpring* link = dynamic_cast<ChLinkSpring*>(*ilink)) {
      ChIrrTools::drawSpring(m_app.GetVideoDriver(), 0.05,
        link->GetEndPoint1Abs(),
        link->GetEndPoint2Abs(),
        video::SColor(255, 150, 20, 20), 80, 15, true);
    }
  }
}

void ChIrrGuiDriver::renderLinks()
{
  std::list<chrono::ChLink*>::iterator ilink = m_app.GetSystem()->Get_linklist()->begin();
  for (; ilink != m_app.GetSystem()->Get_linklist()->end(); ++ilink) {
    if (ChLinkDistance* link = dynamic_cast<ChLinkDistance*>(*ilink)) {
      ChIrrTools::drawSegment(m_app.GetVideoDriver(),
                              link->GetEndPoint1Abs(),
                              link->GetEndPoint2Abs(),
                              video::SColor(255, 0, 20, 0), true);
    }
  }
}

void ChIrrGuiDriver::renderGrid()
{
  ChCoordsys<> gridCsys(ChVector<>(0, 0, m_terrainHeight + 0.02),
                        chrono::Q_from_AngAxis(-CH_C_PI_2, VECT_Z));

  ChIrrTools::drawGrid(m_app.GetVideoDriver(),
                       0.5, 0.5, 100, 100,
                       gridCsys,
                       video::SColor(255, 80, 130, 255),
                       true);
}

void ChIrrGuiDriver::renderLinGauge(const std::string& msg,
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

void ChIrrGuiDriver::renderTextBox(const std::string& msg,
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

void ChIrrGuiDriver::renderStats()
{
  char msg[100];

  sprintf(msg, "Camera mode: %s", m_camera.GetStateName().c_str());
  renderTextBox(std::string(msg), m_HUD_x, m_HUD_y + 10, 120, 15);

  sprintf(msg, "Steering: %+.2f", m_steering);
  renderLinGauge(std::string(msg), m_steering, true, m_HUD_x, m_HUD_y + 40, 120, 15);

  sprintf(msg, "Throttle: %+.2f", m_throttle*100.);
  renderLinGauge(std::string(msg), m_throttle, false, m_HUD_x, m_HUD_y + 60, 120, 15);

  double speed = m_car.GetVehicleSpeed();
  sprintf(msg, "Speed: %+.2f", speed);
  renderLinGauge(std::string(msg), speed/30, false, m_HUD_x, m_HUD_y + 80, 120, 15);

  ChPowertrain* ptrain = m_car.m_powertrain;

  double engine_rpm = ptrain->GetMotorSpeed() * 60 / chrono::CH_C_2PI;
  sprintf(msg, "Eng. RPM: %+.2f", engine_rpm);
  renderLinGauge(std::string(msg), engine_rpm / 7000, false, m_HUD_x, m_HUD_y + 120, 120, 15);

  double engine_torque = ptrain->GetMotorTorque();
  sprintf(msg, "Eng. Nm: %+.2f", engine_torque);
  renderLinGauge(std::string(msg), engine_torque / 600, false, m_HUD_x, m_HUD_y + 140, 120, 15);

  if (ChShaftsPowertrain* powertrain = dynamic_cast<ChShaftsPowertrain*>(ptrain))
  {
    double tc_slip = powertrain->m_torqueconverter->GetSlippage();
    sprintf(msg, "T.conv. slip: %+.2f", tc_slip);
    renderLinGauge(std::string(msg), tc_slip / 1, false, m_HUD_x, m_HUD_y + 160, 120, 15);

    double tc_torquein = -powertrain->m_torqueconverter->GetTorqueReactionOnInput();
    sprintf(msg, "T.conv. in  Nm: %+.2f", tc_torquein);
    renderLinGauge(std::string(msg), tc_torquein / 600, false, m_HUD_x, m_HUD_y + 180, 120, 15);

    double tc_torqueout = powertrain->m_torqueconverter->GetTorqueReactionOnOutput();
    sprintf(msg, "T.conv. out Nm: %+.2f", tc_torqueout);
    renderLinGauge(std::string(msg), tc_torqueout / 600, false, m_HUD_x, m_HUD_y + 200, 120, 15);

    double torque_wheelL = -powertrain->m_rear_differential->GetTorqueReactionOn2();
    sprintf(msg, "Torque wheel L: %+.2f", torque_wheelL);
    renderLinGauge(std::string(msg), torque_wheelL / 5000, false, m_HUD_x, m_HUD_y + 220, 120, 15);

    double torque_wheelR = -powertrain->m_rear_differential->GetTorqueReactionOn3();
    sprintf(msg, "Torque wheel R: %+.2f", torque_wheelR);
    renderLinGauge(std::string(msg), torque_wheelR / 5000, false, m_HUD_x, m_HUD_y + 240, 120, 15);

	int ngear = powertrain->GetSelectedGear();
	ChShaftsPowertrain::eDriveMode drivemode = powertrain->GetDriveMode();
	switch (drivemode)
	{
		case ChShaftsPowertrain::FORWARD:
			sprintf(msg, "Gear: forward, n.gear: %d", ngear);
			break;
		case ChShaftsPowertrain::NEUTRAL:
			sprintf(msg, "Gear: neutral %d", ngear);
			break;
		case ChShaftsPowertrain::REVERSE:
			sprintf(msg, "Gear: reverse %d", ngear);
			break;
		default: 
			sprintf(msg, ""); 
			break;
	}
    renderLinGauge(std::string(msg), (double)ngear / 4.0, false, m_HUD_x, m_HUD_y + 260, 120, 15);
  }

}


} // end namespace chrono
