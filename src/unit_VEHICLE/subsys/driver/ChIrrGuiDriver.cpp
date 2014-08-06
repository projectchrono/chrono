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

#include "subsys/driver/ChIrrGuiDriver.h"

using namespace irr;

namespace chrono {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChIrrGuiDriver::ChIrrGuiDriver(ChIrrApp&         app,
                               const ChVehicle&  car,
                               int               tlc_X,
                               int               tlc_Y)
: m_app(app),
  m_car(car),
  m_terrainHeight(0),
  m_camera(car.GetChassis())
{
  app.SetUserEventReceiver(this);

  gui::IGUIStaticText* text_inputs = app.GetIGUIEnvironment()->addStaticText(
    L"", core::rect<s32>(tlc_X, tlc_Y, tlc_X + 200, tlc_Y + 75), true, false, 0, -1, true);
    

  text_inputs->setBackgroundColor(video::SColor(255, 200, 200, 200));

  m_text_throttle = app.GetIGUIEnvironment()->addStaticText(
    L"Throttle: 0",
    core::rect<s32>(10, 10, 150, 25), false, false, text_inputs);

  m_text_steering = app.GetIGUIEnvironment()->addStaticText(
    L"Steering: 0",
    core::rect<s32>(10, 30, 150, 45), false, false, text_inputs);

  m_text_speed = app.GetIGUIEnvironment()->addStaticText(
    L"Speed: 0",
    core::rect<s32>(10, 50, 150, 65), false, false, text_inputs);
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
    case KEY_DOWN:
      m_camera.Zoom(1);
      return true;
    case KEY_UP:
      m_camera.Zoom(-1);
      return true;
    }
  } else {
    char msg[100];

    switch (event.KeyInput.Key) {
    case KEY_KEY_A:
      setSteering(m_steering - 0.1, -1, 1);
      sprintf(msg, "Steering: %+.2f", m_steering);
      m_text_steering->setText(core::stringw(msg).c_str());
      return true;
    case KEY_KEY_D:
      setSteering(m_steering + 0.1, -1, 1);
      sprintf(msg, "Steering: %+.2f", m_steering);
      m_text_steering->setText(core::stringw(msg).c_str());
      return true;
    case KEY_KEY_W:
      setThrottle(m_throttle + 0.1, -1, 1);
      sprintf(msg, "Throttle: %+.2f", m_throttle*100.);
      m_text_throttle->setText(core::stringw(msg).c_str());
      return true;
    case KEY_KEY_S:
      setThrottle(m_throttle - 0.1, -1, 1);
      sprintf(msg, "Throttle: %+.2f", m_throttle*100.);
      m_text_throttle->setText(core::stringw(msg).c_str());
      return true;

    case KEY_KEY_1:
      m_camera.SetState(utils::ChChaseCamera::Chase);
      return true;
    case KEY_KEY_2:
      m_camera.SetState(utils::ChChaseCamera::Follow);
      return true;
    case KEY_KEY_3:
      m_camera.SetState(utils::ChChaseCamera::Track);
      return true;
    }
  }

  return false;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChIrrGuiDriver::CreateCamera(const ChVector<>& ptOnChassis,
                                  double            chaseDist,
                                  double            chaseHeight)
{
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

void ChIrrGuiDriver::UpdateCamera(double step_size)
{
  // Update the ChChaseCamera
  m_camera.Update(step_size);
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

void ChIrrGuiDriver::renderStats()
{
  char msg[100];
  sprintf(msg, "Speed: %+.2f", m_car.GetVehicleSpeed());
  m_text_speed->setText(core::stringw(msg).c_str());
}


} // end namespace chrono
