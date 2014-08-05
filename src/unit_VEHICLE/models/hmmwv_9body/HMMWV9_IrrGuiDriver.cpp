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
// Irrlicht-based GUI driver for the HMMWV9 model. This class implements the
// functionality required by its base ChDriver class using keyboard inputs.
// As an Irrlicht event receiver, its OnEvent() callback is used to keep track
// and update the current driver inputs. As such it does not need to override
// the default no-op Update() virtual method.
//
// In addition, this class provides additional Irrlicht support for the HMMWV9
// model:
//  - implements a custom camera (which follows the vehicle)
//  - provides support for rendering links, force elements, displaying stats,
//    etc.  In order to render these elements, call the its DrawAll() method
//    instead of ChIrrAppInterface::DrawAll().
//
// =============================================================================

#include "HMMWV9_IrrGuiDriver.h"

using namespace chrono;
using namespace irr;

namespace hmmwv9 {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV9_IrrGuiDriver::HMMWV9_IrrGuiDriver(ChIrrApp&              app,
                                         const HMMWV9_Vehicle&  car,
                                         int                    tlc_X,
                                         int                    tlc_Y)
: m_app(app),
  m_car(car),
  m_terrainHeight(0)
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
bool HMMWV9_IrrGuiDriver::OnEvent(const SEvent& event)
{
  // user hit a key, while not holding it down
  if (event.EventType == EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown)
  {
    char msg[100];

    switch (event.KeyInput.Key) {
    case KEY_KEY_A:
      setSteering(m_steering - 0.1);
      sprintf(msg, "Steering: %+.2f", m_steering);
      m_text_steering->setText(core::stringw(msg).c_str());
      return true;
    case KEY_KEY_D:
      setSteering(m_steering + 0.1);
      sprintf(msg, "Steering: %+.2f", m_steering);
      m_text_steering->setText(core::stringw(msg).c_str());
      return true;
    case KEY_KEY_W:
      setThrottle(m_throttle + 0.1);
      sprintf(msg, "Throttle: %+.2f", m_throttle*100.);
      m_text_throttle->setText(core::stringw(msg).c_str());
      return true;
    case KEY_KEY_S:
      setThrottle(m_throttle - 0.1);
      sprintf(msg, "Throttle: %+.2f", m_throttle*100.);
      m_text_throttle->setText(core::stringw(msg).c_str());
      return true;

    case KEY_DOWN:
      m_cam_multiplier *= 1.01;
      return true;
    case KEY_UP:
      m_cam_multiplier /= 1.01;
    }
  }

  return false;

}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV9_IrrGuiDriver::CreateCamera(const chrono::ChVector<>& cam_offset)
{
  m_cam_multiplier = 1;
  m_cam_offset = cam_offset;

  m_app.GetSceneManager()->addCameraSceneNode(
    m_app.GetSceneManager()->getRootSceneNode(),
    core::vector3df(0, 0, 0), core::vector3df(0, 0, 0));

  m_app.GetSceneManager()->getActiveCamera()->setUpVector(core::vector3df(0, 0, 1));

  updateCamera();
}

void HMMWV9_IrrGuiDriver::updateCamera()
{
  const ChVector<>& car_pos = m_car.GetChassisPos();
  ChVector<>        cam_pos = m_car.GetChassis()->GetCoord().TrasformLocalToParent(m_cam_multiplier * m_cam_offset);

  scene::ICameraSceneNode *camera = m_app.GetSceneManager()->getActiveCamera();

  camera->setPosition(core::vector3df((f32)cam_pos.x, (f32)cam_pos.y, (f32)cam_pos.z));
  camera->setTarget(core::vector3df((f32)car_pos.x, (f32)car_pos.y, (f32)car_pos.z));
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV9_IrrGuiDriver::DrawAll()
{
  updateCamera();

  renderGrid();

  m_app.DrawAll();

  renderSprings();
  renderLinks();
  renderStats();
}

void HMMWV9_IrrGuiDriver::renderSprings()
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

void HMMWV9_IrrGuiDriver::renderLinks()
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

void HMMWV9_IrrGuiDriver::renderGrid()
{
  ChCoordsys<> gridCsys(ChVector<>(0, 0, m_terrainHeight + 0.02),
    chrono::Q_from_AngAxis(-CH_C_PI_2, VECT_Z));

  ChIrrTools::drawGrid(m_app.GetVideoDriver(),
    0.5, 0.5, 100, 100,
    gridCsys,
    video::SColor(255, 80, 130, 255),
    true);
}

void HMMWV9_IrrGuiDriver::renderStats()
{
  char msg[100];
  sprintf(msg, "Speed: %+.2f", m_car.GetVehicleSpeed());
  m_text_speed->setText(core::stringw(msg).c_str());
}


} // end namespace hmmwv9
