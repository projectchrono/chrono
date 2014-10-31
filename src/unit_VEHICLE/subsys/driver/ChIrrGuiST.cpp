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
// Suspension tester GUI, only steering really matters
//
// =============================================================================

#include <algorithm>

#include "subsys/driver/ChIrrGuiST.h"

using namespace irr;

namespace chrono {


// -----------------------------------------------------------------------------
ChIrrGuiST::ChIrrGuiST(ChIrrApp&           app,
                               ChSuspensionTest&   tester,
                               const ChVector<>&   ptOnChassis,
                               double              chaseDist,
                               double              chaseHeight,
                               int                 HUD_x,
                               int                 HUD_y)
: m_app(app),
  m_tester(tester),
  m_HUD_x(HUD_x),
  m_HUD_y(HUD_y),
  m_terrainHeight(0),
  m_steeringDelta(1.0/50),
  m_camera(car.GetChassis()),
  m_stepsize(1e-3),
  m_throttle(0),
  m_braking(0)
{
  app.SetUserEventReceiver(this);

  // Initialize the ChChaseCamera
  m_camera.Initialize(ptOnChassis, car.GetLocalDriverCoordsys(), chaseDist, chaseHeight);
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
bool ChIrrGuiST::OnEvent(const SEvent& event)
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

    case KEY_KEY_V:
      m_car.LogConstraintViolations();
      return true;
    }

  }

  return false;
}

// -----------------------------------------------------------------------------
void ChIrrGuiST::Advance(double step)
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
void ChIrrGuiST::DrawAll()
{
  renderGrid();

  m_app.DrawAll();

  renderSprings();
  renderLinks();
  renderStats();
}

void ChIrrGuiST::renderSprings()
{
  std::list<chrono::ChLink*>::iterator ilink = m_app.GetSystem()->Get_linklist()->begin();
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

void ChIrrGuiST::renderLinks()
{
  std::list<chrono::ChLink*>::iterator ilink = m_app.GetSystem()->Get_linklist()->begin();
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

void ChIrrGuiST::renderGrid()
{
  ChCoordsys<> gridCsys(ChVector<>(0, 0, m_terrainHeight + 0.02),
                        chrono::Q_from_AngAxis(-CH_C_PI_2, VECT_Z));

  ChIrrTools::drawGrid(m_app.GetVideoDriver(),
                       0.5, 0.5, 100, 100,
                       gridCsys,
                       video::SColor(255, 80, 130, 255),
                       true);
}

void ChIrrGuiST::renderLinGauge(const std::string& msg,
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

void ChIrrGuiST::renderTextBox(const std::string& msg,
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

void ChIrrGuiST::renderStats()
{
  char msg[100];

  sprintf(msg, "Camera mode: %s", m_camera.GetStateName().c_str());
  renderTextBox(std::string(msg), m_HUD_x, m_HUD_y + 10, 120, 15);

  sprintf(msg, "Steering: %+.2f", m_steering);
  renderLinGauge(std::string(msg), m_steering, true, m_HUD_x, m_HUD_y + 40, 120, 15);
}


} // end namespace chrono
