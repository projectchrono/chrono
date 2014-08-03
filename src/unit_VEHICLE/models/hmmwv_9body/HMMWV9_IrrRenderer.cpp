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
// Class for additional Irrlicht support for the HMMWV9 model. This class 
// implements a custom camera and provides support for rendering links and force
// elements. Its OnFrame() method must be invoked every time the Irrlicht scene
// is redrawn, after the call to ChIrrAppInterface::DrawAll().
//
// =============================================================================

#include "unit_IRRLICHT/ChIrrTools.h"

#include "HMMWV9_IrrRenderer.h"

using namespace chrono;
using namespace irr;


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV9_IrrRenderer::HMMWV9_IrrRenderer(ChIrrApp&                 app,
                                       const HMMWV9_Vehicle&     car,
                                       double                    terrainHeight,
                                       const chrono::ChVector<>& cam_pos,
                                       const chrono::ChVector<>& cam_targ,
                                       const chrono::ChVector<>& cam_offset)
: m_app(app),
  m_car(car),
  m_cam_offset(cam_offset),
  m_terrainHeight(terrainHeight)
{
  createCamera(cam_pos, cam_targ);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV9_IrrRenderer::OnFrame()
{
  updateCamera();
  renderSprings();
  renderLinks();
  renderGrid();
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV9_IrrRenderer::createCamera(const chrono::ChVector<>& pos,
                                      const chrono::ChVector<>& targ)
{
  core::vector3df pos_irr = core::vector3df((f32)pos.x, (f32)pos.y, (f32)pos.z);
  core::vector3df targ_irr = core::vector3df((f32)targ.x, (f32)targ.y, (f32)targ.z);

  m_app.GetSceneManager()->addCameraSceneNode(
    m_app.GetSceneManager()->getRootSceneNode(),
    pos_irr, targ_irr);

  m_app.GetSceneManager()->getActiveCamera()->setUpVector(core::vector3df(0, 0, 1));
}

void HMMWV9_IrrRenderer::updateCamera()
{
  const ChVector<>& car_pos = m_car.GetChassisPos();
  ChVector<>        cam_pos = car_pos + m_cam_offset;
  core::vector3df   cam_pos_irr((f32)cam_pos.x, (f32)cam_pos.y, (f32)cam_pos.z);

  scene::ICameraSceneNode *camera = m_app.GetSceneManager()->getActiveCamera();

  camera->setPosition(cam_pos_irr);
  camera->setTarget(core::vector3df((f32)car_pos.x, (f32)car_pos.y, (f32)car_pos.z));
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV9_IrrRenderer::renderSprings()
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

void HMMWV9_IrrRenderer::renderLinks()
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

void HMMWV9_IrrRenderer::renderGrid()
{
  ChCoordsys<> gridCsys(ChVector<>(0, 0, m_terrainHeight + 0.02),
                        chrono::Q_from_AngAxis(-CH_C_PI_2, VECT_Z));

  ChIrrTools::drawGrid(m_app.GetVideoDriver(),
                       0.5, 0.5, 100, 100,
                       gridCsys,
                       video::SColor(255, 80, 130, 255),
                       true);
}

