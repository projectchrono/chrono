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
//    etc.  Its DrawAll() method must be invoked every time the Irrlicht scene
//    is redrawn, after the call to ChIrrAppInterface::DrawAll().
//
// =============================================================================

#ifndef HMMWV9_IRRGUIDRIVER_H
#define HMMWV9_IRRGUIDRIVER_H


#include "physics/ChSystem.h"
#include "unit_IRRLICHT/ChIrrApp.h"

#include "ChDriver.h"

#include "HMMWV9_Vehicle.h"

namespace hmmwv9 {

class HMMWV9_IrrGuiDriver : public chrono::ChDriver, public irr::IEventReceiver
{
public:
  HMMWV9_IrrGuiDriver(irr::ChIrrApp&         app,
                      const HMMWV9_Vehicle&  car,
                      int                    tlc_X = 740,
                      int                    tlc_Y = 20);

  ~HMMWV9_IrrGuiDriver() {}

  virtual bool OnEvent(const irr::SEvent& event);

  void CreateCamera(const chrono::ChVector<>& cam_offset);
  void DrawAll();

  void SetTerrainHeight(double height) { m_terrainHeight = height; }

private:

  void updateCamera();
  void renderSprings();
  void renderLinks();
  void renderGrid();
  void renderStats();

  irr::ChIrrAppInterface&   m_app;
  const HMMWV9_Vehicle&     m_car;
  chrono::ChVector<>        m_cam_offset;

  double m_cam_multiplier;
  double m_terrainHeight;

  irr::gui::IGUIStaticText* m_text_throttle;
  irr::gui::IGUIStaticText* m_text_steering;
  irr::gui::IGUIStaticText* m_text_speed;

};


} // end namespace hmmwv9


#endif