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

#ifndef CH_IRRGUIDRIVER_H
#define CH_IRRGUIDRIVER_H


#include "physics/ChSystem.h"
#include "unit_IRRLICHT/ChIrrApp.h"

#include "utils/ChChaseCamera.h"

#include "subsys/ChApiSubsys.h"
#include "subsys/ChDriver.h"
#include "subsys/ChVehicle.h"

namespace chrono {

class CH_SUBSYS_API ChIrrGuiDriver : public ChDriver, public irr::IEventReceiver
{
public:
  ChIrrGuiDriver(irr::ChIrrApp&    app,
                 const ChVehicle&  car,
                 int               tlc_X = 740,
                 int               tlc_Y = 20);

  ~ChIrrGuiDriver() {}

  virtual bool OnEvent(const irr::SEvent& event);

  void CreateCamera(const ChVector<>&  ptOnChassis,
                    double             chaseDist,
                    double             chaseHeight);
  void UpdateCamera(double step_size);
  void DrawAll();

  void SetTerrainHeight(double height) { m_terrainHeight = height; }

private:

  void renderSprings();
  void renderLinks();
  void renderGrid();
  void renderStats();
  void renderLinGauge(std::string& msg, double factor, int xpos,int ypos, int lenght, int height =12);

  irr::ChIrrAppInterface&   m_app;
  const ChVehicle&          m_car;

  utils::ChChaseCamera      m_camera;

  double m_terrainHeight;

  irr::gui::IGUIStaticText* m_text_throttle;
  irr::gui::IGUIStaticText* m_text_steering;
  irr::gui::IGUIStaticText* m_text_speed;

  int  HUD_x;
  int  HUD_y;

};


} // end namespace chrono


#endif
