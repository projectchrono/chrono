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
                 const ChVector<>& ptOnChassis,
                 double            chaseDist,
                 double            chaseHeight,
                 int               HUD_x = 740,
                 int               HUD_y = 20);

  ~ChIrrGuiDriver() {}

  virtual bool OnEvent(const irr::SEvent& event);

  virtual void Advance(double step);

  void DrawAll();

  void SetTerrainHeight(double height) { m_terrainHeight = height; }

  void SetThrottleDelta(double delta)  { m_throttleDelta = delta; }
  void SetSteeringDelta(double delta)  { m_steeringDelta = delta; }
  void SetBrakingDelta (double delta)  { m_brakingDelta = delta; }
private:

  void renderSprings();
  void renderLinks();
  void renderGrid();
  void renderStats();
  void renderLinGauge(const std::string& msg,
                      double factor, bool sym,
                      int xpos, int ypos,
                      int length = 120, int height = 15);
  void renderTextBox(const std::string& msg,
                     int xpos, int ypos,
                     int length = 120, int height = 15);

  irr::ChIrrAppInterface&   m_app;
  const ChVehicle&          m_car;

  utils::ChChaseCamera      m_camera;

  double m_terrainHeight;
  double m_throttleDelta;
  double m_steeringDelta;
  double m_brakingDelta;

  int  m_HUD_x;
  int  m_HUD_y;

};


} // end namespace chrono


#endif
