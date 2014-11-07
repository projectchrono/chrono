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
// Similar to ChIrrGuiST, but only worry about steering IO
//
// =============================================================================

#ifndef CH_IRRGUIDRIVER_H
#define CH_IRRGUIDRIVER_H


#include "physics/ChSystem.h"
#include "unit_IRRLICHT/ChIrrApp.h"

#include "ChronoT_config.h"

#include "utils/ChChaseCamera.h"

#include "subsys/ChApiSubsys.h"
#include "subsys/ChDriver.h"
#include "subsys/ChSuspensionTest.h"


namespace chrono {

class CH_SUBSYS_API ChIrrGuiST : public ChDriver, public irr::IEventReceiver
{
public:

  /// camera should be in the front of the vehicle, facing backward,
  ///  target center of suspensions.
  ///  Also, you can set the steer limits [-steer_limit, steer_limit]
  ChIrrGuiST(
    irr::ChIrrApp&      app,
    ChSuspensionTest&   tester,
    const ChVector<>&   ptOnChassis,
    double              chaseDist,
    double              chaseHeight,
    double              steer_limit = 1.0,
    double              shaker_limit = 0.10,  // min/max jounce/rebound
    int                 HUD_x = 740,
    int                 HUD_y = 20
    );

  ~ChIrrGuiST() {}

  virtual bool OnEvent(const irr::SEvent& event);

  virtual void Advance(double step);

  void DrawAll();

  void SetSteeringDelta(double delta) {m_steeringDelta = delta; }
  void SetPostDelta(double delta) {m_postDelta = delta; }
  void SetTerrainHeight(double height) { m_terrainHeight = height; }
  void SetStepsize(double val) { m_stepsize = val; }


  // Accessors
 
  double GetStepsize() const { return m_stepsize; }
  /// Get the left post z displacement
  double Get_post_z_L() const { return m_post_L_disp; }

  /// Get the right post z displacement
  double Get_post_z_R() const { return m_post_R_disp; }

private:

  // control the vertical displacement of shaker posts
  void SetShaker_L(double vertical_disp, double min_z, double max_z);
  void SetShaker_R(double vertical_disp, double min_z, double max_z);

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
  ChSuspensionTest&         m_tester;

  utils::ChChaseCamera      m_camera;

  double m_stepsize;

  double m_terrainHeight;
  double m_steeringDelta;
  double m_postDelta;
  double m_post_L_disp;
  double m_post_R_disp;
  double m_min_post_z;  // TODO: don't hardcode these
  double m_max_post_z;

  int  m_HUD_x;
  int  m_HUD_y;

};


} // end namespace chrono


#endif
