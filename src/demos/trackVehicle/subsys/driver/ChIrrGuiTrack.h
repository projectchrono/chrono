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
// Irrlicht-based GUI driver for the a tracked vehicle. This class implements the
// functionality required by its base ChDriverTrack class using keyboard inputs.
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

#ifndef CH_IRRGUITRACK_H
#define CH_IRRGUITRACK_H

#include "physics/ChSystem.h"
#include "utils/ChUtilsChaseCamera.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "subsys/ChTrackVehicle.h"
#include "subsys/ChApiSubsys.h"
#include "subsys/ChDriverTrack.h"
#include "subsys/powertrain/TrackPowertrain.h"

// #include "subsys/trackVehicle/DriveChain.h"

namespace chrono {
class CH_SUBSYS_API ChIrrGuiTrack : public ChDriverTrack, public irr::IEventReceiver {
  public:
    ChIrrGuiTrack(irrlicht::ChIrrApp& app,
                  ChTrackVehicle& vehicle,
                  const ChVector<>& ptOnChassis,
                  double chaseDist,
                  double chaseHeight,
                  int HUD_x = 740,
                  int HUD_y = 20);

    ~ChIrrGuiTrack() {}

    bool OnEvent(const irr::SEvent& event);

    void Advance(double step);

    /// manually set the camera position
    void Advance(double step, const ChVector<>& cam_pos);

    void DrawAll(bool draw_normal = false);

    void SetTerrainHeight(double height) { m_terrainHeight = height; }

    void SetThrottleDelta(double delta) { m_throttleDelta = delta; }
    void SetSteeringDelta(double delta) { m_steeringDelta = delta; }
    void SetBrakingDelta(double delta) { m_brakingDelta = delta; }
    void SetDampingDelta(double delta) { m_dampingDelta = delta; }

    void SetStepsize(double val) { m_stepsize = val; }

    // absolute c-sys
    void SetCameraPos(const ChVector<>& pos);

    // Accessors
    double GetStepsize() const { return m_stepsize; }

  private:
    void SetDamping(double delta_val, double min_val = 0, double max_val = 2);

    void renderSprings();
    void renderLinks();
    void renderGrid();
    void renderContactShoeGear(double lenScale = 1.0, int chain_id = 0);
    void renderStats();
    void renderLinGauge(const std::string& msg,
                        double factor,
                        bool sym,
                        int xpos,
                        int ypos,
                        int length = 120,
                        int height = 15);
    void renderTextBox(const std::string& msg, int xpos, int ypos, int length = 120, int height = 15);

    irrlicht::ChIrrAppInterface& m_app;
    ChTrackVehicle& m_vehicle;
    ChSharedPtr<TrackPowertrain> m_powertrain;

    utils::ChChaseCamera m_camera;

    double m_stepsize;

    double m_terrainHeight;
    double m_throttleDelta;
    double m_steeringDelta;
    double m_brakingDelta;

    // testing
    double m_dampingDelta;
    double m_dampingVal;  ///< current value of pin friction damping coef

    int m_HUD_x;
    int m_HUD_y;
};

}  // end namespace chrono

#endif
