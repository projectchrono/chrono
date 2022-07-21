// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Irrlicht-based visualization wrapper for vehicles.  This class is a derived
// from ChVisualSystemIrrlicht and provides the following functionality:
//   - rendering of the entire Irrlicht scene
//   - custom chase-camera (which can be controlled with keyboard)
//   - optional rendering of links, springs, stats, etc.
//
// =============================================================================

#ifndef CH_VEHICLE_VISUAL_SYSTEM_IRRLICHT_H
#define CH_VEHICLE_VISUAL_SYSTEM_IRRLICHT_H

#include <string>

#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtilsChaseCamera.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/ChVehicleVisualSystem.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/ChConfigVehicle.h"

#ifdef CHRONO_IRRKLANG
#include <irrKlang.h>
#endif

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_utils
/// @{

// Forward declaration
class ChChaseCameraEventReceiver;  ///< custom event receiver for chase-cam control

/// Customized Chrono Irrlicht visualization for vehicle simulations.
/// This class implements an Irrlicht-based visualization wrapper for vehicles.
/// This class extends ChVisualSystemIrrlicht and provides the following functionality:
///   - rendering of the entire Irrlicht scene
///   - custom chase-camera (which can be controlled with keyboard)
///   - optional rendering of links, springs, stats, etc.
class CH_VEHICLE_API ChVehicleVisualSystemIrrlicht : public ChVehicleVisualSystem, public irrlicht::ChVisualSystemIrrlicht {
  public:
    /// Construct a vehicle Irrlicht visualization system
    ChVehicleVisualSystemIrrlicht();

    virtual ~ChVehicleVisualSystemIrrlicht();

    /// Attach a vehicle to this Irrlicht vehicle visualization system.
    virtual void AttachVehicle(ChVehicle* vehicle) override;

    /// Set the upper-left point of HUD elements.
    void SetHUDLocation(int HUD_x, int HUD_y) {
        m_HUD_x = HUD_x;
        m_HUD_y = HUD_y;
    }

    /// Turn on/off rendering of stats (HUD).
    void EnableStats(bool val) { m_renderStats = val; }

    /// Turn on/off Irrklang sound generation.
    /// Note that this has an effect only if Irrklang support was enabled at configuration.
    void EnableSound(bool sound);

    virtual void Initialize() override;

    /// Render the Irrlicht scene and additional visual elements.
    virtual void Render() override;

    /// Render a horizontal grid at the specified location.
    void RenderGrid(const ChVector<>& loc, int num_divs, double delta);

    /// Render the specified reference frame.
    void RenderFrame(const ChFrame<>& frame, double axis_length = 1);

    /// Advance the dynamics of the chase camera.
    /// The integration of the underlying ODEs is performed using as many steps as needed to advance
    /// by the specified duration.
    void Advance(double step);

  protected:
    /// Render additional graphics
    virtual void renderOtherGraphics() {}

    /// Render additional vehicle information.
    virtual void renderOtherStats(int left, int top) {}

    void renderLinGauge(const std::string& msg,
                        double factor,
                        bool sym,
                        int xpos,
                        int ypos,
                        int length = 120,
                        int height = 15);

    void renderTextBox(const std::string& msg,
                       int xpos,
                       int ypos,
                       int length = 120,
                       int height = 15,
                       irr::video::SColor color = irr::video::SColor(255, 20, 20, 20));

    void renderStats();

    ChChaseCameraEventReceiver* m_camera_control;  ///< event receiver for chase-cam control

    bool m_renderStats;  ///< turn on/off rendering of stats

    int m_HUD_x;  ///< x-coordinate of upper-left corner of HUD elements
    int m_HUD_y;  ///< y-coordinate of upper-left corner of HUD elements

#ifdef CHRONO_IRRKLANG
    irrklang::ISoundEngine* m_sound_engine;  ///< Sound player
    irrklang::ISound* m_car_sound;           ///< Sound
#endif

    friend class ChChaseCameraEventReceiver;
    friend class ChIrrGuiDriver;
    friend class ChIrrGuiDriverSTR;
};

// @} vehicle_utils

}  // end namespace vehicle
}  // end namespace chrono

#endif
