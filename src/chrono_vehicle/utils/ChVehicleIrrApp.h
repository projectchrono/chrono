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
// Authors: Radu Serban
// =============================================================================
//
// Irrlicht-based visualization wrapper for vehicles.  This class is a wrapper
// around a ChIrrApp object and provides the following functionality:
//   - rendering of the entire Irrlicht scene
//   - implements a custom chase-camera (which can be controlled with keyboard)
//   - optional rendering of links, springs, stats, etc.
//
// =============================================================================

#ifndef CH_VEHICLE_IRRAPP_H
#define CH_VEHICLE_IRRAPP_H

#include <string>

#include "chrono/physics/ChSystem.h"
#include "chrono/utils/ChUtilsChaseCamera.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPowertrain.h"
#include "chrono_vehicle/ChVehicle.h"

#ifdef CHRONO_IRRKLANG
#include <irrKlang.h>
#endif

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_utils
/// @{

// Forward declaration
class ChCameraEventReceiver;  ///< custom event receiver for chase-cam control

/// Customized Chrono Irrlicht application for vehicle visualization.
/// This class implements an Irrlicht-based visualization wrapper for vehicles.
/// This class is a wrapper around a ChIrrApp object and provides the following functionality:
///   - rendering of the entire Irrlicht scene
///   - implements a custom chase-camera (which can be controlled with keyboard)
///   - optional rendering of links, springs, stats, etc.
class CH_VEHICLE_API ChVehicleIrrApp : public irrlicht::ChIrrApp {
  public:
    /// Construct a vehicle Irrlicht application.
    ChVehicleIrrApp(
        ChVehicle* vehicle,        ///< pointer to the associated vehicle system
        ChPowertrain* powertrain,  /// pointer to the associated powertrain system
        const wchar_t* title = 0,  ///< window title
        irr::core::dimension2d<irr::u32> dims = irr::core::dimension2d<irr::u32>(1000, 800)  ///< window dimensions
        );

    virtual ~ChVehicleIrrApp();

    /// Create a skybox that has Z pointing up.
    /// Note that the default ChIrrApp::AddTypicalSky() uses Y up.
    void SetSkyBox();

    /// Set parameters for the underlying chase camera.
    void SetChaseCamera(const ChVector<>& ptOnChassis,  ///< tracked point on chassis body (in vehicle reference frame)
                        double chaseDist,               ///< chase distance (behind tracked point)
                        double chaseHeight              ///< chase height (above tracked point)
                        );
    /// Set the step size for integration of the chase-cam dynamics.
    void SetStepsize(double val) { m_stepsize = val; }
    /// Set camera position.
    /// Note that this forces the chase-cam in Track mode.
    void SetChaseCameraPosition(const ChVector<>& pos) { m_camera.SetCameraPos(pos); }
    /// Set camera zoom multipliers.
    void SetChaseCameraMultipliers(double minMult, double maxMult) { m_camera.SetMultLimits(minMult, maxMult); }

    /// Set the upper-left point of HUD elements.
    void SetHUDLocation(int HUD_x, int HUD_y) {
        m_HUD_x = HUD_x;
        m_HUD_y = HUD_y;
    }

    /// Turn on/off rendering of the grid.
    void EnableGrid(bool val) { m_renderGrid = val; }

    /// Turn on/off rendering of stats (HUD).
    void EnableStats(bool val) { m_renderStats = val; }

    /// Set the height at whoich the horizontal grid is rendered.
    void SetGridHeight(double height) { m_gridHeight = height; }

    /// Turn on/off Irrklang sound generation.
    /// Note that this has an effect only if Irrklang support was enabled at configuration.
    void EnableSound(bool sound);

    /// Render the Irrlicht scene and additional visual elements.
    virtual void DrawAll() override;

    /// Update information related to driver inputs.
    void Synchronize(const std::string& msg, double steering, double throttle, double braking);

    /// Advance the dynamics of the chase camera.
    /// The integration of the underlying ODEs is performed using as many steps as needed to advance
    /// by the specified duration.
    void Advance(double step);

    /// Save a snapshot of the last rendered frame to file.
    /// The file name extension determines the image format.
    void WriteImageToFile(const std::string& filename);

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

    ChVehicle* m_vehicle;        ///< pointer to the associated vehicle system
    ChPowertrain* m_powertrain;  ///< pointer to the associated powertrain system

  private:
    void renderGrid();
    void renderStats();

    utils::ChChaseCamera m_camera;            ///< chase camera
    ChCameraEventReceiver* m_camera_control;  ///< event receiver for chase-cam control

    double m_stepsize;  ///< integration step size for chase-cam dynamics

    bool m_renderGrid;     ///< turn on/off rendering of grid
    bool m_renderStats;    ///< turn on/off rendering of stats

    double m_gridHeight;  ///< height of grid

    int m_HUD_x;  ///< x-coordinate of upper-left corner of HUD elements
    int m_HUD_y;  ///< y-coordinate of upper-left corner of HUD elements

    std::string m_driver_msg;  ///< HUD message from driver system
    double m_steering;         ///< driver steering input
    double m_throttle;         ///< driver throttle input
    double m_braking;          ///< driver braking input

#ifdef CHRONO_IRRKLANG
    irrklang::ISoundEngine* m_sound_engine;  ///< Sound player
    irrklang::ISound* m_car_sound;           ///< Sound
#endif

    friend class ChCameraEventReceiver;
    friend class ChIrrGuiDriver;
    friend class ChIrrGuiDriverSTR;
};

// @} vehicle_utils

}  // end namespace vehicle
}  // end namespace chrono

#endif
