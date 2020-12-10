// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young, Jay Taves
// =============================================================================
//
// Concrete SynVisualization class that handles Irrlicht visualization via an
// Irrlicht app. Provides several wrapper functions that setup commonly used
// cameras and views.
//
// =============================================================================

#ifndef SYN_IRR_VEHICLE_VIS_H
#define SYN_IRR_VEHICLE_VIS_H

#include "chrono_synchrono/visualization/SynVisualization.h"

#include "chrono_synchrono/vehicle/SynWheeledVehicle.h"
#include "chrono_synchrono/vehicle/SynTrackedVehicle.h"

#include "chrono_vehicle/ChDriver.h"

#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
#include "chrono_vehicle/tracked_vehicle/utils/ChTrackedVehicleIrrApp.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_visualization
/// @{

/// Concrete SynVisualization class that handles Irrlicht visualization via an Irrlicht app.
class SYN_API SynIrrVehicleVisualization : public SynVisualization {
  public:
    SynIrrVehicleVisualization(std::shared_ptr<vehicle::ChDriver> driver = nullptr,
                               double step_size = 1e-3,
                               double render_step_size = 1. / 50);
    ~SynIrrVehicleVisualization() {}

    /// @brief Third-person view that will follow behind the vehicle
    void InitializeAsDefaultChaseCamera(std::shared_ptr<SynVehicle> vehicle,
                                        double following_distance = 6.0,
                                        std::wstring window_title = L"");

    /// @brief Third-person view that will follow behind the vehicle
    void InitializeAsDefaultTrackedChaseCamera(std::shared_ptr<SynTrackedVehicle> vehicle,
                                               double following_distance = 6.0,
                                               std::wstring window_title = L"");

    /// @brief Allow external creation and attachment of a IrrApp
    void AttachIrrApp(std::shared_ptr<vehicle::ChVehicleIrrApp> app);

    std::shared_ptr<vehicle::ChVehicleIrrApp> GetIrrApp() { return m_app; }

    /// @brief Advance the visualizer, taking note of any initialization and render step questions
    virtual void Update(double step) override;

    virtual void Initialize() override {}

    /// Set the handle to the ChDriver
    void SetDriver(std::shared_ptr<vehicle::ChDriver> driver) { m_driver = driver; }

    /// Whether images are saved
    void SetSave(bool save) { m_save = save; }
    bool GetSave() { return m_save; }

    /// Whether we visualize the Irrlicht view with a window
    void SetVisualize(bool visualize) { m_visualize = visualize; }
    bool GetVisualize() { return m_visualize; }

    /// How often rendering is done
    double GetRenderStepSize() { return m_render_step_size; }
    void SetRenderStepSize(double render_step_size) { m_render_step_size = render_step_size; }

    /// Simulation step size
    double GetStepSize() { return m_step_size; }
    void SetStepSize(double step_size) { m_step_size = step_size; }

  private:
    double m_step_size;         ///< simulation step size
    double m_render_step_size;  ///< time interval between two render frame [FPS]

    int m_step_number;   ///< times update has been called
    int m_render_steps;  ///< updates between each render frame

    bool m_visualize = true;  ///< if should visualize irrlicht window

    // Values for saving irrlicht window
    bool m_save = false;               ///< whether or not save the window to file
    std::string m_save_path = "img_";  ///< path to where these files should be saved
    int m_render_frame = 0;            ///< render frame number, used for file name when saving

    std::shared_ptr<vehicle::ChVehicleIrrApp> m_app;  ///< handle to irrlicht app

    std::shared_ptr<vehicle::ChDriver> m_driver;  ///< handle to vehicle driver
};

/// @} synchrono_visualization

}  // namespace synchrono
}  // namespace chrono

#endif
