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
// Concrete SynVisualization class that handles Sensor visualization via a
// ChSensorManager. Provides several wrapper functions that setup commonly used
// cameras and views.
//
// =============================================================================

#ifndef SYN_SENSOR_VIS_H
#define SYN_SENSOR_VIS_H

#include "chrono_synchrono/visualization/SynVisualization.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/ChSensor.h"

#include <string>
#include <iostream>

using namespace chrono;
using namespace chrono::sensor;

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_visualization
/// @{

/// Concrete SynVisualization class that handles Sensor visualization via a ChSensorManager.
class SYN_API SynSensorVisualization : public SynVisualization {
  public:
    /// Constructs a sensor vis
    SynSensorVisualization();
    ~SynSensorVisualization() {}

    /// @brief Call ChSensorManager.Update() and .ReconstructScenes() if necessary
    virtual void Update(double step) override;

    /// Initialize the sensor visualizer
    /// Basically just adds the sensor to the sensor manager, if possible
    virtual void Initialize() override;

    void InitializeDefaultSensorManager(ChSystem* system);

    /// Creates and attaches a default chase (third person) camera sensor
    void InitializeAsDefaultChaseCamera(std::shared_ptr<ChBodyAuxRef> body_ref,
                                        unsigned int width = 1280,
                                        unsigned int height = 720,
                                        float fps = 30,
                                        std::string window_title = "Default Chase Camera Sensor");

    /// Creates and attaches a default birds eye view camera sensor
    void InitializeAsDefaultBirdsEyeCamera(std::shared_ptr<ChBodyAuxRef> body_ref,
                                           double z = 285,
                                           unsigned int width = 1280,
                                           unsigned int height = 720,
                                           float fps = 30,
                                           std::string window_title = "Default Camera Sensor");

    /// Add a ChFilterRGBA8Access to the attached sensor
    /// Must be a camera
    void AddFilterRGBA8Access();

    /// Add a ChFilterVisualize to the attached sensor
    /// Must be a camera or lidar
    void AddFilterVisualize(unsigned int w = 1280, unsigned int h = 720);

    /// Add a ChFilterSave to the attached sensor
    /// Must be a camera or lidar
    void AddFilterSave(std::string file_path = "");

    /// Set the ChSensor
    void SetSensor(std::shared_ptr<ChSensor> sensor) { m_sensor = sensor; }

    /// Get the ChSensor
    std::shared_ptr<ChSensor> GetSensor() { return m_sensor; }

    /// Set the ChSensorManager
    void SetSensorManager(std::shared_ptr<ChSensorManager> manager) { m_sensor_manager = manager; }

    /// Get the ChSensorManager
    std::shared_ptr<ChSensorManager> GetSensorManager() { return m_sensor_manager; }

    bool HasSensorManager() { return m_sensor_manager != nullptr; }

  private:
    std::shared_ptr<ChSensor> m_sensor;  ///< handle to wrapped sensor
    std::shared_ptr<ChSensorManager> m_sensor_manager;
    bool m_needs_reconstruction;
};

/// @} synchrono_visualization

}  // namespace synchrono
}  // namespace chrono

#endif  // SYN_SENSOR_VIS_H
