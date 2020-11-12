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

#include "chrono/core/ChLog.h"

#include "chrono_synchrono/visualization/SynSensorVisualization.h"

#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/ChLidarSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterSave.h"

namespace chrono {
namespace synchrono {

SynSensorVisualization::SynSensorVisualization()
    : SynVisualization(SynVisualization::SENSOR), m_needs_reconstruction(true) {}

void SynSensorVisualization::Update(double step) {
    if (m_needs_reconstruction) {
        m_sensor_manager->ReconstructScenes();
        m_needs_reconstruction = false;
    }
    m_sensor_manager->Update();
}

void SynSensorVisualization::Initialize() {
    if (m_sensor_manager == nullptr) {
        GetLog() << "SynSensorVisualization::Initialize: Sensor manager has not been initialized. Doing nothing..."
                 << "\n";
        return;
    } else if (m_sensor == nullptr) {
        GetLog() << "SynSensorVisualization::Initialize: Sensor has not been created or attached. Doing nothing..."
                 << "\n";
        return;
    }
    m_sensor_manager->AddSensor(m_sensor);
}

void SynSensorVisualization::InitializeDefaultSensorManager(ChSystem* system) {
    if (m_sensor_manager == nullptr) {
        // create the sensor manager and a camera
        m_sensor_manager = chrono_types::make_shared<ChSensorManager>(system);

        // add default lights
        m_sensor_manager->scene->AddPointLight({100, 100, 100}, {1, 1, 1}, 5000);
        m_sensor_manager->scene->AddPointLight({-100, -100, 100}, {1, 1, 1}, 5000);
    } else {
        GetLog() << "SensorVisualizationTypeManager::InitializeDefaultSensorManager: Sensor manager has already been "
                    "initialized. Doing nothing..."
                 << "\n";
    }
}

void SynSensorVisualization::InitializeAsDefaultChaseCamera(std::shared_ptr<ChBodyAuxRef> body_ref,
                                                            unsigned int width,
                                                            unsigned int height,
                                                            float fps,
                                                            std::string window_title) {
    m_sensor = chrono_types::make_shared<ChCameraSensor>(
        body_ref,                                                                      // body camera is attached to
        fps,                                                                           // update rate in Hz
        chrono::ChFrame<double>({-8, 0, 3}, Q_from_AngAxis(CH_C_PI / 20, {0, 1, 0})),  // offset pose
        width,                                                                         // image width
        height,                                                                        // image height
        CH_C_PI / 3);                                                                  // horizontal field of view
    m_sensor->SetLag(1 / fps);
    m_sensor->SetName(window_title);
}

void SynSensorVisualization::InitializeAsDefaultBirdsEyeCamera(std::shared_ptr<ChBodyAuxRef> body_ref,
                                                               double z,
                                                               unsigned int width,
                                                               unsigned int height,
                                                               float fps,
                                                               std::string window_title) {
    m_sensor = chrono_types::make_shared<ChCameraSensor>(
        body_ref,                                                                     // body camera is attached to
        fps,                                                                          // update rate in Hz
        chrono::ChFrame<double>({0, 0, z}, Q_from_AngAxis(CH_C_PI / 2., {0, 1, 0})),  // offset pose
        width,                                                                        // image width
        height,                                                                       // image height
        CH_C_PI / 3);                                                                 // horizontal field of view
    m_sensor->SetLag(1 / fps);
    m_sensor->SetName(window_title);
}

void SynSensorVisualization::AddFilterRGBA8Access() {
    auto camera = std::static_pointer_cast<ChCameraSensor>(m_sensor);
    if (camera) {
        camera->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    } else {
        GetLog() << "SynSensorVisualization::AddFilterRGBA8Access: Attached sensor is not a camera. Doing nothing..."
                 << "\n";
    }
}

void SynSensorVisualization::AddFilterVisualize(unsigned int w, unsigned int h) {
    auto camera = std::static_pointer_cast<ChCameraSensor>(m_sensor);
    if (camera) {
        camera->PushFilter(chrono_types::make_shared<ChFilterVisualize>(w, h));
        return;
    }
    auto lidar = std::static_pointer_cast<ChLidarSensor>(m_sensor);
    if (lidar) {
        lidar->PushFilter(chrono_types::make_shared<ChFilterVisualize>(w, h));
        return;
    }
    GetLog()
        << "SynSensorVisualization::AddFilterVisualize: Attached sensor is not a camera or a lidar. Doing nothing..."
        << "\n";
}

void SynSensorVisualization::AddFilterSave(std::string file_path) {
    auto camera = std::static_pointer_cast<ChCameraSensor>(m_sensor);
    if (camera) {
        camera->PushFilter(chrono_types::make_shared<ChFilterSave>(file_path));
        return;
    }
    auto lidar = std::static_pointer_cast<ChLidarSensor>(m_sensor);
    if (lidar) {
        lidar->PushFilter(chrono_types::make_shared<ChFilterSave>(file_path));
        return;
    }
    GetLog() << "SynSensorVisualization::AddFilterSave: Attached sensor is not a camera or a lidar. Doing nothing..."
             << "\n";
}
}  // namespace synchrono
}  // namespace chrono
