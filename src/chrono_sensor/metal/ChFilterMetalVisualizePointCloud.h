// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Kyle Sha
// =============================================================================
// Metal 3D point-cloud viewer for lidar (the analog of the GL-based
// ChFilterVisualizePointCloud). Projects each lidar beam's range back to a 3D
// point and renders the cloud from an auto-orbiting camera in a Cocoa/Metal
// window. Add it after a lidar sensor.
// =============================================================================

#ifndef CH_FILTER_METAL_VISUALIZE_POINTCLOUD_H
#define CH_FILTER_METAL_VISUALIZE_POINTCLOUD_H

#include <memory>
#include <string>
#include <vector>

#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/sensors/ChSensorBuffer.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_metal
/// @{

class CH_SENSOR_API ChFilterMetalVisualizePointCloud : public ChFilter {
  public:
    ChFilterMetalVisualizePointCloud(int w, int h, float point_size = 3.f, std::string name = "Metal Lidar Point Cloud");
    ~ChFilterMetalVisualizePointCloud() override;

    void Apply() override;
    void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) override;
    bool WindowOpen() const;

  private:
    struct Impl;
    Impl* p = nullptr;
    std::shared_ptr<SensorHostDIBuffer> m_di;
    std::shared_ptr<SensorHostRadarBuffer> m_radar;  // radar returns (colored by Doppler)
    // lidar beam geometry captured from the sensor at Initialize
    float m_hfov = 0, m_vmin = 0, m_vmax = 0;
    unsigned int m_beam_w = 0, m_beam_h = 0;
    int m_w, m_h;
    float m_ptsize;
    std::string m_name;
    std::vector<float> m_pts;  // interleaved x,y,z,r,g,b
};

/// @} sensor_metal
}  // namespace sensor
}  // namespace chrono

#endif
