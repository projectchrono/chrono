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
// Metal-native equivalent of ChFilterVisualize: opens a Cocoa/Metal window and
// displays the RGBA8 sensor feed live as the simulation runs (the OpenGL-based
// ChFilterVisualize is unavailable on macOS). Add it after the render filter.
// =============================================================================

#ifndef CH_FILTER_METAL_VISUALIZE_H
#define CH_FILTER_METAL_VISUALIZE_H

#include <memory>
#include <string>
#include <vector>

#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/sensors/ChSensorBuffer.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_metal
/// @{

class CH_SENSOR_API ChFilterMetalVisualize : public ChFilter {
  public:
    ChFilterMetalVisualize(int w, int h, std::string name = "Metal Camera");
    ~ChFilterMetalVisualize() override;

    void Apply() override;
    void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) override;

    /// False once the user has closed the window (before the window opens, returns true).
    bool WindowOpen() const;

    /// Enable interactive orbit control: left-drag rotates the camera around the
    /// target, scroll wheel zooms. The filter then owns the sensor's offset pose,
    /// so the demo should NOT also set it. `target` is in the sensor's parent-body
    /// frame (e.g. (0,0,1) ~ the middle of a vehicle); `dist` is the start radius.
    void EnableOrbitControl(float dist = 8.0f, float target_x = 0.0f, float target_y = 0.0f, float target_z = 1.0f);

  private:
    struct Impl;
    Impl* p = nullptr;
    // The filter accepts whichever host buffer the sensor produces and colorizes
    // it to RGBA8 for display.
    std::shared_ptr<SensorHostRGBA8Buffer> m_rgba8;
    std::shared_ptr<SensorHostSemanticBuffer> m_semantic;
    std::shared_ptr<SensorHostDIBuffer> m_di;
    std::vector<unsigned char> m_tmp;  // RGBA8 conversion scratch
    int m_w, m_h;
    std::string m_win_name;
    ChSensor* m_sensor = nullptr;  // non-owning; used to drive the offset pose in orbit mode
    bool m_orbit = false;
};

/// @} sensor_metal
}  // namespace sensor
}  // namespace chrono

#endif
