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
// First filter in a Metal-rendered sensor's graph. Allocates the host output
// buffer and (in Apply) renders the scene with Metal into it. Mirrors
// ChFilterVulkanRTRender. Header is pure C++; Metal lives in the .mm.
// =============================================================================

#ifndef CH_FILTER_METAL_RT_RENDER_H
#define CH_FILTER_METAL_RT_RENDER_H

#include <memory>

#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/sensors/ChMetalSensor.h"
#include "chrono_sensor/sensors/ChSensorBuffer.h"
#include "chrono_sensor/metal/ChMetalRTDevice.h"
#include "chrono_sensor/metal/ChMetalRTScene.h"
#include "chrono_sensor/metal/ChMetalRTRenderer.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_metal
/// @{

class CH_SENSOR_API ChFilterMetalRTRender : public ChFilter {
  public:
    ChFilterMetalRTRender(std::shared_ptr<ChMetalRTDevice> device, std::shared_ptr<ChMetalRTScene> scene);
    ~ChFilterMetalRTRender() override;

    void Apply() override;
    void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) override;

    void SetRayRecursions(int rec) { m_ray_recursions = rec > 0 ? rec : 1; }
    void SetScene(std::shared_ptr<ChMetalRTScene> scene) { m_scene = std::move(scene); }

  private:
    std::shared_ptr<ChMetalRTDevice> m_device;
    std::shared_ptr<ChMetalRTScene> m_scene;
    std::weak_ptr<ChMetalSensor> m_sensor;
    std::shared_ptr<ChMetalRTRenderer> m_renderer;
    bool m_renderer_built = false;

    /// This render's RNG stream seed, from ChSensorManager::GetDeterministicSeed. Derived once in

    /// Initialize rather than per frame: the seed identifies the buffer, not the frame, and

    /// GetDeterministicSeed reads the clock when no fixed base seed is set, which would otherwise

    /// make every frame a different stream.

    unsigned long long m_rng_seed = 0;

    std::shared_ptr<SensorHostRGBA8Buffer> m_buffer_rgba8;
    std::shared_ptr<SensorHostRGBDHalf4Buffer> m_buffer_rgbd;  ///< PHYS_CAMERA: linear radiance + primary-hit depth
    std::shared_ptr<SensorHostDepthBuffer> m_buffer_depth;
    std::shared_ptr<SensorHostNormalBuffer> m_buffer_normal;
    std::shared_ptr<SensorHostSemanticBuffer> m_buffer_semantic;
    std::shared_ptr<SensorHostDIBuffer> m_buffer_di;
    std::shared_ptr<SensorHostRadarBuffer> m_buffer_radar;

    float m_time_stamp = 0.f;
    int m_ray_recursions = 1;

    friend class ChMetalRTEngine;
};

/// @} sensor_metal
}  // namespace sensor
}  // namespace chrono

#endif
