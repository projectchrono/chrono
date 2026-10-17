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
// Metal ray-tracing engine. Mirrors ChVulkanRTEngine's manager-facing contract
// (AssignSensor / ConstructScene / UpdateSensors) but targets Apple Metal.
// =============================================================================

#ifndef CH_METAL_RT_ENGINE_H
#define CH_METAL_RT_ENGINE_H

#include <memory>
#include <vector>

#include "chrono/physics/ChSystem.h"
#include "chrono_sensor/ChApiSensor.h"
#include "chrono_sensor/sensors/ChMetalSensor.h"
#include "chrono_sensor/metal/ChFilterMetalRTRender.h"
#include "chrono_sensor/metal/ChMetalRTDevice.h"
#include "chrono_sensor/metal/ChMetalRTScene.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_metal
/// @{

class CH_SENSOR_API ChMetalRTEngine {
  public:
    ChMetalRTEngine(ChSystem* sys, int device_id, int max_scene_reflections = 9, bool verbose = false, bool debug = false);
    ~ChMetalRTEngine();

    void AssignSensor(std::shared_ptr<ChMetalSensor> sensor);
    void UpdateSensors(std::shared_ptr<ChMetalRTScene> scene);
    void ConstructScene();

    int GetDevice() const { return static_cast<int>(m_device_id); }
    int GetNumSensor() const { return static_cast<int>(m_assigned_sensors.size()); }
    std::vector<std::shared_ptr<ChMetalSensor>> GetSensor() const { return m_assigned_sensors; }

    std::shared_ptr<ChMetalRTDevice> GetMetalDevice() const { return m_device; }

  private:
    ChSystem* m_system = nullptr;
    uint32_t m_device_id = 0;
    int m_recursions = 1;
    bool m_verbose = false;
    bool m_debug = false;

    std::shared_ptr<ChMetalRTDevice> m_device;
    std::shared_ptr<ChMetalRTScene> m_scene;
    std::vector<std::shared_ptr<ChMetalSensor>> m_assigned_sensors;
    std::vector<std::shared_ptr<ChFilterMetalRTRender>> m_assigned_renderers;
};

/// @} sensor_metal

}  // namespace sensor
}  // namespace chrono

#endif
