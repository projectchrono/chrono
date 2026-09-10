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
// Base class for rendered sensors backed by the Metal ray-tracing engine.
// Mirrors ChVulkanSensor (and, historically, ChOptixSensor).
// =============================================================================

#ifndef CH_METAL_SENSOR_H
#define CH_METAL_SENSOR_H

#include "chrono_sensor/sensors/ChSensor.h"
#include "chrono_sensor/metal/ChMetalRTDefinitions.h"

namespace chrono {
namespace sensor {

/// @addtogroup sensor_sensors
/// @{

class CH_SENSOR_API ChMetalSensor : public ChSensor {
  public:
    ChMetalSensor(std::shared_ptr<ChBody> parent,
                  float updateRate,
                  ChFrame<double> offsetPose,
                  unsigned int w,
                  unsigned int h,
                  MetalPipelineType pipeline_type = MetalPipelineType::CAMERA);
    ~ChMetalSensor() override;

    MetalPipelineType GetPipelineType() const { return m_pipeline_type; }
    void SetPipelineType(MetalPipelineType pipeline_type) { m_pipeline_type = pipeline_type; }
    unsigned int GetWidth() const { return m_width; }
    unsigned int GetHeight() const { return m_height; }

  private:
    unsigned int m_width = 0;
    unsigned int m_height = 0;
    MetalPipelineType m_pipeline_type = MetalPipelineType::CAMERA;
};

/// @} sensor_sensors

}  // namespace sensor
}  // namespace chrono

#endif
