// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Eric Brandt, Asher Elmquist
// =============================================================================
//
// =============================================================================

#include "chrono_sensor/filters/ChFilterLidarIntensityClip.h"
#include "chrono_sensor/sensors/ChSensor.h"
#include "chrono_sensor/cuda/lidar_clip.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"

namespace chrono {
namespace sensor {

ChFilterLidarIntensityClip::ChFilterLidarIntensityClip(float intensity_thresh, float default_value, std::string name)
    : m_intensity_thresh(intensity_thresh), m_default_dist(default_value), ChFilter(name) {}

CH_SENSOR_API void ChFilterLidarIntensityClip::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                          std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);
    auto pDI = std::dynamic_pointer_cast<SensorDeviceDIBuffer>(bufferInOut);
    if (!pDI)
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    m_bufferInOut = pDI;

    if (auto pOpx = std::dynamic_pointer_cast<ChOptixSensor>(pSensor)) {
        m_cuda_stream = pOpx->GetCudaStream();
    } else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }

    m_bufferInOut = pDI;
}

CH_SENSOR_API void ChFilterLidarIntensityClip::Apply() {
    cuda_lidar_clip((float*)m_bufferInOut->Buffer.get(), (int)m_bufferInOut->Width, (int)m_bufferInOut->Height,
                    m_intensity_thresh, m_default_dist, m_cuda_stream);
}

}  // namespace sensor
}  // namespace chrono
