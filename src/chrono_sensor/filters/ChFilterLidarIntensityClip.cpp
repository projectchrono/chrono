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
#include "chrono_sensor/sensors/ChLidarSensor.h"

#ifdef CHRONO_HAS_OPTIX
#include "chrono_sensor/cuda/lidar_clip.cuh"
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#endif

namespace chrono {
namespace sensor {

ChFilterLidarIntensityClip::ChFilterLidarIntensityClip(float intensity_thresh, float default_value, std::string name)
    : m_intensity_thresh(intensity_thresh), m_default_dist(default_value), ChFilter(name) {}

void ChFilterLidarIntensityClip::Initialize(std::shared_ptr<ChSensor> pSensor,
                                            std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);
    auto pDI = std::dynamic_pointer_cast<SensorDeviceDIBuffer>(bufferInOut);
    if (!pDI)
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    m_bufferInOut = pDI;

    if (auto pLidar = std::dynamic_pointer_cast<ChLidarSensor>(pSensor)) {
#ifdef CHRONO_HAS_OPTIX
        m_cuda_stream = pLidar->GetCudaStream();
#endif
    } else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }
}

void ChFilterLidarIntensityClip::Apply() {
#ifdef CHRONO_HAS_OPTIX
    cuda_lidar_clip((float*)m_bufferInOut->Buffer.get(), (int)m_bufferInOut->Width, (int)m_bufferInOut->Height,
                    m_intensity_thresh, m_default_dist, m_cuda_stream);
#else
    const size_t count = static_cast<size_t>(m_bufferInOut->Width) * m_bufferInOut->Height *
                         (m_bufferInOut->Dual_return ? 2u : 1u);
    for (size_t i = 0; i < count; ++i) {
        if (m_bufferInOut->Buffer[i].intensity < m_intensity_thresh) {
            m_bufferInOut->Buffer[i].range = m_default_dist;
            m_bufferInOut->Buffer[i].intensity = 0.f;
        }
    }
#endif
}

}  // namespace sensor
}  // namespace chrono
