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

#include "chrono_sensor/filters/ChFilterLidarNoise.h"
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/cuda/lidar_noise.cuh"
#include "chrono_sensor/cuda/curand_utils.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#include <chrono>

namespace chrono {
namespace sensor {

ChFilterLidarNoiseXYZI::ChFilterLidarNoiseXYZI(float stdev_range,
                                               float stdev_v_angle,
                                               float stdev_h_angle,
                                               float stdev_intensity,
                                               std::string name)
    : m_stdev_range(stdev_range),
      m_stdev_v_angle(stdev_v_angle),
      m_stdev_h_angle(stdev_h_angle),
      m_stdev_intensity(stdev_intensity),
      ChFilter(name) {}

void ChFilterLidarNoiseXYZI::Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    std::shared_ptr<SensorDeviceXYZIBuffer> pXYZI = std::dynamic_pointer_cast<SensorDeviceXYZIBuffer>(bufferInOut);
    if (!pXYZI) {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    }
    m_bufferInOut = pXYZI;

    if (auto pOpx = std::dynamic_pointer_cast<ChOptixSensor>(pSensor)) {
        m_cuda_stream = pOpx->GetCudaStream();
    } else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }

    m_rng = std::shared_ptr<curandState_t>(
        cudaMallocHelper<curandState_t>(m_bufferInOut->Width * m_bufferInOut->Height), cudaFreeHelper<curandState_t>);
    init_cuda_rng((unsigned int)(std::chrono::high_resolution_clock::now().time_since_epoch().count()), m_rng.get(),
                  m_bufferInOut->Width * m_bufferInOut->Height);
}

void ChFilterLidarNoiseXYZI::Apply() {
    cuda_lidar_noise_normal((float*)m_bufferInOut->Buffer.get(), (int)m_bufferInOut->Width, (int)m_bufferInOut->Height,
                            m_stdev_range, m_stdev_v_angle, m_stdev_h_angle, m_stdev_intensity, m_rng.get(),
                            m_cuda_stream);
}

}  // namespace sensor
}  // namespace chrono
