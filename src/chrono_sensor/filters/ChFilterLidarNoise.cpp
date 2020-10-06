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
#include "chrono_sensor/ChSensor.h"
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

void ChFilterLidarNoiseXYZI::Initialize(std::shared_ptr<ChSensor> pSensor) {}

void ChFilterLidarNoiseXYZI::Apply(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) {
    // this filter CANNOT be the first filter in a sensor's filter list, so the bufferIn CANNOT null.
    assert(bufferInOut != nullptr);
    if (!bufferInOut)
        throw std::runtime_error("The filter was not supplied an input buffer");

    // to grayscale (for now), the incoming buffer must be an optix buffer
    std::shared_ptr<SensorDeviceXYZIBuffer> pXYZI = std::dynamic_pointer_cast<SensorDeviceXYZIBuffer>(bufferInOut);
    if (!pXYZI) {
        throw std::runtime_error("The XYZI noise filter can only be run on a SensorDeviceXYZIBuffer");
    }

    unsigned int width = pXYZI->Width;
    unsigned int height = pXYZI->Height;

    // must initialize noise during first run since we don't know the dimensions in the initialize function
    if (m_noise_init) {
        m_rng = std::shared_ptr<curandState_t>(cudaMallocHelper<curandState_t>(width * height),
                                               cudaFreeHelper<curandState_t>);

        init_cuda_rng(std::chrono::high_resolution_clock::now().time_since_epoch().count(), m_rng.get(),
                      width * height);

        m_noise_init = false;
    }

    cuda_lidar_noise_normal((float*)pXYZI->Buffer.get(), (int)width, (int)height, m_stdev_range, m_stdev_v_angle,
                            m_stdev_h_angle, m_stdev_intensity, m_rng.get());
}

}  // namespace sensor
}  // namespace chrono
