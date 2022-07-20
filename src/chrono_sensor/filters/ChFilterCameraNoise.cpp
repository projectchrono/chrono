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

#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/cuda/camera_noise.cuh"
#include "chrono_sensor/cuda/curand_utils.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#include <chrono>

namespace chrono {
namespace sensor {

ChFilterCameraNoiseConstNormal::ChFilterCameraNoiseConstNormal(float mean, float stdev, std::string name)
    : m_mean(mean), m_stdev(stdev), ChFilter(name) {}

CH_SENSOR_API void ChFilterCameraNoiseConstNormal::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                              std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);
    std::shared_ptr<SensorDeviceRGBA8Buffer> pRGBA = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut);
    std::shared_ptr<SensorDeviceR8Buffer> pR = std::dynamic_pointer_cast<SensorDeviceR8Buffer>(bufferInOut);

    m_rng = std::shared_ptr<curandState_t>(cudaMallocHelper<curandState_t>(bufferInOut->Width * bufferInOut->Height),
                                           cudaFreeHelper<curandState_t>);
    init_cuda_rng((unsigned int)(std::chrono::high_resolution_clock::now().time_since_epoch().count()), m_rng.get(),
                  bufferInOut->Width * bufferInOut->Height);

    if (pRGBA) {
        m_rgba8InOut = pRGBA;
    } else if (pR) {
        m_r8InOut = pR;
    } else {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    }

    if (auto pOpx = std::dynamic_pointer_cast<ChOptixSensor>(pSensor)) {
        m_cuda_stream = pOpx->GetCudaStream();
    }
}

CH_SENSOR_API void ChFilterCameraNoiseConstNormal::Apply() {
    if (m_rgba8InOut) {
        cuda_camera_noise_const_normal((unsigned char*)m_rgba8InOut->Buffer.get(), (int)m_rgba8InOut->Width,
                                       (int)m_rgba8InOut->Height, m_mean, m_stdev, m_rng.get(), m_cuda_stream);
    } else if (m_r8InOut) {
        cuda_camera_noise_const_normal((unsigned char*)m_r8InOut->Buffer.get(), (int)m_r8InOut->Width,
                                       (int)m_r8InOut->Height, m_mean, m_stdev, m_rng.get(), m_cuda_stream);
    }
}

//============ TODO: finish implementing the noise addition kernel
ChFilterCameraNoisePixDep::ChFilterCameraNoisePixDep(float variance_slope, float variance_intercept, std::string name)
    : m_variance_slope(variance_slope), m_variance_intercept(variance_intercept), ChFilter(name) {}

CH_SENSOR_API void ChFilterCameraNoisePixDep::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                         std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);
    std::shared_ptr<SensorDeviceRGBA8Buffer> pRGBA = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut);
    std::shared_ptr<SensorDeviceR8Buffer> pR = std::dynamic_pointer_cast<SensorDeviceR8Buffer>(bufferInOut);

    m_rng = std::shared_ptr<curandState_t>(cudaMallocHelper<curandState_t>(bufferInOut->Width * bufferInOut->Height),
                                           cudaFreeHelper<curandState_t>);
    init_cuda_rng((unsigned int)(std::chrono::high_resolution_clock::now().time_since_epoch().count()), m_rng.get(),
                  bufferInOut->Width * bufferInOut->Height);

    if (pRGBA) {
        m_rgba8InOut = pRGBA;
    } else if (pR) {
        m_r8InOut = pR;
    } else {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    }

    if (auto pOpx = std::dynamic_pointer_cast<ChOptixSensor>(pSensor)) {
        m_cuda_stream = pOpx->GetCudaStream();
    }
}
CH_SENSOR_API void ChFilterCameraNoisePixDep::Apply() {
    if (m_rgba8InOut) {
        cuda_camera_noise_pixel_dependent((unsigned char*)m_rgba8InOut->Buffer.get(), (int)m_rgba8InOut->Width,
                                          (int)m_rgba8InOut->Height, m_variance_slope, m_variance_intercept, m_rng.get(),
                                          m_cuda_stream);
    } else if (m_r8InOut) {
        cuda_camera_noise_pixel_dependent((unsigned char*)m_r8InOut->Buffer.get(), (int)m_r8InOut->Width,
                                          (int)m_r8InOut->Height, m_variance_slope, m_variance_intercept, m_rng.get(),
                                          m_cuda_stream);
    }
}

}  // namespace sensor
}  // namespace chrono
