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

#include <chrono>

#include "chrono_sensor/filters/ChFilterCameraNoise.h"

#if (defined(CHRONO_HAS_VULKAN_RT) || defined(CHRONO_HAS_METAL_RT)) && !defined(CHRONO_HAS_OPTIX)

    #include <algorithm>
    #include <cmath>
    #include <random>

namespace chrono {
namespace sensor {

namespace {
uint8_t clamp_byte(float v) {
    return static_cast<uint8_t>(std::max(0.f, std::min(255.f, v)));
}

void add_noise_rgba(std::shared_ptr<SensorDeviceRGBA8Buffer> buffer, float mean, float stdev, bool pixel_dependent) {
    if (!buffer || !buffer->Buffer)
        return;
    std::mt19937 rng(0xC001FEEDu + buffer->LaunchedCount);
    const size_t count = static_cast<size_t>(buffer->Width) * static_cast<size_t>(buffer->Height);
    for (size_t i = 0; i < count; ++i) {
        auto& p = buffer->Buffer[i];
        const float scale = pixel_dependent ? std::max(1.f, (p.R + p.G + p.B) / 3.f) / 255.f : 1.f;
        std::normal_distribution<float> dist(mean, stdev * scale);
        p.R = clamp_byte(static_cast<float>(p.R) + dist(rng));
        p.G = clamp_byte(static_cast<float>(p.G) + dist(rng));
        p.B = clamp_byte(static_cast<float>(p.B) + dist(rng));
    }
}

void add_noise_r8(std::shared_ptr<SensorDeviceR8Buffer> buffer, float mean, float stdev, bool pixel_dependent) {
    if (!buffer || !buffer->Buffer)
        return;
    std::mt19937 rng(0xBAD5EEDu + buffer->LaunchedCount);
    const size_t count = static_cast<size_t>(buffer->Width) * static_cast<size_t>(buffer->Height);
    for (size_t i = 0; i < count; ++i) {
        const unsigned char value = static_cast<unsigned char>(buffer->Buffer[i]);
        const float scale = pixel_dependent ? std::max(1.f, static_cast<float>(value)) / 255.f : 1.f;
        std::normal_distribution<float> dist(mean, stdev * scale);
        buffer->Buffer[i] = static_cast<char>(clamp_byte(static_cast<float>(value) + dist(rng)));
    }
}
}  // namespace

CH_SENSOR_API ChFilterCameraNoiseConstNormal::ChFilterCameraNoiseConstNormal(float mean, float stdev, std::string name)
    : ChFilter(name), m_mean(mean), m_stdev(stdev) {}

CH_SENSOR_API void ChFilterCameraNoiseConstNormal::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                              std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);
    m_rgba8InOut = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut);
    m_r8InOut = std::dynamic_pointer_cast<SensorDeviceR8Buffer>(bufferInOut);
    if (!m_rgba8InOut && !m_r8InOut)
        InvalidFilterGraphBufferTypeMismatch(pSensor);
}

CH_SENSOR_API void ChFilterCameraNoiseConstNormal::Apply() {
    add_noise_rgba(m_rgba8InOut, m_mean, m_stdev, false);
    add_noise_r8(m_r8InOut, m_mean, m_stdev, false);
}

CH_SENSOR_API ChFilterCameraNoisePixDep::ChFilterCameraNoisePixDep(float variance_slope,
                                                                   float variance_intercept,
                                                                   std::string name)
    : ChFilter(name), m_variance_slope(variance_slope), m_variance_intercept(variance_intercept) {}

CH_SENSOR_API void ChFilterCameraNoisePixDep::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                         std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);
    m_rgba8InOut = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut);
    m_r8InOut = std::dynamic_pointer_cast<SensorDeviceR8Buffer>(bufferInOut);
    if (!m_rgba8InOut && !m_r8InOut)
        InvalidFilterGraphBufferTypeMismatch(pSensor);
}

CH_SENSOR_API void ChFilterCameraNoisePixDep::Apply() {
    add_noise_rgba(m_rgba8InOut, m_variance_intercept, m_variance_slope, true);
    add_noise_r8(m_r8InOut, m_variance_intercept, m_variance_slope, true);
}

}  // namespace sensor
}  // namespace chrono

#else

#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/cuda/camera_noise.cuh"
#include "chrono_sensor/cuda/curand_utils.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"
// For ChSensorManager::GetDeterministicSeed(): derives this buffer's own seed from the user-set
// fixed seed plus the stream's identity, falling back to the wall clock when no seed is set.
#include "chrono_sensor/ChSensorManager.h"

namespace chrono {
namespace sensor {

ChFilterCameraNoiseConstNormal::ChFilterCameraNoiseConstNormal(float mean, float stdev, std::string name) : m_mean(mean), m_stdev(stdev), ChFilter(name) {}

void ChFilterCameraNoiseConstNormal::Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);
    std::shared_ptr<SensorDeviceRGBA8Buffer> pRGBA = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut);
    std::shared_ptr<SensorDeviceR8Buffer> pR = std::dynamic_pointer_cast<SensorDeviceR8Buffer>(bufferInOut);

    m_rng = std::shared_ptr<curandState_t>(cudaMallocHelper<curandState_t>(bufferInOut->Width * bufferInOut->Height), cudaFreeHelper<curandState_t>);
    init_cuda_rng(ChSensorManager::GetDeterministicSeed(pSensor, RngUsage::CameraNoiseConstNormal, GetRngStreamIndex()), m_rng.get(), bufferInOut->Width * bufferInOut->Height);

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

void ChFilterCameraNoiseConstNormal::Apply() {
    if (m_rgba8InOut) {
        cuda_camera_noise_const_normal((unsigned char*)m_rgba8InOut->Buffer.get(), (int)m_rgba8InOut->Width, (int)m_rgba8InOut->Height, m_mean, m_stdev, m_rng.get(),
                                       m_cuda_stream);
    } else if (m_r8InOut) {
        cuda_camera_noise_const_normal((unsigned char*)m_r8InOut->Buffer.get(), (int)m_r8InOut->Width, (int)m_r8InOut->Height, m_mean, m_stdev, m_rng.get(), m_cuda_stream);
    }
}

//// TODO: finish implementing the noise addition kernel

ChFilterCameraNoisePixDep::ChFilterCameraNoisePixDep(float variance_slope, float variance_intercept, std::string name)
    : m_variance_slope(variance_slope), m_variance_intercept(variance_intercept), ChFilter(name) {}

void ChFilterCameraNoisePixDep::Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);
    std::shared_ptr<SensorDeviceRGBA8Buffer> pRGBA = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut);
    std::shared_ptr<SensorDeviceR8Buffer> pR = std::dynamic_pointer_cast<SensorDeviceR8Buffer>(bufferInOut);

    m_rng = std::shared_ptr<curandState_t>(cudaMallocHelper<curandState_t>(bufferInOut->Width * bufferInOut->Height), cudaFreeHelper<curandState_t>);
    init_cuda_rng(ChSensorManager::GetDeterministicSeed(pSensor, RngUsage::CameraNoisePixDep, GetRngStreamIndex()), m_rng.get(), bufferInOut->Width * bufferInOut->Height);

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

void ChFilterCameraNoisePixDep::Apply() {
    if (m_rgba8InOut) {
        cuda_camera_noise_pixel_dependent((unsigned char*)m_rgba8InOut->Buffer.get(), (int)m_rgba8InOut->Width, (int)m_rgba8InOut->Height, m_variance_slope, m_variance_intercept,
                                          m_rng.get(), m_cuda_stream);
    } else if (m_r8InOut) {
        cuda_camera_noise_pixel_dependent((unsigned char*)m_r8InOut->Buffer.get(), (int)m_r8InOut->Width, (int)m_r8InOut->Height, m_variance_slope, m_variance_intercept,
                                          m_rng.get(), m_cuda_stream);
    }
}

#ifdef CHRONO_HAS_OPTIX

void InitCudaRNG(unsigned long long seed, curandState_t* rng_states, unsigned int n_generators) {
    init_cuda_rng(seed, rng_states, n_generators);
}

void CudaCameraNoiseConstNormal(unsigned char* bufPtr, int width, int height, float mean, float stdev, curandState_t* rng, CUstream& stream) {
    cuda_camera_noise_const_normal(bufPtr, width, height, mean, stdev, rng, stream);
}

#endif

}  // namespace sensor
}  // namespace chrono

#endif
