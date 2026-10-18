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
#include "chrono_sensor/sensors/ChLidarSensor.h"

#ifdef CHRONO_HAS_OPTIX
#include "chrono_sensor/cuda/curand_utils.cuh"
#include "chrono_sensor/cuda/lidar_noise.cuh"
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#endif
// For ChSensorManager::GetDeterministicSeed(): derives this buffer's own seed from the user-set
// fixed seed plus the stream's identity, falling back to the wall clock when no seed is set.
#include "chrono_sensor/ChSensorManager.h"

#include <chrono>
#include <cmath>

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

    if (!(m_bufferInOut = std::dynamic_pointer_cast<SensorDeviceXYZIBuffer>(bufferInOut)))
        InvalidFilterGraphBufferTypeMismatch(pSensor);

    if (auto pLidar = std::dynamic_pointer_cast<ChLidarSensor>(pSensor)) {
#ifdef CHRONO_HAS_OPTIX
        m_cuda_stream = pLidar->GetCudaStream();
#endif
    } else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }

#ifdef CHRONO_HAS_OPTIX
    m_rng = std::shared_ptr<curandState_t>(
        cudaMallocHelper<curandState_t>(m_bufferInOut->Width * m_bufferInOut->Height), cudaFreeHelper<curandState_t>);
    init_cuda_rng(ChSensorManager::GetDeterministicSeed(pSensor, RngUsage::LidarNoiseXYZI, GetRngStreamIndex()),
                  m_rng.get(), m_bufferInOut->Width * m_bufferInOut->Height);
#else
    m_rng.seed(static_cast<std::mt19937::result_type>(
        ChSensorManager::GetDeterministicSeed(pSensor, RngUsage::LidarNoiseXYZI, GetRngStreamIndex())));
#endif
}

void ChFilterLidarNoiseXYZI::Apply() {
#ifdef CHRONO_HAS_OPTIX
    cuda_lidar_noise_normal((float*)m_bufferInOut->Buffer.get(), (int)m_bufferInOut->Width, (int)m_bufferInOut->Height,
                            m_stdev_range, m_stdev_v_angle, m_stdev_h_angle, m_stdev_intensity, m_rng.get(),
                            m_cuda_stream);
#else
    std::normal_distribution<float> n_range(0.f, m_stdev_range);
    std::normal_distribution<float> n_v(0.f, m_stdev_v_angle);
    std::normal_distribution<float> n_h(0.f, m_stdev_h_angle);
    std::normal_distribution<float> n_i(0.f, m_stdev_intensity);

    const size_t count = static_cast<size_t>(m_bufferInOut->Beam_return_count);
    for (size_t i = 0; i < count; ++i) {
        PixelXYZI& p = m_bufferInOut->Buffer[i];
        if (p.intensity <= 0.f)
            continue;

        const float old_range = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        if (old_range <= 1e-7f)
            continue;

        const float new_range = std::max(0.f, old_range + n_range(m_rng));
        float h_angle = std::atan2(p.y, p.x) + n_h(m_rng);
        float v_angle = std::atan2(p.z, std::sqrt(p.x * p.x + p.y * p.y)) + n_v(m_rng);
        const float proj_xy = new_range * std::cos(v_angle);
        p.x = proj_xy * std::cos(h_angle);
        p.y = proj_xy * std::sin(h_angle);
        p.z = new_range * std::sin(v_angle);
        p.intensity = std::max(0.f, p.intensity + n_i(m_rng));
    }
#endif
}

}  // namespace sensor
}  // namespace chrono
