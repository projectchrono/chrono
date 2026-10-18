// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Bo-Hsun Chen
// =============================================================================
// 
// =============================================================================

#include "chrono_sensor/ChConfigSensor.h"
#if (defined(CHRONO_HAS_VULKAN_RT) || defined(CHRONO_HAS_METAL_RT)) && !defined(CHRONO_HAS_OPTIX)

    #include "chrono_sensor/ChSensorManager.h"
    #include "chrono_sensor/filters/ChFilterPhysCameraNoise.h"
    #ifdef CHRONO_HAS_METAL_RT
        #include "chrono_sensor/metal/ChMetalPhysCamOps.h"
    #endif

    #include <algorithm>
    #include <cmath>
    #include <memory>
    #include <random>

namespace chrono {
namespace sensor {
ChFilterPhysCameraNoise::ChFilterPhysCameraNoise(float expsr_time,
                                                 ChVector3f dark_current_vec,
                                                 ChVector3f noise_gain_vec,
                                                 ChVector3f STD_read_vec,
                                                 unsigned int FPN_seed,
                                                 std::string name)
    : m_expsr_time(expsr_time), m_FPN_seed(FPN_seed), ChFilter(name) {
    m_dark_currents[0] = dark_current_vec.x();
    m_dark_currents[1] = dark_current_vec.y();
    m_dark_currents[2] = dark_current_vec.z();
    m_noise_gains[0] = noise_gain_vec.x();
    m_noise_gains[1] = noise_gain_vec.y();
    m_noise_gains[2] = noise_gain_vec.z();
    m_STD_reads[0] = STD_read_vec.x();
    m_STD_reads[1] = STD_read_vec.y();
    m_STD_reads[2] = STD_read_vec.z();
}

CH_SENSOR_API void ChFilterPhysCameraNoise::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                       std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);
    m_in_out = std::dynamic_pointer_cast<SensorDeviceHalf4Buffer>(bufferInOut);
    if (!m_in_out)
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    #ifdef CHRONO_HAS_METAL_RT
    // Shot and dark-current noise take their own RNG stream, exactly as the CUDA path does, while
    // the FPN/read stream keeps the user-supplied, reproducible m_FPN_seed. Going through
    // GetDeterministicSeed is what makes ChSensorManager::SetRandomSeed reach this filter, and what
    // keeps two physical cameras in one scene from drawing the same shot-noise sequence.
    //
    // Only on the Metal path, because only the Metal kernel consumes it: the host fallback below
    // draws everything from m_FPN_seed, so deriving a stream it would ignore would add a way for
    // this filter to throw without changing a single pixel.
    m_shot_seed = static_cast<unsigned int>(ChSensorManager::GetDeterministicSeed(pSensor, RngUsage::PhysCameraShotNoise, GetRngStreamIndex()));
    #endif
}

CH_SENSOR_API void ChFilterPhysCameraNoise::Apply() {
    if (!m_in_out || !m_in_out->Buffer)
        return;
    #ifdef CHRONO_HAS_METAL_RT
    if (metal_phys_cam::Noise(m_in_out->Buffer.get(), m_in_out->Width, m_in_out->Height, m_expsr_time, m_dark_currents, m_noise_gains, m_STD_reads, m_shot_seed, m_FPN_seed,
                              m_in_out->LaunchedCount))
        return;
    #endif
    const size_t count = static_cast<size_t>(m_in_out->Width) * m_in_out->Height;
    for (size_t i = 0; i < count; ++i) {
        std::mt19937 rng(static_cast<uint32_t>(m_FPN_seed + 0x9e3779b9u * static_cast<uint32_t>(i + 1u) + 1013904223u * m_in_out->LaunchedCount));
        std::normal_distribution<float> normal(0.f, 1.f);
        float* channels[3] = {&m_in_out->Buffer[i].R, &m_in_out->Buffer[i].G, &m_in_out->Buffer[i].B};
        for (int ch = 0; ch < 3; ++ch) {
            float e_num = *channels[ch] + m_dark_currents[ch] * m_expsr_time;
            e_num += normal(rng) * m_noise_gains[ch] * std::sqrt(std::max(0.f, e_num));
            e_num += normal(rng) * m_STD_reads[ch];
            *channels[ch] = e_num;
        }
    }
}

CH_SENSOR_API void ChFilterPhysCameraNoise::SetFilterCtrlParameters(float expsr_time) { m_expsr_time = expsr_time; }
CH_SENSOR_API void ChFilterPhysCameraNoise::SetFilterModelParameters(ChVector3f dark_current_vec,
                                                                     ChVector3f noise_gain_vec,
                                                                     ChVector3f STD_read_vec) {
    m_dark_currents[0] = dark_current_vec.x();
    m_dark_currents[1] = dark_current_vec.y();
    m_dark_currents[2] = dark_current_vec.z();
    m_noise_gains[0] = noise_gain_vec.x();
    m_noise_gains[1] = noise_gain_vec.y();
    m_noise_gains[2] = noise_gain_vec.z();
    m_STD_reads[0] = STD_read_vec.x();
    m_STD_reads[1] = STD_read_vec.y();
    m_STD_reads[2] = STD_read_vec.z();
}

}  // namespace sensor
}  // namespace chrono

#else


#include "chrono_sensor/filters/ChFilterPhysCameraNoise.h"
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/cuda/phys_cam_ops.cuh"
#include "chrono_sensor/cuda/curand_utils.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"
// For ChSensorManager::GetDeterministicSeed(): derives the shot-noise buffer's own seed from the
// user-set fixed seed plus the stream's identity, falling back to the wall clock when none is set.
// The fixed-pattern-noise buffer below is NOT routed through it: m_FPN_seed is a caller-supplied
// knob and part of the existing public contract of this filter.
#include "chrono_sensor/ChSensorManager.h"
#include <chrono>

namespace chrono {
namespace sensor {
ChFilterPhysCameraNoise::ChFilterPhysCameraNoise(
    float expsr_time, ChVector3f dark_current_vec, ChVector3f noise_gain_vec, ChVector3f STD_read_vec,
    unsigned int FPN_seed, std::string name
) :
    m_expsr_time(expsr_time), m_FPN_seed(FPN_seed), ChFilter(name)
{
    m_dark_currents[0] = dark_current_vec.x();
    m_dark_currents[1] = dark_current_vec.y();
    m_dark_currents[2] = dark_current_vec.z();
    
    m_noise_gains[0] = noise_gain_vec.x();
    m_noise_gains[1] = noise_gain_vec.y();
    m_noise_gains[2] = noise_gain_vec.z();

    m_STD_reads[0] = STD_read_vec.x();
    m_STD_reads[1] = STD_read_vec.y();
    m_STD_reads[2] = STD_read_vec.z();
};

CH_SENSOR_API void ChFilterPhysCameraNoise::Initialize(
    std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut
) {
    if (!bufferInOut) {
        InvalidFilterGraphNullBuffer(pSensor);
    }

    if (auto pRGBAHalf4 = std::dynamic_pointer_cast<SensorDeviceHalf4Buffer>(bufferInOut)) {
        m_in_out = pRGBAHalf4;
    }
    else {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    }

    m_rng_shot = std::shared_ptr<curandState_t>(
        cudaMallocHelper<curandState_t>(bufferInOut->Width * bufferInOut->Height), cudaFreeHelper<curandState_t>
    );
    init_cuda_rng(
        ChSensorManager::GetDeterministicSeed(pSensor, RngUsage::PhysCameraShotNoise, GetRngStreamIndex()),
        m_rng_shot.get(), bufferInOut->Width * bufferInOut->Height
    );
    m_rng_FPN = std::shared_ptr<curandState_t>(
        cudaMallocHelper<curandState_t>(bufferInOut->Width * bufferInOut->Height), cudaFreeHelper<curandState_t>
    );
    init_cuda_rng(m_FPN_seed, m_rng_FPN.get(), bufferInOut->Width * bufferInOut->Height);


    if (auto pOpx = std::dynamic_pointer_cast<ChOptixSensor>(pSensor)) {
        m_cuda_stream = pOpx->GetCudaStream();
    }
}

CH_SENSOR_API void ChFilterPhysCameraNoise::Apply() {    
    cuda_phys_cam_noise(
		m_in_out->Buffer.get(), m_in_out->Width, m_in_out->Height, m_expsr_time, m_dark_currents, m_noise_gains,
        m_STD_reads, m_rng_shot.get(), m_rng_FPN.get(), m_cuda_stream
	);
}

CH_SENSOR_API void ChFilterPhysCameraNoise::SetFilterCtrlParameters(float expsr_time) {
    m_expsr_time = expsr_time;
}

CH_SENSOR_API void ChFilterPhysCameraNoise::SetFilterModelParameters(
    ChVector3f dark_current_vec, ChVector3f noise_gain_vec, ChVector3f STD_read_vec
) {
	m_dark_currents[0] = dark_current_vec.x();
    m_dark_currents[1] = dark_current_vec.y();
    m_dark_currents[2] = dark_current_vec.z();
    
    m_noise_gains[0] = noise_gain_vec.x();
    m_noise_gains[1] = noise_gain_vec.y();
    m_noise_gains[2] = noise_gain_vec.z();

    m_STD_reads[0] = STD_read_vec.x();
    m_STD_reads[1] = STD_read_vec.y();
    m_STD_reads[2] = STD_read_vec.z();
}


}  // namespace sensor
}  // namespace chrono


#endif
