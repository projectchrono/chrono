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
// Filter to convert exposure to digital values with considering ISO, through
// camera response function (CRF) (gamma_correct curve or sigmoid function),
//
// 			sigmoid: 		I = sigmoid(a * lg(E) + b),
//			gamma_correct: 	I = a * (lg(E))^\gamma + b,
//
// where I is image intensity (0 to 65535), E is exposure, a is expsr2dv_gain,
// b is expsr2dv_bias and \gamma is expsr2dv_gain
// 
// =============================================================================

#include "chrono_sensor/ChConfigSensor.h"
#if defined(CHRONO_HAS_VULKAN_RT) && !defined(CHRONO_HAS_OPTIX)

#include "chrono_sensor/filters/ChFilterPhysCameraExpsrToDV.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace chrono {
namespace sensor {
ChFilterPhysCameraExpsrToDV::ChFilterPhysCameraExpsrToDV(float ISO,
                                                         ChVector3f expsr2dv_gain_vec,
                                                         ChVector3f expsr2dv_bias_vec,
                                                         float expsr2dv_gamma,
                                                         int crf_type,
                                                         std::string name)
    : m_ISO(ISO), m_expsr2dv_gamma(expsr2dv_gamma), m_crf_type(crf_type), ChFilter(name) {
    m_expsr2dv_gains[0] = expsr2dv_gain_vec.x();
    m_expsr2dv_gains[1] = expsr2dv_gain_vec.y();
    m_expsr2dv_gains[2] = expsr2dv_gain_vec.z();
    m_expsr2dv_biases[0] = expsr2dv_bias_vec.x();
    m_expsr2dv_biases[1] = expsr2dv_bias_vec.y();
    m_expsr2dv_biases[2] = expsr2dv_bias_vec.z();
}

CH_SENSOR_API void ChFilterPhysCameraExpsrToDV::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                           std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);
    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceHalf4Buffer>(bufferInOut);
    if (!m_buffer_in) {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
        return;
    }
    m_buffer_out = chrono_types::make_shared<SensorDeviceHalf4Buffer>();
    m_buffer_out->Width = m_buffer_in->Width;
    m_buffer_out->Height = m_buffer_in->Height;
    m_buffer_out->Buffer = std::shared_ptr<PixelHalf4[]>(new PixelHalf4[static_cast<size_t>(m_buffer_out->Width) * m_buffer_out->Height]);
    bufferInOut = m_buffer_out;
}

CH_SENSOR_API void ChFilterPhysCameraExpsrToDV::Apply() {
    if (!m_buffer_in || !m_buffer_out || !m_buffer_in->Buffer || !m_buffer_out->Buffer)
        return;
    m_buffer_out->Width = m_buffer_in->Width;
    m_buffer_out->Height = m_buffer_in->Height;
    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
    const size_t count = static_cast<size_t>(m_buffer_in->Width) * m_buffer_in->Height;
    for (size_t i = 0; i < count; ++i) {
        const auto& in = m_buffer_in->Buffer[i];
        PixelHalf4 out{};
        const float vals[3] = {in.R, in.G, in.B};
        float* outs[3] = {&out.R, &out.G, &out.B};
        for (int ch = 0; ch < 3; ++ch) {
            float px_I = 0.f;
            if (m_crf_type == 0) {
                const float e = std::max(1e-12f, m_ISO * vals[ch]);
                px_I = m_expsr2dv_gains[ch] * std::pow(std::log2(e), m_expsr2dv_gamma) + m_expsr2dv_biases[ch];
            } else if (m_crf_type == 1) {
                px_I = m_expsr2dv_gains[ch] * m_ISO * vals[ch] + m_expsr2dv_biases[ch];
                px_I = 1.f / (1.f + std::exp(-px_I));
            } else if (m_crf_type == 2) {
                px_I = m_expsr2dv_gains[ch] * m_ISO * vals[ch] + m_expsr2dv_biases[ch];
            } else {
                throw std::runtime_error("Invalid camera response function type");
            }
            *outs[ch] = px_I;
        }
        out.A = (m_crf_type == 1) ? 1.f : in.A;
        m_buffer_out->Buffer[i] = out;
    }
}

CH_SENSOR_API void ChFilterPhysCameraExpsrToDV::SetFilterCtrlParameters(float ISO) { m_ISO = ISO; }
CH_SENSOR_API void ChFilterPhysCameraExpsrToDV::SetFilterModelParameters(ChVector3f expsr2dv_gain_vec,
                                                                         ChVector3f expsr2dv_bias_vec,
                                                                         float expsr2dv_gamma,
                                                                         int crf_type) {
    m_expsr2dv_gains[0] = expsr2dv_gain_vec.x();
    m_expsr2dv_gains[1] = expsr2dv_gain_vec.y();
    m_expsr2dv_gains[2] = expsr2dv_gain_vec.z();
    m_expsr2dv_biases[0] = expsr2dv_bias_vec.x();
    m_expsr2dv_biases[1] = expsr2dv_bias_vec.y();
    m_expsr2dv_biases[2] = expsr2dv_bias_vec.z();
    m_expsr2dv_gamma = expsr2dv_gamma;
    m_crf_type = crf_type;
}

}  // namespace sensor
}  // namespace chrono

#else


#include "chrono_sensor/filters/ChFilterPhysCameraExpsrToDV.h"
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/cuda/phys_cam_ops.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#include <chrono>

namespace chrono {
namespace sensor {
ChFilterPhysCameraExpsrToDV::ChFilterPhysCameraExpsrToDV(
    float ISO, ChVector3f expsr2dv_gain_vec, ChVector3f expsr2dv_bias_vec, float expsr2dv_gamma, int crf_type,
    std::string name
)
: m_ISO(ISO), m_expsr2dv_gamma(expsr2dv_gamma), m_crf_type(crf_type),ChFilter(name)
{
    m_expsr2dv_gains[0] = expsr2dv_gain_vec.x();
    m_expsr2dv_gains[1] = expsr2dv_gain_vec.y();
    m_expsr2dv_gains[2] = expsr2dv_gain_vec.z();
    
    m_expsr2dv_biases[0] = expsr2dv_bias_vec.x();
    m_expsr2dv_biases[1] = expsr2dv_bias_vec.y();
    m_expsr2dv_biases[2] = expsr2dv_bias_vec.z();
};

CH_SENSOR_API void ChFilterPhysCameraExpsrToDV::Initialize(
    std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut
) {
    if (!bufferInOut) {
        InvalidFilterGraphNullBuffer(pSensor);
    }
    
    if (auto pHalf4 = std::dynamic_pointer_cast<SensorDeviceHalf4Buffer>(bufferInOut)) {
        m_buffer_in = pHalf4;
    }
    else {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    }

    if (auto pOpx = std::dynamic_pointer_cast<ChOptixSensor>(pSensor)) {
        m_cuda_stream = pOpx->GetCudaStream();
    }

    // create new buffer as output
    m_buffer_out = chrono_types::make_shared<SensorDeviceHalf4Buffer>();
    DeviceHalf4BufferPtr b(
        cudaMallocHelper<PixelHalf4>(bufferInOut->Width * bufferInOut->Height), cudaFreeHelper<PixelHalf4>
    );
    m_buffer_out->Buffer = std::move(b);
    m_buffer_out->Width = bufferInOut->Width;
    m_buffer_out->Height = bufferInOut->Height;
    m_buffer_out->LaunchedCount = bufferInOut->LaunchedCount;
    m_buffer_out->TimeStamp = bufferInOut->TimeStamp;
    bufferInOut = m_buffer_out;
}

CH_SENSOR_API void ChFilterPhysCameraExpsrToDV::Apply() {
    // set output buffer configuration
    m_buffer_out->Width = m_buffer_in->Width;
    m_buffer_out->Height = m_buffer_in->Height;
    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
    
    cuda_phys_cam_expsr2dv(
        m_buffer_in->Buffer.get(), m_buffer_out->Buffer.get(), m_buffer_out->Width, m_buffer_out->Height,
        m_ISO, m_expsr2dv_gains, m_expsr2dv_biases, m_expsr2dv_gamma, m_crf_type, m_cuda_stream
    );
}

CH_SENSOR_API void ChFilterPhysCameraExpsrToDV::SetFilterCtrlParameters(float ISO) {
    m_ISO = ISO;
}

CH_SENSOR_API void ChFilterPhysCameraExpsrToDV::SetFilterModelParameters(
    ChVector3f expsr2dv_gain_vec, ChVector3f expsr2dv_bias_vec, float expsr2dv_gamma, int crf_type
) {
	m_expsr2dv_gains[0] = expsr2dv_gain_vec.x();
    m_expsr2dv_gains[1] = expsr2dv_gain_vec.y();
    m_expsr2dv_gains[2] = expsr2dv_gain_vec.z();
    m_expsr2dv_biases[0] = expsr2dv_bias_vec.x();
    m_expsr2dv_biases[1] = expsr2dv_bias_vec.y();
    m_expsr2dv_biases[2] = expsr2dv_bias_vec.z();
    m_expsr2dv_gamma = expsr2dv_gamma;
    m_crf_type = crf_type;
}


}  // namespace sensor
}  // namespace chrono


#endif
