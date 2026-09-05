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
// Filter to do vignetting based on camera control parameters
// 
// =============================================================================

#include "chrono_sensor/ChConfigSensor.h"
#if (defined(CHRONO_HAS_VULKAN_RT) || defined(CHRONO_HAS_METAL_RT)) && !defined(CHRONO_HAS_OPTIX)

    #include "chrono_sensor/filters/ChFilterPhysCameraVignetting.h"
    #ifdef CHRONO_HAS_METAL_RT
        #include "chrono_sensor/metal/ChMetalPhysCamOps.h"
    #endif

    #include <algorithm>
    #include <cmath>
    #include <memory>

namespace chrono {
namespace sensor {
ChFilterPhysCameraVignetting::ChFilterPhysCameraVignetting(float focal_length, float sensor_width, float vignetting_gain, std::string name)
    : m_focal_length(focal_length), m_sensor_width(sensor_width), m_vignetting_gain(vignetting_gain), ChFilter(name) {}

CH_SENSOR_API void ChFilterPhysCameraVignetting::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                            std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);
    m_in_out = std::dynamic_pointer_cast<SensorDeviceHalf4Buffer>(bufferInOut);
    if (!m_in_out)
        InvalidFilterGraphBufferTypeMismatch(pSensor);
}

CH_SENSOR_API void ChFilterPhysCameraVignetting::Apply() {
    if (!m_in_out || !m_in_out->Buffer)
        return;
    const unsigned int img_w = m_in_out->Width;
    const unsigned int img_h = m_in_out->Height;
    #ifdef CHRONO_HAS_METAL_RT
    if (metal_phys_cam::Vignetting(m_in_out->Buffer.get(), img_w, img_h, m_focal_length, m_sensor_width, m_vignetting_gain))
        return;
    #endif
    const size_t count = static_cast<size_t>(img_w) * img_h;
    for (size_t i = 0; i < count; ++i) {
        const unsigned int col = static_cast<unsigned int>(i % img_w);
        const unsigned int row = static_cast<unsigned int>(i / img_w);
        const float x = (static_cast<float>(col) - static_cast<float>(img_w - 1) / 2.f) / static_cast<float>(img_w) * m_sensor_width;
        const float y = (static_cast<float>(row) - static_cast<float>(img_h - 1) / 2.f) / static_cast<float>(img_w) * m_sensor_width;
        const float f = std::max(std::abs(m_focal_length), 1e-12f);
        const float c = std::cos(std::atan(std::sqrt(x * x + y * y) / f));
        const float gain = 1.f - m_vignetting_gain + m_vignetting_gain * c * c * c * c;
        m_in_out->Buffer[i].R *= gain;
        m_in_out->Buffer[i].G *= gain;
        m_in_out->Buffer[i].B *= gain;
    }
}

CH_SENSOR_API void ChFilterPhysCameraVignetting::SetFilterCtrlParameters(float focal_length) { m_focal_length = focal_length; }
CH_SENSOR_API void ChFilterPhysCameraVignetting::SetFilterModelParameters(float sensor_width, float vignetting_gain) {
    m_sensor_width = sensor_width;
    m_vignetting_gain = vignetting_gain;
}

}  // namespace sensor
}  // namespace chrono

#else


#include "chrono_sensor/filters/ChFilterPhysCameraVignetting.h"
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/cuda/phys_cam_ops.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#include <chrono>

namespace chrono {
namespace sensor {
ChFilterPhysCameraVignetting::ChFilterPhysCameraVignetting(
    float focal_length, float sensor_width, float vignetting_gain, std::string name
) : m_focal_length(focal_length), m_sensor_width(sensor_width), m_vignetting_gain(vignetting_gain), ChFilter(name) {};

CH_SENSOR_API void ChFilterPhysCameraVignetting::Initialize(
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

    if (auto pOpx = std::dynamic_pointer_cast<ChOptixSensor>(pSensor)) {
        m_cuda_stream = pOpx->GetCudaStream();
    }
}

CH_SENSOR_API void ChFilterPhysCameraVignetting::Apply() {    
    cuda_phys_cam_vignetting(
		m_in_out->Buffer.get(), m_in_out->Width, m_in_out->Height, m_focal_length, m_sensor_width, m_vignetting_gain,
		m_cuda_stream
	);
}

CH_SENSOR_API void ChFilterPhysCameraVignetting::SetFilterCtrlParameters(float focal_length) {
    m_focal_length = focal_length;
}

CH_SENSOR_API void ChFilterPhysCameraVignetting::SetFilterModelParameters(float sensor_width, float vignetting_gain) {
	m_sensor_width = sensor_width;
	m_vignetting_gain = vignetting_gain;
}


}  // namespace sensor
}  // namespace chrono


#endif
