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
