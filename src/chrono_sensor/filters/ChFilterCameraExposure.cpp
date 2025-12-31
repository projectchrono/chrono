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
// Authors: Bo-Hsun Chen
// =============================================================================
// 
// Filter to correct brightness of the image according to the exposure time
// 
// =============================================================================

//============ TODO: refer ChFilterGrayscale.cpp, ChFilterCameraNoise.cpp, ChFilterImageResize
#include "chrono_sensor/filters/ChFilterCameraExposure.h"
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/cuda/image_ops.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#include <chrono>

namespace chrono {
namespace sensor {
ChFilterCameraExposureCorrect::ChFilterCameraExposureCorrect(float a0, float a1, float a2, float b0, float b1, float b2, float expsr_time,
                                                             std::string name)
    : m_a0(a0), m_a1(a1), m_a2(a2), m_b0(b0), m_b1(b1), m_b2(b2), m_expsr_time(expsr_time), ChFilter(name) {}

CH_SENSOR_API void ChFilterCameraExposureCorrect::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                             std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut) {
        InvalidFilterGraphNullBuffer(pSensor);
    }

    std::shared_ptr<SensorDeviceRGBA8Buffer> pRGBA = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut);
    std::shared_ptr<SensorDeviceR8Buffer> pR = std::dynamic_pointer_cast<SensorDeviceR8Buffer>(bufferInOut);

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

    m_a = m_a0 * log2f(m_expsr_time * 1000.0f) + m_a2;
    m_b = m_b0 * log2f(m_expsr_time * 1000.0f) + m_b2;
}

CH_SENSOR_API void ChFilterCameraExposureCorrect::Apply() {    
    if (m_rgba8InOut) {
        cuda_camera_exposure_correct((unsigned char*)m_rgba8InOut->Buffer.get(), (size_t)m_rgba8InOut->Width,
                                     (size_t)m_rgba8InOut->Height, m_a, m_a1, m_b, m_b1, m_cuda_stream);
    }
    else if (m_r8InOut) {
        cuda_camera_exposure_correct((unsigned char*)m_r8InOut->Buffer.get(), (size_t)m_r8InOut->Width,
                                     (size_t)m_r8InOut->Height, m_a, m_a1, m_b, m_b1, m_cuda_stream);
    }
}

}  // namespace sensor
}  // namespace chrono
