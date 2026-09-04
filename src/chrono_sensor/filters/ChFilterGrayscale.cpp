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

#include "chrono_sensor/filters/ChFilterGrayscale.h"

#if (defined(CHRONO_HAS_VULKAN_RT) || defined(CHRONO_HAS_METAL_RT)) && !defined(CHRONO_HAS_OPTIX)

    #include <algorithm>

namespace chrono {
namespace sensor {

CH_SENSOR_API ChFilterGrayscale::ChFilterGrayscale(std::string name) : ChFilter(name.length() ? name : "Grayscale") {}

CH_SENSOR_API void ChFilterGrayscale::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                 std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);
    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut);
    if (!m_buffer_in) {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
        return;
    }

    m_buffer_out = chrono_types::make_shared<SensorDeviceR8Buffer>();
    m_buffer_out->Width = m_buffer_in->Width;
    m_buffer_out->Height = m_buffer_in->Height;
    m_buffer_out->Buffer = std::shared_ptr<char[]>(new char[m_buffer_out->Width * m_buffer_out->Height]);
    bufferInOut = m_buffer_out;
}

CH_SENSOR_API void ChFilterGrayscale::Apply() {
    if (!m_buffer_in || !m_buffer_out || !m_buffer_in->Buffer)
        return;

    const size_t count = static_cast<size_t>(m_buffer_in->Width) * static_cast<size_t>(m_buffer_in->Height);
    if (!m_buffer_out->Buffer)
        m_buffer_out->Buffer = std::shared_ptr<char[]>(new char[count]);

    m_buffer_out->Width = m_buffer_in->Width;
    m_buffer_out->Height = m_buffer_in->Height;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;

    for (size_t i = 0; i < count; ++i) {
        const auto& p = m_buffer_in->Buffer[i];
        const int gray = static_cast<int>(0.299f * p.R + 0.587f * p.G + 0.114f * p.B + 0.5f);
        m_buffer_out->Buffer[i] = static_cast<char>(std::max(0, std::min(255, gray)));
    }
}

}  // namespace sensor
}  // namespace chrono

#else

#include "chrono_sensor/filters/ChFilterGrayscale.h"
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/cuda/grayscale.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"

namespace chrono {
namespace sensor {

ChFilterGrayscale::ChFilterGrayscale(std::string name) : ChFilter(name) {}
CH_SENSOR_API void ChFilterGrayscale::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                 std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);
    if (auto pRGBA8 = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut)) {
        m_buffer_in = pRGBA8;
    } else {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    }

    if (auto pOpx = std::dynamic_pointer_cast<ChOptixSensor>(pSensor)) {
        m_cuda_stream = pOpx->GetCudaStream();
    } else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }

    // make new buffer for output
    m_buffer_out = chrono_types::make_shared<SensorDeviceR8Buffer>();
    DeviceR8BufferPtr b(cudaMallocHelper<char>(bufferInOut->Width * bufferInOut->Height), cudaFreeHelper<char>);
    m_buffer_out->Buffer = std::move(b);
    m_buffer_out->Width = bufferInOut->Width;
    m_buffer_out->Height = bufferInOut->Height;
    m_buffer_out->LaunchedCount = bufferInOut->LaunchedCount;
    m_buffer_out->TimeStamp = bufferInOut->TimeStamp;
    bufferInOut = m_buffer_out;
}

CH_SENSOR_API void ChFilterGrayscale::Apply() {
    m_buffer_out->Width = m_buffer_in->Width;
    m_buffer_out->Height = m_buffer_in->Height;
    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;

    // perform grayscale operation
    cuda_grayscale(m_buffer_in->Buffer.get(), m_buffer_out->Buffer.get(), (int)m_buffer_out->Width,
                   (int)m_buffer_out->Height, m_cuda_stream);
}

}  // namespace sensor
}  // namespace chrono

#endif
