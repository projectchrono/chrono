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
#include "chrono_sensor/ChSensor.h"
#include "chrono_sensor/cuda/grayscale.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"

namespace chrono {
namespace sensor {

ChFilterGrayscale::ChFilterGrayscale(std::string name) : ChFilter(name) {}

CH_SENSOR_API void ChFilterGrayscale::Apply(std::shared_ptr<ChSensor> pSensor,
                                            std::shared_ptr<SensorBuffer>& bufferInOut) {
    // this filter CANNOT be the first filter in a sensor's filter list, so the bufferIn CANNOT null.
    assert(bufferInOut != nullptr);
    if (!bufferInOut)
        throw std::runtime_error("The grayscale filter was not supplied an input buffer");

    void* device_ptr;

    if (auto pOpx = std::dynamic_pointer_cast<SensorOptixBuffer>(bufferInOut)) {
        // we only know how to convert RGBA8 to grayscale (not any other input format (yet))
        if (pOpx->Buffer->getFormat() != RT_FORMAT_UNSIGNED_BYTE4) {
            throw std::runtime_error("The only optix format that can be converted to grayscale is RGBA8");
        }

        // we need id of first device for this context (should only have 1 anyway)
        int device_id = pOpx->Buffer->getContext()->getEnabledDevices()[0];
        device_ptr = pOpx->Buffer->getDevicePointer(device_id);  // hard coded to grab from device 0
    } else if (auto pRGBA8 = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut)) {
        device_ptr = pRGBA8->Buffer.get();
    } else {
        throw std::runtime_error(
            "The grayscale filter requires that the incoming buffer must be an optix or device RGBA8 buffer");
    }

    // make new buffer if needed
    if (!m_buffer) {
        unsigned int sz = bufferInOut->Width * bufferInOut->Height;
        m_buffer = chrono_types::make_shared<SensorDeviceR8Buffer>();
        DeviceR8BufferPtr b(cudaMallocHelper<char>(sz), cudaFreeHelper<char>);
        m_buffer->Buffer = std::move(b);
    }

    m_buffer->Width = bufferInOut->Width;
    m_buffer->Height = bufferInOut->Height;
    m_buffer->LaunchedCount = bufferInOut->LaunchedCount;
    m_buffer->TimeStamp = bufferInOut->TimeStamp;

    // perform greyscale operation
    cuda_grayscale(device_ptr, m_buffer->Buffer.get(), (int)bufferInOut->Width, (int)bufferInOut->Height);
    bufferInOut = m_buffer;
}

}  // namespace sensor
}  // namespace chrono
