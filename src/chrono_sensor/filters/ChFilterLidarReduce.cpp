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

#include "chrono_sensor/filters/ChFilterLidarReduce.h"
#include "chrono_sensor/ChSensor.h"
#include "chrono_sensor/cuda/lidar_reduce.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"

namespace chrono {
namespace sensor {

ChFilterLidarReduce::ChFilterLidarReduce(LidarReturnMode ret, int reduce_radius, std::string name)
    : m_ret(ret), m_reduce_radius(reduce_radius), ChFilter(name) {}

CH_SENSOR_API void ChFilterLidarReduce::Apply(std::shared_ptr<ChSensor> pSensor,
                                              std::shared_ptr<SensorBuffer>& bufferInOut) {
    // this filter CANNOT be the first filter in a sensor's filter list, so the bufferIn CANNOT null.
    assert(bufferInOut != nullptr);
    if (!bufferInOut)
        throw std::runtime_error("The lidar reduce filter was not supplied an input buffer");

    // to grayscale (for now), the incoming buffer must be an optix buffer
    std::shared_ptr<SensorOptixBuffer> pSen = std::dynamic_pointer_cast<SensorOptixBuffer>(bufferInOut);
    if (!pSen) {
        throw std::runtime_error("The lidar reduce filter requires that the incoming buffer must be an optix buffer");
    }

    RTsize rwidth;
    RTsize rheight;
    pSen->Buffer->getSize(rwidth, rheight);
    unsigned int width = (unsigned int)rwidth;
    unsigned int height = (unsigned int)rheight;

    // we only know how to convert RGBA8 to grayscale (not any other input format (yet))
    if (pSen->Buffer->getFormat() != RT_FORMAT_FLOAT2) {
        throw std::runtime_error("The only format that can be reduced by lidar is FLOAT2/DI (depth,intensity)");
    }

    // std::unique_ptr<int> p1 = std::make_unique<int>(4);
    // std::unique_ptr<int> p2(std::move(p1));

    if (!m_buffer) {
        m_buffer = chrono_types::make_shared<SensorDeviceDIBuffer>();
        DeviceDIBufferPtr b(
            cudaMallocHelper<PixelDI>(width * height / ((m_reduce_radius * 2 - 1) * (m_reduce_radius * 2 - 1))),
            cudaFreeHelper<PixelDI>);
        m_buffer->Buffer = std::move(b);
        m_buffer->Width = width / (m_reduce_radius * 2 - 1);
        m_buffer->Height = height / (m_reduce_radius * 2 - 1);
    }

    // we need id of first device for this context (should only have 1 anyway)
    int device_id = pSen->Buffer->getContext()->getEnabledDevices()[0];
    void* ptr = pSen->Buffer->getDevicePointer(device_id);  // hard coded to grab from device 0

    switch (m_ret) {
        case MEAN_RETURN:
            cuda_lidar_mean_reduce(ptr, m_buffer->Buffer.get(), (int)width, (int)height, m_reduce_radius);
            break;
        case STRONGEST_RETURN:
            cuda_lidar_strong_reduce(ptr, m_buffer->Buffer.get(), (int)width, (int)height, m_reduce_radius);
            break;
        default:
            throw std::runtime_error("Lidar reduce mode not yet supported");
            break;
    }

    m_buffer->LaunchedCount = bufferInOut->LaunchedCount;
    m_buffer->TimeStamp = bufferInOut->TimeStamp;
    bufferInOut = m_buffer;
}

}  // namespace sensor
}  // namespace chrono
