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

#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/ChLidarSensor.h"
#include "chrono_sensor/cuda/pointcloud.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"

namespace chrono {
namespace sensor {

ChFilterPCfromDepth::ChFilterPCfromDepth(std::string name) : ChFilter(name) {}

CH_SENSOR_API void ChFilterPCfromDepth::Apply(std::shared_ptr<ChSensor> pSensor,
                                              std::shared_ptr<SensorBuffer>& bufferInOut) {
    // this filter CANNOT be the first filter in a sensor's filter list, so the bufferIn CANNOT null.
    assert(bufferInOut != nullptr);
    if (!bufferInOut)
        throw std::runtime_error("The Pointcloud filter was not supplied an input buffer");

    // The sensor must be a lidar in order to have correct HFOV and VFOV
    std::shared_ptr<ChLidarSensor> pLidar = std::dynamic_pointer_cast<ChLidarSensor>(pSensor);
    if (!pLidar) {
        throw std::runtime_error("This sensor must be a lidar.");
    }

    // get the pointer to the memory either from optix or from our device buffer
    void* ptr;
    if (auto pOpx = std::dynamic_pointer_cast<SensorOptixBuffer>(bufferInOut)) {
        if (pOpx->Buffer->getFormat() != RT_FORMAT_FLOAT2) {
            throw std::runtime_error(
                "The only Optix format that can be converted to pointcloud is FLOAT2 (Depth, Intensity)");
        }
        // we need id of first device for this context (should only have 1 anyway)
        int device_id = pOpx->Buffer->getContext()->getEnabledDevices()[0];
        ptr = pOpx->Buffer->getDevicePointer(device_id);  // hard coded to grab from device 0
    } else if (auto pDI = std::dynamic_pointer_cast<SensorDeviceDIBuffer>(bufferInOut)) {
        ptr = (void*)pDI->Buffer.get();
    } else {
        throw std::runtime_error("The pointcloud filter cannot be run on the requested input buffer type");
    }
    if (!m_buffer) {
        m_buffer = chrono_types::make_shared<SensorDeviceXYZIBuffer>();
        DeviceXYZIBufferPtr b(cudaMallocHelper<PixelXYZI>(bufferInOut->Width * bufferInOut->Height),
                              cudaFreeHelper<PixelXYZI>);
        m_buffer->Buffer = std::move(b);
        m_buffer->Width = bufferInOut->Width;
        m_buffer->Height = bufferInOut->Height;
    }

    // carry out the conversion from depth to point cloud
    cuda_pointcloud_from_depth(ptr, m_buffer->Buffer.get(), (int)bufferInOut->Width, (int)bufferInOut->Height,
                               pLidar->GetHFOV(), pLidar->GetMaxVertAngle(), pLidar->GetMinVertAngle());

    m_buffer->LaunchedCount = bufferInOut->LaunchedCount;
    m_buffer->TimeStamp = bufferInOut->TimeStamp;
    bufferInOut = m_buffer;
}

}  // namespace sensor
}  // namespace chrono
