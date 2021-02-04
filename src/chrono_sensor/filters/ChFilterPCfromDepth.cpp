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
    bool bool_dualRet;
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
        bool_dualRet = pDI->Dual_return;
    } else {
        throw std::runtime_error("The pointcloud filter cannot be run on the requested input buffer type");
    }
    if (!m_buffer) {
        if (bool_dualRet){
            m_buffer = chrono_types::make_shared<SensorDeviceXYZIBuffer>();
            DeviceXYZIBufferPtr b(cudaMallocHelper<PixelXYZI>(bufferInOut->Width * bufferInOut->Height * 2),
                                  cudaFreeHelper<PixelXYZI>);
            m_buffer->Buffer = std::move(b);
            m_buffer->Width = bufferInOut->Width;
            m_buffer->Height = bufferInOut->Height;
            m_buffer->Dual_return = true;
        } else {
            m_buffer = chrono_types::make_shared<SensorDeviceXYZIBuffer>();
            DeviceXYZIBufferPtr b(cudaMallocHelper<PixelXYZI>(bufferInOut->Width * bufferInOut->Height),
                                  cudaFreeHelper<PixelXYZI>);
            m_buffer->Buffer = std::move(b);
            m_buffer->Width = bufferInOut->Width;
            m_buffer->Height = bufferInOut->Height;
            m_buffer->Dual_return = false;
        }
    }

    // carry out the conversion from depth to point cloud
    if (bool_dualRet){
       cuda_pointcloud_from_depth_dual_return(ptr, m_buffer->Buffer.get(), (int)bufferInOut->Width, (int)bufferInOut->Height,
                                   pLidar->GetHFOV(), pLidar->GetMaxVertAngle(), pLidar->GetMinVertAngle());

    } else {
        cuda_pointcloud_from_depth(ptr, m_buffer->Buffer.get(), (int)bufferInOut->Width, (int)bufferInOut->Height,
                                   pLidar->GetHFOV(), pLidar->GetMaxVertAngle(), pLidar->GetMinVertAngle());

    }

    // counter for beam returns
    m_buffer->Beam_return_count = 0;
    // host buffer to store data
    if (bool_dualRet){
        float* buf = new float[m_buffer->Width * m_buffer->Height * 4 * 2];
        float* new_buf = new float[m_buffer->Width * m_buffer->Height * 4 * 2];
        cudaMemcpy(buf, m_buffer->Buffer.get(), m_buffer->Width * m_buffer->Height * 2 * sizeof(PixelXYZI), cudaMemcpyDeviceToHost);

        for (unsigned int i = 0; i < m_buffer->Width * m_buffer->Height; i++){
            if (buf[i * 8 + 3] > 0) {
                new_buf[m_buffer->Beam_return_count * 8] = buf[i * 8];
                new_buf[m_buffer->Beam_return_count * 8 + 1] = buf[i * 8 + 1];
                new_buf[m_buffer->Beam_return_count * 8 + 2] = buf[i * 8 + 2];
                new_buf[m_buffer->Beam_return_count * 8 + 3] = buf[i * 8 + 3];
                new_buf[m_buffer->Beam_return_count * 8 + 4] = buf[i * 8 + 4];
                new_buf[m_buffer->Beam_return_count * 8 + 5] = buf[i * 8 + 5];
                new_buf[m_buffer->Beam_return_count * 8 + 6] = buf[i * 8 + 6];
                new_buf[m_buffer->Beam_return_count * 8 + 7] = buf[i * 8 + 7];
                m_buffer->Beam_return_count++;
            }
        }
        cudaMemcpy(m_buffer->Buffer.get(), new_buf, m_buffer->Beam_return_count * 2 * sizeof(PixelXYZI), cudaMemcpyHostToDevice);
    } else {
        float* buf = new float[m_buffer->Width * m_buffer->Height * 4];
        float* new_buf = new float[m_buffer->Width * m_buffer->Height * 4];
        cudaMemcpy(buf, m_buffer->Buffer.get(), m_buffer->Width * m_buffer->Height * sizeof(PixelXYZI), cudaMemcpyDeviceToHost);

        for (unsigned int i = 0; i < m_buffer->Width * m_buffer->Height; i++){
            if (buf[i * 4 + 3] > 0) {
                new_buf[m_buffer->Beam_return_count * 4] = buf[i * 4];
                new_buf[m_buffer->Beam_return_count * 4 + 1] = buf[i * 4 + 1];
                new_buf[m_buffer->Beam_return_count * 4 + 2] = buf[i * 4 + 2];
                new_buf[m_buffer->Beam_return_count * 4 + 3] = buf[i * 4 + 3];
                m_buffer->Beam_return_count++;
            }
        }
        cudaMemcpy(m_buffer->Buffer.get(), new_buf, m_buffer->Beam_return_count * sizeof(PixelXYZI), cudaMemcpyHostToDevice);
    }
//    printf("%i %i\n", m_buffer->Dual_return, m_buffer->Width * m_buffer->Height);

    m_buffer->LaunchedCount = bufferInOut->LaunchedCount;
    m_buffer->TimeStamp = bufferInOut->TimeStamp;
    bufferInOut = m_buffer;
}
}  // namespace sensor
}  // namespace chrono
