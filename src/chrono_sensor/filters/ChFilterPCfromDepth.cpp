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
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/cuda/pointcloud.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"

// #include <cuda_runtime_api.h>

namespace chrono {
namespace sensor {

ChFilterPCfromDepth::ChFilterPCfromDepth(std::string name) : ChFilter(name) {}

CH_SENSOR_API void ChFilterPCfromDepth::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                   std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);
    if (!(m_buffer_in = std::dynamic_pointer_cast<SensorDeviceDIBuffer>(bufferInOut)))
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    if (auto pLidar = std::dynamic_pointer_cast<ChLidarSensor>(pSensor)) {
        m_hFOV = pLidar->GetHFOV();
        m_min_vert_angle = pLidar->GetMinVertAngle();
        m_max_vert_angle = pLidar->GetMaxVertAngle();
        m_cuda_stream = pLidar->GetCudaStream();
    } else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }

    // allocate output buffer
    m_buffer_out = chrono_types::make_shared<SensorDeviceXYZIBuffer>();
    DeviceXYZIBufferPtr b(
        cudaMallocHelper<PixelXYZI>(m_buffer_in->Width * m_buffer_in->Height * (m_buffer_in->Dual_return + 1)),
        cudaFreeHelper<PixelXYZI>);
    m_buffer_out->Buffer = std::move(b);
    m_buffer_out->Width = m_buffer_in->Width;
    m_buffer_out->Height = m_buffer_in->Height;
    m_buffer_out->Dual_return = m_buffer_in->Dual_return;
    bufferInOut = m_buffer_out;
}

CH_SENSOR_API void ChFilterPCfromDepth::Apply() {
    // carry out the conversion from depth to point cloud
    if (m_buffer_in->Dual_return) {
        cuda_pointcloud_from_depth_dual_return(m_buffer_in->Buffer.get(), m_buffer_out->Buffer.get(),
                                               (int)m_buffer_in->Width, (int)m_buffer_in->Height, m_hFOV,
                                               m_max_vert_angle, m_min_vert_angle, m_cuda_stream);

    } else {
        cuda_pointcloud_from_depth(m_buffer_in->Buffer.get(), m_buffer_out->Buffer.get(), (int)m_buffer_in->Width,
                                   (int)m_buffer_in->Height, m_hFOV, m_max_vert_angle, m_min_vert_angle, m_cuda_stream);
    }

    // counter for beam returns
    m_buffer_out->Beam_return_count = 0;
    auto buf = std::vector<PixelXYZI>(m_buffer_out->Width * m_buffer_out->Height * (m_buffer_out->Dual_return + 1));
    auto processed_buffer = std::vector<PixelXYZI>(m_buffer_out->Width * m_buffer_out->Height * (m_buffer_out->Dual_return + 1));
    cudaMemcpyAsync(buf.data(), m_buffer_out->Buffer.get(), buf.size() * sizeof(PixelXYZI), cudaMemcpyDeviceToHost,
                    m_cuda_stream);
    cudaStreamSynchronize(m_cuda_stream);
    for (unsigned int i = 0; i < buf.size(); i++) {
        if (buf[i].intensity > 0) {
            processed_buffer[m_buffer_out->Beam_return_count] = buf[i];
            m_buffer_out->Beam_return_count++;
        }
    }
    cudaMemcpyAsync(m_buffer_out->Buffer.get(), processed_buffer.data(), m_buffer_out->Beam_return_count * sizeof(PixelXYZI),
                    cudaMemcpyHostToDevice, m_cuda_stream);

    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
}
}  // namespace sensor
}  // namespace chrono
