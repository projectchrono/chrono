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

#include "chrono_sensor/filters/ChFilterRadarPCfromRange.h"
#include "chrono_sensor/ChRadarSensor.h"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#include "chrono_sensor/cuda/pointcloud.cuh"

namespace chrono {
namespace sensor {

ChFilterRadarPCfromRange::ChFilterRadarPCfromRange(std::string name) : ChFilter(name) {}

CH_SENSOR_API void ChFilterRadarPCfromRange::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                        std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceRangeRcsBuffer>(bufferInOut);
    if (!m_buffer_in)
        InvalidFilterGraphBufferTypeMismatch(pSensor);

    // The sensor must be a radar
    if (auto pRadar = std::dynamic_pointer_cast<ChRadarSensor>(pSensor)) {
        m_cuda_stream = pRadar->GetCudaStream();
        m_hFOV = pRadar->GetHFOV();
        m_max_vert_angle = pRadar->GetMaxVertAngle();
        m_min_vert_angle = pRadar->GetMinVertAngle();
    } else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }

    m_buffer_out = chrono_types::make_shared<SensorDeviceXYZIBuffer>();
    DeviceXYZIBufferPtr b(cudaMallocHelper<PixelXYZI>(bufferInOut->Width * bufferInOut->Height),
                          cudaFreeHelper<PixelXYZI>);
    m_buffer_out->Buffer = std::move(b);
    m_buffer_out->Width = bufferInOut->Width;
    m_buffer_out->Height = bufferInOut->Height;
    bufferInOut = m_buffer_out;
}
CH_SENSOR_API void ChFilterRadarPCfromRange::Apply() {
    cuda_pointcloud_from_depth(m_buffer_in->Buffer.get(), m_buffer_out->Buffer.get(), (int)m_buffer_in->Width,
                               (int)m_buffer_in->Height, m_hFOV, m_max_vert_angle, m_min_vert_angle, m_cuda_stream);

    auto buf = std::vector<PixelXYZI>(m_buffer_out->Width * m_buffer_out->Height);
    cudaMemcpyAsync(buf.data(), m_buffer_out->Buffer.get(),
                    m_buffer_out->Width * m_buffer_out->Height * sizeof(PixelXYZI), cudaMemcpyDeviceToHost,
                    m_cuda_stream);

    auto new_buf = std::vector<PixelXYZI>(m_buffer_out->Width * m_buffer_out->Height);

    cudaStreamSynchronize(m_cuda_stream);
    m_buffer_out->Beam_return_count = 0;
    for (unsigned int i = 0; i < buf.size(); i++) {
        if (buf[i].intensity > 0) {
            new_buf[m_buffer_out->Beam_return_count] = buf[i];
            m_buffer_out->Beam_return_count++;
        }
    }
    cudaMemcpyAsync(m_buffer_out->Buffer.get(), new_buf.data(), m_buffer_out->Beam_return_count * sizeof(PixelXYZI),
                    cudaMemcpyHostToDevice, m_cuda_stream);

    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
}

}  // namespace sensor
}  // namespace chrono