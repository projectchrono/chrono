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
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/cuda/lidar_reduce.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"

namespace chrono {
namespace sensor {

ChFilterLidarReduce::ChFilterLidarReduce(LidarReturnMode ret, int reduce_radius, std::string name)
    : m_ret(ret), m_reduce_radius(reduce_radius), ChFilter(name) {}
CH_SENSOR_API void ChFilterLidarReduce::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                   std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    if (!(m_buffer_in = std::dynamic_pointer_cast<SensorDeviceDIBuffer>(bufferInOut))) {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    }

    if (auto pOpx = std::dynamic_pointer_cast<ChLidarSensor>(pSensor)) {
        m_cuda_stream = pOpx->GetCudaStream();
    } else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }

    switch (m_ret) {
        case LidarReturnMode::DUAL_RETURN: {
            m_buffer_out = chrono_types::make_shared<SensorDeviceDIBuffer>();
            DeviceDIBufferPtr b(cudaMallocHelper<PixelDI>(m_buffer_in->Width * m_buffer_in->Height * 2 /
                                                          ((m_reduce_radius * 2 - 1) * (m_reduce_radius * 2 - 1))),
                                cudaFreeHelper<PixelDI>);
            m_buffer_out->Buffer = std::move(b);
            m_buffer_out->Width = m_buffer_in->Width / (m_reduce_radius * 2 - 1);
            m_buffer_out->Height = m_buffer_in->Height / (m_reduce_radius * 2 - 1);
            m_buffer_out->Dual_return = true;
            break;
        }

        default: {  // all other returns are single, regardless of type
            m_buffer_out = chrono_types::make_shared<SensorDeviceDIBuffer>();
            DeviceDIBufferPtr b(cudaMallocHelper<PixelDI>(m_buffer_in->Width * m_buffer_in->Height /
                                                          ((m_reduce_radius * 2 - 1) * (m_reduce_radius * 2 - 1))),
                                cudaFreeHelper<PixelDI>);
            m_buffer_out->Buffer = std::move(b);
            m_buffer_out->Width = m_buffer_in->Width / (m_reduce_radius * 2 - 1);
            m_buffer_out->Height = m_buffer_in->Height / (m_reduce_radius * 2 - 1);
            m_buffer_out->Dual_return = false;
        }
    }
    bufferInOut = m_buffer_out;
}

CH_SENSOR_API void ChFilterLidarReduce::Apply() {
    switch (m_ret) {
        case LidarReturnMode::DUAL_RETURN:
            cuda_lidar_dual_reduce(m_buffer_in->Buffer.get(), m_buffer_out->Buffer.get(), (int)m_buffer_in->Width,
                                   (int)m_buffer_in->Height, m_reduce_radius, m_cuda_stream);
            break;
        case LidarReturnMode::STRONGEST_RETURN:
            cuda_lidar_strong_reduce(m_buffer_in->Buffer.get(), m_buffer_out->Buffer.get(), (int)m_buffer_in->Width,
                                     (int)m_buffer_in->Height, m_reduce_radius, m_cuda_stream);
            break;
        case LidarReturnMode::FIRST_RETURN:
            cuda_lidar_first_reduce(m_buffer_in->Buffer.get(), m_buffer_out->Buffer.get(), (int)m_buffer_in->Width,
                                    (int)m_buffer_in->Height, m_reduce_radius, m_cuda_stream);
            break;
        default:  // LidarReturnMode::MEAN_RETURN:
            cuda_lidar_mean_reduce(m_buffer_in->Buffer.get(), m_buffer_out->Buffer.get(), (int)m_buffer_in->Width,
                                   (int)m_buffer_in->Height, m_reduce_radius, m_cuda_stream);
            break;
    }

    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
}

}  // namespace sensor
}  // namespace chrono
