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

#ifdef CHRONO_HAS_OPTIX
#include "chrono_sensor/cuda/pointcloud.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#endif

#include <algorithm>
#include <cmath>
#include <vector>

namespace chrono {
namespace sensor {

namespace {

PixelXYZI DepthToXYZI(const PixelDI& p,
                      unsigned int h_index,
                      unsigned int v_index,
                      unsigned int width,
                      unsigned int height,
                      float hfov,
                      float min_v_angle,
                      float max_v_angle) {
    const float v_angle = (static_cast<float>(v_index) / static_cast<float>(std::max(1u, height - 1u))) *
                              (max_v_angle - min_v_angle) +
                          min_v_angle;
    const float h_angle = (static_cast<float>(h_index) / static_cast<float>(std::max(1u, width - 1u))) * hfov -
                          hfov / 2.f;
    const float proj_xy = p.range * std::cos(v_angle);
    PixelXYZI out{};
    out.x = proj_xy * std::cos(h_angle);
    out.y = proj_xy * std::sin(h_angle);
    out.z = p.range * std::sin(v_angle);
    out.intensity = p.intensity;
    return out;
}

}  // namespace

ChFilterPCfromDepth::ChFilterPCfromDepth(std::string name) : ChFilter(name) {}

void ChFilterPCfromDepth::Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);
    if (!(m_buffer_in = std::dynamic_pointer_cast<SensorDeviceDIBuffer>(bufferInOut)))
        InvalidFilterGraphBufferTypeMismatch(pSensor);

    if (auto pLidar = std::dynamic_pointer_cast<ChLidarSensor>(pSensor)) {
        m_hFOV = static_cast<float>(pLidar->GetHFOV());
        m_min_vert_angle = static_cast<float>(pLidar->GetMinVertAngle());
        m_max_vert_angle = static_cast<float>(pLidar->GetMaxVertAngle());
#ifdef CHRONO_HAS_OPTIX
        m_cuda_stream = pLidar->GetCudaStream();
#endif
    } else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }

    const size_t count = static_cast<size_t>(m_buffer_in->Width) * m_buffer_in->Height * (m_buffer_in->Dual_return ? 2u : 1u);
    m_buffer_out = chrono_types::make_shared<SensorDeviceXYZIBuffer>();
#ifdef CHRONO_HAS_OPTIX
    DeviceXYZIBufferPtr b(cudaMallocHelper<PixelXYZI>(count), cudaFreeHelper<PixelXYZI>);
#else
    DeviceXYZIBufferPtr b(new PixelXYZI[count]);
#endif
    m_buffer_out->Buffer = std::move(b);
    m_buffer_out->Width = m_buffer_in->Width;
    m_buffer_out->Height = m_buffer_in->Height;
    m_buffer_out->Dual_return = m_buffer_in->Dual_return;
    m_buffer_out->Beam_return_count = static_cast<unsigned int>(count);
    bufferInOut = m_buffer_out;
}

void ChFilterPCfromDepth::Apply() {
#ifdef CHRONO_HAS_OPTIX
    if (m_buffer_in->Dual_return) {
        cuda_pointcloud_from_depth_dual_return(m_buffer_in->Buffer.get(), m_buffer_out->Buffer.get(),
                                               (int)m_buffer_in->Width, (int)m_buffer_in->Height, m_hFOV,
                                               m_max_vert_angle, m_min_vert_angle, m_cuda_stream);
    } else {
        cuda_pointcloud_from_depth(m_buffer_in->Buffer.get(), m_buffer_out->Buffer.get(), (int)m_buffer_in->Width,
                                   (int)m_buffer_in->Height, m_hFOV, m_max_vert_angle, m_min_vert_angle, m_cuda_stream);
    }

    m_buffer_out->Beam_return_count = 0;
    auto buf = std::vector<PixelXYZI>(m_buffer_out->Width * m_buffer_out->Height * (m_buffer_out->Dual_return + 1));
    auto processed_buffer = std::vector<PixelXYZI>(buf.size());
    cudaMemcpyAsync(buf.data(), m_buffer_out->Buffer.get(), buf.size() * sizeof(PixelXYZI), cudaMemcpyDeviceToHost,
                    m_cuda_stream);
    cudaStreamSynchronize(m_cuda_stream);
    for (unsigned int i = 0; i < buf.size(); i++) {
        if (buf[i].intensity > 0) {
            processed_buffer[m_buffer_out->Beam_return_count] = buf[i];
            m_buffer_out->Beam_return_count++;
        }
    }
    cudaMemcpyAsync(m_buffer_out->Buffer.get(), processed_buffer.data(),
                    m_buffer_out->Beam_return_count * sizeof(PixelXYZI), cudaMemcpyHostToDevice, m_cuda_stream);
#else
    const unsigned int width = m_buffer_in->Width;
    const unsigned int height = m_buffer_in->Height;
    PixelXYZI* output = m_buffer_out->Buffer.get();
    unsigned int out_count = 0;

    for (unsigned int y = 0; y < height; ++y) {
        for (unsigned int x = 0; x < width; ++x) {
            const size_t idx = static_cast<size_t>(y) * width + x;
            if (m_buffer_in->Dual_return) {
                const PixelXYZI strongest = DepthToXYZI(m_buffer_in->Buffer[2 * idx], x, y, width, height, m_hFOV,
                                                        m_min_vert_angle, m_max_vert_angle);
                if (strongest.intensity > 0.f)
                    output[out_count++] = strongest;
                const PixelXYZI shortest = DepthToXYZI(m_buffer_in->Buffer[2 * idx + 1], x, y, width, height, m_hFOV,
                                                       m_min_vert_angle, m_max_vert_angle);
                if (shortest.intensity > 0.f)
                    output[out_count++] = shortest;
            } else {
                const PixelXYZI point = DepthToXYZI(m_buffer_in->Buffer[idx], x, y, width, height, m_hFOV,
                                                    m_min_vert_angle, m_max_vert_angle);
                if (point.intensity > 0.f)
                    output[out_count++] = point;
            }
        }
    }
    m_buffer_out->Beam_return_count = out_count;
#endif

    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
    m_buffer_out->Dual_return = m_buffer_in->Dual_return;
}
}  // namespace sensor
}  // namespace chrono
