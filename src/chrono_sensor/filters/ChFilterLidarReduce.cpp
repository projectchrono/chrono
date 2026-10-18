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

#ifdef CHRONO_HAS_OPTIX
#include "chrono_sensor/cuda/lidar_reduce.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#endif

#include <algorithm>
#include <cmath>
#include <limits>

namespace chrono {
namespace sensor {

namespace {

PixelDI MakeEmptyReturn() {
    PixelDI p{};
    p.range = 0.f;
    p.intensity = 0.f;
    return p;
}

#ifndef CHRONO_HAS_OPTIX
PixelDI ReduceMean(const PixelDI* input, unsigned int in_width, unsigned int out_x, unsigned int out_y, int radius) {
    const int d = radius * 2 - 1;
    float sum_range = 0.f;
    float sum_intensity = 0.f;
    int n_contributing = 0;
    for (int iy = 0; iy < d; ++iy) {
        for (int ix = 0; ix < d; ++ix) {
            const size_t in_index = static_cast<size_t>(d * out_y + iy) * in_width + static_cast<size_t>(d * out_x + ix);
            const PixelDI sample = input[in_index];
            sum_intensity += sample.intensity;
            if (sample.intensity > 1e-6f) {
                sum_range += sample.range;
                ++n_contributing;
            }
        }
    }
    PixelDI out = MakeEmptyReturn();
    if (n_contributing > 0) {
        out.range = sum_range / static_cast<float>(n_contributing);
        out.intensity = sum_intensity / static_cast<float>(d * d);
    }
    return out;
}

PixelDI ReduceStrongest(const PixelDI* input, unsigned int in_width, unsigned int out_x, unsigned int out_y, int radius) {
    const int d = radius * 2 - 1;
    const float kernel_radius = 0.05f;
    PixelDI out = MakeEmptyReturn();
    float intensity_at_strongest = 0.f;

    for (int iy = 0; iy < d; ++iy) {
        for (int ix = 0; ix < d; ++ix) {
            const size_t in_index = static_cast<size_t>(d * out_y + iy) * in_width + static_cast<size_t>(d * out_x + ix);
            const float local_range = input[in_index].range;
            float local_intensity = input[in_index].intensity;

            for (int ky = 0; ky < d; ++ky) {
                for (int kx = 0; kx < d; ++kx) {
                    const size_t inner_index = static_cast<size_t>(d * out_y + ky) * in_width + static_cast<size_t>(d * out_x + kx);
                    const float range = input[inner_index].range;
                    const float intensity = input[inner_index].intensity;
                    const float diff = std::abs(range - local_range);
                    if (inner_index != in_index && diff < kernel_radius)
                        local_intensity += ((kernel_radius - diff) / kernel_radius) * intensity;
                }
            }

            local_intensity /= static_cast<float>(d * d);
            if (local_intensity > intensity_at_strongest) {
                intensity_at_strongest = local_intensity;
                out.range = local_range;
                out.intensity = local_intensity;
            }
        }
    }
    return out;
}

PixelDI ReduceFirst(const PixelDI* input, unsigned int in_width, unsigned int out_x, unsigned int out_y, int radius) {
    const int d = radius * 2 - 1;
    const float kernel_radius = 0.05f;
    PixelDI out = MakeEmptyReturn();
    float shortest = std::numeric_limits<float>::max();

    for (int iy = 0; iy < d; ++iy) {
        for (int ix = 0; ix < d; ++ix) {
            const size_t in_index = static_cast<size_t>(d * out_y + iy) * in_width + static_cast<size_t>(d * out_x + ix);
            const float local_range = input[in_index].range;
            const float ray_intensity = input[in_index].intensity;
            float local_intensity = ray_intensity;

            for (int ky = 0; ky < d; ++ky) {
                for (int kx = 0; kx < d; ++kx) {
                    const size_t inner_index = static_cast<size_t>(d * out_y + ky) * in_width + static_cast<size_t>(d * out_x + kx);
                    const float range = input[inner_index].range;
                    const float intensity = input[inner_index].intensity;
                    const float diff = std::abs(range - local_range);
                    if (inner_index != in_index && diff < kernel_radius)
                        local_intensity += ((kernel_radius - diff) / kernel_radius) * intensity;
                }
            }

            local_intensity /= static_cast<float>(d * d);
            if (ray_intensity > 0.f && local_range < shortest) {
                shortest = local_range;
                out.range = local_range;
                out.intensity = local_intensity;
            }
        }
    }
    return out;
}

PixelDI ReduceLast(const PixelDI* input, unsigned int in_width, unsigned int out_x, unsigned int out_y, int radius) {
    const int d = radius * 2 - 1;
    PixelDI out = MakeEmptyReturn();
    float longest = -std::numeric_limits<float>::max();
    for (int iy = 0; iy < d; ++iy) {
        for (int ix = 0; ix < d; ++ix) {
            const size_t in_index = static_cast<size_t>(d * out_y + iy) * in_width + static_cast<size_t>(d * out_x + ix);
            const PixelDI sample = input[in_index];
            if (sample.intensity > 0.f && sample.range > longest) {
                longest = sample.range;
                out = sample;
            }
        }
    }
    return out;
}
#endif

}  // namespace

ChFilterLidarReduce::ChFilterLidarReduce(LidarReturnMode ret, int reduce_radius, std::string name)
    : m_ret(ret), m_reduce_radius(reduce_radius), ChFilter(name) {}

void ChFilterLidarReduce::Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    if (!(m_buffer_in = std::dynamic_pointer_cast<SensorDeviceDIBuffer>(bufferInOut)))
        InvalidFilterGraphBufferTypeMismatch(pSensor);

    if (auto pLidar = std::dynamic_pointer_cast<ChLidarSensor>(pSensor)) {
#ifdef CHRONO_HAS_OPTIX
        m_cuda_stream = pLidar->GetCudaStream();
#endif
    } else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }

    const unsigned int sample_dim = std::max(1, m_reduce_radius * 2 - 1);
    const unsigned int out_w = std::max(1u, m_buffer_in->Width / sample_dim);
    const unsigned int out_h = std::max(1u, m_buffer_in->Height / sample_dim);
    const size_t out_count = static_cast<size_t>(out_w) * out_h;
    const bool dual = m_ret == LidarReturnMode::DUAL_RETURN;

    m_buffer_out = chrono_types::make_shared<SensorDeviceDIBuffer>();
#ifdef CHRONO_HAS_OPTIX
    DeviceDIBufferPtr b(cudaMallocHelper<PixelDI>(out_count * (dual ? 2u : 1u)), cudaFreeHelper<PixelDI>);
#else
    DeviceDIBufferPtr b(new PixelDI[out_count * (dual ? 2u : 1u)]);
#endif
    m_buffer_out->Buffer = std::move(b);
    m_buffer_out->Width = out_w;
    m_buffer_out->Height = out_h;
    m_buffer_out->Dual_return = dual;
    m_buffer_out->Beam_return_count = static_cast<unsigned int>(out_count * (dual ? 2u : 1u));
    bufferInOut = m_buffer_out;
}

void ChFilterLidarReduce::Apply() {
#ifdef CHRONO_HAS_OPTIX
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
        default:
            cuda_lidar_mean_reduce(m_buffer_in->Buffer.get(), m_buffer_out->Buffer.get(), (int)m_buffer_in->Width,
                                   (int)m_buffer_in->Height, m_reduce_radius, m_cuda_stream);
            break;
    }
#else
    const PixelDI* input = m_buffer_in->Buffer.get();
    PixelDI* output = m_buffer_out->Buffer.get();
    const unsigned int out_w = m_buffer_out->Width;
    const unsigned int out_h = m_buffer_out->Height;
    for (unsigned int y = 0; y < out_h; ++y) {
        for (unsigned int x = 0; x < out_w; ++x) {
            const size_t out_idx = static_cast<size_t>(y) * out_w + x;
            if (m_ret == LidarReturnMode::DUAL_RETURN) {
                output[2 * out_idx] = ReduceStrongest(input, m_buffer_in->Width, x, y, m_reduce_radius);
                output[2 * out_idx + 1] = ReduceFirst(input, m_buffer_in->Width, x, y, m_reduce_radius);
            } else if (m_ret == LidarReturnMode::STRONGEST_RETURN) {
                output[out_idx] = ReduceStrongest(input, m_buffer_in->Width, x, y, m_reduce_radius);
            } else if (m_ret == LidarReturnMode::FIRST_RETURN) {
                output[out_idx] = ReduceFirst(input, m_buffer_in->Width, x, y, m_reduce_radius);
            } else if (m_ret == LidarReturnMode::LAST_RETURN) {
                output[out_idx] = ReduceLast(input, m_buffer_in->Width, x, y, m_reduce_radius);
            } else {
                output[out_idx] = ReduceMean(input, m_buffer_in->Width, x, y, m_reduce_radius);
            }
        }
    }
#endif

    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
    m_buffer_out->Beam_return_count = m_buffer_out->Width * m_buffer_out->Height * (m_buffer_out->Dual_return ? 2u : 1u);
}

}  // namespace sensor
}  // namespace chrono
