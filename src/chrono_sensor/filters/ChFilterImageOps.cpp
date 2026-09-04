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

#include "chrono_sensor/filters/ChFilterImageOps.h"

#if (defined(CHRONO_HAS_VULKAN_RT) || defined(CHRONO_HAS_METAL_RT)) && !defined(CHRONO_HAS_OPTIX)

    #include <algorithm>
    #include <cmath>
    #include <cstdint>
    #include <limits>

namespace chrono {
namespace sensor {

namespace {
uint8_t to_byte(float v) {
    return static_cast<uint8_t>(std::max(0.f, std::min(255.f, v)));
}

uint8_t to_byte_unit(float v) {
    return to_byte(std::max(0.f, std::min(1.f, v)) * 255.f + 0.5f);
}

uint16_t to_u16_unit(float v) {
    v = std::max(0.f, std::min(1.f, v));
    return static_cast<uint16_t>(v * 65535.f + 0.5f);
}

PixelRGBA8 rgba(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255) {
    PixelRGBA8 p;
    p.R = r;
    p.G = g;
    p.B = b;
    p.A = a;
    return p;
}
}

CH_SENSOR_API ChFilterImageHalf4ToRGBA8::ChFilterImageHalf4ToRGBA8(std::string name)
    : ChFilter(name.length() ? name : "ImageHalf4ToRGBA8") {}
CH_SENSOR_API void ChFilterImageHalf4ToRGBA8::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                         std::shared_ptr<SensorBuffer>& bufferInOut) {
    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceHalf4Buffer>(bufferInOut);
    if (!m_buffer_in) {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
        return;
    }
    m_buffer_out = chrono_types::make_shared<SensorDeviceRGBA8Buffer>();
    m_buffer_out->Width = m_buffer_in->Width;
    m_buffer_out->Height = m_buffer_in->Height;
    m_buffer_out->Buffer = std::shared_ptr<PixelRGBA8[]>(new PixelRGBA8[m_buffer_out->Width * m_buffer_out->Height]);
    bufferInOut = m_buffer_out;
}
CH_SENSOR_API void ChFilterImageHalf4ToRGBA8::Apply() {
    if (!m_buffer_in || !m_buffer_out || !m_buffer_in->Buffer)
        return;
    m_buffer_out->Width = m_buffer_in->Width;
    m_buffer_out->Height = m_buffer_in->Height;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    const size_t count = static_cast<size_t>(m_buffer_in->Width) * m_buffer_in->Height;
    for (size_t i = 0; i < count; ++i) {
        const auto& p = m_buffer_in->Buffer[i];
        m_buffer_out->Buffer[i] = rgba(to_byte_unit(p.R), to_byte_unit(p.G), to_byte_unit(p.B), to_byte_unit(p.A));
    }
}

CH_SENSOR_API ChFilterRGBDHalf4ToImageHalf4::ChFilterRGBDHalf4ToImageHalf4(std::string name)
    : ChFilter(name.length() ? name : "RGBDHalf4ToImageHalf4") {}
CH_SENSOR_API void ChFilterRGBDHalf4ToImageHalf4::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                             std::shared_ptr<SensorBuffer>& bufferInOut) {
    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceRGBDHalf4Buffer>(bufferInOut);
    if (!m_buffer_in) {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
        return;
    }
    m_buffer_out = chrono_types::make_shared<SensorDeviceHalf4Buffer>();
    m_buffer_out->Width = m_buffer_in->Width;
    m_buffer_out->Height = m_buffer_in->Height;
    m_buffer_out->Buffer = std::shared_ptr<PixelHalf4[]>(new PixelHalf4[m_buffer_out->Width * m_buffer_out->Height]);
    bufferInOut = m_buffer_out;
}
CH_SENSOR_API void ChFilterRGBDHalf4ToImageHalf4::Apply() {
    if (!m_buffer_in || !m_buffer_out || !m_buffer_in->Buffer)
        return;
    m_buffer_out->Width = m_buffer_in->Width;
    m_buffer_out->Height = m_buffer_in->Height;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    const size_t count = static_cast<size_t>(m_buffer_in->Width) * m_buffer_in->Height;
    for (size_t i = 0; i < count; ++i) {
        m_buffer_out->Buffer[i].R = m_buffer_in->Buffer[i].R;
        m_buffer_out->Buffer[i].G = m_buffer_in->Buffer[i].G;
        m_buffer_out->Buffer[i].B = m_buffer_in->Buffer[i].B;
        m_buffer_out->Buffer[i].A = 1.f;
    }
}

CH_SENSOR_API ChFilterRGBDHalf4ToR8::ChFilterRGBDHalf4ToR8(std::string name)
    : ChFilter(name.length() ? name : "RGBDHalf4ToR8") {}
CH_SENSOR_API void ChFilterRGBDHalf4ToR8::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                     std::shared_ptr<SensorBuffer>& bufferInOut) {
    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceRGBDHalf4Buffer>(bufferInOut);
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
CH_SENSOR_API void ChFilterRGBDHalf4ToR8::Apply() {
    if (!m_buffer_in || !m_buffer_out || !m_buffer_in->Buffer)
        return;
    m_buffer_out->Width = m_buffer_in->Width;
    m_buffer_out->Height = m_buffer_in->Height;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    const size_t count = static_cast<size_t>(m_buffer_in->Width) * m_buffer_in->Height;
    float d_min = std::numeric_limits<float>::max();
    float d_max = -std::numeric_limits<float>::max();
    for (size_t i = 0; i < count; ++i) {
        const float d = m_buffer_in->Buffer[i].D;
        if (std::isfinite(d)) {
            d_min = std::min(d_min, d);
            d_max = std::max(d_max, d);
        }
    }
    if (!(d_max > d_min)) {
        d_min = 0.f;
        d_max = 1.f;
    }
    const float inv_range = 1.f / (d_max - d_min);
    for (size_t i = 0; i < count; ++i)
        m_buffer_out->Buffer[i] = static_cast<char>(to_byte_unit((m_buffer_in->Buffer[i].D - d_min) * inv_range));
}

CH_SENSOR_API ChFilterImageHalf4ToRGBA16::ChFilterImageHalf4ToRGBA16(std::string name)
    : ChFilter(name.length() ? name : "ImageHalf4ToRGBA16") {}
CH_SENSOR_API void ChFilterImageHalf4ToRGBA16::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                          std::shared_ptr<SensorBuffer>& bufferInOut) {
    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceHalf4Buffer>(bufferInOut);
    if (!m_buffer_in) {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
        return;
    }
    m_buffer_out = chrono_types::make_shared<SensorDeviceRGBA16Buffer>();
    m_buffer_out->Width = m_buffer_in->Width;
    m_buffer_out->Height = m_buffer_in->Height;
    m_buffer_out->Buffer = std::shared_ptr<PixelRGBA16[]>(new PixelRGBA16[m_buffer_out->Width * m_buffer_out->Height]);
    bufferInOut = m_buffer_out;
}
CH_SENSOR_API void ChFilterImageHalf4ToRGBA16::Apply() {
    if (!m_buffer_in || !m_buffer_out || !m_buffer_in->Buffer)
        return;
    m_buffer_out->Width = m_buffer_in->Width;
    m_buffer_out->Height = m_buffer_in->Height;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    const size_t count = static_cast<size_t>(m_buffer_in->Width) * m_buffer_in->Height;
    for (size_t i = 0; i < count; ++i) {
        m_buffer_out->Buffer[i].R = to_u16_unit(m_buffer_in->Buffer[i].R);
        m_buffer_out->Buffer[i].G = to_u16_unit(m_buffer_in->Buffer[i].G);
        m_buffer_out->Buffer[i].B = to_u16_unit(m_buffer_in->Buffer[i].B);
        m_buffer_out->Buffer[i].A = to_u16_unit(m_buffer_in->Buffer[i].A);
    }
}

CH_SENSOR_API ChFilterDepthToRGBA8::ChFilterDepthToRGBA8(std::string name)
    : ChFilter(name.length() ? name : "DepthToRGBA8") {}
CH_SENSOR_API void ChFilterDepthToRGBA8::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                    std::shared_ptr<SensorBuffer>& bufferInOut) {
    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceDepthBuffer>(bufferInOut);
    if (!m_buffer_in) {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
        return;
    }
    m_buffer_out = chrono_types::make_shared<SensorDeviceRGBA8Buffer>();
    m_buffer_out->Width = m_buffer_in->Width;
    m_buffer_out->Height = m_buffer_in->Height;
    m_buffer_out->Buffer = std::shared_ptr<PixelRGBA8[]>(new PixelRGBA8[m_buffer_out->Width * m_buffer_out->Height]);
    bufferInOut = m_buffer_out;
}
CH_SENSOR_API void ChFilterDepthToRGBA8::Apply() {
    if (!m_buffer_in || !m_buffer_out || !m_buffer_in->Buffer)
        return;
    const size_t count = static_cast<size_t>(m_buffer_in->Width) * m_buffer_in->Height;
    float min_d = std::numeric_limits<float>::max();
    float max_d = 0.f;
    for (size_t i = 0; i < count; ++i) {
        const float d = m_buffer_in->Buffer[i].depth;
        if (std::isfinite(d)) {
            min_d = std::min(min_d, d);
            max_d = std::max(max_d, d);
        }
    }
    if (max_d <= min_d)
        max_d = min_d + 1.f;
    m_buffer_out->Width = m_buffer_in->Width;
    m_buffer_out->Height = m_buffer_in->Height;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    for (size_t i = 0; i < count; ++i) {
        const float t = 1.f - (m_buffer_in->Buffer[i].depth - min_d) / (max_d - min_d);
        const uint8_t g = to_byte(255.f * t);
        m_buffer_out->Buffer[i] = rgba(g, g, g);
    }
}

CH_SENSOR_API ChFilterNormalToRGBA8::ChFilterNormalToRGBA8(std::string name)
    : ChFilter(name.length() ? name : "NormalToRGBA8") {}
CH_SENSOR_API void ChFilterNormalToRGBA8::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                     std::shared_ptr<SensorBuffer>& bufferInOut) {
    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceNormalBuffer>(bufferInOut);
    if (!m_buffer_in) {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
        return;
    }
    m_buffer_out = chrono_types::make_shared<SensorDeviceRGBA8Buffer>();
    m_buffer_out->Width = m_buffer_in->Width;
    m_buffer_out->Height = m_buffer_in->Height;
    m_buffer_out->Buffer = std::shared_ptr<PixelRGBA8[]>(new PixelRGBA8[m_buffer_out->Width * m_buffer_out->Height]);
    bufferInOut = m_buffer_out;
}
CH_SENSOR_API void ChFilterNormalToRGBA8::Apply() {
    if (!m_buffer_in || !m_buffer_out || !m_buffer_in->Buffer)
        return;
    const size_t count = static_cast<size_t>(m_buffer_in->Width) * m_buffer_in->Height;
    m_buffer_out->Width = m_buffer_in->Width;
    m_buffer_out->Height = m_buffer_in->Height;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    for (size_t i = 0; i < count; ++i) {
        const auto& n = m_buffer_in->Buffer[i];
        m_buffer_out->Buffer[i] = rgba(to_byte((n.normal_x * 0.5f + 0.5f) * 255.f),
                                       to_byte((n.normal_y * 0.5f + 0.5f) * 255.f),
                                       to_byte((n.normal_z * 0.5f + 0.5f) * 255.f));
    }
}

CH_SENSOR_API ChFilterImageResize::ChFilterImageResize(int w, int h, std::string name)
    : ChFilter(name.length() ? name : "ImageResize"), m_w(w), m_h(h) {}
CH_SENSOR_API void ChFilterImageResize::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                   std::shared_ptr<SensorBuffer>& bufferInOut) {
    m_buffer_rgba8_in = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut);
    m_buffer_r8_in = std::dynamic_pointer_cast<SensorDeviceR8Buffer>(bufferInOut);
    if (m_buffer_rgba8_in) {
        m_buffer_rgba8_out = chrono_types::make_shared<SensorDeviceRGBA8Buffer>();
        m_buffer_rgba8_out->Width = m_w;
        m_buffer_rgba8_out->Height = m_h;
        m_buffer_rgba8_out->Buffer = std::shared_ptr<PixelRGBA8[]>(new PixelRGBA8[static_cast<size_t>(m_w) * m_h]);
        bufferInOut = m_buffer_rgba8_out;
    } else if (m_buffer_r8_in) {
        m_buffer_r8_out = chrono_types::make_shared<SensorDeviceR8Buffer>();
        m_buffer_r8_out->Width = m_w;
        m_buffer_r8_out->Height = m_h;
        m_buffer_r8_out->Buffer = std::shared_ptr<char[]>(new char[static_cast<size_t>(m_w) * m_h]);
        bufferInOut = m_buffer_r8_out;
    } else {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    }
}
CH_SENSOR_API void ChFilterImageResize::Apply() {
    if (m_buffer_rgba8_in && m_buffer_rgba8_out) {
        for (int y = 0; y < m_h; ++y) {
            const unsigned int sy = static_cast<unsigned int>(static_cast<double>(y) * m_buffer_rgba8_in->Height / std::max(1, m_h));
            for (int x = 0; x < m_w; ++x) {
                const unsigned int sx = static_cast<unsigned int>(static_cast<double>(x) * m_buffer_rgba8_in->Width / std::max(1, m_w));
                m_buffer_rgba8_out->Buffer[static_cast<size_t>(y) * m_w + x] =
                    m_buffer_rgba8_in->Buffer[static_cast<size_t>(std::min(sy, m_buffer_rgba8_in->Height - 1)) * m_buffer_rgba8_in->Width + std::min(sx, m_buffer_rgba8_in->Width - 1)];
            }
        }
        m_buffer_rgba8_out->TimeStamp = m_buffer_rgba8_in->TimeStamp;
        m_buffer_rgba8_out->LaunchedCount = m_buffer_rgba8_in->LaunchedCount;
    } else if (m_buffer_r8_in && m_buffer_r8_out) {
        for (int y = 0; y < m_h; ++y) {
            const unsigned int sy = static_cast<unsigned int>(static_cast<double>(y) * m_buffer_r8_in->Height / std::max(1, m_h));
            for (int x = 0; x < m_w; ++x) {
                const unsigned int sx = static_cast<unsigned int>(static_cast<double>(x) * m_buffer_r8_in->Width / std::max(1, m_w));
                m_buffer_r8_out->Buffer[static_cast<size_t>(y) * m_w + x] =
                    m_buffer_r8_in->Buffer[static_cast<size_t>(std::min(sy, m_buffer_r8_in->Height - 1)) * m_buffer_r8_in->Width + std::min(sx, m_buffer_r8_in->Width - 1)];
            }
        }
        m_buffer_r8_out->TimeStamp = m_buffer_r8_in->TimeStamp;
        m_buffer_r8_out->LaunchedCount = m_buffer_r8_in->LaunchedCount;
    }
}

CH_SENSOR_API ChFilterImgAlias::ChFilterImgAlias(int factor, std::string name)
    : ChFilter(name.length() ? name : "ImgAlias"), m_factor(std::max(1, factor)) {}
CH_SENSOR_API void ChFilterImgAlias::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                std::shared_ptr<SensorBuffer>& bufferInOut) {
    m_buffer_rgba8_in = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut);
    m_buffer_r8_in = std::dynamic_pointer_cast<SensorDeviceR8Buffer>(bufferInOut);
    if (m_buffer_rgba8_in) {
        m_buffer_rgba8_out = chrono_types::make_shared<SensorDeviceRGBA8Buffer>();
        m_buffer_rgba8_out->Width = std::max(1u, m_buffer_rgba8_in->Width / static_cast<unsigned int>(m_factor));
        m_buffer_rgba8_out->Height = std::max(1u, m_buffer_rgba8_in->Height / static_cast<unsigned int>(m_factor));
        m_buffer_rgba8_out->Buffer = std::shared_ptr<PixelRGBA8[]>(new PixelRGBA8[m_buffer_rgba8_out->Width * m_buffer_rgba8_out->Height]);
        bufferInOut = m_buffer_rgba8_out;
    } else if (m_buffer_r8_in) {
        m_buffer_r8_out = chrono_types::make_shared<SensorDeviceR8Buffer>();
        m_buffer_r8_out->Width = std::max(1u, m_buffer_r8_in->Width / static_cast<unsigned int>(m_factor));
        m_buffer_r8_out->Height = std::max(1u, m_buffer_r8_in->Height / static_cast<unsigned int>(m_factor));
        m_buffer_r8_out->Buffer = std::shared_ptr<char[]>(new char[m_buffer_r8_out->Width * m_buffer_r8_out->Height]);
        bufferInOut = m_buffer_r8_out;
    } else {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    }
}
CH_SENSOR_API void ChFilterImgAlias::Apply() {
    if (m_buffer_rgba8_in && m_buffer_rgba8_out) {
        for (unsigned int y = 0; y < m_buffer_rgba8_out->Height; ++y)
            for (unsigned int x = 0; x < m_buffer_rgba8_out->Width; ++x)
                m_buffer_rgba8_out->Buffer[static_cast<size_t>(y) * m_buffer_rgba8_out->Width + x] =
                    m_buffer_rgba8_in->Buffer[static_cast<size_t>(std::min(m_buffer_rgba8_in->Height - 1, y * static_cast<unsigned int>(m_factor))) * m_buffer_rgba8_in->Width + std::min(m_buffer_rgba8_in->Width - 1, x * static_cast<unsigned int>(m_factor))];
        m_buffer_rgba8_out->TimeStamp = m_buffer_rgba8_in->TimeStamp;
        m_buffer_rgba8_out->LaunchedCount = m_buffer_rgba8_in->LaunchedCount;
    } else if (m_buffer_r8_in && m_buffer_r8_out) {
        for (unsigned int y = 0; y < m_buffer_r8_out->Height; ++y)
            for (unsigned int x = 0; x < m_buffer_r8_out->Width; ++x)
                m_buffer_r8_out->Buffer[static_cast<size_t>(y) * m_buffer_r8_out->Width + x] =
                    m_buffer_r8_in->Buffer[static_cast<size_t>(std::min(m_buffer_r8_in->Height - 1, y * static_cast<unsigned int>(m_factor))) * m_buffer_r8_in->Width + std::min(m_buffer_r8_in->Width - 1, x * static_cast<unsigned int>(m_factor))];
        m_buffer_r8_out->TimeStamp = m_buffer_r8_in->TimeStamp;
        m_buffer_r8_out->LaunchedCount = m_buffer_r8_in->LaunchedCount;
    }
}

}  // namespace sensor
}  // namespace chrono

#else

#include "chrono_sensor/filters/ChFilterImageOps.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChPhysCameraSensor.h"
#include "chrono_sensor/cuda/image_ops.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#include <iostream>

namespace chrono {
namespace sensor {

CH_SENSOR_API ChFilterImageHalf4ToRGBA8::ChFilterImageHalf4ToRGBA8(std::string name) : ChFilter(name) {}
CH_SENSOR_API void ChFilterImageHalf4ToRGBA8::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                         std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    if (auto pOpx = std::dynamic_pointer_cast<ChOptixSensor>(pSensor)) {
        m_cuda_stream = pOpx->GetCudaStream();

    } else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }

    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceHalf4Buffer>(bufferInOut);
    if (m_buffer_in) {
        m_buffer_out = chrono_types::make_shared<SensorDeviceRGBA8Buffer>();
        DeviceRGBA8BufferPtr b(cudaMallocHelper<PixelRGBA8>(m_buffer_in->Width * m_buffer_in->Height),
                               cudaFreeHelper<PixelRGBA8>);
        m_buffer_out->Buffer = std::move(b);
        m_buffer_out->Width = m_buffer_in->Width;
        m_buffer_out->Height = m_buffer_in->Height;
        bufferInOut = m_buffer_out;
    } else {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    }
}

CH_SENSOR_API void ChFilterImageHalf4ToRGBA8::Apply() {
    cuda_image_half4_to_uchar4(m_buffer_in->Buffer.get(), m_buffer_out->Buffer.get(), m_buffer_out->Width,
                               m_buffer_out->Height, m_cuda_stream);

    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
}


//------------------------------------------------------//
// Member functions of ChFilterImageHalf4ToRGBA16 class //
//------------------------------------------------------//

CH_SENSOR_API ChFilterImageHalf4ToRGBA16::ChFilterImageHalf4ToRGBA16(std::string name) : ChFilter(name) {}

CH_SENSOR_API void ChFilterImageHalf4ToRGBA16::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                          std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    if (auto pOpx = std::dynamic_pointer_cast<ChOptixSensor>(pSensor)) {
        m_cuda_stream = pOpx->GetCudaStream();
    }
    else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }

    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceHalf4Buffer>(bufferInOut);
    if (m_buffer_in) {
        m_buffer_out = chrono_types::make_shared<SensorDeviceRGBA16Buffer>();
        DeviceRGBA16BufferPtr b(cudaMallocHelper<PixelRGBA16>(m_buffer_in->Width * m_buffer_in->Height),
                               cudaFreeHelper<PixelRGBA16>);
        m_buffer_out->Buffer = std::move(b);
        m_buffer_out->Width = m_buffer_in->Width;
        m_buffer_out->Height = m_buffer_in->Height;
        bufferInOut = m_buffer_out;
    }
    else {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    }
}

CH_SENSOR_API void ChFilterImageHalf4ToRGBA16::Apply() {
    cuda_image_half4_to_uint16_t4(m_buffer_in->Buffer.get(), m_buffer_out->Buffer.get(), m_buffer_out->Width,
                                  m_buffer_out->Height, m_cuda_stream);

    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
    
}


//---------------------------------------------------------//
// Member functions of ChFilterRGBDHalf4ToImageHalf4 class //
//---------------------------------------------------------//

CH_SENSOR_API ChFilterRGBDHalf4ToImageHalf4::ChFilterRGBDHalf4ToImageHalf4(std::string name) : ChFilter(name) {}

CH_SENSOR_API void ChFilterRGBDHalf4ToImageHalf4::Initialize(
    std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut
    ) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    if (auto pOpx = std::dynamic_pointer_cast<ChOptixSensor>(pSensor)) {
        m_cuda_stream = pOpx->GetCudaStream();
    }
    else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }

    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceRGBDHalf4Buffer>(bufferInOut);
    if (m_buffer_in) {
        m_buffer_out = chrono_types::make_shared<SensorDeviceHalf4Buffer>();
        DeviceHalf4BufferPtr b(cudaMallocHelper<PixelHalf4>(m_buffer_in->Width * m_buffer_in->Height),
                               cudaFreeHelper<PixelHalf4>);
        m_buffer_out->Buffer = std::move(b);
        m_buffer_out->Width = m_buffer_in->Width;
        m_buffer_out->Height = m_buffer_in->Height;
        bufferInOut = m_buffer_out;
    }
    else {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    }
}

CH_SENSOR_API void ChFilterRGBDHalf4ToImageHalf4::Apply() {
    cuda_image_RGBDhalf4_to_Half4(
        m_buffer_in->Buffer.get(), m_buffer_out->Buffer.get(), m_buffer_out->Width, m_buffer_out->Height, m_cuda_stream
    );
    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
}

CH_SENSOR_API ChFilterDepthToRGBA8::ChFilterDepthToRGBA8(std::string name) : ChFilter(name) {}
CH_SENSOR_API void ChFilterDepthToRGBA8::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                         std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    if (auto pOpx = std::dynamic_pointer_cast<ChOptixSensor>(pSensor)) {
        m_cuda_stream = pOpx->GetCudaStream();

    } else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }

    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceDepthBuffer>(bufferInOut);
    if (m_buffer_in) {
        m_buffer_out = chrono_types::make_shared<SensorDeviceRGBA8Buffer>();
        DeviceRGBA8BufferPtr b(cudaMallocHelper<PixelRGBA8>(m_buffer_in->Width * m_buffer_in->Height),
                               cudaFreeHelper<PixelRGBA8>);
        m_buffer_out->Buffer = std::move(b);
        m_buffer_out->Width = m_buffer_in->Width;
        m_buffer_out->Height = m_buffer_in->Height;
        bufferInOut = m_buffer_out;

     
    } else {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    }
}

CH_SENSOR_API void ChFilterDepthToRGBA8::Apply() {
    cuda_depth_to_uchar4(m_buffer_in->Buffer.get(), m_buffer_out->Buffer.get(), m_buffer_out->Width,
                               m_buffer_out->Height, m_cuda_stream);

    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
}


//-------------------------------------------------//
// Member functions of ChFilterRGBDHalf4ToR8 class //
//-------------------------------------------------//

CH_SENSOR_API ChFilterRGBDHalf4ToR8::ChFilterRGBDHalf4ToR8(std::string name) : ChFilter(name) {}
CH_SENSOR_API void ChFilterRGBDHalf4ToR8::Initialize(
    std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut
    ) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    if (auto pOpx = std::dynamic_pointer_cast<ChOptixSensor>(pSensor)) {
        m_cuda_stream = pOpx->GetCudaStream();
    }
    else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }

    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceRGBDHalf4Buffer>(bufferInOut);
    if (m_buffer_in) {
        // create output buffer
        m_buffer_out = chrono_types::make_shared<SensorDeviceR8Buffer>();
        DeviceR8BufferPtr b(cudaMallocHelper<char>(m_buffer_in->Width * m_buffer_in->Height),
                            cudaFreeHelper<char>);
        m_buffer_out->Buffer = std::move(b);
        m_buffer_out->Width = m_buffer_in->Width;
        m_buffer_out->Height = m_buffer_in->Height;
        bufferInOut = m_buffer_out;
    }
    else {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    }
}

CH_SENSOR_API void ChFilterRGBDHalf4ToR8::Apply() {
    cuda_RGBDhalf4_to_uchar(
        m_buffer_in->Buffer.get(), m_buffer_out->Buffer.get(), m_buffer_out->Width, m_buffer_out->Height, m_cuda_stream
    );
    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
}


//-------------------------------------------------//
// Member functions of ChFilterNormalToRGBA8 class //
//-------------------------------------------------//

CH_SENSOR_API ChFilterNormalToRGBA8::ChFilterNormalToRGBA8(std::string name) : ChFilter(name) {}

CH_SENSOR_API void ChFilterNormalToRGBA8::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                     std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    if (auto pOpx = std::dynamic_pointer_cast<ChOptixSensor>(pSensor)) {
        m_cuda_stream = pOpx->GetCudaStream();
    }
    else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }

    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceNormalBuffer>(bufferInOut);
    if (m_buffer_in) {
        m_buffer_out = chrono_types::make_shared<SensorDeviceRGBA8Buffer>();
        DeviceRGBA8BufferPtr b(cudaMallocHelper<PixelRGBA8>(m_buffer_in->Width * m_buffer_in->Height),
                               cudaFreeHelper<PixelRGBA8>);
        m_buffer_out->Buffer = std::move(b);
        m_buffer_out->Width = m_buffer_in->Width;
        m_buffer_out->Height = m_buffer_in->Height;
        bufferInOut = m_buffer_out;
    }
    else {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    }
}

CH_SENSOR_API void ChFilterNormalToRGBA8::Apply() {
    cuda_normal_to_uchar4(m_buffer_in->Buffer.get(), m_buffer_out->Buffer.get(), m_buffer_out->Width,
                          m_buffer_out->Height, m_cuda_stream);
    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;
}


CH_SENSOR_API ChFilterImageResize::ChFilterImageResize(int w, int h, std::string name)
    : m_w(w), m_h(h), ChFilter(name) {}

CH_SENSOR_API void ChFilterImageResize::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                   std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    if (auto pCam = std::dynamic_pointer_cast<ChCameraSensor>(pSensor)) {
        m_cuda_stream = {};
        m_cuda_stream.hStream = pCam->GetCudaStream();
        m_cuda_stream.nCudaDeviceId = 0;  // TODO: allow multiple GPU usage

    } else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }

    m_buffer_rgba8_in = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut);
    m_buffer_r8_in = std::dynamic_pointer_cast<SensorDeviceR8Buffer>(bufferInOut);

    if (m_buffer_rgba8_in) {
        m_buffer_rgba8_out = chrono_types::make_shared<SensorDeviceRGBA8Buffer>();
        DeviceRGBA8BufferPtr b(cudaMallocHelper<PixelRGBA8>(m_w * m_h), cudaFreeHelper<PixelRGBA8>);
        m_buffer_rgba8_out->Buffer = std::move(b);
        m_buffer_rgba8_out->Width = m_w;
        m_buffer_rgba8_out->Height = m_h;
        bufferInOut = m_buffer_rgba8_out;
    } else if (m_buffer_r8_in) {
        m_buffer_r8_out = chrono_types::make_shared<SensorDeviceR8Buffer>();
        DeviceR8BufferPtr b(cudaMallocHelper<char>(m_w * m_h), cudaFreeHelper<char>);
        m_buffer_r8_out->Buffer = std::move(b);
        m_buffer_r8_out->Width = m_w;
        m_buffer_r8_out->Height = m_h;
        bufferInOut = m_buffer_r8_out;
    } else {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    }
}

CH_SENSOR_API void ChFilterImageResize::Apply() {
    if (m_buffer_rgba8_in) {
        nppiResize_8u_C4R_Ctx((unsigned char*)m_buffer_rgba8_in->Buffer.get(), m_buffer_rgba8_in->Width * 4,
                              NppiSize({(int)m_buffer_rgba8_in->Width, (int)m_buffer_rgba8_in->Height}),
                              NppiRect({0, 0, (int)m_buffer_rgba8_in->Width, (int)m_buffer_rgba8_in->Height}),
                              (unsigned char*)m_buffer_rgba8_out->Buffer.get(), m_w * 4, NppiSize({(int)m_w, (int)m_h}),
                              NppiRect({0, 0, (int)m_w, (int)m_h}), NPPI_INTER_LINEAR, m_cuda_stream);
        m_buffer_rgba8_out->LaunchedCount = m_buffer_rgba8_in->LaunchedCount;
        m_buffer_rgba8_out->TimeStamp = m_buffer_rgba8_in->TimeStamp;

    } else if (m_buffer_r8_in) {
        nppiResize_8u_C1R_Ctx((unsigned char*)m_buffer_r8_in->Buffer.get(), m_buffer_r8_in->Width,
                              NppiSize({(int)m_buffer_r8_in->Width, (int)m_buffer_r8_in->Height}),
                              NppiRect({0, 0, (int)m_buffer_r8_in->Width, (int)m_buffer_r8_in->Height}),
                              (unsigned char*)m_buffer_r8_out->Buffer.get(), m_w, NppiSize({(int)m_w, (int)m_h}),
                              NppiRect({0, 0, (int)m_w, (int)m_h}), NPPI_INTER_LINEAR, m_cuda_stream);
        m_buffer_r8_out->LaunchedCount = m_buffer_r8_in->LaunchedCount;
        m_buffer_r8_out->TimeStamp = m_buffer_r8_in->TimeStamp;
    }
}

CH_SENSOR_API ChFilterImgAlias::ChFilterImgAlias(int factor, std::string name) : m_factor(factor), ChFilter(name) {}

CH_SENSOR_API void ChFilterImgAlias::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    if (auto pCam = std::dynamic_pointer_cast<ChCameraSensor>(pSensor)) {
        m_cuda_stream = pCam->GetCudaStream();
        // m_cuda_stream = {};
        // m_cuda_stream.hStream = pCam->GetCudaStream();
        // m_cuda_stream.nCudaDeviceId = 0;  // TODO: allow multiple GPU usage
    }
    else if (auto p_phys_cam = std::dynamic_pointer_cast<ChPhysCameraSensor>(pSensor)) {
        m_cuda_stream = p_phys_cam->GetCudaStream();
        // m_cuda_stream = {};
        // m_cuda_stream.hStream = pCam->GetCudaStream();
        // m_cuda_stream.nCudaDeviceId = 0;  // TODO: allow multiple GPU usage
    } else {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }
    
    unsigned int width_out = bufferInOut->Width / m_factor;
    unsigned int height_out = bufferInOut->Height / m_factor;

    m_buffer_rgba8_in = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut);
    m_buffer_r8_in = std::dynamic_pointer_cast<SensorDeviceR8Buffer>(bufferInOut);
    m_buffer_float4_in = std::dynamic_pointer_cast<SensorDeviceFloat4Buffer>(bufferInOut);

    if (m_buffer_rgba8_in) {
        m_buffer_rgba8_out = chrono_types::make_shared<SensorDeviceRGBA8Buffer>();
        DeviceRGBA8BufferPtr b(cudaMallocHelper<PixelRGBA8>(width_out * height_out), cudaFreeHelper<PixelRGBA8>);
        m_buffer_rgba8_out->Buffer = std::move(b);
        m_buffer_rgba8_out->Width = width_out;
        m_buffer_rgba8_out->Height = height_out;
        bufferInOut = m_buffer_rgba8_out;
    } else if (m_buffer_rgba16_in = std::dynamic_pointer_cast<SensorDeviceRGBA16Buffer>(bufferInOut)) {
        m_buffer_rgba16_out = chrono_types::make_shared<SensorDeviceRGBA16Buffer>();
        DeviceRGBA16BufferPtr b(cudaMallocHelper<PixelRGBA16>(width_out * height_out), cudaFreeHelper<PixelRGBA16>);
        m_buffer_rgba16_out->Buffer = std::move(b);
        m_buffer_rgba16_out->Width = width_out;
        m_buffer_rgba16_out->Height = height_out;
        bufferInOut = m_buffer_rgba16_out;
    } else if (m_buffer_r8_in) {
        m_buffer_r8_out = chrono_types::make_shared<SensorDeviceR8Buffer>();
        DeviceR8BufferPtr b(cudaMallocHelper<char>(width_out * height_out), cudaFreeHelper<char>);
        m_buffer_r8_out->Buffer = std::move(b);
        m_buffer_r8_out->Width = width_out;
        m_buffer_r8_out->Height = height_out;
        bufferInOut = m_buffer_r8_out;
    } else if (m_buffer_float4_in) {
        m_buffer_float4_out = chrono_types::make_shared<SensorDeviceFloat4Buffer>();
        DeviceFloat4BufferPtr b(cudaMallocHelper<PixelFloat4>(width_out * height_out), cudaFreeHelper<PixelFloat4>);
        m_buffer_float4_out->Buffer = std::move(b);
        m_buffer_float4_out->Width = width_out;
        m_buffer_float4_out->Height = height_out;
        bufferInOut = m_buffer_float4_out;
    } else {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    }
}
CH_SENSOR_API void ChFilterImgAlias::Apply() {
    // if (m_buffer_rgba8_in) {
    //     nppiFilterBoxBorder_8u_AC4R_Ctx((unsigned char*)m_buffer_rgba8_in->Buffer.get(), Npp32s nSrcStep,
    //                                     NppiSize({(int)m_buffer_rgba8_in->Width, (int)m_buffer_rgba8_in->Height}),
    //                                     NppiPoint oSrcOffset, (unsigned char*)m_buffer_rgba8_out->Buffer.get(),
    //                                     Npp32s nDstStep,
    //                                     NppiRect({0, 0, (int)m_buffer_rgba8_in->Width,
    //                                     (int)m_buffer_rgba8_in->Height}), NppiSize({m_factor+2,m_factor+2}),
    //                                     NppiPoint({0, 0}), NppiBorderType eBorderType, m_cuda_stream);
    //     // cuda_image_alias(m_buffer_rgba8_in->Buffer.get(), m_buffer_rgba8_out->Buffer.get(),
    //     //                  (int)m_buffer_rgba8_out->Width, (int)m_buffer_rgba8_out->Height, m_factor,
    //     //                  sizeof(PixelRGBA8), m_cuda_stream);
    //     m_buffer_rgba8_out->LaunchedCount = m_buffer_rgba8_in->LaunchedCount;
    //     m_buffer_rgba8_out->TimeStamp = m_buffer_rgba8_in->TimeStamp;
    // } else if (m_buffer_r8_in) {
    //     // cuda_image_alias(m_buffer_r8_in->Buffer.get(), m_buffer_r8_out->Buffer.get(), (int)m_buffer_r8_out->Width,
    //     //                  (int)m_buffer_r8_out->Height, m_factor, sizeof(char), m_cuda_stream);
    //     m_buffer_r8_out->LaunchedCount = m_buffer_r8_in->LaunchedCount;
    //     m_buffer_r8_out->TimeStamp = m_buffer_r8_in->TimeStamp;
    // } else if (m_buffer_float4_in) {
    //     // cuda_image_alias_float(m_buffer_float4_in->Buffer.get(), m_buffer_float4_out->Buffer.get(),
    //     //                        (int)m_buffer_float4_out->Width, (int)m_buffer_float4_out->Height, m_factor, 4,
    //     //                        m_cuda_stream);
    //     m_buffer_float4_out->LaunchedCount = m_buffer_float4_in->LaunchedCount;
    //     m_buffer_float4_out->TimeStamp = m_buffer_float4_in->TimeStamp;
    // }
    if (m_buffer_rgba8_in) {
        // cuda_image_gauss_blur_char(m_buffer_rgba8_in->Buffer.get(), (int)m_buffer_rgba8_in->Width,
        // (int)m_buffer_rgba8_in->Height, sizeof(PixelRGBA8), m_factor,m_cuda_stream);
        cuda_image_alias(m_buffer_rgba8_in->Buffer.get(), m_buffer_rgba8_out->Buffer.get(),
                         (int)m_buffer_rgba8_out->Width, (int)m_buffer_rgba8_out->Height, m_factor, sizeof(PixelRGBA8),
                         m_cuda_stream);
        m_buffer_rgba8_out->LaunchedCount = m_buffer_rgba8_in->LaunchedCount;
        m_buffer_rgba8_out->TimeStamp = m_buffer_rgba8_in->TimeStamp;

    } else if (m_buffer_rgba16_in) {
        cuda_image_alias_rgba16(
            m_buffer_rgba16_in->Buffer.get(), m_buffer_rgba16_out->Buffer.get(), (int)m_buffer_rgba16_out->Width,
            (int)m_buffer_rgba16_out->Height, m_factor, 4, m_cuda_stream
        );
        m_buffer_rgba16_out->LaunchedCount = m_buffer_rgba16_in->LaunchedCount;
        m_buffer_rgba16_out->TimeStamp = m_buffer_rgba16_in->TimeStamp;
    } else if (m_buffer_r8_in) {
        cuda_image_alias(m_buffer_r8_in->Buffer.get(), m_buffer_r8_out->Buffer.get(), (int)m_buffer_r8_out->Width,
                         (int)m_buffer_r8_out->Height, m_factor, sizeof(char), m_cuda_stream);
        m_buffer_r8_out->LaunchedCount = m_buffer_r8_in->LaunchedCount;
        m_buffer_r8_out->TimeStamp = m_buffer_r8_in->TimeStamp;
    } else if (m_buffer_float4_in) {
        cuda_image_alias_float(m_buffer_float4_in->Buffer.get(), m_buffer_float4_out->Buffer.get(),
                               (int)m_buffer_float4_out->Width, (int)m_buffer_float4_out->Height, m_factor, 4,
                               m_cuda_stream);
        m_buffer_float4_out->LaunchedCount = m_buffer_float4_in->LaunchedCount;
        m_buffer_float4_out->TimeStamp = m_buffer_float4_in->TimeStamp;
    }
}

}  // namespace sensor
}  // namespace chrono

#endif
