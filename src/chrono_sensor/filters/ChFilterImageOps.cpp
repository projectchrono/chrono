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
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/cuda/image_ops.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"

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

        // cuda_image_gauss_blur_char(m_buffer_rgba8_in->Buffer.get(), (int)m_buffer_rgba8_in->Width, (int)m_buffer_rgba8_in->Height, sizeof(PixelRGBA8), m_factor,m_cuda_stream);
        cuda_image_alias(m_buffer_rgba8_in->Buffer.get(), m_buffer_rgba8_out->Buffer.get(),
                         (int)m_buffer_rgba8_out->Width, (int)m_buffer_rgba8_out->Height, m_factor, sizeof(PixelRGBA8),
                         m_cuda_stream);
        m_buffer_rgba8_out->LaunchedCount = m_buffer_rgba8_in->LaunchedCount;
        m_buffer_rgba8_out->TimeStamp = m_buffer_rgba8_in->TimeStamp;

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
