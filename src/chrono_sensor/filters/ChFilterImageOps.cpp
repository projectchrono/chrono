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
#include "chrono_sensor/ChSensor.h"
#include "chrono_sensor/cuda/image_ops.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"

#include <npp.h>

namespace chrono {
namespace sensor {

CH_SENSOR_API ChFilterImageResize::ChFilterImageResize(int w, int h, std::string name)
    : m_w(w), m_h(h), ChFilter(name) {}

CH_SENSOR_API void ChFilterImageResize::Apply(std::shared_ptr<ChSensor> pSensor,
                                              std::shared_ptr<SensorBuffer>& bufferInOut) {
    // this filter CANNOT be the first filter in a sensor's filter list, so the bufferIn CANNOT null.
    assert(bufferInOut != nullptr);
    if (!bufferInOut)
        throw std::runtime_error("The lidar reduce filter was not supplied an input buffer");

    if (auto pOpx = std::dynamic_pointer_cast<SensorOptixBuffer>(bufferInOut)) {
        if (pOpx->Buffer->getFormat() != RT_FORMAT_UNSIGNED_BYTE4) {
            throw std::runtime_error(
                "The only optix format that can be resized is  by lidar is RT_FORMAT_UNSIGNED_BYTE4");
        }

        if (!m_buffer_rgba8) {
            m_buffer_rgba8 = chrono_types::make_shared<SensorDeviceRGBA8Buffer>();
            DeviceRGBA8BufferPtr b(cudaMallocHelper<PixelRGBA8>(m_w * m_h), cudaFreeHelper<PixelRGBA8>);
            m_buffer_rgba8->Buffer = std::move(b);
            m_buffer_rgba8->Width = m_w;
            m_buffer_rgba8->Height = m_h;
        }

        // we need id of first device for this context (should only have 1 anyway)
        int device_id = pOpx->Buffer->getContext()->getEnabledDevices()[0];
        void* ptr = pOpx->Buffer->getDevicePointer(device_id);  // hard coded to grab from device 0
        nppiResize_8u_C4R((unsigned char*)ptr, bufferInOut->Width * 4,
                          NppiSize({(int)bufferInOut->Width, (int)bufferInOut->Height}),
                          NppiRect({0, 0, (int)bufferInOut->Width, (int)bufferInOut->Height}),
                          (unsigned char*)m_buffer_rgba8->Buffer.get(), m_w * 4, NppiSize({(int)m_w, (int)m_h}),
                          NppiRect({0, 0, (int)m_w, (int)m_h}), NPPI_INTER_CUBIC);

        m_buffer_rgba8->LaunchedCount = bufferInOut->LaunchedCount;
        m_buffer_rgba8->TimeStamp = bufferInOut->TimeStamp;
        bufferInOut = m_buffer_rgba8;

    } else if (auto pRGBA = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut)) {
        if (!m_buffer_rgba8) {
            m_buffer_rgba8 = chrono_types::make_shared<SensorDeviceRGBA8Buffer>();
            DeviceRGBA8BufferPtr b(cudaMallocHelper<PixelRGBA8>(m_w * m_h), cudaFreeHelper<PixelRGBA8>);
            m_buffer_rgba8->Buffer = std::move(b);
            m_buffer_rgba8->Width = m_w;
            m_buffer_rgba8->Height = m_h;
        }

        nppiResize_8u_C4R((unsigned char*)pRGBA->Buffer.get(), bufferInOut->Width * 4,
                          NppiSize({(int)bufferInOut->Width, (int)bufferInOut->Height}),
                          NppiRect({0, 0, (int)bufferInOut->Width, (int)bufferInOut->Height}),
                          (unsigned char*)m_buffer_rgba8->Buffer.get(), m_w * 4, NppiSize({(int)m_w, (int)m_h}),
                          NppiRect({0, 0, (int)m_w, (int)m_h}), NPPI_INTER_LINEAR);

        m_buffer_rgba8->LaunchedCount = bufferInOut->LaunchedCount;
        m_buffer_rgba8->TimeStamp = bufferInOut->TimeStamp;
        bufferInOut = m_buffer_rgba8;
    } else if (auto pR = std::dynamic_pointer_cast<SensorDeviceR8Buffer>(bufferInOut)) {
        if (!m_buffer_r8) {
            m_buffer_r8 = chrono_types::make_shared<SensorDeviceR8Buffer>();
            DeviceR8BufferPtr b(cudaMallocHelper<char>(m_w * m_h), cudaFreeHelper<char>);
            m_buffer_r8->Buffer = std::move(b);
            m_buffer_r8->Width = m_w;
            m_buffer_r8->Height = m_h;
        }

        nppiResize_8u_C1R((unsigned char*)pR->Buffer.get(), bufferInOut->Width,
                          NppiSize({(int)bufferInOut->Width, (int)bufferInOut->Height}),
                          NppiRect({0, 0, (int)bufferInOut->Width, (int)bufferInOut->Height}),
                          (unsigned char*)m_buffer_r8->Buffer.get(), m_w, NppiSize({(int)m_w, (int)m_h}),
                          NppiRect({0, 0, (int)m_w, (int)m_h}), NPPI_INTER_LINEAR);
        m_buffer_r8->LaunchedCount = bufferInOut->LaunchedCount;
        m_buffer_r8->TimeStamp = bufferInOut->TimeStamp;
        bufferInOut = m_buffer_r8;
    } else {
        throw std::runtime_error("The image resizing filter requires Optix, RGBA8, or R8 buffer");
    }
}

CH_SENSOR_API ChFilterImgAlias::ChFilterImgAlias(int factor, std::string name) : m_factor(factor), ChFilter(name) {}

CH_SENSOR_API void ChFilterImgAlias::Apply(std::shared_ptr<ChSensor> pSensor,
                                           std::shared_ptr<SensorBuffer>& bufferInOut) {
    // this filter CANNOT be the first filter in a sensor's filter list, so the bufferIn CANNOT be null.
    assert(bufferInOut != nullptr);
    if (!bufferInOut)
        throw std::runtime_error("The lidar reduce filter was not supplied an input buffer");

    unsigned int width_out = bufferInOut->Width / m_factor;
    unsigned int height_out = bufferInOut->Height / m_factor;

    // if the buffer is an optix buffer of UBYTE4
    if (auto pOpx = std::dynamic_pointer_cast<SensorOptixBuffer>(bufferInOut)) {
        if (pOpx->Buffer->getFormat() != RT_FORMAT_UNSIGNED_BYTE4) {
            throw std::runtime_error(
                "The only optix format that can be resized is  by lidar is RT_FORMAT_UNSIGNED_BYTE4");
        }

        if (!m_buffer_rgba8) {
            m_buffer_rgba8 = chrono_types::make_shared<SensorDeviceRGBA8Buffer>();
            DeviceRGBA8BufferPtr b(cudaMallocHelper<PixelRGBA8>(width_out * height_out), cudaFreeHelper<PixelRGBA8>);
            m_buffer_rgba8->Buffer = std::move(b);
            m_buffer_rgba8->Width = width_out;
            m_buffer_rgba8->Height = height_out;
        }

        //
        // // we need id of first device for this context (should only have 1 anyway)
        int device_id = pOpx->Buffer->getContext()->getEnabledDevices()[0];
        void* ptr = pOpx->Buffer->getDevicePointer(device_id);  // hard coded to grab from device 0

        // nppiFilterGaussBorder_8u_C4R((unsigned char*)ptr, width * 4, NppiSize({(int)width, (int)height}),
        //                              NppiPoint({0, 0}), (unsigned char*)ptr, width * 4,
        //                              NppiSize({(int)width, (int)height}), NPP_MASK_SIZE_3_X_3, NPP_BORDER_REPLICATE);

        // auto err = nppiResize_8u_C4R((unsigned char*)ptr, width * 4, NppiSize({(int)width, (int)height}),
        //                              NppiRect({0, 0, width, height}), (unsigned char*)m_buffer_rgba8->Buffer.get(),
        //                              width_out * 4, NppiSize({width_out, height_out}),
        //                              NppiRect({0, 0, width_out, height_out}), 2);
        cuda_image_alias(ptr, m_buffer_rgba8->Buffer.get(), (int)width_out, (int)height_out, m_factor,
                         sizeof(PixelRGBA8));

        m_buffer_rgba8->LaunchedCount = bufferInOut->LaunchedCount;
        m_buffer_rgba8->TimeStamp = bufferInOut->TimeStamp;
        bufferInOut = m_buffer_rgba8;
    } else if (auto pRGBA = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut)) {
        if (!m_buffer_rgba8) {
            m_buffer_rgba8 = chrono_types::make_shared<SensorDeviceRGBA8Buffer>();
            DeviceRGBA8BufferPtr b(cudaMallocHelper<PixelRGBA8>(width_out * height_out), cudaFreeHelper<PixelRGBA8>);
            m_buffer_rgba8->Buffer = std::move(b);
            m_buffer_rgba8->Width = bufferInOut->Width / m_factor;
            m_buffer_rgba8->Height = bufferInOut->Height / m_factor;
        }

        // cuda_image_gauss_blur_char(pRGBA->Buffer.get(), (int)width, (int)height, 4, m_factor);

        // nppiFilterGaussBorder_8u_C4R((unsigned char*)pRGBA->Buffer.get(), width * 4,
        //                              NppiSize({(int)width, (int)height}), NppiPoint({0, 0}),
        //                              (unsigned char*)pRGBA->Buffer.get(), width * 4,
        //                              NppiSize({(int)width, (int)height}), NPP_MASK_SIZE_3_X_3, NPP_BORDER_REPLICATE);

        cuda_image_alias(pRGBA->Buffer.get(), m_buffer_rgba8->Buffer.get(), (int)width_out, (int)height_out, m_factor,
                         sizeof(PixelRGBA8));

        m_buffer_rgba8->LaunchedCount = bufferInOut->LaunchedCount;
        m_buffer_rgba8->TimeStamp = bufferInOut->TimeStamp;
        bufferInOut = m_buffer_rgba8;
    } else if (auto pR = std::dynamic_pointer_cast<SensorDeviceR8Buffer>(bufferInOut)) {
        if (!m_buffer_r8) {
            m_buffer_r8 = chrono_types::make_shared<SensorDeviceR8Buffer>();
            DeviceR8BufferPtr b(cudaMallocHelper<char>(width_out * height_out), cudaFreeHelper<char>);
            m_buffer_r8->Buffer = std::move(b);
            m_buffer_r8->Width = bufferInOut->Width / m_factor;
            m_buffer_r8->Height = bufferInOut->Height / m_factor;
        }

        // cuda_image_gauss_blur_char(pR->Buffer.get(), (int)width, (int)height, 1, m_factor);
        cuda_image_alias(pR->Buffer.get(), m_buffer_r8->Buffer.get(), (int)width_out, (int)height_out, m_factor,
                         sizeof(char));

        m_buffer_r8->LaunchedCount = bufferInOut->LaunchedCount;
        m_buffer_r8->TimeStamp = bufferInOut->TimeStamp;
        bufferInOut = m_buffer_r8;
    } else {
        throw std::runtime_error("The image antialiasing downscale filter requires Optix, RGBA8, or R8 buffer");
    }
}

}  // namespace sensor
}  // namespace chrono
