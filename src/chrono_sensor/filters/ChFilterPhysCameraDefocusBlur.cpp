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
// Authors: Bo-Hsun Chen
// =============================================================================
// 
// Filter to do defocus blur based on camera control parameters and depth map
// 
// =============================================================================

#include "chrono_sensor/filters/ChFilterPhysCameraDefocusBlur.h"
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/cuda/phys_cam_ops.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#include <chrono>

namespace chrono {
namespace sensor {
ChFilterPhysCameraDefocusBlur::ChFilterPhysCameraDefocusBlur(
    float focal_length, float focus_dist, float aperture_num, float pixel_size, float defocus_gain, float defocus_bias,
    std::string name
) :
    m_focal_length(focal_length), m_focus_dist(focus_dist), m_aperture_num(aperture_num), m_pixel_size(pixel_size),
    m_defocus_gain(defocus_gain), m_defocus_bias(defocus_bias), ChFilter(name)
{}

CH_SENSOR_API void ChFilterPhysCameraDefocusBlur::Initialize(
    std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut
) {
    if (!bufferInOut) {
        InvalidFilterGraphNullBuffer(pSensor);
    }
    
    if (auto pRGBD = std::dynamic_pointer_cast<SensorDeviceRGBDHalf4Buffer>(bufferInOut)) {
        m_buffer_in = pRGBD;
    }
    else {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    }

    if (auto pOpx = std::dynamic_pointer_cast<ChOptixSensor>(pSensor)) {
        m_cuda_stream = pOpx->GetCudaStream();
    }

    // make new buffer for output
    m_buffer_out = chrono_types::make_shared<SensorDeviceHalf4Buffer>();
    DeviceHalf4BufferPtr b(
        cudaMallocHelper<PixelHalf4>(bufferInOut->Width * bufferInOut->Height), cudaFreeHelper<PixelHalf4>
    );
    m_buffer_out->Buffer = std::move(b);
    m_buffer_out->Width = bufferInOut->Width;
    m_buffer_out->Height = bufferInOut->Height;
    m_buffer_out->LaunchedCount = bufferInOut->LaunchedCount;
    m_buffer_out->TimeStamp = bufferInOut->TimeStamp;
    bufferInOut = m_buffer_out;
}

CH_SENSOR_API void ChFilterPhysCameraDefocusBlur::Apply() {
    m_buffer_out->Width = m_buffer_in->Width;
    m_buffer_out->Height = m_buffer_in->Height;
    m_buffer_out->LaunchedCount = m_buffer_in->LaunchedCount;
    m_buffer_out->TimeStamp = m_buffer_in->TimeStamp;

    // perform defocus blur operation in phys_cam_ops.cu
    cuda_phys_cam_defocus_blur(
        m_buffer_in->Buffer.get(), m_buffer_out->Buffer.get(), m_buffer_out->Width, m_buffer_out->Height,
        m_focal_length, m_focus_dist, m_aperture_num, m_pixel_size, m_defocus_gain, m_defocus_bias, m_cuda_stream
    );
}

CH_SENSOR_API void ChFilterPhysCameraDefocusBlur::SetFilterCtrlParameters(
    float focal_length, float focus_dist, float aperture_num
) {
    m_focal_length = focal_length;
    m_focus_dist = focus_dist;
    m_aperture_num = aperture_num;
}

CH_SENSOR_API void ChFilterPhysCameraDefocusBlur::SetFilterModelParameters(
    float pixel_size, float defocus_gain, float defocus_bias
) {
    m_pixel_size = pixel_size;
    m_defocus_gain = defocus_gain;
    m_defocus_bias = defocus_bias;
}

}  // namespace sensor
}  // namespace chrono
