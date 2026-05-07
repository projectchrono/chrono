// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
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
// Filter to aggregate illumination irradiance over exposure time and pixel area
// with considering aperture number, i.e., integrate irradiance of each pixel to
// energy
// 
// =============================================================================

#include "chrono_sensor/filters/ChFilterPhysCameraAggregator.h"
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/cuda/phys_cam_ops.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#include <chrono>

namespace chrono {
namespace sensor {
ChFilterPhysCameraAggregator::ChFilterPhysCameraAggregator(
    float aperture_num, float expsr_time, float pixel_size, float max_scene_light_amount,
    ChVector3f rgb_QE_vec, float aggregator_gain, std::string name
    ):
    m_aperture_num(aperture_num), m_expsr_time(expsr_time), m_pixel_size(pixel_size),
    m_max_scene_light_amount(max_scene_light_amount), m_aggregator_gain(aggregator_gain), ChFilter(name)
{
    m_rgb_QEs[0] = rgb_QE_vec.x();
    m_rgb_QEs[1] = rgb_QE_vec.y();
    m_rgb_QEs[2] = rgb_QE_vec.z();
};

CH_SENSOR_API void ChFilterPhysCameraAggregator::Initialize(
    std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut
    ) {
    if (!bufferInOut) {
        InvalidFilterGraphNullBuffer(pSensor);
    }

    if (auto pRGBAHalf4 = std::dynamic_pointer_cast<SensorDeviceHalf4Buffer>(bufferInOut)) {
        m_in_out = pRGBAHalf4;
    }
    else {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    }

    if (auto pOpx = std::dynamic_pointer_cast<ChOptixSensor>(pSensor)) {
        m_cuda_stream = pOpx->GetCudaStream();
    }
}

CH_SENSOR_API void ChFilterPhysCameraAggregator::Apply() {    
    cuda_phys_cam_aggregator(
		m_in_out->Buffer.get(), m_in_out->Width, m_in_out->Height, m_aperture_num, m_expsr_time, m_pixel_size,
        m_max_scene_light_amount, m_rgb_QEs, m_aggregator_gain, m_cuda_stream
	);

}

CH_SENSOR_API void ChFilterPhysCameraAggregator::SetFilterCtrlParameters(float aperture_num, float expsr_time) {
    m_aperture_num = aperture_num;
    m_expsr_time = expsr_time;
}

CH_SENSOR_API void ChFilterPhysCameraAggregator::SetFilterModelParameters(
    float pixel_size, float max_scene_light_amount, ChVector3f rgb_QE_vec, float aggregator_gain
) {
	m_pixel_size = pixel_size;
    m_max_scene_light_amount = max_scene_light_amount;
    m_rgb_QEs[0] = rgb_QE_vec.x();
    m_rgb_QEs[1] = rgb_QE_vec.y();
    m_rgb_QEs[2] = rgb_QE_vec.z();
    m_aggregator_gain = aggregator_gain;
}


}  // namespace sensor
}  // namespace chrono
