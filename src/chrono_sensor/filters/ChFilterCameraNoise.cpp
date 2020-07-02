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

#include "chrono_sensor/filters/ChFilterCameraNoise.h"
#include "chrono_sensor/ChSensor.h"
#include "chrono_sensor/cuda/camera_noise.cuh"

namespace chrono {
namespace sensor {

ChFilterCameraNoiseConstNormal::ChFilterCameraNoiseConstNormal(float mean, float stdev, std::string name)
    : m_mean(mean), m_stdev(stdev), ChFilter(name) {}

CH_SENSOR_API void ChFilterCameraNoiseConstNormal::Apply(std::shared_ptr<ChSensor> pSensor,
                                                         std::shared_ptr<SensorBuffer>& bufferInOut) {
    // this filter CANNOT be the first filter in a sensor's filter list, so the bufferIn CANNOT null.
    assert(bufferInOut != nullptr);
    if (!bufferInOut)
        throw std::runtime_error("The filter was not supplied an input buffer");

    // to grayscale (for now), the incoming buffer must be an optix buffer
    // std::shared_ptr<SensorOptixBuffer> pSen = std::dynamic_pointer_cast<SensorOptixBuffer>(bufferInOut);

    std::shared_ptr<SensorOptixBuffer> pOpx = std::dynamic_pointer_cast<SensorOptixBuffer>(bufferInOut);
    std::shared_ptr<SensorDeviceRGBA8Buffer> pRGBA = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut);
    // std::shared_ptr<SensorDeviceR8Buffer> pR = std::dynamic_pointer_cast<SensorDeviceR8Buffer>(bufferInOut);

    if (!pOpx && !pRGBA) {
        throw std::runtime_error("The camera noise filter requires that the incoming buffer be optix or RGBA8");
    }

    if (pOpx) {
        RTsize rwidth;
        RTsize rheight;
        pOpx->Buffer->getSize(rwidth, rheight);
        unsigned int width = (unsigned int)rwidth;
        unsigned int height = (unsigned int)rheight;

        // we only know how to convert RGBA8 to grayscale (not any other input format (yet))
        if (pOpx->Buffer->getFormat() != RT_FORMAT_UNSIGNED_BYTE4) {
            throw std::runtime_error("The only format that have noise added is BYTE4");
        }

        // we need id of first device for this context (should only have 1 anyway)
        int device_id = pOpx->Buffer->getContext()->getEnabledDevices()[0];
        void* ptr = pOpx->Buffer->getDevicePointer(device_id);  // hard coded to grab from device 0
        cuda_camera_noise_const_normal(ptr, (int)width, (int)height, m_mean, m_stdev);
    } else if (pRGBA) {
        unsigned int width = pRGBA->Width;
        unsigned int height = pRGBA->Height;

        void* ptr = pRGBA->Buffer.get();
        cuda_camera_noise_const_normal(ptr, (int)width, (int)height, m_mean, m_stdev);
    }
}

//============ TODO: finish implementing the noise addition kernel
ChFilterCameraNoisePixDep::ChFilterCameraNoisePixDep(float gain, float sigma_read, float sigma_adc, std::string name)
    : m_gain(gain), m_sigma_read(sigma_read), m_sigma_adc(sigma_adc), ChFilter(name) {}

CH_SENSOR_API void ChFilterCameraNoisePixDep::Apply(std::shared_ptr<ChSensor> pSensor,
                                                    std::shared_ptr<SensorBuffer>& bufferInOut) {
    // this filter CANNOT be the first filter in a sensor's filter list, so the bufferIn CANNOT null.
    assert(bufferInOut != nullptr);
    if (!bufferInOut)
        throw std::runtime_error("The filter was not supplied an input buffer");

    std::shared_ptr<SensorOptixBuffer> pOpx = std::dynamic_pointer_cast<SensorOptixBuffer>(bufferInOut);
    std::shared_ptr<SensorDeviceRGBA8Buffer> pRGBA = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut);
    // std::shared_ptr<SensorDeviceR8Buffer> pR = std::dynamic_pointer_cast<SensorDeviceR8Buffer>(bufferInOut);

    if (!pOpx && !pRGBA) {
        throw std::runtime_error("The camera noise filter requires that the incoming buffer be optix or RGBA8");
    }

    if (pOpx) {
        RTsize rwidth;
        RTsize rheight;
        pOpx->Buffer->getSize(rwidth, rheight);
        unsigned int width = (unsigned int)rwidth;
        unsigned int height = (unsigned int)rheight;

        // we only know how to convert RGBA8 to grayscale (not any other input format (yet))
        if (pOpx->Buffer->getFormat() != RT_FORMAT_UNSIGNED_BYTE4) {
            throw std::runtime_error("The only format that have noise added is BYTE4");
        }

        // we need id of first device for this context (should only have 1 anyway)
        int device_id = pOpx->Buffer->getContext()->getEnabledDevices()[0];
        void* ptr = pOpx->Buffer->getDevicePointer(device_id);  // hard coded to grab from device 0
        cuda_camera_noise_pixel_dependent(ptr, (int)width, (int)height, m_gain, m_sigma_read, m_sigma_adc);
    } else if (pRGBA) {
        unsigned int width = pRGBA->Width;
        unsigned int height = pRGBA->Height;

        void* ptr = pRGBA->Buffer.get();
        cuda_camera_noise_pixel_dependent(ptr, (int)width, (int)height, m_gain, m_sigma_read, m_sigma_adc);
    }
}

}  // namespace sensor
}  // namespace chrono
