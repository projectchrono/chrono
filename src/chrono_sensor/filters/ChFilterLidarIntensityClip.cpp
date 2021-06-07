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

#include "chrono_sensor/filters/ChFilterLidarIntensityClip.h"
#include "chrono_sensor/ChSensor.h"
#include "chrono_sensor/cuda/lidar_clip.cuh"
#include "chrono_sensor/utils/CudaMallocHelper.h"

namespace chrono {
namespace sensor {

ChFilterLidarIntensityClip::ChFilterLidarIntensityClip(float intensity_thresh, float default_value, std::string name)
    : m_intensity_thresh(intensity_thresh), m_default_dist(default_value), ChFilter(name) {}

CH_SENSOR_API void ChFilterLidarIntensityClip::Apply(std::shared_ptr<ChSensor> pSensor,
                                                     std::shared_ptr<SensorBuffer>& bufferInOut) {
    // this filter CANNOT be the first filter in a sensor's filter list, so the bufferIn CANNOT null.
    assert(bufferInOut != nullptr);
    if (!bufferInOut)
        throw std::runtime_error("The lidar clip filter was not supplied an input buffer");

    // to grayscale (for now), the incoming buffer must be an optix buffer
    auto pOpx = std::dynamic_pointer_cast<SensorOptixBuffer>(bufferInOut);
    auto pDI = std::dynamic_pointer_cast<SensorDeviceDIBuffer>(bufferInOut);
    if (!pOpx && !pDI) {
        throw std::runtime_error(
            "The lidar clip filter requires that the incoming buffer must be an optix buffer or DI buffer");
    }

    unsigned int width;
    unsigned int height;
    void* ptr;

    if (pOpx) {  // optix buffer for Depth+Intensity

        RTsize rwidth;
        RTsize rheight;
        pOpx->Buffer->getSize(rwidth, rheight);
        width = (unsigned int)rwidth;
        height = (unsigned int)rheight;

        // we only know how to convert RGBA8 to grayscale (not any other input format (yet))
        if (pOpx->Buffer->getFormat() != RT_FORMAT_FLOAT2) {
            throw std::runtime_error("The only format that can be reduced by lidar is FLOAT2/DI (depth,intensity)");
        }

        // we need id of first device for this context (should only have 1 anyway)
        int device_id = pOpx->Buffer->getContext()->getEnabledDevices()[0];
        ptr = pOpx->Buffer->getDevicePointer(device_id);
    }

    else if (pDI) {  // sensor buffer for Depth+Intensity
        width = pDI->Width;
        height = pDI->Height;
        ptr = pDI->Buffer.get();
    }

    cuda_lidar_clip((float*)ptr, (int)width, (int)height, m_intensity_thresh, m_default_dist);
}

}  // namespace sensor
}  // namespace chrono
