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

#ifndef CHFILTEROPTIXRENDER_H
#define CHFILTEROPTIXRENDER_H

#include <memory>
#include "chrono_sensor/filters/ChFilter.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/optix/ChOptixUtils.h"
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/optix/ChOptixPipeline.h"
#include "chrono_sensor/optix/ChOptixDefinitions.h"

namespace chrono {
namespace sensor {

// forward declaration
class ChSensor;

/// @addtogroup sensor_filters
/// @{

class CH_SENSOR_API ChOptixDenoiser {
  public:
    ChOptixDenoiser(OptixDeviceContext context);
    ~ChOptixDenoiser();
    void Initialize(unsigned int w,
                    unsigned int h,
                    CUstream stream,
                    half4* input_buffer,
                    half4* albedo_buffer,
                    half4* normal_buffer,
                    half4* output_buffer);
    void Execute();

  private:
    CUstream m_cuda_stream;
    OptixDenoiser m_denoiser = nullptr;
    OptixDenoiserParams m_params = {};
    CUdeviceptr md_intensity = 0;
    CUdeviceptr md_scratch = 0;
    uint32_t m_scratch_size = 0;
    CUdeviceptr md_state = 0;
    uint32_t m_state_size = 0;
    std::vector<OptixImage2D> md_inputs;
    OptixImage2D md_output;
};

/// A filter that generates data for a ChOptixSensor
class CH_SENSOR_API ChFilterOptixRender : public ChFilter {
  public:
    /// Class constructor
    ChFilterOptixRender();

    virtual ~ChFilterOptixRender();

    /// Apply function. Generates data for ChOptixSensors
    /// @param pSensor A pointer to the sensor on which the filter is attached.
    /// @param bufferInOut A buffer that is passed into the filter.
    virtual void Apply();

    /// Initializes all data needed by the filter access apply function.
    /// @param pSensor A pointer to the sensor.
    /// @param bufferInOut A pointer to the process buffer
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

  private:
    /// Finds visual filters
    std::shared_ptr<ChFilterVisualize> FindOnlyVisFilter(std::shared_ptr<ChSensor> pSensor);

    std::shared_ptr<SensorBuffer> m_bufferOut;
    std::weak_ptr<ChOptixSensor> m_optixSensor;  ///< for holding a weak reference to parent sensor
    CUstream m_cuda_stream;                      ///< reference to a cuda stream

    std::shared_ptr<ChOptixDenoiser> m_denoiser;  ///< denoiser in case there is global illumination
    std::shared_ptr<curandState_t> m_rng;         ///< rng buffer for camera jitter or ray bounces

    // Special handles that will accessed by ChOptixEngine
    OptixPipeline m_optix_pipeline;  ///< to hold reference to thte optix pipeline of this sensor
    ContextParameters* m_optix_params;
    std::shared_ptr<OptixShaderBindingTable> m_optix_sbt;
    std::shared_ptr<Record<RaygenParameters>> m_raygen_record;  ///< ray generation record
    float m_time_stamp;                                         ///< time stamp for when the data (render) was launched

    friend class ChOptixEngine;  ///< ChOptixEngine is allowed to set and use the private members
};

/// @}

}  // namespace sensor
}  // namespace chrono

#endif
