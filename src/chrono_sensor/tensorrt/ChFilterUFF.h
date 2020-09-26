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
// Authors: Asher Elmquist
// =============================================================================
//
// =============================================================================

#ifndef CHFILTERUFF_H
#define CHFILTERUFF_H

// #include "chrono_sensor/ChSensorBuffer.h"
#include "chrono_sensor/filters/ChFilter.h"
#include "NvInfer.h"
#include "NvUffParser.h"
#include <NvUtils.h>
#include <cuda_runtime_api.h>
#include "chrono_sensor/tensorrt/ChTRTUtils.h"

#include <iostream>

namespace chrono {
namespace sensor {

/// @addtogroup sensor_tensorrt
/// @{

/// A filter that processes data through a pre-trained neural network, based on UFF format
class CH_SENSOR_API ChFilterUFF : public ChFilter {
  public:
    /// Class constructor
    ChFilterUFF(std::string name = {});

    /// Apply function runs data through neural network
    /// @param pSensor The sensor used for processing
    /// @bufferInOut A shared pointer for passing data between filters
    virtual void Apply(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);

    /// Initialize function for generating any information or structures needed once
    /// @param pSensor The sensor used for processing
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor);

  private:
    std::unique_ptr<nvinfer1::ICudaEngine, TRTDestroyer> m_inference_engine;  ///< object used for inference
    std::unique_ptr<nvinfer1::IExecutionContext, TRTDestroyer>
        m_inference_context;  ///< executing context for inference

    std::shared_ptr<float> m_input;        ///< input buffers for processing
    std::shared_ptr<float> m_output;       ///< output buffers for processing
    std::vector<void*> m_process_buffers;  ///< vector for pointers which inlcude input and output buffers
};
/// @} sensor_tensorrt

}  // namespace sensor
}  // namespace chrono

#endif
