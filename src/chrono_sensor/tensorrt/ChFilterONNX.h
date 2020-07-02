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

#ifndef CHFILTERONNX_H
#define CHFILTERONNX_H

// #include "chrono_sensor/ChSensorBuffer.h"
#include "chrono_sensor/filters/ChFilter.h"
#include "NvInfer.h"
#include "NvOnnxParser.h"
#include <NvUtils.h>
#include <cuda_runtime_api.h>
#include "chrono_sensor/tensorrt/ChTRTUtils.h"

#include <iostream>

namespace chrono {
namespace sensor {

// a filter that adds Gaussian noise across an image with constant mean and standard deviation
class CH_SENSOR_API ChFilterONNX : public ChFilter {
  public:
    ChFilterONNX(std::string name = {});

    virtual void Apply(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut);
    virtual void Initialize(std::shared_ptr<ChSensor> pSensor);

  private:
    std::unique_ptr<nvinfer1::ICudaEngine, TRTDestroyer> m_inference_engine;
    std::unique_ptr<nvinfer1::IExecutionContext, TRTDestroyer> m_inference_context;

    std::shared_ptr<float> m_input;
    std::shared_ptr<float> m_output;
    std::vector<void*> m_process_buffers;
    // std::shared_ptr<SensorDeviceRGBA8Buffer> m_buffer;
};

}  // namespace sensor
}  // namespace chrono

#endif
