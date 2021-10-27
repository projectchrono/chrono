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
// Authors: Radu Serban, Asher Elmquist
// =============================================================================
//
// Driver that loads a .onnx file and performs inference
// Example uses a camera image and 2 extra input data
//
// =============================================================================

#include <string>
#include <vector>
#include <memory>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChDriver.h"
#include "chrono_vehicle/ChVehicle.h"

#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <NvUtils.h>
#include <cuda_runtime_api.h>
#include "chrono_sensor/tensorrt/ChTRTUtils.h"

#include "chrono_sensor/filters/ChFilterAccess.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::sensor;

class InferenceDriver : public ChDriver {
  public:
    // Constructor
    InferenceDriver(ChVehicle& vehicle,                   // Reference to the underlying vehicle
                    std::string file_name,                // File name of the .onnx file to load
                    UserRGBA8BufferPtr input_img,         // Pointer to the input image data
                    std::vector<float>* input_2,          // Pointer to the extra data
                    LoggerVerbosity verbose_level = NONE  // Logger verbosity (NONE, PARTIAL, ALL)
    );

    // Destructor
    ~InferenceDriver() {}

    // Set the steering delta
    void SetDeltas(float dS, float dT, float dB) {
        m_dS = dS;
        m_dT = dT;
        m_dB = dB;
    }

    /// Initialize this driver system.
    virtual void Initialize();

    /// Update the state of this driver system at the current time.
    virtual void Synchronize(double time);

    /// Advance the state of this driver system by the specified time step.
    virtual void Advance(double step);

  private:
    // .onnx file name
    std::string m_file_name;

    // Verbosity
    LoggerVerbosity m_verbose_level;

    // TensorRT related objects
    std::unique_ptr<nvinfer1::ICudaEngine, TRTDestroyer> m_inference_engine;
    std::unique_ptr<nvinfer1::IExecutionContext, TRTDestroyer> m_inference_context;

    // Driving deltas
    float m_dS;
    float m_dT;
    float m_dB;

    std::shared_ptr<float> in1;
    std::shared_ptr<float> in2;
    std::shared_ptr<float> out;

    // Target driver inputs
    double m_target_steering = 0;
    double m_target_throttle = 0;
    double m_target_braking = 0;

    // Reference to input data
    UserRGBA8BufferPtr m_input_img;
    std::vector<float>* m_input_2;

    // References to data on the gpu
    bool m_inputs_set = false;
    std::shared_ptr<uint8_t> img_gpu;      // RGBA buffer on the gpu (must be preprocessed into the model)
    std::vector<void*> m_process_buffers;  // Holds the pointer to the inputs and outputs to the model on the gpu
    std::vector<float> m_output_targets;   // Output vector on the host
};
