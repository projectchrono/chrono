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

#include "chrono_sensor/tensorrt/ChFilterONNX.h"
#include "chrono_sensor/sensors/ChSensor.h"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#include "chrono_sensor/cuda/nn_prep.cuh"

namespace chrono {
namespace sensor {

using namespace nvonnxparser;
using namespace nvinfer1;

ChFilterONNX::ChFilterONNX(std::string name) : ChFilter(name) {}
void ChFilterONNX::Apply() {
    // run inference pass
    preprocess_RGBA8_to_FLOAT4_CHW(m_buffer_in->Buffer.get(), m_process_buffers[0], 4, m_buffer_in->Height,
                                   m_buffer_in->Width);
    m_inference_context->executeV2(&m_process_buffers[0]);
    postprocess_FLOAT4_to_RGBA8_CHW(m_process_buffers[1], m_buffer_in->Buffer.get(), 4, m_buffer_in->Height,
                                    m_buffer_in->Width);
}
void ChFilterONNX::Initialize(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (!bufferInOut)
        InvalidFilterGraphNullBuffer(pSensor);

    m_buffer_in = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut);
    if (!m_buffer_in) {
        InvalidFilterGraphBufferTypeMismatch(pSensor);
    }

    // std::string network_model = "../data/sensor/neural_nets/identity_3layer_1920x1080.onnx";
    std::string network_model = "../data/sensor/neural_nets/identity_720p_2L.onnx";

    Logger l(NONE);  // NONE, PARTIAL,ALL
    // create inference builder
    auto builder = std::unique_ptr<IBuilder, TRTDestroyer>(createInferBuilder(l));
    if (!builder) {
        throw std::runtime_error("Could not create inference builder");
    }
    builder->setMaxBatchSize(1);
    // builder->setFp16Mode(true);

    // create network definition
    const auto explicitBatch = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network = std::unique_ptr<INetworkDefinition, TRTDestroyer>(builder->createNetworkV2(explicitBatch));

    if (!network) {
        throw std::runtime_error("Could not create network definition");
    }

    // create builder configuration
    auto config = std::unique_ptr<IBuilderConfig, TRTDestroyer>(builder->createBuilderConfig());
    if (!config) {
        throw std::runtime_error("Could not create builder configuration");
    }

    // create UFF parser
    auto parser = std::unique_ptr<IParser, TRTDestroyer>(createParser(*network, l));
    if (!parser) {
        throw std::runtime_error("Could not create ONNX parser");
    }

    // parse the given network from the onnx model file
    parser->parseFromFile(network_model.c_str(), NONE);

    // create inference engine
    m_inference_engine = std::unique_ptr<ICudaEngine, TRTDestroyer>(builder->buildCudaEngine(*network));
    if (!m_inference_engine) {
        throw std::runtime_error("Could not create ONNX inference engine");
    }

    m_inference_context =
        std::unique_ptr<IExecutionContext, TRTDestroyer>(m_inference_engine->createExecutionContext());
    if (!m_inference_context) {
        throw std::runtime_error("Could not create ONNX inference context");
    }

    // initialize the buffer vector -> should be two vectors (input, output)
    m_process_buffers = std::vector<void*>(2);

    m_input = std::shared_ptr<float>(
        cudaMallocHelper<float>(m_buffer_in->Width * m_buffer_in->Height * 4 * sizeof(float)), cudaFreeHelper<float>);
    m_process_buffers[0] = m_input.get();
    m_output = std::shared_ptr<float>(
        cudaMallocHelper<float>(m_buffer_in->Width * m_buffer_in->Height * 4 * sizeof(float)), cudaFreeHelper<float>);
    m_process_buffers[1] = m_output.get();
}
}  // namespace sensor
}  // namespace chrono
