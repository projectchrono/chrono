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
#include "chrono_sensor/ChSensor.h"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#include "chrono_sensor/cuda/nn_prep.cuh"

namespace chrono {
namespace sensor {

using namespace nvonnxparser;
using namespace nvinfer1;

ChFilterONNX::ChFilterONNX(std::string name) : ChFilter(name) {}
void ChFilterONNX::Apply(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) {
    // make sure buffer is the correct type
    std::shared_ptr<SensorOptixBuffer> pOpx = std::dynamic_pointer_cast<SensorOptixBuffer>(bufferInOut);
    std::shared_ptr<SensorDeviceRGBA8Buffer> pBuf = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut);
    if (!pOpx && !pBuf) {
        throw std::runtime_error("ONNX parser filter only accepts RGBA8 buffer as input");
    }

    unsigned int width;
    unsigned int height;

    void* device_ptr;

    if (pOpx) {
        RTsize rwidth;
        RTsize rheight;
        pOpx->Buffer->getSize(rwidth, rheight);
        width = (unsigned int)rwidth;
        height = (unsigned int)rheight;

        if (pOpx->Buffer->getFormat() != RT_FORMAT_UNSIGNED_BYTE4) {
            throw std::runtime_error(
                "The only optix format that can be resized is  by lidar is RT_FORMAT_UNSIGNED_BYTE4");
        }

        // we need id of first device for this context (should only have 1 anyway)
        int device_id = pOpx->Buffer->getContext()->getEnabledDevices()[0];
        device_ptr = pOpx->Buffer->getDevicePointer(device_id);  // hard coded to grab from device 0

    } else if (pBuf) {
        width = pBuf->Width;
        height = pBuf->Height;
        device_ptr = pBuf.get();
    }

    // if (!m_buffer) {
    //     m_buffer = chrono_types::make_shared<SensorDeviceRGBA8Buffer>();
    //     DeviceRGBA8BufferPtr b(cudaMallocHelper<PixelRGBA8>(width * height), cudaFreeHelper<PixelRGBA8>);
    //     m_buffer->Buffer = std::move(b);
    //     m_buffer->Width = width;
    //     m_buffer->Height = height;
    // }
    if (!m_input) {
        m_input =
            std::shared_ptr<float>(cudaMallocHelper<float>(width * height * 4 * sizeof(float)), cudaFreeHelper<float>);
        m_process_buffers[0] = m_input.get();
    }
    if (!m_output) {
        m_output =
            std::shared_ptr<float>(cudaMallocHelper<float>(width * height * 4 * sizeof(float)), cudaFreeHelper<float>);
        m_process_buffers[1] = m_output.get();
    }

    // m_buffer->LaunchedCount = pSensor->GetNumLaunches();

    // run inference pass
    preprocess_RGBA8_to_FLOAT4_CHW(device_ptr, m_process_buffers[0], 4, height, width);
    m_inference_context->executeV2(&m_process_buffers[0]);
    postprocess_FLOAT4_to_RGBA8_CHW(m_process_buffers[1], device_ptr, 4, height, width);
    // set the output buffer
    // bufferInOut = m_buffer;
}
void ChFilterONNX::Initialize(std::shared_ptr<ChSensor> pSensor) {
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
    // std::cout << "Network parsed\n";
    // std::cout << "Network input dims: " << network->getInput(0)->getDimensions().d[0] << ", "
    //           << network->getInput(0)->getDimensions().d[1] << ", " << network->getInput(0)->getDimensions().d[2]
    //           << ", " << network->getInput(0)->getDimensions().d[3] << std::endl;
    // std::cout << "Network output dims: " << network->getOutput(0)->getDimensions().d[0] << ", "
    //           << network->getOutput(0)->getDimensions().d[1] << ", " << network->getOutput(0)->getDimensions().d[2]
    //           << ", " << network->getOutput(0)->getDimensions().d[3] << std::endl;

    // initialize the buffer vector -> should be two vectors (input, output)
    m_process_buffers = std::vector<void*>(2);
}

}  // namespace sensor
}  // namespace chrono
