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

#include "chrono_sensor/tensorrt/ChFilterUFF.h"
#include "chrono_sensor/ChSensor.h"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#include "chrono_sensor/cuda/nn_prep.cuh"

namespace chrono {
namespace sensor {

using namespace nvuffparser;
using namespace nvinfer1;

ChFilterUFF::ChFilterUFF(std::string name) : ChFilter(name) {}
void ChFilterUFF::Apply(std::shared_ptr<ChSensor> pSensor, std::shared_ptr<SensorBuffer>& bufferInOut) {
    // make sure buffer is the correct type
    std::shared_ptr<SensorOptixBuffer> pOpx = std::dynamic_pointer_cast<SensorOptixBuffer>(bufferInOut);
    std::shared_ptr<SensorDeviceRGBA8Buffer> pBuf = std::dynamic_pointer_cast<SensorDeviceRGBA8Buffer>(bufferInOut);
    if (!pOpx && !pBuf) {
        throw std::runtime_error("UFF parser filter only accepts RGBA8 buffer as input");
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
    preprocess_RGBA8_to_FLOAT4(device_ptr, m_process_buffers[0], width * height * 4);
    m_inference_context->execute(1, &m_process_buffers[0]);
    postprocess_FLOAT4_to_RGBA8(m_process_buffers[1], device_ptr, width * height * 4);
    // set the output buffer
    // bufferInOut = m_buffer;
}
void ChFilterUFF::Initialize(std::shared_ptr<ChSensor> pSensor) {
    std::string network_model = "../data/sensor/neural_nets/identity_2layer_1920x1080";

    Logger l(NONE);
    // create inference builder
    auto builder = std::unique_ptr<IBuilder, TRTDestroyer>(createInferBuilder(l));
    if (!builder) {
        throw std::runtime_error("Could not create inference builder");
    }
    // builder->setFp16Mode(true);

    // create network definition
    auto network = std::unique_ptr<INetworkDefinition, TRTDestroyer>(builder->createNetwork());
    if (!network) {
        throw std::runtime_error("Could not create network definition");
    }

    // create builder configuration
    auto config = std::unique_ptr<IBuilderConfig, TRTDestroyer>(builder->createBuilderConfig());
    if (!config) {
        throw std::runtime_error("Could not create builder configuration");
    }

    // create UFF parser
    auto parser = std::unique_ptr<IUffParser, TRTDestroyer>(createUffParser());
    if (!parser) {
        throw std::runtime_error("Could not create UFF parser");
    }

    // register the input and output layers
    parser->registerInput("input", nvinfer1::Dims3(1080, 1920, 4),
                          UffInputOrder::kNHWC);   // TODO: dynamic input name
    parser->registerOutput("output/convolution");  // TODO: dynamic output name

    // parse the given network
    // parser->parse(network_model.c_str(), *network, nvinfer1::DataType::kINT8);
    parser->parse(network_model.c_str(), *network, nvinfer1::DataType::kHALF);
    // parser->parse(network_model.c_str(), *network, nvinfer1::DataType::kFLOAT);

    // create inference engine
    m_inference_engine = std::unique_ptr<ICudaEngine, TRTDestroyer>(builder->buildCudaEngine(*network));

    // auto context_deleter = [&](int* IExecutionContext) { m_inference_context->destroy(); };
    m_inference_context =
        std::unique_ptr<IExecutionContext, TRTDestroyer>(m_inference_engine->createExecutionContext());

    // initialize the buffer vector -> should be two vectors (input, output)
    m_process_buffers = std::vector<void*>(2);
}

}  // namespace sensor
}  // namespace chrono
