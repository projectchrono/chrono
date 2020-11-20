

#include "InferenceDriver.h"

#include "chrono_sensor/ChSensor.h"
#include "chrono_sensor/utils/CudaMallocHelper.h"
#include "chrono_sensor/cuda/nn_prep.cuh"

#include <algorithm>

using namespace nvonnxparser;
using namespace nvinfer1;

InferenceDriver::InferenceDriver(ChVehicle& vehicle,
                                 std::string file_name,
                                 UserRGBA8BufferPtr input_img,
                                 std::vector<float>* input_2,
                                 LoggerVerbosity verbose_level)
    : m_file_name(file_name), ChDriver(vehicle), m_verbose_level(verbose_level) {
    m_input_2 = input_2;
    m_input_img = input_img;
    m_output_targets = std::vector<float>(2);  // 2 outputs steering, throttle
    m_throttle = 0;
    m_steering = 0;
    m_braking = 0;
    m_output_targets[0] = m_throttle;
    m_output_targets[1] = m_steering;
    m_dT = 1;
    m_dS = 1;
    m_dB = 1;
}

void InferenceDriver::Initialize() {
    Logger l(m_verbose_level);  // NONE, PARTIAL, ALL

    // Create inference builder
    auto builder = std::unique_ptr<IBuilder, TRTDestroyer>(createInferBuilder(l));
    if (!builder) {
        throw std::runtime_error("Could not create inference builder");
    }
    builder->setMaxBatchSize(1);

    // Create network definition
    const auto explicitBatch = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network = std::unique_ptr<INetworkDefinition, TRTDestroyer>(builder->createNetworkV2(explicitBatch));

    if (!network) {
        throw std::runtime_error("Could not create network definition");
    }

    // Create builder configuration
    auto config = std::unique_ptr<IBuilderConfig, TRTDestroyer>(builder->createBuilderConfig());
    if (!config) {
        throw std::runtime_error("Could not create builder configuration");
    }

    // Create UFF parser
    auto parser = std::unique_ptr<IParser, TRTDestroyer>(createParser(*network, l));
    if (!parser) {
        throw std::runtime_error("Could not create ONNX parser");
    }

    // Parse the given network from the onnx model file
    parser->parseFromFile(m_file_name.c_str(), NONE);

    std::cout << "ONNX File: " << m_file_name.c_str() << std::endl;

    // Create inference engine
    m_inference_engine = std::unique_ptr<ICudaEngine, TRTDestroyer>(builder->buildCudaEngine(*network));
    if (!m_inference_engine) {
        throw std::runtime_error("Could not create ONNX inference engine");
    }

    // Create inference context
    m_inference_context =
        std::unique_ptr<IExecutionContext, TRTDestroyer>(m_inference_engine->createExecutionContext());
    if (!m_inference_context) {
        throw std::runtime_error("Could not create ONNX inference context");
    }

    // -----------------------------------
    // Specific to this "multisensor" demo
    // -----------------------------------
    // Allocate space on the GPU that will be used during inference
    img_gpu = std::shared_ptr<uint8_t>(
        cudaMallocHelper<uint8_t>(m_input_img->Height * m_input_img->Width * sizeof(PixelRGBA8)),
        cudaFreeHelper<uint8_t>);

    m_process_buffers = std::vector<void*>(3);
    in1 = std::shared_ptr<float>(cudaMallocHelper<float>(m_input_img->Height * m_input_img->Width * 3 * sizeof(float)),
                                 cudaFreeHelper<float>);
    m_process_buffers[0] = in1.get();

    in2 = std::shared_ptr<float>(cudaMallocHelper<float>(m_input_2->size() * sizeof(float)), cudaFreeHelper<float>);
    m_process_buffers[1] = in2.get();

    out = std::shared_ptr<float>(cudaMallocHelper<float>(m_input_2->size() * sizeof(float)), cudaFreeHelper<float>);
    m_process_buffers[2] = out.get();
}

void InferenceDriver::Synchronize(double time) {
    if (m_input_img->Buffer) {
        // 1. copy data into structures on img_gpu
        cudaMemcpy(img_gpu.get(), m_input_img->Buffer.get(),
                   m_input_img->Width * m_input_img->Height * sizeof(PixelRGBA8), cudaMemcpyHostToDevice);
        cudaMemcpy(m_process_buffers[1], m_input_2->data(), m_input_2->size() * sizeof(float), cudaMemcpyHostToDevice);

        // 2. preprocess data on gpu
        // slice out first three channels. Channel first is handled in network, slicing is not!
        preprocess_RGBA8_to_FLOAT3(img_gpu.get(), m_process_buffers[0], m_input_img->Height, m_input_img->Width);

        // 3. perform inference
        m_inference_context->executeV2(m_process_buffers.data());

        // 4. move prediction back to host
        cudaMemcpy(m_output_targets.data(), m_process_buffers[2], m_input_2->size() * sizeof(float),
                   cudaMemcpyDeviceToHost);

        // 5. set steering, throttle, braking apropriately
        m_target_throttle = (m_output_targets[0] + 1) / 2.0;
        m_target_steering = m_output_targets[1];

        // std::cout << "Steering: " << m_steering << ", throttle: " << m_throttle << std::endl;
    }
}

void InferenceDriver::Advance(double step) {
    // Step wise update that ensures driver inputs aren't reached too fast
    m_throttle = std::max(m_throttle - m_dT, std::min(m_target_throttle, m_throttle + m_dT));
    m_steering = std::max(m_steering - m_dS, std::min(m_target_steering, m_steering + m_dS));
    m_braking = std::max(m_braking - m_dB, std::min(m_target_braking, m_braking + m_dB));
}
