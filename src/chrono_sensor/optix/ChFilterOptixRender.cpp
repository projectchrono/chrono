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
// Authors: Asher Elmquist, Eric Brandt
// =============================================================================
//
// =============================================================================

#include "chrono_sensor/optix/ChFilterOptixRender.h"
#include <assert.h>
#include <algorithm>
#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/ChLidarSensor.h"
#include "chrono_sensor/ChRadarSensor.h"
#include "chrono_sensor/ChSensor.h"
#include "chrono_sensor/ChSensorBuffer.h"
#include "chrono_sensor/optix/ChOptixUtils.h"
#include "chrono_sensor/cuda/curand_utils.cuh"
#include <optix_stubs.h>

#include "chrono_sensor/utils/CudaMallocHelper.h"

namespace chrono {
namespace sensor {

ChFilterOptixRender::ChFilterOptixRender() : ChFilter("OptixRenderer") {}
ChFilterOptixRender::~ChFilterOptixRender() {}

CH_SENSOR_API void ChFilterOptixRender::Apply() {
    // std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    auto pOptixSensor = m_optixSensor.lock();
    m_bufferOut->LaunchedCount = pOptixSensor->GetNumLaunches();
    m_bufferOut->TimeStamp = m_time_stamp;

    cudaMemcpyAsync(reinterpret_cast<void*>(m_optix_sbt->raygenRecord), m_raygen_record.get(),
                    sizeof(Record<RaygenParameters>), cudaMemcpyHostToDevice, m_cuda_stream);

    OPTIX_ERROR_CHECK(optixLaunch(m_optix_pipeline, m_cuda_stream, reinterpret_cast<CUdeviceptr>(m_optix_params),
                                  sizeof(ContextParameters), m_optix_sbt.get(),
                                  m_bufferOut->Width,   // launch width
                                  m_bufferOut->Height,  // launch height
                                  1                     // launch depth
                                  ));

    // run the denoiser if it has been created
    if (m_denoiser) {
        m_denoiser->Execute();
    }

    // below is only needed when timing the ray tracing operation
    // cudaStreamSynchronize(
    //     m_cuda_stream);  // TODO: let the stream by synchronized by the optix engine at the end of
    // //                                       // processing, or by filters that require synchronization
    // std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    // std::cout << "Filter's ray tracing time: " << wall_time.count() << std::endl;
}

CH_SENSOR_API void ChFilterOptixRender::Initialize(std::shared_ptr<ChSensor> pSensor,
                                                   std::shared_ptr<SensorBuffer>& bufferInOut) {
    if (bufferInOut) {
        throw std::runtime_error("The optix render filter must be the first filter in the list");
    }
    auto pOptixSensor = std::dynamic_pointer_cast<ChOptixSensor>(pSensor);
    if (!pOptixSensor) {
        InvalidFilterGraphSensorTypeMismatch(pSensor);
    }
    m_optixSensor = pOptixSensor;
    // get the sensor reference frames
    ChFrame<double> f_offset = pOptixSensor->GetOffsetPose();
    ChFrame<double> f_body = pOptixSensor->GetParent()->GetAssetsFrame();
    ChFrame<double> global_loc = f_body * f_offset;

    if (auto cam = std::dynamic_pointer_cast<ChCameraSensor>(pSensor)) {
        auto bufferOut = chrono_types::make_shared<SensorDeviceFloat4Buffer>();
        DeviceFloat4BufferPtr b(cudaMallocHelper<PixelFloat4>(pOptixSensor->GetWidth() * pOptixSensor->GetHeight()),
                                cudaFreeHelper<PixelFloat4>);
        bufferOut->Buffer = std::move(b);
        m_raygen_record->data.specific.camera.hFOV = cam->GetHFOV();
        m_raygen_record->data.specific.camera.frame_buffer = reinterpret_cast<float4*>(bufferOut->Buffer.get());
        m_raygen_record->data.specific.camera.use_gi = cam->GetUseGI();
        m_bufferOut = bufferOut;

        if (cam->GetUseGI() && m_denoiser) {
            float4* frame_buffer = cudaMallocHelper<float4>(pOptixSensor->GetWidth() * pOptixSensor->GetHeight());
            float4* albedo_buffer = cudaMallocHelper<float4>(pOptixSensor->GetWidth() * pOptixSensor->GetHeight());
            float4* normal_buffer = cudaMallocHelper<float4>(pOptixSensor->GetWidth() * pOptixSensor->GetHeight());

            m_raygen_record->data.specific.camera.frame_buffer = frame_buffer;
            m_raygen_record->data.specific.camera.albedo_buffer = albedo_buffer;
            m_raygen_record->data.specific.camera.normal_buffer = normal_buffer;

            m_denoiser->Initialize(cam->GetWidth(), cam->GetHeight(), cam->GetCudaStream(), frame_buffer, albedo_buffer,
                                   normal_buffer, reinterpret_cast<float4*>(bufferOut->Buffer.get()));

            // initialize rng buffer for ray bounces
            m_rng = std::shared_ptr<curandState_t>(
                cudaMallocHelper<curandState_t>(pOptixSensor->GetWidth() * pOptixSensor->GetHeight()),
                cudaFreeHelper<curandState_t>);
            init_cuda_rng((unsigned int)std::chrono::high_resolution_clock::now().time_since_epoch().count(),
                          m_rng.get(), pOptixSensor->GetWidth() * pOptixSensor->GetHeight());
            m_raygen_record->data.specific.camera.rng_buffer = m_rng.get();
        }

    } else if (auto lidar = std::dynamic_pointer_cast<ChLidarSensor>(pSensor)) {
        auto bufferOut = chrono_types::make_shared<SensorDeviceDIBuffer>();
        DeviceDIBufferPtr b(cudaMallocHelper<PixelDI>(pOptixSensor->GetWidth() * pOptixSensor->GetHeight()),
                            cudaFreeHelper<PixelDI>);
        bufferOut->Buffer = std::move(b);
        m_raygen_record->data.specific.lidar.max_vert_angle = lidar->GetMaxVertAngle();
        m_raygen_record->data.specific.lidar.min_vert_angle = lidar->GetMinVertAngle();
        m_raygen_record->data.specific.lidar.hFOV = lidar->GetHFOV();
        m_raygen_record->data.specific.lidar.max_distance = lidar->GetMaxDistance();
        m_raygen_record->data.specific.lidar.clip_near = lidar->GetClipNear();
        m_raygen_record->data.specific.lidar.frame_buffer = reinterpret_cast<float2*>(bufferOut->Buffer.get());
        m_raygen_record->data.specific.lidar.beam_shape = lidar->GetBeamShape();
        m_raygen_record->data.specific.lidar.sample_radius = lidar->GetSampleRadius();
        m_raygen_record->data.specific.lidar.horiz_div_angle = lidar->GetHorizDivAngle();
        m_raygen_record->data.specific.lidar.vert_div_angle = lidar->GetVertDivAngle();
        m_bufferOut = bufferOut;
    } else if (auto radar = std::dynamic_pointer_cast<ChRadarSensor>(pSensor)) {
        auto bufferOut = chrono_types::make_shared<SensorDeviceRangeRcsBuffer>();
        DeviceRangeRcsBufferPtr b(cudaMallocHelper<PixelRangeRcs>(pOptixSensor->GetWidth() * pOptixSensor->GetHeight()),
                                  cudaFreeHelper<PixelRangeRcs>);
        bufferOut->Buffer = std::move(b);
        m_raygen_record->data.specific.radar.max_vert_angle = radar->GetMaxVertAngle();
        m_raygen_record->data.specific.radar.min_vert_angle = radar->GetMinVertAngle();
        m_raygen_record->data.specific.radar.hFOV = radar->GetHFOV();
        m_raygen_record->data.specific.radar.max_distance = radar->GetMaxDistance();
        m_raygen_record->data.specific.radar.clip_near = radar->GetClipNear();
        m_raygen_record->data.specific.radar.frame_buffer = reinterpret_cast<float2*>(bufferOut->Buffer.get());
        m_bufferOut = bufferOut;
    } else {
        throw std::runtime_error("This type of sensor not supported yet by OptixRender filter");
    }
    m_bufferOut->Width = pOptixSensor->GetWidth();
    m_bufferOut->Height = pOptixSensor->GetHeight();
    m_bufferOut->LaunchedCount = pOptixSensor->GetNumLaunches();
    m_bufferOut->TimeStamp = m_time_stamp;

    m_cuda_stream = pOptixSensor->GetCudaStream();

    cudaMemcpyAsync(reinterpret_cast<void*>(m_optix_sbt->raygenRecord), m_raygen_record.get(),
                    sizeof(Record<RaygenParameters>), cudaMemcpyHostToDevice, m_cuda_stream);
    cudaStreamSynchronize(m_cuda_stream);
    // gives our output buffer to the next filter in the graph
    bufferInOut = m_bufferOut;
}

CH_SENSOR_API std::shared_ptr<ChFilterVisualize> ChFilterOptixRender::FindOnlyVisFilter(
    std::shared_ptr<ChSensor> pSensor) {
    int cnt = (int)std::count_if(
        pSensor->GetFilterList().begin(), pSensor->GetFilterList().end(),
        [](std::shared_ptr<ChFilter> f) { return std::dynamic_pointer_cast<ChFilterVisualize>(f) != nullptr; });

    if (cnt == 1) {
        auto it = std::find_if(
            pSensor->GetFilterList().begin(), pSensor->GetFilterList().end(),
            [](std::shared_ptr<ChFilter> f) { return std::dynamic_pointer_cast<ChFilterVisualize>(f) != nullptr; });
        return std::dynamic_pointer_cast<ChFilterVisualize>(*it);
    }
    return nullptr;
}

CH_SENSOR_API ChOptixDenoiser::ChOptixDenoiser(OptixDeviceContext context) : m_cuda_stream(0) {
    // initialize the optix denoiser
    OptixDenoiserOptions options = {};
    options.inputKind = OPTIX_DENOISER_INPUT_RGB_ALBEDO_NORMAL;
    OPTIX_ERROR_CHECK(optixDenoiserCreate(context, &options, &m_denoiser));
    // OPTIX_ERROR_CHECK(optixDenoiserSetModel(m_denoiser, OPTIX_DENOISER_MODEL_KIND_HDR, nullptr, 0));
    OPTIX_ERROR_CHECK(optixDenoiserSetModel(m_denoiser, OPTIX_DENOISER_MODEL_KIND_LDR, nullptr, 0));
    std::cout << "Constructed denoiser\n";
}

CH_SENSOR_API ChOptixDenoiser::~ChOptixDenoiser() {
    // denoiser is responsible for freeing input, normal, and albedo since the render filter will clear out the output
    // buffer
    cudaFree(reinterpret_cast<void*>(md_inputs[0].data));
    cudaFree(reinterpret_cast<void*>(md_inputs[1].data));
    cudaFree(reinterpret_cast<void*>(md_inputs[2].data));
    optixDenoiserDestroy(m_denoiser);
}

CH_SENSOR_API void ChOptixDenoiser::Initialize(unsigned int w,
                                               unsigned int h,
                                               CUstream stream,
                                               float4* input_buffer,
                                               float4* albedo_buffer,
                                               float4* normal_buffer,
                                               float4* output_buffer) {
    m_cuda_stream = stream;

    // intialize device memory for the denoiser
    OptixDenoiserSizes denoiser_sizes;
    OPTIX_ERROR_CHECK(optixDenoiserComputeMemoryResources(m_denoiser, w, h, &denoiser_sizes));
    m_scratch_size = static_cast<uint32_t>(denoiser_sizes.withoutOverlapScratchSizeInBytes);
    CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&md_intensity), sizeof(float)));
    CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&md_scratch), m_scratch_size));
    CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&md_state), denoiser_sizes.stateSizeInBytes));
    m_state_size = static_cast<uint32_t>(denoiser_sizes.stateSizeInBytes);

    // set the inputs and outputs
    md_inputs = std::vector<OptixImage2D>(3);
    md_inputs[0].data = reinterpret_cast<CUdeviceptr>(input_buffer);
    md_inputs[0].width = w;
    md_inputs[0].height = h;
    md_inputs[0].rowStrideInBytes = w * sizeof(float4);
    md_inputs[0].pixelStrideInBytes = sizeof(float4);
    md_inputs[0].format = OPTIX_PIXEL_FORMAT_FLOAT4;
    
    md_inputs[1].data = reinterpret_cast<CUdeviceptr>(albedo_buffer);
    md_inputs[1].width = w;
    md_inputs[1].height = h;
    md_inputs[1].rowStrideInBytes = w * sizeof(float4);
    md_inputs[1].pixelStrideInBytes = sizeof(float4);
    md_inputs[1].format = OPTIX_PIXEL_FORMAT_FLOAT4;

    md_inputs[2].data = reinterpret_cast<CUdeviceptr>(normal_buffer);
    md_inputs[2].width = w;
    md_inputs[2].height = h;
    md_inputs[2].rowStrideInBytes = w * sizeof(float4);
    md_inputs[2].pixelStrideInBytes = sizeof(float4);
    md_inputs[2].format = OPTIX_PIXEL_FORMAT_FLOAT4;

    md_output = {};
    md_output.data = reinterpret_cast<CUdeviceptr>(output_buffer);
    md_output.width = w;
    md_output.height = h;
    md_output.rowStrideInBytes = w * sizeof(float4);
    md_output.pixelStrideInBytes = sizeof(float4);
    md_output.format = OPTIX_PIXEL_FORMAT_FLOAT4;

    OPTIX_ERROR_CHECK(
        optixDenoiserSetup(m_denoiser, m_cuda_stream, w, h, md_state, m_state_size, md_scratch, m_scratch_size));
    m_params.denoiseAlpha = 0;
    m_params.hdrIntensity = md_intensity;
    m_params.blendFactor = 0.0f;
}

CH_SENSOR_API void ChOptixDenoiser::Execute() {
    OPTIX_ERROR_CHECK(optixDenoiserComputeIntensity(m_denoiser, m_cuda_stream, md_inputs.data(), md_intensity,
                                                    md_scratch, m_scratch_size));

    OPTIX_ERROR_CHECK(optixDenoiserInvoke(m_denoiser, m_cuda_stream, &m_params, md_state, m_state_size,
                                          md_inputs.data(), 3, 0, 0, &md_output, md_scratch, m_scratch_size));
}

}  // namespace sensor
}  // namespace chrono
