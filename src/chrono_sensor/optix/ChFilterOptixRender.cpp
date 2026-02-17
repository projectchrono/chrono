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
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChPhysCameraSensor.h"
#include "chrono_sensor/sensors/ChSegmentationCamera.h"
#include "chrono_sensor/sensors/ChDepthCamera.h"
#include "chrono_sensor/sensors/ChNormalCamera.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChRadarSensor.h"
#include "chrono_sensor/sensors/ChSensor.h"
#include "chrono_sensor/sensors/ChSensorBuffer.h"
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
    //                      // processing, or by filters that require synchronization
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
    ChFrame<double> f_body = pOptixSensor->GetParent()->GetVisualModelFrame();
    ChFrame<double> global_loc = f_body * f_offset;
    /// Camera sensor
    if (auto cam = std::dynamic_pointer_cast<ChCameraSensor>(pSensor)) {
        auto bufferOut = chrono_types::make_shared<SensorDeviceHalf4Buffer>();
        DeviceHalf4BufferPtr b(cudaMallocHelper<PixelHalf4>(pOptixSensor->GetWidth() * pOptixSensor->GetHeight()),
                               cudaFreeHelper<PixelHalf4>);
        bufferOut->Buffer = std::move(b);
        m_raygen_record->data.specific.camera.hFOV = cam->GetHFOV();
        m_raygen_record->data.specific.camera.gamma = cam->GetGamma();
        m_raygen_record->data.specific.camera.super_sample_factor = cam->GetSampleFactor();
        m_raygen_record->data.specific.camera.frame_buffer = reinterpret_cast<half4*>(bufferOut->Buffer.get());
        m_raygen_record->data.specific.camera.use_gi = cam->GetUseGI();
        m_raygen_record->data.specific.camera.use_denoiser = cam->GetUseDenoiser();
        m_raygen_record->data.specific.camera.use_fog = cam->GetUseFog();
        m_raygen_record->data.specific.camera.lens_model = cam->GetLensModelType();
        m_raygen_record->data.specific.camera.lens_parameters = cam->GetLensParameters();
        m_raygen_record->data.specific.camera.integrator = cam->GetIntegrator();
        // make_float3(cam->GetLensParameters().x(), cam->GetLensParameters().y()], cam->GetLensParameters().z());
        m_bufferOut = bufferOut;

        // Initialize rng_buffer for ray generations, diffuse ray bounces, or motion blur
        m_rng = std::shared_ptr<curandState_t>(
            cudaMallocHelper<curandState_t>(pOptixSensor->GetWidth() * pOptixSensor->GetHeight()),
            cudaFreeHelper<curandState_t>);

        init_cuda_rng((unsigned int)std::chrono::high_resolution_clock::now().time_since_epoch().count(),
                      m_rng.get(), pOptixSensor->GetWidth() * pOptixSensor->GetHeight());
        m_raygen_record->data.specific.camera.rng_buffer = m_rng.get();

        if (cam->GetUseDenoiser() && m_denoiser) {
            half4* frame_buffer = cudaMallocHelper<half4>(pOptixSensor->GetWidth() * pOptixSensor->GetHeight());
            half4* albedo_buffer = cudaMallocHelper<half4>(pOptixSensor->GetWidth() * pOptixSensor->GetHeight());
            half4* normal_buffer = cudaMallocHelper<half4>(pOptixSensor->GetWidth() * pOptixSensor->GetHeight());
            m_raygen_record->data.specific.camera.frame_buffer = frame_buffer;  // reinterpret_cast<half4*>(bufferOut->Buffer.get());
            m_raygen_record->data.specific.camera.albedo_buffer = albedo_buffer;
            m_raygen_record->data.specific.camera.normal_buffer = normal_buffer;

            m_denoiser->Initialize(cam->GetWidth(), cam->GetHeight(), cam->GetCudaStream(), frame_buffer, albedo_buffer,
                                   normal_buffer, reinterpret_cast<half4*>(bufferOut->Buffer.get()));
        }
    }
    /// Physics-based camera
    else if (auto phys_cam = std::dynamic_pointer_cast<ChPhysCameraSensor>(pSensor)) {
        auto bufferOut = chrono_types::make_shared<SensorDeviceRGBDHalf4Buffer>();
        DeviceRGBDHalf4BufferPtr b(cudaMallocHelper<PixelRGBDHalf4>(pOptixSensor->GetWidth() * pOptixSensor->GetHeight()), 
                                   cudaFreeHelper<PixelRGBDHalf4>);
        
        bufferOut->Buffer = std::move(b);
        m_raygen_record->data.specific.phys_camera.hFOV = phys_cam->GetHFOV();
        m_raygen_record->data.specific.phys_camera.gamma = phys_cam->GetGamma();
        m_raygen_record->data.specific.phys_camera.super_sample_factor = phys_cam->GetSampleFactor();
        m_raygen_record->data.specific.phys_camera.rgbd_buffer = reinterpret_cast<half4*>(bufferOut->Buffer.get());
        m_raygen_record->data.specific.phys_camera.use_gi = phys_cam->GetUseGI();
        m_raygen_record->data.specific.phys_camera.use_denoiser = phys_cam->GetUseDenoiser();
        m_raygen_record->data.specific.phys_camera.use_fog = phys_cam->GetUseFog();
        m_raygen_record->data.specific.phys_camera.lens_model = phys_cam->GetLensModelType();
        m_raygen_record->data.specific.phys_camera.lens_parameters = phys_cam->GetLensParameters();
        m_raygen_record->data.specific.phys_camera.aperture_num = phys_cam->GetApertureNum();
        m_raygen_record->data.specific.phys_camera.expsr_time = phys_cam->GetExpsrTime();
        m_raygen_record->data.specific.phys_camera.ISO = phys_cam->GetISO();
        m_raygen_record->data.specific.phys_camera.focal_length = phys_cam->GetFocalLength();
        m_raygen_record->data.specific.phys_camera.focus_dist = phys_cam->GetFocusDistance();
        m_raygen_record->data.specific.phys_camera.max_scene_light_amount = phys_cam->GetMaxSceneLightAmount();
        m_raygen_record->data.specific.phys_camera.sensor_width = phys_cam->GetSensorWidth();
        m_raygen_record->data.specific.phys_camera.pixel_size = phys_cam->GetPixelSize();
        m_raygen_record->data.specific.phys_camera.gain_params = phys_cam->GetGainParams();
        m_raygen_record->data.specific.phys_camera.noise_params = phys_cam->GetNoiseParams();
        m_bufferOut = bufferOut;

        // Initialize rng_buffer for ray generations, diffuse ray bounces, or motion blur
        m_rng = std::shared_ptr<curandState_t>(
            cudaMallocHelper<curandState_t>(pOptixSensor->GetWidth() * pOptixSensor->GetHeight()),
            cudaFreeHelper<curandState_t>);

        init_cuda_rng((unsigned int)std::chrono::high_resolution_clock::now().time_since_epoch().count(),
                        m_rng.get(), pOptixSensor->GetWidth() * pOptixSensor->GetHeight());
        m_raygen_record->data.specific.phys_camera.rng_buffer = m_rng.get();

        if (phys_cam->GetUseDenoiser() && m_denoiser) {
            half4* rgbd_buffer = cudaMallocHelper<half4>(pOptixSensor->GetWidth() * pOptixSensor->GetHeight());
            half4* albedo_buffer = cudaMallocHelper<half4>(pOptixSensor->GetWidth() * pOptixSensor->GetHeight());
            half4* normal_buffer = cudaMallocHelper<half4>(pOptixSensor->GetWidth() * pOptixSensor->GetHeight());
            m_raygen_record->data.specific.phys_camera.rgbd_buffer = rgbd_buffer; // reinterpret_cast<half4*>(bufferOut->Buffer.get());
            m_raygen_record->data.specific.phys_camera.albedo_buffer = albedo_buffer;
            m_raygen_record->data.specific.phys_camera.normal_buffer = normal_buffer;

            m_denoiser->Initialize(phys_cam->GetWidth(), phys_cam->GetHeight(), phys_cam->GetCudaStream(), rgbd_buffer,
                                   albedo_buffer, normal_buffer, reinterpret_cast<half4*>(bufferOut->Buffer.get()));
        }

    } else if (auto segmenter = std::dynamic_pointer_cast<ChSegmentationCamera>(pSensor)) {
        auto bufferOut = chrono_types::make_shared<SensorDeviceSemanticBuffer>();
        DeviceSemanticBufferPtr b(cudaMallocHelper<PixelSemantic>(pOptixSensor->GetWidth() * pOptixSensor->GetHeight()),
                                  cudaFreeHelper<PixelSemantic>);
        bufferOut->Buffer = std::move(b);
        m_raygen_record->data.specific.segmentation.hFOV = segmenter->GetHFOV();
        m_raygen_record->data.specific.segmentation.frame_buffer = reinterpret_cast<ushort2*>(bufferOut->Buffer.get());
        m_raygen_record->data.specific.segmentation.lens_model = segmenter->GetLensModelType();
        m_raygen_record->data.specific.camera.lens_parameters = segmenter->GetLensParameters(); // Is this a bug?
        // make_float3(cam->GetLensParameters().x(), cam->GetLensParameters().y()], cam->GetLensParameters().z());

        if (segmenter->GetCollectionWindow() > 0.f) {
            // initialize rng buffer for ray bounces or motion blur
            m_rng = std::shared_ptr<curandState_t>(
                cudaMallocHelper<curandState_t>(pOptixSensor->GetWidth() * pOptixSensor->GetHeight()),
                cudaFreeHelper<curandState_t>);

            init_cuda_rng((unsigned int)std::chrono::high_resolution_clock::now().time_since_epoch().count(),
                          m_rng.get(), pOptixSensor->GetWidth() * pOptixSensor->GetHeight());
            m_raygen_record->data.specific.segmentation.rng_buffer = m_rng.get();
        }

        m_bufferOut = bufferOut;

    } else if (auto depthCamera = std::dynamic_pointer_cast<ChDepthCamera>(pSensor) ) {
        auto bufferOut = chrono_types::make_shared<SensorDeviceDepthBuffer>();
        DeviceDepthBufferPtr b(cudaMallocHelper<PixelDepth>(pOptixSensor->GetWidth() * pOptixSensor->GetHeight()),
                                  cudaFreeHelper<PixelDepth>);
        bufferOut->Buffer = std::move(b);
        m_raygen_record->data.specific.depthCamera.hFOV = depthCamera->GetHFOV();
        m_raygen_record->data.specific.depthCamera.frame_buffer = reinterpret_cast<float*>(bufferOut->Buffer.get());
        m_raygen_record->data.specific.depthCamera.lens_model = depthCamera->GetLensModelType();
        m_raygen_record->data.specific.depthCamera.lens_parameters = depthCamera->GetLensParameters();
        m_raygen_record->data.specific.depthCamera.max_depth = depthCamera->GetMaxDepth();
            // make_float3(cam->GetLensParameters().x(), cam->GetLensParameters().y()], cam->GetLensParameters().z());

        if (depthCamera->GetCollectionWindow() > 0.f) {
            // initialize rng buffer for ray bounces or motion blur
            m_rng = std::shared_ptr<curandState_t>(
                cudaMallocHelper<curandState_t>(pOptixSensor->GetWidth() * pOptixSensor->GetHeight()),
                cudaFreeHelper<curandState_t>);

            init_cuda_rng((unsigned int)std::chrono::high_resolution_clock::now().time_since_epoch().count(),
                          m_rng.get(), pOptixSensor->GetWidth() * pOptixSensor->GetHeight());
            m_raygen_record->data.specific.depthCamera.rng_buffer = m_rng.get();
        }

        m_bufferOut = bufferOut;
    
    }
    else if (auto normalCamera = std::dynamic_pointer_cast<ChNormalCamera>(pSensor) ) {
        auto bufferOut = chrono_types::make_shared<SensorDeviceNormalBuffer>();
        DeviceNormalBufferPtr b(cudaMallocHelper<PixelNormal>(pOptixSensor->GetWidth() * pOptixSensor->GetHeight()),
                                cudaFreeHelper<PixelNormal>);
        bufferOut->Buffer = std::move(b);
        m_raygen_record->data.specific.normalCamera.hFOV = normalCamera->GetHFOV();
        m_raygen_record->data.specific.normalCamera.frame_buffer = reinterpret_cast<float3*>(bufferOut->Buffer.get());
        m_raygen_record->data.specific.normalCamera.lens_model = normalCamera->GetLensModelType();
        m_raygen_record->data.specific.normalCamera.lens_parameters = normalCamera->GetLensParameters();
        // make_float3(cam->GetLensParameters().x(), cam->GetLensParameters().y()], cam->GetLensParameters().z());

        if (normalCamera->GetCollectionWindow() > 0.f) {
            // initialize rng buffer for ray bounces or motion blur
            m_rng = std::shared_ptr<curandState_t>(
                cudaMallocHelper<curandState_t>(pOptixSensor->GetWidth() * pOptixSensor->GetHeight()),
                cudaFreeHelper<curandState_t>);

            init_cuda_rng((unsigned int)std::chrono::high_resolution_clock::now().time_since_epoch().count(),
                          m_rng.get(), pOptixSensor->GetWidth() * pOptixSensor->GetHeight());
            m_raygen_record->data.specific.normalCamera.rng_buffer = m_rng.get();
        }

        m_bufferOut = bufferOut;
    
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
        auto bufferOut = chrono_types::make_shared<SensorDeviceRadarBuffer>();
        DeviceRadarBufferPtr b(cudaMallocHelper<RadarReturn>(pOptixSensor->GetWidth() * pOptixSensor->GetHeight()),
                               cudaFreeHelper<RadarReturn>);
        bufferOut->Buffer = std::move(b);
        m_raygen_record->data.specific.radar.vFOV = radar->GetVFOV();
        m_raygen_record->data.specific.radar.hFOV = radar->GetHFOV();
        m_raygen_record->data.specific.radar.max_distance = radar->GetMaxDistance();
        m_raygen_record->data.specific.radar.clip_near = radar->GetClipNear();
        m_raygen_record->data.specific.radar.frame_buffer = reinterpret_cast<float*>(bufferOut->Buffer.get());
        m_bufferOut = bufferOut;
    }
    
    //// ---- Register Your Customized Sensor Here ---- ////
    //// ----(initialize sensor PRD in OptiX according to the sensor declared in C++) ---- ////
    
    else {
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
    OptixDenoiserOptions denoiser_options = {};
    denoiser_options.guideAlbedo = 1;
    denoiser_options.guideNormal = 1;
    denoiser_options.denoiseAlpha = OPTIX_DENOISER_ALPHA_MODE_COPY; // default value, can be changed when invoking the denoiser

    OPTIX_ERROR_CHECK(optixDenoiserCreate(context, OPTIX_DENOISER_MODEL_KIND_LDR, &denoiser_options, &m_denoiser));
}

CH_SENSOR_API ChOptixDenoiser::~ChOptixDenoiser() {
    // denoiser is responsible for freeing input, normal, and albedo since the render filter will clear out the output
    // buffer
    cudaFree(reinterpret_cast<void*>(md_inputs[0].data));
    cudaFree(reinterpret_cast<void*>(md_inputs[1].data));
    cudaFree(reinterpret_cast<void*>(md_inputs[2].data));
    cudaFree(reinterpret_cast<void*>(md_scratch));
    cudaFree(reinterpret_cast<void*>(md_state));
    optixDenoiserDestroy(m_denoiser);
}

CH_SENSOR_API void ChOptixDenoiser::Initialize(unsigned int w,
                                               unsigned int h,
                                               CUstream stream,
                                               half4* input_buffer,
                                               half4* albedo_buffer,
                                               half4* normal_buffer,
                                               half4* output_buffer) {
    m_cuda_stream = stream;

    // intialize device memory for the denoiser
    OptixDenoiserSizes denoiser_sizes;
    OPTIX_ERROR_CHECK(optixDenoiserComputeMemoryResources(m_denoiser, w, h, &denoiser_sizes));
    m_scratch_size = static_cast<uint32_t>(denoiser_sizes.withoutOverlapScratchSizeInBytes);
    CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&md_scratch), m_scratch_size));
    CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&md_state), denoiser_sizes.stateSizeInBytes));
    m_state_size = static_cast<uint32_t>(denoiser_sizes.stateSizeInBytes);

    // set the inputs and outputs
    md_inputs = std::vector<OptixImage2D>(3);
    md_inputs[0].data = reinterpret_cast<CUdeviceptr>(input_buffer);
    md_inputs[0].width = w;
    md_inputs[0].height = h;
    md_inputs[0].rowStrideInBytes = w * sizeof(half4);
    md_inputs[0].pixelStrideInBytes = sizeof(half4);
    md_inputs[0].format = OPTIX_PIXEL_FORMAT_HALF4;

    md_inputs[1].data = reinterpret_cast<CUdeviceptr>(albedo_buffer);
    md_inputs[1].width = w;
    md_inputs[1].height = h;
    md_inputs[1].rowStrideInBytes = w * sizeof(half4);
    md_inputs[1].pixelStrideInBytes = sizeof(half4);
    md_inputs[1].format = OPTIX_PIXEL_FORMAT_HALF4;

    md_inputs[2].data = reinterpret_cast<CUdeviceptr>(normal_buffer);
    md_inputs[2].width = w;
    md_inputs[2].height = h;
    md_inputs[2].rowStrideInBytes = w * sizeof(half4);
    md_inputs[2].pixelStrideInBytes = sizeof(half4);
    md_inputs[2].format = OPTIX_PIXEL_FORMAT_HALF4;

    md_output = {};
    md_output.data = reinterpret_cast<CUdeviceptr>(output_buffer);
    md_output.width = w;
    md_output.height = h;
    md_output.rowStrideInBytes = w * sizeof(half4);
    md_output.pixelStrideInBytes = sizeof(half4);
    md_output.format = OPTIX_PIXEL_FORMAT_HALF4;

    OPTIX_ERROR_CHECK(optixDenoiserSetup(m_denoiser, m_cuda_stream, w, h, md_state, m_state_size, md_scratch, m_scratch_size));
    m_params.hdrIntensity = 0;
    m_params.blendFactor = 0.f;
}

CH_SENSOR_API void ChOptixDenoiser::Execute() {
    // will not compute intensity since we are assuming we don't have HDR images
    OptixDenoiserLayer inLayer = {};
    inLayer.input = *(md_inputs.data());
    inLayer.previousOutput = md_output;  // only in temporal mode
    inLayer.output = md_output;

    OptixDenoiserGuideLayer guideLayer = {};
    guideLayer.albedo = md_inputs[1];
    guideLayer.normal = md_inputs[2];

    OPTIX_ERROR_CHECK(optixDenoiserInvoke(m_denoiser,     // denoiser OK
                                          m_cuda_stream,  // CUstream stream OK
                                          &m_params,      /// OptixDenoiserParams* params OK
                                          md_state,       // denoiserState OK
                                          m_state_size,   // denoiserStateSizeInBytes OK
                                          &guideLayer,    // OptixDenoiserGuideLayer* guidelayer
                                          &inLayer,       // OptixeDenoiserLayer* layer
                                          1,              // uint num layers
                                          0,              // uint inputOffsetX
                                          0,              // uint inputOffsetY
                                          md_scratch,     // CUdeviceptr scratch
                                          m_scratch_size  // scratchSizeInBytes
                                          ));
}

}  // namespace sensor
}  // namespace chrono
