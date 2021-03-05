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

#include "chrono_sensor/filters/ChFilterOptixRender.h"
#include <assert.h>
#include <algorithm>
#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/ChLidarSensor.h"
#include "chrono_sensor/ChSensor.h"
#include "chrono_sensor/ChSensorBuffer.h"

#include <cstdlib>
#include <ctime>
#include <chrono>

using std::chrono::high_resolution_clock;
using std::chrono::duration;

namespace chrono {
namespace sensor {

ChFilterOptixRender::ChFilterOptixRender() : ChFilter("OptixRenderer") {}

CH_SENSOR_API void ChFilterOptixRender::Apply(std::shared_ptr<ChSensor> pSensor,
                                              std::shared_ptr<SensorBuffer>& bufferInOut) {
    // this filter is presumed to be the first filter in a sensor's filter list, so the bufferIn should be null.
    assert(bufferInOut == nullptr);

    // to render, the sensor *must* inherit from ChOptixSensor
    std::shared_ptr<ChOptixSensor> pOptixSensor = std::dynamic_pointer_cast<ChOptixSensor>(pSensor);
    if (!pOptixSensor) {
        throw std::runtime_error(
            "The optix render filter must be attached to a sensor that inherits from ChOptixSensor");
    }

    m_buffer->Width = pOptixSensor->m_width;
    m_buffer->Height = pOptixSensor->m_height;
    m_buffer->LaunchedCount = pOptixSensor->GetNumLaunches();
    m_buffer->TimeStamp = pOptixSensor->m_time_stamp;

    high_resolution_clock::time_point start;
    high_resolution_clock::time_point end;
    duration<double, std::milli> duration_sec;

    try {
        if (m_use_gi) {
            // start = high_resolution_clock::now();
            commandListWithDenoiser->execute();
            // m_program["gi_pass"]->setInt(1);
            // pOptixSensor->m_context->launch(pOptixSensor->m_launch_index, pOptixSensor->m_width,
            //                                            pOptixSensor->m_height);
            // m_program["gi_pass"]->setInt(0);
            // end = high_resolution_clock::now();
            // duration_sec = std::chrono::duration_cast<duration<double, std::milli>>(end - start);
            // std::cout << duration_sec.count() << std::endl;
        } else {
            // start = high_resolution_clock::now();
            commandListWithDenoiser->execute();
            // end = high_resolution_clock::now();
            // duration_sec = std::chrono::duration_cast<duration<double, std::milli>>(end - start);
            // std::cout << duration_sec.count() << std::endl;
        }

    } catch (const optix::Exception& ex) {
        throw std::runtime_error("Error launching optix render for sensor " + pSensor->GetName() + ": " +
                                 ex.getErrorString());
    }

    bufferInOut = m_buffer;
}

CH_SENSOR_API void ChFilterOptixRender::Initialize(std::shared_ptr<ChSensor> pSensor) {
    std::shared_ptr<ChOptixSensor> pOptixSensor = std::dynamic_pointer_cast<ChOptixSensor>(pSensor);
    if (!pOptixSensor) {
        throw std::runtime_error(
            "The optix render filter must be attached to a sensor that inherits from ChOptixSensor");
    }

    // allocate the buffer and ray generation program
    optix::Context context = pOptixSensor->m_context;
    std::cout << pOptixSensor->RenderProgramString().program_name << std::endl;
    m_use_gi = pOptixSensor->m_use_gi;

    optix::Program ray_gen_program = GetRTProgram(context, pOptixSensor->RenderProgramString().file_name,
                                                  pOptixSensor->RenderProgramString().program_name);

    // get the sensor reference frames
    ChFrame<double> f_offset = pSensor->GetOffsetPose();
    ChFrame<double> f_body = pSensor->GetParent()->GetAssetsFrame();
    ChFrame<double> global_loc = f_body * f_offset;

    // set the render reference frames for OptiX
    ray_gen_program["gi_pass"]->setInt(0);
    ray_gen_program["c_pos"]->setFloat((float)global_loc.GetPos().x(), (float)global_loc.GetPos().y(),
                                       (float)global_loc.GetPos().z());
    ray_gen_program["c_forward"]->setFloat((float)global_loc.GetA()(0), (float)global_loc.GetA()(2),
                                           (float)global_loc.GetA()(5));  // camera forward
    ray_gen_program["c_left"]->setFloat((float)global_loc.GetA()(1), (float)global_loc.GetA()(3),
                                        (float)global_loc.GetA()(6));  // camera left
    ray_gen_program["c_up"]->setFloat((float)global_loc.GetA()(2), (float)global_loc.GetA()(4),
                                      (float)global_loc.GetA()(7));  // camera up

    // go through any custom parameters
    for (auto param : pOptixSensor->RayLaunchParameters()) {
        std::string var_name = std::get<0>(param);
        RTobjecttype type_enum = std::get<1>(param);
        void* var_value = std::get<2>(param);

        switch (type_enum) {
            case RT_OBJECTTYPE_FLOAT: {
                ray_gen_program[var_name]->setFloat(*reinterpret_cast<float*>(var_value));
            } break;
            case RT_OBJECTTYPE_FLOAT2: {
                float* f2 = reinterpret_cast<float*>(var_value);
                ray_gen_program[var_name]->setFloat(f2[0], f2[1]);
            } break;
            case RT_OBJECTTYPE_FLOAT3: {
                float* f3 = reinterpret_cast<float*>(var_value);
                ray_gen_program[var_name]->setFloat(f3[0], f3[1], f3[2]);
            } break;
            case RT_OBJECTTYPE_FLOAT4: {
                float* f4 = reinterpret_cast<float*>(var_value);
                ray_gen_program[var_name]->setFloat(f4[0], f4[1], f4[2], f4[3]);
            } break;
            case RT_OBJECTTYPE_INT: {
                ray_gen_program[var_name]->setInt(*reinterpret_cast<int*>(var_value));
            } break;
            case RT_OBJECTTYPE_INT2: {
                int* i2 = reinterpret_cast<int*>(var_value);
                ray_gen_program[var_name]->setInt(i2[0], i2[1]);
            } break;
            case RT_OBJECTTYPE_INT3: {
                int* i3 = reinterpret_cast<int*>(var_value);
                ray_gen_program[var_name]->setInt(i3[0], i3[1], i3[2]);
            } break;
            case RT_OBJECTTYPE_INT4: {
                int* i4 = reinterpret_cast<int*>(var_value);
                ray_gen_program[var_name]->setInt(i4[0], i4[1], i4[2], i4[3]);
            } break;
            case RT_OBJECTTYPE_UNSIGNED_INT: {
                ray_gen_program[var_name]->setUint(*reinterpret_cast<unsigned int*>(var_value));
            } break;
            case RT_OBJECTTYPE_UNSIGNED_INT2: {
                unsigned int* ui2 = reinterpret_cast<unsigned int*>(var_value);
                ray_gen_program[var_name]->setUint(ui2[0], ui2[1]);
            } break;
            case RT_OBJECTTYPE_UNSIGNED_INT3: {
                unsigned int* ui3 = reinterpret_cast<unsigned int*>(var_value);
                ray_gen_program[var_name]->setUint(ui3[0], ui3[1], ui3[2]);
            } break;
            case RT_OBJECTTYPE_UNSIGNED_INT4: {
                unsigned int* ui4 = reinterpret_cast<unsigned int*>(var_value);
                ray_gen_program[var_name]->setUint(ui4[0], ui4[1], ui4[2], ui4[3]);
            } break;
            default:
                std::cerr << "WARNING: custom parameter type not yet supported\n";
                break;
        }
    }

    m_program = ray_gen_program;

    // give the ray gen program back to the sensor (DANGEROUS THREADING ISSUES - SHOULD THE SENSOR REALLY EXPOSE
    // THIS TO THE USER?)
    pOptixSensor->m_ray_gen = m_program;

    unsigned int numEntryPoints = context->getEntryPointCount();
    context->setEntryPointCount(numEntryPoints + 1);  // set the entry point count
    pOptixSensor->m_launch_index = numEntryPoints;
    context->setRayGenerationProgram(pOptixSensor->m_launch_index, pOptixSensor->m_ray_gen);

    optix::Buffer buffer = AllocateBuffer(pSensor);
    pOptixSensor->m_ray_gen["output_buffer"]->set(buffer);
    m_buffer = std::make_unique<SensorOptixBuffer>();
    m_buffer->Buffer = buffer;
    m_buffer->Width = pOptixSensor->m_width;
    m_buffer->Height = pOptixSensor->m_height;

    commandListWithDenoiser = pOptixSensor->m_context->createCommandList();
    commandListWithDenoiser->appendLaunch(pOptixSensor->m_launch_index, pOptixSensor->m_width, pOptixSensor->m_height);

    optix::Buffer gi_tonemap_output_buffer;
    if (pOptixSensor->m_use_tonemapper) {  // only a camera would want to use a tonemapper, lidar and radar will ignore
                                           // this process
        gi_tonemap_output_buffer = AllocateBuffer(pSensor);
        optix::PostprocessingStage tonemapStage =
            pOptixSensor->m_context->createBuiltinPostProcessingStage("TonemapperSimple");

        tonemapStage->declareVariable("input_buffer")->set(buffer);
        tonemapStage->declareVariable("output_buffer")->set(gi_tonemap_output_buffer);
        tonemapStage->declareVariable("exposure")->setFloat(1.0f);
        tonemapStage->declareVariable("gamma")->setFloat(2.2f);
        commandListWithDenoiser->appendPostprocessingStage(tonemapStage, pOptixSensor->m_width, pOptixSensor->m_height);
        if (!pOptixSensor->m_use_gi) {
            m_buffer->Buffer = gi_tonemap_output_buffer;
        }
    }

    // setup denoiser buffer for global illumination
    if (pOptixSensor->m_use_gi) {
        optix::Buffer normal_buffer = AllocateBuffer(pSensor);
        optix::Buffer albedo_buffer = AllocateBuffer(pSensor);
        optix::Buffer gi_denoised_output_buffer = AllocateBuffer(pSensor);

        pOptixSensor->m_ray_gen["gi_pass_normal_buffer"]->set(normal_buffer);
        pOptixSensor->m_ray_gen["gi_pass_albedo_buffer"]->set(albedo_buffer);

        // Setup Optix denoiser
        optix::PostprocessingStage denoiserStage =
            pOptixSensor->m_context->createBuiltinPostProcessingStage("DLDenoiser");
        if (pOptixSensor->m_use_tonemapper) {
            denoiserStage->declareVariable("input_buffer")->set(gi_tonemap_output_buffer);
        } else {
            denoiserStage->declareVariable("input_buffer")->set(buffer);
        }
        denoiserStage->declareVariable("output_buffer")->set(gi_denoised_output_buffer);
        denoiserStage->declareVariable("blend")->setFloat(0);
        denoiserStage->declareVariable("input_albedo_buffer")->set(albedo_buffer);
        denoiserStage->declareVariable("input_normal_buffer")->set(normal_buffer);

        commandListWithDenoiser->appendPostprocessingStage(denoiserStage, pOptixSensor->m_width,
                                                           pOptixSensor->m_height);

        m_buffer->Buffer = gi_denoised_output_buffer;
    }

    commandListWithDenoiser->finalize();

    // check that the context is valid
    try {
        pOptixSensor->m_context->validate();
    } catch (const optix::Exception& ex) {
        std::string str = ex.getErrorString();
        std::cout << str;
    }
}

CH_SENSOR_API optix::Buffer ChFilterOptixRender::AllocateBuffer(std::shared_ptr<ChSensor> pSensor) {
    // to render, the sensor *must* inherit from ChOptixSensor
    if (auto pOptixSensor = std::dynamic_pointer_cast<ChOptixSensor>(pSensor)) {
        optix::Buffer buffer = pOptixSensor->m_context->createBuffer(RT_BUFFER_INPUT_OUTPUT | RT_BUFFER_COPY_ON_DIRTY,
                                                                     pOptixSensor->RenderBufferFormat(),
                                                                     pOptixSensor->m_width, pOptixSensor->m_height);
        return buffer;
    } else {
        throw std::runtime_error("Cannot allocate OptiX buffer for non-optix sensor");
    }

    return NULL;
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
}  // namespace sensor
}  // namespace chrono
