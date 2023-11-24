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
//
// =============================================================================

#include "chrono_sensor/optix/ChOptixPipeline.h"
#include "chrono_sensor/optix/ChOptixUtils.h"

#include "chrono/core/ChGlobal.h"
#include "chrono_thirdparty/filesystem/path.h"

#include <optix_stack_size.h>
#include <optix_stubs.h>

namespace chrono {
namespace sensor {

ChOptixPipeline::ChOptixPipeline(OptixDeviceContext context, unsigned int trace_depth, bool debug)
    : m_trace_depth(trace_depth), m_context(context), m_debug(debug) {
    // define some pipeline basics
    m_pipeline_compile_options = {
        true,                                    // use motion blur
        OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_ANY,  // traversableGraphFlags
        3,                          // all ray gens should pack data into a pointer (2 ints worth) and a Ray type
        8,                          // geometry uses 8 attributes
        OPTIX_EXCEPTION_FLAG_NONE,  // exceptionFlags
        "params",                   // pipelineLaunchParamsVariableName
        0                           // use custom primitives and triangles
    };
    if (m_debug) {
        m_pipeline_compile_options.exceptionFlags = OPTIX_EXCEPTION_FLAG_DEBUG;
    }

    CompileBaseShaders();
    AssembleBaseProgramGroups();
    CreateBaseSBT();
}

ChOptixPipeline::~ChOptixPipeline() {
    Cleanup();
}

void ChOptixPipeline::Cleanup() {
    //=== do the same as when resetting pipeline
    CleanMaterials();

    //=== do extra cleanup only needed when destroying optix pipeline

    // optix modules
    if (m_box_intersection_module) {
        OPTIX_ERROR_CHECK(optixModuleDestroy(m_box_intersection_module));
        m_box_intersection_module = 0;
    }
    if (m_sphere_intersection_module) {
        OPTIX_ERROR_CHECK(optixModuleDestroy(m_sphere_intersection_module));
        m_sphere_intersection_module = 0;
    }
    if (m_cyl_intersection_module) {
        OPTIX_ERROR_CHECK(optixModuleDestroy(m_cyl_intersection_module));
        m_cyl_intersection_module = 0;
    }
    if (m_camera_raygen_module) {
        OPTIX_ERROR_CHECK(optixModuleDestroy(m_camera_raygen_module));
        m_camera_raygen_module = 0;
    }
    if (m_lidar_raygen_module) {
        OPTIX_ERROR_CHECK(optixModuleDestroy(m_lidar_raygen_module));
        m_lidar_raygen_module = 0;
    }
    if (m_radar_raygen_module) {
        OPTIX_ERROR_CHECK(optixModuleDestroy(m_radar_raygen_module));
        m_radar_raygen_module = 0;
    }
    if (m_material_shading_module) {
        OPTIX_ERROR_CHECK(optixModuleDestroy(m_material_shading_module));
        m_material_shading_module = 0;
    }
    if (m_miss_module) {
        OPTIX_ERROR_CHECK(optixModuleDestroy(m_miss_module));
        m_miss_module = 0;
    }
    // === optix program groups ===
    // raygen groups
    if (m_camera_raygen_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_camera_raygen_group));
        m_camera_raygen_group = 0;
    }
    // if (m_camera_fov_lens_raygen_group) {
    //     OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_camera_fov_lens_raygen_group));
    //     m_camera_fov_lens_raygen_group = 0;
    // }
    if (m_segmentation_raygen_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_segmentation_raygen_group));
        m_segmentation_raygen_group = 0;
    }
    // if (m_segmentation_fov_lens_raygen_group) {
    //     OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_segmentation_fov_lens_raygen_group));
    //     m_segmentation_fov_lens_raygen_group = 0;
    // }
    if (m_lidar_single_raygen_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_lidar_single_raygen_group));
        m_lidar_single_raygen_group = 0;
    }
    if (m_lidar_multi_raygen_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_lidar_multi_raygen_group));
        m_lidar_multi_raygen_group = 0;
    }
    if (m_radar_raygen_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_radar_raygen_group));
        m_radar_raygen_group = 0;
    }

    // miss groups
    if (m_miss_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_miss_group));
        m_miss_group = 0;
    }

    // hit groups
    if (m_hit_box_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_hit_box_group));
        m_hit_box_group = 0;
    }

    if (m_hit_sphere_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_hit_sphere_group));
        m_hit_sphere_group = 0;
    }
    if (m_hit_cyl_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_hit_cyl_group));
        m_hit_cyl_group = 0;
    }

    if (m_hit_mesh_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_hit_mesh_group));
        m_hit_mesh_group = 0;
    }
    // clean up environment map data if it exists
    // clear out and free texture samplers
    if (md_miss_img_texture) {
        CUDA_ERROR_CHECK(cudaFreeArray(md_miss_img_texture));
        md_miss_img_texture = 0;
    }
    if (md_miss_texture_sampler) {
        CUDA_ERROR_CHECK(cudaDestroyTextureObject(md_miss_texture_sampler));
        md_miss_texture_sampler = 0;
    }

    // pipelines
    for (auto p : m_pipelines) {
        OPTIX_ERROR_CHECK(optixPipelineDestroy(p));
    }
    m_pipelines.clear();

    // raygen records
    m_raygen_records.clear();

    // shader binding tables
    m_sbts.clear();

    // miss records
    if (md_miss_record) {
        CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void*>(md_miss_record)));
        md_miss_record = {};
    }
}

void ChOptixPipeline::CleanMaterials() {
    // clear out all material records and information so we can start fresh

    // clear the material records
    if (md_material_records) {
        CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void*>(md_material_records)));
        md_material_records = {};
    }
    m_material_records.clear();

    // reset mesh pool
    if (md_mesh_pool) {
        CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void*>(md_mesh_pool)));
        md_mesh_pool = {};
    }
    for (int i = 0; i < m_mesh_buffers_dptrs.size(); i++) {
        if (m_mesh_buffers_dptrs[i]) {
            CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void*>(m_mesh_buffers_dptrs[i])));
        }
    }
    m_mesh_buffers_dptrs.clear();
    m_mesh_pool.clear();
    m_known_meshes.clear();
    // clear our deformable meshes
    m_deformable_meshes.clear();

    // reset material pool
    if (md_material_pool) {
        CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void*>(md_material_pool)));
        md_material_pool = {};
    }
    m_material_pool.clear();

    // clear out and free texture samplers
    for (auto it : m_img_textures) {
        if (it.second)
            CUDA_ERROR_CHECK(cudaFreeArray(it.second));
    }
    m_img_textures.clear();
    for (auto it : m_texture_samplers) {
        if (it.second)
            CUDA_ERROR_CHECK(cudaDestroyTextureObject(it.second));
    }
    m_texture_samplers.clear();

    // pool defaults (1 per sensor type)
    m_default_material_inst = false;
    m_default_material_id = 0;
}

void ChOptixPipeline::CompileBaseShaders() {
    OptixModuleCompileOptions module_compile_options = {};
    if (m_debug) {
        module_compile_options.optLevel = OPTIX_COMPILE_OPTIMIZATION_LEVEL_0;
        module_compile_options.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_FULL;
    } else {
        module_compile_options.optLevel = OPTIX_COMPILE_OPTIMIZATION_LEVEL_3;
        module_compile_options.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_NONE;
    }

    // get and compile all of our modules
    // intersection shaders

    auto start_compile = std::chrono::high_resolution_clock::now();

    GetShaderFromFile(m_context, m_box_intersection_module, "box", module_compile_options, m_pipeline_compile_options);
    GetShaderFromFile(m_context, m_sphere_intersection_module, "sphere", module_compile_options,
                      m_pipeline_compile_options);
    GetShaderFromFile(m_context, m_cyl_intersection_module, "cylinder", module_compile_options,
                      m_pipeline_compile_options);
    // material shaders
    GetShaderFromFile(m_context, m_material_shading_module, "material_shaders", module_compile_options,
                      m_pipeline_compile_options);
    // ray gen shaders
    GetShaderFromFile(m_context, m_camera_raygen_module, "camera", module_compile_options, m_pipeline_compile_options);
    GetShaderFromFile(m_context, m_lidar_raygen_module, "lidar", module_compile_options, m_pipeline_compile_options);
    GetShaderFromFile(m_context, m_radar_raygen_module, "radar", module_compile_options, m_pipeline_compile_options);
    GetShaderFromFile(m_context, m_miss_module, "miss", module_compile_options, m_pipeline_compile_options);

    auto end_compile = std::chrono::high_resolution_clock::now();

    auto wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(end_compile - start_compile);
    std::cout << "Shader compile time: " << wall_time.count() << std::endl;
}

void ChOptixPipeline::CreateOptixProgramGroup(OptixProgramGroup& group,
                                              OptixProgramGroupKind k,
                                              OptixModule is_module,
                                              const char* is_name,
                                              OptixModule ch_module,
                                              const char* ch_name) {
    char log[2048];
    size_t sizeof_log = sizeof(log);

    OptixProgramGroupOptions group_options = {};
    OptixProgramGroupDesc group_desc = {};
    group_desc.kind = k;

    switch (k) {
        case OPTIX_PROGRAM_GROUP_KIND_HITGROUP: {
            group_desc.hitgroup.moduleIS = is_module;
            group_desc.hitgroup.entryFunctionNameIS = is_name;
            group_desc.hitgroup.moduleCH = ch_module;
            group_desc.hitgroup.entryFunctionNameCH = ch_name;
            break;
        }
        case OPTIX_PROGRAM_GROUP_KIND_MISS: {
            group_desc.miss.module = ch_module;
            group_desc.miss.entryFunctionName = ch_name;
            break;
        }
        case OPTIX_PROGRAM_GROUP_KIND_RAYGEN: {
            group_desc.raygen.module = ch_module;
            group_desc.raygen.entryFunctionName = ch_name;
            break;
        }
        default:
            break;
    }

    OPTIX_ERROR_CHECK(optixProgramGroupCreate(m_context, &group_desc, 1, &group_options, log, &sizeof_log, &group));
}

void ChOptixPipeline::AssembleBaseProgramGroups() {
    // box intersection and shading
    CreateOptixProgramGroup(m_hit_box_group, OPTIX_PROGRAM_GROUP_KIND_HITGROUP, m_box_intersection_module,
                            "__intersection__box_intersect", m_material_shading_module,
                            "__closesthit__material_shader");
    // sphere intersection and shading
    CreateOptixProgramGroup(m_hit_sphere_group, OPTIX_PROGRAM_GROUP_KIND_HITGROUP, m_sphere_intersection_module,
                            "__intersection__sphere_intersect", m_material_shading_module,
                            "__closesthit__material_shader");
    // cylinder intersection and shading
    CreateOptixProgramGroup(m_hit_cyl_group, OPTIX_PROGRAM_GROUP_KIND_HITGROUP, m_cyl_intersection_module,
                            "__intersection__cylinder_intersect", m_material_shading_module,
                            "__closesthit__material_shader");
    // mesh shading
    CreateOptixProgramGroup(m_hit_mesh_group, OPTIX_PROGRAM_GROUP_KIND_HITGROUP, nullptr, nullptr,
                            m_material_shading_module, "__closesthit__material_shader");
    // miss shading
    CreateOptixProgramGroup(m_miss_group, OPTIX_PROGRAM_GROUP_KIND_MISS, nullptr, nullptr, m_miss_module,
                            "__miss__shader");
    // camera pinhole raygen
    CreateOptixProgramGroup(m_camera_raygen_group, OPTIX_PROGRAM_GROUP_KIND_RAYGEN, nullptr, nullptr,
                            m_camera_raygen_module, "__raygen__camera");
    // // camera fov lens raygen
    // CreateOptixProgramGroup(m_camera_fov_lens_raygen_group, OPTIX_PROGRAM_GROUP_KIND_RAYGEN, nullptr, nullptr,
    //                         m_camera_raygen_module, "__raygen__camera_fov_lens");
    // segmentation pinhole raygen
    CreateOptixProgramGroup(m_segmentation_raygen_group, OPTIX_PROGRAM_GROUP_KIND_RAYGEN, nullptr, nullptr,
                            m_camera_raygen_module, "__raygen__segmentation");
    // // segmentation fov lens raygen
    // CreateOptixProgramGroup(m_segmentation_fov_lens_raygen_group, OPTIX_PROGRAM_GROUP_KIND_RAYGEN, nullptr, nullptr,
    //                         m_camera_raygen_module, "__raygen__segmentation_fov_lens");
    // lidar single raygen
    CreateOptixProgramGroup(m_lidar_single_raygen_group, OPTIX_PROGRAM_GROUP_KIND_RAYGEN, nullptr, nullptr,
                            m_lidar_raygen_module, "__raygen__lidar_single");
    // lidar multi raygen
    CreateOptixProgramGroup(m_lidar_multi_raygen_group, OPTIX_PROGRAM_GROUP_KIND_RAYGEN, nullptr, nullptr,
                            m_lidar_raygen_module, "__raygen__lidar_multi");
    // radar raygen
    CreateOptixProgramGroup(m_radar_raygen_group, OPTIX_PROGRAM_GROUP_KIND_RAYGEN, nullptr, nullptr,
                            m_radar_raygen_module, "__raygen__radar");
}

void ChOptixPipeline::CreateBaseSBT() {
    // miss record - should only ever need one of these
    CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&md_miss_record), sizeof(Record<MissParameters>)));
    Record<MissParameters> miss_rec;
    OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_miss_group, &miss_rec));

    // camera miss record data
    miss_rec.data.camera_miss.mode = BackgroundMode::GRADIENT;
    miss_rec.data.camera_miss.color_zenith = {0.2f, 0.3f, 0.4f};
    miss_rec.data.camera_miss.color_zenith = {0.7f, 0.8f, 0.9f};

    CUDA_ERROR_CHECK(cudaMemcpy(reinterpret_cast<void*>(md_miss_record), &miss_rec, sizeof(Record<MissParameters>),
                                cudaMemcpyHostToDevice));
}

void ChOptixPipeline::UpdateBackground(Background b) {
    // miss record - should only ever need one of these
    Record<MissParameters> miss_rec;
    OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_miss_group, &miss_rec));

    miss_rec.data.camera_miss.mode = b.mode;
    miss_rec.data.camera_miss.color_zenith = make_float3(b.color_zenith.x(), b.color_zenith.y(), b.color_zenith.z());
    miss_rec.data.camera_miss.color_horizon =
        make_float3(b.color_horizon.x(), b.color_horizon.y(), b.color_horizon.z());

    if (b.mode == BackgroundMode::ENVIRONMENT_MAP) {
        // destroy these objects if they already exist
        if (md_miss_img_texture) {
            CUDA_ERROR_CHECK(cudaFreeArray(md_miss_img_texture));
        }
        if (md_miss_texture_sampler) {
            CUDA_ERROR_CHECK(cudaDestroyTextureObject(md_miss_texture_sampler));
        }
        CreateDeviceTexture(md_miss_texture_sampler, md_miss_img_texture, b.env_tex, false, false);
        miss_rec.data.camera_miss.env_map = md_miss_texture_sampler;
    }

    CUDA_ERROR_CHECK(cudaMemcpy(reinterpret_cast<void*>(md_miss_record), &miss_rec, sizeof(Record<MissParameters>),
                                cudaMemcpyHostToDevice));
}

void ChOptixPipeline::SpawnPipeline(PipelineType type) {
    // build up the pipeline for this specific sensor
    // always add in this order. Will match the shader binding table that is constructed elsewhere.
    // 1. raygen group
    // 2. hit groups
    // 3. miss groups

    std::vector<OptixProgramGroup> program_groups;

    // create the sbt that corresponds to the pipeline
    auto b = chrono_types::make_shared<OptixShaderBindingTable>();

    // raygen record
    CUdeviceptr d_raygen_record;
    CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_raygen_record), sizeof(Record<RaygenParameters>)));
    b->raygenRecord = d_raygen_record;
    auto raygen_record = chrono_types::make_shared<Record<RaygenParameters>>();
    raygen_record->data.t0 = 0.f;
    raygen_record->data.t1 = 1.f;
    raygen_record->data.pos0 = {0.f, 0.f, 0.f};       // default value
    raygen_record->data.rot0 = {1.f, 0.f, 0.f, 0.f};  // default value
    raygen_record->data.pos1 = {0.f, 0.f, 0.f};       // default value
    raygen_record->data.rot1 = {1.f, 0.f, 0.f, 0.f};  // default value
    m_raygen_records.push_back(raygen_record);

    // add raygen program group first
    switch (type) {
        case PipelineType::CAMERA: {
            program_groups.push_back(m_camera_raygen_group);
            OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_camera_raygen_group, raygen_record.get()));
            raygen_record->data.specific.camera.hFOV = 3.14f / 4;      // default value
            raygen_record->data.specific.camera.frame_buffer = {};     // default value
            raygen_record->data.specific.camera.use_gi = false;        // default value
            raygen_record->data.specific.camera.use_fog = true;        // default value
            raygen_record->data.specific.camera.gamma = 2.2f;          // default value
            raygen_record->data.specific.camera.lens_model = PINHOLE;  // default value
            raygen_record->data.specific.camera.lens_parameters = {};
            break;
        }

            // case PipelineType::CAMERA_FOV_LENS: {
            //     program_groups.push_back(m_camera_fov_lens_raygen_group);
            //     OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_camera_fov_lens_raygen_group, raygen_record.get()));
            //     raygen_record->data.specific.camera.hFOV = 3.14f / 4;   // default value
            //     raygen_record->data.specific.camera.frame_buffer = {};  // default value
            //     raygen_record->data.specific.camera.use_gi = false;     // default value
            //     raygen_record->data.specific.camera.use_fog = true;     // default value
            //     raygen_record->data.specific.camera.gamma = 2.2f;        // default value
            //     break;
            // }

        case PipelineType::SEGMENTATION: {
            program_groups.push_back(m_segmentation_raygen_group);
            OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_segmentation_raygen_group, raygen_record.get()));
            raygen_record->data.specific.segmentation.hFOV = 3.14f / 4;      // default value
            raygen_record->data.specific.segmentation.frame_buffer = {};     // default value
            raygen_record->data.specific.segmentation.lens_model = PINHOLE;  // default value
            raygen_record->data.specific.segmentation.lens_parameters = {};
            break;
        }

            // case PipelineType::SEGMENTATION_FOV_LENS: {
            //     program_groups.push_back(m_segmentation_fov_lens_raygen_group);
            //     OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_segmentation_fov_lens_raygen_group,
            //     raygen_record.get())); raygen_record->data.specific.segmentation.hFOV = 3.14f / 4;   // default value
            //     raygen_record->data.specific.segmentation.frame_buffer = {};  // default value
            //     break;
            // }

        case PipelineType::LIDAR_SINGLE: {
            program_groups.push_back(m_lidar_single_raygen_group);
            OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_lidar_single_raygen_group, raygen_record.get()));
            raygen_record->data.specific.lidar.frame_buffer = {};                         // default value
            raygen_record->data.specific.lidar.max_vert_angle = 1.f;                      // default value
            raygen_record->data.specific.lidar.min_vert_angle = -1.f;                     // default value
            raygen_record->data.specific.lidar.hFOV = (float)CH_C_2PI;                    // default value
            raygen_record->data.specific.lidar.beam_shape = LidarBeamShape::RECTANGULAR;  // default value
            raygen_record->data.specific.lidar.sample_radius = 1;                         // default value
            raygen_record->data.specific.lidar.horiz_div_angle = 0.f;                     // default value
            raygen_record->data.specific.lidar.vert_div_angle = 0.f;                      // default value
            raygen_record->data.specific.lidar.max_distance = 200.f;                      // default value
            raygen_record->data.specific.lidar.clip_near = 0.f;                           // default value
            break;
        }

        case PipelineType::LIDAR_MULTI: {
            program_groups.push_back(m_lidar_multi_raygen_group);
            OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_lidar_multi_raygen_group, raygen_record.get()));
            raygen_record->data.specific.lidar.frame_buffer = {};                         // default value
            raygen_record->data.specific.lidar.max_vert_angle = 1.f;                      // default value
            raygen_record->data.specific.lidar.min_vert_angle = -1.f;                     // default value
            raygen_record->data.specific.lidar.hFOV = (float)CH_C_2PI;                    // default value
            raygen_record->data.specific.lidar.beam_shape = LidarBeamShape::RECTANGULAR;  // default value
            raygen_record->data.specific.lidar.sample_radius = 1;                         // default value
            raygen_record->data.specific.lidar.horiz_div_angle = 0.f;                     // default value
            raygen_record->data.specific.lidar.vert_div_angle = 0.f;                      // default value
            raygen_record->data.specific.lidar.max_distance = 200.f;                      // default value
            raygen_record->data.specific.lidar.clip_near = 0.f;                           // default value
            break;
        }

        case PipelineType::RADAR: {
            program_groups.push_back(m_radar_raygen_group);
            OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_radar_raygen_group, raygen_record.get()));
            raygen_record->data.specific.radar.frame_buffer = {};      // default value
            raygen_record->data.specific.radar.vFOV = (float)CH_C_PI;  // default value
            raygen_record->data.specific.radar.hFOV = (float)CH_C_PI;  // default value
            raygen_record->data.specific.radar.max_distance = 200.f;   // default value
            raygen_record->data.specific.radar.clip_near = 0.f;        // default value
            break;
        }
        default:
            throw ChException("Unsupported pipeline type: unknown type");
    }

    program_groups.push_back(m_hit_box_group);
    program_groups.push_back(m_hit_sphere_group);
    program_groups.push_back(m_hit_cyl_group);
    program_groups.push_back(m_hit_mesh_group);
    program_groups.push_back(m_miss_group);

    OptixPipelineLinkOptions pipeline_link_options = {m_trace_depth};

    char log[2048];
    size_t sizeof_log = sizeof(log);
    // OptixPipeline pipeline;
    m_pipelines.emplace_back();
    auto id = m_pipelines.size() - 1;
    OPTIX_ERROR_CHECK(optixPipelineCreate(m_context, &m_pipeline_compile_options, &pipeline_link_options,
                                          program_groups.data(), static_cast<unsigned int>(program_groups.size()), log,
                                          &sizeof_log, &m_pipelines[id]));
    OptixStackSizes stack_sizes = {};
    for (auto& prog_group : program_groups) {
        OPTIX_ERROR_CHECK(optixUtilAccumulateStackSizes(prog_group, &stack_sizes, m_pipelines[id]));
    }
    uint32_t direct_callable_stack_size_from_traversal;
    uint32_t direct_callable_stack_size_from_state;
    uint32_t continuation_stack_size;
    OPTIX_ERROR_CHECK(optixUtilComputeStackSizes(&stack_sizes, m_trace_depth,
                                                 0,  // maxCCDepth
                                                 0,  // maxDCDepth
                                                 &direct_callable_stack_size_from_traversal,
                                                 &direct_callable_stack_size_from_state, &continuation_stack_size));
    OPTIX_ERROR_CHECK(optixPipelineSetStackSize(m_pipelines[id], direct_callable_stack_size_from_traversal,
                                                direct_callable_stack_size_from_state, continuation_stack_size,
                                                4  // max tree depth (IAS root, motion t, static t, GAS)
                                                ));
    // set the miss program record - same for all pipelines
    b->missRecordBase = md_miss_record;
    b->missRecordCount = 1;  // only one function for missed rays
    b->missRecordStrideInBytes = static_cast<uint32_t>(sizeof(Record<MissParameters>));

    // set the shader program record - same for all pipelines
    b->hitgroupRecordBase = md_material_records;
    b->hitgroupRecordCount =
        static_cast<unsigned int>(m_material_records.size());  // we are pushing one back for each object
    b->hitgroupRecordStrideInBytes = static_cast<uint32_t>(sizeof(Record<MaterialRecordParameters>));
    m_sbts.push_back(b);
}

OptixPipeline& ChOptixPipeline::GetPipeline(unsigned int id) {
    if (id >= m_pipelines.size() || !m_pipelines[id]) {
        throw std::runtime_error("Cannot return a null pipeline!");
    }
    return m_pipelines[id];
}

std::shared_ptr<OptixShaderBindingTable> ChOptixPipeline::GetSBT(unsigned int id) {
    if (id >= m_sbts.size()) {
        throw std::runtime_error("Index out of bounds for shader binding tables!");
    }
    return m_sbts[id];
}

std::shared_ptr<Record<RaygenParameters>> ChOptixPipeline::GetRayGenRecord(unsigned int id) {
    if (id >= m_raygen_records.size()) {
        throw std::runtime_error("Index out of bounds for raygen records!");
    }
    return m_raygen_records[id];
}

void ChOptixPipeline::UpdateAllSBTs() {
    // clear the old memory and assume different number of materials have been added
    if (md_material_records) {
        CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void*>(md_material_records)));
        md_material_records = {};
    }

    // move the material records to the device
    CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&md_material_records),
                                sizeof(Record<MaterialRecordParameters>) * m_material_records.size()));
    CUDA_ERROR_CHECK(cudaMemcpy(reinterpret_cast<void*>(md_material_records), m_material_records.data(),
                                sizeof(Record<MaterialRecordParameters>) * m_material_records.size(),
                                cudaMemcpyHostToDevice));

    // make sure all sbts are updated to have correct parameters
    for (int b = 0; b < m_sbts.size(); b++) {
        m_sbts[b]->hitgroupRecordBase = md_material_records;
        // we are pushing one back for each ray type of each material
        m_sbts[b]->hitgroupRecordCount = static_cast<unsigned int>(m_material_records.size());
        m_sbts[b]->hitgroupRecordStrideInBytes = static_cast<uint32_t>(sizeof(Record<MaterialRecordParameters>));
    }
}

void ChOptixPipeline::UpdateAllPipelines() {
    // TODO: make sure all pipelines reflect correct modules needed
}

// move the mesh pool to the device and return the pointer to the pool
CUdeviceptr ChOptixPipeline::GetMeshPool() {
    // clear out the old pool
    if (md_mesh_pool) {
        CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void*>(md_mesh_pool)));
        md_mesh_pool = {};
    }

    // allocate memory for new pool
    CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&md_mesh_pool), sizeof(MeshParameters) * m_mesh_pool.size()));
    // move the material pool to the device
    CUDA_ERROR_CHECK(cudaMemcpy(reinterpret_cast<void*>(md_mesh_pool), m_mesh_pool.data(),
                                sizeof(MeshParameters) * m_mesh_pool.size(), cudaMemcpyHostToDevice));

    return md_mesh_pool;
}

// move the material pool to the device and return the point to the pool
CUdeviceptr ChOptixPipeline::GetMaterialPool() {
    // clear out the old pool
    if (md_material_pool) {
        CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void*>(md_material_pool)));
        md_material_pool = {};
    }

    // allocate memory for new pool
    CUDA_ERROR_CHECK(
        cudaMalloc(reinterpret_cast<void**>(&md_material_pool), sizeof(MaterialParameters) * m_material_pool.size()));
    // move the material pool to the device
    CUDA_ERROR_CHECK(cudaMemcpy(reinterpret_cast<void*>(md_material_pool), m_material_pool.data(),
                                sizeof(MaterialParameters) * m_material_pool.size(), cudaMemcpyHostToDevice));

    return md_material_pool;
}

unsigned int ChOptixPipeline::GetMaterial(std::shared_ptr<ChVisualMaterial> mat) {
    if (mat) {
        MaterialParameters material;
        material.Kd = {mat->GetDiffuseColor().R, mat->GetDiffuseColor().G, mat->GetDiffuseColor().B};
        material.Ks = {mat->GetSpecularColor().R, mat->GetSpecularColor().G, mat->GetSpecularColor().B};
        material.fresnel_exp = mat->GetFresnelExp();
        material.fresnel_min = mat->GetFresnelMin();
        material.fresnel_max = mat->GetFresnelMax();
        material.transparency = mat->GetOpacity();
        material.roughness = mat->GetRoughness();
        material.metallic = mat->GetMetallic();
        material.use_specular_workflow = mat->GetUseSpecularWorkflow();
        material.lidar_intensity = 1.f;    // TODO: allow setting of this in the visual material chrono-side
        material.radar_backscatter = 1.f;  // TODO: allow setting of this in the visual material chrono-side
        material.kn_tex = 0;               // explicitely null as default
        material.kd_tex = 0;               // explicitely null as default
        material.ks_tex = 0;               // explicitely null as default
        material.metallic_tex = 0;         // explicitely null as default
        material.roughness_tex = 0;        // explicitely null as default
        material.opacity_tex = 0;          // explicitely null as default
        material.weight_tex = 0;
        material.class_id = mat->GetClassID();
        material.instance_id = mat->GetInstanceID();

        material.tex_scale = {mat->GetTextureScale().x(), mat->GetTextureScale().y()};

        // normal texture
        if (mat->GetNormalMapTexture() != "") {
            cudaArray_t d_img_array;
            CreateDeviceTexture(material.kn_tex, d_img_array, mat->GetNormalMapTexture());
        }
        // diffuse texture
        if (mat->GetKdTexture() != "") {
            cudaArray_t d_img_array;
            CreateDeviceTexture(material.kd_tex, d_img_array, mat->GetKdTexture());
        }
        // specular texture
        if (mat->GetKsTexture() != "") {
            cudaArray_t d_img_array;
            CreateDeviceTexture(material.ks_tex, d_img_array, mat->GetKsTexture());
        }
        // metalic texture
        if (mat->GetMetallicTexture() != "") {
            cudaArray_t d_img_array;
            CreateDeviceTexture(material.metallic_tex, d_img_array, mat->GetMetallicTexture());
        }
        // roughness texture
        if (mat->GetRoughnessTexture() != "") {
            cudaArray_t d_img_array;
            CreateDeviceTexture(material.roughness_tex, d_img_array, mat->GetRoughnessTexture());
        }
        // opacity texture
        if (mat->GetOpacityTexture() != "") {
            cudaArray_t d_img_array;
            CreateDeviceTexture(material.opacity_tex, d_img_array, mat->GetOpacityTexture());
        }
        // weight texture
        if (mat->GetWeightTexture() != "") {
            cudaArray_t d_img_array;
            CreateDeviceTexture(material.weight_tex, d_img_array, mat->GetWeightTexture());
        }

        m_material_pool.push_back(material);
        return static_cast<unsigned int>(m_material_pool.size() - 1);

    } else {
        if (!m_default_material_inst) {
            MaterialParameters material;
            material.Kd = {.5f, .5f, .5f};
            material.Ks = {.2f, .2f, .2f};
            material.fresnel_exp = 5.f;
            material.fresnel_min = 0.f;
            material.fresnel_max = 1.f;
            material.transparency = 1.f;
            material.roughness = 1.f;
            material.metallic = 0.0f;
            material.lidar_intensity = 1.f;
            material.radar_backscatter = 1.f;
            material.kd_tex = 0;
            material.ks_tex = 0;
            material.kn_tex = 0;
            material.roughness_tex = 0;
            material.metallic_tex = 0;
            material.opacity_tex = 0;
            material.weight_tex = 0;
            material.use_specular_workflow = 0;
            material.class_id = 0;
            material.instance_id = 0;
            material.tex_scale = {1.f, 1.f};

            m_material_pool.push_back(material);
            m_default_material_id = static_cast<unsigned int>(m_material_pool.size() - 1);
            m_default_material_inst = true;
        }

        return m_default_material_id;
    }
}

unsigned int ChOptixPipeline::GetBoxMaterial(std::vector<std::shared_ptr<ChVisualMaterial>> mat_list) {
    unsigned int material_id;
    // if (mat) {
    //     material_id = GetMaterial(mat);
    // } else {
    //     material_id = GetMaterial();
    // }
    if (mat_list.size() > 0) {
        material_id = GetMaterial(mat_list[0]);
        for (int i = 1; i < mat_list.size(); i++) {
            GetMaterial(mat_list[i]);
        }
    } else {
        material_id = GetMaterial();
    }
    // record when hit by any ray type
    Record<MaterialRecordParameters> mat_record;
    OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_hit_box_group, &mat_record));
    mat_record.data.material_pool_id = material_id;
    mat_record.data.num_blended_materials = (unsigned int)mat_list.size();
    m_material_records.push_back(mat_record);

    return static_cast<unsigned int>(m_material_records.size() - 1);
}

unsigned int ChOptixPipeline::GetSphereMaterial(std::vector<std::shared_ptr<ChVisualMaterial>> mat_list) {
    unsigned int material_id;
    // if (mat) {
    //     material_id = GetMaterial(mat);
    // } else {
    //     material_id = GetMaterial();
    // }
    if (mat_list.size() > 0) {
        material_id = GetMaterial(mat_list[0]);
        for (int i = 1; i < mat_list.size(); i++) {
            GetMaterial(mat_list[i]);
        }
    } else {
        material_id = GetMaterial();
    }
    // record when hit by any ray type
    Record<MaterialRecordParameters> mat_record;
    OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_hit_sphere_group, &mat_record));
    mat_record.data.material_pool_id = material_id;
    mat_record.data.num_blended_materials = (unsigned int)mat_list.size();
    m_material_records.push_back(mat_record);

    return static_cast<unsigned int>(m_material_records.size() - 1);
}

unsigned int ChOptixPipeline::GetCylinderMaterial(std::vector<std::shared_ptr<ChVisualMaterial>> mat_list) {
    unsigned int material_id;
    // if (mat) {
    //     material_id = GetMaterial(mat);
    // } else {
    //     material_id = GetMaterial();
    // }
    if (mat_list.size() > 0) {
        material_id = GetMaterial(mat_list[0]);
        for (int i = 1; i < mat_list.size(); i++) {
            GetMaterial(mat_list[i]);
        }
    } else {
        material_id = GetMaterial();
    }
    // record when hit by any ray type
    Record<MaterialRecordParameters> mat_record;
    OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_hit_cyl_group, &mat_record));
    mat_record.data.material_pool_id = material_id;
    mat_record.data.num_blended_materials = 1;  // TODO: change mat to list
    m_material_records.push_back(mat_record);

    return static_cast<unsigned int>(m_material_records.size() - 1);
}

// this will actually make a new material (new mesh info), but will apply a default coloring/texture
unsigned int ChOptixPipeline::GetRigidMeshMaterial(CUdeviceptr& d_vertices,
                                                   CUdeviceptr& d_indices,
                                                   std::shared_ptr<ChVisualShapeTriangleMesh> mesh_shape,
                                                   std::vector<std::shared_ptr<ChVisualMaterial>> mat_list) {
    auto mesh = mesh_shape->GetMesh();

    // check if this mesh is known, if so, we can just get the mesh pool id directly
    bool mesh_found = false;
    unsigned int mesh_id = 0;
    for (int i = 0; i < m_known_meshes.size(); i++) {
        if (mesh == std::get<0>(m_known_meshes[i])) {
            mesh_found = true;
            mesh_id = std::get<1>(m_known_meshes[i]);
            d_vertices = reinterpret_cast<CUdeviceptr>(m_mesh_pool[mesh_id].vertex_buffer);
            d_indices = reinterpret_cast<CUdeviceptr>(m_mesh_pool[mesh_id].vertex_index_buffer);
            break;
        }
    }

    if (!mesh_found) {
        // make sure chrono mesh is setup as expected
        if (mesh->getIndicesMaterials().size() == 0) {
            mesh->getIndicesMaterials() = std::vector<int>(mesh->getIndicesVertexes().size(), 0);
        }

        // move the chrono data to contiguous data structures to be copied to gpu
        std::vector<uint4> vertex_index_buffer = std::vector<uint4>(mesh->getIndicesVertexes().size());
        std::vector<uint4> normal_index_buffer = std::vector<uint4>(mesh->getIndicesNormals().size());
        std::vector<uint4> uv_index_buffer = std::vector<uint4>(mesh->getIndicesUV().size());
        std::vector<unsigned int> mat_index_buffer;
        std::vector<float4> vertex_buffer = std::vector<float4>(mesh->getCoordsVertices().size());
        std::vector<float4> normal_buffer = std::vector<float4>(mesh->getCoordsNormals().size());
        std::vector<float2> uv_buffer = std::vector<float2>(mesh->getCoordsUV().size());

        // not optional for vertex indices
        for (int i = 0; i < mesh->getIndicesVertexes().size(); i++) {
            vertex_index_buffer[i] = make_uint4((unsigned int)mesh->getIndicesVertexes()[i].x(),  //
                                                (unsigned int)mesh->getIndicesVertexes()[i].y(),  //
                                                (unsigned int)mesh->getIndicesVertexes()[i].z(), 0);
        }
        uint4* d_vertex_index_buffer = {};
        CUDA_ERROR_CHECK(
            cudaMalloc(reinterpret_cast<void**>(&d_vertex_index_buffer), sizeof(uint4) * vertex_index_buffer.size()));
        CUDA_ERROR_CHECK(cudaMemcpy(reinterpret_cast<void*>(d_vertex_index_buffer), vertex_index_buffer.data(),
                                    sizeof(uint4) * vertex_index_buffer.size(), cudaMemcpyHostToDevice));
        m_mesh_buffers_dptrs.push_back(reinterpret_cast<CUdeviceptr>(d_vertex_index_buffer));
        d_indices = reinterpret_cast<CUdeviceptr>(d_vertex_index_buffer);

        uint4* d_normal_index_buffer = {};
        if (normal_index_buffer.size() > 0) {  // optional whether there are normal indices
            for (int i = 0; i < mesh->getIndicesNormals().size(); i++) {
                normal_index_buffer[i] = make_uint4((unsigned int)mesh->getIndicesNormals()[i].x(),  //
                                                    (unsigned int)mesh->getIndicesNormals()[i].y(),  //
                                                    (unsigned int)mesh->getIndicesNormals()[i].z(), 0);
            }
            CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_normal_index_buffer),
                                        sizeof(uint4) * normal_index_buffer.size()));
            CUDA_ERROR_CHECK(cudaMemcpy(reinterpret_cast<void*>(d_normal_index_buffer), normal_index_buffer.data(),
                                        sizeof(uint4) * normal_index_buffer.size(), cudaMemcpyHostToDevice));
            m_mesh_buffers_dptrs.push_back(reinterpret_cast<CUdeviceptr>(d_normal_index_buffer));
        }

        uint4* d_uv_index_buffer = {};
        if (uv_index_buffer.size() > 0) {  // optional whether there are uv indices
            for (int i = 0; i < mesh->getIndicesUV().size(); i++) {
                uv_index_buffer[i] = make_uint4((unsigned int)mesh->getIndicesUV()[i].x(),  //
                                                (unsigned int)mesh->getIndicesUV()[i].y(),  //
                                                (unsigned int)mesh->getIndicesUV()[i].z(), 0);
            }
            CUDA_ERROR_CHECK(
                cudaMalloc(reinterpret_cast<void**>(&d_uv_index_buffer), sizeof(uint4) * uv_index_buffer.size()));
            CUDA_ERROR_CHECK(cudaMemcpy(reinterpret_cast<void*>(d_uv_index_buffer), uv_index_buffer.data(),
                                        sizeof(uint4) * uv_index_buffer.size(), cudaMemcpyHostToDevice));
            m_mesh_buffers_dptrs.push_back(reinterpret_cast<CUdeviceptr>(d_uv_index_buffer));
        }

        unsigned int* d_mat_index_buffer = {};
        if (mesh->getIndicesMaterials().size() > 0) {  // optional whether there are material indices
            std::copy(mesh->getIndicesMaterials().begin(), mesh->getIndicesMaterials().end(),
                      std::back_inserter(mat_index_buffer));

            CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_mat_index_buffer),
                                        sizeof(unsigned int) * mat_index_buffer.size()));
            CUDA_ERROR_CHECK(cudaMemcpy(reinterpret_cast<void*>(d_mat_index_buffer), mat_index_buffer.data(),
                                        sizeof(unsigned int) * mat_index_buffer.size(), cudaMemcpyHostToDevice));
            m_mesh_buffers_dptrs.push_back(reinterpret_cast<CUdeviceptr>(d_mat_index_buffer));
        }

        // there have to be some vertices for this to be a mesh (not optional)
        for (int i = 0; i < mesh->getCoordsVertices().size(); i++) {
            vertex_buffer[i] = make_float4((float)mesh->getCoordsVertices()[i].x(),  //
                                           (float)mesh->getCoordsVertices()[i].y(),  //
                                           (float)mesh->getCoordsVertices()[i].z(), 0.f);
        }
        float4* d_vertex_buffer = {};
        CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_vertex_buffer), sizeof(float4) * vertex_buffer.size()));
        CUDA_ERROR_CHECK(cudaMemcpy(reinterpret_cast<void*>(d_vertex_buffer), vertex_buffer.data(),
                                    sizeof(float4) * vertex_buffer.size(), cudaMemcpyHostToDevice));
        m_mesh_buffers_dptrs.push_back(reinterpret_cast<CUdeviceptr>(d_vertex_buffer));
        d_vertices =
            reinterpret_cast<CUdeviceptr>(d_vertex_buffer);  // we will reuse this for constructing the geometry

        float4* d_normal_buffer = {};
        if (normal_buffer.size() > 0) {  // optional for there to be vertex normals
            for (int i = 0; i < mesh->getCoordsNormals().size(); i++) {
                normal_buffer[i] = make_float4((float)mesh->getCoordsNormals()[i].x(),  //
                                               (float)mesh->getCoordsNormals()[i].y(),  //
                                               (float)mesh->getCoordsNormals()[i].z(), 0.f);
            }
            CUDA_ERROR_CHECK(
                cudaMalloc(reinterpret_cast<void**>(&d_normal_buffer), sizeof(float4) * normal_buffer.size()));
            CUDA_ERROR_CHECK(cudaMemcpy(reinterpret_cast<void*>(d_normal_buffer), normal_buffer.data(),
                                        sizeof(float4) * normal_buffer.size(), cudaMemcpyHostToDevice));
            m_mesh_buffers_dptrs.push_back(reinterpret_cast<CUdeviceptr>(d_normal_buffer));
        }
        float2* d_uv_buffer = {};
        if (uv_buffer.size() > 0) {  // optional for there to be uv coordinates
            for (int i = 0; i < mesh->getCoordsUV().size(); i++) {
                uv_buffer[i] = make_float2((float)mesh->getCoordsUV()[i].x(),  //
                                           (float)mesh->getCoordsUV()[i].y());
            }
            CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_uv_buffer), sizeof(float2) * uv_buffer.size()));
            CUDA_ERROR_CHECK(cudaMemcpy(reinterpret_cast<void*>(d_uv_buffer), uv_buffer.data(),
                                        sizeof(float2) * uv_buffer.size(), cudaMemcpyHostToDevice));
            m_mesh_buffers_dptrs.push_back(reinterpret_cast<CUdeviceptr>(d_uv_buffer));
        }

        // pack the material record
        MeshParameters mesh_data;
        mesh_data.vertex_buffer = d_vertex_buffer;
        mesh_data.normal_buffer = d_normal_buffer;
        mesh_data.uv_buffer = d_uv_buffer;
        mesh_data.vertex_index_buffer = d_vertex_index_buffer;
        mesh_data.normal_index_buffer = d_normal_index_buffer;
        mesh_data.uv_index_buffer = d_uv_index_buffer;
        mesh_data.mat_index_buffer = d_mat_index_buffer;

        // special case where SCM has uv coordinates, but no uv indices
        // TODO: can we make these changes in SCM directly?
        if (uv_index_buffer.size() == 0 && uv_buffer.size() > 0) {
            mesh_data.uv_index_buffer = d_vertex_index_buffer;
        }

        m_mesh_pool.push_back(mesh_data);

        mesh_id = static_cast<unsigned int>(m_mesh_pool.size() - 1);

        // push this mesh to our known meshes
        m_known_meshes.push_back(std::make_tuple(mesh, mesh_id));
    }

    unsigned int material_id;
    if (mat_list.size() > 0) {
        material_id = GetMaterial(mat_list[0]);
        for (int i = 1; i < mat_list.size(); i++) {
            GetMaterial(mat_list[i]);
        }
    } else {
        material_id = GetMaterial();
    }
    // record when hit by any ray type
    Record<MaterialRecordParameters> mat_record;
    OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_hit_mesh_group, &mat_record));
    mat_record.data.material_pool_id = material_id;
    // assume that if meshes have weight map, materials should be blended
    // if materials should be blended, all materials will have weight map
    mat_record.data.num_blended_materials = 1;
    if (mat_list.size() > 0 && mat_list[0]->GetWeightTexture() != "") {
        mat_record.data.num_blended_materials = (unsigned int)mat_list.size();
    } else {
        mat_record.data.num_blended_materials = 1;
    }

    mat_record.data.mesh_pool_id = mesh_id;
    m_material_records.push_back(mat_record);

    return static_cast<unsigned int>(m_material_records.size() - 1);
}

unsigned int ChOptixPipeline::GetDeformableMeshMaterial(CUdeviceptr& d_vertices,
                                                        CUdeviceptr& d_indices,
                                                        std::shared_ptr<ChVisualShapeTriangleMesh> mesh_shape,
                                                        std::vector<std::shared_ptr<ChVisualMaterial>> mat_list) {
    unsigned int mat_id = GetRigidMeshMaterial(d_vertices, d_indices, mesh_shape, mat_list);

    unsigned int mesh_id = m_material_records[mat_id].data.mesh_pool_id;
    CUdeviceptr d_normals = reinterpret_cast<CUdeviceptr>(m_mesh_pool[mesh_id].normal_buffer);
    unsigned int num_triangles = static_cast<unsigned int>(mesh_shape->GetMesh()->getIndicesVertexes().size());
    m_deformable_meshes.push_back(std::make_tuple(mesh_shape, d_vertices, d_normals, num_triangles));

    return mat_id;
}

void ChOptixPipeline::UpdateDeformableMeshes() {
    for (int i = 0; i < m_deformable_meshes.size(); i++) {
        std::shared_ptr<ChVisualShapeTriangleMesh> mesh_shape = std::get<0>(m_deformable_meshes[i]);
        CUdeviceptr d_vertices = std::get<1>(m_deformable_meshes[i]);
        CUdeviceptr d_normals = std::get<2>(m_deformable_meshes[i]);
        unsigned int num_prev_triangles = std::get<3>(m_deformable_meshes[i]);

        auto mesh = mesh_shape->GetMesh();

        // if the mesh has changed size, we need to recreate the entire mesh (not very nice)
        if (num_prev_triangles != mesh_shape->GetMesh()->getIndicesVertexes().size()) {
            throw std::runtime_error("Error: changing mesh size not supported by Chrono::Sensor");
        }

        // update all the vertex locations

        std::vector<float4> vertex_buffer = std::vector<float4>(mesh->getCoordsVertices().size());
        for (int j = 0; j < mesh->getCoordsVertices().size(); j++) {
            vertex_buffer[j] = make_float4((float)mesh->getCoordsVertices()[j].x(),  //
                                           (float)mesh->getCoordsVertices()[j].y(),  //
                                           (float)mesh->getCoordsVertices()[j].z(),  //
                                           0.f);                                     // padding for alignment
        }
        CUDA_ERROR_CHECK(cudaMemcpy(reinterpret_cast<void*>(d_vertices), vertex_buffer.data(),
                                    sizeof(float4) * vertex_buffer.size(), cudaMemcpyHostToDevice));

        // update all the normals if normal exist
        if (mesh_shape->GetMesh()->getCoordsNormals().size() > 0) {
            std::vector<float4> normal_buffer = std::vector<float4>(mesh->getCoordsNormals().size());
            for (int j = 0; j < mesh->getCoordsNormals().size(); j++) {
                normal_buffer[j] = make_float4((float)mesh->getCoordsNormals()[j].x(),  //
                                               (float)mesh->getCoordsNormals()[j].y(),  //
                                               (float)mesh->getCoordsNormals()[j].z(),  //
                                               0.f);                                    // padding for alignment
            }
            CUDA_ERROR_CHECK(cudaMemcpy(reinterpret_cast<void*>(d_normals), normal_buffer.data(),
                                        sizeof(float4) * normal_buffer.size(), cudaMemcpyHostToDevice));
        }

        // TODO: for SCM terrain, make use of the list of modified vertices
    }
}

void ChOptixPipeline::UpdateObjectVelocity() {
    for (int i = 0; i < m_bodies.size(); i++) {
        m_material_records[i].data.translational_velocity = {(float)m_bodies[i]->GetPos_dt().x(),
                                                             (float)m_bodies[i]->GetPos_dt().y(),
                                                             (float)m_bodies[i]->GetPos_dt().z()};
        m_material_records[i].data.angular_velocity = {(float)m_bodies[i]->GetWvel_par().x(),
                                                       (float)m_bodies[i]->GetWvel_par().y(),
                                                       (float)m_bodies[i]->GetWvel_par().z()};
        m_material_records[i].data.objectId = i;
    }
    CUDA_ERROR_CHECK(cudaMemcpy(reinterpret_cast<void*>(md_material_records), m_material_records.data(),
                                sizeof(Record<MaterialRecordParameters>) * m_material_records.size(),
                                cudaMemcpyHostToDevice));
}

void ChOptixPipeline::CreateDeviceTexture(cudaTextureObject_t& d_tex_sampler,
                                          cudaArray_t& d_img_array,
                                          std::string file_name,
                                          bool mirror,
                                          bool exclude_from_material_cleanup) {
    if (!filesystem::path(file_name).exists()) {
        throw std::runtime_error("Error, file not found: " + file_name);
    }

    // if we have already loaded this texture, instance the texture
    if (m_texture_samplers.find(file_name) != m_texture_samplers.end() &&
        m_img_textures.find(file_name) != m_img_textures.end()) {
        d_tex_sampler = m_texture_samplers[file_name];
        d_img_array = m_img_textures[file_name];
        return;
    }

    ByteImageData img = LoadByteImage(file_name);

    // if image is not 4 channels, make it so
    std::vector<unsigned char> img_data;
    int32_t pitch;
    cudaChannelFormatDesc channel_desc;

    if (img.c == 4) {
        // need to flip the image to match typical texturing
        img_data = std::vector<unsigned char>(img.h * img.w * 4);
        for (int i = 0; i < img.h; i++) {
            for (int j = 0; j < img.w; j++) {
                img_data[i * img.w * 4 + j * 4 + 0] = img.data[(img.h - i - 1) * img.w * 4 + j * 4 + 0];
                img_data[i * img.w * 4 + j * 4 + 1] = img.data[(img.h - i - 1) * img.w * 4 + j * 4 + 1];
                img_data[i * img.w * 4 + j * 4 + 2] = img.data[(img.h - i - 1) * img.w * 4 + j * 4 + 2];
                img_data[i * img.w * 4 + j * 4 + 3] = img.data[(img.h - i - 1) * img.w * 4 + j * 4 + 3];
            }
        }
        channel_desc = cudaCreateChannelDesc<uchar4>();
        pitch = img.w * 4 * sizeof(unsigned char);
    } else if (img.c == 3) {
        img_data = std::vector<unsigned char>(img.h * img.w * 4);
        for (int i = 0; i < img.h; i++) {
            for (int j = 0; j < img.w; j++) {
                img_data[i * img.w * 4 + j * 4 + 0] = img.data[(img.h - i - 1) * img.w * 3 + j * 3 + 0];
                img_data[i * img.w * 4 + j * 4 + 1] = img.data[(img.h - i - 1) * img.w * 3 + j * 3 + 1];
                img_data[i * img.w * 4 + j * 4 + 2] = img.data[(img.h - i - 1) * img.w * 3 + j * 3 + 2];
                img_data[i * img.w * 4 + j * 4 + 3] = 255;
            }
        }
        channel_desc = cudaCreateChannelDesc<uchar4>();
        pitch = img.w * 4 * sizeof(unsigned char);
    } else if (img.c == 2) {
        img_data = std::vector<unsigned char>(img.h * img.w * 4);
        for (int i = 0; i < img.h; i++) {
            for (int j = 0; j < img.w; j++) {
                img_data[i * img.w * 4 + j * 4 + 0] = img.data[(img.h - i - 1) * img.w * 2 + j * 2 + 0];
                img_data[i * img.w * 4 + j * 4 + 1] = img.data[(img.h - i - 1) * img.w * 2 + j * 2 + 0];
                img_data[i * img.w * 4 + j * 4 + 2] = img.data[(img.h - i - 1) * img.w * 2 + j * 2 + 0];
                img_data[i * img.w * 4 + j * 4 + 3] = img.data[(img.h - i - 1) * img.w * 2 + j * 2 + 1];
            }
        }
        channel_desc = cudaCreateChannelDesc<uchar4>();
        pitch = img.w * 4 * sizeof(unsigned char);
    } else if (img.c == 1) {
        img_data = std::vector<unsigned char>(img.h * img.w);
        for (int i = 0; i < img.h; i++) {
            for (int j = 0; j < img.w; j++) {
                img_data[i * img.w + j] = img.data[(img.h - i - 1) * img.w + j];
            }
        }
        channel_desc = cudaCreateChannelDesc<unsigned char>();
        pitch = img.w * sizeof(unsigned char);
    } else {
        throw std::runtime_error("Error: invalid img channel size=" + std::to_string(img.c));
    }

    CUDA_ERROR_CHECK(cudaMallocArray(&d_img_array, &channel_desc, img.w, img.h));
    CUDA_ERROR_CHECK(
        cudaMemcpy2DToArray(d_img_array, 0, 0, img_data.data(), pitch, pitch, img.h, cudaMemcpyHostToDevice));
    // m_img_textures.push_back(d_img_array);

    cudaResourceDesc resource_description = {};
    resource_description.resType = cudaResourceTypeArray;
    resource_description.res.array.array = d_img_array;

    cudaTextureDesc texture_description = {};
    texture_description.addressMode[0] = mirror ? cudaAddressModeMirror : cudaAddressModeWrap;
    texture_description.addressMode[1] = mirror ? cudaAddressModeMirror : cudaAddressModeWrap;
    texture_description.addressMode[2] = mirror ? cudaAddressModeMirror : cudaAddressModeWrap;
    texture_description.filterMode = cudaFilterModeLinear;
    // texture_description.filterMode = cudaFilterModePoint;
    texture_description.readMode = cudaReadModeNormalizedFloat;
    texture_description.normalizedCoords = 1;
    texture_description.maxAnisotropy = 1;
    texture_description.maxMipmapLevelClamp = 99;
    texture_description.minMipmapLevelClamp = 0;
    texture_description.mipmapFilterMode = cudaFilterModePoint;
    // texture_description.mipmapFilterMode = cudaFilterModeLinear;
    // texture_description.borderColor[0] = 1.0f;
    texture_description.sRGB = 0;

    // Create texture sampler
    CUDA_ERROR_CHECK(cudaCreateTextureObject(&d_tex_sampler, &resource_description, &texture_description, nullptr));

    // push this to the vectors for cleanup unless specifically specified not to (background image)
    if (!exclude_from_material_cleanup) {
        // push the sampler and the image to places they can get destroyed later
        m_texture_samplers[file_name] = d_tex_sampler;
        m_img_textures[file_name] = d_img_array;
    }
}

}  // namespace sensor
}  // namespace chrono
