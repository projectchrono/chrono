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
    if (m_debug) {
        m_pipeline_compile_options = {
            true,                                    // use motion blur
            OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_ANY,  // traversableGraphFlags
            2,                                       // all ray gens should pack data into a pointer
            8,                                       // geometry uses 8 attributes
            OPTIX_EXCEPTION_FLAG_DEBUG,              // exceptionFlags
            "params",                                // pipelineLaunchParamsVariableName
            0                                        // use custom primitives and triangles
        };
    } else {
        m_pipeline_compile_options = {
            true,                                    // use motion blur
            OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_ANY,  // traversableGraphFlags
            2,                                       // all ray gens should pack data into a pointer
            8,                                       // geometry uses 8 attributes
            OPTIX_EXCEPTION_FLAG_NONE,               // exceptionFlags
            "params",                                // pipelineLaunchParamsVariableName
            0                                        // use custom primitives and triangles
        };
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
    if (m_camera_shading_module) {
        OPTIX_ERROR_CHECK(optixModuleDestroy(m_camera_shading_module));
        m_camera_shading_module = 0;
    }
    if (m_shadow_shading_module) {
        OPTIX_ERROR_CHECK(optixModuleDestroy(m_shadow_shading_module));
        m_shadow_shading_module = 0;
    }
    if (m_lidar_shading_module) {
        OPTIX_ERROR_CHECK(optixModuleDestroy(m_lidar_shading_module));
        m_lidar_shading_module = 0;
    }
    if (m_radar_shading_module) {
        OPTIX_ERROR_CHECK(optixModuleDestroy(m_radar_shading_module));
        m_radar_shading_module = 0;
    }
    if (m_miss_module) {
        OPTIX_ERROR_CHECK(optixModuleDestroy(m_miss_module));
        m_miss_module = 0;
    }
    // === optix program groups ===
    // raygen groups
    if (m_camera_pinhole_raygen_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_camera_pinhole_raygen_group));
        m_camera_pinhole_raygen_group = 0;
    }
    if (m_camera_fov_lens_raygen_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_camera_fov_lens_raygen_group));
        m_camera_fov_lens_raygen_group = 0;
    }
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
    if (m_camera_miss_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_camera_miss_group));
        m_camera_miss_group = 0;
    }
    if (m_shadow_miss_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_shadow_miss_group));
        m_shadow_miss_group = 0;
    }
    if (m_lidar_miss_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_lidar_miss_group));
        m_lidar_miss_group = 0;
    }
    if (m_radar_miss_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_radar_miss_group));
        m_radar_miss_group = 0;
    }

    // hit groups
    if (m_camera_hit_box_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_camera_hit_box_group));
        m_camera_hit_box_group = 0;
    }
    if (m_shadow_hit_box_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_shadow_hit_box_group));
        m_shadow_hit_box_group = 0;
    }
    if (m_lidar_hit_box_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_lidar_hit_box_group));
        m_lidar_hit_box_group = 0;
    }
    if (m_radar_hit_box_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_radar_hit_box_group));
        m_radar_hit_box_group = 0;
    }
    if (m_camera_hit_sphere_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_camera_hit_sphere_group));
        m_camera_hit_sphere_group = 0;
    }
    if (m_shadow_hit_sphere_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_shadow_hit_sphere_group));
        m_shadow_hit_sphere_group = 0;
    }
    if (m_lidar_hit_sphere_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_lidar_hit_sphere_group));
        m_lidar_hit_sphere_group = 0;
    }
    if (m_radar_hit_sphere_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_radar_hit_sphere_group));
        m_radar_hit_sphere_group = 0;
    }
    if (m_camera_hit_cyl_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_camera_hit_cyl_group));
        m_camera_hit_cyl_group = 0;
    }
    if (m_shadow_hit_cyl_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_shadow_hit_cyl_group));
        m_shadow_hit_cyl_group = 0;
    }
    if (m_lidar_hit_cyl_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_lidar_hit_cyl_group));
        m_lidar_hit_cyl_group = 0;
    }
    if (m_radar_hit_cyl_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_radar_hit_cyl_group));
        m_radar_hit_cyl_group = 0;
    }
    if (m_camera_hit_mesh_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_camera_hit_mesh_group));
        m_camera_hit_mesh_group = 0;
    }
    if (m_shadow_hit_mesh_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_shadow_hit_mesh_group));
        m_shadow_hit_mesh_group = 0;
    }
    if (m_lidar_hit_mesh_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_lidar_hit_mesh_group));
        m_lidar_hit_mesh_group = 0;
    }
    if (m_radar_hit_mesh_group) {
        OPTIX_ERROR_CHECK(optixProgramGroupDestroy(m_radar_hit_mesh_group));
        m_radar_hit_mesh_group = 0;
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

    // for (int i = 0; i < m_img_textures.size(); i++) {
    //     if (m_img_textures[i]) {
    //         CUDA_ERROR_CHECK(cudaFreeArray(m_img_textures[i]));
    //     }
    // }
    // m_img_textures.clear();

    // clear out and free images on the device
    // for (int i = 0; i < m_texture_samplers.size(); i++) {
    //     CUDA_ERROR_CHECK(cudaDestroyTextureObject(m_texture_samplers[i]));
    // }
    // m_texture_samplers.clear();

    // reset instanced record parameters
    m_default_box_record_inst = false;
    m_default_box_record_id = 0;
    m_default_sphere_record_inst = false;
    m_default_sphere_record_id = 0;
    m_default_cyl_record_inst = false;
    m_default_cyl_record_id = 0;

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
    GetShaderFromFile(m_context, m_box_intersection_module, "box", module_compile_options, m_pipeline_compile_options);
    GetShaderFromFile(m_context, m_sphere_intersection_module, "sphere", module_compile_options,
                      m_pipeline_compile_options);
    GetShaderFromFile(m_context, m_cyl_intersection_module, "cylinder", module_compile_options,
                      m_pipeline_compile_options);
    GetShaderFromFile(m_context, m_camera_shading_module, "camera_shaders", module_compile_options,
                      m_pipeline_compile_options);
    GetShaderFromFile(m_context, m_lidar_shading_module, "lidar_shaders", module_compile_options,
                      m_pipeline_compile_options);
    GetShaderFromFile(m_context, m_radar_shading_module, "radar_shaders", module_compile_options,
                      m_pipeline_compile_options);
    GetShaderFromFile(m_context, m_shadow_shading_module, "shadow_shaders", module_compile_options,
                      m_pipeline_compile_options);
    GetShaderFromFile(m_context, m_camera_raygen_module, "camera", module_compile_options, m_pipeline_compile_options);
    GetShaderFromFile(m_context, m_lidar_raygen_module, "lidar", module_compile_options, m_pipeline_compile_options);
    GetShaderFromFile(m_context, m_radar_raygen_module, "radar", module_compile_options, m_pipeline_compile_options);
    GetShaderFromFile(m_context, m_miss_module, "miss", module_compile_options, m_pipeline_compile_options);
    // GetShaderFromFile(m_context, m_exception_module, "exception", module_compile_options,
}
void ChOptixPipeline::AssembleBaseProgramGroups() {
    char log[2048];
    size_t sizeof_log = sizeof(log);

    // camera-hitting-box program group
    OptixProgramGroupOptions camera_hit_box_group_options = {};
    OptixProgramGroupDesc camera_hit_box_group_desc = {};
    camera_hit_box_group_desc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    camera_hit_box_group_desc.hitgroup.moduleIS = m_box_intersection_module;
    camera_hit_box_group_desc.hitgroup.entryFunctionNameIS = "__intersection__box_intersect";
    camera_hit_box_group_desc.hitgroup.moduleCH = m_camera_shading_module;
    camera_hit_box_group_desc.hitgroup.entryFunctionNameCH = "__closesthit__camera_shader";
    OPTIX_ERROR_CHECK(optixProgramGroupCreate(m_context, &camera_hit_box_group_desc, 1, &camera_hit_box_group_options,
                                              log, &sizeof_log, &m_camera_hit_box_group));

    // shadow-hitting-box program group
    OptixProgramGroupOptions shadow_hit_box_group_options = {};
    OptixProgramGroupDesc shadow_hit_box_group_desc = {};
    shadow_hit_box_group_desc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    shadow_hit_box_group_desc.hitgroup.moduleIS = m_box_intersection_module;
    shadow_hit_box_group_desc.hitgroup.entryFunctionNameIS = "__intersection__box_intersect";
    shadow_hit_box_group_desc.hitgroup.moduleCH = m_shadow_shading_module;
    shadow_hit_box_group_desc.hitgroup.entryFunctionNameCH = "__closesthit__shadow_shader";
    OPTIX_ERROR_CHECK(optixProgramGroupCreate(m_context, &shadow_hit_box_group_desc, 1, &shadow_hit_box_group_options,
                                              log, &sizeof_log, &m_shadow_hit_box_group));

    // lidar-hitting-box program group
    OptixProgramGroupOptions lidar_hit_box_group_options = {};
    OptixProgramGroupDesc lidar_hit_box_group_desc = {};
    lidar_hit_box_group_desc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    lidar_hit_box_group_desc.hitgroup.moduleIS = m_box_intersection_module;
    lidar_hit_box_group_desc.hitgroup.entryFunctionNameIS = "__intersection__box_intersect";
    lidar_hit_box_group_desc.hitgroup.moduleCH = m_lidar_shading_module;
    lidar_hit_box_group_desc.hitgroup.entryFunctionNameCH = "__closesthit__lidar_shader";
    OPTIX_ERROR_CHECK(optixProgramGroupCreate(m_context, &lidar_hit_box_group_desc, 1, &lidar_hit_box_group_options,
                                              log, &sizeof_log, &m_lidar_hit_box_group));

    // radar-hitting-box program group
    OptixProgramGroupOptions radar_hit_box_group_options = {};
    OptixProgramGroupDesc radar_hit_box_group_desc = {};
    radar_hit_box_group_desc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    radar_hit_box_group_desc.hitgroup.moduleIS = m_box_intersection_module;
    radar_hit_box_group_desc.hitgroup.entryFunctionNameIS = "__intersection__box_intersect";
    radar_hit_box_group_desc.hitgroup.moduleCH = m_radar_shading_module;
    radar_hit_box_group_desc.hitgroup.entryFunctionNameCH = "__closesthit__radar_shader";
    OPTIX_ERROR_CHECK(optixProgramGroupCreate(m_context, &radar_hit_box_group_desc, 1, &radar_hit_box_group_options,
                                              log, &sizeof_log, &m_radar_hit_box_group));

    // camera-hitting-sphere program group
    OptixProgramGroupOptions camera_hit_sphere_group_options = {};
    OptixProgramGroupDesc camera_hit_sphere_group_desc = {};
    camera_hit_sphere_group_desc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    camera_hit_sphere_group_desc.hitgroup.moduleIS = m_sphere_intersection_module;
    camera_hit_sphere_group_desc.hitgroup.entryFunctionNameIS = "__intersection__sphere_intersect";
    camera_hit_sphere_group_desc.hitgroup.moduleCH = m_camera_shading_module;
    camera_hit_sphere_group_desc.hitgroup.entryFunctionNameCH = "__closesthit__camera_shader";
    OPTIX_ERROR_CHECK(optixProgramGroupCreate(m_context, &camera_hit_sphere_group_desc, 1,
                                              &camera_hit_sphere_group_options, log, &sizeof_log,
                                              &m_camera_hit_sphere_group));

    // shadow-hitting-sphere program group
    OptixProgramGroupOptions shadow_hit_sphere_group_options = {};
    OptixProgramGroupDesc shadow_hit_sphere_group_desc = {};
    shadow_hit_sphere_group_desc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    shadow_hit_sphere_group_desc.hitgroup.moduleIS = m_sphere_intersection_module;
    shadow_hit_sphere_group_desc.hitgroup.entryFunctionNameIS = "__intersection__sphere_intersect";
    shadow_hit_sphere_group_desc.hitgroup.moduleCH = m_shadow_shading_module;
    shadow_hit_sphere_group_desc.hitgroup.entryFunctionNameCH = "__closesthit__shadow_shader";
    OPTIX_ERROR_CHECK(optixProgramGroupCreate(m_context, &shadow_hit_sphere_group_desc, 1,
                                              &shadow_hit_sphere_group_options, log, &sizeof_log,
                                              &m_shadow_hit_sphere_group));

    // lidar-hitting-sphere program group
    OptixProgramGroupOptions lidar_hit_sphere_group_options = {};
    OptixProgramGroupDesc lidar_hit_sphere_group_desc = {};
    lidar_hit_sphere_group_desc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    lidar_hit_sphere_group_desc.hitgroup.moduleIS = m_sphere_intersection_module;
    lidar_hit_sphere_group_desc.hitgroup.entryFunctionNameIS = "__intersection__sphere_intersect";
    lidar_hit_sphere_group_desc.hitgroup.moduleCH = m_lidar_shading_module;
    lidar_hit_sphere_group_desc.hitgroup.entryFunctionNameCH = "__closesthit__lidar_shader";
    OPTIX_ERROR_CHECK(optixProgramGroupCreate(m_context, &lidar_hit_sphere_group_desc, 1,
                                              &lidar_hit_sphere_group_options, log, &sizeof_log,
                                              &m_lidar_hit_sphere_group));
    // radar-hitting-sphere program group
    OptixProgramGroupOptions radar_hit_sphere_group_options = {};
    OptixProgramGroupDesc radar_hit_sphere_group_desc = {};
    radar_hit_sphere_group_desc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    radar_hit_sphere_group_desc.hitgroup.moduleIS = m_sphere_intersection_module;
    radar_hit_sphere_group_desc.hitgroup.entryFunctionNameIS = "__intersection__sphere_intersect";
    radar_hit_sphere_group_desc.hitgroup.moduleCH = m_radar_shading_module;
    radar_hit_sphere_group_desc.hitgroup.entryFunctionNameCH = "__closesthit__radar_shader";
    OPTIX_ERROR_CHECK(optixProgramGroupCreate(m_context, &radar_hit_sphere_group_desc, 1,
                                              &radar_hit_sphere_group_options, log, &sizeof_log,
                                              &m_radar_hit_sphere_group));

    // camera-hitting-cylinder program group
    OptixProgramGroupOptions camera_hit_cyl_group_options = {};
    OptixProgramGroupDesc camera_hit_cyl_group_desc = {};
    camera_hit_cyl_group_desc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    camera_hit_cyl_group_desc.hitgroup.moduleIS = m_cyl_intersection_module;
    camera_hit_cyl_group_desc.hitgroup.entryFunctionNameIS = "__intersection__cylinder_intersect";
    camera_hit_cyl_group_desc.hitgroup.moduleCH = m_camera_shading_module;
    camera_hit_cyl_group_desc.hitgroup.entryFunctionNameCH = "__closesthit__camera_shader";
    OPTIX_ERROR_CHECK(optixProgramGroupCreate(m_context, &camera_hit_cyl_group_desc, 1, &camera_hit_cyl_group_options,
                                              log, &sizeof_log, &m_camera_hit_cyl_group));

    // shadow-hitting-cylinder program group
    OptixProgramGroupOptions shadow_hit_cyl_group_options = {};
    OptixProgramGroupDesc shadow_hit_cyl_group_desc = {};
    shadow_hit_cyl_group_desc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    shadow_hit_cyl_group_desc.hitgroup.moduleIS = m_cyl_intersection_module;
    shadow_hit_cyl_group_desc.hitgroup.entryFunctionNameIS = "__intersection__cylinder_intersect";
    shadow_hit_cyl_group_desc.hitgroup.moduleCH = m_shadow_shading_module;
    shadow_hit_cyl_group_desc.hitgroup.entryFunctionNameCH = "__closesthit__shadow_shader";
    OPTIX_ERROR_CHECK(optixProgramGroupCreate(m_context, &shadow_hit_cyl_group_desc, 1, &shadow_hit_cyl_group_options,
                                              log, &sizeof_log, &m_shadow_hit_cyl_group));

    // lidar-hitting-cylinder program group
    OptixProgramGroupOptions lidar_hit_cyl_group_options = {};
    OptixProgramGroupDesc lidar_hit_cyl_group_desc = {};
    lidar_hit_cyl_group_desc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    lidar_hit_cyl_group_desc.hitgroup.moduleIS = m_cyl_intersection_module;
    lidar_hit_cyl_group_desc.hitgroup.entryFunctionNameIS = "__intersection__cylinder_intersect";
    lidar_hit_cyl_group_desc.hitgroup.moduleCH = m_lidar_shading_module;
    lidar_hit_cyl_group_desc.hitgroup.entryFunctionNameCH = "__closesthit__lidar_shader";
    OPTIX_ERROR_CHECK(optixProgramGroupCreate(m_context, &lidar_hit_cyl_group_desc, 1, &lidar_hit_cyl_group_options,
                                              log, &sizeof_log, &m_lidar_hit_cyl_group));
    // radar-hitting-cylinder program group
    OptixProgramGroupOptions radar_hit_cyl_group_options = {};
    OptixProgramGroupDesc radar_hit_cyl_group_desc = {};
    radar_hit_cyl_group_desc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    radar_hit_cyl_group_desc.hitgroup.moduleIS = m_cyl_intersection_module;
    radar_hit_cyl_group_desc.hitgroup.entryFunctionNameIS = "__intersection__cylinder_intersect";
    radar_hit_cyl_group_desc.hitgroup.moduleCH = m_radar_shading_module;
    radar_hit_cyl_group_desc.hitgroup.entryFunctionNameCH = "__closesthit__radar_shader";
    OPTIX_ERROR_CHECK(optixProgramGroupCreate(m_context, &radar_hit_cyl_group_desc, 1, &radar_hit_cyl_group_options,
                                              log, &sizeof_log, &m_radar_hit_cyl_group));

    // camera-hitting-mesh program group
    OptixProgramGroupOptions camera_hit_mesh_group_options = {};
    OptixProgramGroupDesc camera_hit_mesh_group_desc = {};
    camera_hit_mesh_group_desc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    camera_hit_mesh_group_desc.hitgroup.moduleCH = m_camera_shading_module;
    camera_hit_mesh_group_desc.hitgroup.entryFunctionNameCH = "__closesthit__camera_shader";
    OPTIX_ERROR_CHECK(optixProgramGroupCreate(m_context, &camera_hit_mesh_group_desc, 1, &camera_hit_mesh_group_options,
                                              log, &sizeof_log, &m_camera_hit_mesh_group));

    // shadow-hitting-mesh program group
    OptixProgramGroupOptions shadow_hit_mesh_group_options = {};
    OptixProgramGroupDesc shadow_hit_mesh_group_desc = {};
    shadow_hit_mesh_group_desc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    shadow_hit_mesh_group_desc.hitgroup.moduleCH = m_shadow_shading_module;
    shadow_hit_mesh_group_desc.hitgroup.entryFunctionNameCH = "__closesthit__shadow_shader";
    OPTIX_ERROR_CHECK(optixProgramGroupCreate(m_context, &shadow_hit_mesh_group_desc, 1, &shadow_hit_mesh_group_options,
                                              log, &sizeof_log, &m_shadow_hit_mesh_group));

    // lidar-hitting-mesh program group
    OptixProgramGroupOptions lidar_hit_mesh_group_options = {};
    OptixProgramGroupDesc lidar_hit_mesh_group_desc = {};
    lidar_hit_mesh_group_desc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    lidar_hit_mesh_group_desc.hitgroup.moduleCH = m_lidar_shading_module;
    lidar_hit_mesh_group_desc.hitgroup.entryFunctionNameCH = "__closesthit__lidar_shader";
    OPTIX_ERROR_CHECK(optixProgramGroupCreate(m_context, &lidar_hit_mesh_group_desc, 1, &lidar_hit_mesh_group_options,
                                              log, &sizeof_log, &m_lidar_hit_mesh_group));
    // radar-hitting-mesh program group
    OptixProgramGroupOptions radar_hit_mesh_group_options = {};
    OptixProgramGroupDesc radar_hit_mesh_group_desc = {};
    radar_hit_mesh_group_desc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    radar_hit_mesh_group_desc.hitgroup.moduleCH = m_radar_shading_module;
    radar_hit_mesh_group_desc.hitgroup.entryFunctionNameCH = "__closesthit__radar_shader";
    OPTIX_ERROR_CHECK(optixProgramGroupCreate(m_context, &radar_hit_mesh_group_desc, 1, &radar_hit_mesh_group_options,
                                              log, &sizeof_log, &m_radar_hit_mesh_group));

    // camera miss program group
    OptixProgramGroupOptions miss_group_options = {};
    OptixProgramGroupDesc miss_group_desc = {};
    miss_group_desc.kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
    miss_group_desc.miss.module = m_miss_module;
    miss_group_desc.miss.entryFunctionName = "__miss__camera";
    OPTIX_ERROR_CHECK(optixProgramGroupCreate(m_context, &miss_group_desc, 1, &miss_group_options, log, &sizeof_log,
                                              &m_camera_miss_group));
    // shadown miss group (null)
    miss_group_desc.miss.module = nullptr;
    miss_group_desc.miss.entryFunctionName = nullptr;
    OPTIX_ERROR_CHECK(optixProgramGroupCreate(m_context, &miss_group_desc, 1, &miss_group_options, log, &sizeof_log,
                                              &m_shadow_miss_group));

    // lidar miss group
    miss_group_desc.miss.module = m_miss_module;
    miss_group_desc.miss.entryFunctionName = "__miss__lidar";
    OPTIX_ERROR_CHECK(optixProgramGroupCreate(m_context, &miss_group_desc, 1, &miss_group_options, log, &sizeof_log,
                                              &m_lidar_miss_group));
    // radar miss group
    miss_group_desc.miss.module = m_miss_module;
    miss_group_desc.miss.entryFunctionName = "__miss__radar";
    OPTIX_ERROR_CHECK(optixProgramGroupCreate(m_context, &miss_group_desc, 1, &miss_group_options, log, &sizeof_log,
                                              &m_radar_miss_group));
    // camera pinhole raygen
    OptixProgramGroupOptions camera_pinhole_raygen_group_options = {};
    OptixProgramGroupDesc camera_pinhole_raygen_group_desc = {};
    camera_pinhole_raygen_group_desc.kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
    camera_pinhole_raygen_group_desc.raygen.module = m_camera_raygen_module;
    camera_pinhole_raygen_group_desc.raygen.entryFunctionName = "__raygen__camera_pinhole";
    OPTIX_ERROR_CHECK(optixProgramGroupCreate(m_context, &camera_pinhole_raygen_group_desc, 1,
                                              &camera_pinhole_raygen_group_options, log, &sizeof_log,
                                              &m_camera_pinhole_raygen_group));

    // camera fov lens raygen
    OptixProgramGroupOptions camera_fov_raygen_group_options = {};
    OptixProgramGroupDesc camera_fov_raygen_group_desc = {};
    camera_fov_raygen_group_desc.kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
    camera_fov_raygen_group_desc.raygen.module = m_camera_raygen_module;
    camera_fov_raygen_group_desc.raygen.entryFunctionName = "__raygen__camera_fov_lens";
    OPTIX_ERROR_CHECK(optixProgramGroupCreate(m_context, &camera_fov_raygen_group_desc, 1,
                                              &camera_fov_raygen_group_options, log, &sizeof_log,
                                              &m_camera_fov_lens_raygen_group));

    // lidar single raygen
    OptixProgramGroupOptions lidar_single_raygen_group_options = {};
    OptixProgramGroupDesc lidar_single_raygen_group_desc = {};
    lidar_single_raygen_group_desc.kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
    lidar_single_raygen_group_desc.raygen.module = m_lidar_raygen_module;
    lidar_single_raygen_group_desc.raygen.entryFunctionName = "__raygen__lidar_single";
    OPTIX_ERROR_CHECK(optixProgramGroupCreate(m_context, &lidar_single_raygen_group_desc, 1,
                                              &lidar_single_raygen_group_options, log, &sizeof_log,
                                              &m_lidar_single_raygen_group));
    // lidar multi raygen
    OptixProgramGroupOptions lidar_multi_raygen_group_options = {};
    OptixProgramGroupDesc lidar_multi_raygen_group_desc = {};
    lidar_multi_raygen_group_desc.kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
    lidar_multi_raygen_group_desc.raygen.module = m_lidar_raygen_module;
    lidar_multi_raygen_group_desc.raygen.entryFunctionName = "__raygen__lidar_multi";
    OPTIX_ERROR_CHECK(optixProgramGroupCreate(m_context, &lidar_multi_raygen_group_desc, 1,
                                              &lidar_multi_raygen_group_options, log, &sizeof_log,
                                              &m_lidar_multi_raygen_group));
    // radar raygen
    OptixProgramGroupOptions radar_raygen_group_options = {};
    OptixProgramGroupDesc radar_raygen_group_desc = {};
    radar_raygen_group_desc.kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
    radar_raygen_group_desc.raygen.module = m_radar_raygen_module;
    radar_raygen_group_desc.raygen.entryFunctionName = "__raygen__radar";
    OPTIX_ERROR_CHECK(optixProgramGroupCreate(m_context, &radar_raygen_group_desc, 1, &radar_raygen_group_options, log,
                                              &sizeof_log, &m_radar_raygen_group));
}

void ChOptixPipeline::CreateBaseSBT() {
    // miss record - should only ever need one of these
    CUDA_ERROR_CHECK(
        cudaMalloc(reinterpret_cast<void**>(&md_miss_record), sizeof(Record<MissParameters>) * RAY_TYPE_COUNT));
    Record<MissParameters> miss_rec[RAY_TYPE_COUNT];
    OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_camera_miss_group, &miss_rec[CAMERA_RAY_TYPE]));
    OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_shadow_miss_group, &miss_rec[SHADOW_RAY_TYPE]));
    OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_lidar_miss_group, &miss_rec[LIDAR_RAY_TYPE]));
    OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_radar_miss_group, &miss_rec[RADAR_RAY_TYPE]));

    // camera miss record data
    miss_rec[CAMERA_RAY_TYPE].data.camera_miss.mode = BackgroundMode::GRADIENT;
    miss_rec[CAMERA_RAY_TYPE].data.camera_miss.color_zenith = {0.2f, 0.3f, 0.4f};
    miss_rec[CAMERA_RAY_TYPE].data.camera_miss.color_zenith = {0.7, 0.8f, 0.9f};

    // shadow miss record data (doesn't matter) so we leave it out

    // lidar miss record data
    miss_rec[LIDAR_RAY_TYPE].data.lidar_miss.default_range = 0.f;
    miss_rec[LIDAR_RAY_TYPE].data.lidar_miss.default_intensity = 0.f;

    // lidar miss record data
    miss_rec[RADAR_RAY_TYPE].data.radar_miss.default_range = 0.f;
    miss_rec[RADAR_RAY_TYPE].data.radar_miss.default_rcs = 0.f;

    CUDA_ERROR_CHECK(cudaMemcpy(reinterpret_cast<void*>(md_miss_record), miss_rec,
                                sizeof(Record<MissParameters>) * RAY_TYPE_COUNT, cudaMemcpyHostToDevice));
}

void ChOptixPipeline::UpdateBackground(Background b) {
    // miss record - should only ever need one of these
    size_t sizeof_miss_record = sizeof(Record<MissParameters>);
    Record<MissParameters> miss_rec[RAY_TYPE_COUNT];
    OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_camera_miss_group, &miss_rec[CAMERA_RAY_TYPE]));
    OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_shadow_miss_group, &miss_rec[SHADOW_RAY_TYPE]));
    OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_lidar_miss_group, &miss_rec[LIDAR_RAY_TYPE]));
    OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_radar_miss_group, &miss_rec[RADAR_RAY_TYPE]));

    miss_rec[0].data.camera_miss.mode = b.mode;
    miss_rec[0].data.camera_miss.color_zenith = make_float3(b.color_zenith.x(), b.color_zenith.y(), b.color_zenith.z());
    miss_rec[0].data.camera_miss.color_horizon =
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
        miss_rec[0].data.camera_miss.env_map = md_miss_texture_sampler;
    }

    // shadow miss record data (doesn't matter) so we leave it out

    // lidar miss record data
    miss_rec[2].data.lidar_miss.default_range = 0.f;
    miss_rec[2].data.lidar_miss.default_intensity = 0.f;

    // lidar miss record data
    miss_rec[3].data.radar_miss.default_range = 0.f;
    miss_rec[3].data.radar_miss.default_rcs = 0.f;

    CUDA_ERROR_CHECK(cudaMemcpy(reinterpret_cast<void*>(md_miss_record), miss_rec,
                                sizeof(Record<MissParameters>) * RAY_TYPE_COUNT, cudaMemcpyHostToDevice));
}

void ChOptixPipeline::SpawnPipeline(PipelineType type) {
    // build up the pipeline for this specific sensor
    // always add in this order. Will match the shader binding table that is constructed elsewhere.
    // 1. raygen group
    // 2. hit groups
    // 3. miss groups

    std::vector<OptixProgramGroup> program_groups;

    // create the sbt that corresponds to the pipeline
    // OptixShaderBindingTable b = {};
    auto b = chrono_types::make_shared<OptixShaderBindingTable>();

    // add raygen program group first
    switch (type) {
        case PipelineType::CAMERA_PINHOLE: {
            program_groups.push_back(m_camera_pinhole_raygen_group);
            // ray gen record for a pinhole camera
            CUdeviceptr d_raygen_record;
            CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_raygen_record), sizeof(Record<RaygenParameters>)));
            b->raygenRecord = d_raygen_record;

            auto raygen_record = chrono_types::make_shared<Record<RaygenParameters>>();
            OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_camera_pinhole_raygen_group, raygen_record.get()));
            raygen_record->data.t0 = 0.f;
            raygen_record->data.t1 = 1.f;
            raygen_record->data.pos0 = {0.f, 0.f, 0.f};             // default value
            raygen_record->data.rot0 = {1.f, 0.f, 0.f, 0.f};        // default value
            raygen_record->data.pos1 = {0.f, 0.f, 0.f};             // default value
            raygen_record->data.rot1 = {1.f, 0.f, 0.f, 0.f};        // default value
            raygen_record->data.specific.camera.hFOV = 3.14 / 4.0;  // default value
            raygen_record->data.specific.camera.frame_buffer = {};  // default value
            raygen_record->data.specific.camera.use_gi = false;     // default value
            m_raygen_records.push_back(raygen_record);
            break;
        }

        case PipelineType::CAMERA_FOV_LENS: {
            program_groups.push_back(m_camera_fov_lens_raygen_group);
            // ray gen record for a pinhole camera
            CUdeviceptr d_raygen_record;
            CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_raygen_record), sizeof(Record<RaygenParameters>)));
            b->raygenRecord = d_raygen_record;

            auto raygen_record = chrono_types::make_shared<Record<RaygenParameters>>();
            OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_camera_fov_lens_raygen_group, raygen_record.get()));
            raygen_record->data.t0 = 0.f;
            raygen_record->data.t1 = 1.f;
            raygen_record->data.pos0 = {0.f, 0.f, 0.f};             // default value
            raygen_record->data.rot0 = {1.f, 0.f, 0.f, 0.f};        // default value
            raygen_record->data.pos1 = {0.f, 0.f, 0.f};             // default value
            raygen_record->data.rot1 = {1.f, 0.f, 0.f, 0.f};        // default value
            raygen_record->data.specific.camera.hFOV = 3.14 / 4.0;  // default value
            raygen_record->data.specific.camera.frame_buffer = {};  // default value
            raygen_record->data.specific.camera.use_gi = false;     // default value
            m_raygen_records.push_back(raygen_record);
            break;
        }

        case PipelineType::LIDAR_SINGLE: {
            program_groups.push_back(m_lidar_single_raygen_group);
            // ray gen record for a lidar
            CUdeviceptr d_raygen_record;
            CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_raygen_record), sizeof(Record<RaygenParameters>)));
            b->raygenRecord = d_raygen_record;

            auto raygen_record = chrono_types::make_shared<Record<RaygenParameters>>();
            OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_lidar_single_raygen_group, raygen_record.get()));
            raygen_record->data.t0 = 0.f;                                                 // default value
            raygen_record->data.t1 = 1.f;                                                 // default value
            raygen_record->data.pos0 = {0.f, 0.f, 0.f};                                   // default value
            raygen_record->data.rot0 = {1.f, 0.f, 0.f, 0.f};                              // default value
            raygen_record->data.pos1 = {0.f, 0.f, 0.f};                                   // default value
            raygen_record->data.rot1 = {1.f, 0.f, 0.f, 0.f};                              // default value
            raygen_record->data.specific.lidar.frame_buffer = {};                         // default value
            raygen_record->data.specific.lidar.max_vert_angle = 1.f;                      // default value
            raygen_record->data.specific.lidar.min_vert_angle = -1.f;                     // default value
            raygen_record->data.specific.lidar.hFOV = CH_C_2PI;                           // default value
            raygen_record->data.specific.lidar.beam_shape = LidarBeamShape::RECTANGULAR;  // default value
            raygen_record->data.specific.lidar.sample_radius = 1;                         // default value
            raygen_record->data.specific.lidar.horiz_div_angle = 0.f;                     // default value
            raygen_record->data.specific.lidar.vert_div_angle = 0.f;                      // default value
            raygen_record->data.specific.lidar.max_distance = 200.f;                      // default value
            raygen_record->data.specific.lidar.clip_near = 0.f;                           // default value
            m_raygen_records.push_back(raygen_record);
            break;
        }

        case PipelineType::LIDAR_MULTI: {
            program_groups.push_back(m_lidar_multi_raygen_group);
            // ray gen record for a lidar
            CUdeviceptr d_raygen_record;
            CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_raygen_record), sizeof(Record<RaygenParameters>)));
            b->raygenRecord = d_raygen_record;

            auto raygen_record = chrono_types::make_shared<Record<RaygenParameters>>();
            OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_lidar_multi_raygen_group, raygen_record.get()));
            raygen_record->data.t0 = 0.f;                                                 // default value
            raygen_record->data.t1 = 1.f;                                                 // default value
            raygen_record->data.pos0 = {0.f, 0.f, 0.f};                                   // default value
            raygen_record->data.rot0 = {1.f, 0.f, 0.f, 0.f};                              // default value
            raygen_record->data.pos1 = {0.f, 0.f, 0.f};                                   // default value
            raygen_record->data.rot1 = {1.f, 0.f, 0.f, 0.f};                              // default value
            raygen_record->data.specific.lidar.frame_buffer = {};                         // default value
            raygen_record->data.specific.lidar.max_vert_angle = 1.f;                      // default value
            raygen_record->data.specific.lidar.min_vert_angle = -1.f;                     // default value
            raygen_record->data.specific.lidar.hFOV = CH_C_2PI;                           // default value
            raygen_record->data.specific.lidar.beam_shape = LidarBeamShape::RECTANGULAR;  // default value
            raygen_record->data.specific.lidar.sample_radius = 1;                         // default value
            raygen_record->data.specific.lidar.horiz_div_angle = 0.f;                     // default value
            raygen_record->data.specific.lidar.vert_div_angle = 0.f;                      // default value
            raygen_record->data.specific.lidar.max_distance = 200.f;                      // default value
            raygen_record->data.specific.lidar.clip_near = 0.f;                           // default value
            m_raygen_records.push_back(raygen_record);
            break;
        }

        case PipelineType::RADAR: {
            program_groups.push_back(m_radar_raygen_group);
            // ray gen record for a radar
            CUdeviceptr d_raygen_record;
            CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_raygen_record), sizeof(Record<RaygenParameters>)));
            b->raygenRecord = d_raygen_record;

            auto raygen_record = chrono_types::make_shared<Record<RaygenParameters>>();
            OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_radar_raygen_group, raygen_record.get()));
            raygen_record->data.t0 = 0.f;                              // default value
            raygen_record->data.t1 = 1.f;                              // default value
            raygen_record->data.pos0 = {0.f, 0.f, 0.f};                // default value
            raygen_record->data.rot0 = {1.f, 0.f, 0.f, 0.f};           // default value
            raygen_record->data.pos1 = {0.f, 0.f, 0.f};                // default value
            raygen_record->data.rot1 = {1.f, 0.f, 0.f, 0.f};           // default value
            raygen_record->data.specific.radar.frame_buffer = {};      // default value
            raygen_record->data.specific.radar.max_vert_angle = 1.f;   // default value
            raygen_record->data.specific.radar.min_vert_angle = -1.f;  // default value
            raygen_record->data.specific.radar.hFOV = CH_C_PI;         // default value
            raygen_record->data.specific.radar.max_distance = 200.f;   // default value
            raygen_record->data.specific.radar.clip_near = 0.f;        // default value
            m_raygen_records.push_back(raygen_record);
            break;
        }
        default:
            throw ChException("Unsupported pipeline type: unknown type");
    }

    // TOOD: make this a list that only gets expanded with the geometry that requires the module is added
    // we will have a base group with is dependent on what is in the scene, otherwise each pipeline will
    // need all the possible hit groups in it

    // exception group for debugging
    // program_groups.push_back(m_exception_group);

    // ray vs box groups
    program_groups.push_back(m_camera_hit_box_group);
    program_groups.push_back(m_shadow_hit_box_group);
    program_groups.push_back(m_lidar_hit_box_group);
    program_groups.push_back(m_radar_hit_box_group);

    // ray vs sphere groups
    program_groups.push_back(m_camera_hit_sphere_group);
    program_groups.push_back(m_shadow_hit_sphere_group);
    program_groups.push_back(m_lidar_hit_sphere_group);
    program_groups.push_back(m_radar_hit_sphere_group);

    // ray vs cylinder groups
    program_groups.push_back(m_camera_hit_cyl_group);
    program_groups.push_back(m_shadow_hit_cyl_group);
    program_groups.push_back(m_lidar_hit_cyl_group);
    program_groups.push_back(m_radar_hit_cyl_group);

    // ray vs mesh groups
    program_groups.push_back(m_camera_hit_mesh_group);
    program_groups.push_back(m_shadow_hit_mesh_group);
    program_groups.push_back(m_lidar_hit_mesh_group);
    program_groups.push_back(m_radar_hit_mesh_group);

    program_groups.push_back(m_camera_miss_group);
    program_groups.push_back(m_shadow_miss_group);
    program_groups.push_back(m_lidar_miss_group);
    program_groups.push_back(m_radar_miss_group);

    OptixPipelineLinkOptions pipeline_link_options = {m_trace_depth, OPTIX_COMPILE_DEBUG_LEVEL_FULL};

    char log[2048];
    size_t sizeof_log = sizeof(log);
    // OptixPipeline pipeline;
    m_pipelines.emplace_back();
    const int id = m_pipelines.size() - 1;
    OPTIX_ERROR_CHECK(optixPipelineCreate(m_context, &m_pipeline_compile_options, &pipeline_link_options,
                                          program_groups.data(), static_cast<unsigned int>(program_groups.size()), log,
                                          &sizeof_log, &m_pipelines[id]));
    OptixStackSizes stack_sizes = {};
    for (auto& prog_group : program_groups) {
        OPTIX_ERROR_CHECK(optixUtilAccumulateStackSizes(prog_group, &stack_sizes));
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
    b->missRecordCount = RAY_TYPE_COUNT;
    b->missRecordStrideInBytes = static_cast<uint32_t>(sizeof(Record<MissParameters>));

    // set the shader program record - same for all pipelines
    b->hitgroupRecordBase = md_material_records;
    b->hitgroupRecordCount = m_material_records.size();  // we are pushing one back for each ray type of each material
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
        m_sbts[b]->hitgroupRecordCount = m_material_records.size();  // we are pushing one back for each ray type of
                                                                     // each material
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
        material.Kd = {mat->GetDiffuseColor().x(), mat->GetDiffuseColor().y(), mat->GetDiffuseColor().z()};
        material.Ks = {mat->GetSpecularColor().x(), mat->GetSpecularColor().y(), mat->GetSpecularColor().z()};
        material.fresnel_exp = mat->GetFresnelExp();
        material.fresnel_min = mat->GetFresnelMin();
        material.fresnel_max = mat->GetFresnelMax();
        material.transparency = mat->GetTransparency();
        material.roughness = mat->GetRoughness();
        material.metallic = mat->GetMetallic();
        material.lidar_intensity = 1.f;    // TODO: allow setting of this in the visual material chrono-side
        material.radar_backscatter = 1.f;  // TODO: allow setting of this in the visual material chrono-side

        // diffuse texture
        if (mat->GetKdTexture() != "") {
            cudaTextureObject_t d_tex_sampler;
            cudaArray_t d_img_array;
            CreateDeviceTexture(d_tex_sampler, d_img_array, mat->GetKdTexture());
            material.kd_tex = d_tex_sampler;
        } else {
            material.kd_tex = 0;  // explicitely null
        }

        // normal texture
        if (mat->GetNormalMapTexture() != "") {
            cudaTextureObject_t d_tex_sampler;
            cudaArray_t d_img_array;
            CreateDeviceTexture(d_tex_sampler, d_img_array, mat->GetNormalMapTexture());
            material.kn_tex = d_tex_sampler;
        } else {
            material.kn_tex = 0;  // explicitely null
        }

        // metalic texture
        if (mat->GetMetallicTexture() != "") {
            cudaTextureObject_t d_tex_sampler;
            cudaArray_t d_img_array;
            CreateDeviceTexture(d_tex_sampler, d_img_array, mat->GetMetallicTexture());
            material.metallic_tex = d_tex_sampler;
        } else {
            material.metallic_tex = 0;  // explicitely null
        }

        // roughness texture
        if (mat->GetRoughnessTexture() != "") {
            cudaTextureObject_t d_tex_sampler;
            cudaArray_t d_img_array;
            CreateDeviceTexture(d_tex_sampler, d_img_array, mat->GetRoughnessTexture());
            material.roughness_tex = d_tex_sampler;
        } else {
            material.roughness_tex = 0;  // explicitely null
        }

        // opacity texture
        if (mat->GetOpacityTexture() != "") {
            cudaTextureObject_t d_tex_sampler;
            cudaArray_t d_img_array;
            CreateDeviceTexture(d_tex_sampler, d_img_array, mat->GetOpacityTexture());
            material.opacity_tex = d_tex_sampler;
        } else {
            material.opacity_tex = 0;  // explicitely null
        }

        m_material_pool.push_back(material);
        return m_material_pool.size() - 1;

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
            material.kn_tex = 0;
            material.roughness_tex = 0;
            material.metallic_tex = 0;
            material.opacity_tex = 0;
            m_material_pool.push_back(material);
            m_default_material_id = m_material_pool.size() - 1;
            m_default_material_inst = true;
        }

        return m_default_material_id;
    }
}

unsigned int ChOptixPipeline::GetBoxMaterial(std::shared_ptr<ChVisualMaterial> mat) {
    if (mat) {
        unsigned int material_id = GetMaterial(mat);

        // record for box when hit by camera
        Record<MaterialRecordParameters> camera_mat_record;
        OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_camera_hit_box_group, &camera_mat_record));
        camera_mat_record.data.material_pool_id = material_id;
        m_material_records.push_back(camera_mat_record);

        // record for box when hit by shadow
        Record<MaterialRecordParameters> shadow_mat_record;
        OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_shadow_hit_box_group, &shadow_mat_record));
        shadow_mat_record.data.material_pool_id = material_id;
        m_material_records.push_back(shadow_mat_record);

        // record for box when hit by lidar
        Record<MaterialRecordParameters> lidar_mat_record;
        OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_lidar_hit_box_group, &lidar_mat_record));
        lidar_mat_record.data.material_pool_id = material_id;
        m_material_records.push_back(lidar_mat_record);

        // record for box when hit by radar
        Record<MaterialRecordParameters> radar_mat_record;
        OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_radar_hit_box_group, &radar_mat_record));
        radar_mat_record.data.material_pool_id = material_id;
        m_material_records.push_back(radar_mat_record);

        return m_material_records.size() / RAY_TYPE_COUNT - 1;
    } else {
        // std::cout << "Getting default box material" << std::endl;
        // material will only be instantiated once
        if (!m_default_box_record_inst) {
            unsigned int material_id = GetMaterial();

            // record when hit by camera
            Record<MaterialRecordParameters> camera_mat_record;
            OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_camera_hit_box_group, &camera_mat_record));
            camera_mat_record.data.material_pool_id = material_id;
            m_material_records.push_back(camera_mat_record);

            // record when hit by shadow
            Record<MaterialRecordParameters> shadow_mat_record;
            OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_shadow_hit_box_group, &shadow_mat_record));
            shadow_mat_record.data.material_pool_id = material_id;
            m_material_records.push_back(shadow_mat_record);

            // record when hit by lidar
            Record<MaterialRecordParameters> lidar_mat_record;
            OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_lidar_hit_box_group, &lidar_mat_record));
            lidar_mat_record.data.material_pool_id = material_id;
            m_material_records.push_back(lidar_mat_record);

            // record when hit by radar
            Record<MaterialRecordParameters> radar_mat_record;
            OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_radar_hit_box_group, &radar_mat_record));
            radar_mat_record.data.material_pool_id = material_id;
            m_material_records.push_back(radar_mat_record);

            // md_material_records.push_back(d_box_records);
            m_default_box_record_inst = true;
            m_default_box_record_id = m_material_records.size() / RAY_TYPE_COUNT - 1;
        }

        // std::cout << "Material list: " << m_material_records.size() << ", material is is " <<
        // m_default_box_material_id
        //           << std::endl;
        return m_default_box_record_id;
    }
}

unsigned int ChOptixPipeline::GetSphereMaterial(std::shared_ptr<ChVisualMaterial> mat) {
    if (mat) {
        unsigned int material_id = GetMaterial(mat);

        // record for box when hit by camera
        Record<MaterialRecordParameters> camera_mat_record;
        OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_camera_hit_sphere_group, &camera_mat_record));
        camera_mat_record.data.material_pool_id = material_id;
        m_material_records.push_back(camera_mat_record);

        // record when hit by shadow
        Record<MaterialRecordParameters> shadow_mat_record;
        OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_shadow_hit_sphere_group, &shadow_mat_record));
        shadow_mat_record.data.material_pool_id = material_id;
        m_material_records.push_back(shadow_mat_record);

        // record when hit by lidar
        Record<MaterialRecordParameters> lidar_mat_record;
        OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_lidar_hit_sphere_group, &lidar_mat_record));
        lidar_mat_record.data.material_pool_id = material_id;
        m_material_records.push_back(lidar_mat_record);

        // record when hit by radar
        Record<MaterialRecordParameters> radar_mat_record;
        OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_radar_hit_sphere_group, &radar_mat_record));
        radar_mat_record.data.material_pool_id = material_id;
        m_material_records.push_back(radar_mat_record);

        return m_material_records.size() / RAY_TYPE_COUNT - 1;
    } else {
        // std::cout << "Getting default sphere material" << std::endl;
        // material will only be instantiated once
        if (!m_default_sphere_record_inst) {
            unsigned int material_id = GetMaterial();

            // record for sphere when hit by camera
            Record<MaterialRecordParameters> camera_mat_record;
            OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_camera_hit_sphere_group, &camera_mat_record));
            camera_mat_record.data.material_pool_id = material_id;
            m_material_records.push_back(camera_mat_record);

            // record when hit by shadow
            Record<MaterialRecordParameters> shadow_mat_record;
            OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_shadow_hit_sphere_group, &shadow_mat_record));
            shadow_mat_record.data.material_pool_id = material_id;
            m_material_records.push_back(shadow_mat_record);

            // record when hit by shadow
            Record<MaterialRecordParameters> lidar_mat_record;
            OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_lidar_hit_sphere_group, &lidar_mat_record));
            lidar_mat_record.data.material_pool_id = material_id;
            m_material_records.push_back(lidar_mat_record);

            // record when hit by radar
            Record<MaterialRecordParameters> radar_mat_record;
            OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_radar_hit_sphere_group, &radar_mat_record));
            radar_mat_record.data.material_pool_id = material_id;
            m_material_records.push_back(radar_mat_record);

            // md_material_records.push_back(d_box_records);
            m_default_sphere_record_inst = true;
            m_default_sphere_record_id = m_material_records.size() / RAY_TYPE_COUNT - 1;
        }

        // std::cout << "Material list: " << m_material_records.size() << ", material is is " <<
        // m_default_box_material_id
        //           << std::endl;
        return m_default_sphere_record_id;
    }
}

unsigned int ChOptixPipeline::GetCylinderMaterial(std::shared_ptr<ChVisualMaterial> mat) {
    if (mat) {
        unsigned int material_id = GetMaterial(mat);

        // material for box when hit by camera
        Record<MaterialRecordParameters> camera_mat_record;
        OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_camera_hit_cyl_group, &camera_mat_record));
        camera_mat_record.data.material_pool_id = material_id;
        m_material_records.push_back(camera_mat_record);

        // material when hit by shadow
        Record<MaterialRecordParameters> shadow_mat_record;
        OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_shadow_hit_cyl_group, &shadow_mat_record));
        shadow_mat_record.data.material_pool_id = material_id;
        m_material_records.push_back(shadow_mat_record);

        // material when hit by lidar
        Record<MaterialRecordParameters> lidar_mat_record;
        OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_lidar_hit_cyl_group, &lidar_mat_record));
        lidar_mat_record.data.material_pool_id = material_id;
        m_material_records.push_back(lidar_mat_record);

        // material when hit by radar
        Record<MaterialRecordParameters> radar_mat_record;
        OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_radar_hit_cyl_group, &radar_mat_record));
        radar_mat_record.data.material_pool_id = material_id;
        m_material_records.push_back(radar_mat_record);

        return m_material_records.size() / RAY_TYPE_COUNT - 1;
    } else {
        // std::cout << "Getting default cylinder material" << std::endl;
        // material will only be instantiated once
        if (!m_default_cyl_record_inst) {
            unsigned int material_id = GetMaterial();

            // record for cylinder when hit by camera
            Record<MaterialRecordParameters> camera_mat_record;
            OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_camera_hit_cyl_group, &camera_mat_record));
            camera_mat_record.data.material_pool_id = material_id;
            m_material_records.push_back(camera_mat_record);

            // record when hit by shadow
            Record<MaterialRecordParameters> shadow_mat_record;
            OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_shadow_hit_cyl_group, &shadow_mat_record));
            shadow_mat_record.data.material_pool_id = material_id;
            m_material_records.push_back(shadow_mat_record);

            // record when hit by lidar
            Record<MaterialRecordParameters> lidar_mat_record;
            OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_lidar_hit_cyl_group, &lidar_mat_record));
            lidar_mat_record.data.material_pool_id = material_id;
            m_material_records.push_back(lidar_mat_record);

            // record when hit by radar
            Record<MaterialRecordParameters> radar_mat_record;
            OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_radar_hit_cyl_group, &radar_mat_record));
            radar_mat_record.data.material_pool_id = material_id;
            m_material_records.push_back(radar_mat_record);

            m_default_cyl_record_inst = true;
            m_default_cyl_record_id = m_material_records.size() / RAY_TYPE_COUNT - 1;
        }
        return m_default_cyl_record_id;
    }
}

// this will actually make a new material (new mesh info), but will apply a default coloring/texture
unsigned int ChOptixPipeline::GetRigidMeshMaterial(CUdeviceptr& d_vertices,
                                                   CUdeviceptr& d_indices,
                                                   std::shared_ptr<ChTriangleMeshShape> mesh_shape,
                                                   std::vector<std::shared_ptr<ChVisualMaterial>> mat_list) {
    auto mesh = mesh_shape->GetMesh();

    // check if this mesh is known, if so, we can just get the mesh pool id directly
    bool mesh_found = false;
    unsigned int mesh_id;
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
        if (mesh->m_face_col_indices.size() == 0) {
            mesh->m_face_col_indices = std::vector<ChVector<int>>(mesh->getIndicesVertexes().size());
        }

        // move the chrono data to contiguous data structures to be copied to gpu
        std::vector<uint4> vertex_index_buffer = std::vector<uint4>(mesh->getIndicesVertexes().size());
        std::vector<uint4> normal_index_buffer = std::vector<uint4>(mesh->getIndicesNormals().size());
        std::vector<uint4> uv_index_buffer = std::vector<uint4>(mesh->getIndicesUV().size());
        std::vector<unsigned int> mat_index_buffer = std::vector<unsigned int>(mesh->getIndicesColors().size());
        std::vector<float4> vertex_buffer = std::vector<float4>(mesh->getCoordsVertices().size());
        std::vector<float4> normal_buffer = std::vector<float4>(mesh->getCoordsNormals().size());
        std::vector<float2> uv_buffer = std::vector<float2>(mesh->getCoordsUV().size());

        // unsigned int mesh_size_in_bytes = vertex_index_buffer.size() * sizeof(uint3);
        // mesh_size_in_bytes += normal_index_buffer.size() * sizeof(uint3);
        // mesh_size_in_bytes += uv_index_buffer.size() * sizeof(uint3);
        // mesh_size_in_bytes += mat_index_buffer.size() * sizeof(unsigned ints);
        // mesh_size_in_bytes += vertex_buffer.size() * sizeof(float3);
        // mesh_size_in_bytes += normal_buffer.size() * sizeof(float3);
        // mesh_size_in_bytes += uv_buffer.size() * sizeof(float2);
        // std::cout << "Unique mesh with size = " << mesh_size_in_bytes << std::endl;
        // std::cout << "vertex_index_buffer = " << vertex_index_buffer.size() << std::endl;
        // std::cout << "normal_index_buffer = " << normal_index_buffer.size() << std::endl;
        // std::cout << "uv_index_buffer = " << uv_index_buffer.size() << std::endl;
        // std::cout << "mat_index_buffer = " << mat_index_buffer.size() << std::endl;
        // std::cout << "vertex_buffer = " << vertex_buffer.size() << std::endl;
        // std::cout << "normal_buffer = " << normal_buffer.size() << std::endl;
        // std::cout << "uv_buffer = " << uv_buffer.size() << std::endl;

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
        if (mat_index_buffer.size() > 0) {  // optional whether there are material indices
            for (int i = 0; i < mesh->getIndicesColors().size(); i++) {
                mat_index_buffer[i] = (unsigned int)mesh->getIndicesColors()[i].x();
            }

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

        mesh_id = m_mesh_pool.size() - 1;

        // push this mesh to our known meshes
        m_known_meshes.push_back(std::make_tuple(mesh, mesh_id));
    }

    // record for mesh when hit by camera
    Record<MaterialRecordParameters> camera_mat_record;
    OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_camera_hit_mesh_group, &camera_mat_record));
    // record for mesh when hit by shadow
    Record<MaterialRecordParameters> shadow_mat_record;
    OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_shadow_hit_mesh_group, &shadow_mat_record));
    // record for mesh when hit by lidar
    Record<MaterialRecordParameters> lidar_mat_record;
    OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_lidar_hit_mesh_group, &lidar_mat_record));
    // record for mesh when hit by radar
    Record<MaterialRecordParameters> radar_mat_record;
    OPTIX_ERROR_CHECK(optixSbtRecordPackHeader(m_radar_hit_mesh_group, &radar_mat_record));

    if (mat_list.size() == 0) {
        unsigned int material_id = GetMaterial();
        camera_mat_record.data.material_pool_id = material_id;
        shadow_mat_record.data.material_pool_id = material_id;
        lidar_mat_record.data.material_pool_id = material_id;
        radar_mat_record.data.material_pool_id = material_id;
    } else {
        unsigned int material_id = GetMaterial(mat_list[0]);
        camera_mat_record.data.material_pool_id = material_id;
        shadow_mat_record.data.material_pool_id = material_id;
        lidar_mat_record.data.material_pool_id = material_id;
        radar_mat_record.data.material_pool_id = material_id;
        for (int i = 1; i < mat_list.size(); i++) {
            GetMaterial(mat_list[i]);
        }
    }
    camera_mat_record.data.mesh_pool_id = mesh_id;
    shadow_mat_record.data.mesh_pool_id = mesh_id;
    lidar_mat_record.data.mesh_pool_id = mesh_id;
    radar_mat_record.data.mesh_pool_id = mesh_id;
    m_material_records.push_back(camera_mat_record);
    m_material_records.push_back(shadow_mat_record);
    m_material_records.push_back(lidar_mat_record);
    m_material_records.push_back(radar_mat_record);

    // return the material index
    return m_material_records.size() / RAY_TYPE_COUNT - 1;
}

unsigned int ChOptixPipeline::GetDeformableMeshMaterial(CUdeviceptr& d_vertices,
                                                        CUdeviceptr& d_indices,
                                                        std::shared_ptr<ChTriangleMeshShape> mesh_shape,
                                                        std::vector<std::shared_ptr<ChVisualMaterial>> mat_list) {
    unsigned int mat_id = GetRigidMeshMaterial(d_vertices, d_indices, mesh_shape, mat_list);

    unsigned int mesh_id = m_material_records[mat_id * RAY_TYPE_COUNT].data.mesh_pool_id;
    CUdeviceptr d_normals = reinterpret_cast<CUdeviceptr>(m_mesh_pool[mesh_id].normal_buffer);
    unsigned int num_triangles = mesh_shape->GetMesh()->getIndicesVertexes().size();
    m_deformable_meshes.push_back(std::make_tuple(mesh_shape, d_vertices, d_normals, num_triangles));

    return mat_id;
}

void ChOptixPipeline::UpdateDeformableMeshes() {
    for (int i = 0; i < m_deformable_meshes.size(); i++) {
        std::shared_ptr<ChTriangleMeshShape> mesh_shape = std::get<0>(m_deformable_meshes[i]);
        CUdeviceptr d_vertices = std::get<1>(m_deformable_meshes[i]);
        CUdeviceptr d_normals = std::get<2>(m_deformable_meshes[i]);
        unsigned int num_prev_triangles = std::get<3>(m_deformable_meshes[i]);

        auto mesh = mesh_shape->GetMesh();

        // if the mesh has changed size, we need to recreate the entire mesh (not very nice)
        if (num_prev_triangles != mesh_shape->GetMesh()->getIndicesVertexes().size()) {
            throw std::runtime_error("Error: changing mesh size not supported by Chrono::Sensor");
        }

        // update all the vertex locations

        std::vector<float3> vertex_buffer = std::vector<float3>(mesh->getCoordsVertices().size());
        for (int j = 0; j < mesh->getCoordsVertices().size(); j++) {
            vertex_buffer[j] = make_float3((float)mesh->getCoordsVertices()[j].x(),  //
                                           (float)mesh->getCoordsVertices()[j].y(),  //
                                           (float)mesh->getCoordsVertices()[j].z());
        }
        CUDA_ERROR_CHECK(cudaMemcpy(reinterpret_cast<void*>(d_vertices), vertex_buffer.data(),
                                    sizeof(float3) * vertex_buffer.size(), cudaMemcpyHostToDevice));

        // update all the normals if normal exist
        if (mesh_shape->GetMesh()->getCoordsNormals().size() > 0) {
            std::vector<float3> normal_buffer = std::vector<float3>(mesh->getCoordsNormals().size());
            for (int j = 0; j < mesh->getCoordsNormals().size(); j++) {
                normal_buffer[j] = make_float3((float)mesh->getCoordsNormals()[j].x(),  //
                                               (float)mesh->getCoordsNormals()[j].y(),  //
                                               (float)mesh->getCoordsNormals()[j].z());
            }
            CUDA_ERROR_CHECK(cudaMemcpy(reinterpret_cast<void*>(d_normals), normal_buffer.data(),
                                        sizeof(float3) * normal_buffer.size(), cudaMemcpyHostToDevice));
        }

        // TODO: for SCM terrain, make use of the list of modified vertices
    }
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
