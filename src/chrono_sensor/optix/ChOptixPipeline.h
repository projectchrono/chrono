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

#ifndef CHOPTIXPIPELINE_H
#define CHOPTIXPIPELINE_H

#include <optix.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>

#include "chrono_sensor/optix/ChOptixDefinitions.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono_sensor/optix/scene/ChScene.h"

#include "chrono_sensor/ChApiSensor.h"

#include <vector>
#include <memory>

namespace chrono {
namespace sensor {

template <typename T>
struct Record {
    __align__(OPTIX_SBT_RECORD_ALIGNMENT) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
    T data;
};

enum class PipelineType {
    CAMERA_PINHOLE,   // pinhole camera model
    CAMERA_FOV_LENS,  // FOV lens model
    LIDAR_SINGLE,     // single sample lidar
    LIDAR_MULTI,      // multi sample lidar
    RADAR             // radar model
};
// TODO: how do we allow custom ray gen programs? (Is that every going to be a thing?)

class CH_SENSOR_API ChOptixPipeline {
  public:
    ChOptixPipeline(OptixDeviceContext context, unsigned int trace_depth, bool debug);
    ~ChOptixPipeline();

    void SpawnPipeline(PipelineType type);
    unsigned int GetBoxMaterial(std::shared_ptr<ChVisualMaterial> mat = nullptr);
    unsigned int GetSphereMaterial(std::shared_ptr<ChVisualMaterial> mat = nullptr);
    unsigned int GetCylinderMaterial(std::shared_ptr<ChVisualMaterial> mat = nullptr);

    unsigned int GetRigidMeshMaterial(CUdeviceptr& d_vertices,
                                      CUdeviceptr& d_indices,
                                      std::shared_ptr<ChTriangleMeshShape> sphere_shape,
                                      std::vector<std::shared_ptr<ChVisualMaterial>> mat_list);

    unsigned int GetDeformableMeshMaterial(CUdeviceptr& d_vertices,
                                           CUdeviceptr& d_indices,
                                           std::shared_ptr<ChTriangleMeshShape> sphere_shape,
                                           std::vector<std::shared_ptr<ChVisualMaterial>> mat_list);

    void UpdateDeformableMeshes();

    void UpdateAllSBTs();
    void UpdateAllPipelines();
    void UpdateObjectVelocity();

    CUdeviceptr GetMeshPool();
    CUdeviceptr GetMaterialPool();

    void AddBody(std::shared_ptr<ChBody> body){m_bodies.push_back(body);}

    OptixPipeline& GetPipeline(unsigned int id);
    std::shared_ptr<OptixShaderBindingTable> GetSBT(unsigned int id);
    std::shared_ptr<Record<RaygenParameters>> GetRayGenRecord(unsigned int id);

    void Cleanup();
    void CleanMaterials();

    void UpdateBackground(Background b);

  private:
    // private class functions
    void CompileBaseShaders();
    void AssembleBaseProgramGroups();
    void CreateBaseSBT();

    unsigned int GetMaterial(std::shared_ptr<ChVisualMaterial> mat = nullptr);

    void CreateDeviceTexture(cudaTextureObject_t& d_tex_sampler,
                             cudaArray_t& d_img_array,
                             std::string file_name,
                             bool mirror = false,
                             bool exclude_from_material_cleanup = false);

    // Private class variables - DO NOT MODIFY THE CONTEXT
    OptixDeviceContext m_context;

    unsigned int m_trace_depth;
    bool m_debug;

    // modules - we only make one of each - do not clear when rebuilding root
    OptixModule m_box_intersection_module = 0;     // box.cu file
    OptixModule m_sphere_intersection_module = 0;  // sphere.cu file
    OptixModule m_cyl_intersection_module = 0;     // cylinder.cu file
    OptixModule m_camera_raygen_module = 0;        // camera.cu file
    OptixModule m_lidar_raygen_module = 0;         // lidar.cu file
    OptixModule m_radar_raygen_module = 0;         // lidar.cu file
    OptixModule m_material_shading_module = 0;     // material shader file
    // OptixModule m_camera_shading_module = 0;       // camera_shaders.cu file
    // OptixModule m_shadow_shading_module = 0;       // shadow_shaders.cu
    // OptixModule m_lidar_shading_module = 0;        // lidar_shaders.cu file
    // OptixModule m_radar_shading_module = 0;        // lidar_shaders.cu file
    OptixModule m_miss_module = 0;  // miss.cu
    // OptixModule m_exception_module = 0;            // exception.cu

    // program groups - we only make one of each - do not clear when rebuilding root
    OptixProgramGroup m_camera_pinhole_raygen_group = 0;
    OptixProgramGroup m_camera_fov_lens_raygen_group = 0;
    OptixProgramGroup m_lidar_single_raygen_group = 0;
    OptixProgramGroup m_lidar_multi_raygen_group = 0;
    OptixProgramGroup m_radar_raygen_group = 0;

    OptixProgramGroup m_hit_box_group = 0;
    // OptixProgramGroup m_camera_hit_box_group = 0;
    // OptixProgramGroup m_shadow_hit_box_group = 0;
    // OptixProgramGroup m_lidar_hit_box_group = 0;
    // OptixProgramGroup m_radar_hit_box_group = 0;

    OptixProgramGroup m_hit_sphere_group = 0;
    // OptixProgramGroup m_camera_hit_sphere_group = 0;
    // OptixProgramGroup m_shadow_hit_sphere_group = 0;
    // OptixProgramGroup m_lidar_hit_sphere_group = 0;
    // OptixProgramGroup m_radar_hit_sphere_group = 0;

    OptixProgramGroup m_hit_cyl_group = 0;
    // OptixProgramGroup m_camera_hit_cyl_group = 0;
    // OptixProgramGroup m_shadow_hit_cyl_group = 0;
    // OptixProgramGroup m_lidar_hit_cyl_group = 0;
    // OptixProgramGroup m_radar_hit_cyl_group = 0;

    OptixProgramGroup m_hit_mesh_group = 0;
    // OptixProgramGroup m_camera_hit_mesh_group = 0;
    // OptixProgramGroup m_shadow_hit_mesh_group = 0;
    // OptixProgramGroup m_lidar_hit_mesh_group = 0;
    // OptixProgramGroup m_radar_hit_mesh_group = 0;

    OptixProgramGroup m_miss_group = 0;
    // OptixProgramGroup m_camera_miss_group = 0;
    // OptixProgramGroup m_shadow_miss_group = 0;
    // OptixProgramGroup m_lidar_miss_group = 0;
    // OptixProgramGroup m_radar_miss_group = 0;
    // OptixProgramGroup m_exception_group = 0;

    // compile options - TODO: should probably depend on the pipeline - do not clear for now
    OptixPipelineCompileOptions m_pipeline_compile_options;

    // the pipelines that are in use - do not clear when rebuilding root
    std::vector<OptixPipeline> m_pipelines;
    std::vector<std::shared_ptr<Record<RaygenParameters>>> m_raygen_records;

    // refresh this with new material information without clearing when root get rebuilt
    std::vector<std::shared_ptr<OptixShaderBindingTable>> m_sbts;

    // miss record - do not clear
    CUdeviceptr md_miss_record = {};  ///< handle to the miss record on the device

    // material records - clear when rebuilding root
    std::vector<Record<MaterialRecordParameters>> m_material_records;  ///< vector of all material records
    CUdeviceptr md_material_records = {};                              ///< handle to the material records on the device

    // material pool (host and device)
    std::vector<MaterialParameters> m_material_pool;
    CUdeviceptr md_material_pool = {};

    // mesh data pool
    std::vector<MeshParameters> m_mesh_pool;        ///< mesh pool on the device
    CUdeviceptr md_mesh_pool = {};                  ///< struct to know which buffers make up which meshes
    std::vector<CUdeviceptr> m_mesh_buffers_dptrs;  ///< for keeping a handle to free later

    // texture and image handles
    // std::vector<cudaTextureObject_t> m_texture_samplers;  ///< for keeping a handle to free later
    // std::vector<cudaArray_t> m_img_textures;              ///< for keeping a handle to free later
    std::unordered_map<std::string, cudaTextureObject_t> m_texture_samplers;  ///< for keeping a handle to free later
    std::unordered_map<std::string, cudaArray_t> m_img_textures;              ///< for keeping a handle to free later

    cudaTextureObject_t md_miss_texture_sampler = {};  ///< handle to the environment texture sampler
    cudaArray_t md_miss_img_texture = {};              ///< handle to the environment image texture

    /// keep track of chrono meshes we've added and their corresponding mesh pool id
    std::vector<std::tuple<std::shared_ptr<geometry::ChTriangleMeshConnected>, unsigned int>> m_known_meshes;

    /// list of deformable meshes <mesh shape, dvertices, dnormals, num prev triangles>
    std::vector<std::tuple<std::shared_ptr<ChTriangleMeshShape>, CUdeviceptr, CUdeviceptr, unsigned int>>
        m_deformable_meshes;

    /// keep track of chrono bodies in scene so we can update velocity to device
    std::vector<std::shared_ptr<ChBody>> m_bodies;
    // record defaults - clear when rebuilding root
    // bool m_default_box_record_inst = false;
    // unsigned int m_default_box_record_id;
    // bool m_default_sphere_record_inst = false;
    // unsigned int m_default_sphere_record_id;
    // bool m_default_cyl_record_inst = false;
    // unsigned int m_default_cyl_record_id;

    // default material in the material pool
    bool m_default_material_inst = false;
    unsigned int m_default_material_id;
};
}  // namespace sensor
}  // namespace chrono

#endif