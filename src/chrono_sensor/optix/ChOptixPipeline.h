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
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono_sensor/optix/scene/ChScene.h"

#include "chrono_sensor/ChApiSensor.h"

#include <vector>
#include <memory>

namespace chrono {
namespace sensor {

/// @addtogroup sensor_optix
/// @{

template <typename T>
struct Record {
    __align__(OPTIX_SBT_RECORD_ALIGNMENT) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
    T data;
};

/// The type of ray tracing used to model the sensor
enum class PipelineType {
    CAMERA,  ///< camera rendering pipeline
    // CAMERA_FOV_LENS,        ///< FOV lens model
    SEGMENTATION,  ///< segmentation camera pipeline
    // SEGMENTATION_FOV_LENS,  ///< FOV lens segmentation camera
    LIDAR_SINGLE,  ///< single sample lidar
    LIDAR_MULTI,   ///< multi sample lidar
    RADAR          ///< radar model
};
// TODO: how do we allow custom ray gen programs? (Is that ever going to be a thing?)

/// Class to hold all the Shader Binding Table parameters adnd manage the ray tracing pipeline, materials, ray gen
/// programs
class CH_SENSOR_API ChOptixPipeline {
  public:
    /// Class constructor. Creates a ChOptixPipeline that can manage sensors, materials, and SBT information
    /// @param context The optix context to which this pipeline will be attached
    /// @param trace_depth the maximum trace depth for ray recusions in this pipeline (contant for all sensors in the
    /// pipeline)
    /// @param debug whether the pipeline should be run in debug mode. Only use for development as this slows down the
    /// ray tracing
    ChOptixPipeline(OptixDeviceContext context, unsigned int trace_depth, bool debug);

    /// Class destructor. Cleans up all data associated with the pipeline
    ~ChOptixPipeline();

    /// Create a new pipeline for a specific type of sensor
    /// @param type the type of sensor/ray tracing to add to the pipeline manager
    void SpawnPipeline(PipelineType type);

    /// Creates a new box material
    /// @param mat the chrono material from which to create an optix material
    /// @returns an id pointing to the material that was created
    unsigned int GetBoxMaterial(std::vector<std::shared_ptr<ChVisualMaterial>> mat_list = {});

    /// Creates a new sphere material
    /// @param mat the chrono material from which to create an optix material
    /// @returns an id pointing to the material that was created
    unsigned int GetSphereMaterial(std::vector<std::shared_ptr<ChVisualMaterial>> mat_list = {});

    /// Creates a new cylinder material
    /// @param mat the chrono material from which to create an optix material
    /// @returns an id pointing to the material that was created
    unsigned int GetCylinderMaterial(std::vector<std::shared_ptr<ChVisualMaterial>> mat_list = {});

    /// Creates a new rigid material/mesh in optix
    /// @param[out] d_vertices a device pointer where the mesh's vertices will be stored
    /// @param[out] d_indices a device pointer where the mesh's indices will be stored
    /// @param[in] sphere_shape the chrono mesh to add to the optix scene
    /// @param[in] mat_list the chrono materials from which to create an optix material
    /// @returns an id pointing to the material that was created
    unsigned int GetRigidMeshMaterial(CUdeviceptr& d_vertices,
                                      CUdeviceptr& d_indices,
                                      std::shared_ptr<ChVisualShapeTriangleMesh> sphere_shape,
                                      std::vector<std::shared_ptr<ChVisualMaterial>> mat_list);

    /// Creates a new deformable material/mesh in optix
    /// @param[out] d_vertices a device pointer where the mesh's vertices will be stored
    /// @param[out] d_indices a device pointer where the mesh's indices will be stored
    /// @param[in] sphere_shape the chrono mesh to add to the optix scene
    /// @param[in] mat_list the chrono materials from which to create an optix material
    /// @returns an id pointing to the material that was created
    unsigned int GetDeformableMeshMaterial(CUdeviceptr& d_vertices,
                                           CUdeviceptr& d_indices,
                                           std::shared_ptr<ChVisualShapeTriangleMesh> sphere_shape,
                                           std::vector<std::shared_ptr<ChVisualMaterial>> mat_list);

    /// Function to update all the deformable meshes in the optix scene based on their chrono meshes
    void UpdateDeformableMeshes();

    /// Function to update all the shader binding tables associated with this optix scene
    void UpdateAllSBTs();

    /// Function to update all the pipeline associated with this optix scene
    void UpdateAllPipelines();

    /// Function to update all object velocities. Only required for sensors that measure velocity with is a dynamic
    /// property
    void UpdateObjectVelocity();

    /// Function to access the mesh pool on the device
    /// @returns a device pointer to the collection of meshes
    CUdeviceptr GetMeshPool();

    /// Function to access the material pool on the device
    /// @returns a device pointer to the collection of materials
    CUdeviceptr GetMaterialPool();

    /// Add a new body to the optix scene based on a chrono body
    /// @param body the Chrono body from which to create an optix object
    void AddBody(std::shared_ptr<ChBody> body) { m_bodies.push_back(body); }

    /// OptixPipeline accessor function
    /// @param id the id of the desired pipeline
    /// @returns a reference to the specified OptixPipeline
    OptixPipeline& GetPipeline(unsigned int id);

    /// SBT accessor
    /// @param id the id of the pipeline whose shader binding table should be returned
    /// @returns a shared pointer to the specified shader binding table
    std::shared_ptr<OptixShaderBindingTable> GetSBT(unsigned int id);

    /// Raygen record accessor
    /// @param id the id of the pipline whose raygen record should be returned
    /// @returns a shared pointer to the specified raygen record
    std::shared_ptr<Record<RaygenParameters>> GetRayGenRecord(unsigned int id);

    /// Explicit cleanup function for freeing memory associated with this pipeline
    /// which should be freed before reconstructing the pipeline. Any reusable varaibles
    /// will not be cleaned up here
    void Cleanup();

    /// Explicit material cleanup function for freeing all memory associate with materials
    /// in the optix scene
    void CleanMaterials();

    /// Updates the background of the scene. Only used for cameras
    /// @param b a new background to use for the scene. The old background will be removed
    void UpdateBackground(Background b);

  private:
    void CompileBaseShaders();
    void AssembleBaseProgramGroups();
    void CreateBaseSBT();
    void CreateOptixProgramGroup(OptixProgramGroup& group,
                                 OptixProgramGroupKind k,
                                 OptixModule is_module,
                                 const char* is_name,
                                 OptixModule ch_module,
                                 const char* ch_name);

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
    OptixModule m_miss_module = 0;                 // miss.cu

    // program groups - we only make one of each - do not clear when rebuilding root
    OptixProgramGroup m_camera_raygen_group = 0;
    // OptixProgramGroup m_camera_fov_lens_raygen_group = 0;
    OptixProgramGroup m_segmentation_raygen_group = 0;
    // OptixProgramGroup m_segmentation_fov_lens_raygen_group = 0;
    OptixProgramGroup m_lidar_single_raygen_group = 0;
    OptixProgramGroup m_lidar_multi_raygen_group = 0;
    OptixProgramGroup m_radar_raygen_group = 0;

    OptixProgramGroup m_hit_box_group = 0;
    OptixProgramGroup m_hit_sphere_group = 0;
    OptixProgramGroup m_hit_cyl_group = 0;
    OptixProgramGroup m_hit_mesh_group = 0;
    OptixProgramGroup m_miss_group = 0;

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
    std::unordered_map<std::string, cudaTextureObject_t> m_texture_samplers;  ///< for keeping a handle to free later
    std::unordered_map<std::string, cudaArray_t> m_img_textures;              ///< for keeping a handle to free later

    cudaTextureObject_t md_miss_texture_sampler = {};  ///< handle to the environment texture sampler
    cudaArray_t md_miss_img_texture = {};              ///< handle to the environment image texture

    /// keep track of chrono meshes we've added and their corresponding mesh pool id
    std::vector<std::tuple<std::shared_ptr<geometry::ChTriangleMeshConnected>, unsigned int>> m_known_meshes;

    /// list of deformable meshes <mesh shape, dvertices, dnormals, num prev triangles>
    std::vector<std::tuple<std::shared_ptr<ChVisualShapeTriangleMesh>, CUdeviceptr, CUdeviceptr, unsigned int>>
        m_deformable_meshes;

    // default material in the material pool
    bool m_default_material_inst = false;
    unsigned int m_default_material_id;

    // bodies in simulation
    std::vector<std::shared_ptr<ChBody>> m_bodies;
};

/// @} sensor_optix

}  // namespace sensor
}  // namespace chrono

#endif