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

#ifndef CHOPTIXGEOMETRY_H
#define CHOPTIXGEOMETRY_H

#include <optix.h>
#include "chrono_sensor/ChApiSensor.h"
#include "chrono/core/ChFrame.h"
#include "chrono/physics/ChBody.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"

#include <deque>

namespace chrono {
namespace sensor {

/// @addtogroup sensor_optix
/// @{

/// Transform struct for packing a translation, rotation, and scale
struct Transform {
    float data[12];
};

/// Optix Geometry class that is responsible for managing all geometric information in the optix scene
/// This handles the acceleration structure and transforms
class CH_SENSOR_API ChOptixGeometry {
  public:
    /// Class constructor
    /// @param context optix context for which the ChOptixGeometry maintains the scene
    ChOptixGeometry(OptixDeviceContext context);

    /// Class destructor
    ~ChOptixGeometry();

    /// Add a box geometry to the optix scene
    /// @param body the Chrono Body that drives the box
    /// @param asset_frame the Chrono frame that specifies how the asset is attached to the body
    /// @param scale the scale of the box
    /// @param mat_id the material id associated with the box
    void AddBox(std::shared_ptr<ChBody> body, ChFrame<double> asset_frame, ChVector<double> scale, unsigned int mat_id);

    /// Add a sphere geometry to the optix scene
    /// @param body the Chrono Body that drives the sphere
    /// @param asset_frame the Chrono frame that specifies how the asset is attached to the body
    /// @param scale the scale of the sphere
    /// @param mat_id the material id associated with the sphere
    void AddSphere(std::shared_ptr<ChBody> body,
                   ChFrame<double> asset_frame,
                   ChVector<double> scale,
                   unsigned int mat_id);

    /// Add a cylinder geometry to the optix scene
    /// @param body the Chrono Body that drives the cylinder
    /// @param asset_frame the Chrono frame that specifies how the asset is attached to the body
    /// @param scale the scale of the cylinder
    /// @param mat_id the material id associated with the cylinder
    void AddCylinder(std::shared_ptr<ChBody> body,
                     ChFrame<double> asset_frame,
                     ChVector<double> scale,
                     unsigned int mat_id);

    /// Add a rigid mesh to the optix scene
    /// @param d_vertices a device pointer to the vertices of the mesh
    /// @param d_indices a device pointer to the indices of the mesh
    /// @param mesh_shape the Chrono mesh shape that defines the mesh
    /// @param body the Chrono body on which the mesh is attached
    /// @param asset_frame the Chrono frame that specifies how the asset is attached to the body
    /// @param scale the scale of the mesh
    /// @param mat_id the material id associated with the mesh
    unsigned int AddRigidMesh(CUdeviceptr d_vertices,
                              CUdeviceptr d_indices,
                              std::shared_ptr<ChVisualShapeTriangleMesh> mesh_shape,
                              std::shared_ptr<ChBody> body,
                              ChFrame<double> asset_frame,
                              ChVector<double> scale,
                              unsigned int mat_id);

    /// Add a deformable mesh to the optix scene
    /// @param d_vertices a device pointer to the vertices of the mesh
    /// @param d_indices a device pointer to the indices of the mesh
    /// @param mesh_shape the Chrono mesh shape that defines the mesh
    /// @param body the Chrono body on which the mesh is attached
    /// @param asset_frame the Chrono frame that specifies how the asset is attached to the body
    /// @param scale the scale of the mesh
    /// @param mat_id the material id associated with the mesh
    void AddDeformableMesh(CUdeviceptr d_vertices,
                           CUdeviceptr d_indices,
                           std::shared_ptr<ChVisualShapeTriangleMesh> mesh_shape,
                           std::shared_ptr<ChBody> body,
                           ChFrame<double> asset_frame,
                           ChVector<double> scale,
                           unsigned int mat_id);

    /// Create the root node and acceleration structure of the scene
    ///@return A traversable handle to the root node
    OptixTraversableHandle CreateRootStructure();

    /// Rebuild the root acceleration structure for when the root changes
    void RebuildRootStructure();

    /// Update the list of transforms associated with the bodies and assets at the start time
    void UpdateBodyTransformsStart(float t_start, float t_target_end);

    /// Update the list of transforms associated with the bodies and assets at the end time
    void UpdateBodyTransformsEnd(float t_end);

    /// Update the deformable meshes based on how the meshes changed in Chrono
    void UpdateDeformableMeshes();

    /// Cleanup the entire optix geometry manager, cleans and frees device pointers and root structure
    void Cleanup();

    /// Origin offset function for moving the origin to reduce large translations
    void SetOriginOffset(ChVector<float> origin_offset) { m_origin_offset = origin_offset; }

  private:
    /// Function to add an object to the scene given the handle
    /// @param mat_id the material id for the object
    /// @param body the chrono body associated with the object
    /// @param asset_frame the frame associated with the asset relative to the body
    /// @param scale the scale of the object
    /// @param gas_handle the handle to the geometry acceleration structure
    void AddGenericObject(unsigned int mat_id,
                          std::shared_ptr<ChBody> body,
                          ChFrame<double> asset_frame,
                          ChVector<double> scale,
                          OptixTraversableHandle gas_handle);

    /// Function to build a geometry acceleration structure for a triangle mesh
    /// @param mesh_shape the chrono mesh representing this asset
    /// @param d_vertices the device pointer to the vertices of the mesh
    /// @param d_indices the device pointer to the indices of the mesh
    /// @param compact_no_update if the GAS should be made without updating, and with compaction
    /// @param rebuild whether this is a rebuild, or a first build
    /// @param gas_id the id of the GAS is it has already been made (in case of rebuild)
    unsigned int BuildTrianglesGAS(std::shared_ptr<ChVisualShapeTriangleMesh> mesh_shape,
                                   CUdeviceptr d_vertices,
                                   CUdeviceptr d_indices,
                                   bool compact_no_update = true,
                                   bool rebuild = false,
                                   unsigned int gas_id = 0);

    /// Function ot convert scale, rotation, translation to top 3 rows of transform matrix
    /// @param[in] s the scale vector
    /// @param[in] a the rotation matrix
    /// @param[in] b the translation vector
    /// @param[out] t a pointer to where the inverse transform matrix should be placed
    static void GetT3x4FromSRT(const ChVector<double>& s,
                               const ChMatrix33<double>& a,
                               const ChVector<double>& b,
                               float* t);

    /// Function to convert scale, rotation, translation to top 3 rows of inverse transform matrix
    /// @param[in] s the scale vector
    /// @param[in] a the rotation matrix
    /// @param[in] b the translation vector
    /// @param[out] t a pointer to where the inverse transform matrix should be placed
    static void GetInvT3x4FromSRT(const ChVector<double>& s,
                                  const ChMatrix33<double>& a,
                                  const ChVector<double>& b,
                                  float* t);

    OptixDeviceContext m_context;  ///< handle to the device context -> we do not own, so will not clean up

    // primitive ids for instancing
    unsigned int m_box_gas_id;     ///< id of the single box geometry acceleration structure
    bool m_box_inst = false;       ///< whether a box has been created
    unsigned int m_sphere_gas_id;  ///< id of the single sphere geometry acceleration structure
    bool m_sphere_inst = false;    ///< whether a sphere has been created
    unsigned int m_cyl_gas_id;     ///< id of the single cylinder geometry acceleration structure
    bool m_cyl_inst = false;       ///< whether a cylinder has been created

    // intance and root buffers
    std::vector<OptixInstance> m_instances;  ///< host vector of geometry instances
    CUdeviceptr md_instances = {};           ///< device pointer to the instances on the device
    CUdeviceptr md_root_temp_buffer = {};    ///< device pointer to the root acceleration temporary buffer
    CUdeviceptr md_root_output_buffer = {};  ///< device pointer to the root acceleration buffer
    size_t md_root_temp_buffer_size;         ///< size of the root temporary buffer on the device
    size_t md_root_output_buffer_size;       ///< size of the root output buffer on the device
    OptixTraversableHandle m_root;           ///< handle to the root acceleration structure

    // GAS buffers, handles, and transforms
    std::vector<CUdeviceptr> m_gas_buffers;                       ///< all the gas buffers
    std::vector<OptixTraversableHandle> m_gas_handles;            ///< all the gas handles
    std::vector<OptixMatrixMotionTransform> m_motion_transforms;  ///< vector of all the motion transforms
    CUdeviceptr md_motion_transforms = {};                        ///< solid block of motion transforms on the device
    std::vector<OptixTraversableHandle> m_motion_handles;         ///< vector of all the motion transforms

    // index buffers for objects pointing to their materials and GAS
    std::vector<unsigned int> m_obj_mat_ids;  ///< id of the material of the box

    // vectors referencing data from Chrono
    std::vector<std::shared_ptr<ChBody>> m_bodies;         ///< chrono bodies in the scene
    std::vector<ChFrame<double>> m_obj_body_frames_start;  ///< frame at time=start used for the geometry
    std::vector<ChFrame<double>> m_obj_body_frames_end;    ///< frame at time=end used for the geometry
    std::vector<ChFrame<double>> m_obj_asset_frames;       ///< constant frame used for the geometry
    std::vector<ChVector<double>> m_obj_scales;  ///< asset frame scales since ChFrame makes the Amatrix orthonormal
    ChVector<float> m_origin_offset;             ///< origin offset for the scene

    std::vector<std::tuple<float, float, std::vector<ChFrame<double>>>>
        m_obj_body_frames_start_tmps;  ///< need to potentially hold multiple starts and will move it to
                                       ///< start under lock when the corresponding end has been packed

    /// keep track of chrono meshes we've added and their corresponding mesh pool id for automatic instancing
    std::vector<std::tuple<CUdeviceptr, unsigned int>> m_known_meshes;

    /// deformable mesh list <mesh shape, d_vertices, d_indices, gas id>
    std::vector<std::tuple<std::shared_ptr<ChVisualShapeTriangleMesh>, CUdeviceptr, CUdeviceptr, unsigned int>>
        m_deformable_meshes;

    float m_start_time;  ///< time corresponding to start frame
    float m_end_time;    ///< time corresponding to end frame
};

/// @} sensor_optix

}  // namespace sensor
}  // namespace chrono

#endif