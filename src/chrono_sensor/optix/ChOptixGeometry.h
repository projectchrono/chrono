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
#include "chrono/assets/ChTriangleMeshShape.h"

#include <deque>

namespace chrono {
namespace sensor {

struct Transform {
    float data[12];
};

class CH_SENSOR_API ChOptixGeometry {
  public:
    ChOptixGeometry(OptixDeviceContext context);
    ~ChOptixGeometry();
    void AddBox(std::shared_ptr<ChBody> body, ChFrame<double> asset_frame, ChVector<double> scale, unsigned int mat_id);
    void AddSphere(std::shared_ptr<ChBody> body,
                   ChFrame<double> asset_frame,
                   ChVector<double> scale,
                   unsigned int mat_id);
    void AddCylinder(std::shared_ptr<ChBody> body,
                     ChFrame<double> asset_frame,
                     ChVector<double> scale,
                     unsigned int mat_id);
    unsigned int AddRigidMesh(CUdeviceptr d_vertices,
                              CUdeviceptr d_indices,
                              std::shared_ptr<ChTriangleMeshShape> mesh_shape,
                              std::shared_ptr<ChBody> body,
                              ChFrame<double> asset_frame,
                              ChVector<double> scale,
                              unsigned int mat_id);

    void AddDeformableMesh(CUdeviceptr d_vertices,
                           CUdeviceptr d_indices,
                           std::shared_ptr<ChTriangleMeshShape> mesh_shape,
                           std::shared_ptr<ChBody> body,
                           ChFrame<double> asset_frame,
                           ChVector<double> scale,
                           unsigned int mat_id);

    OptixTraversableHandle CreateRootStructure();
    void RebuildRootStructure();
    void UpdateBodyTransformsStart(float t_start, float t_target_end);
    void UpdateBodyTransformsEnd(float t_end);
    void UpdateDeformableMeshes();

    void Cleanup();

  private:
    void AddGenericObject(unsigned int mat_id,
                          std::shared_ptr<ChBody> body,
                          ChFrame<double> asset_frame,
                          ChVector<double> scale,
                          OptixTraversableHandle gas_handle);

    unsigned int BuildTrianglesGAS(std::shared_ptr<ChTriangleMeshShape> mesh_shape,
                                   CUdeviceptr d_vertices,
                                   CUdeviceptr d_indices,
                                   bool compact_no_update = true,
                                   bool rebuild = false,
                                   unsigned int gas_id = 0);

    static void GetT3x4FromSRT(const ChVector<double>& s,
                               const ChMatrix33<double>& a,
                               const ChVector<double>& b,
                               float* t);
    static void GetInvT3x4FromSRT(const ChVector<double>& s,
                                  const ChMatrix33<double>& a,
                                  const ChVector<double>& b,
                                  float* t);

    OptixDeviceContext m_context;  ///< handle to the device context -> we do not own, so will not clean up

    // primitive ids for instancing
    unsigned int m_box_gas_id;
    bool m_box_inst = false;
    unsigned int m_sphere_gas_id;
    bool m_sphere_inst = false;
    unsigned int m_cyl_gas_id;
    bool m_cyl_inst = false;

    // intance and root buffers
    std::vector<OptixInstance> m_instances;
    CUdeviceptr md_instances = {};
    CUdeviceptr md_root_temp_buffer = {};
    CUdeviceptr md_root_output_buffer = {};
    size_t md_root_temp_buffer_size;
    size_t md_root_output_buffer_size;
    OptixTraversableHandle m_root;

    // GAS buffers, handles, and transforms
    std::vector<CUdeviceptr> m_gas_buffers;                       ///< all the gas buffers
    std::vector<OptixTraversableHandle> m_gas_handles;            ///< all the gas handles
    std::vector<OptixMatrixMotionTransform> m_motion_transforms;  ///< vector of all the motion transforms
    CUdeviceptr md_motion_transforms = {};                        ///< solid block of motion transforms on the device
    std::vector<OptixTraversableHandle> m_motion_handles;         ///< vector of all the motion transforms

    // index buffers for objects pointing to their materials and GAS
    // std::vector<unsigned int> m_gas_ids;      ///< id of the material of the box //TODO: may not need
    std::vector<unsigned int> m_obj_mat_ids;  ///< id of the material of the box

    // vectors referencing data from Chrono
    std::vector<std::shared_ptr<ChBody>> m_bodies;         ///< chrono bodies in the scene
    std::vector<ChFrame<double>> m_obj_body_frames_start;  ///< frame at time=start used for the geometry
    std::vector<ChFrame<double>> m_obj_body_frames_end;    ///< frame at time=end used for the geometry
    std::vector<ChFrame<double>> m_obj_asset_frames;       ///< constant frame used for the geometry
    std::vector<ChVector<double>> m_obj_scales;  ///< asset frame scales since ChFrame makes the Amatrix orthonormal

    std::vector<std::tuple<float, float, std::vector<ChFrame<double>>>>
        m_obj_body_frames_start_tmps;  ///< need to potentially hold multiple starts and will move it to
                                       ///< start under lock when the corresponding end has been packed

    /// keep track of chrono meshes we've added and their corresponding mesh pool id
    std::vector<std::tuple<CUdeviceptr, unsigned int>> m_known_meshes;

    /// deformable mesh list <mesh shape, d_vertices, d_indices, gas id>
    std::vector<std::tuple<std::shared_ptr<ChTriangleMeshShape>, CUdeviceptr, CUdeviceptr, unsigned int>>
        m_deformable_meshes;

    float m_start_time;  ///< time corresponding to start frame
    float m_end_time;    ///< time corresponding to end frame
};

}  // namespace sensor
}  // namespace chrono

#endif