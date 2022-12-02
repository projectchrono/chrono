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

#include "chrono_sensor/optix/ChOptixGeometry.h"

#include "chrono_sensor/optix/ChOptixDefinitions.h"

#include <cuda.h>
#include <cuda_runtime.h>
#include <optix_stubs.h>

#include "chrono_sensor/optix/ChOptixUtils.h"

#include <chrono>

namespace chrono {
namespace sensor {

ChOptixGeometry::ChOptixGeometry(OptixDeviceContext context) : m_context(context), m_start_time(0.f), m_end_time(1.f) {}

ChOptixGeometry::~ChOptixGeometry() {
    Cleanup();
}

void ChOptixGeometry::Cleanup() {
    // intance and root buffer cleanup
    if (md_instances) {
        CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void*>(md_instances)));
        md_instances = {};
    }
    if (md_root_temp_buffer) {
        CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void*>(md_root_temp_buffer)));
        md_root_temp_buffer = {};
    }
    if (md_root_output_buffer) {
        CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void*>(md_root_output_buffer)));
        md_root_output_buffer = {};
    }
    m_instances.clear();

    // GAS buffers, handles, and tranform cleanup
    if (md_motion_transforms) {
        CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void*>(md_motion_transforms)));
        md_motion_transforms = {};
    }

    for (int i = 0; i < m_gas_buffers.size(); i++) {
        if (m_gas_buffers[i]) {
            CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void*>(m_gas_buffers[i])));
            m_gas_buffers[i] = {};
        }
    }
    m_gas_buffers.clear();
    m_gas_handles.clear();
    m_motion_transforms.clear();
    m_motion_handles.clear();

    // Clean primitives ids and instantiation bools
    m_box_inst = false;
    m_box_gas_id = 0;
    m_sphere_inst = false;
    m_sphere_gas_id = 0;
    m_cyl_inst = false;
    m_cyl_gas_id = 0;

    // clear out chrono reference data
    m_obj_scales.clear();
    m_obj_asset_frames.clear();
    m_obj_body_frames_start.clear();
    m_obj_body_frames_end.clear();
    m_bodies.clear();

    // clear out our list of known meshes
    m_known_meshes.clear();

    // clear our deformable meshes
    m_deformable_meshes.clear();

    // clear out index buffers
    m_obj_mat_ids.clear();
}

void ChOptixGeometry::AddGenericObject(unsigned int mat_id,
                                       std::shared_ptr<ChBody> body,
                                       ChFrame<double> asset_frame,
                                       ChVector<double> scale,
                                       OptixTraversableHandle gas_handle) {
    // add to list of box instances
    m_obj_mat_ids.push_back(mat_id);
    m_bodies.push_back(body);
    m_obj_body_frames_start.push_back(body->GetFrame_REF_to_abs());
    m_obj_body_frames_end.push_back(body->GetFrame_REF_to_abs());

    // create the motion transform for this box
    m_motion_transforms.emplace_back();
    m_motion_handles.emplace_back();
    auto i = m_motion_transforms.size() - 1;
    m_obj_asset_frames.push_back(asset_frame);
    m_obj_scales.push_back(scale);

    m_motion_transforms[i].child = gas_handle;
    m_motion_transforms[i].motionOptions.numKeys = 2;               // TODO: would we need more than this?
    m_motion_transforms[i].motionOptions.timeBegin = m_start_time;  // default at start, will be updated
    m_motion_transforms[i].motionOptions.timeEnd = m_end_time;      // default at start, will be updated
    m_motion_transforms[i].motionOptions.flags = OPTIX_MOTION_FLAG_NONE;

    const float t[12] = {1.f, 0.f, 0.f, 0.f,   // row 1
                         0.f, 1.f, 0.f, 0.f,   // row 2
                         0.f, 0.f, 1.f, 0.f};  // row 3
    memcpy(m_motion_transforms[i].transform[0], t, 12 * sizeof(float));
    memcpy(m_motion_transforms[i].transform[1], t, 12 * sizeof(float));
}

void ChOptixGeometry::AddBox(std::shared_ptr<ChBody> body,
                             ChFrame<double> asset_frame,
                             ChVector<double> scale,
                             unsigned int mat_id) {
    if (!m_box_inst) {
        // create first box instance
        OptixAabb aabb[1];
        *aabb = {-.5f, -.5f, -.5f, .5f, .5f, .5f};
        CUdeviceptr d_aabb;
        CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_aabb), sizeof(OptixAabb)));
        CUDA_ERROR_CHECK(cudaMemcpy(reinterpret_cast<void*>(d_aabb), &aabb, sizeof(OptixAabb), cudaMemcpyHostToDevice));
        uint32_t aabb_input_flags[] = {OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT};
        // uint32_t aabb_input_flags[] = {OPTIX_GEOMETRY_FLAG_NONE};
        // const uint32_t sbt_index[] = {0};  // TODO: may need to check this when we have multiple types of ojbects
        // CUdeviceptr d_sbt_index;
        // CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_sbt_index), sizeof(sbt_index)));
        // CUDA_ERROR_CHECK(
        //     cudaMemcpy(reinterpret_cast<void*>(d_sbt_index), sbt_index, sizeof(sbt_index), cudaMemcpyHostToDevice));

        OptixBuildInput aabb_input = {};
        aabb_input.type = OPTIX_BUILD_INPUT_TYPE_CUSTOM_PRIMITIVES;
        aabb_input.customPrimitiveArray.aabbBuffers = &d_aabb;
        aabb_input.customPrimitiveArray.flags = aabb_input_flags;
        aabb_input.customPrimitiveArray.numSbtRecords = 1;
        aabb_input.customPrimitiveArray.numPrimitives = 1;
        aabb_input.customPrimitiveArray.sbtIndexOffsetBuffer = 0;  // d_sbt_index;
        aabb_input.customPrimitiveArray.sbtIndexOffsetSizeInBytes = sizeof(uint32_t);
        aabb_input.customPrimitiveArray.sbtIndexOffsetStrideInBytes = sizeof(uint32_t);
        aabb_input.customPrimitiveArray.primitiveIndexOffset = 0;

        OptixAccelBuildOptions accel_options = {
            OPTIX_BUILD_FLAG_ALLOW_COMPACTION,  // buildFlags
            OPTIX_BUILD_OPERATION_BUILD         // operation
        };

        // building box GAS
        OptixAccelBufferSizes gas_buffer_sizes;
        CUdeviceptr d_temp_buffer_gas;
        OPTIX_ERROR_CHECK(optixAccelComputeMemoryUsage(m_context, &accel_options, &aabb_input, 1, &gas_buffer_sizes));
        CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_temp_buffer_gas), gas_buffer_sizes.tempSizeInBytes));
        CUdeviceptr d_buffer_temp_output_gas_and_compacted_size;

        size_t compactedSizeOffset = (size_t)((gas_buffer_sizes.outputSizeInBytes + 8ull - 1 / 8ull) * 8ull);

        CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_buffer_temp_output_gas_and_compacted_size),
                                    compactedSizeOffset + 8));
        OptixAccelEmitDesc emitProperty = {};
        emitProperty.type = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE;
        emitProperty.result = (CUdeviceptr)((char*)d_buffer_temp_output_gas_and_compacted_size + compactedSizeOffset);

        m_gas_handles.emplace_back();
        m_gas_buffers.emplace_back();
        m_box_gas_id = static_cast<unsigned int>(m_gas_handles.size() - 1);

        OPTIX_ERROR_CHECK(optixAccelBuild(m_context, 0, &accel_options, &aabb_input, 1, d_temp_buffer_gas,
                                          gas_buffer_sizes.tempSizeInBytes, d_buffer_temp_output_gas_and_compacted_size,
                                          gas_buffer_sizes.outputSizeInBytes, &m_gas_handles[m_box_gas_id],
                                          &emitProperty, 1));
        CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void*>(d_temp_buffer_gas)));

        size_t compacted_gas_size;
        CUDA_ERROR_CHECK(cudaMemcpy(&compacted_gas_size, reinterpret_cast<void*>(emitProperty.result), sizeof(size_t),
                                    cudaMemcpyDeviceToHost));

        if (compacted_gas_size < gas_buffer_sizes.outputSizeInBytes) {
            CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&m_gas_buffers[m_box_gas_id]), compacted_gas_size));

            // use handle as input and output
            OPTIX_ERROR_CHECK(optixAccelCompact(m_context, 0, m_gas_handles[m_box_gas_id], m_gas_buffers[m_box_gas_id],
                                                compacted_gas_size, &m_gas_handles[m_box_gas_id]));
            CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void*>(d_buffer_temp_output_gas_and_compacted_size)));
        } else {
            m_gas_buffers[m_box_gas_id] = d_buffer_temp_output_gas_and_compacted_size;
        }
        CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void*>(d_aabb)));
        // CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void*>(d_sbt_index)));
        m_box_inst = true;
    }

    AddGenericObject(mat_id, body, asset_frame, scale, m_gas_handles[m_box_gas_id]);
}

void ChOptixGeometry::AddSphere(std::shared_ptr<ChBody> body,
                                ChFrame<double> asset_frame,
                                ChVector<double> scale,
                                unsigned int mat_id) {
    if (!m_sphere_inst) {
        // create first sphere instance
        OptixAabb aabb[1];
        *aabb = {-1.f, -1.f, -1.f, 1.f, 1.f, 1.f};
        CUdeviceptr d_aabb;
        CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_aabb), sizeof(OptixAabb)));
        CUDA_ERROR_CHECK(cudaMemcpy(reinterpret_cast<void*>(d_aabb), &aabb, sizeof(OptixAabb), cudaMemcpyHostToDevice));
        uint32_t aabb_input_flags[] = {OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT};
        // uint32_t aabb_input_flags[] = {OPTIX_GEOMETRY_FLAG_NONE};
        // const uint32_t sbt_index[] = {0};
        OptixBuildInput aabb_input = {};
        aabb_input.type = OPTIX_BUILD_INPUT_TYPE_CUSTOM_PRIMITIVES;
        aabb_input.customPrimitiveArray.aabbBuffers = &d_aabb;
        aabb_input.customPrimitiveArray.flags = aabb_input_flags;
        aabb_input.customPrimitiveArray.numSbtRecords = 1;
        aabb_input.customPrimitiveArray.numPrimitives = 1;
        aabb_input.customPrimitiveArray.sbtIndexOffsetBuffer = 0;  // d_sbt_index;
        aabb_input.customPrimitiveArray.sbtIndexOffsetSizeInBytes = sizeof(uint32_t);
        aabb_input.customPrimitiveArray.sbtIndexOffsetStrideInBytes = sizeof(uint32_t);
        aabb_input.customPrimitiveArray.primitiveIndexOffset = 0;

        OptixAccelBuildOptions accel_options = {
            OPTIX_BUILD_FLAG_ALLOW_COMPACTION,  // buildFlags
            OPTIX_BUILD_OPERATION_BUILD         // operation
        };

        // building sphere GAS
        OptixAccelBufferSizes gas_buffer_sizes;
        CUdeviceptr d_temp_buffer_gas;
        OPTIX_ERROR_CHECK(optixAccelComputeMemoryUsage(m_context, &accel_options, &aabb_input, 1, &gas_buffer_sizes));
        CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_temp_buffer_gas), gas_buffer_sizes.tempSizeInBytes));
        CUdeviceptr d_buffer_temp_output_gas_and_compacted_size;

        size_t compactedSizeOffset = (size_t)((gas_buffer_sizes.outputSizeInBytes + 8ull - 1 / 8ull) * 8ull);

        CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_buffer_temp_output_gas_and_compacted_size),
                                    compactedSizeOffset + 8));
        OptixAccelEmitDesc emitProperty = {};
        emitProperty.type = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE;
        emitProperty.result = (CUdeviceptr)((char*)d_buffer_temp_output_gas_and_compacted_size + compactedSizeOffset);

        m_gas_handles.emplace_back();
        m_gas_buffers.emplace_back();
        m_sphere_gas_id = static_cast<unsigned int>(m_gas_handles.size() - 1);

        OPTIX_ERROR_CHECK(optixAccelBuild(m_context, 0, &accel_options, &aabb_input, 1, d_temp_buffer_gas,
                                          gas_buffer_sizes.tempSizeInBytes, d_buffer_temp_output_gas_and_compacted_size,
                                          gas_buffer_sizes.outputSizeInBytes, &m_gas_handles[m_sphere_gas_id],
                                          &emitProperty, 1));
        CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void**>(d_temp_buffer_gas)));

        size_t compacted_gas_size;
        CUDA_ERROR_CHECK(cudaMemcpy(&compacted_gas_size, reinterpret_cast<void*>(emitProperty.result), sizeof(size_t),
                                    cudaMemcpyDeviceToHost));

        if (compacted_gas_size < gas_buffer_sizes.outputSizeInBytes) {
            CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&m_gas_buffers[m_sphere_gas_id]), compacted_gas_size));

            // use handle as input and output
            OPTIX_ERROR_CHECK(optixAccelCompact(m_context, 0, m_gas_handles[m_sphere_gas_id],
                                                m_gas_buffers[m_sphere_gas_id], compacted_gas_size,
                                                &m_gas_handles[m_sphere_gas_id]));
            CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void*>(d_buffer_temp_output_gas_and_compacted_size)));
        } else {
            m_gas_buffers[m_sphere_gas_id] = d_buffer_temp_output_gas_and_compacted_size;
        }
        CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void*>(d_aabb)));
        m_sphere_inst = true;
    }

    AddGenericObject(mat_id, body, asset_frame, scale, m_gas_handles[m_sphere_gas_id]);
}

void ChOptixGeometry::AddCylinder(std::shared_ptr<ChBody> body,
                                  ChFrame<double> asset_frame,
                                  ChVector<double> scale,
                                  unsigned int mat_id) {
    if (!m_cyl_inst) {
        // create first cylinder instance
        OptixAabb aabb[1];
        *aabb = {-1.f, -.5f, -1.f, 1.f, .5f, 1.f};
        CUdeviceptr d_aabb;
        CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_aabb), sizeof(OptixAabb)));
        CUDA_ERROR_CHECK(cudaMemcpy(reinterpret_cast<void*>(d_aabb), &aabb, sizeof(OptixAabb), cudaMemcpyHostToDevice));
        uint32_t aabb_input_flags[] = {OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT};
        // uint32_t aabb_input_flags[] = {OPTIX_GEOMETRY_FLAG_NONE};
        // const uint32_t sbt_index[] = {0};
        OptixBuildInput aabb_input = {};
        aabb_input.type = OPTIX_BUILD_INPUT_TYPE_CUSTOM_PRIMITIVES;
        aabb_input.customPrimitiveArray.aabbBuffers = &d_aabb;
        aabb_input.customPrimitiveArray.flags = aabb_input_flags;
        aabb_input.customPrimitiveArray.numSbtRecords = 1;
        aabb_input.customPrimitiveArray.numPrimitives = 1;
        aabb_input.customPrimitiveArray.sbtIndexOffsetBuffer = 0;  // d_sbt_index;
        aabb_input.customPrimitiveArray.sbtIndexOffsetSizeInBytes = sizeof(uint32_t);
        aabb_input.customPrimitiveArray.sbtIndexOffsetStrideInBytes = sizeof(uint32_t);
        aabb_input.customPrimitiveArray.primitiveIndexOffset = 0;

        OptixAccelBuildOptions accel_options = {
            OPTIX_BUILD_FLAG_ALLOW_COMPACTION,  // buildFlags
            OPTIX_BUILD_OPERATION_BUILD         // operation
        };

        // building cylinder GAS
        OptixAccelBufferSizes gas_buffer_sizes;
        CUdeviceptr d_temp_buffer_gas;
        OPTIX_ERROR_CHECK(optixAccelComputeMemoryUsage(m_context, &accel_options, &aabb_input, 1, &gas_buffer_sizes));
        CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_temp_buffer_gas), gas_buffer_sizes.tempSizeInBytes));
        CUdeviceptr d_buffer_temp_output_gas_and_compacted_size;

        size_t compactedSizeOffset = (size_t)((gas_buffer_sizes.outputSizeInBytes + 8ull - 1 / 8ull) * 8ull);

        CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_buffer_temp_output_gas_and_compacted_size),
                                    compactedSizeOffset + 8));
        OptixAccelEmitDesc emitProperty = {};
        emitProperty.type = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE;
        emitProperty.result = (CUdeviceptr)((char*)d_buffer_temp_output_gas_and_compacted_size + compactedSizeOffset);

        m_gas_handles.emplace_back();
        m_gas_buffers.emplace_back();
        m_cyl_gas_id = static_cast<unsigned int>(m_gas_handles.size() - 1);

        OPTIX_ERROR_CHECK(optixAccelBuild(m_context, 0, &accel_options, &aabb_input, 1, d_temp_buffer_gas,
                                          gas_buffer_sizes.tempSizeInBytes, d_buffer_temp_output_gas_and_compacted_size,
                                          gas_buffer_sizes.outputSizeInBytes, &m_gas_handles[m_cyl_gas_id],
                                          &emitProperty, 1));
        CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void**>(d_temp_buffer_gas)));

        size_t compacted_gas_size;
        CUDA_ERROR_CHECK(cudaMemcpy(&compacted_gas_size, reinterpret_cast<void*>(emitProperty.result), sizeof(size_t),
                                    cudaMemcpyDeviceToHost));

        if (compacted_gas_size < gas_buffer_sizes.outputSizeInBytes) {
            CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&m_gas_buffers[m_cyl_gas_id]), compacted_gas_size));

            // use handle as input and output
            OPTIX_ERROR_CHECK(optixAccelCompact(m_context, 0, m_gas_handles[m_cyl_gas_id], m_gas_buffers[m_cyl_gas_id],
                                                compacted_gas_size, &m_gas_handles[m_cyl_gas_id]));
            CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void*>(d_buffer_temp_output_gas_and_compacted_size)));
        } else {
            m_gas_buffers[m_cyl_gas_id] = d_buffer_temp_output_gas_and_compacted_size;
        }
        CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void*>(d_aabb)));
        m_cyl_inst = true;
    }

    AddGenericObject(mat_id, body, asset_frame, scale, m_gas_handles[m_cyl_gas_id]);
}

unsigned int ChOptixGeometry::AddRigidMesh(CUdeviceptr d_vertices,
                                           CUdeviceptr d_indices,
                                           std::shared_ptr<ChTriangleMeshShape> mesh_shape,
                                           std::shared_ptr<ChBody> body,
                                           ChFrame<double> asset_frame,
                                           ChVector<double> scale,
                                           unsigned int mat_id) {
    // std::cout << "Adding rigid mesh to optix!\n";
    bool mesh_found = false;
    unsigned int mesh_gas_id = 0;
    for (int i = 0; i < m_known_meshes.size(); i++) {
        if (d_vertices == std::get<0>(m_known_meshes[i])) {
            mesh_found = true;
            mesh_gas_id = std::get<1>(m_known_meshes[i]);

            // std::cout << "Found an existing mesh we should use. Will use the following parameters:\n";
            // std::cout << "mesh_gas_id: " << mesh_gas_id << std::endl;
            // std::cout << "mat_id: " << mat_id << std::endl;
            break;
        }
    }

    if (!mesh_found) {
        mesh_gas_id = BuildTrianglesGAS(mesh_shape, d_vertices, d_indices);
        // push this mesh to our known meshes
        m_known_meshes.push_back(std::make_tuple(d_vertices, mesh_gas_id));
        // std::cout << "Created a mesh from scratch:\n";
    }

    AddGenericObject(mat_id, body, asset_frame, scale, m_gas_handles[mesh_gas_id]);

    return mesh_gas_id;
}

void ChOptixGeometry::AddDeformableMesh(CUdeviceptr d_vertices,
                                        CUdeviceptr d_indices,
                                        std::shared_ptr<ChTriangleMeshShape> mesh_shape,
                                        std::shared_ptr<ChBody> body,
                                        ChFrame<double> asset_frame,
                                        ChVector<double> scale,
                                        unsigned int mat_id) {
    unsigned int mesh_gas_id = BuildTrianglesGAS(mesh_shape, d_vertices, d_indices, false);
    AddGenericObject(mat_id, body, asset_frame, scale, m_gas_handles[mesh_gas_id]);
    m_deformable_meshes.push_back(std::make_tuple(mesh_shape, d_vertices, d_indices, mesh_gas_id));
}

void ChOptixGeometry::UpdateDeformableMeshes() {
    for (int i = 0; i < m_deformable_meshes.size(); i++) {
        std::shared_ptr<ChTriangleMeshShape> mesh_shape = std::get<0>(m_deformable_meshes[i]);
        CUdeviceptr d_vertices = std::get<1>(m_deformable_meshes[i]);
        CUdeviceptr d_indices = std::get<2>(m_deformable_meshes[i]);
        unsigned int gas_id = std::get<3>(m_deformable_meshes[i]);

        // perform a rebuild of the triange acceleration structure
        BuildTrianglesGAS(mesh_shape, d_vertices, d_indices, false, true, gas_id);
    }
}

unsigned int ChOptixGeometry::BuildTrianglesGAS(std::shared_ptr<ChTriangleMeshShape> mesh_shape,
                                                CUdeviceptr d_vertices,
                                                CUdeviceptr d_indices,
                                                bool compact_no_update,
                                                bool rebuild,
                                                unsigned int gas_id) {
    auto mesh = mesh_shape->GetMesh();

    OptixAccelBuildOptions accel_options = {OPTIX_BUILD_FLAG_ALLOW_COMPACTION, OPTIX_BUILD_OPERATION_BUILD};
    if (!compact_no_update) {
        accel_options.buildFlags = OPTIX_BUILD_FLAG_ALLOW_UPDATE;
        // accel_options.buildFlags = OPTIX_BUILD_FLAG_NONE;
    }

    // uint32_t triangle_flags[] = {OPTIX_GEOMETRY_FLAG_NONE};
    uint32_t triangle_flags[] = {OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT};
    OptixBuildInput mesh_input = {};
    mesh_input.type = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;

    // vertex data/params
    mesh_input.triangleArray.vertexBuffers = &d_vertices;
    mesh_input.triangleArray.numVertices = static_cast<unsigned int>(mesh->getCoordsVertices().size());
    mesh_input.triangleArray.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
    // TODO: if vertices get padded to float4, this need to reflect that
    mesh_input.triangleArray.vertexStrideInBytes = sizeof(float4);  // sizeof(float3);

    // index data/params
    mesh_input.triangleArray.indexBuffer = d_indices;
    mesh_input.triangleArray.numIndexTriplets = static_cast<unsigned int>(mesh->getIndicesVertexes().size());
    mesh_input.triangleArray.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
    // TODO: if vertices get padded to uint4, this need to reflect that
    mesh_input.triangleArray.indexStrideInBytes = sizeof(uint4);  // sizeof(uint3);

    mesh_input.triangleArray.flags = triangle_flags;
    mesh_input.triangleArray.numSbtRecords = 1;
    mesh_input.triangleArray.sbtIndexOffsetBuffer = 0;

    // build the triangle acceleration structure
    OptixAccelBufferSizes gas_buffer_sizes;
    OPTIX_ERROR_CHECK(optixAccelComputeMemoryUsage(m_context, &accel_options, &mesh_input,
                                                   1,  // num_build_inputs
                                                   &gas_buffer_sizes));
    CUdeviceptr d_temp_buffer_gas;
    CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_temp_buffer_gas), gas_buffer_sizes.tempSizeInBytes));

    unsigned int mesh_gas_id = gas_id;
    if (!rebuild) {
        m_gas_handles.emplace_back();
        m_gas_buffers.emplace_back();
        mesh_gas_id = static_cast<unsigned int>(m_gas_handles.size() - 1);
    }

    if (compact_no_update) {
        size_t compactedSizeOffset = (size_t)((gas_buffer_sizes.outputSizeInBytes + 8ull - 1 / 8ull) * 8ull);

        CUdeviceptr d_buffer_temp_output_gas_and_compacted_size;
        CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&d_buffer_temp_output_gas_and_compacted_size),
                                    compactedSizeOffset + 8));
        OptixAccelEmitDesc emitProperty = {};
        emitProperty.type = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE;
        emitProperty.result = (CUdeviceptr)((char*)d_buffer_temp_output_gas_and_compacted_size + compactedSizeOffset);
        OPTIX_ERROR_CHECK(optixAccelBuild(m_context, 0, &accel_options, &mesh_input, 1, d_temp_buffer_gas,
                                          gas_buffer_sizes.tempSizeInBytes, d_buffer_temp_output_gas_and_compacted_size,
                                          gas_buffer_sizes.outputSizeInBytes, &m_gas_handles[mesh_gas_id],
                                          &emitProperty, 1));
        CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void**>(d_temp_buffer_gas)));

        size_t compacted_gas_size;
        CUDA_ERROR_CHECK(cudaMemcpy(&compacted_gas_size, reinterpret_cast<void*>(emitProperty.result), sizeof(size_t),
                                    cudaMemcpyDeviceToHost));

        if (compacted_gas_size < gas_buffer_sizes.outputSizeInBytes) {
            if (rebuild && m_gas_buffers[mesh_gas_id]) {
                CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void*>(m_gas_buffers[mesh_gas_id])));
            }
            CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&m_gas_buffers[mesh_gas_id]), compacted_gas_size));

            // use handle as input and output
            OPTIX_ERROR_CHECK(optixAccelCompact(m_context, 0, m_gas_handles[mesh_gas_id], m_gas_buffers[mesh_gas_id],
                                                compacted_gas_size, &m_gas_handles[mesh_gas_id]));
            CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void*>(d_buffer_temp_output_gas_and_compacted_size)));
        } else {
            m_gas_buffers[mesh_gas_id] = d_buffer_temp_output_gas_and_compacted_size;
        }

    } else {
        if (!rebuild) {
            CUDA_ERROR_CHECK(
                cudaMalloc(reinterpret_cast<void**>(&m_gas_buffers[mesh_gas_id]), gas_buffer_sizes.outputSizeInBytes));
        }
        OPTIX_ERROR_CHECK(optixAccelBuild(m_context, 0, &accel_options, &mesh_input, 1, d_temp_buffer_gas,
                                          gas_buffer_sizes.tempSizeInBytes, m_gas_buffers[mesh_gas_id],
                                          gas_buffer_sizes.outputSizeInBytes, &m_gas_handles[mesh_gas_id], nullptr, 0));
        CUDA_ERROR_CHECK(cudaFree(reinterpret_cast<void**>(d_temp_buffer_gas)));
    }

    return mesh_gas_id;
}

OptixTraversableHandle ChOptixGeometry::CreateRootStructure() {
    // std::cout << "Creating root structure with " << m_obj_mat_ids.size() << " objects\n";

    // malloc the gpu memory for motion transform
    CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&md_motion_transforms),
                                m_motion_transforms.size() * sizeof(OptixMatrixMotionTransform)));
    CUDA_ERROR_CHECK(cudaMemcpy(reinterpret_cast<void*>(md_motion_transforms), m_motion_transforms.data(),
                                m_motion_transforms.size() * sizeof(OptixMatrixMotionTransform),
                                cudaMemcpyHostToDevice));

    // loop through and make motion transforms into handles
    // std::cout << "Creating motion handles: " << m_motion_transforms.size()
    //           << " with size=" << sizeof(OptixMatrixMotionTransform) << std::endl;

    for (int i = 0; i < m_motion_transforms.size(); i++) {
        OPTIX_ERROR_CHECK(optixConvertPointerToTraversableHandle(
            m_context, md_motion_transforms + i * sizeof(OptixMatrixMotionTransform),
            OPTIX_TRAVERSABLE_TYPE_MATRIX_MOTION_TRANSFORM, &m_motion_handles[i]));
    }

    // update and build the instances
    size_t instance_size_in_bytes = sizeof(OptixInstance) * m_obj_mat_ids.size();
    m_instances = std::vector<OptixInstance>(m_obj_mat_ids.size());
    memset(m_instances.data(), 0, instance_size_in_bytes);
    CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&md_instances), instance_size_in_bytes));

    OptixBuildInput instance_input = {};
    instance_input.type = OPTIX_BUILD_INPUT_TYPE_INSTANCES;
    // instance_input.type = OPTIX_BUILD_INPUT_TYPE_INSTANCE_POINTERS;
    instance_input.instanceArray.instances = md_instances;
    instance_input.instanceArray.numInstances = static_cast<unsigned int>(m_instances.size());
    OptixAccelBuildOptions accel_options = {};
    // accel_options.buildFlags = OPTIX_BUILD_FLAG_NONE;
    // accel_options.buildFlags = OPTIX_BUILD_FLAG_ALLOW_UPDATE;
    // accel_options.buildFlags = OPTIX_BUILD_FLAG_PREFER_FAST_TRACE | OPTIX_BUILD_FLAG_ALLOW_UPDATE;
    accel_options.buildFlags = OPTIX_BUILD_FLAG_PREFER_FAST_TRACE;  // | OPTIX_BUILD_FLAG_ALLOW_UPDATE;
    accel_options.operation = OPTIX_BUILD_OPERATION_BUILD;
    accel_options.motionOptions.numKeys = 2;               // default at start
    accel_options.motionOptions.timeBegin = m_start_time;  // default at start
    accel_options.motionOptions.timeEnd = m_end_time;      // default at start
    accel_options.motionOptions.flags = OPTIX_MOTION_FLAG_NONE;

    OptixAccelBufferSizes ias_buffer_sizes;
    OPTIX_ERROR_CHECK(optixAccelComputeMemoryUsage(m_context, &accel_options, &instance_input,
                                                   1,  // num build inputs
                                                   &ias_buffer_sizes));
    md_root_temp_buffer_size = ias_buffer_sizes.tempSizeInBytes;
    md_root_output_buffer_size = ias_buffer_sizes.outputSizeInBytes;

    CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&md_root_temp_buffer), ias_buffer_sizes.tempSizeInBytes));
    CUDA_ERROR_CHECK(cudaMalloc(reinterpret_cast<void**>(&md_root_output_buffer), ias_buffer_sizes.outputSizeInBytes));

    // pack the data for each instance
    for (int i = 0; i < m_instances.size(); i++) {
        const float t[12] = {1.f, 0.f, 0.f, 0.f,  //
                             0.f, 1.f, 0.f, 0.f,  //
                             0.f, 0.f, 1.f, 0.f};
        m_instances[i].traversableHandle = m_motion_handles[i];
        m_instances[i].flags = OPTIX_INSTANCE_FLAG_DISABLE_TRANSFORM |
                               OPTIX_INSTANCE_FLAG_DISABLE_ANYHIT;  // | OPTIX_INSTANCE_FLAG_FLIP_TRIANGLE_FACING;
        // m_instances[i].flags = OPTIX_INSTANCE_FLAG_NONE;
        m_instances[i].instanceId = i;
        m_instances[i].sbtOffset = m_obj_mat_ids[i];
        m_instances[i].visibilityMask = 1;
        memcpy(m_instances[i].transform, t, sizeof(float) * 12);
    }

    CUDA_ERROR_CHECK(cudaMemcpy(reinterpret_cast<void*>(md_instances), m_instances.data(), instance_size_in_bytes,
                                cudaMemcpyHostToDevice));
    OPTIX_ERROR_CHECK(optixAccelBuild(m_context,
                                      0,  // CUDA stream
                                      &accel_options, &instance_input,
                                      1,  // num build inputs
                                      md_root_temp_buffer, ias_buffer_sizes.tempSizeInBytes, md_root_output_buffer,
                                      ias_buffer_sizes.outputSizeInBytes, &m_root,
                                      nullptr,  // emitted property list
                                      0         // num emitted properties
                                      ));
                                      
    return m_root;
}

// rebuilding the structure without creating anything new
void ChOptixGeometry::RebuildRootStructure() {
    
    m_end_time = m_end_time > (m_start_time+1e-2)
                     ? m_end_time
                     : m_end_time + 1e-2;  // need to ensure start time is at least slightly after end time

    for (int i = 0; i < m_motion_transforms.size(); i++) {
        // update the motion transforms
        const ChFrame<double> f_start = m_obj_body_frames_start[i] * m_obj_asset_frames[i];
        const ChVector<double> pos_start = f_start.GetPos();
        const ChMatrix33<double> rot_mat_start = f_start.Amatrix;

        const ChFrame<double> f_end = m_obj_body_frames_end[i] * m_obj_asset_frames[i];
        const ChVector<double> pos_end = f_end.GetPos();
        const ChMatrix33<double> rot_mat_end = f_end.Amatrix;

        // m_motion_transforms[i].motionOptions.numKeys = 2;               // TODO: would we need more than this?
        m_motion_transforms[i].motionOptions.timeBegin = m_start_time;  // default at start, will be updated
        m_motion_transforms[i].motionOptions.timeEnd = m_end_time;      // default at start, will be updated
        // m_motion_transforms[i].motionOptions.flags = OPTIX_MOTION_FLAG_NONE;

        // ChVector<> old_pos = {m_motion_transforms[i].transform[0][3], m_motion_transforms[i].transform[0][7],
        //                       m_motion_transforms[i].transform[0][11]};
        // ChVector<> diff = old_pos - f_start.GetPos();
        // max_pos_diff = std::max(max_pos_diff, diff.Length());

        GetT3x4FromSRT(m_obj_scales[i], rot_mat_start, pos_start, m_motion_transforms[i].transform[0]);
        GetT3x4FromSRT(m_obj_scales[i], rot_mat_end, pos_end, m_motion_transforms[i].transform[1]);
    }

    // std::cout << "Motion transforms: " << m_motion_transforms.size() << std::endl;
    // std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    CUDA_ERROR_CHECK(cudaMemcpy(reinterpret_cast<void*>(md_motion_transforms), m_motion_transforms.data(),
                                m_motion_transforms.size() * sizeof(OptixMatrixMotionTransform),
                                cudaMemcpyHostToDevice));

    // std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    // std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();

    // std::cout << "Rebuilding root structure with " << m_instances.size() << " objects\n";
    OptixBuildInput instance_input = {};
    instance_input.type = OPTIX_BUILD_INPUT_TYPE_INSTANCES;
    instance_input.instanceArray.instances = md_instances;
    instance_input.instanceArray.numInstances = static_cast<unsigned int>(m_instances.size());
    OptixAccelBuildOptions accel_options = {};
    // accel_options.buildFlags = OPTIX_BUILD_FLAG_NONE;
    accel_options.buildFlags = OPTIX_BUILD_FLAG_PREFER_FAST_TRACE;// | OPTIX_BUILD_FLAG_ALLOW_UPDATE;
    // accel_options.buildFlags = OPTIX_BUILD_FLAG_PREFER_FAST_TRACE;
    // accel_options.buildFlags = OPTIX_BUILD_FLAG_PREFER_FAST_BUILD;
    // accel_options.buildFlags = OPTIX_BUILD_FLAG_ALLOW_UPDATE;
    // if (max_pos_diff > pos_diff_threshold) {
    accel_options.operation = OPTIX_BUILD_OPERATION_BUILD;
    // std::cout << "Building scene\n";
    // } else {
    //     accel_options.operation = OPTIX_BUILD_OPERATION_UPDATE;
    //     std::cout << "Updating scene\n";
    // }

    accel_options.motionOptions.numKeys = 2;  // default at start TODO: should this always be 2?
    accel_options.motionOptions.timeBegin = m_start_time;
    accel_options.motionOptions.timeEnd = m_end_time;
    accel_options.motionOptions.flags = OPTIX_MOTION_FLAG_NONE;
    // accel_options.operation = OPTIX_BUILD_OPERATION_UPDATE;

    OptixAccelBufferSizes ias_buffer_sizes;
    OPTIX_ERROR_CHECK(optixAccelComputeMemoryUsage(m_context, &accel_options, &instance_input,
                                                   1,  // num build inputs
                                                   &ias_buffer_sizes));
    OPTIX_ERROR_CHECK(optixAccelBuild(m_context,
                                      0,  // CUDA stream
                                      &accel_options, &instance_input,
                                      1,  // num build inputs
                                      md_root_temp_buffer, ias_buffer_sizes.tempSizeInBytes, md_root_output_buffer,
                                      ias_buffer_sizes.outputSizeInBytes, &m_root,
                                      nullptr,  // emitted property list
                                      0         // num emitted properties
                                      ));

    cudaDeviceSynchronize();
    // std::chrono::high_resolution_clock::time_point t4 = std::chrono::high_resolution_clock::now();

    // std::cout << "Rebuilt root acceleration structure, addr = " << m_root << std::endl;
    // std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
    // std::cout << "Transform pack time on cpu: " << wall_time.count() << std::endl;
    // wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    // std::cout << "Transform cudamemcpy time: " << wall_time.count() << std::endl;
    // wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2);
    // std::cout << "Instance cudamemcpy time: " << wall_time.count() << std::endl;
    // wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3);
    // std::cout << "Optix rebuild time: " << wall_time.count() << std::endl;
    // std::cout << "Scene rebuild complete" << std::endl;
    // std::cout << "Rebuilt root acceleration structure, addr = " << m_root << std::endl;
}

void ChOptixGeometry::UpdateBodyTransformsStart(float t_start, float t_target_end) {
    // std::cout << "Updating body transforms for start keyframe\n";
    std::vector<ChFrame<double>> frames = std::vector<ChFrame<double>>(m_bodies.size());
    for (int i = 0; i < frames.size(); i++) {
        frames[i] = m_bodies[i]->GetFrame_REF_to_abs();
    }
    auto keyframe = std::make_tuple(t_start, t_target_end, std::move(frames));
    m_obj_body_frames_start_tmps.push_back(keyframe);
}

void ChOptixGeometry::UpdateBodyTransformsEnd(float t_end) {
    // std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    // pack the end frame
    m_end_time = t_end;
    for (int i = 0; i < m_bodies.size(); i++) {
        m_obj_body_frames_end[i] = m_bodies[i]->GetFrame_REF_to_abs();
    }

    // std::cout << "Loaded the set of end transforms for time=" << t_end << std::endl;

    // std::cout << "Start keyframes to begin with: " << m_obj_body_frames_start_tmps.size() << std::endl;
    // since and end has been packed, move the oldest start_tmp to start (first will always be the earliest)
    for (int i = 0; i < m_obj_body_frames_start_tmps.size(); i++) {
        float target_end = std::get<1>(m_obj_body_frames_start_tmps[i]);
        if (target_end <= t_end) {
            m_start_time = std::get<0>(m_obj_body_frames_start_tmps[i]);
            m_obj_body_frames_start = std::move(std::get<2>(m_obj_body_frames_start_tmps[i]));
            // std::cout << "Found the first start transforms with start time=" << m_start_time
            //           << ", target end time=" << target_end << std::endl;
            break;  // we can break since the first one that meets our criteria will be the one with the earliest
                    // start time
        }
    }

    // clear out and starts that have target end time <= this end time
    int i = 0;
    while (i < m_obj_body_frames_start_tmps.size()) {
        float target_end = std::get<1>(m_obj_body_frames_start_tmps[i]);
        if (target_end <= t_end) {
            m_obj_body_frames_start_tmps.erase(m_obj_body_frames_start_tmps.begin() + i);
            i--;
        }
        i++;
    }

    // std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();

    // // std::cout << "Created root acceleration structure, addr = " << m_root << std::endl;
    // std::chrono::duration<double> wall_time = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    // std::cout << "Updating End transforms on main thread: " << wall_time.count() << std::endl;

    // std::cout << "Start keyframes left: " << m_obj_body_frames_start_tmps.size() << std::endl;

    // std::cout << "Start frames. t=" << m_start_time << ", size=" << m_obj_body_frames_start.size() << std::endl;
    // std::cout << "End frames. t=" << m_end_time << ", size=" << m_obj_body_frames_end.size() << std::endl;
}

void ChOptixGeometry::GetT3x4FromSRT(const ChVector<double>& s,
                                     const ChMatrix33<double>& a,
                                     const ChVector<double>& b,
                                     float* t) {
    t[0] = (float)(s.x() * a(0));
    t[1] = (float)(s.y() * a(1));
    t[2] = (float)(s.z() * a(2));
    t[3] = (float)b.x();

    t[4] = (float)(s.x() * a(3));
    t[5] = (float)(s.y() * a(4));
    t[6] = (float)(s.z() * a(5));
    t[7] = (float)b.y();

    t[8] = (float)(s.x() * a(6));
    t[9] = (float)(s.y() * a(7));
    t[10] = (float)(s.z() * a(8));
    t[11] = (float)b.z();
}
void ChOptixGeometry::GetInvT3x4FromSRT(const ChVector<double>& s,
                                        const ChMatrix33<double>& a,
                                        const ChVector<double>& b,
                                        float* t) {
    t[0] = (float)(a(0) / s.x());
    t[1] = (float)(a(3) / s.x());
    t[2] = (float)(a(6) / s.x());
    t[3] = (float)(-a(0) * b.x() - a(3) * b.y() - a(6) * b.z() / s.x());

    t[4] = (float)(a(1) / s.y());
    t[5] = (float)(a(4) / s.y());
    t[6] = (float)(a(7) / s.y());
    t[7] = (float)(-a(1) * b.x() - a(4) * b.y() - a(7) * b.z()) / s.y();

    t[8] = (float)(a(2) / s.z());
    t[9] = (float)(a(5) / s.z());
    t[10] = (float)(a(8) / s.z());
    t[11] = (float)(-a(2) * b.x() - a(5) * b.y() - a(8) * b.z()) / s.z();
}

}  // namespace sensor
}  // namespace chrono
