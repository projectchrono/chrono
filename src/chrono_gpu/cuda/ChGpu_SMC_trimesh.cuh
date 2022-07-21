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
// Authors: Conlain Kelly, Nic Olsen, Ruochun Zhang, Dan Negrut
// =============================================================================

#pragma once

#include "chrono_gpu/ChGpuDefines.h"
#include "chrono_gpu/physics/ChSystemGpuMesh_impl.h"

#include "chrono_gpu/cuda/ChGpuHelpers.cuh"
#include "chrono_gpu/cuda/ChGpuCUDAalloc.hpp"

// these define things that mess with cub
#include "chrono_gpu/cuda/ChGpuCollision.cuh"
#include "chrono_gpu/cuda/ChGpuBoxTriangle.cuh"
#include "chrono_gpu/cuda/ChCudaMathUtils.cuh"

using chrono::gpu::ChSystemGpu_impl;
using chrono::gpu::ChSystemGpuMesh_impl;

/// @addtogroup gpu_cuda
/// @{

// Triangle bounding box will be enlarged by 1/SAFETY_PARAM, ensuring triangles lie between 2 SDs
// are getting some love
static const int SAFETY_PARAM = 1000;

/// Point is in the LRF, rot_mat rotates LRF to GRF, pos translates LRF to GRF
/// LRF: local reference frame
/// GRF: global reference frame
template <class IN_T, class IN_T3, class OUT_T3 = IN_T3>
__device__ OUT_T3 apply_frame_transform(const IN_T3& point, const IN_T* pos, const IN_T* rot_mat) {
    OUT_T3 result;

    // Apply rotation matrix to point
    result.x = rot_mat[0] * point.x + rot_mat[1] * point.y + rot_mat[2] * point.z;
    result.y = rot_mat[3] * point.x + rot_mat[4] * point.y + rot_mat[5] * point.z;
    result.z = rot_mat[6] * point.x + rot_mat[7] * point.y + rot_mat[8] * point.z;

    // Apply translation
    result.x += pos[0];
    result.y += pos[1];
    result.z += pos[2];

    return result;
}

/// Convert position vector from user units to scaled units.
template <class T3>
__device__ void convert_pos_UU2SU(T3& pos, ChSystemGpu_impl::GranParamsPtr gran_params) {
    pos.x /= gran_params->LENGTH_UNIT;
    pos.y /= gran_params->LENGTH_UNIT;
    pos.z /= gran_params->LENGTH_UNIT;
}

/// Takes in a triangle ID and figures out an SD AABB for broadphase use
__inline__ __device__ void triangle_figureOutSDBox(const float3& vA,
                                                   const float3& vB,
                                                   const float3& vC,
                                                   int* L,
                                                   int* U,
                                                   ChSystemGpu_impl::GranParamsPtr gran_params) {
    int64_t min_pt_x = MIN(vA.x, MIN(vB.x, vC.x));
    int64_t min_pt_y = MIN(vA.y, MIN(vB.y, vC.y));
    int64_t min_pt_z = MIN(vA.z, MIN(vB.z, vC.z));

    // Enlarge bounding box
    min_pt_x -= gran_params->SD_size_X_SU / SAFETY_PARAM;
    min_pt_y -= gran_params->SD_size_Y_SU / SAFETY_PARAM;
    min_pt_z -= gran_params->SD_size_Z_SU / SAFETY_PARAM;

    int64_t max_pt_x = MAX(vA.x, MAX(vB.x, vC.x));
    int64_t max_pt_y = MAX(vA.y, MAX(vB.y, vC.y));
    int64_t max_pt_z = MAX(vA.z, MAX(vB.z, vC.z));

    max_pt_x += gran_params->SD_size_X_SU / SAFETY_PARAM;
    max_pt_y += gran_params->SD_size_Y_SU / SAFETY_PARAM;
    max_pt_z += gran_params->SD_size_Z_SU / SAFETY_PARAM;

    int3 tmp = pointSDTriplet(min_pt_x, min_pt_y, min_pt_z, gran_params);
    L[0] = tmp.x;
    L[1] = tmp.y;
    L[2] = tmp.z;

    tmp = pointSDTriplet(max_pt_x, max_pt_y, max_pt_z, gran_params);
    U[0] = tmp.x;
    U[1] = tmp.y;
    U[2] = tmp.z;
}

/// Takes in a triangle's position in UU and finds out how many SDs it touches.
/// Triangle broadphase is done in float by applying the frame transform
/// and then converting the GRF position to SU
inline __device__ unsigned int triangle_countTouchedSDs(unsigned int triangleID,
                                                        const ChSystemGpuMesh_impl::TriangleSoupPtr triangleSoup,
                                                        ChSystemGpu_impl::GranParamsPtr gran_params,
                                                        ChSystemGpuMesh_impl::MeshParamsPtr tri_params) {
    float3 vA, vB, vC;

    // Transform LRF to GRF
    unsigned int fam = triangleSoup->triangleFamily_ID[triangleID];
    vA = apply_frame_transform<float, float3>(triangleSoup->node1[triangleID], tri_params->fam_frame_broad[fam].pos,
                                              tri_params->fam_frame_broad[fam].rot_mat);
    vB = apply_frame_transform<float, float3>(triangleSoup->node2[triangleID], tri_params->fam_frame_broad[fam].pos,
                                              tri_params->fam_frame_broad[fam].rot_mat);
    vC = apply_frame_transform<float, float3>(triangleSoup->node3[triangleID], tri_params->fam_frame_broad[fam].pos,
                                              tri_params->fam_frame_broad[fam].rot_mat);

    // Convert UU to SU
    convert_pos_UU2SU<float3>(vA, gran_params);
    convert_pos_UU2SU<float3>(vB, gran_params);
    convert_pos_UU2SU<float3>(vC, gran_params);

    // bottom-left and top-right corners
    int L[3];
    int U[3];
    triangle_figureOutSDBox(vA, vB, vC, L, U, gran_params);
    // Case 1: All vetices are in the same SD
    if (L[0] == U[0] && L[1] == U[1] && L[2] == U[2]) {
        unsigned int currSD = SDTripletID(L, gran_params);
        if (currSD != NULL_CHGPU_ID) {
            return 1;
        } else {
            // TODO optimize me?
            return 0;
        }
    }

    unsigned int n_axes_diff = 0;  // Count axes that have different SD bounds
    unsigned int axes_diff;        // axis of variation (if only one)

    for (int i = 0; i < 3; i++) {
        if (L[i] != U[i]) {
            axes_diff = i;  // If there are more than one, this won't be used anyway
            n_axes_diff++;
        }
    }
    unsigned int numSDsTouched = 0;

    // Case 2: Triangle lies in a Nx1x1, 1xNx1, or 1x1xN block of SDs
    if (n_axes_diff == 1) {
        // add one since it's in each of these SDs
        int SD_i[3] = {L[0], L[1], L[2]};  // start at 'bottom' and move up
        for (int i = L[axes_diff]; i <= U[axes_diff]; i++) {
            SD_i[axes_diff] = i;  // current SD index along this direction
            unsigned int currSD = SDTripletID(SD_i, gran_params);
            if (currSD != NULL_CHGPU_ID) {
                numSDsTouched++;
            }
        }
        return numSDsTouched;
    }

    // Case 3: Triangle spans more than one dimension of spheresTouchingThisSD
    // TODO the halfSize is inflated a bit to allow detection of triangle facets lie right
    // between SDs, is it a good practice? (we know that without it, the SD touching detection is bugged)
    float SDcenter[3];
    float SDhalfSizes[3];
    for (int i = L[0]; i <= U[0]; i++) {
        for (int j = L[1]; j <= U[1]; j++) {
            for (int k = L[2]; k <= U[2]; k++) {
                SDhalfSizes[0] = (gran_params->SD_size_X_SU + gran_params->SD_size_X_SU / SAFETY_PARAM) / 2;
                SDhalfSizes[1] = (gran_params->SD_size_Y_SU + gran_params->SD_size_Y_SU / SAFETY_PARAM) / 2;
                SDhalfSizes[2] = (gran_params->SD_size_Z_SU + gran_params->SD_size_Z_SU / SAFETY_PARAM) / 2;

                SDcenter[0] = gran_params->BD_frame_X + (int64_t)(i * 2 + 1) * (int64_t)gran_params->SD_size_X_SU / 2;
                SDcenter[1] = gran_params->BD_frame_Y + (int64_t)(j * 2 + 1) * (int64_t)gran_params->SD_size_Y_SU / 2;
                SDcenter[2] = gran_params->BD_frame_Z + (int64_t)(k * 2 + 1) * (int64_t)gran_params->SD_size_Z_SU / 2;

                if (check_TriangleBoxOverlap(SDcenter, SDhalfSizes, vA, vB, vC)) {
                    unsigned int currSD = SDTripletID(i, j, k, gran_params);
                    if (currSD != NULL_CHGPU_ID) {
                        numSDsTouched++;
                    }
                }
            }
        }
    }
    return numSDsTouched;
}

/// Takes in a triangle's position in UU and finds out what SDs it touches.
/// Triangle broadphase is done in float by applying the frame transform
/// and then converting the GRF position to SU
inline __device__ void triangle_figureOutTouchedSDs(unsigned int triangleID,
                                                    const ChSystemGpuMesh_impl::TriangleSoupPtr triangleSoup,
                                                    unsigned int* touchedSDs,
                                                    const ChSystemGpu_impl::GranParamsPtr& gran_params,
                                                    const ChSystemGpuMesh_impl::MeshParamsPtr& tri_params) {
    float3 vA, vB, vC;

    // Transform LRF to GRF
    unsigned int fam = triangleSoup->triangleFamily_ID[triangleID];
    vA = apply_frame_transform<float, float3>(triangleSoup->node1[triangleID], tri_params->fam_frame_broad[fam].pos,
                                              tri_params->fam_frame_broad[fam].rot_mat);
    vB = apply_frame_transform<float, float3>(triangleSoup->node2[triangleID], tri_params->fam_frame_broad[fam].pos,
                                              tri_params->fam_frame_broad[fam].rot_mat);
    vC = apply_frame_transform<float, float3>(triangleSoup->node3[triangleID], tri_params->fam_frame_broad[fam].pos,
                                              tri_params->fam_frame_broad[fam].rot_mat);

    // Convert UU to SU
    convert_pos_UU2SU<float3>(vA, gran_params);
    convert_pos_UU2SU<float3>(vB, gran_params);
    convert_pos_UU2SU<float3>(vC, gran_params);

    // bottom-left and top-right corners
    int L[3];
    int U[3];
    triangle_figureOutSDBox(vA, vB, vC, L, U, gran_params);

    // TODO modularize more code
    // Case 1: All vetices are in the same SD
    if (L[0] == U[0] && L[1] == U[1] && L[2] == U[2]) {
        // if we get here and don't have a valid SD, this should be an error
        unsigned int currSD = SDTripletID(L, gran_params);
        if (currSD != NULL_CHGPU_ID) {
            touchedSDs[0] = currSD;
        }
        return;
    }

    unsigned int SD_count = 0;

    unsigned int n_axes_diff = 0;  // Count axes that have different SD bounds
    unsigned int axes_diff;        // axis of variation (if only one)

    for (int i = 0; i < 3; i++) {
        if (L[i] != U[i]) {
            axes_diff = i;  // If there are more than one, this won't be used anyway
            n_axes_diff++;
        }
    }

    // Case 2: Triangle lies in a Nx1x1, 1xNx1, or 1x1xN block of SDs
    if (n_axes_diff == 1) {
        int SD_i[3] = {L[0], L[1], L[2]};  // start at 'bottom' and move up
        for (int i = L[axes_diff]; i <= U[axes_diff]; i++) {
            SD_i[axes_diff] = i;  // current SD index along this direction

            unsigned int currSD = SDTripletID(SD_i, gran_params);
            if (currSD != NULL_CHGPU_ID) {
                touchedSDs[SD_count++] = currSD;
            }
        }
        return;
    }

    // Case 3: Triangle spans more than one dimension of spheresTouchingThisSD
    float SDcenter[3];
    float SDhalfSizes[3];
    for (int i = L[0]; i <= U[0]; i++) {
        for (int j = L[1]; j <= U[1]; j++) {
            for (int k = L[2]; k <= U[2]; k++) {
                SDhalfSizes[0] = (gran_params->SD_size_X_SU + gran_params->SD_size_X_SU / SAFETY_PARAM) / 2;
                SDhalfSizes[1] = (gran_params->SD_size_Y_SU + gran_params->SD_size_Y_SU / SAFETY_PARAM) / 2;
                SDhalfSizes[2] = (gran_params->SD_size_Z_SU + gran_params->SD_size_Z_SU / SAFETY_PARAM) / 2;

                SDcenter[0] = gran_params->BD_frame_X + (int64_t)(i * 2 + 1) * (int64_t)gran_params->SD_size_X_SU / 2;
                SDcenter[1] = gran_params->BD_frame_Y + (int64_t)(j * 2 + 1) * (int64_t)gran_params->SD_size_Y_SU / 2;
                SDcenter[2] = gran_params->BD_frame_Z + (int64_t)(k * 2 + 1) * (int64_t)gran_params->SD_size_Z_SU / 2;

                if (check_TriangleBoxOverlap(SDcenter, SDhalfSizes, vA, vB, vC)) {
                    unsigned int currSD = SDTripletID(i, j, k, gran_params);
                    if (currSD != NULL_CHGPU_ID) {
                        touchedSDs[SD_count++] = currSD;
                    }
                }
            }
        }
    }
}

__global__ void determineCountOfSDsTouchedByEachTriangle(
    const ChSystemGpuMesh_impl::TriangleSoupPtr d_triangleSoup,
    unsigned int* Triangle_NumSDsTouching,  //!< number of SDs touching this Triangle
    ChSystemGpu_impl::GranParamsPtr gran_params,
    ChSystemGpuMesh_impl::MeshParamsPtr mesh_params) {
    // Figure out what triangleID this thread will handle. We work with a 1D block structure and a 1D grid structure
    unsigned int myTriangleID = threadIdx.x + blockIdx.x * blockDim.x;

    if (myTriangleID < d_triangleSoup->nTrianglesInSoup) {
        Triangle_NumSDsTouching[myTriangleID] =
            triangle_countTouchedSDs(myTriangleID, d_triangleSoup, gran_params, mesh_params);
    }
}

/// <summary>
/// kernel is called to populate two arrays: one holds on to what SDs touch a triangle, and then a companion array that
/// keeps repeating the triangle ID; in the end, these two arrays will be handled by a CUB sort-by-key op
/// </summary>
/// <param name="d_triangleSoup">- the collection of triangles in this mesh soup</param>
/// <param name="Triangle_NumSDsTouching">- number of SDs touched by each triangle</param>
/// <param name="TriangleSDCompositeOffsets">- offsets in the array that say each SD what triangles touch it</param>
/// <param name="Triangle_SDsComposite">- the list of SDs touched by each triangle; triangle by triangle</param>
/// <param name="Triangle_TriIDsComposite">- array that goes hand in hand with Triangle_SDsComposite; it repeats the
/// triangle ID for a subsequent sort by key op that is performed elsewhere</param> <param name="gran_params"></param>
/// <param name="mesh_params"></param> <returns></returns>
__global__ void storeSDsTouchedByEachTriangle(const ChSystemGpuMesh_impl::TriangleSoupPtr d_triangleSoup,
                                              const unsigned int* Triangle_NumSDsTouching,
                                              const unsigned int* TriangleSDCompositeOffsets,
                                              unsigned int* Triangle_SDsComposite,
                                              unsigned int* Triangle_TriIDsComposite,
                                              ChSystemGpu_impl::GranParamsPtr gran_params,
                                              ChSystemGpuMesh_impl::MeshParamsPtr mesh_params) {
    // Figure out what triangleID this thread will handle. We work with a 1D block structure and a 1D grid structure
    unsigned int myTriangleID = threadIdx.x + blockIdx.x * blockDim.x;

    if (myTriangleID < d_triangleSoup->nTrianglesInSoup) {
        triangle_figureOutTouchedSDs(myTriangleID, d_triangleSoup,
                                     Triangle_SDsComposite + TriangleSDCompositeOffsets[myTriangleID], gran_params,
                                     mesh_params);
        unsigned int numSDsTouched = Triangle_NumSDsTouching[myTriangleID];
        unsigned int offsetInComposite = TriangleSDCompositeOffsets[myTriangleID];
        for (unsigned int i = 0; i < numSDsTouched; i++) {
            // write back this triangle ID to be sorted
            Triangle_TriIDsComposite[offsetInComposite + i] = myTriangleID;
        }
    }
}

/// <summary>
/// Upon input, pSD_numTrianglesTouching contains a bunch of zeros. Upon return, the array will reflect the fact that
/// some SDs are touched by one or more triangles
/// </summary>
/// <param name="d_SDs_touched">- list of SDs that happen to be touched by at least one triangle</param>
/// <param name="d_howManyTrianglesTouchTheTouchedSDs">- if SD is in the list above, it says how many triangles touch
/// it</param> <param name="nSDs_touchedByTriangles">- how many SDs are actually touched by at least one
/// triangle</param> <param name="pSD_numTrianglesTouching">- [in/out] array of size SDs, populated with numbre of
/// triangles touched by each SD</param> <returns></returns>
__global__ void finalizeSD_numTrianglesTouching(const unsigned int* d_SDs_touched,
                                                const unsigned int* d_howManyTrianglesTouchTheTouchedSDs,
                                                const unsigned int* nSDs_touchedByTriangles,
                                                unsigned int* pSD_numTrianglesTouching) {
    unsigned int threadID = threadIdx.x + blockIdx.x * blockDim.x;
    if (threadID < (*nSDs_touchedByTriangles)) {
        // this thread has work to do
        unsigned int whichSD = d_SDs_touched[threadID];
        unsigned int howManyTrianglesAreTouchingThisSD = d_howManyTrianglesTouchTheTouchedSDs[threadID];
        pSD_numTrianglesTouching[whichSD] = howManyTrianglesAreTouchingThisSD;
    }
}

/// @} gpu_cuda
