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
// Authors: Conlain Kelly, Nic Olsen, Dan Negrut
// =============================================================================

#pragma once

#include "chrono_granular/physics/ChGranularTriMesh.h"
#include "chrono_granular/ChGranularDefines.h"
#include "chrono_granular/physics/ChGranularHelpers.cuh"
#include "ChGranularCUDAalloc.hpp"

// these define things that mess with cub
#include "chrono_granular/physics/ChGranularCollision.cuh"
#include "chrono_granular/physics/ChGranularBoxTriangle.cuh"
#include "chrono_granular/utils/ChCudaMathUtils.cuh"
// get nicer handles to pointer names, enforce const-ness on the mesh params
typedef const chrono::granular::ChGranParams_trimesh* MeshParamsPtr;
typedef chrono::granular::ChTriangleSoup<float3>* TriangleSoupPtr;

// Triangle bounding box will be enlarged by 1/SAFETY_PARAM, ensuring triangles lie between 2 SDs
// are getting some love
const int SAFETY_PARAM = 1000;

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

template <class T3>
__device__ void convert_pos_UU2SU(T3& pos, GranParamsPtr gran_params) {
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
                                                   GranParamsPtr gran_params) {
    int3 min_pt;
    min_pt.x = MIN(vA.x, MIN(vB.x, vC.x));
    min_pt.y = MIN(vA.y, MIN(vB.y, vC.y));
    min_pt.z = MIN(vA.z, MIN(vB.z, vC.z));

    // Enlarge bounding box
    min_pt.x -= gran_params->SD_size_X_SU/SAFETY_PARAM;
    min_pt.y -= gran_params->SD_size_Y_SU/SAFETY_PARAM;
    min_pt.z -= gran_params->SD_size_Z_SU/SAFETY_PARAM;

    int3 max_pt;
    max_pt.x = MAX(vA.x, MAX(vB.x, vC.x));
    max_pt.y = MAX(vA.y, MAX(vB.y, vC.y));
    max_pt.z = MAX(vA.z, MAX(vB.z, vC.z));

    max_pt.x += gran_params->SD_size_X_SU/SAFETY_PARAM;
    max_pt.y += gran_params->SD_size_Y_SU/SAFETY_PARAM;
    max_pt.z += gran_params->SD_size_Z_SU/SAFETY_PARAM;

    int3 tmp = pointSDTriplet(min_pt.x, min_pt.y, min_pt.z, gran_params);
    L[0] = tmp.x;
    L[1] = tmp.y;
    L[2] = tmp.z;

    tmp = pointSDTriplet(max_pt.x, max_pt.y, max_pt.z, gran_params);
    U[0] = tmp.x;
    U[1] = tmp.y;
    U[2] = tmp.z;
}

/// Takes in a triangle's position in UU and finds out how many SDs it touches
/// Triangle broadphase is done in float by applying the frame transform
/// and then converting the GRF position to SU
inline __device__ unsigned int triangle_countTouchedSDs(unsigned int triangleID,
                                                        const TriangleSoupPtr triangleSoup,
                                                        GranParamsPtr gran_params,
                                                        MeshParamsPtr tri_params) {
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
        if (currSD != NULL_GRANULAR_ID) {
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
            if (currSD != NULL_GRANULAR_ID) {
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
                SDhalfSizes[0] = (gran_params->SD_size_X_SU + gran_params->SD_size_X_SU/SAFETY_PARAM) / 2;
                SDhalfSizes[1] = (gran_params->SD_size_Y_SU + gran_params->SD_size_Y_SU/SAFETY_PARAM) / 2;
                SDhalfSizes[2] = (gran_params->SD_size_Z_SU + gran_params->SD_size_Z_SU/SAFETY_PARAM) / 2;

                SDcenter[0] = gran_params->BD_frame_X + (i * 2 + 1) * gran_params->SD_size_X_SU / 2;
                SDcenter[1] = gran_params->BD_frame_Y + (j * 2 + 1) * gran_params->SD_size_Y_SU / 2;
                SDcenter[2] = gran_params->BD_frame_Z + (k * 2 + 1) * gran_params->SD_size_Z_SU / 2;

                if (check_TriangleBoxOverlap(SDcenter, SDhalfSizes, vA, vB, vC)) {
                    unsigned int currSD = SDTripletID(i, j, k, gran_params);
                    if (currSD != NULL_GRANULAR_ID) {
                        numSDsTouched++;
                    }
                }
            }
        }
    }
    return numSDsTouched;
}

/// Takes in a triangle's position in UU and finds out what SDs it touches
/// Triangle broadphase is done in float by applying the frame transform
/// and then converting the GRF position to SU
inline __device__ void triangle_figureOutTouchedSDs(unsigned int triangleID,
                                                    const TriangleSoupPtr triangleSoup,
                                                    unsigned int* touchedSDs,
                                                    GranParamsPtr gran_params,
                                                    MeshParamsPtr tri_params) {
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
        if (currSD != NULL_GRANULAR_ID) {
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
            if (currSD != NULL_GRANULAR_ID) {
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
                SDhalfSizes[0] = (gran_params->SD_size_X_SU + gran_params->SD_size_X_SU/SAFETY_PARAM) / 2;
                SDhalfSizes[1] = (gran_params->SD_size_Y_SU + gran_params->SD_size_Y_SU/SAFETY_PARAM) / 2;
                SDhalfSizes[2] = (gran_params->SD_size_Z_SU + gran_params->SD_size_Z_SU/SAFETY_PARAM) / 2;

                SDcenter[0] = gran_params->BD_frame_X + (i * 2 + 1) * gran_params->SD_size_X_SU / 2;
                SDcenter[1] = gran_params->BD_frame_Y + (j * 2 + 1) * gran_params->SD_size_Y_SU / 2;
                SDcenter[2] = gran_params->BD_frame_Z + (k * 2 + 1) * gran_params->SD_size_Z_SU / 2;

                if (check_TriangleBoxOverlap(SDcenter, SDhalfSizes, vA, vB, vC)) {
                    unsigned int currSD = SDTripletID(i, j, k, gran_params);
                    if (currSD != NULL_GRANULAR_ID) {
                        touchedSDs[SD_count++] = currSD;
                    }
                }
            }
        }
    }
}

__global__ void triangleSoup_CountSDsTouched(
    const TriangleSoupPtr d_triangleSoup,
    unsigned int* Triangle_NumSDsTouching,  //!< number of SDs touching this Triangle
    GranParamsPtr gran_params,
    MeshParamsPtr mesh_params) {
    // Figure out what triangleID this thread will handle. We work with a 1D block structure and a 1D grid structure
    unsigned int myTriangleID = threadIdx.x + blockIdx.x * blockDim.x;

    if (myTriangleID < d_triangleSoup->nTrianglesInSoup) {
        Triangle_NumSDsTouching[myTriangleID] =
            triangle_countTouchedSDs(myTriangleID, d_triangleSoup, gran_params, mesh_params);
    }
}

__global__ void triangleSoup_StoreSDsTouched(
    const TriangleSoupPtr d_triangleSoup,
    unsigned int* Triangle_NumSDsTouching,     //!< number of SDs touching this Triangle
    unsigned int* TriangleSDCompositeOffsets,  //!< number of SDs touching this Triangle
    unsigned int* Triangle_SDsComposite,       //!< number of SDs touching this Triangle
    unsigned int* Triangle_TriIDsComposite,    //!< number of SDs touching this Triangle
    GranParamsPtr gran_params,
    MeshParamsPtr mesh_params) {
    // Figure out what triangleID this thread will handle. We work with a 1D block structure and a 1D grid structure
    unsigned int myTriangleID = threadIdx.x + blockIdx.x * blockDim.x;

    if (myTriangleID < d_triangleSoup->nTrianglesInSoup) {
        triangle_figureOutTouchedSDs(myTriangleID, d_triangleSoup,
                                     Triangle_SDsComposite + TriangleSDCompositeOffsets[myTriangleID], gran_params,
                                     mesh_params);
        // TODO could be faster
        for (unsigned int i = 0; i < Triangle_NumSDsTouching[myTriangleID]; i++) {
            // write back this triangle ID to be sorted
            Triangle_TriIDsComposite[TriangleSDCompositeOffsets[myTriangleID] + i] = myTriangleID;
        }
    }
}
