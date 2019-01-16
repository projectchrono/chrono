// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dan Negrut, Conlain Kelly, Nic Olsen
// =============================================================================
/*! \file */
// These two must be included first

// NOTE: DON'T MOVE OR CHANGE THESE THREE LINES
#include "chrono/ChConfig.h"
#undef CHRONO_HAS_SSE
#undef CHRONO_HAS_AVX

#include "chrono_granular/utils/ChGranularUtilities.cuh"
#include "chrono_granular/physics/ChGranularGPU_SMC.cuh"
#include "chrono_granular/physics/ChGranularTriMesh.h"

// these define things that mess with cub
#include "chrono_granular/physics/ChGranularCollision.cuh"
#include "chrono_granular/physics/ChGranularBoxTriangle.cuh"
#include "chrono_granular/utils/ChCudaMathUtils.cuh"

namespace chrono {
namespace granular {
typedef const ChSystemGranular_MonodisperseSMC_trimesh::ChGranParams_trimesh* MeshParamsPtr;
typedef ChTriangleSoup<float3>* TriangleSoupPtr;

/// point is in the LRF, rot_mat rotates LRF to GRF, pos translates LRF to GRF
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
__device__ void triangle_figureOutSDBox(const float3& vA,
                                        const float3& vB,
                                        const float3& vC,
                                        const bool inflated,
                                        int* L,
                                        int* U,
                                        GranParamsPtr gran_params) {
    int3 min_pt;
    min_pt.x = MIN(vA.x, MIN(vB.x, vC.x));
    min_pt.y = MIN(vA.y, MIN(vB.y, vC.y));
    min_pt.z = MIN(vA.z, MIN(vB.z, vC.z));

    int3 max_pt;
    max_pt.x = MAX(vA.x, MAX(vB.x, vC.x));
    max_pt.y = MAX(vA.y, MAX(vB.y, vC.y));
    max_pt.z = MAX(vA.z, MAX(vB.z, vC.z));

    if (inflated) {
        int3 offset =
            make_int3(gran_params->sphereRadius_SU, gran_params->sphereRadius_SU, gran_params->sphereRadius_SU);
        min_pt = min_pt - offset;
        max_pt = max_pt + offset;
    }

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
__device__ unsigned int triangle_countTouchedSDs(unsigned int triangleID,
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
    const bool inflated = triangleSoup->inflated[fam];
    triangle_figureOutSDBox(vA, vB, vC, inflated, L, U, gran_params);
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
    float SDcenter[3];
    float SDhalfSizes[3];
    for (int i = L[0]; i <= U[0]; i++) {
        for (int j = L[1]; j <= U[1]; j++) {
            for (int k = L[2]; k <= U[2]; k++) {
                SDhalfSizes[0] = gran_params->SD_size_X_SU / 2;
                SDhalfSizes[1] = gran_params->SD_size_Y_SU / 2;
                SDhalfSizes[2] = gran_params->SD_size_Z_SU / 2;

                SDcenter[0] = gran_params->BD_frame_X + (i * 2 + 1) * SDhalfSizes[0];
                SDcenter[1] = gran_params->BD_frame_Y + (j * 2 + 1) * SDhalfSizes[1];
                SDcenter[2] = gran_params->BD_frame_Z + (k * 2 + 1) * SDhalfSizes[2];

                if (inflated || check_TriangleBoxOverlap(SDcenter, SDhalfSizes, vA, vB, vC)) {
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
__device__ void triangle_figureOutTouchedSDs(unsigned int triangleID,
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
    const bool inflated = triangleSoup->inflated[fam];
    triangle_figureOutSDBox(vA, vB, vC, inflated, L, U, gran_params);

    // TODO modularize more code
    // Case 1: All vetices are in the same SD
    if (L[0] == U[0] && L[1] == U[1] && L[2] == U[2]) {
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
        if (SD_count >= MAX_SDs_TOUCHED_BY_TRIANGLE) {
            ABORTABORTABORT("SD_count exceeds MAX_SDs_TOUCHED_BY_TRIANGLE\n");
        }
        return;
    }

    // Case 3: Triangle spans more than one dimension of spheresTouchingThisSD
    float SDcenter[3];
    float SDhalfSizes[3];
    for (int i = L[0]; i <= U[0]; i++) {
        for (int j = L[1]; j <= U[1]; j++) {
            for (int k = L[2]; k <= U[2]; k++) {
                SDhalfSizes[0] = gran_params->SD_size_X_SU / 2;
                SDhalfSizes[1] = gran_params->SD_size_Y_SU / 2;
                SDhalfSizes[2] = gran_params->SD_size_Z_SU / 2;

                SDcenter[0] = gran_params->BD_frame_X + (i * 2 + 1) * SDhalfSizes[0];
                SDcenter[1] = gran_params->BD_frame_Y + (j * 2 + 1) * SDhalfSizes[1];
                SDcenter[2] = gran_params->BD_frame_Z + (k * 2 + 1) * SDhalfSizes[2];

                // If mesh is inflated, we don't have a higher-resultion check yet
                if (inflated || check_TriangleBoxOverlap(SDcenter, SDhalfSizes, vA, vB, vC)) {
                    unsigned int currSD = SDTripletID(i, j, k, gran_params);
                    if (currSD != NULL_GRANULAR_ID) {
                        touchedSDs[SD_count++] = currSD;
                        if (SD_count == MAX_SDs_TOUCHED_BY_TRIANGLE) {
                            ABORTABORTABORT("SD_count exceeds MAX_SDs_TOUCHED_BY_TRIANGLE\n");
                        }
                    }
                }
            }
        }
    }
}

void ChSystemGranular_MonodisperseSMC_trimesh::resetTriangleForces() {
    gpuErrchk(cudaMemset(meshSoup_DEVICE->generalizedForcesPerFamily, 0, 6 * MAX_TRIANGLE_FAMILIES * sizeof(float)));
}
// Reset triangle broadphase data structures
void ChSystemGranular_MonodisperseSMC_trimesh::resetTriangleBroadphaseInformation() {
    gpuErrchk(cudaMemset(SD_numTrianglesTouching.data(), 0, SD_numTrianglesTouching.size() * sizeof(unsigned int)));
    gpuErrchk(cudaMemset(SD_TriangleCompositeOffsets.data(), NULL_GRANULAR_ID,
                         SD_TriangleCompositeOffsets.size() * sizeof(unsigned int)));
    gpuErrchk(cudaMemset(triangles_in_SD_composite.data(), NULL_GRANULAR_ID,
                         triangles_in_SD_composite.size() * sizeof(unsigned int)));
}

template <unsigned int CUB_THREADS>  //!< Number of CUB threads engaged in block-collective CUB operations. Should be a
                                     //!< multiple of 32
__global__ void triangleSoupBroadPhase(
    const TriangleSoupPtr d_triangleSoup,
    unsigned int* triangles_in_SD_composite,    //!< which triangles touch this SD?
    unsigned int* SD_numTrianglesTouching,      //!< number of triangles touching this SD
    unsigned int* SD_TriangleCompositeOffsets,  //!< offset of triangles in the composite array for each SD
    GranParamsPtr gran_params,
    MeshParamsPtr mesh_params) {
    /// Set aside shared memory
    volatile __shared__ unsigned int compositeOffsets[CUB_THREADS * MAX_SDs_TOUCHED_BY_TRIANGLE];
    volatile __shared__ bool shMem_head_flags[CUB_THREADS * MAX_SDs_TOUCHED_BY_TRIANGLE];

    typedef cub::BlockRadixSort<unsigned int, CUB_THREADS, MAX_SDs_TOUCHED_BY_TRIANGLE, unsigned int> BlockRadixSortOP;
    __shared__ typename BlockRadixSortOP::TempStorage temp_storage_sort;

    typedef cub::BlockDiscontinuity<unsigned int, CUB_THREADS> Block_Discontinuity;
    __shared__ typename Block_Discontinuity::TempStorage temp_storage_disc;

    unsigned int triangleIDs[MAX_SDs_TOUCHED_BY_TRIANGLE];
    unsigned int SDsTouched[MAX_SDs_TOUCHED_BY_TRIANGLE];

    // Figure out what triangleID this thread will handle. We work with a 1D block structure and a 1D grid structure
    unsigned int myTriangleID = threadIdx.x + blockIdx.x * blockDim.x;
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_TRIANGLE; i++) {
        // start with a clean slate
        triangleIDs[i] = myTriangleID;
        SDsTouched[i] = NULL_GRANULAR_ID;
    }

    __syncthreads();

    if (myTriangleID < d_triangleSoup->nTrianglesInSoup) {
        triangle_figureOutTouchedSDs(myTriangleID, d_triangleSoup, SDsTouched, gran_params, mesh_params);
    }

    __syncthreads();

    BlockRadixSortOP(temp_storage_sort).Sort(SDsTouched, triangleIDs);
    __syncthreads();

    // Do a winningStreak search on whole block, might not have high utilization here
    bool head_flags[MAX_SDs_TOUCHED_BY_TRIANGLE];
    Block_Discontinuity(temp_storage_disc).FlagHeads(head_flags, SDsTouched, cub::Inequality());
    __syncthreads();

    // Write back to shared memory; eight-way bank conflicts here - to revisit later
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_TRIANGLE; i++) {
        shMem_head_flags[MAX_SDs_TOUCHED_BY_TRIANGLE * threadIdx.x + i] = head_flags[i];
    }
    __syncthreads();

    // Seed offsetInComposite_SphInSD_Array with "no valid ID" so that we know later on what is legit;
    // No shmem bank coflicts here, good access...
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_TRIANGLE; i++) {
        compositeOffsets[i * CUB_THREADS + threadIdx.x] = NULL_GRANULAR_ID;
    }

    __syncthreads();

    // Count how many times an SD shows up in conjunction with the collection of CUB_THREADS triangles. There
    // will be some thread divergence here.
    // Loop through each potential SD, after sorting, and see if it is the start of a head
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_TRIANGLE; i++) {
        // SD currently touched, could easily be inlined
        unsigned int touchedSD = SDsTouched[i];
        if (touchedSD != NULL_GRANULAR_ID && head_flags[i]) {
            // current index into shared datastructure of length 8*CUB_THREADS, could easily be inlined
            unsigned int idInShared = MAX_SDs_TOUCHED_BY_TRIANGLE * threadIdx.x + i;
            unsigned int winningStreak = 0;
            // This is the beginning of a sequence of SDs with a new ID
            do {
                winningStreak++;
                // Go until we run out of threads on the warp or until we find a new head
            } while (idInShared + winningStreak < MAX_SDs_TOUCHED_BY_TRIANGLE * CUB_THREADS &&
                     !(shMem_head_flags[idInShared + winningStreak]));

            // Store start of new entries
            unsigned int offset = atomicAdd(SD_TriangleCompositeOffsets + touchedSD, winningStreak);

            // Produce the offsets for this streak of triangles with identical SD ids
            for (unsigned int triInStreak = 0; triInStreak < winningStreak; triInStreak++) {
                compositeOffsets[idInShared + triInStreak] = offset++;
            }
        }
    }

    __syncthreads();  // needed since we write to shared memory above; i.e., offsetInComposite_SphInSD_Array

    // Write out the data now; register with triangles_in_SD_composite each triangle that touches a certain SD
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_TRIANGLE; i++) {
        unsigned int offset = compositeOffsets[MAX_SDs_TOUCHED_BY_TRIANGLE * threadIdx.x + i];
        if (offset != NULL_GRANULAR_ID) {
            triangles_in_SD_composite[offset] = triangleIDs[i];
        }
    }
}

template <unsigned int CUB_THREADS>  //!< Number of CUB threads engaged in block-collective CUB operations. Should be a
                                     //!< multiple of 32
__global__ void triangleSoupBroadPhase_dryrun(
    const TriangleSoupPtr d_triangleSoup,
    unsigned int* SD_numTrianglesTouching,  //!< number of triangles touching this SD
    GranParamsPtr gran_params,
    MeshParamsPtr mesh_params) {
    /// Set aside shared memory
    volatile __shared__ bool shMem_head_flags[CUB_THREADS * MAX_SDs_TOUCHED_BY_TRIANGLE];

    typedef cub::BlockRadixSort<unsigned int, CUB_THREADS, MAX_SDs_TOUCHED_BY_TRIANGLE> BlockRadixSortOP;
    __shared__ typename BlockRadixSortOP::TempStorage temp_storage_sort;

    typedef cub::BlockDiscontinuity<unsigned int, CUB_THREADS> Block_Discontinuity;
    __shared__ typename Block_Discontinuity::TempStorage temp_storage_disc;

    unsigned int SDsTouched[MAX_SDs_TOUCHED_BY_TRIANGLE];

    // Figure out what triangleID this thread will handle. We work with a 1D block structure and a 1D grid structure
    unsigned int myTriangleID = threadIdx.x + blockIdx.x * blockDim.x;
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_TRIANGLE; i++) {
        // start with a clean slate
        SDsTouched[i] = NULL_GRANULAR_ID;
    }

    if (myTriangleID < d_triangleSoup->nTrianglesInSoup) {
        triangle_figureOutTouchedSDs(myTriangleID, d_triangleSoup, SDsTouched, gran_params, mesh_params);
    }

    BlockRadixSortOP(temp_storage_sort).Sort(SDsTouched);
    __syncthreads();

    // Do a winningStreak search on whole block, might not have high utilization here
    bool head_flags[MAX_SDs_TOUCHED_BY_TRIANGLE];
    Block_Discontinuity(temp_storage_disc).FlagHeads(head_flags, SDsTouched, cub::Inequality());
    __syncthreads();

    // Write back to shared memory; eight-way bank conflicts here - to revisit later
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_TRIANGLE; i++) {
        shMem_head_flags[MAX_SDs_TOUCHED_BY_TRIANGLE * threadIdx.x + i] = head_flags[i];
    }

    __syncthreads();

    // Count how many times an SD shows up in conjunction with the collection of CUB_THREADS triangles. There
    // will be some thread divergence here.
    // Loop through each potential SD, after sorting, and see if it is the start of a head
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_TRIANGLE; i++) {
        // SD currently touched, could easily be inlined
        unsigned int touchedSD = SDsTouched[i];
        if (touchedSD != NULL_GRANULAR_ID && head_flags[i]) {
            // current index into shared datastructure of length 8*CUB_THREADS, could easily be inlined
            unsigned int idInShared = MAX_SDs_TOUCHED_BY_TRIANGLE * threadIdx.x + i;
            unsigned int winningStreak = 0;
            // This is the beginning of a sequence of SDs with a new ID
            do {
                winningStreak++;
                // Go until we run out of threads on the warp or until we find a new head
            } while (idInShared + winningStreak < MAX_SDs_TOUCHED_BY_TRIANGLE * CUB_THREADS &&
                     !(shMem_head_flags[idInShared + winningStreak]));

            // Store start of new entries
            unsigned int offset = atomicAdd(SD_numTrianglesTouching + touchedSD, winningStreak);

            if (offset + winningStreak >= MAX_TRIANGLE_COUNT_PER_SD) {
                ABORTABORTABORT("TOO MANY TRIANGLES IN SD %u,  %u TRIANGLES TOUCHING, MAX IS %u\n", touchedSD,
                                offset + winningStreak, MAX_TRIANGLE_COUNT_PER_SD);
            }
        }
    }

    __syncthreads();  // needed since we write to shared memory above; i.e., offsetInComposite_SphInSD_Array
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

template <unsigned int N_CUDATHREADS>
__global__ void interactionTerrain_TriangleSoup(
    TriangleSoupPtr d_triangleSoup,  //!< Contains information pertaining to triangle soup (in device mem.)
    sphereDataStruct sphere_data,
    unsigned int* triangles_in_SD_composite,    //!< Big array that works in conjunction with SD_numTrianglesTouching.
    unsigned int* SD_numTrianglesTouching,      //!< number of triangles touching this SD
    unsigned int* SD_TriangleCompositeOffsets,  //!< offset of triangles in the composite array for each SD
    GranParamsPtr gran_params,
    MeshParamsPtr mesh_params) {
    __shared__ unsigned int triangleIDs[MAX_TRIANGLE_COUNT_PER_SD];  //!< global ID of the triangles touching this SD

    __shared__ int sphere_X[MAX_COUNT_OF_SPHERES_PER_SD];  //!< X coordinate of the grElement
    __shared__ int sphere_Y[MAX_COUNT_OF_SPHERES_PER_SD];  //!< Y coordinate of the grElement
    __shared__ int sphere_Z[MAX_COUNT_OF_SPHERES_PER_SD];  //!< Z coordinate of the grElement

    __shared__ float sphere_X_DOT[MAX_COUNT_OF_SPHERES_PER_SD];
    __shared__ float sphere_Y_DOT[MAX_COUNT_OF_SPHERES_PER_SD];
    __shared__ float sphere_Z_DOT[MAX_COUNT_OF_SPHERES_PER_SD];

    // TODO figure out how we can do this better with no friction
    __shared__ float3 omega[MAX_COUNT_OF_SPHERES_PER_SD];

    __shared__ double3 node1[MAX_TRIANGLE_COUNT_PER_SD];  //!< Coordinates of the 1st node of the triangle
    __shared__ double3 node2[MAX_TRIANGLE_COUNT_PER_SD];  //!< Coordinates of the 2nd node of the triangle
    __shared__ double3 node3[MAX_TRIANGLE_COUNT_PER_SD];  //!< Coordinates of the 3rd node of the triangle

    // define an alias first
    unsigned int thisSD = blockIdx.x;

    if (SD_numTrianglesTouching[thisSD] == 0) {
        return;  // no triangle touches this SD; return right away
    }
    unsigned int spheresTouchingThisSD = sphere_data.SD_NumSpheresTouching[thisSD];
    if (spheresTouchingThisSD == 0) {
        return;  // no sphere to speak of in this SD
    }

    // Getting here means that there are both triangles and DEs in this SD.
    // First, figure out which bucket stores the triangles associated with this SD.
    unsigned int numSDTriangles = SD_numTrianglesTouching[thisSD];

    // Unpleasant fact: this bucket might store more than the triangles associated with this SD. The narrow phase is
    // done for ALL triangles in this bucket with the expectation that if a triangle does not belong to this SD, the
    // narrow phase will prune this triangle fast and the penalty associated with storing triangles from multiple
    // SDs into one bucket is not stiff.
    // Sphere angular velocities

    unsigned int sphereIDLocal = threadIdx.x;
    unsigned int sphereIDGlobal = NULL_GRANULAR_ID;
    // Bring in data from global into shmem. Only a subset of threads get to do this.
    // Note that we're not using shared memory very heavily, so our bandwidth is pretty low
    if (sphereIDLocal < spheresTouchingThisSD) {
        unsigned int SD_composite_offset = sphere_data.SD_SphereCompositeOffsets[thisSD];

        // TODO standardize this
        // We may need long ints to index into composite array
        size_t offset_in_composite_Array = SD_composite_offset + sphereIDLocal;
        sphereIDGlobal = sphere_data.spheres_in_SD_composite[offset_in_composite_Array];
        sphere_X[sphereIDLocal] = sphere_data.pos_X[sphereIDGlobal];
        sphere_Y[sphereIDLocal] = sphere_data.pos_Y[sphereIDGlobal];
        sphere_Z[sphereIDLocal] = sphere_data.pos_Z[sphereIDGlobal];
        if (gran_params->friction_mode != chrono::granular::GRAN_FRICTION_MODE::FRICTIONLESS) {
            omega[sphereIDLocal] =
                make_float3(sphere_data.sphere_Omega_X[sphereIDGlobal], sphere_data.sphere_Omega_Y[sphereIDGlobal],
                            sphere_data.sphere_Omega_Z[sphereIDGlobal]);
        }
        sphere_X_DOT[sphereIDLocal] = sphere_data.pos_X_dt[sphereIDGlobal];
        sphere_Y_DOT[sphereIDLocal] = sphere_data.pos_Y_dt[sphereIDGlobal];
        sphere_Z_DOT[sphereIDLocal] = sphere_data.pos_Z_dt[sphereIDGlobal];
    }
    // Populate the shared memory with mesh triangle data
    unsigned int tripsToCoverTriangles = (numSDTriangles + blockDim.x - 1) / blockDim.x;
    unsigned int local_ID = threadIdx.x;
    for (unsigned int triangTrip = 0; triangTrip < tripsToCoverTriangles; triangTrip++) {
        if (local_ID < numSDTriangles) {
            unsigned int SD_composite_offset = SD_TriangleCompositeOffsets[thisSD];
            if (SD_composite_offset == NULL_GRANULAR_ID) {
                ABORTABORTABORT("Invalid composite offset %u for SD %u, touching %u triangles\n", NULL_GRANULAR_ID,
                                thisSD, numSDTriangles);
            }
            size_t offset_in_composite_Array = SD_composite_offset + local_ID;

            unsigned int globalID = triangles_in_SD_composite[offset_in_composite_Array];
            triangleIDs[local_ID] = globalID;

            // Read node positions from global memory into shared memory
            // NOTE implicit cast from float to double here
            unsigned int fam = d_triangleSoup->triangleFamily_ID[globalID];
            node1[local_ID] = apply_frame_transform<double, float3, double3>(
                d_triangleSoup->node1[globalID], mesh_params->fam_frame_narrow[fam].pos,
                mesh_params->fam_frame_narrow[fam].rot_mat);

            node2[local_ID] = apply_frame_transform<double, float3, double3>(
                d_triangleSoup->node2[globalID], mesh_params->fam_frame_narrow[fam].pos,
                mesh_params->fam_frame_narrow[fam].rot_mat);

            node3[local_ID] = apply_frame_transform<double, float3, double3>(
                d_triangleSoup->node3[globalID], mesh_params->fam_frame_narrow[fam].pos,
                mesh_params->fam_frame_narrow[fam].rot_mat);

            convert_pos_UU2SU<double3>(node1[local_ID], gran_params);
            convert_pos_UU2SU<double3>(node2[local_ID], gran_params);
            convert_pos_UU2SU<double3>(node3[local_ID], gran_params);
        }
        local_ID += blockDim.x;
    }

    __syncthreads();  // this call ensures data is in its place in shared memory

    float3 sphere_force = {0.f, 0.f, 0.f};
    float3 sphere_AngAcc = {0.f, 0.f, 0.f};
    if (sphereIDLocal < spheresTouchingThisSD) {
        // loop over each triangle in the bucket and compute the force this sphere (thread) exerts on it
        for (unsigned int triangleLocalID = 0; triangleLocalID < numSDTriangles; triangleLocalID++) {
            /// we have a valid sphere and a valid triganle; check if in contact
            float3 normal;  // Unit normal from pt2 to pt1 (triangle contact point to sphere contact point)
            float depth;    // Negative in overlap
            float3 pt1_float;

            // Transform LRF to GRF
            const unsigned int fam = d_triangleSoup->triangleFamily_ID[triangleIDs[triangleLocalID]];
            bool valid_contact = false;

            // vector from center of mesh body to contact point, assume this can be held in a float
            float3 fromCenter;

            {
                double3 pt1;  // Contact point on triangle
                // NOTE implicit cast from int to double
                double3 sphCntr =
                    make_double3(sphere_X[sphereIDLocal], sphere_Y[sphereIDLocal], sphere_Z[sphereIDLocal]);
                valid_contact = face_sphere_cd(node1[triangleLocalID], node2[triangleLocalID], node3[triangleLocalID],
                                               sphCntr, gran_params->sphereRadius_SU, d_triangleSoup->inflated[fam],
                                               d_triangleSoup->inflation_radii[fam], normal, depth, pt1) &&
                                SDTripletID(pointSDTriplet(pt1.x, pt1.y, pt1.z, gran_params), gran_params) == thisSD;
                pt1_float = make_float3(pt1.x, pt1.y, pt1.z);

                double3 meshCenter_double =
                    make_double3(mesh_params->fam_frame_narrow[fam].pos[0], mesh_params->fam_frame_narrow[fam].pos[1],
                                 mesh_params->fam_frame_narrow[fam].pos[2]);
                convert_pos_UU2SU<double3>(meshCenter_double, gran_params);

                double3 fromCenter_double = pt1 - meshCenter_double;
                fromCenter = make_float3(fromCenter_double.x, fromCenter_double.y, fromCenter_double.z);
            }

            // If there is a collision, add an impulse to the sphere
            if (valid_contact) {
                // TODO contact models
                // Use the CD information to compute the force on the grElement
                float3 delta = -depth * normal;
                float3 force_accum = mesh_params->Kn_s2m_SU * delta;

                // Compute force updates for adhesion term, opposite the spring term
                // NOTE ratio is wrt the weight of a sphere of mass 1
                // NOTE the cancelation of two negatives
                force_accum = force_accum + gran_params->adhesionAcc_s2w * delta / depth;

                // Velocity difference, it's better to do a coalesced access here than a fragmented access
                // inside
                float3 v_rel =
                    make_float3(sphere_X_DOT[sphereIDLocal] - d_triangleSoup->vel[fam].x,
                                sphere_Y_DOT[sphereIDLocal] - d_triangleSoup->vel[fam].y,
                                sphere_Z_DOT[sphereIDLocal] -
                                    d_triangleSoup->vel[fam].z);  // TODO use shared memory if there is any left...

                // TODO assumes pos is the center of mass of the mesh
                // TODO can this be float?
                float3 meshCenter =
                    make_float3(mesh_params->fam_frame_broad[fam].pos[0], mesh_params->fam_frame_broad[fam].pos[1],
                                mesh_params->fam_frame_broad[fam].pos[2]);
                convert_pos_UU2SU<float3>(meshCenter, gran_params);

                // NOTE depth is negative and normal points from triangle to sphere center
                float3 r = pt1_float + normal * (depth / 2) - meshCenter;

                // Add angular velocity contribution from mesh
                v_rel = v_rel + Cross(d_triangleSoup->omega[fam], r);

                // add tangential components if they exist
                if (gran_params->friction_mode != chrono::granular::GRAN_FRICTION_MODE::FRICTIONLESS) {
                    // Vector from the center of sphere to center of contact volume
                    float3 r_A = -(gran_params->sphereRadius_SU + depth / 2.f) * normal;
                    v_rel = v_rel + Cross(omega[sphereIDLocal], r_A);
                }

                // Forace accumulator on sphere for this sphere-triangle collision
                // Compute force updates for normal spring term

                // Compute force updates for damping term
                // NOTE assumes sphere mass of 1
                float fam_mass_SU = d_triangleSoup->familyMass_SU[fam];
                constexpr float sphere_mass_SU = gran_params->sphere_mass_SU;
                float m_eff = sphere_mass_SU * fam_mass_SU / (sphere_mass_SU + fam_mass_SU);
                float3 vrel_n = Dot(v_rel, normal) * normal;
                v_rel = v_rel - vrel_n;  // v_rel is now tangential relative velocity

                // Add normal damping term
                force_accum = force_accum + -mesh_params->Gamma_n_s2m_SU * m_eff * vrel_n;

                if (gran_params->friction_mode != chrono::granular::GRAN_FRICTION_MODE::FRICTIONLESS) {
                    if (gran_params->friction_mode == chrono::granular::GRAN_FRICTION_MODE::SINGLE_STEP) {
                        // both tangential terms combine for single step
                        float combined_tangent_coeff =
                            mesh_params->Kt_s2m_SU * gran_params->stepSize_SU + mesh_params->Gamma_t_s2m_SU * m_eff;

                        // we dotted out normal component of v, so v_rel is the tangential component
                        // TODO apply model multiplier
                        float3 tangent_force = -combined_tangent_coeff * v_rel;

                        // Vector from sphere center to center of contact volume
                        // NOTE depth is negative
                        float3 sphere_r = -(gran_params->sphereRadius_SU + depth / 2.f) * normal;

                        // TODO could do scaling to eliminate a factor of radius?
                        sphere_AngAcc =
                            sphere_AngAcc + Cross(sphere_r, tangent_force) /
                                                (gran_params->sphereInertia_by_r * gran_params->sphereRadius_SU);

                        // add to total forces
                        force_accum = force_accum + tangent_force;
                    }
                }

                // Use the CD information to compute the force and torque on the family of this triangle
                sphere_force = sphere_force + force_accum;

                // Force on the mesh is opposite the force on the sphere
                float3 force_total = -1.f * force_accum;

                float3 torque = Cross(fromCenter, force_total);
                // TODO we could be much smarter about reducing this atomic write
                unsigned int fam = d_triangleSoup->triangleFamily_ID[triangleIDs[triangleLocalID]];
                atomicAdd(d_triangleSoup->generalizedForcesPerFamily + fam * 6 + 0, force_total.x);
                atomicAdd(d_triangleSoup->generalizedForcesPerFamily + fam * 6 + 1, force_total.y);
                atomicAdd(d_triangleSoup->generalizedForcesPerFamily + fam * 6 + 2, force_total.z);

                atomicAdd(d_triangleSoup->generalizedForcesPerFamily + fam * 6 + 3, torque.x);
                atomicAdd(d_triangleSoup->generalizedForcesPerFamily + fam * 6 + 4, torque.y);
                atomicAdd(d_triangleSoup->generalizedForcesPerFamily + fam * 6 + 5, torque.z);
            }
        }  // end of per-triangle loop
        // write back sphere forces
        atomicAdd(sphere_data.sphere_force_X + sphereIDGlobal, sphere_force.x);
        atomicAdd(sphere_data.sphere_force_Y + sphereIDGlobal, sphere_force.y);
        atomicAdd(sphere_data.sphere_force_Z + sphereIDGlobal, sphere_force.z);

        if (gran_params->friction_mode != chrono::granular::GRAN_FRICTION_MODE::FRICTIONLESS) {
            // write back torques for later
            atomicAdd(sphere_data.sphere_ang_acc_X + sphereIDGlobal, sphere_AngAcc.x);
            atomicAdd(sphere_data.sphere_ang_acc_Y + sphereIDGlobal, sphere_AngAcc.y);
            atomicAdd(sphere_data.sphere_ang_acc_Z + sphereIDGlobal, sphere_AngAcc.z);
        }
    }  // end sphere id check
}  // end kernel

__host__ void ChSystemGranular_MonodisperseSMC_trimesh::runTriangleBroadphase() {
    VERBOSE_PRINTF("Resetting broadphase info!\n");

    sphereDataStruct sphere_data;

    packSphereDataPointers(sphere_data);

    const int nthreads = 32;

    int nblocks = (meshSoup_DEVICE->nTrianglesInSoup + nthreads - 1) / nthreads;
    // broadphase the triangles
    // TODO check these block/thread counts
    triangleSoupBroadPhase_dryrun<nthreads>
        <<<nblocks, nthreads>>>(meshSoup_DEVICE, SD_numTrianglesTouching.data(), gran_params, tri_params);

    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaPeekAtLastError());

    void* d_temp_storage = NULL;
    size_t temp_storage_bytes = 0;

    // num spheres in last SD

    unsigned int* out_ptr = SD_TriangleCompositeOffsets.data();
    unsigned int* in_ptr = SD_numTrianglesTouching.data();

    // copy data into the tmp array
    gpuErrchk(cudaMemcpy(out_ptr, in_ptr, nSDs * sizeof(unsigned int), cudaMemcpyDeviceToDevice));
    cub::DeviceScan::ExclusiveSum(d_temp_storage, temp_storage_bytes, in_ptr, out_ptr, nSDs);

    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaPeekAtLastError());
    // Allocate temporary storage
    gpuErrchk(cudaMalloc(&d_temp_storage, temp_storage_bytes));

    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaPeekAtLastError());
    // Run exclusive prefix sum
    cub::DeviceScan::ExclusiveSum(d_temp_storage, temp_storage_bytes, in_ptr, out_ptr, nSDs);

    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaPeekAtLastError());
    // total number of sphere entries to record
    unsigned int num_entries = out_ptr[nSDs - 1] + in_ptr[nSDs - 1];
    triangles_in_SD_composite.resize(num_entries, NULL_GRANULAR_ID);
    // printf("FIRST RUN: %u entries total!\n", num_entries);

    // printf("first run: num entries is %u, theoretical max is %u\n", num_entries, nSDs *
    // MAX_TRIANGLE_COUNT_PER_SD);

    // back up the offsets
    // TODO use a cached allocator, CUB provides one
    std::vector<unsigned int, cudallocator<unsigned int>> SD_TriangleCompositeOffsets_tmp;
    SD_TriangleCompositeOffsets_tmp.resize(SD_TriangleCompositeOffsets.size());
    gpuErrchk(cudaMemcpy(SD_TriangleCompositeOffsets_tmp.data(), SD_TriangleCompositeOffsets.data(),
                         nSDs * sizeof(unsigned int), cudaMemcpyDeviceToDevice));

    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaPeekAtLastError());

    triangleSoupBroadPhase<nthreads>
        <<<nblocks, nthreads>>>(meshSoup_DEVICE, triangles_in_SD_composite.data(), SD_numTrianglesTouching.data(),
                                SD_TriangleCompositeOffsets.data(), gran_params, tri_params);
    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaPeekAtLastError());

    // restore the old offsets
    gpuErrchk(cudaMemcpy(SD_TriangleCompositeOffsets.data(), SD_TriangleCompositeOffsets_tmp.data(),
                         nSDs * sizeof(unsigned int), cudaMemcpyDeviceToDevice));

    // for (unsigned int i = 0; i < nSDs; i++) {
    //     printf("SD %d has offset %u, N %u \n", i, out_ptr[i], in_ptr[i]);
    // }
    //
    // for (unsigned int i = 0; i < num_entries; i++) {
    //     printf("entry %u is %u\n", i, triangles_in_SD_composite[i]);
    // }

    gpuErrchk(cudaFree(d_temp_storage));
}

__host__ void ChSystemGranular_MonodisperseSMC_trimesh::runTriangleBroadphase_rewrite() {
    VERBOSE_PRINTF("Resetting broadphase info!\n");

    sphereDataStruct sphere_data;

    packSphereDataPointers(sphere_data);

    std::vector<unsigned int, cudallocator<unsigned int>> Triangle_NumSDsTouching;
    std::vector<unsigned int, cudallocator<unsigned int>> Triangle_SDsCompositeOffsets;

    Triangle_NumSDsTouching.resize(meshSoup_DEVICE->nTrianglesInSoup, 0);
    Triangle_SDsCompositeOffsets.resize(meshSoup_DEVICE->nTrianglesInSoup, 0);

    const int nthreads = CUDA_THREADS_PER_BLOCK;
    int nblocks = (meshSoup_DEVICE->nTrianglesInSoup + nthreads - 1) / nthreads;
    triangleSoup_CountSDsTouched<<<nblocks, nthreads>>>(meshSoup_DEVICE, Triangle_NumSDsTouching.data(), gran_params,
                                                        tri_params);

    unsigned int numTriangles = meshSoup_DEVICE->nTrianglesInSoup;

    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaPeekAtLastError());

    unsigned int num_entries = 0;

    // do prefix scan
    {
        void* d_temp_storage = NULL;
        size_t temp_storage_bytes = 0;
        unsigned int* out_ptr = Triangle_SDsCompositeOffsets.data();
        unsigned int* in_ptr = Triangle_NumSDsTouching.data();

        // copy data into the tmp array
        gpuErrchk(cudaMemcpy(out_ptr, in_ptr, numTriangles * sizeof(unsigned int), cudaMemcpyDeviceToDevice));
        cub::DeviceScan::ExclusiveSum(d_temp_storage, temp_storage_bytes, in_ptr, out_ptr, numTriangles);

        gpuErrchk(cudaDeviceSynchronize());
        gpuErrchk(cudaPeekAtLastError());
        // Allocate temporary storage
        gpuErrchk(cudaMalloc(&d_temp_storage, temp_storage_bytes));

        gpuErrchk(cudaDeviceSynchronize());
        gpuErrchk(cudaPeekAtLastError());
        // Run exclusive prefix sum
        cub::DeviceScan::ExclusiveSum(d_temp_storage, temp_storage_bytes, in_ptr, out_ptr, numTriangles);

        gpuErrchk(cudaDeviceSynchronize());
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaFree(d_temp_storage));
        num_entries = out_ptr[numTriangles - 1] + in_ptr[numTriangles - 1];
        // printf("%u entries total!\n", num_entries);
    }

    // for (unsigned int i = 0; i < Triangle_NumSDsTouching.size(); i++) {
    //     printf("Triangle %u touches %u SDs, offset is %u\n", i, Triangle_NumSDsTouching[i],
    //            Triangle_SDsCompositeOffsets[i]);
    // }
    // total number of sphere entries to record
    // to be sorted
    // produced by sort
    std::vector<unsigned int, cudallocator<unsigned int>> Triangle_SDsComposite_SDs_out;
    std::vector<unsigned int, cudallocator<unsigned int>> Triangle_SDsComposite_TriIDs_out;

    Triangle_SDsComposite_SDs_out.resize(num_entries, NULL_GRANULAR_ID);
    Triangle_SDsComposite_TriIDs_out.resize(num_entries, NULL_GRANULAR_ID);

    gpuErrchk(cudaDeviceSynchronize());
    gpuErrchk(cudaPeekAtLastError());
    // sort key-value where the key is SD id, value is triangle ID in composite array
    {
        // tmp values used for sort
        std::vector<unsigned int, cudallocator<unsigned int>> Triangle_SDsComposite_SDs;
        std::vector<unsigned int, cudallocator<unsigned int>> Triangle_SDsComposite_TriIDs;
        Triangle_SDsComposite_SDs.resize(num_entries, NULL_GRANULAR_ID);
        Triangle_SDsComposite_TriIDs.resize(num_entries, NULL_GRANULAR_ID);

        // printf("first run: num entries is %u, theoretical max is %u\n", num_entries, nSDs *
        // MAX_TRIANGLE_COUNT_PER_SD);
        triangleSoup_StoreSDsTouched<<<nblocks, nthreads>>>(
            meshSoup_DEVICE, Triangle_NumSDsTouching.data(), Triangle_SDsCompositeOffsets.data(),
            Triangle_SDsComposite_SDs.data(), Triangle_SDsComposite_TriIDs.data(), gran_params, tri_params);
        unsigned int num_items = num_entries;
        unsigned int* d_keys_in = Triangle_SDsComposite_SDs.data();
        unsigned int* d_keys_out = Triangle_SDsComposite_SDs_out.data();
        unsigned int* d_values_in = Triangle_SDsComposite_TriIDs.data();
        unsigned int* d_values_out = Triangle_SDsComposite_TriIDs_out.data();

        gpuErrchk(cudaDeviceSynchronize());

        // Determine temporary device storage requirements
        void* d_temp_storage = NULL;
        size_t temp_storage_bytes = 0;
        // Run sorting operation
        // pass null, cub tells us what it needs
        cub::DeviceRadixSort::SortPairs(d_temp_storage, temp_storage_bytes, d_keys_in, d_keys_out, d_values_in,
                                        d_values_out, num_items);
        gpuErrchk(cudaDeviceSynchronize());

        // Allocate temporary storage
        gpuErrchk(cudaMalloc(&d_temp_storage, temp_storage_bytes));
        gpuErrchk(cudaDeviceSynchronize());

        cub::DeviceRadixSort::SortPairs(d_temp_storage, temp_storage_bytes, d_keys_in, d_keys_out, d_values_in,
                                        d_values_out, num_items);
        gpuErrchk(cudaDeviceSynchronize());

        gpuErrchk(cudaFree(d_temp_storage));
    }
    // now Triangle_SDsComposite_SDs_out has an ordered list of active SDs, with one entry for each triangle
    //
    // for (unsigned int i = 0; i < Triangle_SDsComposite_TriIDs_out.size(); i++) {
    //     printf("composite entry %u is SD %u, triangle %u\n", i, Triangle_SDsComposite_SDs_out[i],
    //            Triangle_SDsComposite_TriIDs_out[i]);
    // }

    // offsets of each SD in composite array
    std::vector<unsigned int, cudallocator<unsigned int>> SD_TriangleCompositeOffsets_tmp;
    std::vector<unsigned int, cudallocator<unsigned int>> SD_numTrianglesTouching_tmp;

    SD_TriangleCompositeOffsets_tmp.resize(nSDs, NULL_GRANULAR_ID);
    SD_numTrianglesTouching_tmp.resize(nSDs, 0);

    // get first active SD
    unsigned int prev_SD = Triangle_SDsComposite_SDs_out.at(0);
    // first SD has offset 0
    SD_TriangleCompositeOffsets.at(prev_SD) = 0;
    // number of triangles in current SD
    unsigned int curr_count = 0;
    // offset to current SD
    unsigned int curr_offset = 0;

    // simultaneously do a prefix scan and a store, but on host
    // TODO optimize and test
    // TODO can we do this with a weird prefix scan operation?
    for (unsigned int i = 0; i < Triangle_SDsComposite_SDs_out.size(); i++) {
        unsigned int curr_SD = Triangle_SDsComposite_SDs_out.at(i);
        // this is the start of a new SD
        if (prev_SD != curr_SD) {
            // printf("change! SD %u has curr count %u, offset %u, prev is %u\n",curr_count, curr_offset,  );
            // store the count for this SD
            SD_numTrianglesTouching.at(prev_SD) = curr_count;
            // reset count
            curr_count = 0;
            // set this SD to have offset after the previous one ends
            SD_TriangleCompositeOffsets.at(curr_SD) = curr_offset;
        }
        curr_count++;
        curr_offset++;
        // now this is the active SD to check against
        prev_SD = curr_SD;
    }

    // right now we only store counts at the end of a streak, so we need to store the last streak
    // TODO is this always right???
    SD_numTrianglesTouching.at(prev_SD) = curr_count;

    // for (unsigned int i = 0; i < SD_numTrianglesTouching.size(); i++) {
    //     printf("tri count index %u is usual %u, other %u\n", i, SD_numTrianglesTouching[i],
    //            SD_numTrianglesTouching_tmp[i]);
    // }
    //
    // for (unsigned int i = 0; i < SD_TriangleCompositeOffsets.size(); i++) {
    //     printf("offset index %u is usual %u, other %u\n", i, SD_TriangleCompositeOffsets[i],
    //            SD_TriangleCompositeOffsets_tmp[i]);
    // }
    //
    // for (unsigned int i = 0; i < triangles_in_SD_composite.size(); i++) {
    //     printf("composite index %u is usual %u, other %u\n", i, triangles_in_SD_composite[i],
    //            Triangle_SDsComposite_TriIDs_out[i]);
    // }

    triangles_in_SD_composite.resize(Triangle_SDsComposite_TriIDs_out.size());

    // copy the composite data to the primary location
    gpuErrchk(cudaMemcpy(triangles_in_SD_composite.data(), Triangle_SDsComposite_TriIDs_out.data(),
                         Triangle_SDsComposite_TriIDs_out.size() * sizeof(unsigned int), cudaMemcpyDeviceToDevice));

    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());
}
__host__ double ChSystemGranular_MonodisperseSMC_trimesh::advance_simulation(float duration) {
    // Figure our the number of blocks that need to be launched to cover the box
    unsigned int nBlocks = (nSpheres + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

    // Settling simulation loop.
    float duration_SU = duration / gran_params->TIME_UNIT;
    determineNewStepSize_SU();  // doesn't always change the timestep
    unsigned int nsteps = std::round(duration_SU / stepSize_SU);

    sphereDataStruct sphere_data;
    packSphereDataPointers(sphere_data);
    // cudaMemAdvise(gran_params, sizeof(*gran_params), cudaMemAdviseSetReadMostly, dev_ID);

    VERBOSE_PRINTF("advancing by %f at timestep %f, %u timesteps at approx user timestep %f\n", duration_SU,
                   stepSize_SU, nsteps, duration / nsteps);

    VERBOSE_PRINTF("Starting Main Simulation loop!\n");

    float time_elapsed_SU = 0;  // time elapsed in this call (SU)
    // Run the simulation, there are aggressive synchronizations because we want to have no race conditions
    for (; time_elapsed_SU < stepSize_SU * nsteps; time_elapsed_SU += stepSize_SU) {
        determineNewStepSize_SU();  // doesn't always change the timestep

        // Update the position and velocity of the BD, if relevant
        if (!BD_is_fixed) {
            updateBDPosition(stepSize_SU);
        }
        resetSphereForces();
        resetBCForces();
        resetTriangleForces();
        resetTriangleBroadphaseInformation();

        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        VERBOSE_PRINTF("Starting computeSphereForces!\n");

        if (gran_params->friction_mode == FRICTIONLESS) {
            // Compute sphere-sphere forces
            computeSphereForces_frictionless<<<nSDs, MAX_COUNT_OF_SPHERES_PER_SD>>>(
                sphere_data, gran_params, BC_type_list.data(), BC_params_list_SU.data(), BC_params_list_SU.size());
            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());
        } else if (gran_params->friction_mode == SINGLE_STEP || gran_params->friction_mode == MULTI_STEP) {
            // figure out who is contacting
            determineContactPairs<<<nSDs, MAX_COUNT_OF_SPHERES_PER_SD>>>(sphere_data, gran_params);
            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());

            computeSphereContactForces<<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(
                sphere_data, gran_params, BC_type_list.data(), BC_params_list_SU.data(), BC_params_list_SU.size(),
                nSpheres);
            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());
        }

        if (meshSoup_DEVICE->nTrianglesInSoup != 0 && mesh_collision_enabled) {
            gpuErrchk(cudaPeekAtLastError());
            gpuErrchk(cudaDeviceSynchronize());
            // NOTE only run one of these!!!
            // old version has a limitation of number of SDs a triangle can touche
            // runTriangleBroadphase();
            // rewrite uses host for some stuff and might be a little slower
            runTriangleBroadphase_rewrite();
        }
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        if (meshSoup_DEVICE->nFamiliesInSoup != 0 && mesh_collision_enabled) {
            // TODO please do not use a template here
            // compute sphere-triangle forces
            interactionTerrain_TriangleSoup<CUDA_THREADS_PER_BLOCK><<<nSDs, MAX_COUNT_OF_SPHERES_PER_SD>>>(
                meshSoup_DEVICE, sphere_data, triangles_in_SD_composite.data(), SD_numTrianglesTouching.data(),
                SD_TriangleCompositeOffsets.data(), gran_params, tri_params);
        }
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        VERBOSE_PRINTF("Resetting broadphase info!\n");

        resetBroadphaseInformation();

        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        VERBOSE_PRINTF("Starting updatePositions!\n");
        updatePositions<CUDA_THREADS_PER_BLOCK>
            <<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(stepSize_SU, sphere_data, nSpheres, gran_params);
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        runSphereBroadphase();

        packSphereDataPointers(sphere_data);

        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());
        elapsedSimTime += stepSize_SU * gran_params->TIME_UNIT;  // Advance current time
    }

    return time_elapsed_SU * gran_params->TIME_UNIT;  // return elapsed UU time
}
}  // namespace granular
}  // namespace chrono
