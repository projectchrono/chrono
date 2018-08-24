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

#include "chrono_granular/utils/ChGranularUtilities_CUDA.cuh"
#include "chrono_granular/physics/ChGranularGPU_SMC.cuh"
#include "chrono_granular/physics/ChGranularTriMesh.h"

// these define things that mess with cub
#include "chrono_granular/physics/ChGranularCollision.cuh"
#include "chrono_granular/physics/ChGranularBoxTriangle.cuh"
#include "chrono_granular/utils/ChCudaMathUtils.cuh"

// TODO should this go here?

namespace chrono {
namespace granular {
typedef const ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::GranParamsHolder_trimesh* MeshParamsPtr;
typedef ChTriangleSoup<float>* TriangleSoupPtr;

/// point is in the LRF, rot_mat rotates LRF to GRF, pos translates LRF to GRF
template <class T, class T3>
__device__ T3 apply_frame_transform(const T3& point, const T* pos, const T* rot_mat) {
    T3 result;

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
__device__ void convert_pos_UU2SU(T3& pos, ParamsPtr gran_params) {
    pos.x /= gran_params->LENGTH_UNIT;
    pos.y /= gran_params->LENGTH_UNIT;
    pos.z /= gran_params->LENGTH_UNIT;
}

/// Takes in a triangle's position in UU and finds out what SDs it touches
/// Triangle broadphase is done in float by applying the frame transform
/// and then converting the GRF position to SU
__device__ void triangle_figureOutTouchedSDs(unsigned int triangleID,
                                             const TriangleSoupPtr triangleSoup,
                                             unsigned int* touchedSDs,
                                             ParamsPtr gran_params,
                                             MeshParamsPtr tri_params) {
    float3 vA, vB, vC;  // vertices of the triangle

    // Read LRF UU vertices of the triangle
    // Coalesced memory accesses; we have an int to float conversion here
    vA.x = triangleSoup->node1_X[triangleID];
    vA.y = triangleSoup->node1_Y[triangleID];
    vA.z = triangleSoup->node1_Z[triangleID];

    vB.x = triangleSoup->node2_X[triangleID];
    vB.y = triangleSoup->node2_Y[triangleID];
    vB.z = triangleSoup->node2_Z[triangleID];

    vC.x = triangleSoup->node3_X[triangleID];
    vC.y = triangleSoup->node3_Y[triangleID];
    vC.z = triangleSoup->node3_Z[triangleID];

    // Transform LRF to GRF
    unsigned int fam = triangleSoup->triangleFamily_ID[triangleID];
    vA = apply_frame_transform<float, float3>(vA, tri_params->fam_frame_broad[fam].pos,
                                              tri_params->fam_frame_broad[fam].rot_mat);
    vB = apply_frame_transform<float, float3>(vB, tri_params->fam_frame_broad[fam].pos,
                                              tri_params->fam_frame_broad[fam].rot_mat);
    vC = apply_frame_transform<float, float3>(vC, tri_params->fam_frame_broad[fam].pos,
                                              tri_params->fam_frame_broad[fam].rot_mat);

    // Convert UU to SU
    convert_pos_UU2SU<float3>(vA, gran_params);
    convert_pos_UU2SU<float3>(vB, gran_params);
    convert_pos_UU2SU<float3>(vC, gran_params);

    // Perform broadphase on UU vertices
    uint3 SDA = pointSDTriplet(vA.x, vA.y, vA.z, gran_params);  // SD indices for point A
    uint3 SDB = pointSDTriplet(vB.x, vB.y, vB.z, gran_params);  // SD indices for point B
    uint3 SDC = pointSDTriplet(vC.x, vC.y, vC.z, gran_params);  // SD indices for point C

    unsigned int L[3];  // Min SD index along each axis
    unsigned int U[3];  // Max SD index along each axis

    L[0] = MIN(SDA.x, MIN(SDB.x, SDC.x));
    L[1] = MIN(SDA.y, MIN(SDB.y, SDC.y));
    L[2] = MIN(SDA.z, MIN(SDB.z, SDC.z));

    U[0] = MAX(SDA.x, MAX(SDB.x, SDC.x));
    U[1] = MAX(SDA.y, MAX(SDB.y, SDC.y));
    U[2] = MAX(SDA.z, MAX(SDB.z, SDC.z));

    // Case 1: All vetices are in the same SD
    if (L[0] == U[0] && L[1] == U[1] && L[2] == U[2]) {
        touchedSDs[0] = SDTripletID(L, gran_params);
        return;
    }

    unsigned int SD_count = 0;
    unsigned int n_axes_diff = 0;  // Count axes that have different SD bounds
    unsigned int axes_diff;

    for (unsigned int i = 0; i < 3; i++) {
        if (L[i] != U[i]) {
            axes_diff = i;  // If there are more than one, this won't be used anyway
            n_axes_diff++;
        }
    }

    // Case 2: Triangle lies in a Nx1x1, 1xNx1, or 1x1xN block of SDs
    if (n_axes_diff == 1) {
        unsigned int SD_i[3] = {L[0], L[1], L[2]};
        if (U[axes_diff] - L[axes_diff] >= MAX_SDs_TOUCHED_BY_TRIANGLE) {
            ABORTABORTABORT("SD_count exceeds MAX_SDs_TOUCHED_BY_TRIANGLE\n");
        }
        for (unsigned int i = L[axes_diff]; i <= U[axes_diff]; i++) {
            SD_i[axes_diff] = i;
            touchedSDs[SD_count++] = SDTripletID(SD_i, gran_params);
        }
        return;
    }

    // Case 3: Triangle spans more than one dimension of nSD_spheres
    float SDcenter[3];
    float SDhalfSizes[3];
    for (unsigned int i = L[0]; i <= U[0]; i++) {
        for (unsigned int j = L[1]; j <= U[1]; j++) {
            for (unsigned int k = L[2]; k <= U[2]; k++) {
                SDhalfSizes[0] = gran_params->SD_size_X_SU / 2;
                SDhalfSizes[1] = gran_params->SD_size_Y_SU / 2;
                SDhalfSizes[2] = gran_params->SD_size_Z_SU / 2;

                SDcenter[0] = gran_params->BD_frame_X + (i * 2 + 1) * SDhalfSizes[0];
                SDcenter[1] = gran_params->BD_frame_Y + (j * 2 + 1) * SDhalfSizes[1];
                SDcenter[2] = gran_params->BD_frame_Z + (k * 2 + 1) * SDhalfSizes[2];

                if (check_TriangleBoxOverlap(SDcenter, SDhalfSizes, vA, vB, vC)) {
                    touchedSDs[SD_count++] = SDTripletID(i, j, k, gran_params);
                    if (SD_count == MAX_SDs_TOUCHED_BY_TRIANGLE) {
                        ABORTABORTABORT("SD_count exceeds MAX_SDs_TOUCHED_BY_TRIANGLE\n");
                    }
                }
            }
        }
    }
}

// Sums all values of var parameter within a block and writes the final result to dest
template <typename T>
__device__ void block_sum_shfl(T var, unsigned int blocksize, T* dest) {
    __shared__ T shMem[warp_size];  // Enough entries for each warp to write its reduction

    // Each warp sums all copies of var from its lanes into var on lane 0
    for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2) {
        var += __shfl_down_sync(0xffffffff, var, offset);
    }

    // Sum all lane 0 vars
    // Each lane 0 writes its var into shared mem
    if ((threadIdx.x & (warp_size - 1)) == 0) {
        unsigned int offsetShMem = threadIdx.x / warp_size;
        shMem[offsetShMem] = var;
    }

    __syncthreads();  // Wait for all warps to finish writing to shared mem

    // Warp 0 sums vars from shared mem written by all warps in the block
    if (threadIdx.x / warp_size == 0) {
        var = shMem[threadIdx.x];
        for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2) {
            var += __shfl_down_sync(0xffffffff, var, offset);
        }
    }

    if (threadIdx.x == 0) {
        *dest = var;
    }
}

// Reset velocity update data structures
void ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::resetTriangleBroadphaseInformation() {
    gpuErrchk(cudaMemset(SD_isTouchingTriangle.data(), 0, SD_isTouchingTriangle.size() * sizeof(unsigned int)));
    gpuErrchk(cudaMemset(BUCKET_countsOfTrianglesTouching.data(), 0,
                         BUCKET_countsOfTrianglesTouching.size() * sizeof(unsigned int)));
    gpuErrchk(cudaMemset(triangles_in_BUCKET_composite.data(), 0,
                         triangles_in_BUCKET_composite.size() * sizeof(unsigned int)));
}

/**
 * This kernel carries out broadphase for the triangle soup
 *
 * Nomenclature:
 *   - SD: subdomain.
 *   - BD: the big-domain, which is the union of all SDs
 *   - NULL_GRANULAR_ID: the equivalent of a non-sphere SD ID, or a non-sphere ID
 *
 * Template arguments:
 *   - CUB_THREADS: the number of threads used in this kernel, comes into play when invoking CUB block collectives
 *
 * Arguments:
 * SD_isTouchingTriangle - array that for each SD indicates how many triangles touch this SD
 *
 * Assumptions:
 *   - The size of the SD for the granular material and for the mesh is the same.
 *   - A mesh triangle cannot touch more than MAX_SDs_TOUCHED_BY_TRIANGLE SDs
 *
 * Basic idea: use domain decomposition on the rectangular box and figure out how the buckets that each triangle
 * touches. The subdomains are axis-aligned relative to the reference frame associated with the *box*. The origin of the
 * box is in the corner of the box. Each CPB is an AAB.
 *
 */
template <unsigned int CUB_THREADS>  //!< Number of threads engaged in block-collective CUB operations (multiple of 32)
__global__ void triangleSoupBroadPhase(
    const TriangleSoupPtr d_triangleSoup,
    unsigned int*
        BUCKET_countsOfTrianglesTouching,  //!< Array that for each BKT indicates how many triangles touch this BKT
    unsigned int*
        triangles_in_BUCKET_composite,  //!< Big array that works in conjunction with BUCKET_countsOfTrianglesTouching.
                                        //!< "triangles_in_BUCKET_composite" says which BKT contains what triangles.
    unsigned int* SD_isTouchingTriangle,  //!< If SD 629 has any triangle touching it, then
                                          //!< SD_isTouchingTriangle[629]>0.
    ParamsPtr gran_params,
    MeshParamsPtr mesh_params) {
    /// Set aside shared memory
    volatile __shared__ unsigned int offsetInComposite_TriangleInBKT_Array[CUB_THREADS * MAX_SDs_TOUCHED_BY_TRIANGLE];
    volatile __shared__ bool shMem_head_flags[CUB_THREADS * MAX_SDs_TOUCHED_BY_TRIANGLE];

    typedef cub::BlockRadixSort<unsigned int, CUB_THREADS, MAX_SDs_TOUCHED_BY_TRIANGLE, unsigned int> BlockRadixSortOP;
    __shared__ typename BlockRadixSortOP::TempStorage temp_storage_sort;

    typedef cub::BlockDiscontinuity<unsigned int, CUB_THREADS> Block_Discontinuity;
    __shared__ typename Block_Discontinuity::TempStorage temp_storage_disc;

    unsigned int triangleIDs[MAX_SDs_TOUCHED_BY_TRIANGLE];
    unsigned int SDsTouched[MAX_SDs_TOUCHED_BY_TRIANGLE];
    unsigned int BKTsTouched[MAX_SDs_TOUCHED_BY_TRIANGLE];

    // Figure out what triangleID this thread will handle. We work with a 1D block structure and a 1D grid structure
    unsigned int myTriangleID = threadIdx.x + blockIdx.x * blockDim.x;
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_TRIANGLE; i++) {
        /// start with a clean slate
        triangleIDs[i] = myTriangleID;
        SDsTouched[i] = NULL_GRANULAR_ID;
        BKTsTouched[i] = NULL_GRANULAR_ID;
    }

    if (myTriangleID < d_triangleSoup->nTrianglesInSoup) {
        triangle_figureOutTouchedSDs(myTriangleID, d_triangleSoup, SDsTouched, gran_params, mesh_params);
    }

    __syncthreads();

    // We are actually not interested in SDs touched, but rather buckets touched. This next step associates SDs with
    // "buckets". To save memory, since most SDs have no triangles, we "randomly" associate several SDs with a bucket.
    // While the assignment of SDs to buckets is "random," the assignment scheme is deterministic: for instance, the
    // triangles in SD 239 will always go to bucket 71. Several SDs send their triangles to the same bucket.
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_TRIANGLE; i++) {
        if (SDsTouched[i] != NULL_GRANULAR_ID) {
            BKTsTouched[i] = hashmapBKTid(SDsTouched[i]) % TRIANGLEBUCKET_COUNT;
        }
    }
    // Earmark SDs that are touched by at least one triangle. This step is needed since when computing the
    // mesh-GrMat interaction we only want to do narrow phase on an SD that actually is touched by triangles. Keep
    // in mind that several SDs deposit their triangles in the same bucket. As such, later on during narrow phase/force
    // computation, if an SD looks for a bucket and sees triangles in there, if we know that this SD is touching zero
    // triangles then that SD is not going to do narrow phase on the triangles in that bucket since these triangles
    // actually are associated with other SDs that happen to deposit their triangles in this same bucket.
    BlockRadixSortOP(temp_storage_sort).Sort(SDsTouched, triangleIDs);
    __syncthreads();

    // Do a winningStreak search on whole block, might not have high utilization here
    bool head_flags[MAX_SDs_TOUCHED_BY_TRIANGLE];
    Block_Discontinuity(temp_storage_disc).FlagHeads(head_flags, SDsTouched, cub::Inequality());
    __syncthreads();

    // If a thread is associated with a legit discontinuity; i.e., not one associated with NULL_GRANULAR_ID, it should
    // flag that SD as being touched by a triangle
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_TRIANGLE; i++) {
        if (head_flags[i] && SDsTouched[i] != NULL_GRANULAR_ID) {
            atomicAdd(SD_isTouchingTriangle + SDsTouched[i], 1);  // Mark that this SD is touched
        }
    }

    // Restore triangleIDS
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_TRIANGLE; i++) {
        if (BKTsTouched[i] != NULL_GRANULAR_ID) {
            triangleIDs[i] = myTriangleID;
        }
    }

    // Back at working with buckets. For all purposes, the role that SDs play in this kernel is over. Everything else in
    // this kernel deals with buckets.
    BlockRadixSortOP(temp_storage_sort).Sort(BKTsTouched, triangleIDs);
    __syncthreads();

    // Prep work that allows later on to do a winningStreak search on whole block
    Block_Discontinuity(temp_storage_disc).FlagHeads(head_flags, BKTsTouched, cub::Inequality());
    __syncthreads();

    // Write back to shared memory; TODO eight-way bank conflicts here - to revisit later.
    // This information should be in shared memory since one thread might reach beyond the MAX_SDs_TOUCHED_BY_TRIANGLE
    // slots associated with it. For instance, BKT 14 might be associated with 50 triangles, in which case the winning
    // streak would be very long and go beyond what a thread stores in its registers. The only way to allow a thread to
    // see what the neighboring thread stores is via shared memory.
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_TRIANGLE; i++) {
        shMem_head_flags[MAX_SDs_TOUCHED_BY_TRIANGLE * threadIdx.x + i] = head_flags[i];
    }

    // Seed offsetInComposite_TriangleInBKT_Array with "no valid ID" so that we know later on what is legit;
    // No shmem bank coflicts here, good access pattern...
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_TRIANGLE; i++) {
        offsetInComposite_TriangleInBKT_Array[i * CUB_THREADS + threadIdx.x] = NULL_GRANULAR_ID;
    }

    __syncthreads();

    // Count how many times a Bucket shows up in conjunction with the collection of CUB_THREADS triangles. There
    // will be some thread divergence here.
    // Loop through each potential BKT, after sorting, and see if it is the start of a head
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_TRIANGLE; i++) {
        // BKT currently touched, could easily be inlined
        unsigned int touchedBucket = BKTsTouched[i];
        if (head_flags[i] && (touchedBucket != NULL_GRANULAR_ID)) {
            // current index into shared datastructure of length 8*CUB_THREADS, could easily be inlined
            unsigned int idInSharedMem = MAX_SDs_TOUCHED_BY_TRIANGLE * threadIdx.x + i;
            unsigned int winningStreak = 0;
            // This is the beginning of a sequence of BKTs with a new ID
            do {
                winningStreak++;
                // Go until we run out of threads on the warp or until we find a new head. Note that the order in this
                // "while" test below is important - it prevents out of bounds indexing
            } while (idInSharedMem + winningStreak < MAX_SDs_TOUCHED_BY_TRIANGLE * CUB_THREADS &&
                     !(shMem_head_flags[idInSharedMem + winningStreak]));

            // if (touchedSD >= nSDs_X * nSDs_Y * nSDs_Z) {
            //     printf("invalid SD index %u on thread %u\n", mySphereID, touchedSD);
            // }

            // Store start of new entries
            unsigned int offset = atomicAdd(BUCKET_countsOfTrianglesTouching + touchedBucket, winningStreak);

            // The value offset now gives a *relative* offset in the composite array.
            // Get the absolute offset
            offset += touchedBucket * MAX_TRIANGLE_COUNT_PER_BUCKET;

            // Produce the offsets for this streak of triangles with identical BKT ids
            for (unsigned int w = 0; w < winningStreak; w++)
                offsetInComposite_TriangleInBKT_Array[idInSharedMem + w] = offset++;
        }
    }

    __syncthreads();  // needed since we write to shared memory above; i.e., offsetInComposite_SphInSD_Array

    // Write out the data now; register with triangles_in_BUCKET_composite each triangle that touches a certain BKT
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_TRIANGLE; i++) {
        unsigned int offset = offsetInComposite_TriangleInBKT_Array[MAX_SDs_TOUCHED_BY_TRIANGLE * threadIdx.x + i];
        if (offset != NULL_GRANULAR_ID) {
            triangles_in_BUCKET_composite[offset] = triangleIDs[i];
        }
    }
}

/**
This kernel call figures out forces on a sphere and carries out numerical integration to get the velocity updates of a
sphere.
N_CUDATHREADS - Number of threads in a CUDA block
MAX_NSPHERES_PER_SD - Max number of elements per SD. Shoudld be a power of two
TRIANGLE_FAMILIES - The number of families that the triangles can belong to

Overview of implementation: One warp of threads will work on 32 triangles at a time to figure out the force that
they impress on a particular sphere. Note that each sphere enlists the services of one warp. If there are, say,
73 triangles touching this SD, it will take three trips to figure out the total force that the triangles will
impress upon the sphere that is active. If there are 256 threads in the block, then there will be 8 "active"
spheres since there are 8 warps in the block. Each thread in the block has enough registers to accummulate the
force felt by each "family", force that is the result of an interaction between a triangle and a sphere.
Say if sphere 232 touches a triangle that belongs to family 2, then a set of 6 generalized forces is going to
be produced to account for the interaction between the said triangle and sphere 232.
*/

// TODO fix this
#define TRIANGLE_FAMILIES 4

template <unsigned int N_CUDATHREADS>
__global__ void interactionTerrain_TriangleSoup(
    const float alpha_h_bar,
    TriangleSoupPtr d_triangleSoup,  //!< Contains information pertaining to triangle soup (in device mem.)
    int* d_sphere_pos_X,
    int* d_sphere_pos_Y,
    int* d_sphere_pos_Z,
    float* d_sphere_pos_X_dt,
    float* d_sphere_pos_Y_dt,
    float* d_sphere_pos_Z_dt,
    float* d_sphere_pos_X_dt_update,
    float* d_sphere_pos_Y_dt_update,
    float* d_sphere_pos_Z_dt_update,
    unsigned int* BKT_countsOfTrianglesTouching,  //!< Array that for each SD indicates how many triangles touch this SD
    unsigned int* triangles_in_BKT_composite,     //!< Big array that works in conjunction with SD_isTouchingTriangle.
    unsigned int*
        SD_countsOfGrElemsTouching,         //!< Array that for each SD indicates how many grain elements touch this SD
    unsigned int* grElems_in_SD_composite,  //!< Big array that works in conjunction with SD_countsOfGrElemsTouching.
                                            //!< "grElems_in_SD_composite" says which SD contains what grElements.
    unsigned int* SD_isTouchingTriangle,    //!< The length of this array is equal to number of SDs. If SD 423 is
                                            //!< touched by any triangle, then SD_isTouchingTriangle[423]>0.

    ParamsPtr gran_params,
    MeshParamsPtr mesh_params) {
    __shared__ unsigned int grElemID[MAX_COUNT_OF_DEs_PER_SD];        //!< global ID of the grElements touching this SD
    __shared__ unsigned int triangID[MAX_TRIANGLE_COUNT_PER_BUCKET];  //!< global ID of the triangles touching this SD

    __shared__ int sphX[MAX_COUNT_OF_DEs_PER_SD];  //!< X coordinate of the grElement
    __shared__ int sphY[MAX_COUNT_OF_DEs_PER_SD];  //!< Y coordinate of the grElement
    __shared__ int sphZ[MAX_COUNT_OF_DEs_PER_SD];  //!< Z coordinate of the grElement

    __shared__ float sphere_X_DOT[MAX_COUNT_OF_DEs_PER_SD];
    __shared__ float sphere_Y_DOT[MAX_COUNT_OF_DEs_PER_SD];
    __shared__ float sphere_Z_DOT[MAX_COUNT_OF_DEs_PER_SD];

    __shared__ double node1_X[MAX_TRIANGLE_COUNT_PER_BUCKET];  //!< X coordinate of the 1st node of the triangle
    __shared__ double node1_Y[MAX_TRIANGLE_COUNT_PER_BUCKET];  //!< Y coordinate of the 1st node of the triangle
    __shared__ double node1_Z[MAX_TRIANGLE_COUNT_PER_BUCKET];  //!< Z coordinate of the 1st node of the triangle

    __shared__ double node2_X[MAX_TRIANGLE_COUNT_PER_BUCKET];  //!< X coordinate of the 2nd node of the triangle
    __shared__ double node2_Y[MAX_TRIANGLE_COUNT_PER_BUCKET];  //!< Y coordinate of the 2nd node of the triangle
    __shared__ double node2_Z[MAX_TRIANGLE_COUNT_PER_BUCKET];  //!< Z coordinate of the 2nd node of the triangle

    __shared__ double node3_X[MAX_TRIANGLE_COUNT_PER_BUCKET];  //!< X coordinate of the 3rd node of the triangle
    __shared__ double node3_Y[MAX_TRIANGLE_COUNT_PER_BUCKET];  //!< Y coordinate of the 3rd node of the triangle
    __shared__ double node3_Z[MAX_TRIANGLE_COUNT_PER_BUCKET];  //!< Z coordinate of the 3rd node of the triangle

    volatile __shared__ float tempShMem[6 * (N_CUDATHREADS / warp_size)];  // used to do a block-level reduce

    float forceActingOnSphere[3];  //!< 3 registers will hold the value of the force on the sphere
    float genForceActingOnMeshes[TRIANGLE_FAMILIES * 6];  //!< 6 components per family: 3 forces and 3 torques

    // define an alias first
#define thisSD blockIdx.x

    if (SD_isTouchingTriangle[thisSD] == 0) {
        return;  // no triangle touches this SD; return right away
    }
    unsigned int nSD_spheres = SD_countsOfGrElemsTouching[thisSD];
    if (nSD_spheres == 0) {
        return;  // no sphere to speak of in this SD
    }

    // Getting here means that there are both triangles and DEs in this SD.
    // First, figure out which bucket stores the triangles associated with this SD.
    unsigned int whichBKT = hashmapBKTid(thisSD) % TRIANGLEBUCKET_COUNT;
    unsigned int nBKT_triangles = BKT_countsOfTrianglesTouching[whichBKT];

    // Unpleasant fact: this bucket might store more than the triangles associated with this SD. The narrow phase is
    // done for ALL triangles in this bucket with the expectation that if a triangle does not belong to this SD, the
    // narrow phase will prune this triangle fast and the penalty associated with storing triangles from multiple SDs
    // into one bucket is not stiff.

    // Populate the shared memory with terrain data
    unsigned int tripsToCoverSpheres = (nSD_spheres + blockDim.x - 1) / blockDim.x;
    unsigned int local_ID = threadIdx.x;
    for (unsigned int sphereTrip = 0; sphereTrip < tripsToCoverSpheres; sphereTrip++) {
        if (local_ID < nSD_spheres) {
            unsigned int globalID = grElems_in_SD_composite[local_ID + thisSD * MAX_COUNT_OF_DEs_PER_SD];
            grElemID[local_ID] = globalID;
            sphX[local_ID] = d_sphere_pos_X[globalID];
            sphY[local_ID] = d_sphere_pos_Y[globalID];
            sphZ[local_ID] = d_sphere_pos_Z[globalID];

            sphere_X_DOT[local_ID] = d_sphere_pos_X_dt[globalID];
            sphere_Y_DOT[local_ID] = d_sphere_pos_Y_dt[globalID];
            sphere_Z_DOT[local_ID] = d_sphere_pos_Z_dt[globalID];
        }
        local_ID += blockDim.x;
    }
    // Populate the shared memory with mesh triangle data
    unsigned int tripsToCoverTriangles = (nBKT_triangles + blockDim.x - 1) / blockDim.x;
    local_ID = threadIdx.x;
    for (unsigned int triangTrip = 0; triangTrip < tripsToCoverTriangles; triangTrip++) {
        if (local_ID < nBKT_triangles) {
            unsigned int globalID = triangles_in_BKT_composite[local_ID + whichBKT * MAX_TRIANGLE_COUNT_PER_BUCKET];
            triangID[local_ID] = globalID;

            // Read node positions from global memory into shared memory
            // NOTE implicit cast from float to double here
            node1_X[local_ID] = d_triangleSoup->node1_X[globalID];
            node1_Y[local_ID] = d_triangleSoup->node1_Y[globalID];
            node1_Z[local_ID] = d_triangleSoup->node1_Z[globalID];

            node2_X[local_ID] = d_triangleSoup->node2_X[globalID];
            node2_Y[local_ID] = d_triangleSoup->node2_Y[globalID];
            node2_Z[local_ID] = d_triangleSoup->node2_Z[globalID];

            node3_X[local_ID] = d_triangleSoup->node3_X[globalID];
            node3_Y[local_ID] = d_triangleSoup->node3_Y[globalID];
            node3_Z[local_ID] = d_triangleSoup->node3_Z[globalID];
        }
        local_ID += blockDim.x;
    }

    __syncthreads();  // this call ensures data is in its place in shared memory

    // Zero out the force and torque at the onset of the computation
    for (local_ID = 0; local_ID < TRIANGLE_FAMILIES; local_ID++) {
        unsigned int dummyOffset = 6 * local_ID;
        /// forces acting on the triangle, in global reference frame
        genForceActingOnMeshes[dummyOffset++] = 0.f;
        genForceActingOnMeshes[dummyOffset++] = 0.f;
        genForceActingOnMeshes[dummyOffset++] = 0.f;
        /// torques with respect to global reference frame, expressed in global reference frame
        genForceActingOnMeshes[dummyOffset++] = 0.f;
        genForceActingOnMeshes[dummyOffset++] = 0.f;
        genForceActingOnMeshes[dummyOffset] = 0.f;
    }

    // Each sphere has one warp of threads dedicated to identifying all triangles that this sphere
    // touches. Upon a contact event, we'll compute the normal force on the sphere; and, the force and torque
    // impressed upon the triangle that comes in contact with this sphere

    unsigned int nSpheresProcessedAtOneTime = blockDim.x / warp_size;  // One warp allocated to slave serving one sphere
    tripsToCoverSpheres = (nSD_spheres + nSpheresProcessedAtOneTime - 1) / nSpheresProcessedAtOneTime;
    tripsToCoverTriangles = (nBKT_triangles + warp_size - 1) / warp_size;

    unsigned sphere_Local_ID = threadIdx.x / warp_size;  // the local ID of the sphere served by one warp of threads
    for (unsigned int sphereTrip = 0; sphereTrip < tripsToCoverSpheres; sphereTrip++) {
        /// before starting dealing with a sphere, zero out the forces acting on it; all threads in the block are
        /// doing this
        forceActingOnSphere[0] = 0.f;
        forceActingOnSphere[1] = 0.f;
        forceActingOnSphere[2] = 0.f;
        if (sphere_Local_ID < nSD_spheres) {
            // Figure out which triangles this sphere collides with; each thread in a warp slaving for this sphere
            // looks at one triangle at a time. The collection of threads in the warp sweeps through all the
            // triangles in this BUCKET. NOTE: to avoid double-counting, a sphere-triangle collision event is
            // counted only if the collision point is in this SD.
            unsigned int targetTriangle = (threadIdx.x & (warp_size - 1));  // computes modulo 32 of the thread index
            for (unsigned int triangTrip = 0; triangTrip < tripsToCoverTriangles; triangTrip++) {
                if (targetTriangle < nBKT_triangles) {
                    /// we have a valid sphere and a valid triganle; check if in contact
                    double3 norm;
                    double3 pt1;
                    float depth;

                    // Transform vertices into GRF SU
                    double3 A, B, C;  // vertices of the triangle

                    // Read LRF UU vertices of the triangle
                    // We have an int to double conversion here
                    // NOTE: uncoalesced
                    A.x = node1_X[targetTriangle];
                    A.y = node1_Y[targetTriangle];
                    A.z = node1_Z[targetTriangle];

                    B.x = node2_X[targetTriangle];
                    B.y = node2_Y[targetTriangle];
                    B.z = node2_Z[targetTriangle];

                    C.x = node3_X[targetTriangle];
                    C.y = node3_Y[targetTriangle];
                    C.z = node3_Z[targetTriangle];

                    // Transform LRF to GRF
                    unsigned int fam = d_triangleSoup->triangleFamily_ID[triangID[targetTriangle]];
                    A = apply_frame_transform<double, double3>(A, mesh_params->fam_frame_narrow[fam].pos,
                                                               mesh_params->fam_frame_narrow[fam].rot_mat);
                    B = apply_frame_transform<double, double3>(B, mesh_params->fam_frame_narrow[fam].pos,
                                                               mesh_params->fam_frame_narrow[fam].rot_mat);
                    C = apply_frame_transform<double, double3>(C, mesh_params->fam_frame_narrow[fam].pos,
                                                               mesh_params->fam_frame_narrow[fam].rot_mat);

                    // Convert UU to SU
                    convert_pos_UU2SU<double3>(A, gran_params);
                    convert_pos_UU2SU<double3>(B, gran_params);
                    convert_pos_UU2SU<double3>(C, gran_params);

                    // NOTE implicit cast from int to double
                    double3 sphCntr = make_double3(sphX[sphere_Local_ID], sphY[sphere_Local_ID], sphZ[sphere_Local_ID]);

                    // TODO Conlain, check this force computation
                    // If there is a collision, add an impulse to the sphere
                    if (face_sphere_cd(A, B, C, sphCntr, gran_params->sphereRadius_SU, norm, depth, pt1) &&
                        SDTripletID(pointSDTriplet(pt1.x, pt1.y, pt1.z, gran_params), gran_params) == thisSD) {
                        float scalingFactor = alpha_h_bar * mesh_params->Kn_s2m_SU;

                        // Use the CD information to compute the force on the grElement
                        double deltaX = -depth * norm.x;
                        double deltaY = -depth * norm.y;
                        double deltaZ = -depth * norm.z;

                        // Velocity difference, it's better to do a coalesced access here than a fragmented access
                        // inside
                        // TODO: Assumes stationary mesh
                        float deltaX_dot_n = sphere_X_DOT[sphere_Local_ID];
                        float deltaY_dot_n = sphere_Y_DOT[sphere_Local_ID];
                        float deltaZ_dot_n = sphere_Z_DOT[sphere_Local_ID];

                        float projection = deltaX_dot_n * norm.x + deltaY_dot_n * norm.y + deltaZ_dot_n * norm.z;

                        deltaX_dot_n = projection * norm.x;
                        deltaY_dot_n = projection * norm.y;
                        deltaZ_dot_n = projection * norm.z;

                        // Compute force updates for spring term
                        float springTermX = scalingFactor * deltaX;
                        float springTermY = scalingFactor * deltaY;
                        float springTermZ = scalingFactor * deltaZ;

                        // Compute force updates for damping term
                        // TODO effective mass based on mesh mass
                        const float m_eff = 0.5;

                        float dampingTermX = -mesh_params->Gamma_n_s2m_SU * deltaX_dot_n * m_eff * alpha_h_bar;
                        float dampingTermY = -mesh_params->Gamma_n_s2m_SU * deltaY_dot_n * m_eff * alpha_h_bar;
                        float dampingTermZ = -mesh_params->Gamma_n_s2m_SU * deltaZ_dot_n * m_eff * alpha_h_bar;

                        // Compute force updates for cohesion term, is opposite the spring term
                        // TODO What to use for the mass being affected by gravity??
                        float cohesionConstant =
                            1.0 * gran_params->gravMag_SU * gran_params->cohesion_ratio * alpha_h_bar;

                        // NOTE the cancelation of two negatives
                        float cohesionTermX = cohesionConstant * deltaX / depth;
                        float cohesionTermY = cohesionConstant * deltaY / depth;
                        float cohesionTermZ = cohesionConstant * deltaZ / depth;

                        // Sum contributing forces
                        float bodyA_X_velCorr = springTermX + dampingTermX + cohesionTermX;
                        float bodyA_Y_velCorr = springTermY + dampingTermY + cohesionTermY;
                        float bodyA_Z_velCorr = springTermZ + dampingTermZ + cohesionTermZ;

                        // TODO: Use the CD information to compute the force and torque on the family of this triangle
                        forceActingOnSphere[0] += bodyA_X_velCorr;
                        forceActingOnSphere[1] += bodyA_Y_velCorr;
                        forceActingOnSphere[2] += bodyA_Z_velCorr;

                        // total force is opposite the triangle normal
                        // force on mesh is total force projected from the contact point on the triangle to mesh center
                        // torque
                        // TODO assumes pos is the center of mass of the mesh
                        // TODO precision?
                        double3 meshCenter = make_double3(mesh_params->fam_frame_narrow[fam].pos[0],
                                                          mesh_params->fam_frame_narrow[fam].pos[1],
                                                          mesh_params->fam_frame_narrow[fam].pos[2]);

                        double3 toCenter = pt1 - meshCenter;
                        toCenter = toCenter / Length(toCenter);
                        double3 force_total =
                            make_double3(bodyA_X_velCorr, bodyA_Y_velCorr, bodyA_Z_velCorr) / alpha_h_bar;
                        double3 force_N = Dot(force_total, toCenter) * toCenter;

                        // TODO reorder for register use
                        // TODO be waaaayyyy less dumb about register use
                        double3 force_T = force_total - force_N;
                        double3 fromCenter = meshCenter - pt1;
                        double3 torque = Cross(fromCenter, force_T);

                        unsigned int fam = d_triangleSoup->triangleFamily_ID[targetTriangle];
                        genForceActingOnMeshes[fam * 6 + 0] += force_N.x;
                        genForceActingOnMeshes[fam * 6 + 1] += force_N.y;
                        genForceActingOnMeshes[fam * 6 + 2] += force_N.z;

                        genForceActingOnMeshes[fam * 6 + 3] += torque.x;
                        genForceActingOnMeshes[fam * 6 + 4] += torque.y;
                        genForceActingOnMeshes[fam * 6 + 5] += torque.z;
                    }
                }
                targetTriangle += warp_size;
            }  // end of per-triangle loop

            // down to the point where we need to collect the forces from all the threads in the warp; this is a
            // warp reduce operation. The resultant force acting on this grElement is stored in the first lane of
            // the warp. NOTE: In this warp-level operations participate only the warps that are slaving for a
            // sphere; i.e., some warps see no action
            for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2) {
                forceActingOnSphere[0] += __shfl_down_sync(0xffffffff, forceActingOnSphere[0], offset);
            }
            for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2) {
                forceActingOnSphere[1] += __shfl_down_sync(0xffffffff, forceActingOnSphere[1], offset);
            }
            for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2) {
                forceActingOnSphere[2] += __shfl_down_sync(0xffffffff, forceActingOnSphere[2], offset);
            }

            // done with the computation of all the contacts that the triangles impress on this sphere. Update the
            // position of the sphere based on this force

            // Write velocity update for this sphere back to global memory from each lane 0
            if ((threadIdx.x & (warp_size - 1)) == 0) {
                atomicAdd(d_sphere_pos_X_dt_update + grElemID[sphere_Local_ID], forceActingOnSphere[0]);
                atomicAdd(d_sphere_pos_Y_dt_update + grElemID[sphere_Local_ID], forceActingOnSphere[1]);
                atomicAdd(d_sphere_pos_Z_dt_update + grElemID[sphere_Local_ID], forceActingOnSphere[2]);
            }
        }                                               // end of valid sphere if
        sphere_Local_ID += nSpheresProcessedAtOneTime;  // go to next set of spheress
    }                                                   // end of per-sphere loop

    // Done computing the forces acting on the triangles in this SD. A block reduce is carried out next. Start by doing
    // a reduce at the warp level.
    for (unsigned int fam = 0; fam < TRIANGLE_FAMILIES; fam++) {
        /// six generalized forces acting on the triangle, expressed in the global reference frame
        unsigned int dummyIndx = 6 * fam;
        for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2) {
            genForceActingOnMeshes[dummyIndx] +=
                __shfl_down_sync(0xffffffff, genForceActingOnMeshes[dummyIndx], offset);
        }
        dummyIndx++;

        for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2) {
            genForceActingOnMeshes[dummyIndx] +=
                __shfl_down_sync(0xffffffff, genForceActingOnMeshes[dummyIndx], offset);
        }
        dummyIndx++;

        for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2) {
            genForceActingOnMeshes[dummyIndx] +=
                __shfl_down_sync(0xffffffff, genForceActingOnMeshes[dummyIndx], offset);
        }
        dummyIndx++;

        for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2) {
            genForceActingOnMeshes[dummyIndx] +=
                __shfl_down_sync(0xffffffff, genForceActingOnMeshes[dummyIndx], offset);
        }
        dummyIndx++;

        for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2) {
            genForceActingOnMeshes[dummyIndx] +=
                __shfl_down_sync(0xffffffff, genForceActingOnMeshes[dummyIndx], offset);
        }
        dummyIndx++;

        for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2) {
            genForceActingOnMeshes[dummyIndx] +=
                __shfl_down_sync(0xffffffff, genForceActingOnMeshes[dummyIndx], offset);
        }
    }

    __syncthreads();

    // Lane zero in each warp holds the result of a warp-level reduce operation. Sum up these "Lane zero" values in
    // the final result, which is block-level
    bool threadIsLaneZeroInWarp = ((threadIdx.x & (warp_size - 1)) == 0);
    for (unsigned int fam = 0; fam < TRIANGLE_FAMILIES; fam++) {
        unsigned int offsetGenForceArray = 6 * fam;
        // Place in ShMem forces/torques (expressed in global reference frame) acting on this family of triangles
        if (threadIsLaneZeroInWarp) {
            unsigned int offsetShMem = 6 * (threadIdx.x / warp_size);
            tempShMem[offsetShMem++] = genForceActingOnMeshes[offsetGenForceArray++];
            tempShMem[offsetShMem++] = genForceActingOnMeshes[offsetGenForceArray++];
            tempShMem[offsetShMem++] = genForceActingOnMeshes[offsetGenForceArray++];

            tempShMem[offsetShMem++] = genForceActingOnMeshes[offsetGenForceArray++];
            tempShMem[offsetShMem++] = genForceActingOnMeshes[offsetGenForceArray++];
            tempShMem[offsetShMem] = genForceActingOnMeshes[offsetGenForceArray];
        }
        __syncthreads();

        float tri_force_torque[3];
        if (threadIdx.x < warp_size) {
            // only first warp in block participates in this reduce operation.
            // ASSUMPTION: warp_size is larger than or equal to N_CUDATHREADS / warp_size. This is true today as
            // N_CUDATHREADS cannot be larger than 1024 and warp_size is 32.

            // Work on forces first. Place data from ShMem into registers associated w/ first warp
            unsigned int offsetShMem = 6 * threadIdx.x;
            if (threadIdx.x < (N_CUDATHREADS / warp_size)) {
                tri_force_torque[0] = tempShMem[offsetShMem++];
                tri_force_torque[1] = tempShMem[offsetShMem++];
                tri_force_torque[2] = tempShMem[offsetShMem++];  // NOTE: ++ is needed here, offsetShMem used later
            } else {
                // this is hit only by a subset of threads from first warp of the block
                tri_force_torque[0] = 0.f;
                tri_force_torque[1] = 0.f;
                tri_force_torque[2] = 0.f;
            }

            offsetGenForceArray = 6 * fam;
            // X component of the force on mesh "fam"
            for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2) {
                tri_force_torque[0] += __shfl_down_sync(0xffffffff, tri_force_torque[0], offset);
            }
            genForceActingOnMeshes[offsetGenForceArray++] = tri_force_torque[0];

            // Y component of the force on mesh "fam"
            for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2) {
                tri_force_torque[1] += __shfl_down_sync(0xffffffff, tri_force_torque[1], offset);
            }
            genForceActingOnMeshes[offsetGenForceArray++] = tri_force_torque[1];

            // Z component of the force on mesh "fam"
            for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2) {
                tri_force_torque[2] += __shfl_down_sync(0xffffffff, tri_force_torque[2], offset);
            }
            genForceActingOnMeshes[offsetGenForceArray++] = tri_force_torque[2];

            // Finally, work on torques
            if (threadIdx.x < (N_CUDATHREADS / warp_size)) {
                tri_force_torque[0] = tempShMem[offsetShMem++];
                tri_force_torque[1] = tempShMem[offsetShMem++];
                tri_force_torque[2] = tempShMem[offsetShMem];
            } else {
                // this is hit only by a subset of threads from first warp of the block
                tri_force_torque[0] = 0.f;
                tri_force_torque[1] = 0.f;
                tri_force_torque[2] = 0.f;
            }

            // X component of the torque on mesh "fam"
            for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2) {
                tri_force_torque[0] += __shfl_down_sync(0xffffffff, tri_force_torque[0], offset);
            }
            genForceActingOnMeshes[offsetGenForceArray++] = tri_force_torque[0];

            // Y component of the torque on mesh "fam"
            for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2) {
                tri_force_torque[1] += __shfl_down_sync(0xffffffff, tri_force_torque[1], offset);
            }
            genForceActingOnMeshes[offsetGenForceArray++] = tri_force_torque[1];

            // Z component of the torque on mesh "fam"
            for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2) {
                tri_force_torque[2] += __shfl_down_sync(0xffffffff, tri_force_torque[2], offset);
            }
            genForceActingOnMeshes[offsetGenForceArray] = tri_force_torque[2];
        }  /// this is the end of the "for each mesh" loop

        // At this point, the first thread of the block has in genForceActingOnMeshes[6*TRIANGLE_FAMILIES] the
        // forces and torques acting on each mesh family. Bcast the force values to all threads in the warp.
        // To this end, synchronize all threads in warp and get "value" from lane 0
        for (fam = 0; fam < 6 * TRIANGLE_FAMILIES; fam++) {
            genForceActingOnMeshes[fam] = __shfl_sync(0xffffffff, genForceActingOnMeshes[fam], 0);
        }
        // At this point, all threads in the *first* warp have the generalized forces acting on all meshes. Do an
        // atomic add to compund the value of the generalized forces acting on the meshes that come in contact with
        // the granular material.
        if (threadIdx.x < warp_size) {
            unsigned int nTrips = (6 * TRIANGLE_FAMILIES + warp_size - 1) / warp_size;
            for (unsigned int fam_trip = 0; fam_trip < nTrips; fam_trip++) {
                unsigned int offset = threadIdx.x + fam_trip * warp_size;
                if (offset < 6 * TRIANGLE_FAMILIES) {
                    atomicAdd(d_triangleSoup->generalizedForcesPerFamily + offset, genForceActingOnMeshes[offset]);
                }
            }
        }
    }
}

/// Copy const triangle data to device
void ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::copy_triangle_data_to_device() {
    // unified memory does some copying for us, cool
    tri_params->Gamma_n_s2m_SU = 0;  // no damping on mesh for now
    tri_params->Kn_s2m_SU = K_n_s2m_SU;
    tri_params->Gamma_n_s2m_SU = Gamma_n_s2m_SU;

    SD_isTouchingTriangle.resize(nSDs);
}

__host__ void ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::initialize() {
    switch_to_SimUnits();

    double K_stiffness = get_max_K();
    float scalingFactor = (1.f / (1.f * gran_params->psi_T * gran_params->psi_T * gran_params->psi_h));
    K_n_s2m_SU = scalingFactor * (YoungModulus_SPH2MESH / K_stiffness);
    Gamma_n_s2m_SU = 0.005;

    generate_DEs();

    // Set aside memory for holding data structures worked with. Get some initializations going
    printf("setup_simulation\n");
    setup_simulation();
    copy_const_data_to_device();
    copy_triangle_data_to_device();
    copyBD_Frame_to_device();
    gpuErrchk(cudaDeviceSynchronize());

    determine_new_stepSize_SU();

    // Seed arrays that are populated by the kernel call
    resetBroadphaseInformation();

    // Figure our the number of blocks that need to be launched to cover the box
    unsigned int nBlocks = (nDEs + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;
    printf("doing priming!\n");
    printf("max possible composite offset is %zu\n", (size_t)nSDs * MAX_COUNT_OF_DEs_PER_SD);

    primingOperationsRectangularBox<CUDA_THREADS_PER_BLOCK>
        <<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(pos_X.data(), pos_Y.data(), pos_Z.data(), SD_NumOf_DEs_Touching.data(),
                                              DEs_in_SD_composite.data(), nDEs, gran_params);
    gpuErrchk(cudaDeviceSynchronize());
    printf("priming finished!\n");

    printf("z grav term with timestep %f is %f\n", stepSize_SU, stepSize_SU * stepSize_SU * gravity_Z_SU);
}

__host__ void ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::advance_simulation(float duration) {
    // Figure our the number of blocks that need to be launched to cover the box
    unsigned int nBlocks = (nDEs + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

    // Settling simulation loop.
    float duration_SU = std::ceil(duration / (gran_params->TIME_UNIT * gran_params->psi_h));
    unsigned int nsteps = duration_SU / stepSize_SU;

    VERBOSE_PRINTF("advancing by %f at timestep %f, %u timesteps at approx user timestep %f\n", duration_SU,
                   stepSize_SU, nsteps, duration / nsteps);

    VERBOSE_PRINTF("Starting Main Simulation loop!\n");
    // Run the simulation, there are aggressive synchronizations because we want to have no race conditions
    for (float crntTime_SU = 0; crntTime_SU < stepSize_SU * nsteps; crntTime_SU += stepSize_SU) {
        determine_new_stepSize_SU();  // doesn't always change the timestep

        // Update the position and velocity of the BD, if relevant
        if (!BD_is_fixed) {
            updateBDPosition(stepSize_SU);
        }
        resetUpdateInformation();
        resetTriangleBroadphaseInformation();

        VERBOSE_PRINTF("Starting computeVelocityUpdates!\n");

        // Compute forces and crank into vel updates, we have 2 kernels to avoid a race condition
        computeVelocityUpdates<MAX_COUNT_OF_DEs_PER_SD><<<nSDs, MAX_COUNT_OF_DEs_PER_SD>>>(
            stepSize_SU, pos_X.data(), pos_Y.data(), pos_Z.data(), pos_X_dt_update.data(), pos_Y_dt_update.data(),
            pos_Z_dt_update.data(), SD_NumOf_DEs_Touching.data(), DEs_in_SD_composite.data(), pos_X_dt.data(),
            pos_Y_dt.data(), pos_Z_dt.data(), gran_params);

        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        if (meshSoup_DEVICE->nTrianglesInSoup != 0) {
            // broadphase the triangles
            // TODO check these block/thread counts
            triangleSoupBroadPhase<CUDA_THREADS_PER_BLOCK>
                <<<(meshSoup_DEVICE->nTrianglesInSoup + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK,
                   CUDA_THREADS_PER_BLOCK>>>(meshSoup_DEVICE, BUCKET_countsOfTrianglesTouching.data(),
                                             triangles_in_BUCKET_composite.data(), SD_isTouchingTriangle.data(),
                                             gran_params, tri_params);
        }
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        if (meshSoup_DEVICE->nFamiliesInSoup != 0) {
            // TODO please do not use a template here
            // compute sphere-triangle forces
            interactionTerrain_TriangleSoup<CUDA_THREADS_PER_BLOCK><<<nSDs, MAX_COUNT_OF_DEs_PER_SD>>>(
                stepSize_SU, meshSoup_DEVICE, pos_X.data(), pos_Y.data(), pos_Z.data(), pos_X_dt.data(),
                pos_Y_dt.data(), pos_Z_dt.data(), pos_X_dt_update.data(), pos_Y_dt_update.data(),
                pos_Z_dt_update.data(), BUCKET_countsOfTrianglesTouching.data(), triangles_in_BUCKET_composite.data(),
                SD_NumOf_DEs_Touching.data(), DEs_in_SD_composite.data(), SD_isTouchingTriangle.data(), gran_params,
                tri_params);
        }
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        VERBOSE_PRINTF("Resetting broadphase info!\n");

        resetBroadphaseInformation();

        VERBOSE_PRINTF("Starting updatePositions!\n");
        updatePositions<CUDA_THREADS_PER_BLOCK><<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(
            stepSize_SU, pos_X.data(), pos_Y.data(), pos_Z.data(), pos_X_dt.data(), pos_Y_dt.data(), pos_Z_dt.data(),
            pos_X_dt_update.data(), pos_Y_dt_update.data(), pos_Z_dt_update.data(), SD_NumOf_DEs_Touching.data(),
            DEs_in_SD_composite.data(), nDEs, gran_params);

        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());
        elapsedSimTime += stepSize_SU * gran_params->TIME_UNIT * gran_params->psi_h;  // Advance current time
    }
    return;
}
}  // namespace granular
}  // namespace chrono
