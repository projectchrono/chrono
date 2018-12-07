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

    // Perform broadphase on UU vertices
    int3 SDA = pointSDTriplet(vA.x, vA.y, vA.z, gran_params);  // SD indices for point A
    int3 SDB = pointSDTriplet(vB.x, vB.y, vB.z, gran_params);  // SD indices for point B
    int3 SDC = pointSDTriplet(vC.x, vC.y, vC.z, gran_params);  // SD indices for point C

    int L[3];  // Min SD index along each axis
    int U[3];  // Max SD index along each axis

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
        if (U[axes_diff] - L[axes_diff] >= MAX_SDs_TOUCHED_BY_TRIANGLE) {
            ABORTABORTABORT("SD_count exceeds MAX_SDs_TOUCHED_BY_TRIANGLE\n");
        }
        for (int i = L[axes_diff]; i <= U[axes_diff]; i++) {
            SD_i[axes_diff] = i;  // current SD index along this direction
            touchedSDs[SD_count++] = SDTripletID(SD_i, gran_params);
        }
        return;
    }

    // Case 3: Triangle spans more than one dimension of nSD_spheres
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

void ChSystemGranular_MonodisperseSMC_trimesh::resetTriangleForces() {
    gpuErrchk(cudaMemset(meshSoup_DEVICE->generalizedForcesPerFamily, 0, 6 * MAX_TRIANGLE_FAMILIES * sizeof(float)));
}
// Reset triangle broadphase data structures
void ChSystemGranular_MonodisperseSMC_trimesh::resetTriangleBroadphaseInformation() {
    gpuErrchk(cudaMemset(SD_isTouchingTriangle.data(), 0, SD_isTouchingTriangle.size() * sizeof(unsigned int)));
    gpuErrchk(cudaMemset(BUCKET_countsOfTrianglesTouching.data(), 0,
                         BUCKET_countsOfTrianglesTouching.size() * sizeof(unsigned int)));
    gpuErrchk(cudaMemset(triangles_in_BUCKET_composite.data(), NULL_GRANULAR_ID,
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
    GranParamsPtr gran_params,
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

            // Store start of new entries
            unsigned int offset = atomicAdd(BUCKET_countsOfTrianglesTouching + touchedBucket, winningStreak);

            if (offset + winningStreak > MAX_TRIANGLE_COUNT_PER_BUCKET) {
                ABORTABORTABORT("TOO MANY TRIANGLES IN BUCKET %u, SD %u,  %u TRIANGLES TOUCHING, MAX IS %u\n",
                                touchedBucket, SDsTouched[i], offset + winningStreak, MAX_TRIANGLE_COUNT_PER_BUCKET);
            }
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
MAX_TRIANGLE_FAMILIES - The number of families that the triangles can belong to

Overview of implementation: One warp of threads will work on 32 triangles at a time to figure out the force that
they impress on a particular sphere. Note that each sphere enlists the services of one warp. If there are, say,
73 triangles touching this SD, it will take three trips to figure out the total force that the triangles will
impress upon the sphere that is active. If there are 256 threads in the block, then there will be 8 "active"
spheres since there are 8 warps in the block. Each thread in the block has enough registers to accummulate the
force felt by each "family", force that is the result of an interaction between a triangle and a sphere.
Say if sphere 232 touches a triangle that belongs to family 2, then a set of 6 generalized forces is going to
be produced to account for the interaction between the said triangle and sphere 232.
*/

template <unsigned int N_CUDATHREADS>
__global__ void interactionTerrain_TriangleSoup(
    TriangleSoupPtr d_triangleSoup,  //!< Contains information pertaining to triangle soup (in device mem.)
    sphereDataStruct sphere_data,
    unsigned int* BKT_countsOfTrianglesTouching,  //!< Array that for each SD indicates how many triangles touch this SD
    unsigned int* triangles_in_BKT_composite,     //!< Big array that works in conjunction with SD_isTouchingTriangle.
    unsigned int*
        SD_countsOfGrElemsTouching,         //!< Array that for each SD indicates how many grain elements touch this SD
    unsigned int* grElems_in_SD_composite,  //!< Big array that works in conjunction with SD_countsOfGrElemsTouching.
                                            //!< "grElems_in_SD_composite" says which SD contains what grElements.
    unsigned int* SD_isTouchingTriangle,    //!< The length of this array is equal to number of SDs. If SD 423 is
                                            //!< touched by any triangle, then SD_isTouchingTriangle[423]>0.

    GranParamsPtr gran_params,
    MeshParamsPtr mesh_params) {
    __shared__ unsigned int grElemID[MAX_COUNT_OF_DEs_PER_SD];        //!< global ID of the grElements touching this SD
    __shared__ unsigned int triangID[MAX_TRIANGLE_COUNT_PER_BUCKET];  //!< global ID of the triangles touching this SD

    __shared__ int sphX[MAX_COUNT_OF_DEs_PER_SD];  //!< X coordinate of the grElement
    __shared__ int sphY[MAX_COUNT_OF_DEs_PER_SD];  //!< Y coordinate of the grElement
    __shared__ int sphZ[MAX_COUNT_OF_DEs_PER_SD];  //!< Z coordinate of the grElement

    __shared__ float sphere_X_DOT[MAX_COUNT_OF_DEs_PER_SD];
    __shared__ float sphere_Y_DOT[MAX_COUNT_OF_DEs_PER_SD];
    __shared__ float sphere_Z_DOT[MAX_COUNT_OF_DEs_PER_SD];

    // TODO figure out how we can do this better with no friction
    __shared__ float3 omega[MAX_COUNT_OF_DEs_PER_SD];

    __shared__ double3 node1[MAX_TRIANGLE_COUNT_PER_BUCKET];  //!< Coordinates of the 1st node of the triangle
    __shared__ double3 node2[MAX_TRIANGLE_COUNT_PER_BUCKET];  //!< Coordinates of the 2nd node of the triangle
    __shared__ double3 node3[MAX_TRIANGLE_COUNT_PER_BUCKET];  //!< Coordinates of the 3rd node of the triangle

    // define an alias first
    unsigned int thisSD = blockIdx.x;

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
    // narrow phase will prune this triangle fast and the penalty associated with storing triangles from multiple
    // SDs into one bucket is not stiff.

    // Populate the shared memory with terrain data
    unsigned int tripsToCoverSpheres = (nSD_spheres + blockDim.x - 1) / blockDim.x;
    unsigned int local_ID = threadIdx.x;
    for (unsigned int sphereTrip = 0; sphereTrip < tripsToCoverSpheres; sphereTrip++) {
        if (local_ID < nSD_spheres) {
            unsigned int globalID = grElems_in_SD_composite[local_ID + thisSD * MAX_COUNT_OF_DEs_PER_SD];
            grElemID[local_ID] = globalID;
            sphX[local_ID] = sphere_data.pos_X[globalID];
            sphY[local_ID] = sphere_data.pos_Y[globalID];
            sphZ[local_ID] = sphere_data.pos_Z[globalID];

            sphere_X_DOT[local_ID] = sphere_data.pos_X_dt[globalID];
            sphere_Y_DOT[local_ID] = sphere_data.pos_Y_dt[globalID];
            sphere_Z_DOT[local_ID] = sphere_data.pos_Z_dt[globalID];

            // Sphere angular velocities
            if (gran_params->friction_mode != chrono::granular::GRAN_FRICTION_MODE::FRICTIONLESS) {
                omega[local_ID] =
                    make_float3(sphere_data.sphere_Omega_X[globalID], sphere_data.sphere_Omega_Y[globalID],
                                sphere_data.sphere_Omega_Z[globalID]);
            }
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
        float3 sphere_force = {0.f, 0.f, 0.f};
        float3 sphere_AngAcc = {0.f, 0.f, 0.f};
        if (sphere_Local_ID < nSD_spheres) {
            // Figure out which triangles this sphere collides with; each thread in a warp slaving for this sphere
            // looks at one triangle at a time. The collection of threads in the warp sweeps through all the
            // triangles in this BUCKET. NOTE: to avoid double-counting, a sphere-triangle collision event is
            // counted only if the collision point is in this SD.
            unsigned int targetTriangle = (threadIdx.x & (warp_size - 1));  // computes modulo 32 of the thread index
            for (unsigned int triangTrip = 0; triangTrip < tripsToCoverTriangles; triangTrip++) {
                if (targetTriangle < nBKT_triangles) {
                    /// we have a valid sphere and a valid triganle; check if in contact
                    float3 normal;  // Unit normal from pt2 to pt1 (triangle contact point to sphere contact point)
                    double3 pt1;    // Contact point on triangle
                    float depth;    // Negative in overlap

                    // Transform LRF to GRF
                    const unsigned int fam = d_triangleSoup->triangleFamily_ID[triangID[targetTriangle]];

                    // NOTE implicit cast from int to double
                    double3 sphCntr = make_double3(sphX[sphere_Local_ID], sphY[sphere_Local_ID], sphZ[sphere_Local_ID]);

                    // If there is a collision, add an impulse to the sphere
                    if (face_sphere_cd(node1[targetTriangle], node2[targetTriangle], node3[targetTriangle], sphCntr,
                                       gran_params->sphereRadius_SU, normal, depth, pt1) &&
                        SDTripletID(pointSDTriplet(pt1.x, pt1.y, pt1.z, gran_params), gran_params) == thisSD) {
                        // TODO contact models
                        // Use the CD information to compute the force on the grElement
                        const float3 delta = -depth * normal;

                        // Velocity difference, it's better to do a coalesced access here than a fragmented access
                        // inside
                        float3 v_rel = make_float3(
                            sphere_X_DOT[sphere_Local_ID] - d_triangleSoup->vel[fam].x,
                            sphere_Y_DOT[sphere_Local_ID] - d_triangleSoup->vel[fam].y,
                            sphere_Z_DOT[sphere_Local_ID] -
                                d_triangleSoup->vel[fam].z);  // TODO use shared memory if there is any left...
                        const float3 pt1_float = make_float3(pt1.x, pt1.y, pt1.z);

                        // TODO assumes pos is the center of mass of the mesh
                        // TODO can this be float?
                        float3 meshCenter = make_float3(mesh_params->fam_frame_broad[fam].pos[0],
                                                        mesh_params->fam_frame_broad[fam].pos[1],
                                                        mesh_params->fam_frame_broad[fam].pos[2]);
                        convert_pos_UU2SU<float3>(meshCenter, gran_params);

                        // NOTE depth is negative and normal points from triangle to sphere center
                        const float3 r = pt1_float + normal * (depth / 2) - meshCenter;

                        // Add angular velocity contribution from mesh
                        v_rel = v_rel + Cross(d_triangleSoup->omega[fam], r);

                        // add tangential components if they exist
                        if (gran_params->friction_mode != chrono::granular::GRAN_FRICTION_MODE::FRICTIONLESS) {
                            // Vector from the center of sphere to center of contact volume
                            const float3 r_A = -(gran_params->sphereRadius_SU + depth / 2.f) * normal;
                            v_rel = v_rel + Cross(omega[sphere_Local_ID], r_A);
                        }

                        // Forace accumulator on sphere for this sphere-triangle collision
                        // Compute force updates for normal spring term
                        float3 force_accum = mesh_params->Kn_s2m_SU * delta;

                        // Compute force updates for damping term
                        // NOTE assumes sphere mass of 1
                        const float fam_mass_SU = d_triangleSoup->familyMass_SU[fam];
                        const float m_eff = fam_mass_SU / (1.f + fam_mass_SU);
                        const float3 vrel_n = Dot(v_rel, normal) * normal;
                        v_rel = v_rel - vrel_n;  // v_rel is now tangential relative velocity

                        // Add normal damping term
                        force_accum = force_accum + -mesh_params->Gamma_n_s2m_SU * m_eff * vrel_n;

                        // TODO apply model multiplier here

                        if (gran_params->friction_mode != chrono::granular::GRAN_FRICTION_MODE::FRICTIONLESS) {
                            if (gran_params->friction_mode == chrono::granular::GRAN_FRICTION_MODE::SINGLE_STEP) {
                                // both tangential terms combine for single step
                                const float combined_tangent_coeff = mesh_params->Kt_s2m_SU * gran_params->alpha_h_bar +
                                                                     mesh_params->Gamma_t_s2m_SU * m_eff;

                                // we dotted out normal component of v, so v_rel is the tangential component
                                // TODO apply model multiplier
                                const float3 tangent_force = -combined_tangent_coeff * v_rel;

                                // Vector from sphere center to center of contact volume
                                // NOTE depth is negative
                                const float3 r = -(gran_params->sphereRadius_SU + depth / 2.f) * normal;

                                // TODO could do scaling to eliminate a factor of radius?
                                sphere_AngAcc = sphere_AngAcc +
                                                Cross(r, tangent_force) /
                                                    (gran_params->sphereInertia_by_r * gran_params->sphereRadius_SU);

                                // add to total forces
                                force_accum = force_accum + tangent_force;
                            }
                        }

                        // Compute force updates for adhesion term, opposite the spring term
                        // NOTE ratio is wrt the weight of a sphere of mass 1
                        const float adhesionConstant = gran_params->gravMag_SU * mesh_params->adhesion_ratio_s2m;

                        // Add adhesion term
                        // NOTE the cancelation of two negatives
                        force_accum = force_accum + adhesionConstant / depth * delta;

                        // Use the CD information to compute the force and torque on the family of this triangle
                        sphere_force = sphere_force + force_accum;

                        // Force on the mesh is opposite the force on the sphere
                        const float3 force_total = -1.f * force_accum;

                        // point from center of body to contact point
                        float3 fromCenter;
                        {
                            double3 meshCenter_double = make_double3(mesh_params->fam_frame_narrow[fam].pos[0],
                                                                     mesh_params->fam_frame_narrow[fam].pos[1],
                                                                     mesh_params->fam_frame_narrow[fam].pos[2]);
                            convert_pos_UU2SU<double3>(meshCenter_double, gran_params);

                            const double3 fromCenter_double = pt1 - meshCenter_double;
                            fromCenter = make_float3(fromCenter_double.x, fromCenter_double.y, fromCenter_double.z);
                        }
                        const float3 torque = Cross(fromCenter, force_total);

                        const unsigned int fam = d_triangleSoup->triangleFamily_ID[triangID[targetTriangle]];
                        atomicAdd(d_triangleSoup->generalizedForcesPerFamily + fam * 6 + 0, force_total.x);
                        atomicAdd(d_triangleSoup->generalizedForcesPerFamily + fam * 6 + 1, force_total.y);
                        atomicAdd(d_triangleSoup->generalizedForcesPerFamily + fam * 6 + 2, force_total.z);

                        atomicAdd(d_triangleSoup->generalizedForcesPerFamily + fam * 6 + 3, torque.x);
                        atomicAdd(d_triangleSoup->generalizedForcesPerFamily + fam * 6 + 4, torque.y);
                        atomicAdd(d_triangleSoup->generalizedForcesPerFamily + fam * 6 + 5, torque.z);
                    }
                }
                targetTriangle += warp_size;
            }  // end of per-triangle loop

            // down to the point where we need to collect the forces from all the threads in the warp; this is a
            // warp reduce operation. The resultant force acting on this grElement is stored in the first lane of
            // the warp. NOTE: In this warp-level operations participate only the warps that are slaving for a
            // sphere; i.e., some warps see no action
            for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2) {
                sphere_force.x += __shfl_down_sync(0xffffffff, sphere_force.x, offset);
                sphere_force.y += __shfl_down_sync(0xffffffff, sphere_force.y, offset);
                sphere_force.z += __shfl_down_sync(0xffffffff, sphere_force.z, offset);

                sphere_AngAcc.x += __shfl_down_sync(0xffffffff, sphere_AngAcc.x, offset);
                sphere_AngAcc.y += __shfl_down_sync(0xffffffff, sphere_AngAcc.y, offset);
                sphere_AngAcc.z += __shfl_down_sync(0xffffffff, sphere_AngAcc.z, offset);
            }

            // done with the computation of all the contacts that the triangles impress on this sphere. Update the
            // position of the sphere based on this force

            // Write velocity update for this sphere back to global memory from each lane 0
            if ((threadIdx.x & (warp_size - 1)) == 0) {
                atomicAdd(sphere_data.sphere_force_X + grElemID[sphere_Local_ID], sphere_force.x);
                atomicAdd(sphere_data.sphere_force_Y + grElemID[sphere_Local_ID], sphere_force.y);
                atomicAdd(sphere_data.sphere_force_Z + grElemID[sphere_Local_ID], sphere_force.z);

                if (gran_params->friction_mode != chrono::granular::GRAN_FRICTION_MODE::FRICTIONLESS) {
                    // write back torques for later
                    atomicAdd(sphere_data.sphere_ang_acc_X + grElemID[sphere_Local_ID], sphere_AngAcc.x);
                    atomicAdd(sphere_data.sphere_ang_acc_Y + grElemID[sphere_Local_ID], sphere_AngAcc.y);
                    atomicAdd(sphere_data.sphere_ang_acc_Z + grElemID[sphere_Local_ID], sphere_AngAcc.z);
                }
            }
        }                                               // end of valid sphere if
        sphere_Local_ID += nSpheresProcessedAtOneTime;  // go to next set of spheress
    }                                                   // end of per-sphere loop
}  // namespace granular

/// Copy const triangle data to device
void ChSystemGranular_MonodisperseSMC_trimesh::copy_triangle_data_to_device() {
    // unified memory does some copying for us, cool
    tri_params->Kn_s2m_SU = K_n_s2m_SU;
    tri_params->Kt_s2m_SU = K_t_s2m_SU;
    tri_params->Gamma_n_s2m_SU = Gamma_n_s2m_SU;
    tri_params->Gamma_t_s2m_SU = Gamma_t_s2m_SU;
    tri_params->adhesion_ratio_s2m = adhesion_s2m_over_gravity;

    SD_isTouchingTriangle.resize(nSDs);
}

__host__ double ChSystemGranular_MonodisperseSMC_trimesh::advance_simulation(float duration) {
    sphereDataStruct sphere_data;

    packSphereDataPointers(sphere_data);

    // Figure our the number of blocks that need to be launched to cover the box
    unsigned int nBlocks = (nDEs + CUDA_THREADS_PER_BLOCK - 1) / CUDA_THREADS_PER_BLOCK;

    // Settling simulation loop.
    float duration_SU = std::ceil(duration / gran_params->TIME_UNIT);
    determineNewStepSize_SU();  // doesn't always change the timestep
    gran_params->alpha_h_bar = stepSize_SU;
    unsigned int nsteps = duration_SU / stepSize_SU;

    VERBOSE_PRINTF("advancing by %f at timestep %f, %u timesteps at approx user timestep %f\n", duration_SU,
                   stepSize_SU, nsteps, duration / nsteps);

    VERBOSE_PRINTF("Starting Main Simulation loop!\n");

    float time_elapsed_SU = 0;  // time elapsed in this call (SU)
    // Run the simulation, there are aggressive synchronizations because we want to have no race conditions
    for (; time_elapsed_SU < stepSize_SU * nsteps; time_elapsed_SU += stepSize_SU) {
        determineNewStepSize_SU();  // doesn't always change the timestep
        gran_params->alpha_h_bar = stepSize_SU;

        // Update the position and velocity of the BD, if relevant
        if (!BD_is_fixed) {
            updateBDPosition(stepSize_SU);
        }
        resetSphereForces();
        resetTriangleForces();
        resetTriangleBroadphaseInformation();

        VERBOSE_PRINTF("Starting computeSphereForces!\n");

        // Compute sphere-sphere forces
        computeSphereForces<MAX_COUNT_OF_DEs_PER_SD><<<nSDs, MAX_COUNT_OF_DEs_PER_SD>>>(
            sphere_data, gran_params, BC_type_list.data(), BC_params_list_SU.data(), BC_params_list_SU.size());

        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        if (meshSoup_DEVICE->nTrianglesInSoup != 0 && mesh_collision_enabled) {
            const int nthreads = 64;
            // broadphase the triangles
            // TODO check these block/thread counts
            triangleSoupBroadPhase<nthreads>
                <<<(meshSoup_DEVICE->nTrianglesInSoup + nthreads - 1) / nthreads, nthreads>>>(
                    meshSoup_DEVICE, BUCKET_countsOfTrianglesTouching.data(), triangles_in_BUCKET_composite.data(),
                    SD_isTouchingTriangle.data(), gran_params, tri_params);
        }
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        if (meshSoup_DEVICE->nFamiliesInSoup != 0 && mesh_collision_enabled) {
            // TODO please do not use a template here
            // compute sphere-triangle forces
            interactionTerrain_TriangleSoup<CUDA_THREADS_PER_BLOCK><<<nSDs, MAX_COUNT_OF_DEs_PER_SD>>>(
                meshSoup_DEVICE, sphere_data, BUCKET_countsOfTrianglesTouching.data(),
                triangles_in_BUCKET_composite.data(), SD_NumOf_DEs_Touching.data(), DEs_in_SD_composite.data(),
                SD_isTouchingTriangle.data(), gran_params, tri_params);
        }
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        VERBOSE_PRINTF("Resetting broadphase info!\n");

        resetBroadphaseInformation();

        VERBOSE_PRINTF("Starting updatePositions!\n");
        updatePositions<CUDA_THREADS_PER_BLOCK>
            <<<nBlocks, CUDA_THREADS_PER_BLOCK>>>(stepSize_SU, sphere_data, nDEs, gran_params);

        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());
        elapsedSimTime += stepSize_SU * gran_params->TIME_UNIT;  // Advance current time
    }
    return time_elapsed_SU * gran_params->TIME_UNIT;  // return elapsed UU time
}
}  // namespace granular
}  // namespace chrono
