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
#include "chrono_granular/physics/ChGranularGPU_SMC.cuh"
#include "chrono_granular/physics/ChGranularTriMesh.h"
// these define things that mess with cub
#include "chrono_granular/physics/ChGranularCollision.cuh"
#include "chrono_granular/physics/ChGranularBoxTriangle.cuh"
#include "chrono_granular/utils/ChCudaMathUtils.cuh"

// TODO should this go here?
// NOTE warpSize is a cuda environment value, but it is cc-dependent
#if __CUDA_ARCH__ <= 600
// all devices of compute capability <= 6.0
static const int warp_size = 32;
#else
static const int warp_size = warpSize;
#endif

#define NUM_TRIANGLE_FAMILIES 4

#define Triangle_Soup chrono::granular::ChTriangleSoup

typedef const chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::GranParamsHolder_trimesh*
    MeshParamsPtr;

/// Takes in a triangle's position and finds out what SDs it touches
__device__ void triangle_figureOutTouchedSDs(unsigned int triangleID,
                                             const Triangle_Soup<float>* triangleSoup,
                                             unsigned int* touchedSDs,
                                             ParamsPtr gran_params) {
    unsigned int SD_count = 0;
    float3 vA, vB, vC;
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
        touchedSDs[SD_count++] = SDTripletID(L, gran_params);
        return;
    }

    unsigned int n_axes_diff = 0;  // Count axes that have different SD bounds
    unsigned int axes_diff;

    for (unsigned int i = 0; i < 3; i++) {
        if (L[i] != U[i]) {
            axes_diff = i;  // If there is more than one, it won't be used anyway
            n_axes_diff++;
        }
    }

    // Case 2: Triangle lies in a Nx1x1, 1xNx1, or 1x1xN block of SDs
    if (n_axes_diff == 1) {
        unsigned int SD_i[3] = {L[0], L[1], L[2]};
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
                SDhalfSizes[0] = gran_params->d_SD_Ldim_SU;
                SDhalfSizes[1] = gran_params->d_SD_Ddim_SU;
                SDhalfSizes[2] = gran_params->d_SD_Hdim_SU;

                SDcenter[0] = gran_params->d_BD_frame_X + (i * 2 + 1) * SDhalfSizes[0];
                SDcenter[1] = gran_params->d_BD_frame_Y + (j * 2 + 1) * SDhalfSizes[1];
                SDcenter[2] = gran_params->d_BD_frame_Z + (k * 2 + 1) * SDhalfSizes[2];

                if (check_TriangleBoxOverlap(SDcenter, SDhalfSizes, vA, vB, vC)) {
                    touchedSDs[SD_count++] = SDTripletID(i, j, k, gran_params);
                }
            }
        }
    }
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
 * SD_countsOfTrianglesTouching - array that for each SD indicates how many triangles touch this SD
 * triangles_in_SD_composite - big array that works in conjunction with SD_countsOfTrianglesTouching.
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
    Triangle_Soup<float>* d_triangleSoup,
    unsigned int*
        BUCKET_countsOfTrianglesTouching,  //!< Array that for each SD indicates how many triangles touch this SD
    unsigned int*
        triangles_in_BUCKET_composite,  //!< Big array that works in conjunction with SD_countsOfTrianglesTouching.
                                        //!< "triangles_in_SD_composite" says which SD contains what triangles.
    unsigned int* SD_countsOfTrianglesTouching,  //!< If SD 629 has any triangle touching it, then
                                                 //!< SD_countsOfTrianglesTouching[629]>0.
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
        triangle_figureOutTouchedSDs(myTriangleID, d_triangleSoup, SDsTouched, gran_params);
    }

    __syncthreads();

    // Truth be told, we are not interested in SDs touched, but rather buckets touched. This next step associates SDs
    // with "buckets". To save memory, since most SDs have no triangles, we "randomly" associate several SDs with a
    // bucket. While the assignment of SDs to buckets is "random," the assignment scheme is deterministic: for
    // instance, SD 239 would always go to bucket 71.
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_TRIANGLE; i++)
        if (SDsTouched[i] != NULL_GRANULAR_ID) {
            BKTsTouched[i] = hashmapBKTid(SDsTouched[i]) % TRIANGLEBUCKET_COUNT;
        }

    // Earmark SDs that are touched by at least one triangle. This step is needed since when computing the
    // mesh-GrMat interaction we only want to do narrow phase on an SD that actually is touched by triangles. Keep
    // in mind thta several SDs deposit their triangles in the same bucket. As such, later one during narrow phase/force
    // computation, if an SD looks for a bucket and sees triangles in there, if we know that this SD is touching zero
    // triangles then that SD is not going to do narrow phase on the triangles in that bucket since these triangles
    // actually are associated with other SDs that happen to deposit their triangles in this same bucket.
    // NOTE why are we sorting this? also we can't use this storage like that, it's for a key-value sort
    BlockRadixSortOP(temp_storage_sort).Sort(SDsTouched, triangleIDs);
    __syncthreads();

    // Do a winningStreak search on whole block, might not have high utilization here
    bool head_flags[MAX_SDs_TOUCHED_BY_TRIANGLE];
    Block_Discontinuity(temp_storage_disc).FlagHeads(head_flags, SDsTouched, cub::Inequality());
    __syncthreads();

    // If a thread is associated with a legit discontinuity; i.e., not one associated with NULL_GRANULAR_ID, it should
    // flag that SD as being touched by a triangle
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_TRIANGLE; i++) {
        if (head_flags[i] && (SDsTouched[i] != NULL_GRANULAR_ID))
            atomicAdd(SD_countsOfTrianglesTouching, 1);
    }

    // Back at working with buckets. For all purposes, the role that SDs play in this kernel is over.
    BlockRadixSortOP(temp_storage_sort).Sort(BKTsTouched, triangleIDs);
    __syncthreads();

    // Do a winningStreak search on whole block, might not have high utilization here
    Block_Discontinuity(temp_storage_disc).FlagHeads(head_flags, BKTsTouched, cub::Inequality());
    __syncthreads();

    // Write back to shared memory; eight-way bank conflicts here - to revisit later
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_TRIANGLE; i++) {
        shMem_head_flags[MAX_SDs_TOUCHED_BY_TRIANGLE * threadIdx.x + i] = head_flags[i];
    }

    // Seed offsetInComposite_TriangleInSD_Array with "no valid ID" so that we know later on what is legit;
    // No shmem bank coflicts here, good access...
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_TRIANGLE; i++) {
        offsetInComposite_TriangleInBKT_Array[i * CUB_THREADS + threadIdx.x] = NULL_GRANULAR_ID_LONG;
    }

    __syncthreads();

    // Count how many times a Bucket shows up in conjunction with the collection of CUB_THREADS triangles. There
    // will be some thread divergence here.
    // Loop through each potential BKT, after sorting, and see if it is the start of a head
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_TRIANGLE; i++) {
        // SD currently touched, could easily be inlined
        unsigned int touchedBucket = BKTsTouched[i];
        if (head_flags[i] && (touchedBucket != NULL_GRANULAR_ID)) {
            // current index into shared datastructure of length 8*CUB_THREADS, could easily be inlined
            unsigned int idInShared = MAX_SDs_TOUCHED_BY_TRIANGLE * threadIdx.x + i;
            unsigned int winningStreak = 0;
            // This is the beginning of a sequence of BKTs with a new ID
            do {
                winningStreak++;
                // Go until we run out of threads on the warp or until we find a new head
            } while (idInShared + winningStreak < MAX_SDs_TOUCHED_BY_TRIANGLE * CUB_THREADS &&
                     !(shMem_head_flags[idInShared + winningStreak]));

            // if (touchedSD >= d_box_L_SU * d_box_D_SU * d_box_H_SU) {
            //     printf("invalid SD index %u on thread %u\n", mySphereID, touchedSD);
            // }

            // Store start of new entries
            unsigned int offset = atomicAdd(BUCKET_countsOfTrianglesTouching + touchedBucket, winningStreak);

            // The value offset now gives a *relative* offset in the composite array.
            // Get the absolute offset
            offset += touchedBucket * MAX_TRIANGLE_COUNT_PER_BUCKET;

            // Produce the offsets for this streak of triangles with identical BKT ids
            for (unsigned int i = 0; i < winningStreak; i++)
                offsetInComposite_TriangleInBKT_Array[idInShared + i] = offset++;
        }
    }

    __syncthreads();  // needed since we write to shared memory above; i.e., offsetInComposite_SphInSD_Array

    // Write out the data now; register with triangles_in_SD_composite each sphere that touches a certain ID
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_TRIANGLE; i++) {
        unsigned int offset = offsetInComposite_TriangleInBKT_Array[MAX_SDs_TOUCHED_BY_TRIANGLE * threadIdx.x + i];
        if (offset != NULL_GRANULAR_ID_LONG) {
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
    Triangle_Soup<float>* d_triangleSoup,  //!< Contains information pertaining to triangle soup (in device mem.)
    int* d_sphere_pos_X,
    int* d_sphere_pos_Y,
    int* d_sphere_pos_Z,
    float* d_sphere_pos_X_dt,
    float* d_sphere_pos_Y_dt,
    float* d_sphere_pos_Z_dt,
    unsigned int* BKT_countsOfTrianglesTouching,  //!< Array that for each SD indicates how many triangles touch this SD
    unsigned int*
        triangles_in_BKT_composite,  //!< Big array that works in conjunction with SD_countsOfTrianglesTouching.
                                     //!< "triangles_in_SD_composite" says which SD contains what triangles.
    unsigned int*
        SD_countsOfGrElemsTouching,         //!< Array that for each SD indicates how many grain elements touch this SD
    unsigned int* grElems_in_SD_composite,  //!< Big array that works in conjunction with SD_countsOfGrElemsTouching.
                                            //!< "grElems_in_SD_composite" says which SD contains what grElements.
    unsigned int* SD_countsOfTrianglesTouching,  //!< The length of this array is equal to number of SDs. If SD 423 is
                                                 //!< touched by any triangle, then SD_countsOfTrianglesTouching[423]>0.

    ParamsPtr gran_params,
    MeshParamsPtr mesh_params) {
    __shared__ unsigned int grElemID[MAX_COUNT_OF_DEs_PER_SD];        //!< global ID of the grElements touching this SD
    __shared__ unsigned int triangID[MAX_TRIANGLE_COUNT_PER_BUCKET];  //!< global ID of the triangles touching this SD

    __shared__ int sphX[MAX_COUNT_OF_DEs_PER_SD];  //!< X coordinate of the grElement
    __shared__ int sphY[MAX_COUNT_OF_DEs_PER_SD];  //!< Y coordinate of the grElement
    __shared__ int sphZ[MAX_COUNT_OF_DEs_PER_SD];  //!< Z coordinate of the grElement

    __shared__ int node1_X[MAX_TRIANGLE_COUNT_PER_BUCKET];  //!< X coordinate of the 1st node of the triangle
    __shared__ int node1_Y[MAX_TRIANGLE_COUNT_PER_BUCKET];  //!< Y coordinate of the 1st node of the triangle
    __shared__ int node1_Z[MAX_TRIANGLE_COUNT_PER_BUCKET];  //!< Z coordinate of the 1st node of the triangle

    __shared__ int node2_X[MAX_TRIANGLE_COUNT_PER_BUCKET];  //!< X coordinate of the 2nd node of the triangle
    __shared__ int node2_Y[MAX_TRIANGLE_COUNT_PER_BUCKET];  //!< Y coordinate of the 2nd node of the triangle
    __shared__ int node2_Z[MAX_TRIANGLE_COUNT_PER_BUCKET];  //!< Z coordinate of the 2nd node of the triangle

    __shared__ int node3_X[MAX_TRIANGLE_COUNT_PER_BUCKET];  //!< X coordinate of the 3rd node of the triangle
    __shared__ int node3_Y[MAX_TRIANGLE_COUNT_PER_BUCKET];  //!< Y coordinate of the 3rd node of the triangle
    __shared__ int node3_Z[MAX_TRIANGLE_COUNT_PER_BUCKET];  //!< Z coordinate of the 3rd node of the triangle

    volatile __shared__ float tempShMem[6 * (N_CUDATHREADS / warp_size)];  // used to do a block-level reduce

    float forceActingOnSphere[3];  //!< 3 registers will hold the value of the force on the sphere
    float genForceActingOnMeshes[TRIANGLE_FAMILIES * 6];  //!< 6 components per family: 3 forces and 3 torques

    unsigned int thisSD = blockIdx.x;
    unsigned int nSD_triangles = SD_countsOfTrianglesTouching[thisSD];
    unsigned int nSD_spheres = SD_countsOfGrElemsTouching[thisSD];

    if (nSD_triangles == 0 || nSD_spheres == 0)
        return;

    // Getting here means that there are both triangles and DEs in this SD.
    // First, figure out which bucket stores the triangles associated with this SD.
    unsigned int whichBKT = hashmapBKTid(thisSD) % TRIANGLEBUCKET_COUNT;
    unsigned int nBKT_triangles = BKT_countsOfTrianglesTouching[thisSD];

    // Unpleasant fact: this bucket might store more than the triangles associated with this SD. The narrow phase is
    // done for ALL triangles in this bucket with the expectation that if a triangle does not belong to this SD, the
    // narrow phase will prune this triangle fast and the penalty associated with storing triangles from multiple SDs
    // into one bucket is not stiff.

    // Populate the shared memory with terrain data
    unsigned int tripsToCoverSpheres = (nSD_spheres + blockDim.x - 1) / blockDim.x;
    unsigned int local_ID = threadIdx.x;
    for (unsigned int sphereTrip = 0; sphereTrip < tripsToCoverSpheres; sphereTrip++) {
        local_ID += sphereTrip * blockDim.x;
        if (local_ID < nSD_spheres) {
            unsigned int globalID = grElems_in_SD_composite[local_ID + thisSD * MAX_COUNT_OF_DEs_PER_SD];
            grElemID[local_ID] = globalID;
            sphX[local_ID] = d_sphere_pos_X[globalID];
            sphY[local_ID] = d_sphere_pos_Y[globalID];
            sphZ[local_ID] = d_sphere_pos_Z[globalID];
        }
    }
    // Populate the shared memory with mesh triangle data
    unsigned int tripsToCoverTriangles = (nBKT_triangles + blockDim.x - 1) / blockDim.x;
    local_ID = threadIdx.x;
    for (unsigned int triangTrip = 0; triangTrip < tripsToCoverTriangles; triangTrip++) {
        local_ID += triangTrip * blockDim.x;
        if (local_ID < nBKT_triangles) {
            unsigned int globalID = triangles_in_BKT_composite[local_ID + whichBKT * MAX_TRIANGLE_COUNT_PER_BUCKET];
            triangID[local_ID] = globalID;
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
    }

    __syncthreads();  // this call ensures data is in its place in shared memory

    /// Zero out the force and torque at the onset of the computation
    for (local_ID = 0; local_ID < TRIANGLE_FAMILIES; local_ID++) {
        unsigned int dummyOffset = 6 * local_ID;
        /// forces acting on the triangle, in global reference frame
        genForceActingOnMeshes[dummyOffset++] = 0.f;
        genForceActingOnMeshes[dummyOffset++] = 0.f;
        genForceActingOnMeshes[dummyOffset++] = 0.f;
        /// torques with respect to global reference frame, expressed in global reference frame
        genForceActingOnMeshes[dummyOffset++] = 0.f;
        genForceActingOnMeshes[dummyOffset++] = 0.f;
        genForceActingOnMeshes[dummyOffset++] = 0.f;
    }

    // Each sphere has one warp of threads dedicated to identifying all triangles that this sphere
    // touches. Upon a contact event, we'll compute the normal force on the sphere; and, the force and torque
    // impressed upon the triangle

    unsigned int nSpheresProcessedAtOneTime =
        blockDim.x / warp_size;  /// One warp allocated to slave serving one sphere
    tripsToCoverSpheres = (nSD_spheres + nSpheresProcessedAtOneTime - 1) / nSpheresProcessedAtOneTime;
    tripsToCoverTriangles = (nBKT_triangles + warp_size - 1) / warp_size;

    unsigned sphere_Local_ID = threadIdx.x / warp_size;
    for (unsigned int sphereTrip = 0; sphereTrip < tripsToCoverSpheres; sphereTrip++) {
        /// before starting dealing with a sphere, zero out the forces acting on it; all threads in the block are
        /// doing this
        forceActingOnSphere[0] = 0.f;
        forceActingOnSphere[1] = 0.f;
        forceActingOnSphere[2] = 0.f;
        sphere_Local_ID += sphereTrip * nSpheresProcessedAtOneTime;
        if (sphere_Local_ID < nSD_spheres) {
            /// Figure out which triangles this sphere collides with; each thread in a warp slaving for this sphere
            /// looks at one triangle at a time. The collection of threads in the warp sweeps through all the
            /// triangles that touch this SD. NOTE: to avoid double-counting, a sphere-triangle collision event is
            /// counted only if the collision point is in this SD.
            unsigned int targetTriangle = (threadIdx.x & (warp_size - 1));  // computes modulo 32 of the thread index
            for (unsigned int triangTrip = 0; triangTrip < tripsToCoverTriangles; triangTrip++) {
                targetTriangle += triangTrip * warp_size;
                if (targetTriangle < nBKT_triangles) {
                    /// we have a valid sphere and a valid triganle; check if in contact
                    double3 norm;
                    double depth;
                    double3 pt1;
                    double3 pt2;
                    double eff_radius;
                    double3 A = make_double3(node1_X[targetTriangle], node1_Y[targetTriangle], node1_Z[targetTriangle]);
                    double3 B = make_double3(node2_X[targetTriangle], node2_Y[targetTriangle], node2_Z[targetTriangle]);
                    double3 C = make_double3(node3_X[targetTriangle], node3_Y[targetTriangle], node3_Z[targetTriangle]);
                    double3 sphCntr = make_double3(sphX[sphere_Local_ID], sphY[sphere_Local_ID], sphZ[sphere_Local_ID]);
                    face_sphere_cd(A, B, C, sphCntr, gran_params->d_sphereRadius_SU, norm, depth, pt1, pt2, eff_radius);

                    /// Use the CD information to compute the force on the grElement

                    /// Use the CD information to compute the force and torque on the triangle
                }
            }
            /// down to the point where we need to collect the forces from all the threads in the wrap; this is a
            /// warp reduce operation. The resultant force acting on this grElement is stored in the first lane of
            /// the warp. NOTE: In this warp-level operations participate only the warps that are slaving for a
            /// sphere; i.e., some warps see no action
            for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2)
                forceActingOnSphere[0] += __shfl_down_sync(0xffffffff, forceActingOnSphere[0], offset);
            for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2)
                forceActingOnSphere[1] += __shfl_down_sync(0xffffffff, forceActingOnSphere[1], offset);
            for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2)
                forceActingOnSphere[2] += __shfl_down_sync(0xffffffff, forceActingOnSphere[2], offset);

            /// done with the computation of all the contacts that the triangles impress on this sphere. Update the
            /// position of the sphere based on this force
        }
    }
    /// Done computing the forces acting on the triangles in this SD. A block reduce is carried out next. Start by
    /// doing a reduce at the warp level.
    for (local_ID = 0; local_ID < TRIANGLE_FAMILIES; local_ID++) {
        /// six generalized forces acting on the triangle, expressed in the global reference frame
        unsigned int dummyIndx = 6 * local_ID;
        for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2)
            genForceActingOnMeshes[dummyIndx] +=
                __shfl_down_sync(0xffffffff, genForceActingOnMeshes[dummyIndx], offset);
        dummyIndx++;

        for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2)
            genForceActingOnMeshes[dummyIndx] +=
                __shfl_down_sync(0xffffffff, genForceActingOnMeshes[dummyIndx], offset);
        dummyIndx++;

        for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2)
            genForceActingOnMeshes[dummyIndx] +=
                __shfl_down_sync(0xffffffff, genForceActingOnMeshes[dummyIndx], offset);
        dummyIndx++;

        for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2)
            genForceActingOnMeshes[dummyIndx] +=
                __shfl_down_sync(0xffffffff, genForceActingOnMeshes[dummyIndx], offset);
        dummyIndx++;

        for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2)
            genForceActingOnMeshes[dummyIndx] +=
                __shfl_down_sync(0xffffffff, genForceActingOnMeshes[dummyIndx], offset);
        dummyIndx++;

        for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2)
            genForceActingOnMeshes[dummyIndx] +=
                __shfl_down_sync(0xffffffff, genForceActingOnMeshes[dummyIndx], offset);
    }

    __syncthreads();

    /// Lane zero in each warp holds the result of a warp-level reduce operation. Sum up these "Lane zero" values in
    /// the final result, which is block-level
    bool threadIsLaneZeroInWarp = ((threadIdx.x & (warp_size - 1)) == 0);
    for (local_ID = 0; local_ID < TRIANGLE_FAMILIES; local_ID++) {
        unsigned int offsetGenForceArray = 6 * local_ID;
        /// Place in ShMem forces/torques (expressed in global reference frame) acting on this family of triangles
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

        /// Going to trash the values in "forceActingOnSphere", not needed anymore. Reuse the registers, which will
        /// now store the vaule of the triangle force and torque...
        if (threadIdx.x < warp_size) {
            /// only first thread in block participates in this reduce operation.
            /// NOTE: an implicit assumption is made here - warp_size is larger than or equal to N_CUDATHREADS /
            /// warp_size. This is true today as N_CUDATHREADS cannot be larger than 1024 and warp_size is 32.

            /// Work on forces first. Place data from ShMem into registers associated w/ first warp
            unsigned int offsetShMem = 6 * threadIdx.x;
            if (threadIdx.x < (N_CUDATHREADS / warp_size)) {
                forceActingOnSphere[0] = tempShMem[offsetShMem++];
                forceActingOnSphere[1] = tempShMem[offsetShMem++];
                forceActingOnSphere[2] = tempShMem[offsetShMem++];
            } else {
                /// this is hit only by a subset of threads from first warp of the block
                forceActingOnSphere[0] = 0.f;
                forceActingOnSphere[1] = 0.f;
                forceActingOnSphere[2] = 0.f;
            }

            offsetGenForceArray = 6 * local_ID;
            // X component of the force on mesh "local_ID"
            for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2)
                forceActingOnSphere[0] += __shfl_down_sync(0xffffffff, forceActingOnSphere[0], offset);
            genForceActingOnMeshes[offsetGenForceArray++] = forceActingOnSphere[0];

            // Y component of the force on mesh "local_ID"
            for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2)
                forceActingOnSphere[1] += __shfl_down_sync(0xffffffff, forceActingOnSphere[1], offset);
            genForceActingOnMeshes[offsetGenForceArray++] = forceActingOnSphere[1];

            // Z component of the force on mesh "local_ID"
            for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2)
                forceActingOnSphere[2] += __shfl_down_sync(0xffffffff, forceActingOnSphere[2], offset);
            genForceActingOnMeshes[offsetGenForceArray++] = forceActingOnSphere[2];

            /// Finally, work on torques
            if (threadIdx.x < (N_CUDATHREADS / warp_size)) {
                forceActingOnSphere[0] = tempShMem[offsetShMem++];
                forceActingOnSphere[1] = tempShMem[offsetShMem++];
                forceActingOnSphere[2] = tempShMem[offsetShMem];
            } else {
                /// this is hit only by a subset of threads from first warp of the block
                forceActingOnSphere[0] = 0.f;
                forceActingOnSphere[1] = 0.f;
                forceActingOnSphere[2] = 0.f;
            }

            // X component of the torque on mesh "local_ID"
            for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2)
                forceActingOnSphere[0] += __shfl_down_sync(0xffffffff, forceActingOnSphere[0], offset);
            genForceActingOnMeshes[offsetGenForceArray++] = forceActingOnSphere[0];

            // Y component of the torque on mesh "local_ID"
            for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2)
                forceActingOnSphere[1] += __shfl_down_sync(0xffffffff, forceActingOnSphere[1], offset);
            genForceActingOnMeshes[offsetGenForceArray++] = forceActingOnSphere[1];

            // Z component of the torque on mesh "local_ID"
            for (unsigned int offset = warp_size / 2; offset > 0; offset /= 2)
                forceActingOnSphere[2] += __shfl_down_sync(0xffffffff, forceActingOnSphere[2], offset);
            genForceActingOnMeshes[offsetGenForceArray] = forceActingOnSphere[2];
        }  /// this is the end of the "for each mesh" loop

        /// At this point, the first thread of the block has in genForceActingOnMeshes[6*TRIANGLE_FAMILIES] the
        /// forces and torques acting on each mesh family. Bcast the force values to all threads in the warp.
        /// To this end, synchronize all threads in warp and get "value" from lane 0
        for (local_ID = 0; local_ID < 6 * TRIANGLE_FAMILIES; local_ID++)
            genForceActingOnMeshes[local_ID] = __shfl_sync(0xffffffff, genForceActingOnMeshes[local_ID], 0);

        /// At this point, all threads in the first warp have the generalized forces acting on all meshes. Do an
        /// atomic add to compund the value of the generalized forces acting on the meshes that come in contact with
        /// the granular material.
        unsigned int nTrips = (6 * TRIANGLE_FAMILIES) / warp_size;
        for (local_ID = 0; local_ID < nTrips + 1; local_ID++) {
            unsigned int offset = threadIdx.x + local_ID * (6 * TRIANGLE_FAMILIES);
            if (offset < 6 * TRIANGLE_FAMILIES)
                atomicAdd(d_triangleSoup->generalizedForcesPerFamily + offset, genForceActingOnMeshes[offset]);
        }
    }
}

/// Copy const triangle data to device
void chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::copy_triangle_data_to_device() {
    // unified memory does some copying for us, cool
    tri_params->d_Gamma_n_s2m_SU = 0;  // no damping on mesh for now
    tri_params->d_Kn_s2m_SU = 7;       // TODO Nic you get to deal with this
    // tri_params->num_triangle_families = 4;  // TODO make this legit
    // Or Conlain can deal with it later, no way this actually runs cleanly anyways
}

__host__ void chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::initialize() {
    switch_to_SimUnits();
    generate_DEs();

    // Set aside memory for holding data structures worked with. Get some initializations going
    setup_simulation();
    copy_const_data_to_device();
    copy_triangle_data_to_device();
    copyBD_Frame_to_device();
    gpuErrchk(cudaDeviceSynchronize());

    // Seed arrays that are populated by the kernel call
    resetBroadphaseInformation();

    // Figure our the number of blocks that need to be launched to cover the box
    unsigned int nBlocks = (nDEs + CUDA_THREADS - 1) / CUDA_THREADS;
    printf("doing priming!\n");
    printf("max possible composite offset is %zu\n", (size_t)nSDs * MAX_COUNT_OF_DEs_PER_SD);

    primingOperationsRectangularBox<CUDA_THREADS>
        <<<nBlocks, CUDA_THREADS>>>(pos_X.data(), pos_Y.data(), pos_Z.data(), SD_NumOf_DEs_Touching.data(),
                                    DEs_in_SD_composite.data(), nDEs, gran_params);
    gpuErrchk(cudaDeviceSynchronize());
    printf("priming finished!\n");

    VERBOSE_PRINTF("z grav term with timestep %u is %f\n", stepSize_SU, stepSize_SU * stepSize_SU * gravity_Z_SU);
}

__host__ void chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::advance_simulation(
    float duration) {
    // Figure our the number of blocks that need to be launched to cover the box
    unsigned int nBlocks = (nDEs + CUDA_THREADS - 1) / CUDA_THREADS;

    // Settling simulation loop.
    unsigned int duration_SU = std::ceil(duration / (TIME_UNIT * PSI_h));
    unsigned int nsteps = (1.0 * duration_SU) / stepSize_SU;

    printf("advancing by %u at timestep %u, %u timesteps at approx user timestep %f\n", duration_SU, stepSize_SU,
           nsteps, duration / nsteps);

    VERBOSE_PRINTF("Starting Main Simulation loop!\n");
    // Run the simulation, there are aggressive synchronizations because we want to have no race conditions
    for (unsigned int crntTime_SU = 0; crntTime_SU < stepSize_SU * nsteps; crntTime_SU += stepSize_SU) {
        // Update the position and velocity of the BD, if relevant
        if (!BD_is_fixed) {
            updateBDPosition(stepSize_SU);
        }
        resetUpdateInformation();
        update_DMeshSoup_Location();  // TODO where does this go?

        VERBOSE_PRINTF("Starting computeVelocityUpdates!\n");

        // Compute forces and crank into vel updates, we have 2 kernels to avoid a race condition
        computeVelocityUpdates<MAX_COUNT_OF_DEs_PER_SD><<<nSDs, MAX_TRIANGLE_COUNT_PER_BUCKET>>>(
            stepSize_SU, pos_X.data(), pos_Y.data(), pos_Z.data(), pos_X_dt_update.data(), pos_Y_dt_update.data(),
            pos_Z_dt_update.data(), SD_NumOf_DEs_Touching.data(), DEs_in_SD_composite.data(), pos_X_dt.data(),
            pos_Y_dt.data(), pos_Z_dt.data(), gran_params);
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        // broadphase the triangles
        // todo teh mesh soup needs to be unified memory I think
        triangleSoupBroadPhase<CUDA_THREADS><<<nSDs, MAX_COUNT_OF_DEs_PER_SD>>>(
            meshSoup_DEVICE, BUCKET_countsOfTrianglesTouching.data(), triangles_in_BUCKET_composite.data(),
            SD_countsOfTrianglesTouching.data(), gran_params, tri_params);
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());
        // TODO please do not use a template here
        // compute sphere-triangle forces
        interactionTerrain_TriangleSoup<CUDA_THREADS><<<nSDs, MAX_COUNT_OF_DEs_PER_SD>>>(
            meshSoup_DEVICE, pos_X.data(), pos_Y.data(), pos_Z.data(), pos_X_dt_update.data(), pos_Y_dt_update.data(),
            pos_Z_dt_update.data(), SD_NumOf_DEs_Touching.data(), DEs_in_SD_composite.data(),
            BUCKET_countsOfTrianglesTouching.data(), triangles_in_BUCKET_composite.data(),
            SD_countsOfTrianglesTouching.data(), gran_params, tri_params);

        // gpuErrchk(cudaPeekAtLastError());
        // gpuErrchk(cudaDeviceSynchronize());
        //
        // VERBOSE_PRINTF("Starting applyVelocityUpdates!\n");
        // // Apply the updates we just made
        // applyVelocityUpdates<MAX_COUNT_OF_DEs_PER_SD><<<nSDs, MAX_COUNT_OF_DEs_PER_SD>>>(
        //     stepSize_SU, pos_X.data(), pos_Y.data(), pos_Z.data(), pos_X_dt_update.data(), pos_Y_dt_update.data(),
        //     pos_Z_dt_update.data(), SD_NumOf_DEs_Touching.data(), DEs_in_SD_composite.data(), pos_X_dt.data(),
        //     pos_Y_dt.data(), pos_Z_dt.data(), gran_params);

        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());
        VERBOSE_PRINTF("Resetting broadphase info!\n");

        resetBroadphaseInformation();

        VERBOSE_PRINTF("Starting updatePositions!\n");
        updatePositions<CUDA_THREADS><<<nBlocks, CUDA_THREADS>>>(
            stepSize_SU, pos_X.data(), pos_Y.data(), pos_Z.data(), pos_X_dt.data(), pos_Y_dt.data(), pos_Z_dt.data(),
            pos_X_dt_update.data(), pos_Y_dt_update.data(), pos_Z_dt_update.data(), SD_NumOf_DEs_Touching.data(),
            DEs_in_SD_composite.data(), nDEs, gran_params);

        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());
    }
    return;
}
