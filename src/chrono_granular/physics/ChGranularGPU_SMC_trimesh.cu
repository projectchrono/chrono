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

#include <cuda.h>
#include "chrono_granular/ChGranularDefines.h"
#include "chrono_granular/physics/ChGranularCollision.cuh"
#include "chrono_granular/physics/ChGranularTriMesh.h"
#include "chrono_granular/utils/ChCudaMathUtils.cuh"
#include "chrono_granular/utils/ChGranularUtilities_CUDA.cuh"
#include "chrono_thirdparty/cub/cub.cuh"

// TODO should this go here?
// NOTE warpSize is a cuda environment value, but it is cc-dependent
#define warp_size 32

#define CUDA_THREADS 128

// These are the max X, Y, Z dimensions in the BD frame
#define MAX_X_POS_UNSIGNED (d_SD_Ldim_SU * d_box_L_SU)
#define MAX_Y_POS_UNSIGNED (d_SD_Ddim_SU * d_box_D_SU)
#define MAX_Z_POS_UNSIGNED (d_SD_Hdim_SU * d_box_H_SU)

#define Min(a, b) (a < b) ? a : b
#define Max(a, b) (a > b) ? a : b

#define Triangle_Soup chrono::granular::ChTriangleSoup

/// Takes in a triangle's position and finds out what SDs it touches
__device__ void triangle_figureOutTouchedSDs(unsigned int triangleID,
                                             const Triangle_Soup<int>& triangleSoup,
                                             unsigned int* touchedSDs) {
    unsigned int SD_count = 0;
    float3 vA, vB, vC;
    // Coalesced memory accesses; we have an int to float conversion here
    vA.x = triangleSoup.node1_X[triangleID];
    vA.y = triangleSoup.node1_Y[triangleID];
    vA.z = triangleSoup.node1_Z[triangleID];

    vB.x = triangleSoup.node2_X[triangleID];
    vB.y = triangleSoup.node2_Y[triangleID];
    vB.z = triangleSoup.node2_Z[triangleID];

    vC.x = triangleSoup.node3_X[triangleID];
    vC.y = triangleSoup.node3_Y[triangleID];
    vC.z = triangleSoup.node3_Z[triangleID];

    uint3 SDA = pointSDTriplet(vA.x, vA.y, vA.z);  // SD indices for point A
    uint3 SDB = pointSDTriplet(vB.x, vB.y, vB.z);  // SD indices for point B
    uint3 SDC = pointSDTriplet(vC.x, vC.y, vC.z);  // SD indices for point C

    uint3 L;  // Min SD index along each axis
    uint3 U;  // Max SD index along each axis

    L[0] = Min(SDA.x, Min(SDB.x, SDC.x));
    L[1] = Min(SDA.y, Min(SDB.y, SDC.y));
    L[2] = Min(SDA.z, Min(SDB.z, SDC.z));

    U[0] = Max(SDA.x, Max(SDB.x, SDC.x));
    U[1] = Max(SDA.y, Max(SDB.y, SDC.y));
    U[2] = Max(SDA.z, Max(SDB.z, SDC.z));

    // Case 1: All vetices are in the same SD
    if (L[0] == U[0] && L[1] == U[1] && L[2] == U[2]) {
        touchedSDs[SD_count++] = SDTripletID(L);
        return;
    }

    unsigned int n_axes_diff = 0;  // Count axes that have different SD bounds
    unsigned int axis_diff;

    for (unsigned int i = 0; i < 3; i++) {
        if (L[i] != U[i]) {
            axes_diff = i;  // If there is more than one, it won't be used anyway
            n_axes_diff++;
        }
    }

    // Case 2: Triangle lies in a Nx1x1, 1xNx1, or 1x1xN block of SDs
    if (n_axes_diff == 1) {
        uint3 SD_i = L;
        for (unsigned int i = L[axes_diff]; i <= U[axes_diff]; i++) {
            SD_i[axes_diff] = i;
            touchedSDs[SD_count++] = SDTripletID(SD_i);
        }
        return;
    }

    // Case 3: Triangle spans more than one dimension of nSD_spheres
    float SDcenter[3];
    float SDhalfSizes[3];
    uint3 SD_i = L;
    for (unsigned int i = L[0]; i <= U[0]; i++) {
        for (unsigned int j = L[1]; j <= U[1]; j++) {
            for (unsigned int k = L[2]; k <= U[2]; k++) {
                uint3 SD_i(i, j, k);
                SDhalfSizes[0] = d_SD_Ldim_SU;
                SDhalfSizes[1] = d_SD_Ddim_SU;
                SDhalfSizes[2] = d_SD_Hdim_SU;

                SDcenter[0] = d_BD_frame_X + (i * 2 + 1) * SDhalfSizes[0];
                SDcenter[1] = d_BD_frame_Y + (j * 2 + 1) * SDhalfSizes[1];
                SDcenter[2] = d_BD_frame_Z + (k * 2 + 1) * SDhalfSizes[2];

                if (check_TriangleBoxOverlap(SDcenter, SDhalfSizes, vA, vB, vC)) {
                    touchedSDs[SD_count++] = SDTripletID(SD_i);
                }
            }
        }
    }
}

/**
 * This kernel call prepares information that will be used in a subsequent kernel that performs the actual time
 * stepping.
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
 *   - The SDs defining a CPB are a subset of the SDs spanning the terrain (granular material)
 *   - Each CPB has dimensions L x D x H.
 *   - The reference frame associated with the AABB that a CPB is:
 *       - The x-axis is along the length L of the box
 *       - The y-axis is along the width D of the box
 *       - The z-axis is along the height H of the box
 *       - The origin of the CPB is in a corner of the box
 *   - A mesh triangle cannot touch more than eight SDs
 *
 * Basic idea: use domain decomposition on the rectangular box and figure out how many SDs each triangle touches.
 * The subdomains are axis-aligned relative to the reference frame associated with the *box*. The origin of the box
 * is in the corner of the box. Each CPB is an AAB.
 *
 */
template <unsigned int CUB_THREADS>  //!< Number of threads engaged in block-collective CUB operations (multiple of 32)
__global__ void triangleSoupBroadPhase(
    Triangle_Soup<int>& d_triangleSoup,
    unsigned int*
        BUCKET_countsOfTrianglesTouching,  //!< Array that for each SD indicates how many triangles touch this SD
    unsigned int*
        triangles_in_BUCKET_composite  //!< Big array that works in conjunction with SD_countsOfTrianglesTouching.
                                       //!< "triangles_in_SD_composite" says which SD contains what triangles.
) {
    /// Set aside shared memory
    volatile __shared__ unsigned int offsetInComposite_TriangleInSD_Array[CUB_THREADS * MAX_SDs_TOUCHED_BY_TRIANGLE];
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
        /// start with a clean slate
        triangleIDs[i] = myTriangleID;
        SDsTouched[i] = NULL_GRANULAR_ID;
    }

    if (myTriangleID < d_triangleSoup.nTrianglesInSoup)
        triangle_figureOutTouchedSDs(myTriangleID, d_triangleSoup, SDsTouched);

    __syncthreads();

    // Truth be told, we are not interested in SDs touched, but rather buckets touched. This next step associates
    // SDs with "buckets". To save memory, since most SDs have no triangles, we "randomly" associate sevearl SDs
    // with a bucket. While the assignment of SDs to buckets is "random," the assignment scheme is deterministic:
    // for instance, SD 239 would always go to bucket 71.
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_TRIANGLE; i++)
        SDsTouched[i] = hashmapTagGenerator(SDsTouched[i]) % TRIANGLEBUCKET_COUNT;

    // Sort by the ID of the bucket touched
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

    // Seed offsetInComposite_TriangleInSD_Array with "no valid ID" so that we know later on what is legit;
    // No shmem bank coflicts here, good access...
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_TRIANGLE; i++) {
        offsetInComposite_TriangleInSD_Array[i * CUB_THREADS + threadIdx.x] = NULL_GRANULAR_ID_LONG;
    }

    __syncthreads();

    // Count how many times a Bucket shows up in conjunction with the collection of CUB_THREADS spheres. There
    // will be some thread divergence here.
    // Loop through each potential SD, after sorting, and see if it is the start of a head
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_TRIANGLE; i++) {
        // SD currently touched, could easily be inlined
        unsigned int touchedBucket = SDsTouched[i];
        if (touchedBucket != NULL_GRANULAR_ID && head_flags[i]) {
            // current index into shared datastructure of length 8*CUB_THREADS, could easily be inlined
            unsigned int idInShared = MAX_SDs_TOUCHED_BY_TRIANGLE * threadIdx.x + i;
            unsigned int winningStreak = 0;
            // This is the beginning of a sequence of SDs with a new ID
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

            // The value offset now gives a *relative* offset in the composite array; i.e., spheres_in_SD_composite.
            // Get the absolute offset
            offset += touchedBucket * MAX_COUNT_OF_Triangles_PER_SD;

            // Produce the offsets for this streak of spheres with identical SD ids
            for (unsigned int i = 0; i < winningStreak; i++)
                offsetInComposite_TriangleInSD_Array[idInShared + i] = offset++;
        }
    }

    __syncthreads();  // needed since we write to shared memory above; i.e., offsetInComposite_SphInSD_Array

    // Write out the data now; register with triangles_in_SD_composite each sphere that touches a certain ID
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_TRIANGLE; i++) {
        unsigned int offset = offsetInComposite_TriangleInSD_Array[MAX_SDs_TOUCHED_BY_TRIANGLE * threadIdx.x + i];
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
MAX_TRIANGLES_PER_SD - Max number of elements per SD. Shoudld be a power of two
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
#define Terrain chrono::granular::ChManyBodyStateWrapper

template <unsigned int N_CUDATHREADS,
          unsigned int MAX_NSPHERES_PER_SD,
          unsigned int MAX_TRIANGLES_PER_SD,
          unsigned int TRIANGLE_FAMILIES>
__global__ void interactionTerrain_TriangleSoup(
    Triangle_Soup<int>& d_triangleSoup,          //!< Contains information pertaining to triangle soup (in device mem.)
    Terrain& d_terrain,                          //!< Wrapper that stores terrain information available on the device
    unsigned int* SD_countsOfTrianglesTouching,  //!< Array that for each SD indicates how many triangles touch this SD
    unsigned int*
        triangles_in_SD_composite,  //!< Big array that works in conjunction with SD_countsOfTrianglesTouching.
                                    //!< "triangles_in_SD_composite" says which SD contains what triangles.
    unsigned int*
        SD_countsOfGrElemsTouching,         //!< Array that for each SD indicates how many grain elements touch this SD
    unsigned int* grElems_in_SD_composite)  //!< Big array that works in conjunction with SD_countsOfGrElemsTouching.
                                            //!< "grElems_in_SD_composite" says which SD contains what grElements.
{
    __shared__ unsigned int grElemID[MAX_NSPHERES_PER_SD];  //!< global ID of the grElements touching this SD
    __shared__ unsigned int triangID[MAX_NSPHERES_PER_SD];  //!< global ID of the triangles touching this SD

    __shared__ int sphX[MAX_NSPHERES_PER_SD];  //!< X coordinate of the grElement
    __shared__ int sphY[MAX_NSPHERES_PER_SD];  //!< Y coordinate of the grElement
    __shared__ int sphZ[MAX_NSPHERES_PER_SD];  //!< Z coordinate of the grElement

    __shared__ int node1_X[MAX_TRIANGLES_PER_SD];  //!< X coordinate of the 1st node of the triangle
    __shared__ int node1_Y[MAX_TRIANGLES_PER_SD];  //!< Y coordinate of the 1st node of the triangle
    __shared__ int node1_Z[MAX_TRIANGLES_PER_SD];  //!< Z coordinate of the 1st node of the triangle

    __shared__ int node2_X[MAX_TRIANGLES_PER_SD];  //!< X coordinate of the 2nd node of the triangle
    __shared__ int node2_Y[MAX_TRIANGLES_PER_SD];  //!< Y coordinate of the 2nd node of the triangle
    __shared__ int node2_Z[MAX_TRIANGLES_PER_SD];  //!< Z coordinate of the 2nd node of the triangle

    __shared__ int node3_X[MAX_TRIANGLES_PER_SD];  //!< X coordinate of the 3rd node of the triangle
    __shared__ int node3_Y[MAX_TRIANGLES_PER_SD];  //!< Y coordinate of the 3rd node of the triangle
    __shared__ int node3_Z[MAX_TRIANGLES_PER_SD];  //!< Z coordinate of the 3rd node of the triangle

    volatile __shared__ float tempShMem[6 * (N_CUDATHREADS / warp_size)];  // used to do a block-level reduce

    float forceActingOnSphere[3];  //!< 3 registers will hold the value of the force on the sphere
    float genForceActingOnMeshes[TRIANGLE_FAMILIES * 6];  //!< 6 components per family: 3 forces and 3 torques

    unsigned int thisSD = blockIdx.x;
    unsigned int nSD_triangles = SD_countsOfTrianglesTouching[thisSD];
    unsigned int nSD_spheres = SD_countsOfGrElemsTouching[thisSD];

    if (nSD_triangles == 0 || nSD_spheres == 0)
        return;

    // Populate the shared memory with terrain data
    unsigned int tripsToCoverSpheres = (nSD_spheres + blockDim.x - 1) / blockDim.x;
    unsigned int local_ID = threadIdx.x;
    for (unsigned int sphereTrip = 0; sphereTrip < tripsToCoverSpheres; sphereTrip++) {
        local_ID += sphereTrip * blockDim.x;
        if (local_ID < nSD_spheres) {
            unsigned int globalID = grElems_in_SD_composite[local_ID + thisSD * MAX_NSPHERES_PER_SD];
            grElemID[local_ID] = globalID;
            sphX[local_ID] = d_terrain.grElem_X[globalID];
            sphY[local_ID] = d_terrain.grElem_Y[globalID];
            sphZ[local_ID] = d_terrain.grElem_Z[globalID];
        }
    }
    // Populate the shared memory with mesh triangle data
    unsigned int tripsToCoverTriangles = (nSD_triangles + blockDim.x - 1) / blockDim.x;
    local_ID = threadIdx.x;
    for (unsigned int triangTrip = 0; triangTrip < tripsToCoverTriangles; triangTrip++) {
        local_ID += triangTrip * blockDim.x;
        if (local_ID < nSD_triangles) {
            unsigned int globalID = triangles_in_SD_composite[local_ID + thisSD * MAX_TRIANGLES_PER_SD];
            triangID[local_ID] = globalID;
            node1_X[local_ID] = d_triangleSoup.node1_X[globalID];
            node1_Y[local_ID] = d_triangleSoup.node1_Y[globalID];
            node1_Z[local_ID] = d_triangleSoup.node1_Z[globalID];

            node2_X[local_ID] = d_triangleSoup.node2_X[globalID];
            node2_Y[local_ID] = d_triangleSoup.node2_Y[globalID];
            node2_Z[local_ID] = d_triangleSoup.node2_Z[globalID];

            node3_X[local_ID] = d_triangleSoup.node3_X[globalID];
            node3_Y[local_ID] = d_triangleSoup.node3_Y[globalID];
            node3_Z[local_ID] = d_triangleSoup.node3_Z[globalID];
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
    tripsToCoverTriangles = (nSD_triangles + warp_size - 1) / warp_size;

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
                if (targetTriangle < nSD_triangles) {
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
                    face_sphere_cd(A, B, C, sphCntr, d_terrain.sphereRadius, norm, depth, pt1, pt2, eff_radius);

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
                atomicAdd(d_triangleSoup.generalizedForcesPerFamily + offset, genForceActingOnMeshes[offset]);
        }
    }
}

/// Copy const triangle data to device
void chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::copy_triangle_data_to_device() {
    // Handle what's specific to the case when the mesh is present
    // gpuErrchk(cudaMemcpyToSymbol(d_Kn_s2m_SU, &K_n_s2m_SU, sizeof(d_Kn_s2m_SU)));
}

__host__ void chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::run_simulation(float tEnd) {
    NOT_IMPLEMENTED_YET;
}

__host__ void chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::advance_simulation(
    float duration) {
    // Figure our the number of blocks that need to be launched to cover the box
    unsigned int nBlocks = (nDEs + CUDA_THREADS - 1) / CUDA_THREADS;

    // Settling simulation loop.
    unsigned int stepSize_SU = 5;
    unsigned int duration_SU = std::ceil(duration / (TIME_UNIT * PSI_h));
    unsigned int nsteps = (1.0 * duration_SU) / stepSize_SU;

    printf("advancing by %u at timestep %u, %u timesteps at approx user timestep %f\n", duration_SU, stepSize_SU,
           nsteps, duration / nsteps);
    printf("z grav term with timestep %u is %f\n", stepSize_SU, stepSize_SU * stepSize_SU * gravity_Z_SU);

    VERBOSE_PRINTF("Starting Main Simulation loop!\n");
    // Run the simulation, there are aggressive synchronizations because we want to have no race conditions
    for (unsigned int crntTime_SU = 0; crntTime_SU < stepSize_SU * nsteps; crntTime_SU += stepSize_SU) {
        /// DO STEP LOOP HERE

        // // reset forces to zero, note that vel update ~ force for forward euler
        // gpuErrchk(cudaMemset(pos_X_dt_update.data(), 0, nDEs * sizeof(float)));
        // gpuErrchk(cudaMemset(pos_Y_dt_update.data(), 0, nDEs * sizeof(float)));
        // gpuErrchk(cudaMemset(pos_Z_dt_update.data(), 0, nDEs * sizeof(float)));
        //
        // VERBOSE_PRINTF("Starting computeVelocityUpdates!\n");
        //
        // // Compute forces and crank into vel updates, we have 2 kernels to avoid a race condition
        // computeVelocityUpdates<MAX_COUNT_OF_DEs_PER_SD><<<nSDs, MAX_COUNT_OF_DEs_PER_SD>>>(
        //     stepSize_SU, pos_X.data(), pos_Y.data(), pos_Z.data(), pos_X_dt_update.data(),
        //     pos_Y_dt_update.data(), pos_Z_dt_update.data(), SD_NumOf_DEs_Touching.data(),
        //     DEs_in_SD_composite.data(), pos_X_dt.data(), pos_Y_dt.data(), pos_Z_dt.data());
        // gpuErrchk(cudaPeekAtLastError());
        // gpuErrchk(cudaDeviceSynchronize());
        //
        // VERBOSE_PRINTF("Starting applyVelocityUpdates!\n");
        // // Apply the updates we just made
        // applyVelocityUpdates<MAX_COUNT_OF_DEs_PER_SD><<<nSDs, MAX_COUNT_OF_DEs_PER_SD>>>(
        //     stepSize_SU, pos_X.data(), pos_Y.data(), pos_Z.data(), pos_X_dt_update.data(),
        //     pos_Y_dt_update.data(), pos_Z_dt_update.data(), SD_NumOf_DEs_Touching.data(),
        //     DEs_in_SD_composite.data(), pos_X_dt.data(), pos_Y_dt.data(), pos_Z_dt.data());
        //
        // gpuErrchk(cudaPeekAtLastError());
        // gpuErrchk(cudaDeviceSynchronize());
        // VERBOSE_PRINTF("Resetting broadphase info!\n");
        //
        // // Reset broadphase information
        // resetBroadphaseInformation();
        //
        // VERBOSE_PRINTF("Starting updatePositions!\n");
        // updatePositions<CUDA_THREADS><<<nBlocks, CUDA_THREADS>>>(
        //     stepSize_SU, pos_X.data(), pos_Y.data(), pos_Z.data(), pos_X_dt.data(), pos_Y_dt.data(),
        //     pos_Z_dt.data(), SD_NumOf_DEs_Touching.data(), DEs_in_SD_composite.data(), nDEs);
        //
        // gpuErrchk(cudaPeekAtLastError());
        // gpuErrchk(cudaDeviceSynchronize());
    }
    printf("SU radius is %u\n", sphereRadius_SU);
    // Don't write but print verbosely

    return;
}

void chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::cleanupSoup_DEVICE() {
    cudaFree(meshSoup_DEVICE.triangleFamily_ID);

    cudaFree(meshSoup_DEVICE.node1_X);
    cudaFree(meshSoup_DEVICE.node1_Y);
    cudaFree(meshSoup_DEVICE.node1_Z);

    cudaFree(meshSoup_DEVICE.node2_X);
    cudaFree(meshSoup_DEVICE.node2_Y);
    cudaFree(meshSoup_DEVICE.node2_Z);

    cudaFree(meshSoup_DEVICE.node3_X);
    cudaFree(meshSoup_DEVICE.node3_Y);
    cudaFree(meshSoup_DEVICE.node3_Z);

    cudaFree(meshSoup_DEVICE.node1_XDOT);
    cudaFree(meshSoup_DEVICE.node1_YDOT);
    cudaFree(meshSoup_DEVICE.node1_ZDOT);

    cudaFree(meshSoup_DEVICE.node2_XDOT);
    cudaFree(meshSoup_DEVICE.node2_YDOT);
    cudaFree(meshSoup_DEVICE.node2_ZDOT);

    cudaFree(meshSoup_DEVICE.node3_XDOT);
    cudaFree(meshSoup_DEVICE.node3_YDOT);
    cudaFree(meshSoup_DEVICE.node3_ZDOT);

    cudaFree(meshSoup_DEVICE.generalizedForcesPerFamily);
}

void chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::setupSoup_DEVICE(
    unsigned int nTriangles) {
    /// Allocate the DEVICE mesh soup
    meshSoup_DEVICE.nTrianglesInSoup = nTriangles;  // TODO this is not on device?

    gpuErrchk(cudaMalloc(&meshSoup_DEVICE.triangleFamily_ID, nTriangles * sizeof(unsigned int)));

    gpuErrchk(cudaMalloc(&meshSoup_DEVICE.node1_X, nTriangles * sizeof(float)));
    gpuErrchk(cudaMalloc(&meshSoup_DEVICE.node1_Y, nTriangles * sizeof(float)));
    gpuErrchk(cudaMalloc(&meshSoup_DEVICE.node1_Z, nTriangles * sizeof(float)));

    gpuErrchk(cudaMalloc(&meshSoup_DEVICE.node2_X, nTriangles * sizeof(float)));
    gpuErrchk(cudaMalloc(&meshSoup_DEVICE.node2_Y, nTriangles * sizeof(float)));
    gpuErrchk(cudaMalloc(&meshSoup_DEVICE.node2_Z, nTriangles * sizeof(float)));

    gpuErrchk(cudaMalloc(&meshSoup_DEVICE.node3_X, nTriangles * sizeof(float)));
    gpuErrchk(cudaMalloc(&meshSoup_DEVICE.node3_Y, nTriangles * sizeof(float)));
    gpuErrchk(cudaMalloc(&meshSoup_DEVICE.node3_Z, nTriangles * sizeof(float)));

    gpuErrchk(cudaMalloc(&meshSoup_DEVICE.node1_XDOT, nTriangles * sizeof(float)));
    gpuErrchk(cudaMalloc(&meshSoup_DEVICE.node1_YDOT, nTriangles * sizeof(float)));
    gpuErrchk(cudaMalloc(&meshSoup_DEVICE.node1_ZDOT, nTriangles * sizeof(float)));

    gpuErrchk(cudaMalloc(&meshSoup_DEVICE.node2_XDOT, nTriangles * sizeof(float)));
    gpuErrchk(cudaMalloc(&meshSoup_DEVICE.node2_YDOT, nTriangles * sizeof(float)));
    gpuErrchk(cudaMalloc(&meshSoup_DEVICE.node2_ZDOT, nTriangles * sizeof(float)));

    gpuErrchk(cudaMalloc(&meshSoup_DEVICE.node3_XDOT, nTriangles * sizeof(float)));
    gpuErrchk(cudaMalloc(&meshSoup_DEVICE.node3_YDOT, nTriangles * sizeof(float)));
    gpuErrchk(cudaMalloc(&meshSoup_DEVICE.node3_ZDOT, nTriangles * sizeof(float)));
}
