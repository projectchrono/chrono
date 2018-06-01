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
// Authors: Dan Negrut, Conlain Kelly
// =============================================================================

#include <cuda.h>
#include <cassert>
#include "chrono_thirdparty/cub/cub.cuh"
#include "chrono_granular/ChGranularDefines.h"
#include "chrono_granular/physics/ChGranular.h"
//#include "chrono/core/ChVector.h"
#include "chrono_granular/utils/ChGranularUtilities_CUDA.cuh"
#include "chrono_granular/physics/ChGranularCollision.cuh"

// These are the max X, Y, Z dimensions in the BD frame
#define MAX_X_POS_UNSIGNED (d_SD_Ldim_SU * d_box_L_SU)
#define MAX_Y_POS_UNSIGNED (d_SD_Ddim_SU * d_box_D_SU)
#define MAX_Z_POS_UNSIGNED (d_SD_Hdim_SU * d_box_H_SU)

#define CUDA_THREADS 128

// Add verbose checks easily
#define VERBOSE_PRINTF(...)  \
    if (verbose_runtime) {   \
        printf(__VA_ARGS__); \
    }

// Print a user-given error message and crash
#define ABORTABORTABORT(...) \
    {                        \
        printf(__VA_ARGS__); \
        __threadfence();     \
        cub::ThreadTrap();   \
    }


/// Takes in a sphere's position and inserts into the given int array[8] which subdomains, if any, are touched
/// The array is indexed with the ones bit equal to +/- x, twos bit equal to +/- y, and the fours bit equal to +/- z
/// A bit set to 0 means the lower index, whereas 1 means the higher index (lower + 1)
/// The kernel computes global x, y, and z indices for the bottom-left subdomain and then uses those to figure out which
/// subdomains described in the corresponding 8-SD cube are touched by the sphere. The kernel then converts these
/// indices to indices into the global SD list via the (currently local) conv[3] data structure
/// Should be mostly bug-free, especially away from boundaries
__device__ void figureOutTouchedSD_mesh(uint3 nodeIDs, float3* nodeCoordXYZ, unsigned int SDs[8]) {
    float node_1_x;
    float node_1_y;
    float node_1_z;

    float node_2_x;
    float node_2_y;
    float node_2_z;

    float node_3_x;
    float node_3_y;
    float node_3_z;

    // We should use CUDA's fast_math here, all w/ float precision...
}

/**
 * This kernel call prepares information that will be used in a subsequent kernel that performs the actual time
 * stepping.
 *
 * Nomenclature:
 *   - SD: subdomain.
 *   - BD: the big-domain, which is the union of all SDs
 *   - NULL_GRANULAR_ID: the equivalent of a non-sphere SD ID, or a non-sphere ID
 *   - CPB: contact patch box - an AABB that contains the elements of the mesh that come in contact with the terrain.
 *                              Note that one mesh can have several CPB in relation to the same chunk of terrain.
 *
 * Template arguments:
 *   - CUB_THREADS: the number of threads used in this kernel, comes into play when invoking CUB block collectives
 *
 * Arguments:
 * nTriangles - the number of triangles in the mesh
 * node_coordXYZ - array containing XYZ coordinates of the nodes
 * meshConnectivity - array of size 3*nTriangles storing the nodes of each triangle
 * nCPBs - the number of patches in which the mesh can come in contact w/ the granular material
 * LDHdimContactPatches - array for which each entry contains the LDH size of a CPB
 * originCPBs - array for which each entry contains the point with resepect to which the CPB is defined
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
 * The subdomains are axis-aligned relative to the reference frame associated with the *box*. The origin of the box is
 * in the corner of the box. Each CPB is an AAB.
 *
 */
template <
    unsigned int
        CUB_THREADS>  //!< Number of CUB threads engaged in block-collective CUB operations. Should be a multiple of 32
__global__ void
primingOperationsMesh(
    unsigned int nTriangles,  //!< Number of triangles in the implement
    float3* node_coordXYZ,    //!< Pointer to array containing XYZ coordinates of the nodes
    uint3* meshConnectivity,  //!< Array of size 3*nTriangles storing the nodes of each triangle
    unsigned char nPatches,
    uint3* LDHdimContactPatches,
    float3* envelopPrismOrigin,
    unsigned int* SD_countsOfTrianglesTouching,  //!< Array that for each SD indicates how many triangles touch this SD
    unsigned int* triangles_in_SD_composite  //!< Big array that works in conjunction with SD_countsOfTrianglesTouching.
                                             //!< "triangles_in_SD_composite" says which SD contains what triangles.
) {
    uint3 node_IDs;

    /// Set aside shared memory
    volatile __shared__ size_t offsetInComposite_TriangleInSD_Array[CUB_THREADS * 8];
    volatile __shared__ bool shMem_head_flags[CUB_THREADS * 8];

    typedef cub::BlockRadixSort<unsigned int, CUB_THREADS, 8, unsigned int> BlockRadixSortOP;
    __shared__ typename BlockRadixSortOP::TempStorage temp_storage_sort;

    typedef cub::BlockDiscontinuity<unsigned int, CUB_THREADS> Block_Discontinuity;
    __shared__ typename Block_Discontinuity::TempStorage temp_storage_disc;

    // Figure out what triangleID this thread will handle. We work with a 1D block structure and a 1D grid structure
    unsigned int myTriangleID = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned int triangleIDs[8] = {myTriangleID, myTriangleID, myTriangleID, myTriangleID,
                                   myTriangleID, myTriangleID, myTriangleID, myTriangleID};

    // This uses a lot of registers but is needed
    unsigned int SDsTouched[8] = {NULL_GRANULAR_ID, NULL_GRANULAR_ID, NULL_GRANULAR_ID, NULL_GRANULAR_ID,
                                  NULL_GRANULAR_ID, NULL_GRANULAR_ID, NULL_GRANULAR_ID, NULL_GRANULAR_ID};
    if (myTriangleID < nTriangles) {
        // Coalesced mem access
        node_IDs = meshConnectivity[myTriangleID];

        figureOutTouchedSD_mesh(node_IDs, node_coordXYZ, SDsTouched);
    }

    __syncthreads();

    // Sort by the ID of the SD touched
    BlockRadixSortOP(temp_storage_sort).Sort(SDsTouched, triangleIDs);
    __syncthreads();

    // Do a winningStreak search on whole block, might not have high utilization here
    bool head_flags[8];
    Block_Discontinuity(temp_storage_disc).FlagHeads(head_flags, SDsTouched, cub::Inequality());
    __syncthreads();

    // Write back to shared memory; eight-way bank conflicts here - to revisit later
    for (unsigned int i = 0; i < 8; i++) {
        shMem_head_flags[8 * threadIdx.x + i] = head_flags[i];
    }

    // Seed offsetInComposite_TriangleInSD_Array with "no valid ID" so that we know later on what is legit;
    // No shmem bank coflicts here, good access...
    for (unsigned int i = 0; i < 8; i++) {
        offsetInComposite_TriangleInSD_Array[i * CUB_THREADS + threadIdx.x] = NULL_GRANULAR_ID_LONG;
    }

    __syncthreads();

    // Count how many times an SD shows up in conjunction with the collection of CUB_THREADS spheres. There
    // will be some thread divergence here.
    // Loop through each potential SD, after sorting, and see if it is the start of a head
    for (unsigned int i = 0; i < 8; i++) {
        // SD currently touched, could easily be inlined
        unsigned int touchedSD = SDsTouched[i];
        if (touchedSD != NULL_GRANULAR_ID && head_flags[i]) {
            // current index into shared datastructure of length 8*CUB_THREADS, could easily be inlined
            unsigned int idInShared = 8 * threadIdx.x + i;
            unsigned int winningStreak = 0;
            // This is the beginning of a sequence of SDs with a new ID
            do {
                winningStreak++;
                // Go until we run out of threads on the warp or until we find a new head
            } while (idInShared + winningStreak < 8 * CUB_THREADS && !(shMem_head_flags[idInShared + winningStreak]));

            // if (touchedSD >= d_box_L_SU * d_box_D_SU * d_box_H_SU) {
            //     printf("invalid SD index %u on thread %u\n", mySphereID, touchedSD);
            // }

            // Store start of new entries
            size_t offset = atomicAdd(SD_countsOfTrianglesTouching + touchedSD, winningStreak);

            // The value offset now gives a *relative* offset in the composite array; i.e., spheres_in_SD_composite.
            // Get the absolute offset
            offset += ((size_t)touchedSD) * MAX_COUNT_OF_Triangles_PER_SD;

            // Produce the offsets for this streak of spheres with identical SD ids
            for (unsigned int i = 0; i < winningStreak; i++)
                offsetInComposite_TriangleInSD_Array[idInShared + i] = offset++;
        }
    }

    __syncthreads();  // needed since we write to shared memory above; i.e., offsetInComposite_SphInSD_Array

    // Write out the data now; register with triangles_in_SD_composite each sphere that touches a certain ID
    for (unsigned int i = 0; i < 8; i++) {
        size_t offset = offsetInComposite_TriangleInSD_Array[8 * threadIdx.x + i];
        if (offset != NULL_GRANULAR_ID_LONG) {
            triangles_in_SD_composite[offset] = triangleIDs[i];
        }
    }
}

__host__ void chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::run_simulation(float tEnd) {}

/**
This kernel call figures out forces on a sphere and carries out numerical integration to get the velocity updates of a
sphere.
MAX_NSPHERES_PER_SD - Max number of elements per SD. Shoudld be a power of two
MAX_TRIANGLES_PER_SD - Max number of elements per SD. Shoudld be a power of two
TRIANGLE_FAMILIES - The number of families that the triangles can belong to

Overview of implementation: One warp of threads will work on 32 triangles at a time to figure out the force that
they impress on a particular sphere. Note that each sphere enlists the services of one warp. If there are, say,
73 triangles touching this SD, it will take three trips to figure out the total force that the triangles will
impress upon the sphere that is active. If there are 256 threads in the block, then there will be 8 "active"
spheres since there are 8 warps in the block. Each thread in the block has enough registers to accummulate the
force felt by each "family", force that is the results between an interaction between a triangle and a sphere.
Say if sphere 232 touches a triangle that belongs to family 2, then a set of 6 generalized forces is going to 
be produced to account for the interaction between the said triangle and sphere 232.
*/
template <unsigned int N_CUDATHREADS, unsigned int MAX_NSPHERES_PER_SD, unsigned int MAX_TRIANGLES_PER_SD, unsigned int TRIANGLE_FAMILIES>
__global__ void grainTriangleInteraction(
    unsigned int nTriangles,                     //!< Number of triangles checked for collision with the terrain
    unsigned int nGrElements,                    //!< Number of spheres  in the terrain
    float sphereRadius,                          //!< Radius of the sphere in the granular terrain (UserUnits)
    unsigned int* SD_countsOfTrianglesTouching,  //!< Array that for each SD indicates how many triangles touch this SD
    unsigned int*
        triangles_in_SD_composite,  //!< Big array that works in conjunction with SD_countsOfTrianglesTouching.
                                    //!< "triangles_in_SD_composite" says which SD contains what triangles.
    unsigned int*
        SD_countsOfGrElemsTouching,        //!< Array that for each SD indicates how many grain elements touch this SD
    unsigned int* grElems_in_SD_composite  //!< Big array that works in conjunction with SD_countsOfGrElemsTouching.
                                           //!< "grElems_in_SD_composite" says which SD contains what grElements.
) {
    __shared__ unsigned int grElemID[MAX_NSPHERES_PER_SD]; //!< global ID of the grElements touching this SD
    __shared__ unsigned int triangID[MAX_NSPHERES_PER_SD]; //!< global ID of the triangles touching this SD

    __shared__ int sphX[MAX_NSPHERES_PER_SD]; //!< X coordinate of the grElement
    __shared__ int sphY[MAX_NSPHERES_PER_SD]; //!< Y coordinate of the grElement
    __shared__ int sphZ[MAX_NSPHERES_PER_SD]; //!< Z coordinate of the grElement

    __shared__ int node1_X[MAX_TRIANGLES_PER_SD]; //!< X coordinate of the 1st node of the triangle
    __shared__ int node1_Y[MAX_TRIANGLES_PER_SD]; //!< Y coordinate of the 1st node of the triangle
    __shared__ int node1_Z[MAX_TRIANGLES_PER_SD]; //!< Z coordinate of the 1st node of the triangle

    __shared__ int node2_X[MAX_TRIANGLES_PER_SD]; //!< X coordinate of the 2nd node of the triangle
    __shared__ int node2_Y[MAX_TRIANGLES_PER_SD]; //!< Y coordinate of the 2nd node of the triangle
    __shared__ int node2_Z[MAX_TRIANGLES_PER_SD]; //!< Z coordinate of the 2nd node of the triangle

    __shared__ int node3_X[MAX_TRIANGLES_PER_SD]; //!< X coordinate of the 3rd node of the triangle
    __shared__ int node3_Y[MAX_TRIANGLES_PER_SD]; //!< Y coordinate of the 3rd node of the triangle
    __shared__ int node3_Z[MAX_TRIANGLES_PER_SD]; //!< Z coordinate of the 3rd node of the triangle

    volatile __shared__ float tempShMem[N_CUDATHREADS/warpSize][6];  // used to do a block-level reduce

    float forceActingOnSphere[3]; //!< 3 registers will hold the value of the force on the sphere
    float genForceActingOnTriangles[TRIANGLE_FAMILIES][6]; //!< There are 6 components: 3 forces and 3 torques

    ///////////////////////////////////////////////////////////////////////////////////
    /// Fake stuff here, to get to compile
    ///////////////////////////////////////////////////////////////////////////////////
    int * d_grEleme_pos_X;
    int * d_grEleme_pos_Y;
    int * d_grEleme_pos_Z;

    int * d_node1_X_pos_X;
    int * d_node1_Y_pos_Y;
    int * d_node1_Z_pos_Z;

    int * d_node2_X_pos_X;
    int * d_node2_Y_pos_Y;
    int * d_node2_Z_pos_Z;

    int * d_node3_X_pos_X;
    int * d_node3_Y_pos_Y;
    int * d_node3_Z_pos_Z;
    ///////////////////////////////////////////////////////////////////////////////////
    /// End fake stuff
    ///////////////////////////////////////////////////////////////////////////////////

    unsigned int thisSD = blockIdx.x + gridDim.x * (blockIdx.y + gridDim.y * blockIdx.z);
    unsigned int nSD_triangles = SD_countsOfTrianglesTouching[thisSD];
    unsigned int nSD_spheres = SD_countsOfGrElemsTouching[thisSD];

    if (nSD_triangles == 0 || nSD_spheres == 0) return;

    // Populate the shared memory with grElement data
    unsigned int tripsToCoverSpheres = (nSD_spheres + blockDim.x - 1) / blockDim.x;
    unsigned int local_ID = threadIdx.x;
    for (unsigned int sphereTrip = 0; sphereTrip < tripsToCoverSpheres; sphereTrip++) {
        local_ID += sphereTrip * blockDim.x;
        if (local_ID < nSD_spheres) {
            unsigned int globalID = grElems_in_SD_composite[local_ID + thisSD * MAX_NSPHERES_PER_SD];
            grElemID[local_ID] = globalID;
            sphX[local_ID] = d_grEleme_pos_X[globalID];
            sphY[local_ID] = d_grEleme_pos_Y[globalID];
            sphZ[local_ID] = d_grEleme_pos_Z[globalID];
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
            node1_X[local_ID] = d_node1_X_pos_X[globalID];
            node1_Y[local_ID] = d_node1_Y_pos_Y[globalID];
            node1_Z[local_ID] = d_node1_Z_pos_Z[globalID];

            node2_X[local_ID] = d_node2_X_pos_X[globalID];
            node2_Y[local_ID] = d_node2_Y_pos_Y[globalID];
            node2_Z[local_ID] = d_node2_Z_pos_Z[globalID];

            node3_X[local_ID] = d_node3_X_pos_X[globalID];
            node3_Y[local_ID] = d_node3_Y_pos_Y[globalID];
            node3_Z[local_ID] = d_node3_Z_pos_Z[globalID];
        }
    }

    __syncthreads(); // this call ensures data is in its place in shared memory

    /// Zero out the force and torque at the onset of the computation
    for (local_ID = 0; local_ID < TRIANGLE_FAMILIES; local_ID++) {
        /// forces acting on the triangle, in global reference frame
        genForceActingOnTriangles[local_ID][0] = 0.f;
        genForceActingOnTriangles[local_ID][1] = 0.f;
        genForceActingOnTriangles[local_ID][2] = 0.f;
        /// torques with respect to global reference frame, expressed in global reference frame
        genForceActingOnTriangles[local_ID][3] = 0.f;
        genForceActingOnTriangles[local_ID][4] = 0.f;
        genForceActingOnTriangles[local_ID][5] = 0.f;
    }

    // Each sphere has one warp of threads dedicated to identifying all triangles that this sphere
    // touches. Upon a contact event, we'll compute the normal force on the sphere; and, the force and torque impressed
    // upon the triangle

    unsigned int nSpheresProcessedAtOneTime = blockDim.x / warpSize; /// One warp allocated to slave serving one sphere
    tripsToCoverSpheres = (nSD_spheres + nSpheresProcessedAtOneTime - 1) / nSpheresProcessedAtOneTime;
    tripsToCoverTriangles = (nSD_triangles + warpSize - 1) / warpSize; 

    unsigned sphere_Local_ID = threadIdx.x / warpSize;
    for (unsigned int sphereTrip = 0; sphereTrip < tripsToCoverSpheres; sphereTrip++) {
        /// before starting dealing with a sphere, zero out the forces acting on it; all threads in the block are doing
        /// this
        forceActingOnSphere[0] = 0.f;
        forceActingOnSphere[1] = 0.f;
        forceActingOnSphere[2] = 0.f;
        sphere_Local_ID += sphereTrip * nSpheresProcessedAtOneTime;
        if (sphere_Local_ID < nSD_spheres) {
            /// Figure out which triangles this sphere collides with; each thread in a warp slaving for this sphere
            /// looks at one triangle at a time. The collection of threads in the warp sweeps through all the triangles
            /// that touch this SD. NOTE: to avoid double-counting, a sphere-triangle collision event is counted only if
            /// the collision point is in this SD.
            unsigned int targetTriangle =  (threadIdx.x & (warpSize - 1)); // computes modulo 32 of the thread index
            for (unsigned int triangTrip = 0; triangTrip < tripsToCoverTriangles; triangTrip++) {
                targetTriangle += triangTrip * warpSize;
                if (targetTriangle < nSD_triangles) {
                    /// we have a valid sphere and a valid triganle; check if in contact
                    float3 norm;
                    float depth;
                    float3 pt1;
                    float3 pt2;
                    float eff_radius;
                    face_sphere(make_float3(node1_X[targetTriangle], node1_Y[targetTriangle], node1_Z[targetTriangle]),
                                make_float3(node2_X[targetTriangle], node2_Y[targetTriangle], node2_Z[targetTriangle]),
                                make_float3(node3_X[targetTriangle], node3_Y[targetTriangle], node3_Z[targetTriangle]),
                                make_float3(sphX[sphere_Local_ID], sphY[sphere_Local_ID], sphZ[sphere_Local_ID]),
                                sphereRadius, 0.f, norm, depth, pt1, pt2, eff_radius);

                    /// Use the CD information to compute the force on the grElement

                    /// Use the CD information to compute the force and torque on the triangle
                }
            }
            /// down to the point where we need to collect the forces from all the threads in the wrap; this is a warp
            /// reduce operation. The resultant force acting on this grElement is stored in the first lane of the warp.
            /// NOTE: In this warp-level operations participate only the warps that are slaving for a sphere; i.e., some
            /// warps see no action
            for (unsigned int offset = warpSize / 2; offset > 0; offset /= 2) 
                forceActingOnSphere[0] += __shfl_down_sync(0xffffffff, forceActingOnSphere[0], offset);
            for (unsigned int offset = warpSize / 2; offset > 0; offset /= 2)
                forceActingOnSphere[1] += __shfl_down_sync(0xffffffff, forceActingOnSphere[1], offset);
            for (unsigned int offset = warpSize / 2; offset > 0; offset /= 2)
                forceActingOnSphere[2] += __shfl_down_sync(0xffffffff, forceActingOnSphere[2], offset);
                
            /// done with the computation of all the contacts that the triangles impress on this sphere. Update the
            /// position of the sphere based on this force
        }
    }
    /// Done computing the forces acting on the triangles in this SD. A block reduce is carried out next. Start by doing
    /// a reduce at the warp level
    for (local_ID = 0; local_ID < TRIANGLE_FAMILIES; local_ID++) {
        /// forces acting on the triangle, in global reference frame
        for (unsigned int offset = warpSize / 2; offset > 0; offset /= 2)
            genForceActingOnTriangles[local_ID][0] += __shfl_down_sync(0xffffffff, genForceActingOnTriangles[local_ID][0], offset);

        for (unsigned int offset = warpSize / 2; offset > 0; offset /= 2)
            genForceActingOnTriangles[local_ID][1] += __shfl_down_sync(0xffffffff, genForceActingOnTriangles[local_ID][1], offset);

        for (unsigned int offset = warpSize / 2; offset > 0; offset /= 2)
            genForceActingOnTriangles[local_ID][2] += __shfl_down_sync(0xffffffff, genForceActingOnTriangles[local_ID][2], offset);

        for (unsigned int offset = warpSize / 2; offset > 0; offset /= 2)
            genForceActingOnTriangles[local_ID][3] += __shfl_down_sync(0xffffffff, genForceActingOnTriangles[local_ID][3], offset);

        for (unsigned int offset = warpSize / 2; offset > 0; offset /= 2)
            genForceActingOnTriangles[local_ID][4] += __shfl_down_sync(0xffffffff, genForceActingOnTriangles[local_ID][4], offset);

        for (unsigned int offset = warpSize / 2; offset > 0; offset /= 2)
            genForceActingOnTriangles[local_ID][5] += __shfl_down_sync(0xffffffff, genForceActingOnTriangles[local_ID][5], offset);
    }
    __syncthreads();
    /// Lane zero in each warp has the result of a warp-level reduce operation. Sum up these "Lane zero" values in one final result
    bool threadIsLaneZeroInWarp = (threadIdx.x & (warpSize - 1) == 0);
    for (local_ID = 0; local_ID < TRIANGLE_FAMILIES; local_ID++) {
        /// forces acting on the triangle, in global reference frame
        if (threadIsLaneZeroInWarp) {
            unsigned int offset = threadIdx.x / warpSize;
            tempShMem[offset][0] = genForceActingOnTriangles[local_ID][0];
            tempShMem[offset][1] = genForceActingOnTriangles[local_ID][1];
            tempShMem[offset][2] = genForceActingOnTriangles[local_ID][2];

            tempShMem[offset][3] = genForceActingOnTriangles[local_ID][3];
            tempShMem[offset][4] = genForceActingOnTriangles[local_ID][4];
            tempShMem[offset][5] = genForceActingOnTriangles[local_ID][5];
        }
        __syncthreads();

        /// Going to trash the values in "forceActingOnSphere", not needed anymore. Reuse the registers, which will now
        /// store the vaule of the triangle force and torque...
        if (threadIdx.x < warpSize) {
            /// only first thread in block participates in this reduce operation.
            /// Note the implicit assumption made here: warpSize is larger than N_CUDATHREADS / warpSize. This is true
            /// today as N_CUDATHREADS cannot be larger than 1024 and warpSize is 32
            if (threadIdx.x < (N_CUDATHREADS / warpSize)) {
                forceActingOnSphere[0] = tempShMem[threadIdx.x][0];
                forceActingOnSphere[1] = tempShMem[threadIdx.x][1];
                forceActingOnSphere[2] = tempShMem[threadIdx.x][2];
            }
            else {
                forceActingOnSphere[0] = 0.f;
                forceActingOnSphere[1] = 0.f;
                forceActingOnSphere[2] = 0.f;
            }
            for (unsigned int offset = warpSize / 2; offset > 0; offset /= 2)
                forceActingOnSphere[0] += __shfl_down_sync(0xffffffff, forceActingOnSphere[0], offset);
            for (unsigned int offset = warpSize / 2; offset > 0; offset /= 2)
                forceActingOnSphere[1] += __shfl_down_sync(0xffffffff, forceActingOnSphere[1], offset);
            for (unsigned int offset = warpSize / 2; offset > 0; offset /= 2)
                forceActingOnSphere[2] += __shfl_down_sync(0xffffffff, forceActingOnSphere[2], offset);
        }
    }
}