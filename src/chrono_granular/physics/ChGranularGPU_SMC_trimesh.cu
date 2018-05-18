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
#include <cstdio>
#include <fstream>
#include <ios>
#include <iostream>
#include <sstream>
#include <string>
#include "../../chrono_thirdparty/cub/cub.cuh"
#include "../ChGranularDefines.h"
#include "../chrono_granular/physics/ChGranular.h"
#include "chrono/core/ChVector.h"
#include "chrono_granular/utils/ChGranularUtilities_CUDA.cuh"

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

// Use user-defined quantities for coefficients
__constant__ float d_Gamma_n;  //!< contact damping coefficient, expressed in SU
// TODO we need to get the damping coefficient from user
__constant__ float d_K_n;  //!< Radius of the sphere, expressed in SU

__constant__ unsigned int d_sphereRadius_SU;  //!< Radius of the sphere, expressed in SU
__constant__ unsigned int d_SD_Ldim_SU;       //!< Ad-ed L-dimension of the SD box
__constant__ unsigned int d_SD_Ddim_SU;       //!< Ad-ed D-dimension of the SD box
__constant__ unsigned int d_SD_Hdim_SU;       //!< Ad-ed H-dimension of the SD box
__constant__ unsigned int psi_T_dFactor;      //!< factor used in establishing the software-time-unit
__constant__ unsigned int psi_h_dFactor;      //!< factor used in establishing the software-time-unit
__constant__ unsigned int psi_L_dFactor;      //!< factor used in establishing the software-time-unit
__constant__ unsigned int d_box_L_SU;         //!< Ad-ed L-dimension of the BD box in multiples of subdomains
__constant__ unsigned int d_box_D_SU;         //!< Ad-ed D-dimension of the BD box in multiples of subdomains
__constant__ unsigned int d_box_H_SU;         //!< Ad-ed H-dimension of the BD box in multiples of subdomains
__constant__ float gravAcc_X_d_factor_SU;     //!< Device counterpart of the constant gravity_X_SU
__constant__ float gravAcc_Y_d_factor_SU;     //!< Device counterpart of the constant gravity_Y_SU
__constant__ float gravAcc_Z_d_factor_SU;     //!< Device counterpart of the constant gravity_Z_SU

// Changed by updateBDPosition() at every timestep
__constant__ int d_BD_frame_X;  //!< The bottom-left corner xPos of the BD, allows boxes not centered at origin
__constant__ int d_BD_frame_Y;  //!< The bottom-left corner yPos of the BD, allows boxes not centered at origin
__constant__ int d_BD_frame_Z;  //!< The bottom-left corner zPos of the BD, allows boxes not centered at origin
// Disable because stability
// __constant__ float d_BD_frame_X_dot;  //!< The bottom-left corner xPos of the BD, allows boxes not centered at origin
// __constant__ float d_BD_frame_Y_dot;  //!< The bottom-left corner yPos of the BD, allows boxes not centered at origin
// __constant__ float d_BD_frame_Z_dot;  //!< The bottom-left corner zPos of the BD, allows boxes not centered at origin

__constant__ double d_DE_Mass;
__constant__ float d_cohesion_ratio;

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

    const size_t max_composite_index = (size_t)d_box_D_SU * d_box_L_SU * d_box_H_SU * MAX_COUNT_OF_Triangles_PER_SD;

    // Write out the data now; register with triangles_in_SD_composite each sphere that touches a certain ID
    for (unsigned int i = 0; i < 8; i++) {
        size_t offset = offsetInComposite_TriangleInSD_Array[8 * threadIdx.x + i];
        if (offset != NULL_GRANULAR_ID_LONG) {
            if (offset >= max_composite_index) {
                ABORTABORTABORT(
                    "overrun during priming on thread %u block %u, offset is %zu, max is %zu,  sphere is %u\n",
                    threadIdx.x, blockIdx.x, offset, max_composite_index, triangleIDs[i]);
            } else {
                triangles_in_SD_composite[offset] = triangleIDs[i];
            }
        }
    }
}

__host__ void chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless_trimesh::run_simulation(float tEnd) {}
