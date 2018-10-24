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
// Holds common internal functions for GPU granular stuff
//
// =============================================================================
// Authors: Dan Negrut, Conlain Kelly, Nic Olsen
// =============================================================================

#pragma once

#include "../../chrono_thirdparty/cub/cub.cuh"

#include <cuda.h>
#include <cassert>
#include <cstdio>
#include <fstream>
#include <ios>
#include <iostream>
#include <sstream>
#include <string>
#include <cstdint>
#include "../ChGranularDefines.h"
#include "../chrono_granular/physics/ChGranular.h"
#include "chrono/core/ChVector.h"
#include "chrono_granular/utils/ChGranularUtilities_CUDA.cuh"
#include "chrono_granular/utils/ChCudaMathUtils.cuh"
#include "chrono_granular/physics/ChGranularBoundaryConditions.cuh"

using chrono::granular::sphereDataStruct;

// These are the max X, Y, Z dimensions in the BD frame
#define MAX_X_POS_UNSIGNED (gran_params->SD_size_X_SU * gran_params->nSDs_X)
#define MAX_Y_POS_UNSIGNED (gran_params->SD_size_Y_SU * gran_params->nSDs_Y)
#define MAX_Z_POS_UNSIGNED (gran_params->SD_size_Z_SU * gran_params->nSDs_Z)

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

// Decide which SD owns this point in space
// Pass it the Center of Mass location for a DE to get its owner, also used to get contact point
inline __device__ int3 pointSDTriplet(int sphCenter_X, int sphCenter_Y, int sphCenter_Z, ParamsPtr gran_params) {
    // Note that this offset allows us to have moving walls and the like very easily

    int64_t sphCenter_X_modified = -gran_params->BD_frame_X + sphCenter_X;
    int64_t sphCenter_Y_modified = -gran_params->BD_frame_Y + sphCenter_Y;
    int64_t sphCenter_Z_modified = -gran_params->BD_frame_Z + sphCenter_Z;
    int3 n;
    // Get the SD of the sphere's center in the xdir
    n.x = (sphCenter_X_modified) / gran_params->SD_size_X_SU;
    // Same for D and H
    n.y = (sphCenter_Y_modified) / gran_params->SD_size_Y_SU;
    n.z = (sphCenter_Z_modified) / gran_params->SD_size_Z_SU;
    return n;
}

// inline __device__ pointSDID(int sphCenter_X, int sphCenter_Y, int sphCenter_Z, ParamsPtr gran_params) {}

// Convert triplet to single int SD ID
inline __device__ unsigned int SDTripletID(const int i, const int j, const int k, ParamsPtr gran_params) {
    // if we're outside the BD in any direction, this is an invalid SD
    if (i < 0 || i >= gran_params->nSDs_X) {
        return NULL_GRANULAR_ID;
    }
    if (j < 0 || j >= gran_params->nSDs_Y) {
        return NULL_GRANULAR_ID;
    }
    if (k < 0 || k >= gran_params->nSDs_Z) {
        return NULL_GRANULAR_ID;
    }
    return i * gran_params->nSDs_Y * gran_params->nSDs_Z + j * gran_params->nSDs_Z + k;
}

// Convert triplet to single int SD ID
inline __device__ unsigned int SDTripletID(const int3& trip, ParamsPtr gran_params) {
    return SDTripletID(trip.x, trip.y, trip.z, gran_params);
}

// Convert triplet to single int SD ID
inline __device__ unsigned int SDTripletID(const int trip[3], ParamsPtr gran_params) {
    return SDTripletID(trip[0], trip[1], trip[2], gran_params);
}

/// Takes in a sphere's position and inserts into the given int array[8] which subdomains, if any, are touched
/// The array is indexed with the ones bit equal to +/- x, twos bit equal to +/- y, and the fours bit equal to +/- z
/// A bit set to 0 means the lower index, whereas 1 means the higher index (lower + 1)
/// The kernel computes global x, y, and z indices for the bottom-left subdomain and then uses those to figure out
/// which subdomains described in the corresponding 8-SD cube are touched by the sphere. The kernel then converts
/// these indices to indices into the global SD list via the (currently local) conv[3] data structure Should be
/// mostly bug-free, especially away from boundaries
inline __device__ void figureOutTouchedSD(int sphCenter_X,
                                          int sphCenter_Y,
                                          int sphCenter_Z,
                                          unsigned int SDs[MAX_SDs_TOUCHED_BY_SPHERE],
                                          ParamsPtr gran_params) {
    // grab radius
    const unsigned int sphereRadius_SU = gran_params->sphereRadius_SU;
    // I added these to fix a bug, we can inline them if/when needed but they ARE necessary
    // We need to offset so that the bottom-left corner is at the origin
    int64_t sphCenter_X_modified = -gran_params->BD_frame_X + sphCenter_X;
    int64_t sphCenter_Y_modified = -gran_params->BD_frame_Y + sphCenter_Y;
    int64_t sphCenter_Z_modified = -gran_params->BD_frame_Z + sphCenter_Z;
    unsigned int n[3];
    // TODO this doesn't handle if the ball is slightly penetrating the boundary, could result in negative values or end
    // GIDs beyond bounds. We might want to do a check to see if it's outside and set 'valid' accordingly
    // NOTE: This is integer arithmetic to compute the floor. We want to get the first SD below the sphere
    // nx = (xCenter - radius) / wx .
    n[0] = (sphCenter_X_modified - sphereRadius_SU) / gran_params->SD_size_X_SU;
    // Same for D and H
    n[1] = (sphCenter_Y_modified - sphereRadius_SU) / gran_params->SD_size_Y_SU;
    n[2] = (sphCenter_Z_modified - sphereRadius_SU) / gran_params->SD_size_Z_SU;
    // This is kind of gross and hacky, if we're at the bottom boundary, the bottom SD is 0
    // If we're at the top boundary, the top SD is the max
    if (sphCenter_X_modified - (signed int)sphereRadius_SU <= 0) {
        n[0] = 0;
    } else if (sphCenter_X_modified + (signed int)sphereRadius_SU >= MAX_X_POS_UNSIGNED) {
        n[0] = gran_params->nSDs_X - 1 - 1;  // Subtract one for last SD, subtract one more for bottom SD
    }
    if (sphCenter_Y_modified - (signed int)sphereRadius_SU <= 0) {
        n[1] = 0;
    } else if (sphCenter_Y_modified + (signed int)sphereRadius_SU >= MAX_Y_POS_UNSIGNED) {
        n[1] = gran_params->nSDs_Y - 1 - 1;  // Subtract one for last SD, subtract one more for bottom SD
    }
    if (sphCenter_Z_modified - (signed int)sphereRadius_SU <= 0) {
        n[2] = 0;
    } else if (sphCenter_Z_modified + (signed int)sphereRadius_SU >= MAX_Z_POS_UNSIGNED) {
        n[2] = gran_params->nSDs_Z - 1 - 1;  // Subtract one for last SD, subtract one more for bottom SD
    }
    // n[0] += (sphCenter_X_modified - (signed int)sphereRadius_SU <= 0) -
    //         (sphCenter_X_modified + (signed int)sphereRadius_SU >= MAX_X_POS);
    // n[1] += (sphCenter_Y_modified - (signed int)sphereRadius_SU <= 0) -
    //         (sphCenter_Y_modified + (signed int)sphereRadius_SU >= MAX_X_POS);
    // n[2] += (sphCenter_Z_modified - (signed int)sphereRadius_SU <= 0) -
    //         (sphCenter_Z_modified + (signed int)sphereRadius_SU >= MAX_X_POS);
    // This conditional says if we're at the boundary
    unsigned int boundary = sphCenter_X_modified - (signed int)sphereRadius_SU <= 0 ||
                            sphCenter_X_modified + (signed int)sphereRadius_SU >= MAX_X_POS_UNSIGNED ||
                            sphCenter_Y_modified - (signed int)sphereRadius_SU <= 0 ||
                            sphCenter_Y_modified + (signed int)sphereRadius_SU >= MAX_Y_POS_UNSIGNED ||
                            sphCenter_Z_modified - (signed int)sphereRadius_SU <= 0 ||
                            sphCenter_Z_modified + (signed int)sphereRadius_SU >= MAX_Z_POS_UNSIGNED;
    if (n[0] >= gran_params->nSDs_X) {
        ABORTABORTABORT(
            "x is too large, boundary is %u, n is %u, nmax is %u, pos is %d, mod is %d, max is %d, dim is %d\n",
            boundary, n[0], gran_params->nSDs_X, sphCenter_X, sphCenter_X_modified,
            gran_params->SD_size_X_SU * gran_params->nSDs_X, gran_params->SD_size_X_SU, sphereRadius_SU);
    }
    // Find distance from next box in relevant dir to center, we may be straddling the two
    int d[3];                                                              // Store penetrations
    d[0] = (n[0] + 1) * gran_params->SD_size_X_SU - sphCenter_X_modified;  // dx = (nx + 1)* wx - x
    d[1] = (n[1] + 1) * gran_params->SD_size_Y_SU - sphCenter_Y_modified;
    d[2] = (n[2] + 1) * gran_params->SD_size_Z_SU - sphCenter_Z_modified;

    // Calculate global indices from locals
    // ones bit is x, twos bit is y, threes bit is z
    // do some cute bit shifting and snag bit at correct place
    // For each index in SDs
    for (int i = 0; i < MAX_SDs_TOUCHED_BY_SPHERE; i++) {
        SDs[i] = 0;                // Init to 0
        unsigned int valid = 0x1;  // Assume this SD is touched at start

        // s adds an offset to directional index for SDs
        // High/low in x-dir
        // unsigned int s = i & 0x1; // Inlined now
        // Scale to global index and add to total
        SDs[i] += (n[0] + (i & 0x1)) * gran_params->nSDs_Y * gran_params->nSDs_Z;
        // s == own[e] evals true if the current SD is owner
        // If both touch it or we own it, the result is valid
        valid &= (abs(d[0]) < sphereRadius_SU) || ((i & 0x1) == (d[0] < 0));

        // High/low in y-dir
        // s = i & 0x2; // Inlined now
        // Scale to global index and add to total
        SDs[i] += (n[1] + ((i >> 1) & 0x1)) * gran_params->nSDs_Z;
        // If both touch it or we own it, the result is valid
        valid &= (abs(d[1]) < sphereRadius_SU) || (((i >> 1) & 0x1) == (d[1] < 0));

        // High/low in z-dir
        // s = i & 0x4; // Inlined now
        // Scale to global index and add to total
        SDs[i] += (n[2] + ((i >> 2) & 0x1));
        // If both touch it or we own it, the result is valid
        valid &= (abs(d[2]) < sphereRadius_SU) || (((i >> 2) & 0x1) == (d[2] < 0));

        // This is nasty but it checks if we're actually touching a bounary, in which case

        // This ternary is hopefully better than a conditional
        // If valid is false, then the SD is actually NULL_GRANULAR_ID
        if (valid && SDs[i] >= gran_params->nSDs_Y * gran_params->nSDs_X * gran_params->nSDs_Z) {
            ABORTABORTABORT(
                "UH OH!, sd overrun %u, boundary is %u, position is %d, %d, %d on thread %u, block %u, n is %u, %u, "
                "%u\n",
                SDs[i], boundary, sphCenter_X_modified, sphCenter_Y_modified, sphCenter_Z_modified, threadIdx.x,
                blockIdx.x, n[0], n[1], n[2]);
        }
        SDs[i] = (valid ? SDs[i] : NULL_GRANULAR_ID);
    }
}
/**
 * This kernel call prepares information that will be used in a subsequent kernel that performs the actual time
 * stepping.
 *
 * Template arguments:
 *   - CUB_THREADS: the number of threads used in this kernel, comes into play when invoking CUB block collectives
 *
 * Assumptions:
 *   - Granular material is made up of monodisperse spheres.
 *   - The function below assumes the spheres are in a box
 *   - The box has dimensions L x D x H.
 *   - The reference frame associated with the box:
 *       - The x-axis is along the length L of the box
 *       - The y-axis is along the width D of the box
 *       - The z-axis is along the height H of the box
 *   - A sphere cannot touch more than eight SDs
 *
 * Basic idea: use domain decomposition on the rectangular box and figure out how many SDs each sphere touches.
 * The subdomains are axis-aligned relative to the reference frame associated with the *box*. The origin of the box is
 * at the center of the box. The orientation of the box is defined relative to a world inertial reference frame.
 *
 * Nomenclature:
 *   - SD: subdomain.
 *   - BD: the big-domain, which is the union of all SDs
 *   - NULL_GRANULAR_ID: the equivalent of a non-sphere SD ID, or a non-sphere ID
 *
 * Notes:
 *   - The SD with ID=0 is the catch-all SD. This is the SD in which a sphere ends up if its not inside the rectangular
 * box. Usually, there is no sphere in this SD (THIS IS NOT IMPLEMENTED AS SUCH FOR NOW)
 *
 */
template <unsigned int CUB_THREADS>  //!< Number of CUB threads engaged in block-collective CUB operations. Should be a
                                     //!< multiple of 32
__global__ void primingOperationsRectangularBox(sphereDataStruct sphere_data,
                                                unsigned int nSpheres,  //!< Number of spheres in the box
                                                ParamsPtr gran_params) {
    int xSphCenter;
    int ySphCenter;
    int zSphCenter;

    /// Set aside shared memory
    // SD component of offset into composite array
    volatile __shared__ unsigned int SD_composite_offsets[CUB_THREADS * MAX_SDs_TOUCHED_BY_SPHERE];
    // sphere component of offset into composite array
    volatile __shared__ unsigned char sphere_composite_offsets[CUB_THREADS * MAX_SDs_TOUCHED_BY_SPHERE];
    volatile __shared__ bool shMem_head_flags[CUB_THREADS * MAX_SDs_TOUCHED_BY_SPHERE];

    typedef cub::BlockRadixSort<unsigned int, CUB_THREADS, MAX_SDs_TOUCHED_BY_SPHERE, unsigned int> BlockRadixSortOP;
    __shared__ typename BlockRadixSortOP::TempStorage temp_storage_sort;

    typedef cub::BlockDiscontinuity<unsigned int, CUB_THREADS> Block_Discontinuity;
    __shared__ typename Block_Discontinuity::TempStorage temp_storage_disc;

    // Figure out what sphereID this thread will handle. We work with a 1D block structure and a 1D grid structure
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned int sphIDs[MAX_SDs_TOUCHED_BY_SPHERE] = {mySphereID, mySphereID, mySphereID, mySphereID,
                                                      mySphereID, mySphereID, mySphereID, mySphereID};

    // This uses a lot of registers but is needed
    unsigned int SDsTouched[MAX_SDs_TOUCHED_BY_SPHERE] = {NULL_GRANULAR_ID, NULL_GRANULAR_ID, NULL_GRANULAR_ID,
                                                          NULL_GRANULAR_ID, NULL_GRANULAR_ID, NULL_GRANULAR_ID,
                                                          NULL_GRANULAR_ID, NULL_GRANULAR_ID};
    if (mySphereID < nSpheres) {
        // Coalesced mem access
        xSphCenter = sphere_data.pos_X[mySphereID];
        ySphCenter = sphere_data.pos_Y[mySphereID];
        zSphCenter = sphere_data.pos_Z[mySphereID];

        figureOutTouchedSD(xSphCenter, ySphCenter, zSphCenter, SDsTouched, gran_params);
    }

    __syncthreads();

    // Sort by the ID of the SD touched
    BlockRadixSortOP(temp_storage_sort).Sort(SDsTouched, sphIDs);
    __syncthreads();

    // Do a winningStreak search on whole block, might not have high utilization here
    bool head_flags[MAX_SDs_TOUCHED_BY_SPHERE];
    Block_Discontinuity(temp_storage_disc).FlagHeads(head_flags, SDsTouched, cub::Inequality());
    __syncthreads();

    // Write back to shared memory; eight-way bank conflicts here - to revisit later
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_SPHERE; i++) {
        shMem_head_flags[MAX_SDs_TOUCHED_BY_SPHERE * threadIdx.x + i] = head_flags[i];
    }

    // Seed offsetInComposite_SphInSD_Array with "no valid ID" so that we know later on what is legit;
    // No shmem bank coflicts here, good access...
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_SPHERE; i++) {
        SD_composite_offsets[i * CUB_THREADS + threadIdx.x] = NULL_GRANULAR_ID;
        sphere_composite_offsets[i * CUB_THREADS + threadIdx.x] = 0;
    }

    __syncthreads();

    // Count how many times an SD shows up in conjunction with the collection of CUB_THREADS spheres. There
    // will be some thread divergence here.
    // Loop through each potential SD, after sorting, and see if it is the start of a head
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_SPHERE; i++) {
        // SD currently touched, could easily be inlined
        unsigned int touchedSD = SDsTouched[i];
        if (touchedSD != NULL_GRANULAR_ID && head_flags[i]) {
            // current index into shared datastructure of length 8*CUB_THREADS, could easily be inlined
            unsigned int idInShared = MAX_SDs_TOUCHED_BY_SPHERE * threadIdx.x + i;
            unsigned int winningStreak = 0;  // should always be under 256 by simulation constraints
            // This is the beginning of a sequence of SDs with a new ID
            do {
                winningStreak++;
                // Go until we run out of threads on the warp or until we find a new head
            } while (idInShared + winningStreak < MAX_SDs_TOUCHED_BY_SPHERE * CUB_THREADS &&
                     !(shMem_head_flags[idInShared + winningStreak]));

            // Store start of new entries
            unsigned char sphere_offset = atomicAdd(sphere_data.SD_NumOf_DEs_Touching + touchedSD, winningStreak);

            // The value sphere_offset now gives a *relative* offset in the composite array; i.e.,
            // sphere_data.DEs_in_SD_composite. Get the SD component

            // Produce the offsets for this streak of spheres with identical SD ids
            for (unsigned int sphereInStreak = 0; sphereInStreak < winningStreak; sphereInStreak++) {
                // set the SD offset
                SD_composite_offsets[idInShared + sphereInStreak] = touchedSD;
                // add this sphere to that sd, incrementing offset for next guy
                sphere_composite_offsets[idInShared + sphereInStreak] = sphere_offset++;
            }
        }
    }

    __syncthreads();  // needed since we write to shared memory above; i.e., offsetInComposite_SphInSD_Array

    const uint64_t max_composite_index =
        (uint64_t)gran_params->nSDs_Y * gran_params->nSDs_X * gran_params->nSDs_Z * MAX_COUNT_OF_DEs_PER_SD;

    // Write out the data now; reister with sphere_data.DEs_in_SD_composite each sphere that touches a certain ID
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_SPHERE; i++) {
        // Add offsets together
        // bad SD means not a valid offset
        if (SD_composite_offsets[MAX_SDs_TOUCHED_BY_SPHERE * threadIdx.x + i] != NULL_GRANULAR_ID) {
            uint64_t offset =
                (uint64_t)SD_composite_offsets[MAX_SDs_TOUCHED_BY_SPHERE * threadIdx.x + i] * MAX_COUNT_OF_DEs_PER_SD +
                sphere_composite_offsets[MAX_SDs_TOUCHED_BY_SPHERE * threadIdx.x + i];
            if (offset >= max_composite_index) {
                ABORTABORTABORT(
                    "overrun during priming on thread %u block %u, offset is %zu, max is %zu,  sphere is %u\n",
                    threadIdx.x, blockIdx.x, offset, max_composite_index, sphIDs[i]);
            } else {
                sphere_data.DEs_in_SD_composite[offset] = sphIDs[i];
            }
        }
    }
}

/// Compute forces on a sphere created by various
inline __device__ void applyBCForces(const int sphXpos,    //!< Global X position of DE
                                     const int sphYpos,    //!< Global Y position of DE
                                     const int sphZpos,    //!< Global Z position of DE
                                     const float sphXvel,  //!< Global X velocity of DE
                                     const float sphYvel,  //!< Global Y velocity of DE
                                     const float sphZvel,  //!< Global Z velocity of DE
                                     float& X_force,       //!< Force in Xdir
                                     float& Y_force,       //!< Force in Ydir
                                     float& Z_force,       //!< Force in Zdir
                                     ParamsPtr gran_params,
                                     BC_type* bc_type_list,
                                     BC_params_t* bc_params_list,
                                     unsigned int nBCs) {
    // store forces on body
    float xforce = 0;
    float yforce = 0;
    float zforce = 0;
    for (unsigned int i = 0; i < nBCs; i++) {
        switch (bc_type_list[i]) {
            case BC_type::AA_BOX: {
                addBCForces_AABox(sphXpos, sphYpos, sphZpos, sphXvel, sphYvel, sphZvel, xforce, yforce, zforce,
                                  gran_params, bc_params_list[i]);
            }
            case BC_type::SPHERE: {
                addBCForces_Sphere(sphXpos, sphYpos, sphZpos, sphXvel, sphYvel, sphZvel, xforce, yforce, zforce,
                                   gran_params, bc_params_list[i]);
            }
            case BC_type::CONE: {
                addBCForces_Cone(sphXpos, sphYpos, sphZpos, sphXvel, sphYvel, sphZvel, xforce, yforce, zforce,
                                 gran_params, bc_params_list[i]);
            }
        }
    }
    // perform on-the-fly integration for the moment
    X_force += xforce;
    Y_force += yforce;
    Z_force += zforce;
}

/**
This device function computes the forces induces by the walls on the box on a sphere
Input:
  - sphXpos: X location, measured in the box reference system, of the sphere
  - sphYpos: Y location, measured in the box reference system, of the sphere
  - sphZpos: Z location, measured in the box reference system, of the sphere
  - sphXvel: X velocity, measured in the box reference system, of the sphere
  - sphYvel: Y velocity, measured in the box reference system, of the sphere
  - sphZvel: Z velocity, measured in the box reference system, of the sphere

Output:
  - X_Vel_corr: the X component of the force, as represented in the box reference system
  - Y_Vel_corr: the Y component of the force, as represented in the box reference system
  - Z_Vel_corr: the Z component of the force, as represented in the box reference system
*/
inline __device__ void boxWallsEffects(const int sphXpos,    //!< Global X position of DE
                                       const int sphYpos,    //!< Global Y position of DE
                                       const int sphZpos,    //!< Global Z position of DE
                                       const float sphXvel,  //!< Global X velocity of DE
                                       const float sphYvel,  //!< Global Y velocity of DE
                                       const float sphZvel,  //!< Global Z velocity of DE
                                       float& Xforce,        //!< Force in Xdir
                                       float& Yforce,        //!< Force in Ydir
                                       float& Zforce,        //!< Force in Zdir
                                       ParamsPtr gran_params) {
    // classic radius grab, but signed so we can negate it easier
    const signed int sphereRadius_SU = gran_params->sphereRadius_SU;

    // Shift frame so that origin is at lower left corner
    int sphXpos_modified = -gran_params->BD_frame_X + sphXpos;
    int sphYpos_modified = -gran_params->BD_frame_Y + sphYpos;
    int sphZpos_modified = -gran_params->BD_frame_Z + sphZpos;

    // Are we touching wall?
    int touchingWall = 0;

    constexpr float sphere_mass_SU = 1.f;
    constexpr float m_eff = sphere_mass_SU / 2.f;
    // cache force
    float3 wall_force = {0, 0, 0};

    float3 delta_V = make_float3(sphXvel - gran_params->BD_frame_X_dot, sphYvel - gran_params->BD_frame_Y_dot,
                                 sphZvel - gran_params->BD_frame_Z_dot);
    // Do x direction
    // penetration of sphere into bottom x wall
    signed int xpen = sphXpos_modified - (signed int)sphereRadius_SU;
    // if sphere center is within one radius of wall
    touchingWall = (xpen < 0) && abs(xpen) < sphereRadius_SU;
    // in this case, xpen is negative and we want a positive restorative force=
    wall_force.x += touchingWall * (gran_params->Kn_s2w_SU * abs(xpen));
    wall_force.x += -1 * touchingWall * (delta_V.x) * gran_params->Gamma_n_s2w_SU * m_eff;
    // add friction on each wall
    // wall_force.y += -1 * touchingWall *
    //                 (gran_params->mu_t_s2w_SU * delta_V.y * gran_params->alpha_h_bar +
    //                  gran_params->Gamma_t_s2w_SU * m_eff * delta_V.y);
    //
    // wall_force.z += -1 * touchingWall *
    //                 (gran_params->mu_t_s2w_SU * delta_V.z * gran_params->alpha_h_bar +
    //                  gran_params->Gamma_t_s2w_SU * m_eff * delta_V.z);

    // Do top X wall
    xpen = MAX_X_POS_UNSIGNED - (sphXpos_modified + (signed int)sphereRadius_SU);
    touchingWall = (xpen < 0) && abs(xpen) < sphereRadius_SU;
    // in this case, xpen is positive and we want a negative restorative force
    wall_force.x += -1 * touchingWall * gran_params->Kn_s2w_SU * abs(xpen);
    wall_force.x += -1 * touchingWall * (delta_V.x) * gran_params->Gamma_n_s2w_SU * m_eff;
    // add friction on each wall
    // wall_force.y += -1 * touchingWall *
    //                 (gran_params->mu_t_s2w_SU * delta_V.y * gran_params->alpha_h_bar +
    //                  gran_params->Gamma_t_s2w_SU * m_eff * delta_V.y);
    //
    // wall_force.z += -1 * touchingWall *
    //                 (gran_params->mu_t_s2w_SU * delta_V.z * gran_params->alpha_h_bar +
    //                  gran_params->Gamma_t_s2w_SU * m_eff * delta_V.z);

    // do again for y, z
    signed int ypen = sphYpos_modified - (signed int)sphereRadius_SU;
    // true if sphere touching wall
    touchingWall = (ypen < 0) && abs(ypen) < sphereRadius_SU;
    // in this case, ypen is negative and we want a positive restorative force
    wall_force.y += touchingWall * (gran_params->Kn_s2w_SU * abs(ypen));
    wall_force.y += -1 * touchingWall * (delta_V.y) * gran_params->Gamma_n_s2w_SU * m_eff;
    // add friction on each wall
    // wall_force.x += -1 * touchingWall *
    //                 (gran_params->mu_t_s2w_SU * delta_V.x * gran_params->alpha_h_bar +
    //                  gran_params->Gamma_t_s2w_SU * m_eff * delta_V.x);
    //
    // wall_force.z += -1 * touchingWall *
    //                 (gran_params->mu_t_s2w_SU * delta_V.z * gran_params->alpha_h_bar +
    //                  gran_params->Gamma_t_s2w_SU * m_eff * delta_V.z);

    // Do top y wall
    ypen = MAX_Y_POS_UNSIGNED - (sphYpos_modified + (signed int)sphereRadius_SU);
    touchingWall = (ypen < 0) && abs(ypen) < sphereRadius_SU;
    // in this case, ypen is positive and we want a negative restorative force
    wall_force.y += -1 * touchingWall * gran_params->Kn_s2w_SU * abs(ypen);
    wall_force.y += -1 * touchingWall * (delta_V.y) * gran_params->Gamma_n_s2w_SU * m_eff;
    // add friction on each wall
    // wall_force.x += -1 * touchingWall *
    //                 (gran_params->mu_t_s2w_SU * delta_V.x * gran_params->alpha_h_bar +
    //                  gran_params->Gamma_t_s2w_SU * m_eff * delta_V.x);
    //
    // wall_force.z += -1 * touchingWall *
    //                 (gran_params->mu_t_s2w_SU * delta_V.z * gran_params->alpha_h_bar +
    //                  gran_params->Gamma_t_s2w_SU * m_eff * delta_V.z);

    // penetration of sphere into relevant wall
    signed int zpen = sphZpos_modified - (signed int)sphereRadius_SU;
    // true if sphere touching wall
    touchingWall = (zpen < 0) && abs(zpen) < sphereRadius_SU;
    // in this case, zpen is negative and we want a positive restorative force
    wall_force.z += touchingWall * (gran_params->Kn_s2w_SU * abs(zpen));
    wall_force.z += -1 * touchingWall * (delta_V.z) * gran_params->Gamma_n_s2w_SU * m_eff;
    // add friction on each wall
    // wall_force.x += -1 * touchingWall *
    //                 (gran_params->mu_t_s2w_SU * delta_V.x * gran_params->alpha_h_bar +
    //                  gran_params->Gamma_t_s2w_SU * m_eff * delta_V.x);
    //
    // wall_force.y += -1 * touchingWall *
    //                 (gran_params->mu_t_s2w_SU * delta_V.y * gran_params->alpha_h_bar +
    //                  gran_params->Gamma_t_s2w_SU * m_eff * delta_V.y);

    // Do top z wall
    zpen = MAX_Z_POS_UNSIGNED - (sphZpos_modified + (signed int)sphereRadius_SU);
    touchingWall = (zpen < 0) && abs(zpen) < sphereRadius_SU;
    // in this case, zpen is positive and we want a negative restorative force
    wall_force.z += -1 * touchingWall * gran_params->Kn_s2w_SU * abs(zpen);
    wall_force.z += -1 * touchingWall * (delta_V.z) * gran_params->Gamma_n_s2w_SU * m_eff;
    // add friction on each wall
    // wall_force.x += -1 * touchingWall *
    //                 (gran_params->mu_t_s2w_SU * delta_V.x * gran_params->alpha_h_bar +
    //                  gran_params->Gamma_t_s2w_SU * m_eff * delta_V.x);
    //
    // wall_force.y += -1 * touchingWall *
    //                 (gran_params->mu_t_s2w_SU * delta_V.y * gran_params->alpha_h_bar +
    //                  gran_params->Gamma_t_s2w_SU * m_eff * delta_V.y);

    // write back to "return" values
    Xforce += wall_force.x;
    Yforce += wall_force.y;
    Zforce += wall_force.z;
}

/**
This kernel call figures out forces cuased by sphere-sphere interactions

Template arguments:
  - MAX_NSPHERES_PER_SD: the number of threads used in this kernel, comes into play when invoking CUB block
collectives.

Assumptions:
    - A sphere cannot touch more than 12 other spheres (this is a proved results, of 1953).
    - A "char" is stored on 8 bits and as such an unsigned char can hold positive numbers up to and including 255
    - MAX_NSPHERES_PER_SD<256 (we are using in this kernel unsigned char to store IDs)
    - Granular material is made up of monodisperse spheres.
    - The function below assumes the spheres are in a box
    - The box has dimensions L x D x H.
    - The reference frame associated with the box:
      - The x-axis is along the length L of the box
      - The y-axis is along the width D of the box
      - The z-axis is along the height H of the box

NOTE:
  - Upon calling it with more than 256 threads, this kernel is going to make illegal memory accesses
*/
template <unsigned int MAX_NSPHERES_PER_SD>  //!< Number of CUB threads engaged in block-collective CUB operations.
                                             //!< Should be a multiple of 32
__global__ void computeSphereForces(sphereDataStruct sphere_data,
                                    ParamsPtr gran_params,
                                    BC_type* bc_type_list,
                                    BC_params_t* bc_params_list,
                                    unsigned int nBCs) {
    // still moar radius grab
    const unsigned int sphereRadius_SU = gran_params->sphereRadius_SU;

    // Cache positions and velocities in shared memory, this doesn't hurt occupancy at the moment
    __shared__ int sphere_X[MAX_NSPHERES_PER_SD];
    __shared__ int sphere_Y[MAX_NSPHERES_PER_SD];
    __shared__ int sphere_Z[MAX_NSPHERES_PER_SD];
    __shared__ float omega_X[MAX_NSPHERES_PER_SD];
    __shared__ float omega_Y[MAX_NSPHERES_PER_SD];
    __shared__ float omega_Z[MAX_NSPHERES_PER_SD];
    __shared__ float sphere_X_DOT[MAX_NSPHERES_PER_SD];
    __shared__ float sphere_Y_DOT[MAX_NSPHERES_PER_SD];
    __shared__ float sphere_Z_DOT[MAX_NSPHERES_PER_SD];

    unsigned int thisSD = blockIdx.x;
    unsigned int spheresTouchingThisSD = sphere_data.SD_NumOf_DEs_Touching[thisSD];
    unsigned mySphereID;
    unsigned char bodyB_list[MAX_SPHERES_TOUCHED_BY_SPHERE];
    unsigned int ncontacts = 0;
    if (spheresTouchingThisSD == 0) {
        return;  // no spheres here, move along
    }

    // If we overran, we have a major issue, time to crash before we make illegal memory accesses
    if (threadIdx.x == 0 && spheresTouchingThisSD > MAX_NSPHERES_PER_SD) {
        // Crash now
        ABORTABORTABORT("TOO MANY SPHERES! SD %u has %u spheres\n", thisSD, spheresTouchingThisSD);
    }

    // Bring in data from global into shmem. Only a subset of threads get to do this.
    // Note that we're not using shared memory very heavily, so our bandwidth is pretty low
    if (threadIdx.x < spheresTouchingThisSD) {
        // We need int64_ts to index into composite array
        uint64_t offset_in_composite_Array = ((uint64_t)thisSD) * MAX_NSPHERES_PER_SD + threadIdx.x;
        mySphereID = sphere_data.DEs_in_SD_composite[offset_in_composite_Array];
        sphere_X[threadIdx.x] = sphere_data.pos_X[mySphereID];
        sphere_Y[threadIdx.x] = sphere_data.pos_Y[mySphereID];
        sphere_Z[threadIdx.x] = sphere_data.pos_Z[mySphereID];
        omega_X[threadIdx.x] = sphere_data.omega_X[mySphereID];
        omega_Y[threadIdx.x] = sphere_data.omega_Y[mySphereID];
        omega_Z[threadIdx.x] = sphere_data.omega_Z[mySphereID];
        sphere_X_DOT[threadIdx.x] = sphere_data.pos_X_dt[mySphereID];
        sphere_Y_DOT[threadIdx.x] = sphere_data.pos_Y_dt[mySphereID];
        sphere_Z_DOT[threadIdx.x] = sphere_data.pos_Z_dt[mySphereID];
    }

    __syncthreads();  // Needed to make sure data gets in shmem before using it elsewhere

    // Assumes each thread is a body, not the greatest assumption but we can fix that later
    // Note that if we have more threads than bodies, some effort gets wasted.
    unsigned int bodyA = threadIdx.x;

    // Each body looks at each other body and computes the force that the other body exerts on it
    if (bodyA < spheresTouchingThisSD) {
        // Force generated by this contact
        float3 bodyA_force = {0.f, 0.f, 0.f};
        float3 bodyA_torque = {0.f, 0.f, 0.f};

        float scalingFactor = gran_params->Kn_s2s_SU;
        float sphdiameter = 2. * sphereRadius_SU;
        double invSphDiameter = 1. / (2. * sphereRadius_SU);
        unsigned int ownerSD =
            SDTripletID(pointSDTriplet(sphere_X[bodyA], sphere_Y[bodyA], sphere_Z[bodyA], gran_params), gran_params);
        for (unsigned char bodyB = 0; bodyB < spheresTouchingThisSD; bodyB++) {
            // unsigned int theirSphID = sphere_data.DEs_in_SD_composite[thisSD * MAX_NSPHERES_PER_SD + bodyB];
            // Don't check for collision with self
            if (bodyA == bodyB)
                continue;

            // Compute penetration to check for collision, we can use ints provided the diameter is small enough
            int64_t penetration_int = 0;

            // This avoids computing a square to figure our if collision or not
            int64_t deltaX = (sphere_X[bodyA] - sphere_X[bodyB]);
            int64_t deltaY = (sphere_Y[bodyA] - sphere_Y[bodyB]);
            int64_t deltaZ = (sphere_Z[bodyA] - sphere_Z[bodyB]);

            penetration_int = deltaX * deltaX;
            penetration_int += deltaY * deltaY;
            penetration_int += deltaZ * deltaZ;

            // Here we need to check if the contact point is in this SD.

            // Take spatial average of positions to get position of contact point
            // NOTE that we *do* want integer division since the SD-checking code uses ints anyways. Computing this as
            // an int is *much* faster than float, much less double, on Conlain's machine
            int contactX = (sphere_X[bodyB] + sphere_X[bodyA]) / 2;
            int contactY = (sphere_Y[bodyB] + sphere_Y[bodyA]) / 2;
            int contactZ = (sphere_Z[bodyB] + sphere_Z[bodyA]) / 2;

            // We need to make sure we don't count a collision twice -- if a contact pair is in multiple SDs, count it
            // only in the one that holds the contact point
            unsigned int contactSD =
                SDTripletID(pointSDTriplet(contactX, contactY, contactZ, gran_params), gran_params);

            const int64_t contact_threshold = 4l * sphereRadius_SU * sphereRadius_SU;

            // We have a collision here, log it for later
            // not very divergent, super quick
            if (contactSD == thisSD && penetration_int < contact_threshold) {
                bodyB_list[ncontacts] = bodyB;  // Save the collision pair
                ncontacts++;                    // Increment the contact counter
            }
        }

        // NOTE that below here I used double precision because I didn't know how much precision I needed. Reducing the
        // amount of doubles will certainly speed this up
        // Run through and do actual force computations, for these we know each one is a legit collision
        for (unsigned int idx = 0; idx < ncontacts; idx++) {
            // distance between sphere centers divided by sphere diameter

            // unsigned int theirSphID = sphere_data.DEs_in_SD_composite[thisSD * MAX_NSPHERES_PER_SD + bodyB];
            // Don't check for collision with self
            unsigned char bodyB = bodyB_list[idx];

            // NOTE below here, it seems faster to do coalesced shared mem accesses than caching sphere_X[bodyA] and the
            // like in a register
            // This avoids computing a square to figure our if collision or not
            double3 delta_r;  // points from b to a
            delta_r.x = (sphere_X[bodyA] - sphere_X[bodyB]) * invSphDiameter;
            delta_r.y = (sphere_Y[bodyA] - sphere_Y[bodyB]) * invSphDiameter;
            delta_r.z = (sphere_Z[bodyA] - sphere_Z[bodyB]) * invSphDiameter;

            float3 v_rel;

            // Velocity difference, it's better to do a coalesced access here than a fragmented access inside
            v_rel.x = sphere_X_DOT[bodyA] - sphere_X_DOT[bodyB];
            v_rel.y = sphere_Y_DOT[bodyA] - sphere_Y_DOT[bodyB];
            v_rel.z = sphere_Z_DOT[bodyA] - sphere_Z_DOT[bodyB];

            // // distance between sphere centers divided by sphere diameter
            // double d2_normalized = Dot(delta_r, delta_r);

            // Note: this can be accelerated should we decide to go w/ float. Then we can use the CUDA
            // intrinsic:
            // __device__ ​ float rnormf ( int  dim, const float* a)
            // http://docs.nvidia.com/cuda/cuda-math-api/group__CUDA__MATH__SINGLE.html#group__CUDA__MATH__SINGLE

            // Compute penetration term, this becomes the delta as we want it
            // Makes more sense to find rsqrt, the compiler is doing this anyways
            float reciplength = rsqrt(Dot(delta_r, delta_r));
            float penetration = reciplength - 1.;

            // Compute nondim force term

            // Compute force updates for spring term
            float springTermX = scalingFactor * delta_r.x * sphdiameter * penetration;
            float springTermY = scalingFactor * delta_r.y * sphdiameter * penetration;
            float springTermZ = scalingFactor * delta_r.z * sphdiameter * penetration;

            // Compute force updates for damping term
            // Project relative velocity to the normal
            // n = delta_r * reciplength
            // proj = Dot(delta_dot, n)
            float projection = Dot(v_rel, delta_r) * reciplength;

            // delta_dot = proj * n
            float3 vrel_n = projection * make_float3(delta_r.x, delta_r.y, delta_r.z) * reciplength;

            // remove normal component, add back rotational components
            v_rel = v_rel - vrel_n +
                    Cross(make_float3(delta_r.x, delta_r.y, delta_r.z),
                          make_float3(omega_X[bodyA] - omega_X[bodyB], omega_Y[bodyA] - omega_Y[bodyB],
                                      omega_Z[bodyA] - omega_Z[bodyB]));

            // TODO improve this
            // one-step tangential displacement
            float3 ut = v_rel * gran_params->alpha_h_bar;

            constexpr float sphere_mass_SU = 1.f;
            constexpr float m_eff = sphere_mass_SU / 2.f;

            float3 friction_term = -gran_params->mu_t_s2s_SU * ut - gran_params->Gamma_t_s2s_SU * m_eff * v_rel;
            // printf("friction_term is (%f, %f, %f), ut is (%f, %f, %f), mut is %f\n", friction_term.x,
            // friction_term.y,
            //        friction_term.z, ut.x, ut.y, ut.z, gran_params->mu_t_s2s_SU);

            float dampingTermX = -gran_params->Gamma_n_s2s_SU * vrel_n.x * m_eff;
            float dampingTermY = -gran_params->Gamma_n_s2s_SU * vrel_n.y * m_eff;
            float dampingTermZ = -gran_params->Gamma_n_s2s_SU * vrel_n.z * m_eff;

            float cohesionConstant = sphere_mass_SU * gran_params->gravMag_SU * gran_params->cohesion_ratio;

            // Compute force updates for cohesion term, is opposite the spring term
            float cohesionTermX = -cohesionConstant * delta_r.x * reciplength;
            float cohesionTermY = -cohesionConstant * delta_r.y * reciplength;
            float cohesionTermZ = -cohesionConstant * delta_r.z * reciplength;

            // Add damping term to spring term, write back to counter
            bodyA_force.x += springTermX + dampingTermX + cohesionTermX;
            bodyA_force.y += springTermY + dampingTermY + cohesionTermY;
            bodyA_force.z += springTermZ + dampingTermZ + cohesionTermZ;

            // compute torque on body
            // multiply by diameter, but normalize by radius to keep numbers sane
            bodyA_torque = bodyA_torque + 2 * Cross(make_float3(delta_r.x, delta_r.y, delta_r.z), bodyA_force);

            // Now add friction, the cross should have removed it, but there's no need to add it in yet
            bodyA_force = bodyA_force + friction_term;
        }

        // IMPORTANT: Make sure that the sphere belongs to *this* SD, otherwise we'll end up with double counting
        // this force.
        // If this SD owns the body, add its wall forces and grav forces. This should be pretty non-divergent since
        // most spheres won't be on the border
        if (ownerSD == thisSD) {
            // Perhaps this sphere is hitting the wall[s]
            boxWallsEffects(sphere_X[bodyA], sphere_Y[bodyA], sphere_Z[bodyA], sphere_X_DOT[bodyA], sphere_Y_DOT[bodyA],
                            sphere_Z_DOT[bodyA], bodyA_force.x, bodyA_force.y, bodyA_force.z, gran_params);

            applyBCForces(sphere_X[bodyA], sphere_Y[bodyA], sphere_Z[bodyA], sphere_X_DOT[bodyA], sphere_Y_DOT[bodyA],
                          sphere_Z_DOT[bodyA], bodyA_force.x, bodyA_force.y, bodyA_force.z, gran_params, bc_type_list,
                          bc_params_list, nBCs);
            // If the sphere belongs to this SD, add up the gravitational force component.
            bodyA_force.x += gran_params->gravAcc_X_SU;
            bodyA_force.y += gran_params->gravAcc_Y_SU;
            bodyA_force.z += gran_params->gravAcc_Z_SU;

            // gravity exerts no force
        }

        // Write the force back to global memory so that we can apply them AFTER this kernel finishes
        atomicAdd(sphere_data.sphere_force_X + mySphereID, bodyA_force.x);
        atomicAdd(sphere_data.sphere_force_Y + mySphereID, bodyA_force.y);
        atomicAdd(sphere_data.sphere_force_Z + mySphereID, bodyA_force.z);

        // write back torques for later
        atomicAdd(sphere_data.sphere_torque_X + mySphereID, bodyA_torque.x);
        atomicAdd(sphere_data.sphere_torque_Y + mySphereID, bodyA_torque.y);
        atomicAdd(sphere_data.sphere_torque_Z + mySphereID, bodyA_torque.z);
    }
    __syncthreads();
}

/**
 * Numerically integrates force to velocity and velocity to position, then computes the broadphase on the new positions
 */
template <unsigned int CUB_THREADS>  //!< Number of CUB threads engaged in block-collective CUB operations.
                                     //!< Should be a multiple of 32
__global__ void updatePositions(const float alpha_h_bar,  //!< The numerical integration time step
                                sphereDataStruct sphere_data,
                                unsigned int nSpheres,
                                ParamsPtr gran_params,
                                chrono::granular::GRAN_TIME_INTEGRATOR integrator) {
    int xSphCenter;
    int ySphCenter;
    int zSphCenter;
    /// Set aside shared memory
    // SD component of offset into composite array
    volatile __shared__ unsigned int SD_composite_offsets[CUB_THREADS * MAX_SDs_TOUCHED_BY_SPHERE];
    // sphere component of offset into composite array
    volatile __shared__ unsigned char sphere_composite_offsets[CUB_THREADS * MAX_SDs_TOUCHED_BY_SPHERE];
    volatile __shared__ bool shMem_head_flags[CUB_THREADS * MAX_SDs_TOUCHED_BY_SPHERE];

    typedef cub::BlockRadixSort<unsigned int, CUB_THREADS, MAX_SDs_TOUCHED_BY_SPHERE, unsigned int> BlockRadixSortOP;
    __shared__ typename BlockRadixSortOP::TempStorage temp_storage_sort;

    typedef cub::BlockDiscontinuity<unsigned int, CUB_THREADS> Block_Discontinuity;
    __shared__ typename Block_Discontinuity::TempStorage temp_storage_disc;

    // Figure out what sphereID this thread will handle. We work with a 1D block structure and a 1D grid structure
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned int sphIDs[MAX_SDs_TOUCHED_BY_SPHERE] = {mySphereID, mySphereID, mySphereID, mySphereID,
                                                      mySphereID, mySphereID, mySphereID, mySphereID};

    // This uses a lot of registers but is needed
    unsigned int SDsTouched[MAX_SDs_TOUCHED_BY_SPHERE] = {NULL_GRANULAR_ID, NULL_GRANULAR_ID, NULL_GRANULAR_ID,
                                                          NULL_GRANULAR_ID, NULL_GRANULAR_ID, NULL_GRANULAR_ID,
                                                          NULL_GRANULAR_ID, NULL_GRANULAR_ID};

    float old_vel_x = 0;
    float old_vel_y = 0;
    float old_vel_z = 0;
    // Write back velocity updates
    if (mySphereID < nSpheres) {
        // Check to see if we messed up badly somewhere
        if (sphere_data.sphere_force_X[mySphereID] == NAN || sphere_data.sphere_force_Y[mySphereID] == NAN ||
            sphere_data.sphere_force_Z[mySphereID] == NAN) {
            ABORTABORTABORT("NAN force computed -- sphere is %u, x force is %f\n", mySphereID,
                            sphere_data.sphere_force_X[mySphereID]);
        }

        // inertia normalized by radius
        float sphere_inertia = 2.f / 5.f * gran_params->sphereRadius_SU;

        float v_update_x = 0;
        float v_update_y = 0;
        float v_update_z = 0;

        float omega_update_x = 0;
        float omega_update_y = 0;
        float omega_update_z = 0;

        old_vel_x = sphere_data.pos_X_dt[mySphereID];
        old_vel_y = sphere_data.pos_Y_dt[mySphereID];
        old_vel_z = sphere_data.pos_Z_dt[mySphereID];
        // no divergence, same for every thread in block
        switch (integrator) {
            case chrono::granular::GRAN_TIME_INTEGRATOR::FORWARD_EULER: {
                v_update_x = alpha_h_bar * sphere_data.sphere_force_X[mySphereID];
                v_update_y = alpha_h_bar * sphere_data.sphere_force_Y[mySphereID];
                v_update_z = alpha_h_bar * sphere_data.sphere_force_Z[mySphereID];

                // tau = I alpha => alpha = tau / I
                omega_update_x = alpha_h_bar * sphere_data.sphere_torque_X[mySphereID] / sphere_inertia;
                omega_update_y = alpha_h_bar * sphere_data.sphere_torque_Y[mySphereID] / sphere_inertia;
                omega_update_z = alpha_h_bar * sphere_data.sphere_torque_Z[mySphereID] / sphere_inertia;
                break;
            }
            case chrono::granular::GRAN_TIME_INTEGRATOR::CHUNG: {
                float gamma_hat = -.5f;
                float gamma = 3.f / 2.f;
                v_update_x = alpha_h_bar * (sphere_data.sphere_force_X[mySphereID] * gamma +
                                            sphere_data.sphere_force_X_old[mySphereID] * gamma_hat);
                v_update_y = alpha_h_bar * (sphere_data.sphere_force_Y[mySphereID] * gamma +
                                            sphere_data.sphere_force_Y_old[mySphereID] * gamma_hat);
                v_update_z = alpha_h_bar * (sphere_data.sphere_force_Z[mySphereID] * gamma +
                                            sphere_data.sphere_force_Z_old[mySphereID] * gamma_hat);
                break;
            }
        }

        // Probably does not need to be atomic, but no conflicts means it won't be too slow anyways
        atomicAdd(sphere_data.pos_X_dt + mySphereID, v_update_x);
        atomicAdd(sphere_data.pos_Y_dt + mySphereID, v_update_y);
        atomicAdd(sphere_data.pos_Z_dt + mySphereID, v_update_z);

        // printf("tau is (%f, %f, %f), domega is (%f, %f, %f)\n", sphere_data.sphere_torque_X[mySphereID],
        //        sphere_data.sphere_torque_Y[mySphereID], sphere_data.sphere_torque_Z[mySphereID], omega_update_x,
        //        omega_update_y, omega_update_z);

        atomicAdd(sphere_data.omega_X + mySphereID, omega_update_x);
        atomicAdd(sphere_data.omega_Y + mySphereID, omega_update_y);
        atomicAdd(sphere_data.omega_Z + mySphereID, omega_update_z);
    }
    // wait for everyone to finish
    __syncthreads();
    if (mySphereID < nSpheres) {
        float position_update_x = 0;
        float position_update_y = 0;
        float position_update_z = 0;
        // no divergence, same for every thread in block
        switch (integrator) {
            case chrono::granular::GRAN_TIME_INTEGRATOR::FORWARD_EULER: {
                position_update_x = alpha_h_bar * sphere_data.pos_X_dt[mySphereID];
                position_update_y = alpha_h_bar * sphere_data.pos_Y_dt[mySphereID];
                position_update_z = alpha_h_bar * sphere_data.pos_Z_dt[mySphereID];
                break;
            }
            case chrono::granular::GRAN_TIME_INTEGRATOR::CHUNG: {
                float beta = 28.f / 27.f;
                float beta_hat = .5 - beta;
                position_update_x =
                    alpha_h_bar * (old_vel_x + alpha_h_bar * (sphere_data.sphere_force_X[mySphereID] * beta +
                                                              sphere_data.sphere_force_X_old[mySphereID] * beta_hat));
                position_update_y =
                    alpha_h_bar * (old_vel_y + alpha_h_bar * (sphere_data.sphere_force_Y[mySphereID] * beta +
                                                              sphere_data.sphere_force_Y_old[mySphereID] * beta_hat));
                position_update_z =
                    alpha_h_bar * (old_vel_z + alpha_h_bar * (sphere_data.sphere_force_Z[mySphereID] * beta +
                                                              sphere_data.sphere_force_Z_old[mySphereID] * beta_hat));
                break;
            }
        }
        // Perform numerical integration. For now, use Explicit Euler. Hitting cache, also coalesced.
        xSphCenter = position_update_x;
        ySphCenter = position_update_y;
        zSphCenter = position_update_z;

        xSphCenter += sphere_data.pos_X[mySphereID];
        sphere_data.pos_X[mySphereID] = xSphCenter;

        ySphCenter += sphere_data.pos_Y[mySphereID];
        sphere_data.pos_Y[mySphereID] = ySphCenter;

        zSphCenter += sphere_data.pos_Z[mySphereID];
        sphere_data.pos_Z[mySphereID] = zSphCenter;

        figureOutTouchedSD(xSphCenter, ySphCenter, zSphCenter, SDsTouched, gran_params);
    }

    __syncthreads();

    // Sort by the ID of the SD touched
    BlockRadixSortOP(temp_storage_sort).Sort(SDsTouched, sphIDs);
    __syncthreads();

    // Do a winningStreak search on whole block, might not have high utilization here
    bool head_flags[MAX_SDs_TOUCHED_BY_SPHERE];
    Block_Discontinuity(temp_storage_disc).FlagHeads(head_flags, SDsTouched, cub::Inequality());
    __syncthreads();

    // Write back to shared memory; eight-way bank conflicts here - to revisit later
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_SPHERE; i++) {
        shMem_head_flags[MAX_SDs_TOUCHED_BY_SPHERE * threadIdx.x + i] = head_flags[i];
    }

    // Seed offsetInComposite_SphInSD_Array with "no valid ID" so that we know later on what is legit;
    // No shmem bank coflicts here, good access...
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_SPHERE; i++) {
        SD_composite_offsets[i * CUB_THREADS + threadIdx.x] = NULL_GRANULAR_ID;
        sphere_composite_offsets[i * CUB_THREADS + threadIdx.x] = 0;
    }

    __syncthreads();

    // Count how many times an SD shows up in conjunction with the collection of CUB_THREADS spheres. There
    // will be some thread divergence here.
    // Loop through each potential SD, after sorting, and see if it is the start of a head
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_SPHERE; i++) {
        // SD currently touched, could easily be inlined
        unsigned int touchedSD = SDsTouched[i];
        if (touchedSD != NULL_GRANULAR_ID && head_flags[i]) {
            // current index into shared datastructure of length MAX_SDs_TOUCHED_BY_SPHERE*CUB_THREADS, could easily be
            // inlined
            unsigned int idInShared = MAX_SDs_TOUCHED_BY_SPHERE * threadIdx.x + i;
            unsigned int winningStreak = 0;
            // This is the beginning of a sequence of SDs with a new ID
            do {
                winningStreak++;
                // Go until we run out of threads on the warp or until we find a new head
            } while (idInShared + winningStreak < MAX_SDs_TOUCHED_BY_SPHERE * CUB_THREADS &&
                     !(shMem_head_flags[idInShared + winningStreak]));

            if (touchedSD >= gran_params->nSDs_X * gran_params->nSDs_Y * gran_params->nSDs_Z) {
                printf("invalid SD index %u on thread %u\n", mySphereID, touchedSD);
            }

            // Store start of new entries
            unsigned char sphere_offset = atomicAdd(sphere_data.SD_NumOf_DEs_Touching + touchedSD, winningStreak);

            // The value sphere_offset now gives a *relative* offset in the composite array; i.e.,
            // sphere_data.DEs_in_SD_composite. Get the SD component

            // Produce the offsets for this streak of spheres with identical SD ids
            for (unsigned int sphereInStreak = 0; sphereInStreak < winningStreak; sphereInStreak++) {
                // set the SD offset
                SD_composite_offsets[idInShared + sphereInStreak] = touchedSD;
                // add this sphere to that sd, incrementing offset for next guy
                sphere_composite_offsets[idInShared + sphereInStreak] = sphere_offset++;
            }
        }
    }

    __syncthreads();  // needed since we write to shared memory above; i.e., offsetInComposite_SphInSD_Array

    const uint64_t max_composite_index =
        (uint64_t)gran_params->nSDs_Y * gran_params->nSDs_X * gran_params->nSDs_Z * MAX_COUNT_OF_DEs_PER_SD;

    // Write out the data now; reister with sphere_data.DEs_in_SD_composite each sphere that touches a certain ID
    for (unsigned int i = 0; i < MAX_SDs_TOUCHED_BY_SPHERE; i++) {
        // Add offsets together
        // bad SD means not a valid offset
        if (SD_composite_offsets[MAX_SDs_TOUCHED_BY_SPHERE * threadIdx.x + i] != NULL_GRANULAR_ID) {
            uint64_t offset =
                (uint64_t)SD_composite_offsets[MAX_SDs_TOUCHED_BY_SPHERE * threadIdx.x + i] * MAX_COUNT_OF_DEs_PER_SD +
                sphere_composite_offsets[MAX_SDs_TOUCHED_BY_SPHERE * threadIdx.x + i];
            if (offset >= max_composite_index) {
                ABORTABORTABORT(
                    "overrun during priming on thread %u block %u, offset is %zu, max is %zu,  sphere is %u\n",
                    threadIdx.x, blockIdx.x, offset, max_composite_index, sphIDs[i]);
            } else {
                sphere_data.DEs_in_SD_composite[offset] = sphIDs[i];
            }
        }
    }
}
