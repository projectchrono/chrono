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
// Authors: Dan Negrut, Conlain Kelly
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

// These are the max X, Y, Z dimensions in the BD frame
#define MAX_X_POS_UNSIGNED (gran_params->SD_size_X_SU * gran_params->nSDs_X)
#define MAX_Y_POS_UNSIGNED (gran_params->SD_size_Y_SU * gran_params->nSDs_Y)
#define MAX_Z_POS_UNSIGNED (gran_params->SD_size_Z_SU * gran_params->nSDs_Z)

// Do two things: make the naming nicer and require a cosnt pointer everywhere
typedef const chrono::granular::ChSystemGranular::GranParamsHolder* ParamsPtr;

// Decide which SD owns this point in space
// Pass it the Center of Mass location for a DE to get its owner, also used to get contact point
inline __device__ uint3 pointSDTriplet(int sphCenter_X, int sphCenter_Y, int sphCenter_Z, ParamsPtr gran_params) {
    // Note that this offset allows us to have moving walls and the like very easily

    int64_t sphCenter_X_modified = -gran_params->BD_frame_X + sphCenter_X;
    int64_t sphCenter_Y_modified = -gran_params->BD_frame_Y + sphCenter_Y;
    int64_t sphCenter_Z_modified = -gran_params->BD_frame_Z + sphCenter_Z;
    uint3 n;
    // Get the SD of the sphere's center in the xdir
    n.x = (sphCenter_X_modified) / gran_params->SD_size_X_SU;
    // Same for D and H
    n.y = (sphCenter_Y_modified) / gran_params->SD_size_Y_SU;
    n.z = (sphCenter_Z_modified) / gran_params->SD_size_Z_SU;
    return n;
}

// Convert triplet to single int SD ID
inline __device__ unsigned int SDTripletID(const unsigned int i,
                                           const unsigned int j,
                                           const unsigned int k,
                                           ParamsPtr gran_params) {
    return i * gran_params->nSDs_Y * gran_params->nSDs_Z + j * gran_params->nSDs_Z + k;
}

// Convert triplet to single int SD ID
inline __device__ unsigned int SDTripletID(const uint3& trip, ParamsPtr gran_params) {
    return SDTripletID(trip.x, trip.y, trip.z, gran_params);
}

// Convert triplet to single int SD ID
inline __device__ unsigned int SDTripletID(const unsigned int trip[3], ParamsPtr gran_params) {
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
template <
    unsigned int
        CUB_THREADS>  //!< Number of CUB threads engaged in block-collective CUB operations. Should be a multiple of 32
__global__ void
primingOperationsRectangularBox(
    int* d_sphere_pos_X,                       //!< Pointer to array containing data related to the spheres in the box
    int* d_sphere_pos_Y,                       //!< Pointer to array containing data related to the spheres in the box
    int* d_sphere_pos_Z,                       //!< Pointer to array containing data related to the spheres in the box
    unsigned int* SD_countsOfSpheresTouching,  //!< The array that for each SD indicates how many spheres touch this SD
    unsigned int* spheres_in_SD_composite,     //!< Big array that works in conjunction with SD_countsOfSpheresTouching.
                                               //!< "spheres_in_SD_composite" says which SD contains what spheres
    unsigned int nSpheres,                     //!< Number of spheres in the box
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
        xSphCenter = d_sphere_pos_X[mySphereID];
        ySphCenter = d_sphere_pos_Y[mySphereID];
        zSphCenter = d_sphere_pos_Z[mySphereID];

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
            unsigned char sphere_offset = atomicAdd(SD_countsOfSpheresTouching + touchedSD, winningStreak);

            // The value sphere_offset now gives a *relative* offset in the composite array; i.e.,
            // spheres_in_SD_composite. Get the SD component

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

    // Write out the data now; reister with spheres_in_SD_composite each sphere that touches a certain ID
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
                spheres_in_SD_composite[offset] = sphIDs[i];
            }
        }
    }
}

/**
This device function computes the forces induces by the walls on the box on a sphere
Input:
  - alpha_h_bar: the size of the time step
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
inline __device__ void boxWallsEffects(const float alpha_h_bar,  //!< Integration step size.
                                       const int sphXpos,        //!< Global X position of DE
                                       const int sphYpos,        //!< Global Y position of DE
                                       const int sphZpos,        //!< Global Z position of DE
                                       const float sphXvel,      //!< Global X velocity of DE
                                       const float sphYvel,      //!< Global Y velocity of DE
                                       const float sphZvel,      //!< Global Z velocity of DE
                                       float& X_Vel_corr,        //!< Velocity correction in Xdir
                                       float& Y_Vel_corr,        //!< Velocity correction in Xdir
                                       float& Z_Vel_corr,        //!< Velocity correction in Xdir
                                       ParamsPtr gran_params) {
    // classic radius grab
    const unsigned int sphereRadius_SU = gran_params->sphereRadius_SU;

    // Shift frame so that origin is at lower left corner
    int sphXpos_modified = -gran_params->BD_frame_X + sphXpos;
    int sphYpos_modified = -gran_params->BD_frame_Y + sphYpos;
    int sphZpos_modified = -gran_params->BD_frame_Z + sphZpos;

    // penetration into wall
    signed int pen = 0;
    // Are we touching wall?
    int touchingWall = 0;
    // Compute spring factor in force
    float scalingFactor = (1.0f * alpha_h_bar) * gran_params->Kn_s2w_SU;

    // tmp vars to save writes, probably not necessary
    float xcomp = 0;
    float ycomp = 0;
    float zcomp = 0;

    // Do x direction
    // penetration of sphere into bottom x wall
    pen = sphXpos_modified - (signed int)sphereRadius_SU;
    // true if sphere touching wall
    touchingWall = (pen < 0) && abs(pen) < sphereRadius_SU;
    // in this case, pen is negative and we want a positive restorative force
    // Need to get relative velocity
    xcomp += touchingWall * (scalingFactor * abs(pen));

    // Do top X wall
    pen = MAX_X_POS_UNSIGNED - (sphXpos_modified + (signed int)sphereRadius_SU);
    touchingWall = (pen < 0) && abs(pen) < sphereRadius_SU;
    // in this case, pen is positive and we want a negative restorative force
    xcomp += touchingWall * (-1 * scalingFactor * abs(pen));

    // penetration of sphere into relevant wall
    pen = sphYpos_modified - (signed int)sphereRadius_SU;
    // true if sphere touching wall
    touchingWall = (pen < 0) && abs(pen) < sphereRadius_SU;
    // in this case, pen is negative and we want a positive restorative force
    ycomp += touchingWall * (scalingFactor * abs(pen));

    // Do top y wall
    pen = MAX_Y_POS_UNSIGNED - (sphYpos_modified + (signed int)sphereRadius_SU);
    touchingWall = (pen < 0) && abs(pen) < sphereRadius_SU;
    // in this case, pen is positive and we want a negative restorative force
    ycomp += touchingWall * (-1 * scalingFactor * abs(pen));

    // penetration of sphere into relevant wall
    pen = sphZpos_modified - (signed int)sphereRadius_SU;
    // true if sphere touching wall
    touchingWall = (pen < 0) && abs(pen) < sphereRadius_SU;
    // in this case, pen is negative and we want a positive restorative force
    zcomp += touchingWall * (scalingFactor * abs(pen));

    // Do top z wall
    pen = MAX_Z_POS_UNSIGNED - (sphZpos_modified + (signed int)sphereRadius_SU);
    touchingWall = (pen < 0) && abs(pen) < sphereRadius_SU;
    // in this case, pen is positive and we want a negative restorative force
    zcomp += touchingWall * (-1 * scalingFactor * abs(pen));

    // write back to "return" values
    X_Vel_corr += xcomp;
    Y_Vel_corr += ycomp;
    Z_Vel_corr += zcomp;
}
/**
This kernel applies the velocity updates computed in computeVelocityUpdates to each sphere. This has to be its own
kernel since computeVelocityUpdates would otherwise both read and write to the global velocities, an undesirable effect.
NOTE That this function uses the already-integrated forces, so for forward euler the updates are F * dt
*/
template <unsigned int MAX_NSPHERES_PER_SD>  //!< Number of CUB threads engaged in block-collective CUB operations.
                                             //!< Should be a multiple of 32
__global__ void applyVelocityUpdates(
    const float alpha_h_bar,          //!< Value that controls actual step size, not actually needed for this kernel
    int* d_sphere_pos_X,              //!< Pointer to array containing data related to the
                                      //!< spheres in the box
    int* d_sphere_pos_Y,              //!< Pointer to array containing data related to the
                                      //!< spheres in the box
    int* d_sphere_pos_Z,              //!< Pointer to array containing data related to the
                                      //!< spheres in the box
    float* d_sphere_pos_X_dt_update,  //!< Pointer to array containing data related to
                                      //!< the spheres in the box
    float* d_sphere_pos_Y_dt_update,  //!< Pointer to array containing data related to
                                      //!< the spheres in the box
    float* d_sphere_pos_Z_dt_update,  //!< Pointer to array containing data related to
                                      //!< the spheres in the box
    unsigned int* SD_countsOfSpheresTouching,  //!< The array that for each
                                               //!< SD indicates how many
                                               //!< spheres touch this SD
    unsigned int* spheres_in_SD_composite,     //!< Big array that works in conjunction
                                               //!< with SD_countsOfSpheresTouching.

    float* d_sphere_pos_X_dt,
    float* d_sphere_pos_Y_dt,
    float* d_sphere_pos_Z_dt,
    ParamsPtr gran_params) {
    unsigned int thisSD = blockIdx.x;
    unsigned int spheresTouchingThisSD = SD_countsOfSpheresTouching[thisSD];
    unsigned mySphereID;  // Bring in data from global into shmem. Only a subset of threads get to do this.
    if (threadIdx.x < spheresTouchingThisSD) {
        // We need int64_ts to index into composite array
        uint64_t offset_in_composite_Array = ((uint64_t)thisSD) * MAX_NSPHERES_PER_SD + threadIdx.x;
        mySphereID = spheres_in_SD_composite[offset_in_composite_Array];
        unsigned int ownerSD = SDTripletID(pointSDTriplet(d_sphere_pos_X[mySphereID], d_sphere_pos_Y[mySphereID],
                                                          d_sphere_pos_Z[mySphereID], gran_params),
                                           gran_params);

        // Each SD applies force updates to the bodies it *owns*, this should happen once for each body
        if (thisSD == ownerSD) {
            // Check to see if we messed up badly somewhere
            if (d_sphere_pos_X_dt_update[mySphereID] == NAN || d_sphere_pos_Y_dt_update[mySphereID] == NAN ||
                d_sphere_pos_Z_dt_update[mySphereID] == NAN) {
                ABORTABORTABORT("NAN velocity update computed -- sd is %u, sphere is %u, velXcorr is %f\n", thisSD,
                                mySphereID, d_sphere_pos_X_dt_update[mySphereID]);
            }
            // Probably does not need to be atomic, but no conflicts means it won't be too slow anyways
            atomicAdd(d_sphere_pos_X_dt + mySphereID, d_sphere_pos_X_dt_update[mySphereID]);
            atomicAdd(d_sphere_pos_Y_dt + mySphereID, d_sphere_pos_Y_dt_update[mySphereID]);
            atomicAdd(d_sphere_pos_Z_dt + mySphereID, d_sphere_pos_Z_dt_update[mySphereID]);
        }
    }
    __syncthreads();
}

/**
This kernel call figures out forces on a sphere and carries out numerical integration to get the velocity updates of a
sphere.

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
__global__ void computeVelocityUpdates(const float alpha_h_bar,  //!< Value that controls actual step size.
                                       int* d_sphere_pos_X,      //!< Pointer to array containing data related to the
                                                                 //!< spheres in the box
                                       int* d_sphere_pos_Y,      //!< Pointer to array containing data related to the
                                                                 //!< spheres in the box
                                       int* d_sphere_pos_Z,      //!< Pointer to array containing data related to the
                                                                 //!< spheres in the box
                                       float* d_sphere_pos_X_dt_update,  //!< Pointer to array containing data related
                                                                         //!< to the spheres in the box
                                       float* d_sphere_pos_Y_dt_update,  //!< Pointer to array containing data related
                                                                         //!< to the spheres in the box
                                       float* d_sphere_pos_Z_dt_update,  //!< Pointer to array containing data related
                                                                         //!< to the spheres in the box
                                       unsigned int* SD_countsOfSpheresTouching,  //!< The array that for each
                                                                                  //!< SD indicates how many
                                                                                  //!< spheres touch this SD
                                       unsigned int* spheres_in_SD_composite,  //!< Big array that works in conjunction
                                                                               //!< with SD_countsOfSpheresTouching.
                                       float* d_sphere_pos_X_dt,
                                       float* d_sphere_pos_Y_dt,
                                       float* d_sphere_pos_Z_dt,
                                       ParamsPtr gran_params) {
    // still moar radius grab
    const unsigned int sphereRadius_SU = gran_params->sphereRadius_SU;

    // Cache positions and velocities in shared memory, this doesn't hurt occupancy at the moment
    __shared__ int sphere_X[MAX_NSPHERES_PER_SD];
    __shared__ int sphere_Y[MAX_NSPHERES_PER_SD];
    __shared__ int sphere_Z[MAX_NSPHERES_PER_SD];
    __shared__ float sphere_X_DOT[MAX_NSPHERES_PER_SD];
    __shared__ float sphere_Y_DOT[MAX_NSPHERES_PER_SD];
    __shared__ float sphere_Z_DOT[MAX_NSPHERES_PER_SD];

    unsigned int thisSD = blockIdx.x;
    unsigned int spheresTouchingThisSD = SD_countsOfSpheresTouching[thisSD];
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
        mySphereID = spheres_in_SD_composite[offset_in_composite_Array];
        sphere_X[threadIdx.x] = d_sphere_pos_X[mySphereID];
        sphere_Y[threadIdx.x] = d_sphere_pos_Y[mySphereID];
        sphere_Z[threadIdx.x] = d_sphere_pos_Z[mySphereID];
        sphere_X_DOT[threadIdx.x] = d_sphere_pos_X_dt[mySphereID];
        sphere_Y_DOT[threadIdx.x] = d_sphere_pos_Y_dt[mySphereID];
        sphere_Z_DOT[threadIdx.x] = d_sphere_pos_Z_dt[mySphereID];
    }

    __syncthreads();  // Needed to make sure data gets in shmem before using it elsewhere

    // Assumes each thread is a body, not the greatest assumption but we can fix that later
    // Note that if we have more threads than bodies, some effort gets wasted.
    unsigned int bodyA = threadIdx.x;

    // Each body looks at each other body and computes the force that the other body exerts on it
    if (bodyA < spheresTouchingThisSD) {
        // The update we make to the velocities, it's ok for this to be a float since it should be pretty small anyways,
        // gets truncated to int very soon
        float bodyA_X_velCorr = 0.f;
        float bodyA_Y_velCorr = 0.f;
        float bodyA_Z_velCorr = 0.f;

        float scalingFactor = alpha_h_bar * gran_params->Kn_s2s_SU;
        double sphdiameter = 2. * sphereRadius_SU;
        double invSphDiameter = 1. / sphdiameter;
        unsigned int ownerSD =
            SDTripletID(pointSDTriplet(sphere_X[bodyA], sphere_Y[bodyA], sphere_Z[bodyA], gran_params), gran_params);
        for (unsigned char bodyB = 0; bodyB < spheresTouchingThisSD; bodyB++) {
            // unsigned int theirSphID = spheres_in_SD_composite[thisSD * MAX_NSPHERES_PER_SD + bodyB];
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
            double distance_normalized = 0;

            // unsigned int theirSphID = spheres_in_SD_composite[thisSD * MAX_NSPHERES_PER_SD + bodyB];
            // Don't check for collision with self
            unsigned char bodyB = bodyB_list[idx];

            // NOTE below here, it seems faster to do coalesced shared mem accesses than caching sphere_X[bodyA] and the
            // like in a register
            // This avoids computing a square to figure our if collision or not
            double deltaX = (sphere_X[bodyA] - sphere_X[bodyB]) * invSphDiameter;
            double deltaY = (sphere_Y[bodyA] - sphere_Y[bodyB]) * invSphDiameter;
            double deltaZ = (sphere_Z[bodyA] - sphere_Z[bodyB]) * invSphDiameter;

            // Velocity difference, it's better to do a coalesced access here than a fragmented access inside
            float deltaX_dot = sphere_X_DOT[bodyA] - sphere_X_DOT[bodyB];
            float deltaY_dot = sphere_Y_DOT[bodyA] - sphere_Y_DOT[bodyB];
            float deltaZ_dot = sphere_Z_DOT[bodyA] - sphere_Z_DOT[bodyB];

            distance_normalized = deltaX * deltaX;
            distance_normalized += deltaY * deltaY;
            distance_normalized += deltaZ * deltaZ;
            // We now have

            // Note: this can be accelerated should we decide to go w/ float. Then we can use the CUDA
            // intrinsic:
            // __device__ ​ float rnormf ( int  dim, const float* a)
            // http://docs.nvidia.com/cuda/cuda-math-api/group__CUDA__MATH__SINGLE.html#group__CUDA__MATH__SINGLE

            // Compute penetration term, this becomes the delta as we want it
            // Makes more sense to find rsqrt, the compiler is doing this anyways
            float reciplength = rsqrt(distance_normalized);
            float penetration = reciplength - 1.;

            // Compute nondim force term

            // These lines use forward euler right now. Each has an alpha_h_bar term but we can use a different
            // method simply by changing the following computations
            // Compute force updates for spring term
            float springTermX = scalingFactor * deltaX * sphdiameter * penetration;
            float springTermY = scalingFactor * deltaY * sphdiameter * penetration;
            float springTermZ = scalingFactor * deltaZ * sphdiameter * penetration;
            // Compute force updates for damping term
            float dampingTermX = -gran_params->Gamma_n_s2s_SU * alpha_h_bar * deltaX_dot;
            float dampingTermY = -gran_params->Gamma_n_s2s_SU * alpha_h_bar * deltaY_dot;
            float dampingTermZ = -gran_params->Gamma_n_s2s_SU * alpha_h_bar * deltaZ_dot;

            float cohesionConstant = gran_params->gravAcc_Z_SU * gran_params->cohesion_ratio;

            // Compute force updates for cohesion term, is opposite the spring term
            float cohesionTermX = cohesionConstant * deltaX * reciplength;
            float cohesionTermY = cohesionConstant * deltaY * reciplength;
            float cohesionTermZ = cohesionConstant * deltaZ * reciplength;

            // Add damping term to spring term, write back to counter
            bodyA_X_velCorr += springTermX + dampingTermX + cohesionTermX;
            bodyA_Y_velCorr += springTermY + dampingTermY + cohesionTermY;
            bodyA_Z_velCorr += springTermZ + dampingTermZ + cohesionTermZ;
        }
        // NOTE from Conlain -- this latex is broken, I think
        /**
        Compute now the forces on bodyA; i.e, what bodyA feels (if bodyA is in contact w/ anybody in this SD).

        \f[ \mbox{penetration} = \left[\left(\frac{x_A}{2R} -
        \frac{x_B}{2R}\right)^2 + \left(\frac{y_A}{2R} - \frac{y_B}{2R}\right)^2 + \left(\frac{z_A}{2R} -
        \frac{z_B}{2R}\right)^2\right] \f]

        The deformation that enters the computation of the normal contact force is scaled by the square of the step
        size, the stiffness and the particle mass: \f[ h^2 \frac{K}{m} \delta \f] Then, the quantity that comes into
        play in computing the update in positions looks like \f[ h^2 \frac{K}{m} \times 2R \times \left( \frac{
        1}{\sqrt{(\left(\frac{x_A}{2R} - \frac{x_B}{2R}\right)^2 + \left(\frac{y_A}{2R} - \frac{y_B}{2R}\right)^2 +
        \left(\frac{z_A}{2R} - \frac{z_B}{2R}\right)^2)}} -1 \right) \times \begin{bmatrix}
        \frac{x_A}{2R} - \frac{x_B}{2R}  \vspace{0.2cm}\\
        \frac{y_A}{2R} - \frac{y_B}{2R} \vspace{0.2cm}\\
        \frac{z_A}{2R} - \frac{z_B}{2R}
        \end{bmatrix}
        \f]
        */

        // IMPORTANT: Make sure that the sphere belongs to *this* SD, otherwise we'll end up with double counting
        // this force.
        // If this SD owns the body, add its wall forces and grav forces. This should be pretty non-divergent since
        // most spheres won't be on the border
        if (ownerSD == thisSD) {
            // Perhaps this sphere is hitting the wall[s]
            boxWallsEffects(alpha_h_bar, sphere_X[bodyA], sphere_Y[bodyA], sphere_Z[bodyA], sphere_X_DOT[bodyA],
                            sphere_Y_DOT[bodyA], sphere_Z_DOT[bodyA], bodyA_X_velCorr, bodyA_Y_velCorr, bodyA_Z_velCorr,
                            gran_params);
            // If the sphere belongs to this SD, add up the gravitational force component.

            bodyA_X_velCorr += alpha_h_bar * gran_params->gravAcc_X_SU;
            bodyA_Y_velCorr += alpha_h_bar * gran_params->gravAcc_Y_SU;
            bodyA_Z_velCorr += alpha_h_bar * gran_params->gravAcc_Z_SU;
        }

        // Write the velocity updates back to global memory so that we can apply them AFTER this kernel finishes
        atomicAdd(d_sphere_pos_X_dt_update + mySphereID, bodyA_X_velCorr);
        atomicAdd(d_sphere_pos_Y_dt_update + mySphereID, bodyA_Y_velCorr);
        atomicAdd(d_sphere_pos_Z_dt_update + mySphereID, bodyA_Z_velCorr);
    }
    __syncthreads();
}

template <unsigned int CUB_THREADS>  //!< Number of CUB threads engaged in block-collective CUB operations.
                                     //!< Should be a multiple of 32
__global__ void updatePositions(const float alpha_h_bar,          //!< The numerical integration time step
                                int* d_sphere_pos_X,              //!< Pointer to array containing data related to the
                                                                  //!< spheres in the box
                                int* d_sphere_pos_Y,              //!< Pointer to array containing data related to the
                                                                  //!< spheres in the box
                                int* d_sphere_pos_Z,              //!< Pointer to array containing data related to the
                                                                  //!< spheres in the box
                                float* d_sphere_pos_X_dt,         //!< Pointer to array containing data related to
                                                                  //!< the spheres in the box
                                float* d_sphere_pos_Y_dt,         //!< Pointer to array containing data related to
                                                                  //!< the spheres in the box
                                float* d_sphere_pos_Z_dt,         //!< Pointer to array containing data related to
                                                                  //!< the spheres in the box
                                float* d_sphere_pos_X_dt_update,  //!< Pointer to array containing data related to
                                                                  //!< the spheres in the box
                                float* d_sphere_pos_Y_dt_update,  //!< Pointer to array containing data related to
                                                                  //!< the spheres in the box
                                float* d_sphere_pos_Z_dt_update,  //!< Pointer to array containing data related to
                                                                  //!< the spheres in the box
                                unsigned int* SD_countsOfSpheresTouching,  //!< The array that for each
                                                                           //!< SD indicates how many
                                                                           //!< spheres touch this SD
                                unsigned int* spheres_in_SD_composite,     //!< Big array that works in conjunction
                                                                           //!< with SD_countsOfSpheresTouching.
                                                                           //!< "spheres_in_SD_composite" says which
                                                                           //!< SD contains what spheres
                                unsigned int nSpheres,
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
    // Write back velocity updates
    if (mySphereID < nSpheres) {
        // Check to see if we messed up badly somewhere
        if (d_sphere_pos_X_dt_update[mySphereID] == NAN || d_sphere_pos_Y_dt_update[mySphereID] == NAN ||
            d_sphere_pos_Z_dt_update[mySphereID] == NAN) {
            ABORTABORTABORT("NAN velocity update computed -- sphere is %u, velXcorr is %f\n", mySphereID,
                            d_sphere_pos_X_dt_update[mySphereID]);
        }

        // Probably does not need to be atomic, but no conflicts means it won't be too slow anyways
        atomicAdd(d_sphere_pos_X_dt + mySphereID, d_sphere_pos_X_dt_update[mySphereID]);
        atomicAdd(d_sphere_pos_Y_dt + mySphereID, d_sphere_pos_Y_dt_update[mySphereID]);
        atomicAdd(d_sphere_pos_Z_dt + mySphereID, d_sphere_pos_Z_dt_update[mySphereID]);
    }
    // wait for everyone to finish
    __syncthreads();
    if (mySphereID < nSpheres) {
        // Perform numerical integration. For now, use Explicit Euler. Hitting cache, also coalesced.
        xSphCenter = alpha_h_bar * d_sphere_pos_X_dt[mySphereID];
        ySphCenter = alpha_h_bar * d_sphere_pos_Y_dt[mySphereID];
        zSphCenter = alpha_h_bar * d_sphere_pos_Z_dt[mySphereID];

        xSphCenter += d_sphere_pos_X[mySphereID];
        d_sphere_pos_X[mySphereID] = xSphCenter;

        ySphCenter += d_sphere_pos_Y[mySphereID];
        d_sphere_pos_Y[mySphereID] = ySphCenter;

        zSphCenter += d_sphere_pos_Z[mySphereID];
        d_sphere_pos_Z[mySphereID] = zSphCenter;

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
            unsigned char sphere_offset = atomicAdd(SD_countsOfSpheresTouching + touchedSD, winningStreak);

            // The value sphere_offset now gives a *relative* offset in the composite array; i.e.,
            // spheres_in_SD_composite. Get the SD component

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

    // Write out the data now; reister with spheres_in_SD_composite each sphere that touches a certain ID
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
                spheres_in_SD_composite[offset] = sphIDs[i];
            }
        }
    }
}
