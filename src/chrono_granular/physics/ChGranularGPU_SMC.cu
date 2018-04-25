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
// Use user-defined quantities for coefficients
#define K_N (1.f / (1.f * psi_T_dFactor * psi_T_dFactor * psi_h_dFactor))
// TODO we need to get the damping coefficient from user
#define GAMMA_N .005

#define CUDA_THREADS 128

#define ABORTABORTABORT(...) \
    {                        \
        printf(__VA_ARGS__); \
        __threadfence();     \
        asm("trap;");        \
    }

__constant__ unsigned int d_monoDisperseSphRadius_SU;  //!< Radius of the sphere, expressed in SU
__constant__ unsigned int d_SD_Ldim_SU;                //!< Ad-ed L-dimension of the SD box
__constant__ unsigned int d_SD_Ddim_SU;                //!< Ad-ed D-dimension of the SD box
__constant__ unsigned int d_SD_Hdim_SU;                //!< Ad-ed H-dimension of the SD box
__constant__ unsigned int psi_T_dFactor;               //!< factor used in establishing the software-time-unit
__constant__ unsigned int psi_h_dFactor;               //!< factor used in establishing the software-time-unit
__constant__ unsigned int psi_L_dFactor;               //!< factor used in establishing the software-time-unit
__constant__ unsigned int d_box_L_SU;                  //!< Ad-ed L-dimension of the BD box in multiples of subdomains
__constant__ unsigned int d_box_D_SU;                  //!< Ad-ed D-dimension of the BD box in multiples of subdomains
__constant__ unsigned int d_box_H_SU;                  //!< Ad-ed H-dimension of the BD box in multiples of subdomains
__constant__ float gravAcc_X_d_factor_SU;              //!< Device counterpart of the constant gravAcc_X_factor_SU
__constant__ float gravAcc_Y_d_factor_SU;              //!< Device counterpart of the constant gravAcc_Y_factor_SU
__constant__ float gravAcc_Z_d_factor_SU;              //!< Device counterpart of the constant gravAcc_Z_factor_SU

// Changed by updateBDPosition() at every timestep
__constant__ int d_BD_frame_X;  //!< The bottom-left corner xPos of the BD, allows boxes not centered at origin
__constant__ int d_BD_frame_Y;  //!< The bottom-left corner yPos of the BD, allows boxes not centered at origin
__constant__ int d_BD_frame_Z;  //!< The bottom-left corner zPos of the BD, allows boxes not centered at origin
// Disable because stability
// __constant__ float d_BD_frame_X_dot;  //!< The bottom-left corner xPos of the BD, allows boxes not centered at origin
// __constant__ float d_BD_frame_Y_dot;  //!< The bottom-left corner yPos of the BD, allows boxes not centered at origin
// __constant__ float d_BD_frame_Z_dot;  //!< The bottom-left corner zPos of the BD, allows boxes not centered at origin

__constant__ double d_DE_Mass;

// Decide which SD owns this point in space
// Pass it the Center of Mass location for a DE to get its owner, also used to get contact point
__device__ unsigned int figureOutOwnerSD(int sphCenter_X, int sphCenter_Y, int sphCenter_Z) {
    // Note that this offset allows us to have moving walls and the like very easily
    // printf("corner is %d, calc is %d\n", -d_BD_frame_X, (d_box_L_SU * d_SD_Ldim_SU) / 2);
    long int sphCenter_X_modified = -d_BD_frame_X + sphCenter_X;
    long int sphCenter_Y_modified = -d_BD_frame_Y + sphCenter_Y;
    long int sphCenter_Z_modified = -d_BD_frame_Z + sphCenter_Z;
    unsigned int n[3];
    // Get the SD of the sphere's center in the xdir
    n[0] = (sphCenter_X_modified) / d_SD_Ldim_SU;
    // Same for D and H
    n[1] = (sphCenter_Y_modified) / d_SD_Ddim_SU;
    n[2] = (sphCenter_Z_modified) / d_SD_Hdim_SU;
    return n[0] * d_box_D_SU * d_box_H_SU + n[1] * d_box_H_SU + n[2];
}

/// Takes in a sphere's position and inserts into the given int array[8] which subdomains, if any, are touched
/// The array is indexed with the ones bit equal to +/- x, twos bit equal to +/- y, and the fours bit equal to +/- z
/// A bit set to 0 means the lower index, whereas 1 means the higher index (lower + 1)
/// The kernel computes global x, y, and z indices for the bottom-left subdomain and then uses those to figure out which
/// subdomains described in the corresponding 8-SD cube are touched by the sphere. The kernel then converts these
/// indices to indices into the global SD list via the (currently local) conv[3] data structure
/// Should be mostly bug-free, especially away from boundaries
__device__ void figureOutTouchedSD(int sphCenter_X, int sphCenter_Y, int sphCenter_Z, unsigned int SDs[8]) {
    // I added these to fix a bug, we can inline them if/when needed but they ARE necessary
    // We need to offset so that the bottom-left corner is at the origin
    long int sphCenter_X_modified = -d_BD_frame_X + sphCenter_X;
    long int sphCenter_Y_modified = -d_BD_frame_Y + sphCenter_Y;
    long int sphCenter_Z_modified = -d_BD_frame_Z + sphCenter_Z;
    unsigned int n[3];
    // TODO this doesn't handle if the ball is slightly penetrating the boundary, could result in negative values or end
    // GIDs beyond bounds. We might want to do a check to see if it's outside and set 'valid' accordingly
    // NOTE: This is integer arithmetic to compute the floor. We want to get the first SD below the sphere
    // nx = (xCenter - radius) / wx .
    n[0] = (sphCenter_X_modified - d_monoDisperseSphRadius_SU) / d_SD_Ldim_SU;
    // Same for D and H
    n[1] = (sphCenter_Y_modified - d_monoDisperseSphRadius_SU) / d_SD_Ddim_SU;
    n[2] = (sphCenter_Z_modified - d_monoDisperseSphRadius_SU) / d_SD_Hdim_SU;
    // This is kind of gross and hacky, if we're at the bottom boundary, the bottom SD is 0
    // If we're at the top boundary, the top SD is the max
    if (sphCenter_X_modified - (signed int)d_monoDisperseSphRadius_SU <= 0) {
        n[0] = 0;
    } else if (sphCenter_X_modified + (signed int)d_monoDisperseSphRadius_SU >= MAX_X_POS_UNSIGNED) {
        n[0] = d_box_L_SU - 1 - 1;  // Subtract one for last SD, subtract one more for bottom SD
    }
    if (sphCenter_Y_modified - (signed int)d_monoDisperseSphRadius_SU <= 0) {
        n[1] = 0;
    } else if (sphCenter_Y_modified + (signed int)d_monoDisperseSphRadius_SU >= MAX_Y_POS_UNSIGNED) {
        n[1] = d_box_D_SU - 1 - 1;  // Subtract one for last SD, subtract one more for bottom SD
    }
    if (sphCenter_Z_modified - (signed int)d_monoDisperseSphRadius_SU <= 0) {
        n[2] = 0;
    } else if (sphCenter_Z_modified + (signed int)d_monoDisperseSphRadius_SU >= MAX_Z_POS_UNSIGNED) {
        n[2] = d_box_H_SU - 1 - 1;  // Subtract one for last SD, subtract one more for bottom SD
    }
    // n[0] += (sphCenter_X_modified - (signed int)d_monoDisperseSphRadius_SU <= 0) -
    //         (sphCenter_X_modified + (signed int)d_monoDisperseSphRadius_SU >= MAX_X_POS);
    // n[1] += (sphCenter_Y_modified - (signed int)d_monoDisperseSphRadius_SU <= 0) -
    //         (sphCenter_Y_modified + (signed int)d_monoDisperseSphRadius_SU >= MAX_X_POS);
    // n[2] += (sphCenter_Z_modified - (signed int)d_monoDisperseSphRadius_SU <= 0) -
    //         (sphCenter_Z_modified + (signed int)d_monoDisperseSphRadius_SU >= MAX_X_POS);
    // This conditional says if we're at the boundary
    unsigned int boundary = sphCenter_X_modified - (signed int)d_monoDisperseSphRadius_SU <= 0 ||
                            sphCenter_X_modified + (signed int)d_monoDisperseSphRadius_SU >= MAX_X_POS_UNSIGNED ||
                            sphCenter_Y_modified - (signed int)d_monoDisperseSphRadius_SU <= 0 ||
                            sphCenter_Y_modified + (signed int)d_monoDisperseSphRadius_SU >= MAX_Y_POS_UNSIGNED ||
                            sphCenter_Z_modified - (signed int)d_monoDisperseSphRadius_SU <= 0 ||
                            sphCenter_Z_modified + (signed int)d_monoDisperseSphRadius_SU >= MAX_Z_POS_UNSIGNED;
    if (n[0] >= d_box_L_SU) {
        // printf("%d, %d\n", sphCenter_X_modified - d_monoDisperseSphRadius_SU,
        //        sphCenter_X_modified - (signed int)d_monoDisperseSphRadius_SU <= 0);
        ABORTABORTABORT(
            "x is too large, boundary is %u, n is %u, nmax is %u, pos is %d, mod is %d, max is %d, dim is %d\n",
            boundary, n[0], d_box_L_SU, sphCenter_X, sphCenter_X_modified, d_SD_Ldim_SU * d_box_L_SU, d_SD_Ldim_SU,
            d_monoDisperseSphRadius_SU);
    }
    // Find distance from next box in relevant dir to center, we may be straddling the two
    int d[3];                                                 // Store penetrations
    d[0] = (n[0] + 1) * d_SD_Ldim_SU - sphCenter_X_modified;  // dx = (nx + 1)* wx - x
    d[1] = (n[1] + 1) * d_SD_Ddim_SU - sphCenter_Y_modified;
    d[2] = (n[2] + 1) * d_SD_Hdim_SU - sphCenter_Z_modified;

    // Calculate global indices from locals
    // ones bit is x, twos bit is y, threes bit is z
    // do some cute bit shifting and snag bit at correct place
    // For each index in SDs
    for (int i = 0; i < 8; i++) {
        SDs[i] = 0;                // Init to 0
        unsigned int valid = 0x1;  // Assume this SD is touched at start

        // s adds an offset to directional index for SDs
        // High/low in x-dir
        // unsigned int s = i & 0x1; // Inlined now
        // Scale to global index and add to total
        SDs[i] += (n[0] + (i & 0x1)) * d_box_D_SU * d_box_H_SU;
        // s == own[e] evals true if the current SD is owner
        // If both touch it or we own it, the result is valid
        valid &= (abs(d[0]) < d_monoDisperseSphRadius_SU) || ((i & 0x1) == (d[0] < 0));

        // High/low in y-dir
        // s = i & 0x2; // Inlined now
        // Scale to global index and add to total
        SDs[i] += (n[1] + ((i >> 1) & 0x1)) * d_box_H_SU;
        // If both touch it or we own it, the result is valid
        valid &= (abs(d[1]) < d_monoDisperseSphRadius_SU) || (((i >> 1) & 0x1) == (d[1] < 0));

        // High/low in z-dir
        // s = i & 0x4; // Inlined now
        // Scale to global index and add to total
        SDs[i] += (n[2] + ((i >> 2) & 0x1));
        // If both touch it or we own it, the result is valid
        valid &= (abs(d[2]) < d_monoDisperseSphRadius_SU) || (((i >> 2) & 0x1) == (d[2] < 0));

        // This is nasty but it checks if we're actually touching a bounary, in which case

        // This ternary is hopefully better than a conditional
        // If valid is false, then the SD is actually NULL_GRANULAR_ID
        if (valid && SDs[i] >= d_box_D_SU * d_box_L_SU * d_box_H_SU) {
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
    unsigned int nSpheres                      //!< Number of spheres in the box
) {
    int xSphCenter;
    int ySphCenter;
    int zSphCenter;

    /// Set aside shared memory
    volatile __shared__ size_t offsetInComposite_SphInSD_Array[CUB_THREADS * 8];
    volatile __shared__ bool shMem_head_flags[CUB_THREADS * 8];

    typedef cub::BlockRadixSort<unsigned int, CUB_THREADS, 8, unsigned int> BlockRadixSortOP;
    __shared__ typename BlockRadixSortOP::TempStorage temp_storage_sort;

    typedef cub::BlockDiscontinuity<unsigned int, CUB_THREADS> Block_Discontinuity;
    __shared__ typename Block_Discontinuity::TempStorage temp_storage_disc;

    // Figure out what sphereID this thread will handle. We work with a 1D block structure and a 1D grid structure
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned int sphIDs[8] = {mySphereID, mySphereID, mySphereID, mySphereID,
                              mySphereID, mySphereID, mySphereID, mySphereID};

    // This uses a lot of registers but is needed
    unsigned int SDsTouched[8] = {NULL_GRANULAR_ID, NULL_GRANULAR_ID, NULL_GRANULAR_ID, NULL_GRANULAR_ID,
                                  NULL_GRANULAR_ID, NULL_GRANULAR_ID, NULL_GRANULAR_ID, NULL_GRANULAR_ID};
    if (mySphereID < nSpheres) {
        // Coalesced mem access
        xSphCenter = d_sphere_pos_X[mySphereID];
        ySphCenter = d_sphere_pos_Y[mySphereID];
        zSphCenter = d_sphere_pos_Z[mySphereID];

        figureOutTouchedSD(xSphCenter, ySphCenter, zSphCenter, SDsTouched);
    }

    __syncthreads();

    // Sort by the ID of the SD touched
    BlockRadixSortOP(temp_storage_sort).Sort(SDsTouched, sphIDs);
    __syncthreads();

    // Do a winningStreak search on whole block, might not have high utilization here
    bool head_flags[8];
    Block_Discontinuity(temp_storage_disc).FlagHeads(head_flags, SDsTouched, cub::Inequality());
    __syncthreads();

    // Write back to shared memory; eight-way bank conflicts here - to revisit later
    for (unsigned int i = 0; i < 8; i++) {
        shMem_head_flags[8 * threadIdx.x + i] = head_flags[i];
    }

    // Seed offsetInComposite_SphInSD_Array with "no valid ID" so that we know later on what is legit;
    // No shmem bank coflicts here, good access...
    for (unsigned int i = 0; i < 8; i++) {
        offsetInComposite_SphInSD_Array[i * CUB_THREADS + threadIdx.x] = NULL_GRANULAR_ID_LONG;
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
            size_t offset = atomicAdd(SD_countsOfSpheresTouching + touchedSD, winningStreak);

            // The value offset now gives a *relative* offset in the composite array; i.e., spheres_in_SD_composite.
            // Get the absolute offset
            offset += ((size_t)touchedSD) * MAX_COUNT_OF_DEs_PER_SD;

            // Produce the offsets for this streak of spheres with identical SD ids
            for (unsigned int i = 0; i < winningStreak; i++)
                offsetInComposite_SphInSD_Array[idInShared + i] = offset++;
        }
    }

    __syncthreads();  // needed since we write to shared memory above; i.e., offsetInComposite_SphInSD_Array

    const size_t max_composite_index = (size_t)d_box_D_SU * d_box_L_SU * d_box_H_SU * MAX_COUNT_OF_DEs_PER_SD;

    // Write out the data now; reister with spheres_in_SD_composite each sphere that touches a certain ID
    for (unsigned int i = 0; i < 8; i++) {
        size_t offset = offsetInComposite_SphInSD_Array[8 * threadIdx.x + i];
        if (offset != NULL_GRANULAR_ID_LONG) {
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
__device__ void boxWallsEffects(const float alpha_h_bar,  //!< Integration step size.
                                const int sphXpos,        //!< Global X position of DE
                                const int sphYpos,        //!< Global Y position of DE
                                const int sphZpos,        //!< Global Z position of DE
                                const float sphXvel,      //!< Global X velocity of DE
                                const float sphYvel,      //!< Global Y velocity of DE
                                const float sphZvel,      //!< Global Z velocity of DE
                                float& X_Vel_corr,        //!< Velocity correction in Xdir
                                float& Y_Vel_corr,        //!< Velocity correction in Xdir
                                float& Z_Vel_corr         //!< Velocity correction in Xdir
) {
    // Shift frame so that origin is at lower left corner
    int sphXpos_modified = -d_BD_frame_X + sphXpos;
    int sphYpos_modified = -d_BD_frame_Y + sphYpos;
    int sphZpos_modified = -d_BD_frame_Z + sphZpos;

    // penetration into wall
    signed int pen = 0;
    // Are we touching wall?
    int touchingWall = 0;
    // Compute spring factor in force
    float scalingFactor = (1.0f * alpha_h_bar) * K_N;
    // This gamma_n is fake, needs to be given by user
    // Ignore damping at walls for now, something is wrong
    // float dampingCoeff = 0;  //.0001 * alpha_h_bar;
    // dampingCoeff = 0;
    // tmp vars to save writes, probably not necessary
    float xcomp = 0;
    float ycomp = 0;
    float zcomp = 0;

    // Do x direction
    // penetration of sphere into bottom x wall
    pen = sphXpos_modified - (signed int)d_monoDisperseSphRadius_SU;
    // true if sphere touching wall
    touchingWall = (pen < 0) && abs(pen) < d_monoDisperseSphRadius_SU;
    // in this case, pen is negative and we want a positive restorative force
    // Need to get relative velocity
    xcomp += touchingWall * (scalingFactor * abs(pen));

    // Do top X wall
    pen = MAX_X_POS_UNSIGNED - (sphXpos_modified + (signed int)d_monoDisperseSphRadius_SU);
    touchingWall = (pen < 0) && abs(pen) < d_monoDisperseSphRadius_SU;
    // in this case, pen is positive and we want a negative restorative force
    xcomp += touchingWall * (-1 * scalingFactor * abs(pen));

    // penetration of sphere into relevant wall
    pen = sphYpos_modified - (signed int)d_monoDisperseSphRadius_SU;
    // true if sphere touching wall
    touchingWall = (pen < 0) && abs(pen) < d_monoDisperseSphRadius_SU;
    // in this case, pen is negative and we want a positive restorative force
    ycomp += touchingWall * (scalingFactor * abs(pen));

    // Do top y wall
    pen = MAX_Y_POS_UNSIGNED - (sphYpos_modified + (signed int)d_monoDisperseSphRadius_SU);
    touchingWall = (pen < 0) && abs(pen) < d_monoDisperseSphRadius_SU;
    // in this case, pen is positive and we want a negative restorative force
    ycomp += touchingWall * (-1 * scalingFactor * abs(pen));

    // penetration of sphere into relevant wall
    pen = sphZpos_modified - (signed int)d_monoDisperseSphRadius_SU;
    // true if sphere touching wall
    touchingWall = (pen < 0) && abs(pen) < d_monoDisperseSphRadius_SU;
    // in this case, pen is negative and we want a positive restorative force
    zcomp += touchingWall * (scalingFactor * abs(pen));

    // Do top z wall
    pen = MAX_Z_POS_UNSIGNED - (sphZpos_modified + (signed int)d_monoDisperseSphRadius_SU);
    touchingWall = (pen < 0) && abs(pen) < d_monoDisperseSphRadius_SU;
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
    unsigned int alpha_h_bar,      //!< Value that controls actual step size, not actually needed for this kernel
    int* d_sphere_pos_X,           //!< Pointer to array containing data related to the
                                   //!< spheres in the box
    int* d_sphere_pos_Y,           //!< Pointer to array containing data related to the
                                   //!< spheres in the box
    int* d_sphere_pos_Z,           //!< Pointer to array containing data related to the
                                   //!< spheres in the box
    float* d_sphere_pos_X_update,  //!< Pointer to array containing data related to
                                   //!< the spheres in the box
    float* d_sphere_pos_Y_update,  //!< Pointer to array containing data related to
                                   //!< the spheres in the box
    float* d_sphere_pos_Z_update,  //!< Pointer to array containing data related to
                                   //!< the spheres in the box
    unsigned int* SD_countsOfSpheresTouching,  //!< The array that for each
                                               //!< SD indicates how many
                                               //!< spheres touch this SD
    unsigned int* spheres_in_SD_composite,     //!< Big array that works in conjunction
                                               //!< with SD_countsOfSpheresTouching.

    float* d_sphere_pos_X_dt,
    float* d_sphere_pos_Y_dt,
    float* d_sphere_pos_Z_dt) {
    unsigned int thisSD = blockIdx.x;
    unsigned int spheresTouchingThisSD = SD_countsOfSpheresTouching[thisSD];
    unsigned mySphereID;  // Bring in data from global into shmem. Only a subset of threads get to do this.
    if (threadIdx.x < spheresTouchingThisSD) {
        mySphereID = spheres_in_SD_composite[thisSD * MAX_NSPHERES_PER_SD + threadIdx.x];

        unsigned int ownerSD =
            figureOutOwnerSD(d_sphere_pos_X[mySphereID], d_sphere_pos_Y[mySphereID], d_sphere_pos_Z[mySphereID]);
        // Each SD applies force updates to the bodies it *owns*, this should happen once for each body
        if (thisSD == ownerSD) {
            if (d_sphere_pos_X_update[mySphereID] == NAN || d_sphere_pos_Y_update[mySphereID] == NAN ||
                d_sphere_pos_Z_update[mySphereID] == NAN) {
                ABORTABORTABORT("NAN velocity update computed -- sd is %u, sphere is %u, velXcorr is %f\n", thisSD,
                                mySphereID, d_sphere_pos_X_update[mySphereID]);
            }
            // Probably does not need to be atomic, but no conflicts means it won't be too slow anyways
            atomicAdd(d_sphere_pos_X_dt + mySphereID, d_sphere_pos_X_update[mySphereID]);
            atomicAdd(d_sphere_pos_Y_dt + mySphereID, d_sphere_pos_Y_update[mySphereID]);
            atomicAdd(d_sphere_pos_Z_dt + mySphereID, d_sphere_pos_Z_update[mySphereID]);
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
__global__ void computeVelocityUpdates(unsigned int alpha_h_bar,  //!< Value that controls actual step size.
                                       int* d_sphere_pos_X,       //!< Pointer to array containing data related to the
                                                                  //!< spheres in the box
                                       int* d_sphere_pos_Y,       //!< Pointer to array containing data related to the
                                                                  //!< spheres in the box
                                       int* d_sphere_pos_Z,       //!< Pointer to array containing data related to the
                                                                  //!< spheres in the box
                                       float* d_sphere_pos_X_update,  //!< Pointer to array containing data related to
                                                                      //!< the spheres in the box
                                       float* d_sphere_pos_Y_update,  //!< Pointer to array containing data related to
                                                                      //!< the spheres in the box
                                       float* d_sphere_pos_Z_update,  //!< Pointer to array containing data related to
                                                                      //!< the spheres in the box
                                       unsigned int* SD_countsOfSpheresTouching,  //!< The array that for each
                                                                                  //!< SD indicates how many
                                                                                  //!< spheres touch this SD
                                       unsigned int* spheres_in_SD_composite,  //!< Big array that works in conjunction
                                                                               //!< with SD_countsOfSpheresTouching.
                                       float* d_sphere_pos_X_dt,
                                       float* d_sphere_pos_Y_dt,
                                       float* d_sphere_pos_Z_dt) {
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
    unsigned char bodyB_list[12];
    unsigned int ncontacts = 0;

    // Bring in data from global into shmem. Only a subset of threads get to do this.
    // Note that we're not using shared memory very heavily, so our bandwidth is pretty low
    if (threadIdx.x < spheresTouchingThisSD) {
        mySphereID = spheres_in_SD_composite[thisSD * MAX_NSPHERES_PER_SD + threadIdx.x];
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

    // k * delta_t
    // This is ok as a float as well since it is a reasonable number by design

    // If we overran, we have a major issue, time to crash before we make illegal memory accesses
    if (threadIdx.x == 0 && spheresTouchingThisSD > MAX_NSPHERES_PER_SD) {
        // Crash now
        ABORTABORTABORT("TOO MANY SPHERES! SD %u has %u spheres\n", thisSD, spheresTouchingThisSD);
    }

    // Each body looks at each other body and computes the force that the other body exerts on it
    if (bodyA < spheresTouchingThisSD) {
        // The update we make to the velocities, it's ok for this to be a float since it should be pretty small anyways,
        // gets truncated to int very soon
        float bodyA_X_velCorr = 0.f;
        float bodyA_Y_velCorr = 0.f;
        float bodyA_Z_velCorr = 0.f;

        float scalingFactor = alpha_h_bar * K_N;
        double sphdiameter = 2. * d_monoDisperseSphRadius_SU;
        double invSphDiameter = 1. / sphdiameter;
        unsigned int ownerSD = figureOutOwnerSD(sphere_X[bodyA], sphere_Y[bodyA], sphere_Z[bodyA]);
        for (unsigned char bodyB = 0; bodyB < spheresTouchingThisSD; bodyB++) {
            // unsigned int theirSphID = spheres_in_SD_composite[thisSD * MAX_NSPHERES_PER_SD + bodyB];
            // Don't check for collision with self
            if (bodyA == bodyB)
                continue;

            // Compute penetration to check for collision, we can use ints provided the diameter is small enough
            long int penetration_int = 0;

            // This avoids computing a square to figure our if collision or not
            long int deltaX = (sphere_X[bodyA] - sphere_X[bodyB]);
            long int deltaY = (sphere_Y[bodyA] - sphere_Y[bodyB]);
            long int deltaZ = (sphere_Z[bodyA] - sphere_Z[bodyB]);

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
            unsigned int contactSD = figureOutOwnerSD(contactX, contactY, contactZ);

            // We have a collision here, log it for later
            // not very divergent, super quick
            if (contactSD == thisSD && penetration_int < 4 * d_monoDisperseSphRadius_SU * d_monoDisperseSphRadius_SU) {
                bodyB_list[ncontacts] = bodyB;  // Save the collision pair
                ncontacts++;                    // Increment the contact counter
            }
        }

        // NOTE that below here I used double precision because I didn't know how much precision I needed. Reducing the
        // amount of doubles will certainly speed this up
        // Run through and do actual force computations, for these we know each one is a legit collision
        for (unsigned int idx = 0; idx < ncontacts; idx++) {
            double penetration = 0;

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

            penetration = deltaX * deltaX;
            penetration += deltaY * deltaY;
            penetration += deltaZ * deltaZ;
            // We now have

            // Note: this can be accelerated should we decide to go w/ float. Then we can use the CUDA
            // intrinsic:
            // __device__ ​ float rnormf ( int  dim, const float* a)
            // http://docs.nvidia.com/cuda/cuda-math-api/group__CUDA__MATH__SINGLE.html#group__CUDA__MATH__SINGLE

            // Compute penetration term, this becomes the delta as we want it
            // Makes more sense to find rsqrt, the compiler is doing this anyways
            penetration = rsqrt(penetration);
            penetration -= 1.;

            // Compute nondim force term

            // These lines use forward euler right now. Each has an alpha_h_bar term but we can use a different
            // method simply by changing the following computations
            // Compute force updates
            float springTermX = scalingFactor * deltaX * sphdiameter * penetration / d_DE_Mass;
            float springTermY = scalingFactor * deltaY * sphdiameter * penetration / d_DE_Mass;
            float springTermZ = scalingFactor * deltaZ * sphdiameter * penetration / d_DE_Mass;
            // Not sure what gamma_n is supposed to be, but this seems reasonable numerically
            // float gamma_n = GAMMA_N;
            float dampingTermX = -GAMMA_N * alpha_h_bar * deltaX_dot;
            float dampingTermY = -GAMMA_N * alpha_h_bar * deltaY_dot;
            float dampingTermZ = -GAMMA_N * alpha_h_bar * deltaZ_dot;

            // Add damping term to spring term, write back to counter
            bodyA_X_velCorr += springTermX + dampingTermX;
            bodyA_Y_velCorr += springTermY + dampingTermY;
            bodyA_Z_velCorr += springTermZ + dampingTermZ;
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
                            sphere_Y_DOT[bodyA], sphere_Z_DOT[bodyA], bodyA_X_velCorr, bodyA_Y_velCorr,
                            bodyA_Z_velCorr);
            // If the sphere belongs to this SD, add up the gravitational force component.

            bodyA_X_velCorr += alpha_h_bar * gravAcc_X_d_factor_SU;
            bodyA_Y_velCorr += alpha_h_bar * gravAcc_Y_d_factor_SU;
            bodyA_Z_velCorr += alpha_h_bar * gravAcc_Z_d_factor_SU;
        }

        // Write the velocity updates back to global memory so that we can apply them AFTER this kernel finishes
        atomicAdd(d_sphere_pos_X_update + mySphereID, bodyA_X_velCorr);
        atomicAdd(d_sphere_pos_Y_update + mySphereID, bodyA_Y_velCorr);
        atomicAdd(d_sphere_pos_Z_update + mySphereID, bodyA_Z_velCorr);
    }
    __syncthreads();
}

template <unsigned int THRDS_PER_BLOCK>  //!< Number of CUB threads engaged in block-collective CUB operations.
                                         //!< Should be a multiple of 32
__global__ void updatePositions(unsigned int alpha_h_bar,  //!< The numerical integration time step
                                int* d_sphere_pos_X,       //!< Pointer to array containing data related to the
                                                           //!< spheres in the box
                                int* d_sphere_pos_Y,       //!< Pointer to array containing data related to the
                                                           //!< spheres in the box
                                int* d_sphere_pos_Z,       //!< Pointer to array containing data related to the
                                                           //!< spheres in the box
                                float* d_sphere_pos_X_dt,  //!< Pointer to array containing data related to
                                                           //!< the spheres in the box
                                float* d_sphere_pos_Y_dt,  //!< Pointer to array containing data related to
                                                           //!< the spheres in the box
                                float* d_sphere_pos_Z_dt,  //!< Pointer to array containing data related to
                                                           //!< the spheres in the box
                                unsigned int* SD_countsOfSpheresTouching,  //!< The array that for each
                                                                           //!< SD indicates how many
                                                                           //!< spheres touch this SD
                                unsigned int* spheres_in_SD_composite,     //!< Big array that works in conjunction
                                                                           //!< with SD_countsOfSpheresTouching.
                                                                           //!< "spheres_in_SD_composite" says which
                                                                           //!< SD contains what spheres
                                unsigned int nSpheres) {
    int xSphCenter;
    int ySphCenter;
    int zSphCenter;
    /// Set aside shared memory
    volatile __shared__ size_t offsetInComposite_SphInSD_Array[THRDS_PER_BLOCK * 8];
    volatile __shared__ bool shMem_head_flags[THRDS_PER_BLOCK * 8];

    typedef cub::BlockRadixSort<unsigned int, THRDS_PER_BLOCK, 8, unsigned int> BlockRadixSortOP;
    __shared__ typename BlockRadixSortOP::TempStorage temp_storage_sort;

    typedef cub::BlockDiscontinuity<unsigned int, THRDS_PER_BLOCK> Block_Discontinuity;
    __shared__ typename Block_Discontinuity::TempStorage temp_storage_disc;

    // Figure out what sphereID this thread will handle. We work with a 1D block structure and a 1D grid structure
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned int sphIDs[8] = {mySphereID, mySphereID, mySphereID, mySphereID,
                              mySphereID, mySphereID, mySphereID, mySphereID};

    // This uses a lot of registers but is needed
    unsigned int SDsTouched[8] = {NULL_GRANULAR_ID, NULL_GRANULAR_ID, NULL_GRANULAR_ID, NULL_GRANULAR_ID,
                                  NULL_GRANULAR_ID, NULL_GRANULAR_ID, NULL_GRANULAR_ID, NULL_GRANULAR_ID};
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

        figureOutTouchedSD(xSphCenter, ySphCenter, zSphCenter, SDsTouched);
    }

    __syncthreads();

    // Sort by the ID of the SD touched
    BlockRadixSortOP(temp_storage_sort).Sort(SDsTouched, sphIDs);
    __syncthreads();

    // Do a winningStreak search on whole block, might not have high utilization here
    bool head_flags[8];
    Block_Discontinuity(temp_storage_disc).FlagHeads(head_flags, SDsTouched, cub::Inequality());
    __syncthreads();

    // Write back to shared memory; eight-way bank conflicts here - to revisit later
    for (unsigned int i = 0; i < 8; i++) {
        shMem_head_flags[8 * threadIdx.x + i] = head_flags[i];
    }

    // Seed offsetInComposite_SphInSD_Array with "no valid ID" so that we know later on what is legit;
    // No shmem bank coflicts here, good access...
    for (unsigned int i = 0; i < 8; i++) {
        offsetInComposite_SphInSD_Array[i * THRDS_PER_BLOCK + threadIdx.x] = NULL_GRANULAR_ID_LONG;
    }

    __syncthreads();

    // Count how many times an SD shows up in conjunction with the collection of THRDS_PER_BLOCK spheres. There
    // will be some thread divergence here.
    // Loop through each potential SD, after sorting, and see if it is the start of a head
    for (unsigned int i = 0; i < 8; i++) {
        // SD currently touched, could easily be inlined
        unsigned int touchedSD = SDsTouched[i];
        if (touchedSD != NULL_GRANULAR_ID && head_flags[i]) {
            // current index into shared datastructure of length 8*THRDS_PER_BLOCK, could easily be inlined
            unsigned int idInShared = 8 * threadIdx.x + i;
            unsigned int winningStreak = 0;
            // This is the beginning of a sequence of SDs with a new ID
            do {
                winningStreak++;
                // Go until we run out of threads on the warp or until we find a new head
            } while (idInShared + winningStreak < 8 * THRDS_PER_BLOCK &&
                     !(shMem_head_flags[idInShared + winningStreak]));

            if (touchedSD >= d_box_L_SU * d_box_D_SU * d_box_H_SU) {
                printf("invalid SD index %u on thread %u\n", mySphereID, touchedSD);
            }

            // Store start of new entries, this can be a very large number
            size_t offset = atomicAdd(SD_countsOfSpheresTouching + touchedSD, winningStreak);

            // The value offset now gives a *relative* offset in the composite array; i.e., spheres_in_SD_composite.
            // Get the absolute offset
            offset += ((size_t)touchedSD) * MAX_COUNT_OF_DEs_PER_SD;

            // Produce the offsets for this streak of spheres with identical SD ids
            for (unsigned int i = 0; i < winningStreak; i++)
                offsetInComposite_SphInSD_Array[idInShared + i] = offset++;
        }
    }

    __syncthreads();  // needed since we write to shared memory above; i.e., offsetInComposite_SphInSD_Array

    const size_t max_composite_index = (size_t)d_box_D_SU * d_box_L_SU * d_box_H_SU * MAX_COUNT_OF_DEs_PER_SD;

    // Write out the data now; reister with spheres_in_SD_composite each sphere that touches a certain ID
    // what is happening is anything real?
    for (unsigned int i = 0; i < 8; i++) {
        size_t offset = offsetInComposite_SphInSD_Array[8 * threadIdx.x + i];
        if (offset != NULL_GRANULAR_ID_LONG) {
            if (offset >= max_composite_index) {
                ABORTABORTABORT(
                    "overrun during updatePositions on thread %u block %u, offset is %zu, max is %zu,  sphere is %u\n",
                    threadIdx.x, blockIdx.x, offset, max_composite_index, sphIDs[i]);
            } else {
                spheres_in_SD_composite[offset] = sphIDs[i];
            }
        }
    }
}
/// Copy most constant data to device, this should run at start
__host__ void chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless::copyCONSTdata_to_device() {
    // Copy quantities expressed in SU units for the SD dimensions to device
    gpuErrchk(cudaMemcpyToSymbol(d_SD_Ldim_SU, &SD_L_SU, sizeof(d_SD_Ldim_SU)));
    gpuErrchk(cudaMemcpyToSymbol(d_SD_Ddim_SU, &SD_D_SU, sizeof(d_SD_Ddim_SU)));
    gpuErrchk(cudaMemcpyToSymbol(d_SD_Hdim_SU, &SD_H_SU, sizeof(d_SD_Hdim_SU)));
    // Copy global BD size in multiples of SDs to device
    gpuErrchk(cudaMemcpyToSymbol(d_box_L_SU, &nSDs_L_SU, sizeof(d_box_L_SU)));
    gpuErrchk(cudaMemcpyToSymbol(d_box_D_SU, &nSDs_D_SU, sizeof(d_box_D_SU)));
    gpuErrchk(cudaMemcpyToSymbol(d_box_H_SU, &nSDs_H_SU, sizeof(d_box_H_SU)));

    gpuErrchk(cudaMemcpyToSymbol(psi_T_dFactor, &psi_T_Factor, sizeof(psi_T_Factor)));
    gpuErrchk(cudaMemcpyToSymbol(psi_h_dFactor, &psi_h_Factor, sizeof(psi_h_Factor)));
    gpuErrchk(cudaMemcpyToSymbol(psi_L_dFactor, &psi_L_Factor, sizeof(psi_L_Factor)));

    gpuErrchk(cudaMemcpyToSymbol(gravAcc_X_d_factor_SU, &gravAcc_X_factor_SU, sizeof(gravAcc_X_factor_SU)));
    gpuErrchk(cudaMemcpyToSymbol(gravAcc_Y_d_factor_SU, &gravAcc_Y_factor_SU, sizeof(gravAcc_Y_factor_SU)));
    gpuErrchk(cudaMemcpyToSymbol(gravAcc_Z_d_factor_SU, &gravAcc_Z_factor_SU, sizeof(gravAcc_Z_factor_SU)));

    gpuErrchk(
        cudaMemcpyToSymbol(d_monoDisperseSphRadius_SU, &monoDisperseSphRadius_SU, sizeof(d_monoDisperseSphRadius_SU)));
    gpuErrchk(cudaMemcpyToSymbol(d_DE_Mass, &MASS_UNIT, sizeof(MASS_UNIT)));
}

/// Similar to the copyCONSTdata_to_device, but saves us a big copy
/// This can run at every timestep to allow a moving BD
__host__ void chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless::copyBD_Frame_to_device() {
    gpuErrchk(cudaMemcpyToSymbol(d_BD_frame_X, &BD_frame_X, sizeof(BD_frame_X)));
    gpuErrchk(cudaMemcpyToSymbol(d_BD_frame_Y, &BD_frame_Y, sizeof(BD_frame_Y)));
    gpuErrchk(cudaMemcpyToSymbol(d_BD_frame_Z, &BD_frame_Z, sizeof(BD_frame_Z)));

    // gpuErrchk(cudaMemcpyToSymbol(d_BD_frame_X_dot, &BD_frame_X_dot, sizeof(BD_frame_X_dot)));
    // gpuErrchk(cudaMemcpyToSymbol(d_BD_frame_Y_dot, &BD_frame_Y_dot, sizeof(BD_frame_Y_dot)));
    // gpuErrchk(cudaMemcpyToSymbol(d_BD_frame_Z_dot, &BD_frame_Z_dot, sizeof(BD_frame_Z_dot)));
}

/// Copy positions and velocities back to host
void chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless::copyDataBackToHost() {
    // Copy back positions
    // gpuErrchk(cudaMemcpy(pos_X.data(), p_d_CM_X, nDEs * sizeof(int), cudaMemcpyDeviceToHost));
    // gpuErrchk(cudaMemcpy(pos_Y.data(), p_d_CM_Y, nDEs * sizeof(int), cudaMemcpyDeviceToHost));
    // gpuErrchk(cudaMemcpy(pos_Z.data(), p_d_CM_Z, nDEs * sizeof(int), cudaMemcpyDeviceToHost));
    // gpuErrchk(cudaMemcpy(pos_X_dt.data(), pos_X_dt.data(), nDEs * sizeof(int), cudaMemcpyDeviceToHost));
    // gpuErrchk(cudaMemcpy(pos_Y_dt.data(), p_d_CM_YDOT, nDEs * sizeof(int), cudaMemcpyDeviceToHost));
    // gpuErrchk(cudaMemcpy(pos_Z_dt.data(), p_d_CM_ZDOT, nDEs * sizeof(int), cudaMemcpyDeviceToHost));
}

// Check number of spheres in each SD and dump relevant info to file
void chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless::checkSDCounts(std::string ofile,
                                                                                    bool write_out = false,
                                                                                    bool verbose = false) {
    // copyDataBackToHost();
    // Count of DEs in each SD
    unsigned int* sdvals = SD_NumOf_DEs_Touching.data();
    // DEs that are in each SD
    unsigned int* sdSpheres = DEs_in_SD_composite.data();
    // # times each DE appears in some SD
    unsigned int* deCounts = new unsigned int[nDEs];

    // Copy back broadphase-related data
    // gpuErrchk(cudaMemcpy(sdvals, SD_NumOf_DEs_Touching, nSDs * sizeof(unsigned int),
    // cudaMemcpyDeviceToHost)); gpuErrchk(cudaMemcpy(sdSpheres, DEs_in_SD_composite, MAX_COUNT_OF_DEs_PER_SD *
    // nSDs * sizeof(unsigned int),
    //                      cudaMemcpyDeviceToHost));

    // could use memset instead, just need to zero these out
    for (unsigned int i = 0; i < nDEs; i++) {
        deCounts[i] = 0;
    }

    unsigned int max_count = 0;
    unsigned int sum = 0;
    for (unsigned int i = 0; i < nSDs; i++) {
        // printf("count is %u for SD sd %u \n", sdvals[i], i);
        sum += sdvals[i];
        if (sdvals[i] > max_count)
            max_count = sdvals[i];
    }
    // safety checks, if these fail we were probably about to crash
    assert(sum < MAX_COUNT_OF_DEs_PER_SD * nSDs);
    assert(max_count < MAX_COUNT_OF_DEs_PER_SD);
    if (verbose) {
        printf("max DEs per SD is %u\n", max_count);
        printf("total sd/de overlaps is %u\n", sum);
        printf("theoretical total is %u\n", MAX_COUNT_OF_DEs_PER_SD * nSDs);
    }
    // Copy over occurences in SDs
    for (unsigned int i = 0; i < MAX_COUNT_OF_DEs_PER_SD * nSDs; i++) {
        // printf("de id is %d, i is %u\n", sdSpheres[i], i);
        // Check if invalid sphere
        if (sdSpheres[i] == NULL_GRANULAR_ID) {
            // printf("invalid sphere in sd");
        } else {
            assert(sdSpheres[i] < nDEs);
            deCounts[sdSpheres[i]]++;
        }
    }
    if (write_out) {
        writeFile(ofile, deCounts);
    }
    // delete[] sdvals;
    // delete[] sdSpheres;
    delete[] deCounts;
}

void chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless::writeFile(std::string ofile,
                                                                                unsigned int* deCounts) {
    // copyDataBackToHost();
    // unnecessary if called by checkSDCounts()
    // The file writes are a pretty big slowdown in CSV mode
    if (file_write_mode == GRN_OUTPUT_MODE::BINARY) {
        // Write the data as binary to a file, requires later postprocessing that can be done in parallel, this is a
        // much faster write due to no formatting
        std::ofstream ptFile(ofile + ".raw", std::ios::out | std::ios::binary);

        for (unsigned int n = 0; n < nDEs; n++) {
            float absv = sqrt(pos_X_dt.at(n) * pos_X_dt.at(n) + pos_Y_dt.at(n) * pos_Y_dt.at(n) +
                              pos_Z_dt.at(n) * pos_Z_dt.at(n));

            ptFile.write((const char*)&pos_X.at(n), sizeof(int));
            ptFile.write((const char*)&pos_Y.at(n), sizeof(int));
            ptFile.write((const char*)&pos_Z.at(n), sizeof(int));
            ptFile.write((const char*)&pos_X_dt.at(n), sizeof(float));
            ptFile.write((const char*)&pos_Y_dt.at(n), sizeof(float));
            ptFile.write((const char*)&pos_Z_dt.at(n), sizeof(float));
            ptFile.write((const char*)&absv, sizeof(float));
            ptFile.write((const char*)&deCounts[n], sizeof(int));
        }
    } else if (file_write_mode == GRN_OUTPUT_MODE::CSV) {
        // CSV is much slower but requires less postprocessing
        std::ofstream ptFile(ofile + ".csv", std::ios::out);

        // Dump to a stream, write to file only at end
        std::ostringstream outstrstream;
        outstrstream << "x,y,z,vx,vy,vz,absv,nTouched\n";

        for (unsigned int n = 0; n < nDEs; n++) {
            float absv = sqrt(pos_X_dt.at(n) * pos_X_dt.at(n) + pos_Y_dt.at(n) * pos_Y_dt.at(n) +
                              pos_Z_dt.at(n) * pos_Z_dt.at(n));
            outstrstream << pos_X.at(n) << "," << pos_Y.at(n) << "," << pos_Z.at(n) << "," << pos_X_dt.at(n) << ","
                         << pos_Y_dt.at(n) << "," << pos_Z_dt.at(n) << "," << absv << "," << deCounts[n] << "\n";
        }

        ptFile << outstrstream.str();
    } else if (file_write_mode == GRN_OUTPUT_MODE::NONE) {
        // Do nothing, only here for symmetry
    }
}

void chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless::updateBDPosition(const int currTime_SU,
                                                                                       const int stepSize_SU) {
    float timeUU = (1.f * currTime_SU * TIME_UNIT * PSI_h);
    // Frequency of oscillation
    // float frame_X_old = BD_frame_X;
    // float frame_Y_old = BD_frame_Y;
    // float frame_Z_old = BD_frame_Z;
    // Put the bottom-left corner of box wherever the user told us to
    // std::cout << "Time is " << timeUU << std::endl;
    BD_frame_X = (box_L * (BDPositionFunctionX(timeUU))) / LENGTH_UNIT;
    BD_frame_Y = (box_D * (BDPositionFunctionY(timeUU))) / LENGTH_UNIT;
    BD_frame_Z = (box_H * (BDPositionFunctionZ(timeUU))) / LENGTH_UNIT;
    // printf("old X is %f, new is %d \n", frame_X_old, BD_frame_X);
    // velocity is time derivative of position, use a first-order approximation to avoid continuity issues
    // NOTE that as of 4/16/2018, the wall damping term is disabled due to some instability
    // BD_frame_X_dot = (BD_frame_X - frame_X_old) / (1.f * stepSize_SU);
    // BD_frame_Y_dot = (BD_frame_Y - frame_Y_old) / (1.f * stepSize_SU);
    // BD_frame_Z_dot = (BD_frame_Z - frame_Z_old) / (1.f * stepSize_SU);

    copyBD_Frame_to_device();
}

__host__ void chrono::granular::ChSystemGranularMonodisperse_SMC_Frictionless::settle(float tEnd) {
    switch_to_SimUnits();
    generate_DEs();

    // Set aside memory for holding data structures worked with. Get some initializations going
    setup_simulation();
    copyCONSTdata_to_device();
    copyBD_Frame_to_device();
    gpuErrchk(cudaDeviceSynchronize());

    // Seed arrays that are populated by the kernel call
    // const unsigned char allBitsOne = (unsigned char)-1;  // all bits of this variable are 1.
    // Set all the offsets to zero
    SD_NumOf_DEs_Touching.assign(SD_NumOf_DEs_Touching.size(), 0);
    DEs_in_SD_composite.assign(DEs_in_SD_composite.size(), NULL_GRANULAR_ID);

    // Figure our the number of blocks that need to be launched to cover the box
    unsigned int nBlocks = (nDEs + CUDA_THREADS - 1) / CUDA_THREADS;
    printf("doing priming!\n");
    printf("max possible composite offset is %zu\n",
           (size_t)d_box_D_SU * d_box_L_SU * d_box_H_SU * MAX_COUNT_OF_DEs_PER_SD);

    primingOperationsRectangularBox<CUDA_THREADS><<<nBlocks, CUDA_THREADS>>>(
        pos_X.data(), pos_Y.data(), pos_Z.data(), SD_NumOf_DEs_Touching.data(), DEs_in_SD_composite.data(), nDEs);
    gpuErrchk(cudaDeviceSynchronize());
    // Check in first timestep
    checkSDCounts(output_directory + "/step000000", true, false);

    // Settling simulation loop.
    unsigned int stepSize_SU = 5;
    unsigned int tEnd_SU = std::ceil(tEnd / (TIME_UNIT * PSI_h));
    // Which timestep is it?
    unsigned int currstep = 0;
    // Which frame am I rendering?
    unsigned int currframe = 1;

    unsigned int nsteps = (1.0 * tEnd_SU) / stepSize_SU;
    // If we are just doing priming, stop now
    if (nsteps == 0) {
        cleanup_simulation();
        return;
    }

    printf("going until %u at timestep %u, %u timesteps at approx timestep %f\n", tEnd_SU, stepSize_SU, nsteps,
           tEnd / nsteps);
    printf("z grav term with timestep %u is %f\n", stepSize_SU, stepSize_SU * stepSize_SU * gravAcc_Z_factor_SU);

    float fps = 50;
    // Number of frames to render
    int nFrames = fps * tEnd;
    // number of steps to go before rendering a frame
    int rendersteps = nsteps > 0 ? nsteps / nFrames : 0;

    // Run the simulation, there are aggressive synchronizations because we want to have no race conditions
    for (unsigned int crntTime_SU = 0; crntTime_SU < stepSize_SU * nsteps; crntTime_SU += stepSize_SU) {
        // Update the position and velocity of the BD, if relevant
        if (!BD_is_fixed) {
            updateBDPosition(crntTime_SU, stepSize_SU);
        }
        // reset forces to zero, note that vel update ~ force for forward euler
        pos_X_dt_update.assign(pos_X_dt_update.size(), 0);
        pos_Y_dt_update.assign(pos_Y_dt_update.size(), 0);
        pos_Z_dt_update.assign(pos_Z_dt_update.size(), 0);

        // gpuErrchk(cudaDeviceSynchronize());

        // Compute forces and crank into vel updates, we have 2 kernels to avoid a race condition
        computeVelocityUpdates<MAX_COUNT_OF_DEs_PER_SD><<<nSDs, MAX_COUNT_OF_DEs_PER_SD>>>(
            stepSize_SU, pos_X.data(), pos_Y.data(), pos_Z.data(), pos_X_dt_update.data(), pos_Y_dt_update.data(),
            pos_Z_dt_update.data(), SD_NumOf_DEs_Touching.data(), DEs_in_SD_composite.data(), pos_X_dt.data(),
            pos_Y_dt.data(), pos_Z_dt.data());
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        // Apply the updates we just made
        applyVelocityUpdates<MAX_COUNT_OF_DEs_PER_SD><<<nSDs, MAX_COUNT_OF_DEs_PER_SD>>>(
            stepSize_SU, pos_X.data(), pos_Y.data(), pos_Z.data(), pos_X_dt_update.data(), pos_Y_dt_update.data(),
            pos_Z_dt_update.data(), SD_NumOf_DEs_Touching.data(), DEs_in_SD_composite.data(), pos_X_dt.data(),
            pos_Y_dt.data(), pos_Z_dt.data());

        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        // Reset broadphase information
        SD_NumOf_DEs_Touching.assign(SD_NumOf_DEs_Touching.size(), 0);
        DEs_in_SD_composite.assign(DEs_in_SD_composite.size(), NULL_GRANULAR_ID);

        updatePositions<CUDA_THREADS><<<nBlocks, CUDA_THREADS>>>(
            stepSize_SU, pos_X.data(), pos_Y.data(), pos_Z.data(), pos_X_dt.data(), pos_Y_dt.data(), pos_Z_dt.data(),
            SD_NumOf_DEs_Touching.data(), DEs_in_SD_composite.data(), nDEs);

        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());
        // we don't want to render at every timestep, the file write is painful
        currstep++;
        if (currstep % rendersteps == 0) {
            printf("currstep is %u, rendering frame %u\n", currstep, currframe);

            char filename[100];
            sprintf(filename, "%s/step%06d", output_directory.c_str(), currframe++);
            checkSDCounts(std::string(filename), true, false);
        }
    }
    printf("radius is %u\n", monoDisperseSphRadius_SU);
    // Don't write but print verbosely
    checkSDCounts("", false, true);

    cleanup_simulation();
    return;
}
