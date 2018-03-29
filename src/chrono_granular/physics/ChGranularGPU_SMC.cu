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
#include <iostream>
#include <sstream>
#include <string>
#include "../../chrono_thirdparty/cub/cub.cuh"
#include "../ChGranularDefines.h"
#include "../chrono_granular/physics/ChGranular.h"
#include "chrono/core/ChVector.h"
#include "chrono_granular/physics/ChGranularDefines.cuh"
#include "chrono_granular/utils/ChGranularUtilities_CUDA.cuh"

__constant__ unsigned int d_monoDisperseSphRadius_SU;  //!< Radius of the sphere, expressed in SU

#define MAX_X_POS_UNSIGNED (d_SD_Ldim_SU * d_box_L_SU)
#define MAX_Y_POS_UNSIGNED (d_SD_Ddim_SU * d_box_D_SU)
#define MAX_Z_POS_UNSIGNED (d_SD_Hdim_SU * d_box_H_SU)

#define GAMMA_N .004

__constant__ unsigned int d_SD_Ldim_SU;    //!< Ad-ed L-dimension of the SD box
__constant__ unsigned int d_SD_Ddim_SU;    //!< Ad-ed D-dimension of the SD box
__constant__ unsigned int d_SD_Hdim_SU;    //!< Ad-ed H-dimension of the SD box
__constant__ unsigned int psi_T_dFactor;   //!< factor used in establishing the software-time-unit
__constant__ unsigned int psi_h_dFactor;   //!< factor used in establishing the software-time-unit
__constant__ unsigned int psi_L_dFactor;   //!< factor used in establishing the software-time-unit
__constant__ unsigned int d_box_L_SU;      //!< Ad-ed L-dimension of the BD box in multiples of subdomains
__constant__ unsigned int d_box_D_SU;      //!< Ad-ed D-dimension of the BD box in multiples of subdomains
__constant__ unsigned int d_box_H_SU;      //!< Ad-ed H-dimension of the BD box in multiples of subdomains
__constant__ float gravAcc_X_d_factor_SU;  //!< Device counterpart of the constant gravAcc_X_factor_SU
__constant__ float gravAcc_Y_d_factor_SU;  //!< Device counterpart of the constant gravAcc_Y_factor_SU
__constant__ float gravAcc_Z_d_factor_SU;  //!< Device counterpart of the constant gravAcc_Z_factor_SU
__constant__ double d_DE_Mass;

// Decide which SD owns this DE.
// Returns the id of the SD that holds the center of the sphere
__device__ unsigned int figureOutOwnerSD(int sphCenter_X, int sphCenter_Y, int sphCenter_Z) {
    signed int sphCenter_X_modified = (d_box_L_SU * d_SD_Ldim_SU) / 2 + sphCenter_X;
    signed int sphCenter_Y_modified = (d_box_D_SU * d_SD_Ddim_SU) / 2 + sphCenter_Y;
    signed int sphCenter_Z_modified = (d_box_H_SU * d_SD_Hdim_SU) / 2 + sphCenter_Z;
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
    signed int sphCenter_X_modified = (d_box_L_SU * d_SD_Ldim_SU) / 2 + sphCenter_X;
    signed int sphCenter_Y_modified = (d_box_D_SU * d_SD_Ddim_SU) / 2 + sphCenter_Y;
    signed int sphCenter_Z_modified = (d_box_H_SU * d_SD_Hdim_SU) / 2 + sphCenter_Z;
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
        printf("x is too large, boundary is %u, n is %u, nmax is %u, pos is %d, mod is %d, max is %d, dim is %d\n",
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
            printf("UH OH!, sd overrun %u, boundary is %u on thread %u, block %u, n is %u, %u, %u\n", SDs[i], boundary,
                   threadIdx.x, blockIdx.x, n[0], n[1], n[2]);
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
    int* pRawDataX,                            //!< Pointer to array containing data related to the spheres in the box
    int* pRawDataY,                            //!< Pointer to array containing data related to the spheres in the box
    int* pRawDataZ,                            //!< Pointer to array containing data related to the spheres in the box
    unsigned int* SD_countsOfSpheresTouching,  //!< The array that for each SD indicates how many spheres touch this SD
    unsigned int* spheres_in_SD_composite,     //!< Big array that works in conjunction with SD_countsOfSpheresTouching.
                                               //!< "spheres_in_SD_composite" says which SD contains what spheres
    unsigned int nSpheres                      //!< Number of spheres in the box
) {
    int xSphCenter;
    int ySphCenter;
    int zSphCenter;

    /// Set aside shared memory
    volatile __shared__ unsigned int offsetInComposite_SphInSD_Array[CUB_THREADS * 8];
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
        xSphCenter = pRawDataX[mySphereID];
        ySphCenter = pRawDataY[mySphereID];
        zSphCenter = pRawDataZ[mySphereID];

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
        offsetInComposite_SphInSD_Array[i * CUB_THREADS + threadIdx.x] = NULL_GRANULAR_ID;
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
            unsigned int offset = atomicAdd(SD_countsOfSpheresTouching + touchedSD, winningStreak);

            // The value offset now gives a *relative* offset in the composite array; i.e., spheres_in_SD_composite.
            // Get the absolute offset
            offset += touchedSD * MAX_COUNT_OF_DEs_PER_SD;

            // Produce the offsets for this streak of spheres with identical SD ids
            for (unsigned int i = 0; i < winningStreak; i++)
                offsetInComposite_SphInSD_Array[idInShared + i] = offset++;
        }
    }

    __syncthreads();  // needed since we write to shared memory above; i.e., offsetInComposite_SphInSD_Array

    // Write out the data now; reister with spheres_in_SD_composite each sphere that touches a certain ID
    for (unsigned int i = 0; i < 8; i++) {
        unsigned int offset = offsetInComposite_SphInSD_Array[8 * threadIdx.x + i];
        if (offset != NULL_GRANULAR_ID)
            spheres_in_SD_composite[offset] = sphIDs[i];
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
                                const int sphXvel,        //!< Global X velocity of DE
                                const int sphYvel,        //!< Global Y velocity of DE
                                const int sphZvel,        //!< Global Z velocity of DE
                                double& X_Vel_corr,       //!< Velocity correction in Xdir
                                double& Y_Vel_corr,       //!< Velocity correction in Xdir
                                double& Z_Vel_corr        //!< Velocity correction in Xdir
) {
    signed int sphXpos_modified = (d_box_L_SU * d_SD_Ldim_SU) / 2 + sphXpos;
    signed int sphYpos_modified = (d_box_D_SU * d_SD_Ddim_SU) / 2 + sphYpos;
    signed int sphZpos_modified = (d_box_H_SU * d_SD_Hdim_SU) / 2 + sphZpos;
    // unsigned int boundary = sphCenter_X_modified - (signed int)d_monoDisperseSphRadius_SU <= 0 ||
    //                         sphCenter_X_modified + (signed int)d_monoDisperseSphRadius_SU >= MAX_X_POS_UNSIGNED ||
    //                         sphCenter_Y_modified - (signed int)d_monoDisperseSphRadius_SU <= 0 ||
    //                         sphCenter_Y_modified + (signed int)d_monoDisperseSphRadius_SU >= MAX_Y_POS_UNSIGNED ||
    //                         sphCenter_Z_modified - (signed int)d_monoDisperseSphRadius_SU <= 0 ||
    //                         sphCenter_Z_modified + (signed int)d_monoDisperseSphRadius_SU >= MAX_Z_POS_UNSIGNED;

    signed int pen;
    int touchingWall;
    // Compute spring factor in force
    float scalingFactor = (1.0f * alpha_h_bar) / (psi_T_dFactor * psi_T_dFactor * psi_h_dFactor);
    // This gamma_n is fake, needs to be given by user
    float dampingCoeff = -GAMMA_N * alpha_h_bar;

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
    xcomp += touchingWall * (scalingFactor * abs(pen) - dampingCoeff * sphXvel);

    // Do top X wall
    pen = MAX_Y_POS_UNSIGNED - (sphXpos_modified + (signed int)d_monoDisperseSphRadius_SU);
    touchingWall = (pen < 0) && abs(pen) < d_monoDisperseSphRadius_SU;
    // in this case, pen is positive and we want a negative restorative force
    xcomp += -1 * touchingWall * (scalingFactor * abs(pen) - dampingCoeff * sphXvel);

    // penetration of sphere into relevant wall
    pen = sphYpos_modified - (signed int)d_monoDisperseSphRadius_SU;
    // true if sphere touching wall
    touchingWall = (pen < 0) && abs(pen) < d_monoDisperseSphRadius_SU;
    // in this case, pen is negative and we want a positive restorative force
    ycomp += touchingWall * (scalingFactor * abs(pen) - dampingCoeff * sphYvel);

    // Do top y wall
    pen = MAX_Y_POS_UNSIGNED - (sphYpos_modified + (signed int)d_monoDisperseSphRadius_SU);
    touchingWall = (pen < 0) && abs(pen) < d_monoDisperseSphRadius_SU;
    // in this case, pen is positive and we want a negative restorative force
    ycomp += -1 * touchingWall * (scalingFactor * abs(pen) - dampingCoeff * sphYvel);

    // penetration of sphere into relevant wall
    pen = sphZpos_modified - (signed int)d_monoDisperseSphRadius_SU;
    // true if sphere touching wall
    touchingWall = (pen < 0) && abs(pen) < d_monoDisperseSphRadius_SU;
    // in this case, pen is negative and we want a positive restorative force
    zcomp += touchingWall * (scalingFactor * abs(pen) - dampingCoeff * sphZvel);

    // Do top z wall
    pen = MAX_Z_POS_UNSIGNED - (sphZpos_modified + (signed int)d_monoDisperseSphRadius_SU);
    touchingWall = (pen < 0) && abs(pen) < d_monoDisperseSphRadius_SU;
    // in this case, pen is positive and we want a negative restorative force
    zcomp += -1 * touchingWall * (scalingFactor * abs(pen) - dampingCoeff * sphZvel);

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
    unsigned int alpha_h_bar,   //!< Value that controls actual step size, not actually needed for this kernel
    int* pRawDataX,             //!< Pointer to array containing data related to the
                                //!< spheres in the box
    int* pRawDataY,             //!< Pointer to array containing data related to the
                                //!< spheres in the box
    int* pRawDataZ,             //!< Pointer to array containing data related to the
                                //!< spheres in the box
    int* pRawDataX_DOT_update,  //!< Pointer to array containing data related to
                                //!< the spheres in the box
    int* pRawDataY_DOT_update,  //!< Pointer to array containing data related to
                                //!< the spheres in the box
    int* pRawDataZ_DOT_update,  //!< Pointer to array containing data related to
                                //!< the spheres in the box
    unsigned int* SD_countsOfSpheresTouching,  //!< The array that for each
                                               //!< SD indicates how many
                                               //!< spheres touch this SD
    unsigned int* spheres_in_SD_composite,     //!< Big array that works in conjunction
                                               //!< with SD_countsOfSpheresTouching.

    int* pRawDataX_DOT,
    int* pRawDataY_DOT,
    int* pRawDataZ_DOT) {
    unsigned int thisSD = blockIdx.x;
    unsigned int spheresTouchingThisSD = SD_countsOfSpheresTouching[thisSD];
    unsigned mySphereID;  // Bring in data from global into shmem. Only a subset of threads get to do this.
    if (threadIdx.x < spheresTouchingThisSD) {
        mySphereID = spheres_in_SD_composite[thisSD * MAX_NSPHERES_PER_SD + threadIdx.x];

        unsigned int ownerSD = figureOutOwnerSD(pRawDataX[mySphereID], pRawDataY[mySphereID], pRawDataZ[mySphereID]);
        // Each SD applies force updates to the bodies it *owns*, this should happen once for each body
        if (thisSD == ownerSD) {
            // Probably does not need to be atomic, but no conflicts means it won't be too slow anyways
            atomicAdd(pRawDataX_DOT + mySphereID, pRawDataX_DOT_update[mySphereID]);
            atomicAdd(pRawDataY_DOT + mySphereID, pRawDataY_DOT_update[mySphereID]);
            atomicAdd(pRawDataZ_DOT + mySphereID, pRawDataZ_DOT_update[mySphereID]);
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
__global__ void computeVelocityUpdates(unsigned int alpha_h_bar,   //!< Value that controls actual step size.
                                       int* pRawDataX,             //!< Pointer to array containing data related to the
                                                                   //!< spheres in the box
                                       int* pRawDataY,             //!< Pointer to array containing data related to the
                                                                   //!< spheres in the box
                                       int* pRawDataZ,             //!< Pointer to array containing data related to the
                                                                   //!< spheres in the box
                                       int* pRawDataX_DOT_update,  //!< Pointer to array containing data related to
                                                                   //!< the spheres in the box
                                       int* pRawDataY_DOT_update,  //!< Pointer to array containing data related to
                                                                   //!< the spheres in the box
                                       int* pRawDataZ_DOT_update,  //!< Pointer to array containing data related to
                                                                   //!< the spheres in the box
                                       unsigned int* SD_countsOfSpheresTouching,  //!< The array that for each
                                                                                  //!< SD indicates how many
                                                                                  //!< spheres touch this SD
                                       unsigned int* spheres_in_SD_composite  //!< Big array that works in conjunction
                                                                              //!< with SD_countsOfSpheresTouching.
                                                                              //
                                       ,
                                       int* pRawDataX_DOT,
                                       int* pRawDataY_DOT,
                                       int* pRawDataZ_DOT) {
    // Cache positions and velocities in shared memory
    __shared__ int sph_X[MAX_NSPHERES_PER_SD];
    __shared__ int sph_Y[MAX_NSPHERES_PER_SD];
    __shared__ int sph_Z[MAX_NSPHERES_PER_SD];
    __shared__ int sph_X_DOT[MAX_NSPHERES_PER_SD];
    __shared__ int sph_Y_DOT[MAX_NSPHERES_PER_SD];
    __shared__ int sph_Z_DOT[MAX_NSPHERES_PER_SD];
    // unsigned char bodyB_list[12 * MAX_NSPHERES_PER_SD];  // NOTE: max number of spheres that can kiss a sphere
    // is 12.
    unsigned int thisSD = blockIdx.x;
    unsigned int spheresTouchingThisSD = SD_countsOfSpheresTouching[thisSD];
    unsigned mySphereID;

    // Bring in data from global into shmem. Only a subset of threads get to do this.
    if (threadIdx.x < spheresTouchingThisSD) {
        mySphereID = spheres_in_SD_composite[thisSD * MAX_NSPHERES_PER_SD + threadIdx.x];
        sph_X[threadIdx.x] = pRawDataX[mySphereID];
        sph_Y[threadIdx.x] = pRawDataY[mySphereID];
        sph_Z[threadIdx.x] = pRawDataZ[mySphereID];
        sph_X_DOT[threadIdx.x] = pRawDataX_DOT[mySphereID];
        sph_Y_DOT[threadIdx.x] = pRawDataY_DOT[mySphereID];
        sph_Z_DOT[threadIdx.x] = pRawDataZ_DOT[mySphereID];
    }

    __syncthreads();  // Needed to make sure data gets in shmem before using it elsewhere

    // Assumes each thread is a body, not the greatest assumption but we can fix that later
    // Note that if we have more threads than bodies, some effort gets wasted.
    unsigned int bodyA = threadIdx.x;

    // The update we make to the velocities
    double bodyA_X_velCorr = 0.f;
    double bodyA_Y_velCorr = 0.f;
    double bodyA_Z_velCorr = 0.f;

    // k * delta_t
    double scalingFactor = (1.0 * alpha_h_bar) / (1.0 * psi_T_dFactor * psi_T_dFactor * psi_h_dFactor);

    // If we overran, we have a major issue, time to crash before we make illegal memory accesses
    if (spheresTouchingThisSD > MAX_NSPHERES_PER_SD) {
        printf("TOO MANY SPHERES! %u\n", spheresTouchingThisSD);
        // Crash now
        assert(spheresTouchingThisSD < MAX_NSPHERES_PER_SD);
    }

    // NOTE that below here I used double precision because I didn't know how much precision I needed. Reducing the
    // amount of doubles will certainly speed this up

    // Each body looks at each other body and computes the force that the other body exerts on it
    if (bodyA < spheresTouchingThisSD) {
        double sphdiameter = 2. * d_monoDisperseSphRadius_SU;
        double invSphDiameter = 1. / sphdiameter;
        unsigned int ownerSD = figureOutOwnerSD(sph_X[bodyA], sph_Y[bodyA], sph_Z[bodyA]);
        double penetration = 0;
        for (unsigned char bodyB = 0; bodyB < spheresTouchingThisSD; bodyB++) {
            // unsigned int theirSphID = spheres_in_SD_composite[thisSD * MAX_NSPHERES_PER_SD + bodyB];
            // Don't check for collision with self
            if (bodyA == bodyB)
                continue;

            // NOTE below here, it seems faster to do coalesced shared mem accesses than caching sph_X[bodyA] and the
            // like in a register
            // This avoids computing a square to figure our if collision or not
            double deltaX = (sph_X[bodyA] - sph_X[bodyB]) * invSphDiameter;
            double deltaY = (sph_Y[bodyA] - sph_Y[bodyB]) * invSphDiameter;
            double deltaZ = (sph_Z[bodyA] - sph_Z[bodyB]) * invSphDiameter;

            double deltaX_dot = sph_X_DOT[bodyA] - sph_X_DOT[bodyB];
            double deltaY_dot = sph_Y_DOT[bodyA] - sph_Y_DOT[bodyB];
            double deltaZ_dot = sph_Z_DOT[bodyA] - sph_Z_DOT[bodyB];

            penetration = deltaX * deltaX;
            penetration += deltaY * deltaY;
            penetration += deltaZ * deltaZ;

            // Here we need to check if the contact point is in this SD.

            // Take spatial average of positions to get position of contact point
            double contactX = (1.0 * (sph_X[bodyB] + sph_X[bodyA])) / 2.0;
            double contactY = (1.0 * (sph_Y[bodyB] + sph_Y[bodyA])) / 2.0;
            double contactZ = (1.0 * (sph_Z[bodyB] + sph_Z[bodyA])) / 2.0;

            unsigned int contactSD = figureOutOwnerSD(contactX, contactY, contactZ);
            // We need to make sure we don't count a collision twice -- if a contact pair is in multiple SDs, count it
            // only in the one that holds the contact point
            // We have a collision here...
            if (penetration < 1 && contactSD == thisSD) {
                // Note: this can be accelerated should we decide to go w/ float. Then we can use the CUDA
                // intrinsic:
                // __device__ ​ float rnormf ( int  dim, const float* a)
                // http://docs.nvidia.com/cuda/cuda-math-api/group__CUDA__MATH__SINGLE.html#group__CUDA__MATH__SINGLE

                // Compute penetration term, this becomes the delta as we want it
                penetration = sqrt(penetration);
                penetration = 1 / penetration;
                penetration -= 1.;

                // Compute nondim force term

                // These lines use forward euler right now. Each has an alpha_h_bar term but we can use a different
                // method simply by changing the following computations
                // Compute force updates
                double springTermX = scalingFactor * deltaX * sphdiameter * penetration / d_DE_Mass;
                double springTermY = scalingFactor * deltaY * sphdiameter * penetration / d_DE_Mass;
                double springTermZ = scalingFactor * deltaZ * sphdiameter * penetration / d_DE_Mass;
                // Not sure what gamma_n is supposed to be, but this seems reasonable numerically
                double gamma_n = GAMMA_N;
                double dampingTermX = -gamma_n * alpha_h_bar * deltaX_dot;
                double dampingTermY = -gamma_n * alpha_h_bar * deltaY_dot;
                double dampingTermZ = -gamma_n * alpha_h_bar * deltaZ_dot;

                // Add damping term to spring term, write back to counter
                bodyA_X_velCorr += springTermX + dampingTermX;
                bodyA_Y_velCorr += springTermY + dampingTermY;
                bodyA_Z_velCorr += springTermZ + dampingTermZ;
            }
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
            boxWallsEffects(alpha_h_bar, sph_X[bodyA], sph_Y[bodyA], sph_Z[bodyA], sph_X_DOT[bodyA], sph_Y_DOT[bodyA],
                            sph_Z_DOT[bodyA], bodyA_X_velCorr, bodyA_Y_velCorr, bodyA_Z_velCorr);
            // If the sphere belongs to this SD, add up the gravitational force component.
            bodyA_X_velCorr += alpha_h_bar * gravAcc_X_d_factor_SU;
            bodyA_Y_velCorr += alpha_h_bar * gravAcc_Y_d_factor_SU;
            bodyA_Z_velCorr += alpha_h_bar * gravAcc_Z_d_factor_SU;
        }

        // Write the velocity updates back to global memory so that we can apply them AFTER this kernel finishes
        atomicAdd(pRawDataX_DOT_update + mySphereID, (int)bodyA_X_velCorr);
        atomicAdd(pRawDataY_DOT_update + mySphereID, (int)bodyA_Y_velCorr);
        atomicAdd(pRawDataZ_DOT_update + mySphereID, (int)bodyA_Z_velCorr);
    }
    __syncthreads();
}

template <unsigned int THRDS_PER_BLOCK>  //!< Number of CUB threads engaged in block-collective CUB operations.
                                         //!< Should be a multiple of 32
__global__ void updatePositions(unsigned int alpha_h_bar,  //!< The numerical integration time step
                                int* pRawDataX,            //!< Pointer to array containing data related to the
                                                           //!< spheres in the box
                                int* pRawDataY,            //!< Pointer to array containing data related to the
                                                           //!< spheres in the box
                                int* pRawDataZ,            //!< Pointer to array containing data related to the
                                                           //!< spheres in the box
                                int* pRawDataX_DOT,        //!< Pointer to array containing data related to
                                                           //!< the spheres in the box
                                int* pRawDataY_DOT,        //!< Pointer to array containing data related to
                                                           //!< the spheres in the box
                                int* pRawDataZ_DOT,        //!< Pointer to array containing data related to
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
    // NOTE from Conlain -- somebody in this kernel is trashing heap memory and breaking things
    /// Set aside shared memory
    volatile __shared__ unsigned int offsetInComposite_SphInSD_Array[THRDS_PER_BLOCK * 8];
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
        xSphCenter = alpha_h_bar * pRawDataX_DOT[mySphereID];
        ySphCenter = alpha_h_bar * pRawDataY_DOT[mySphereID];
        zSphCenter = alpha_h_bar * pRawDataZ_DOT[mySphereID];
        // if (xSphCenter != 0) {
        //     printf("nonzero velocity update in xdir for spher %u, vel %d, update %d\n", mySphereID,
        //            pRawDataX_DOT[mySphereID], xSphCenter);
        // }

        // if (xSphCenter >= (signed int)d_SD_Ldim_SU * d_box_L_SU) {
        //     printf("x way too big is %d, vel is %d, h is %d\n", xSphCenter, pRawDataX_DOT[mySphereID],
        //     alpha_h_bar);
        // }

        xSphCenter += pRawDataX[mySphereID];
        pRawDataX[mySphereID] = xSphCenter;

        ySphCenter += pRawDataY[mySphereID];
        pRawDataY[mySphereID] = ySphCenter;

        zSphCenter += pRawDataZ[mySphereID];
        pRawDataZ[mySphereID] = zSphCenter;

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
        offsetInComposite_SphInSD_Array[i * THRDS_PER_BLOCK + threadIdx.x] = NULL_GRANULAR_ID;
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

            // if (touchedSD >= d_box_L_SU * d_box_D_SU * d_box_H_SU) {
            //     printf("invalid SD index %u on thread %u\n", mySphereID, touchedSD);
            // }

            // Store start of new entries
            unsigned int offset = atomicAdd(SD_countsOfSpheresTouching + touchedSD, winningStreak);

            // The value offset now gives a *relative* offset in the composite array; i.e., spheres_in_SD_composite.
            // Get the absolute offset
            offset += touchedSD * MAX_COUNT_OF_DEs_PER_SD;

            // if (offset != NULL_GRANULAR_ID &&
            //     offset >= d_box_D_SU * d_box_L_SU * d_box_H_SU * MAX_COUNT_OF_DEs_PER_SD) {
            //     printf("overrun calculated on sd %u, streak is %u on thread %u block %u, offset is %u, sphere is
            //     %u\n",
            //            touchedSD, winningStreak, threadIdx.x, blockIdx.x, offset, sphIDs[i]);
            //     // printf("%u SDs calculated\n", d_box_D_SU * d_box_L_SU * d_box_H_SU);
            // }
            // Produce the offsets for this streak of spheres with identical SD ids
            for (unsigned int i = 0; i < winningStreak; i++)
                offsetInComposite_SphInSD_Array[idInShared + i] = offset++;
        }
    }

    __syncthreads();  // needed since we write to shared memory above; i.e., offsetInComposite_SphInSD_Array

    // Write out the data now; reister with spheres_in_SD_composite each sphere that touches a certain ID
    // what is happening is anything real?
    for (unsigned int i = 0; i < 8; i++) {
        unsigned int offset = offsetInComposite_SphInSD_Array[8 * threadIdx.x + i];
        if (offset != NULL_GRANULAR_ID) {
            if (offset >= d_box_D_SU * d_box_L_SU * d_box_H_SU * MAX_COUNT_OF_DEs_PER_SD) {
                printf("overrun on thread %u block %u, offset is %u, sphere is %u\n", threadIdx.x, blockIdx.x, offset,
                       sphIDs[i]);
            } else {
                spheres_in_SD_composite[offset] = sphIDs[i];
            }
        }
    }
}

__host__ void chrono::ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC::copyCONSTdata_to_device() {
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

void chrono::ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC::copyDataBackToHost() {
    // Copy back positions
    gpuErrchk(cudaMemcpy(h_X_DE.data(), p_d_CM_X, nDEs * sizeof(int), cudaMemcpyDeviceToHost));
    gpuErrchk(cudaMemcpy(h_Y_DE.data(), p_d_CM_Y, nDEs * sizeof(int), cudaMemcpyDeviceToHost));
    gpuErrchk(cudaMemcpy(h_Z_DE.data(), p_d_CM_Z, nDEs * sizeof(int), cudaMemcpyDeviceToHost));
    gpuErrchk(cudaMemcpy(h_XDOT_DE.data(), p_d_CM_XDOT, nDEs * sizeof(int), cudaMemcpyDeviceToHost));
    gpuErrchk(cudaMemcpy(h_YDOT_DE.data(), p_d_CM_YDOT, nDEs * sizeof(int), cudaMemcpyDeviceToHost));
    gpuErrchk(cudaMemcpy(h_ZDOT_DE.data(), p_d_CM_ZDOT, nDEs * sizeof(int), cudaMemcpyDeviceToHost));
}

// Check number of spheres in each SD and dump relevant info to file
void chrono::ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC::checkSDCounts(std::string ofile,
                                                                 bool write_out = false,
                                                                 bool verbose = false) {
    copyDataBackToHost();
    // Count of DEs in each SD
    unsigned int* sdvals = new unsigned int[nSDs];
    // DEs that are in each SD
    unsigned int* sdSpheres = new unsigned int[MAX_COUNT_OF_DEs_PER_SD * nSDs];
    // # times each DE appears in some SD
    unsigned int* deCounts = new unsigned int[nDEs];

    gpuErrchk(cudaMemcpy(sdvals, p_device_SD_NumOf_DEs_Touching, nSDs * sizeof(unsigned int), cudaMemcpyDeviceToHost));

    gpuErrchk(cudaMemcpy(sdSpheres, p_device_DEs_in_SD_composite, MAX_COUNT_OF_DEs_PER_SD * nSDs * sizeof(unsigned int),
                         cudaMemcpyDeviceToHost));

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
    delete[] sdvals;
    delete[] sdSpheres;
    delete[] deCounts;
}

void chrono::ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC::writeFile(std::string ofile, unsigned int* deCounts) {
    copyDataBackToHost();
    std::ofstream ptFile{ofile};
    std::ostringstream outstrstream;
    outstrstream << "x,y,z,vx,vy,vz,absv,nTouched" << std::endl;
    for (unsigned int n = 0; n < nDEs; n++) {
        // Easy cheat -- make a vector and then get the length. Probably super slow
        // Actually not that slow
        ChVector<float> velVec(h_XDOT_DE.at(n), h_YDOT_DE.at(n), h_ZDOT_DE.at(n));
        outstrstream << h_X_DE.at(n) << "," << h_Y_DE.at(n) << "," << h_Z_DE.at(n) << "," << h_XDOT_DE.at(n) << ","
                     << h_YDOT_DE.at(n) << "," << h_ZDOT_DE.at(n) << "," << velVec.Length() << "," << deCounts[n]
                     << std::endl;
    }
    ptFile << outstrstream.str();
}
__host__ void chrono::ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC::settle(float tEnd) {
    switch_to_SimUnits();
    generate_DEs();
    printf("radius is %u\n", monoDisperseSphRadius_SU);

    // Set aside memory for holding data structures worked with. Get some initializations going
    setup_simulation();
    copyCONSTdata_to_device();
    gpuErrchk(cudaDeviceSynchronize());

    // Seed arrays that are populated by the kernel call
    const unsigned char allBitsOne = (unsigned char)-1;  // all bits of this variable are 1.
    // Set all the offsets to zero
    gpuErrchk(cudaMemset(p_device_SD_NumOf_DEs_Touching, 0, nSDs * sizeof(unsigned int)));
    // For each SD, all the spheres touching that SD should have their ID be NULL_GRANULAR_ID
    gpuErrchk(
        cudaMemset(p_device_DEs_in_SD_composite, allBitsOne, MAX_COUNT_OF_DEs_PER_SD * nSDs * sizeof(unsigned int)));

    /// Figure our the number of blocks that need to be launched to cover the box
    unsigned int nBlocks = (nDEs + CUDA_THREADS - 1) / CUDA_THREADS;
    printf("doing priming!\n");

    primingOperationsRectangularBox<CUDA_THREADS><<<nBlocks, CUDA_THREADS>>>(
        p_d_CM_X, p_d_CM_Y, p_d_CM_Z, p_device_SD_NumOf_DEs_Touching, p_device_DEs_in_SD_composite, nDEs);
    gpuErrchk(cudaDeviceSynchronize());
    // Check in first timestep
    checkSDCounts("step000000.csv", true);
    // printf("counts checked\n");
    // Settling simulation loop.
    unsigned int stepSize_SU = 5;
    unsigned int tEnd_SU = std::ceil(tEnd / TIME_UNIT);
    // Which timestep is it?
    unsigned int currstep = 0;
    // Which frame am I rendering?
    unsigned int currframe = 1;
    printf("going until %u at timestep %u\n", tEnd_SU, stepSize_SU);
    unsigned int nsteps = 50;
    nsteps = (1.0 * tEnd_SU) / stepSize_SU;
    // need to store velocity updates, this should go as a class member variable
    int* p_d_CM_XDOT_update;
    int* p_d_CM_YDOT_update;
    int* p_d_CM_ZDOT_update;

    // Set aside memory for velocity update information
    gpuErrchk(cudaMalloc(&p_d_CM_XDOT_update, nDEs * sizeof(int)));
    gpuErrchk(cudaMalloc(&p_d_CM_YDOT_update, nDEs * sizeof(int)));
    gpuErrchk(cudaMalloc(&p_d_CM_ZDOT_update, nDEs * sizeof(int)));

    // Set initial velocity updates to zero
    gpuErrchk(cudaMemset(p_d_CM_XDOT_update, 0, nDEs * sizeof(int)));
    gpuErrchk(cudaMemset(p_d_CM_YDOT_update, 0, nDEs * sizeof(int)));
    gpuErrchk(cudaMemset(p_d_CM_ZDOT_update, 0, nDEs * sizeof(int)));

    // Go to 100 timesteps so we can get a quick run at this point
    for (unsigned int crntTime_SU = 0; crntTime_SU < stepSize_SU * nsteps; crntTime_SU += stepSize_SU) {
        printf("currstep is %u\n", ++currstep);

        // reset forces to zero
        gpuErrchk(cudaMemset(p_d_CM_XDOT_update, 0, nDEs * sizeof(int)));
        gpuErrchk(cudaMemset(p_d_CM_YDOT_update, 0, nDEs * sizeof(int)));
        gpuErrchk(cudaMemset(p_d_CM_ZDOT_update, 0, nDEs * sizeof(int)));

        computeVelocityUpdates<MAX_COUNT_OF_DEs_PER_SD><<<nSDs, MAX_COUNT_OF_DEs_PER_SD>>>(
            stepSize_SU, p_d_CM_X, p_d_CM_Y, p_d_CM_Z, p_d_CM_XDOT_update, p_d_CM_YDOT_update, p_d_CM_ZDOT_update,
            p_device_SD_NumOf_DEs_Touching, p_device_DEs_in_SD_composite, p_d_CM_XDOT, p_d_CM_YDOT, p_d_CM_ZDOT);
        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());
        applyVelocityUpdates<MAX_COUNT_OF_DEs_PER_SD><<<nSDs, MAX_COUNT_OF_DEs_PER_SD>>>(
            stepSize_SU, p_d_CM_X, p_d_CM_Y, p_d_CM_Z, p_d_CM_XDOT_update, p_d_CM_YDOT_update, p_d_CM_ZDOT_update,
            p_device_SD_NumOf_DEs_Touching, p_device_DEs_in_SD_composite, p_d_CM_XDOT, p_d_CM_YDOT, p_d_CM_ZDOT);
        // checkSDCounts("step2.csv", true);

        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());

        gpuErrchk(cudaMemset(p_device_SD_NumOf_DEs_Touching, 0, nSDs * sizeof(unsigned int)));
        gpuErrchk(cudaMemset(p_device_DEs_in_SD_composite, allBitsOne,
                             MAX_COUNT_OF_DEs_PER_SD * nSDs * sizeof(unsigned int)));

        updatePositions<CUDA_THREADS><<<nBlocks, CUDA_THREADS>>>(
            stepSize_SU, p_d_CM_X, p_d_CM_Y, p_d_CM_Z, p_d_CM_XDOT, p_d_CM_YDOT, p_d_CM_ZDOT,
            p_device_SD_NumOf_DEs_Touching, p_device_DEs_in_SD_composite, nDEs);

        gpuErrchk(cudaPeekAtLastError());
        gpuErrchk(cudaDeviceSynchronize());
        if (currstep % 10 == 0) {
            char filename[100];
            sprintf(filename, "results/step%06d.csv", currframe++);
            checkSDCounts(std::string(filename), true, false);
        }
        // writeFile(filename);
    }
    printf("radius is %u\n", monoDisperseSphRadius_SU);
    // Don't write but print verbosely
    checkSDCounts("", false, true);

    cleanup_simulation();
    return;
}
