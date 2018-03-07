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
#include <cstdio>
#include <fstream>
#include "../../chrono_thirdparty/cub/cub.cuh"
#include "../ChGranularDefines.h"
#include "../chrono_granular/physics/ChGranular.h"
#include "chrono_granular/physics/ChGranularDefines.cuh"
#include "chrono_granular/utils/ChGranularUtilities_CUDA.cuh"

__constant__ unsigned int d_monoDisperseSphRadius_AD;  // Pulled from the header

__constant__ unsigned int d_SD_Ldim_AD;  //!< Ad-ed L-dimension of the SD box
__constant__ unsigned int d_SD_Ddim_AD;  //!< Ad-ed D-dimension of the SD box
__constant__ unsigned int d_SD_Hdim_AD;  //!< Ad-ed H-dimension of the SD box

__constant__ unsigned int d_box_L_AD;  //!< Ad-ed L-dimension of the BD box in multiples of subdomains
__constant__ unsigned int d_box_D_AD;  //!< Ad-ed D-dimension of the BD box in multiples of subdomains
__constant__ unsigned int d_box_H_AD;  //!< Ad-ed H-dimension of the BD box in multiples of subdomains

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
    int sphCenter_X_modified = (d_box_L_AD * d_SD_Ldim_AD) / 2 + sphCenter_X;
    int sphCenter_Y_modified = (d_box_D_AD * d_SD_Ddim_AD) / 2 + sphCenter_Y;
    int sphCenter_Z_modified = (d_box_H_AD * d_SD_Hdim_AD) / 2 + sphCenter_Z;
    int n[3];
    // TODO this doesn't handle if the ball is slightly penetrating the boundary, could result in negative values or end
    // GIDs beyond bounds. We might want to do a check to see if it's outside and set 'valid' accordingly
    // NOTE: This is integer arithmetic to compute the floor. We want to get the first SD below the sphere
    // nx = (xCenter - radius) / wx .
    n[0] = (sphCenter_X_modified - d_monoDisperseSphRadius_AD) / d_SD_Ldim_AD;
    // Same for D and H
    n[1] = (sphCenter_Y_modified - d_monoDisperseSphRadius_AD) / d_SD_Ddim_AD;
    n[2] = (sphCenter_Z_modified - d_monoDisperseSphRadius_AD) / d_SD_Hdim_AD;
    // Find distance from next box in relevant dir to center, we may be straddling the two
    int d[3];                                                 // Store penetrations
    d[0] = (n[0] + 1) * d_SD_Ldim_AD - sphCenter_X_modified;  // dx = (nx + 1)* wx - x
    d[1] = (n[1] + 1) * d_SD_Ddim_AD - sphCenter_Y_modified;
    d[2] = (n[2] + 1) * d_SD_Hdim_AD - sphCenter_Z_modified;

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
        SDs[i] += (n[0] + (i & 0x1)) * d_box_D_AD * d_box_H_AD;
        // s == own[e] evals true if the current SD is owner
        // If both touch it or we own it, the result is valid
        valid &= (abs(d[0]) < d_monoDisperseSphRadius_AD) || ((i & 0x1) == (d[0] < 0));

        // High/low in y-dir
        // s = i & 0x2; // Inlined now
        // Scale to global index and add to total
        SDs[i] += (n[1] + ((i >> 1) & 0x1)) * d_box_H_AD;
        // If both touch it or we own it, the result is valid
        valid &= (abs(d[1]) < d_monoDisperseSphRadius_AD) || (((i >> 1) & 0x1) == (d[1] < 0));

        // High/low in z-dir
        // s = i & 0x4; // Inlined now
        // Scale to global index and add to total
        SDs[i] += (n[2] + ((i >> 2) & 0x1));
        // If both touch it or we own it, the result is valid
        valid &= (abs(d[2]) < d_monoDisperseSphRadius_AD) || (((i >> 2) & 0x1) == (d[2] < 0));

        // This ternary is hopefully better than a conditional
        // If valid is false, then the SD is actually NULL_GRANULAR_ID
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

            // if (touchedSD >= d_box_L_AD * d_box_D_AD * d_box_H_AD) {
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
/// Count the number of contacts for each body
/// TODO this does more work than strictly necessary, but is much cleaner
/// we could probably get up 2x speedup on this function call by some cute mapping, but that would require a global map
template <unsigned int MAX_NSPHERES_PER_SD>
__device__ unsigned int dryRunContactCount(unsigned tIdx,
                                           const int sph_X[MAX_NSPHERES_PER_SD],
                                           const int sph_Y[MAX_NSPHERES_PER_SD],
                                           const int sph_Z[MAX_NSPHERES_PER_SD]) {
    unsigned int ncontacts = 0;  // We return this value
    // This function call returns the number of contacts
    unsigned int sphere1 = tIdx;
    for (unsigned int sphere2 = 0; sphere2 < MAX_NSPHERES_PER_SD; sphere2++) {
        // Check both sphere for legal values
        // If either sphere is invalid or the sphere to check has lower index, skip this check
        // Store boolean as uint because why not
        unsigned int invalid =
            (sph_X[sphere1] == ILL_GRANULAR_VAL) || (sph_X[sphere2] == ILL_GRANULAR_VAL) || (sphere1 >= sphere2);
        unsigned int dx = (sph_X[sphere1] - sph_X[sphere2]);
        unsigned int dy = (sph_Y[sphere1] - sph_Y[sphere2]);
        unsigned int dz = (sph_Z[sphere1] - sph_Z[sphere2]);
        unsigned int d2 = dx * dx + dy * dy + dz * dz;
        // True if bodies are in contact, true->1 in c++
        unsigned int contact = (d2 < d_monoDisperseSphRadius_AD * d_monoDisperseSphRadius_AD);
        // Store so we can return it later
        ncontacts += contact && !invalid;
    }

    return ncontacts;
}

// Do the same work as dryrun, but load the sphere IDs this time
template <unsigned int MAX_NSPHERES_PER_SD>
__device__ void populateContactEventInformation_dataStructures(
    unsigned tIdx,
    const int sph_X[MAX_NSPHERES_PER_SD],
    const int sph_Y[MAX_NSPHERES_PER_SD],
    const int sph_Z[MAX_NSPHERES_PER_SD],
    unsigned int thisThrdOffset,
    unsigned int thisThrdCollisionCount,
    volatile unsigned char IDfrstDE_inCntctEvent[MAX_NSPHERES_PER_SD * AVERAGE_COUNT_CONTACTS_PER_DE],
    volatile unsigned char IDscndDE_inCntctEvent[MAX_NSPHERES_PER_SD * AVERAGE_COUNT_CONTACTS_PER_DE]) {
    unsigned int count = 0;  // use this as index into shared memory
    // This function call returns the number of contacts
    unsigned int sphere1 = tIdx;
    // this code shouldn't diverge too much, at worst it will run the 'else' of the if case MAX_NSPHERES_PER_SD time
    for (unsigned int sphere2 = 0; sphere2 < MAX_NSPHERES_PER_SD; sphere2++) {
        // Check both sphere for legal values
        // If either sphere is invalid or the sphere to check has lower index, skip this check
        // Store boolean as uint because it sneaks past conditional checks that way
        unsigned int invalid =
            (sph_X[sphere1] == ILL_GRANULAR_VAL) || (sph_X[sphere2] == ILL_GRANULAR_VAL) || (sphere1 >= sphere2);
        unsigned int dx = (sph_X[sphere1] - sph_X[sphere2]);
        unsigned int dy = (sph_Y[sphere1] - sph_Y[sphere2]);
        unsigned int dz = (sph_Z[sphere1] - sph_Z[sphere2]);
        unsigned int d2 = dx * dx + dy * dy + dz * dz;
        // True if bodies are in contact, true->1 in c++
        unsigned int contact = (d2 < d_monoDisperseSphRadius_AD * d_monoDisperseSphRadius_AD);
        // This is warp divergence but it shouldn't be the _worst_
        if (contact && !invalid) {
            // Write back to shared memory
            IDfrstDE_inCntctEvent[tIdx + count] = sphere1;
            IDscndDE_inCntctEvent[tIdx + count] = sphere2;
            // Increment counter
            count++;
            if (count > thisThrdCollisionCount) {
                printf("BIG BIG ERROR! too many collisions detected!\n");
            }
        }
    }
}

/**
This device function computes the forces induces by the walls on the box on a sphere
Input:
  - sphXpos: X location, measured in the box reference system, of the sphere
  - sphYpos: Y location, measured in the box reference system, of the sphere
  - sphZpos: Z location, measured in the box reference system, of the sphere

Output:
  - Xforce: the X component of the force, as represented in the box reference system
  - Yforce: the Y component of the force, as represented in the box reference system
  - Zforce: the Z component of the force, as represented in the box reference system
*/
template <unsigned int MAX_NSPHERES_PER_SD>
__device__ void boxWallsInducedForce(int sphXpos,
                                     int sphYpos,
                                     int sphZpos,
                                     volatile int& Xforce,
                                     volatile int& Yforce,
                                     volatile int& Zforce) {
    Xforce = ILL_GRANULAR_VAL;
    Yforce = ILL_GRANULAR_VAL;
    Zforce = ILL_GRANULAR_VAL;
}

/**
This device function figures out how many contact events the thread "thrdIndx" needs to take care of.
Input:
    - tIdx: the thread for which we identify the work order
    - blockLvlCollisionEventsCount: the total number of contact events the entire block needs to deal with
Output:
    - myColsnCount: the number of contact events that thread thrdIndx will have to deal with
    - my_offset: offset in the collision data structure where this thread starts
*/
template <unsigned int MAX_NSPHERES_PER_SD>
__device__ void figureOutWorkOrder(unsigned int tIdx,
                                   unsigned int blockLvlCollisionEventsCount,
                                   unsigned int* myColsnCount,
                                   unsigned int* my_offset) {
    const unsigned int num_threads = blockDim.x;  // Get this info
    // We want to calculate the number of collisions per thread, but not undershoot
    unsigned int collisions_per_thread = (blockLvlCollisionEventsCount + num_threads - 1) / num_threads;
    // the number of extra collisions we picked up
    unsigned int rem = num_threads * collisions_per_thread - blockLvlCollisionEventsCount;
    // Offset into total collisions, this should be the same for everyone
    *my_offset = tIdx * collisions_per_thread;
    // Last rem threads should do less work
    // Should be little warp divergence since we're only doing one check and this is a tiny function anyways
    if (tIdx >= num_threads - rem) {
        // Don't do the extra work on this thread
        collisions_per_thread -= 1;
    }
    // Store this thread's work to do
    *myColsnCount = collisions_per_thread;
}

/**
This device function computes the normal force between two spheres that are in contact.
Input:
- delX: the difference in the x direction between the the two spheres; i.e., x_A - x_B
- delY: the difference in the y direction between the the two spheres; i.e., y_A - y_B
- delZ: the difference in the z direction between the the two spheres; i.e., z_A - z_B
Output:
- sphA_Xforce: the force that sphere A "feels" in the x direction
- sphA_Yforce: the force that sphere A "feels" in the y direction
- sphA_Zforce: the force that sphere A "feels" in the z direction
*/
__device__ void computeNormalForce(const int& delX,
                                   const int& delY,
                                   const int& delZ,
                                   int& sphA_Xforce,
                                   int& sphA_Yforce,
                                   int& sphA_Zforce) {
    sphA_Xforce = ILL_GRANULAR_VAL;
    sphA_Yforce = ILL_GRANULAR_VAL;
    sphA_Zforce = ILL_GRANULAR_VAL;
}

/**
This kernel call figures out forces on a sphere and carries out numerical integration to get the velocities of a sphere.

Template arguments:
  - MAX_NSPHERES_PER_SD: the number of threads used in this kernel, comes into play when invoking CUB block collectives.
                         NOTE: It is assumed that MAX_NSPHERES_PER_SD<256 (we are using in this kernel unsigned char to
store IDs)

Assumptions:
  - Granular material is made up of monodisperse spheres.
  - The function below assumes the spheres are in a box
  - The box has dimensions L x D x H.
  - The reference frame associated with the box:
      - The x-axis is along the length L of the box
      - The y-axis is along the width D of the box
      - The z-axis is along the height H of the box
  - A sphere cannot touch more than eight SDs

Basic idea: use domain decomposition on the rectangular box and figure out how many SDs each sphere touches.
The subdomains are axis-aligned relative to the reference frame associated with the *box*. The origin of the box is
at the center of the box. The orientation of the box is defined relative to a world inertial reference frame.

Nomenclature:
  - SD: subdomain.
  - NULL_GRANULAR_ID: the equivalent of a non-sphere SD ID, or a non-sphere ID

Notes:
  - The SD with ID=0 is the catch-all SD. This is the SD in which a sphere ends up if its not inside the rectangular
box. Usually, there is no sphere in this SD (THIS IS NOT IMPLEMENTED AS SUCH FOR NOW)
*/
template <unsigned int MAX_NSPHERES_PER_SD>  //!< Number of CUB threads engaged in block-collective CUB operations.
                                             //!< Should be a multiple of 32
__global__ void updateVelocities(unsigned int timeStep,  //!< The numerical integration time step
                                 int* pRawDataX,         //!< Pointer to array containing data related to the
                                                         //!< spheres in the box
                                 int* pRawDataY,         //!< Pointer to array containing data related to the
                                                         //!< spheres in the box
                                 int* pRawDataZ,         //!< Pointer to array containing data related to the
                                                         //!< spheres in the box
                                 int* pRawDataX_DOT,     //!< Pointer to array containing data related to
                                                         //!< the spheres in the box
                                 int* pRawDataY_DOT,     //!< Pointer to array containing data related to
                                                         //!< the spheres in the box
                                 int* pRawDataZ_DOT,     //!< Pointer to array containing data related to
                                                         //!< the spheres in the box
                                 unsigned int* SD_countsOfSpheresTouching,  //!< The array that for each
                                                                            //!< SD indicates how many
                                                                            //!< spheres touch this SD
                                 unsigned int* spheres_in_SD_composite      //!< Big array that works in
                                                                            //!< conjunction with
                                 //!< SD_countsOfSpheresTouching.
                                 )  //!< "spheres_in_SD_composite" says which SD contains what spheres
{
    __shared__ int sph_X[MAX_NSPHERES_PER_SD];
    __shared__ int sph_Y[MAX_NSPHERES_PER_SD];
    __shared__ int sph_Z[MAX_NSPHERES_PER_SD];

    volatile __shared__ int sph_Xforce[MAX_NSPHERES_PER_SD];
    volatile __shared__ int sph_Yforce[MAX_NSPHERES_PER_SD];
    volatile __shared__ int sph_Zforce[MAX_NSPHERES_PER_SD];

    volatile __shared__ unsigned char ID_frstDE_inCntctEvent[MAX_NSPHERES_PER_SD * AVERAGE_COUNT_CONTACTS_PER_DE];
    volatile __shared__ unsigned char ID_scndDE_inCntctEvent[MAX_NSPHERES_PER_SD * AVERAGE_COUNT_CONTACTS_PER_DE];

    unsigned int mySphere[1];

    typedef cub::BlockRadixSort<unsigned int, MAX_NSPHERES_PER_SD, 1> BlockRadixSortOP;
    __shared__ typename BlockRadixSortOP::TempStorage temp_storage_sort;

    unsigned int spheresTouchingThisSD = SD_countsOfSpheresTouching[blockIdx.x];
    unsigned int dummyUINT01 = blockIdx.x * MAX_NSPHERES_PER_SD;
    mySphere[0] = spheres_in_SD_composite[dummyUINT01 + threadIdx.x];

    // In an attempt to improve likelihood of coalesced mem accesses, do a sort. This will change mySphere.
    BlockRadixSortOP(temp_storage_sort).Sort(mySphere);

    // Bring in data from global into sh mem; compute force impressed by walls on this sphere
    if (threadIdx.x < spheresTouchingThisSD) {
        sph_X[threadIdx.x] = pRawDataX[mySphere[0]];
        sph_Y[threadIdx.x] = pRawDataY[mySphere[0]];
        sph_Z[threadIdx.x] = pRawDataZ[mySphere[0]];
        boxWallsInducedForce<MAX_NSPHERES_PER_SD>(sph_X[threadIdx.x], sph_Y[threadIdx.x], sph_Z[threadIdx.x],
                                                  sph_Xforce[threadIdx.x], sph_Yforce[threadIdx.x],
                                                  sph_Zforce[threadIdx.x]);
    } else {
        sph_X[threadIdx.x] = ILL_GRANULAR_VAL;
        sph_Y[threadIdx.x] = ILL_GRANULAR_VAL;
        sph_Z[threadIdx.x] = ILL_GRANULAR_VAL;

        sph_Xforce[threadIdx.x] = 0;
        sph_Yforce[threadIdx.x] = 0;
        sph_Zforce[threadIdx.x] = 0;
    }

    __syncthreads();

    // Figure out sphere-to-sphere forces
    // Dry run first, in order to figure out offsets into shmem
    unsigned int myCollisionCount = dryRunContactCount<MAX_NSPHERES_PER_SD>(threadIdx.x, sph_X, sph_Y, sph_Z);
    unsigned int myOffset = myCollisionCount;
    unsigned int blockLevelCollisionEventsCount = 0;
    typedef cub::BlockScan<unsigned int, MAX_NSPHERES_PER_SD> BlockScan;
    __shared__ typename BlockScan::TempStorage temp_storage_scan;
    BlockScan(temp_storage_scan).ExclusiveSum(myOffset, myOffset, blockLevelCollisionEventsCount);

    __syncthreads();  // Conlain added this -- do we need to force synchronization here?

    // Populate the data structures with contact event information; i.e., the first and second spheres of
    // each contact event
    populateContactEventInformation_dataStructures<MAX_NSPHERES_PER_SD>(
        threadIdx.x, sph_X, sph_Y, sph_Z, myOffset, myCollisionCount, ID_frstDE_inCntctEvent, ID_scndDE_inCntctEvent);

    __syncthreads();  // We just wrote to shared memory

    // Figure out which contact events this thread needs to deal with. This will change two variables: offset,
    // and myCollisionCount
    figureOutWorkOrder<MAX_NSPHERES_PER_SD>(threadIdx.x, blockLevelCollisionEventsCount, &myCollisionCount, &myOffset);

    // Go ahead and do
    for (unsigned int cntctEvent = myOffset; cntctEvent < myOffset + myCollisionCount; cntctEvent++) {
        unsigned int shMem_offset_sph_A = ID_frstDE_inCntctEvent[cntctEvent];
        unsigned int shMem_offset_sph_B = ID_scndDE_inCntctEvent[cntctEvent];
        int del_X = sph_X[shMem_offset_sph_A] - sph_X[shMem_offset_sph_B];
        int del_Y = sph_Y[shMem_offset_sph_A] - sph_Y[shMem_offset_sph_B];
        int del_Z = sph_Z[shMem_offset_sph_A] - sph_Z[shMem_offset_sph_B];

        // Here we would have several ways of computing the force. For now, assume a Hookean model; i.e., the simplest
        // possible.
        int sph_A_Xforce;
        int sph_A_Yforce;
        int sph_A_Zforce;
        computeNormalForce(del_X, del_Y, del_Z, sph_A_Xforce, sph_A_Yforce, sph_A_Zforce);

        // Update values in shmem for sphere A. This atomicAdd might be expensive...
        atomicAdd((int*)sph_Xforce + shMem_offset_sph_A, sph_A_Xforce);
        atomicAdd((int*)sph_Yforce + shMem_offset_sph_A, sph_A_Yforce);
        atomicAdd((int*)sph_Zforce + shMem_offset_sph_A, sph_A_Zforce);

        // Update values in shmem for sphere B. This atomicAdd might be expensive...
        atomicAdd((int*)sph_Xforce + shMem_offset_sph_B, -sph_A_Xforce);
        atomicAdd((int*)sph_Yforce + shMem_offset_sph_B, -sph_A_Yforce);
        atomicAdd((int*)sph_Zforce + shMem_offset_sph_B, -sph_A_Zforce);
    }

    __syncthreads();
    // NOTE from Conlain -- Explicit Euler is the worst, I vote Chung
    // Ready to do numerical integration. The assumption is that the mass is 1.0.
    // For now, do Explicit Euler. We should be able to do Chung here for improved dissipation. Or even Implicit Euler.
    atomicAdd(pRawDataX_DOT + mySphere[0], timeStep * sph_Xforce[threadIdx.x]);
    atomicAdd(pRawDataY_DOT + mySphere[0], timeStep * sph_Yforce[threadIdx.x]);
    atomicAdd(pRawDataZ_DOT + mySphere[0], timeStep * sph_Zforce[threadIdx.x]);
}

template <
    unsigned int
        THRDS_PER_BLOCK>  //!< Number of CUB threads engaged in block-collective CUB operations. Should be a multiple of
                          //!< 32
__global__ void
updatePositions(
    unsigned int timeStep,                     //!< The numerical integration time step
    int* pRawDataX,                            //!< Pointer to array containing data related to the spheres in the box
    int* pRawDataY,                            //!< Pointer to array containing data related to the spheres in the box
    int* pRawDataZ,                            //!< Pointer to array containing data related to the spheres in the box
    int* pRawDataX_DOT,                        //!< Pointer to array containing data related to the spheres in the box
    int* pRawDataY_DOT,                        //!< Pointer to array containing data related to the spheres in the box
    int* pRawDataZ_DOT,                        //!< Pointer to array containing data related to the spheres in the box
    unsigned int* SD_countsOfSpheresTouching,  //!< The array that for each SD indicates how many spheres touch this SD
    unsigned int* spheres_in_SD_composite,     //!< Big array that works in conjunction with
                                               //!< SD_countsOfSpheresTouching.
                                               //!< "spheres_in_SD_composite" says which SD
                                               //!< contains what spheres
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
        xSphCenter = timeStep * pRawDataX_DOT[mySphereID];
        xSphCenter += pRawDataX[mySphereID];
        pRawDataX[mySphereID] = xSphCenter;

        ySphCenter = timeStep * pRawDataY_DOT[mySphereID];
        ySphCenter += pRawDataY[mySphereID];
        pRawDataY[mySphereID] = ySphCenter;

        zSphCenter = timeStep * pRawDataZ_DOT[mySphereID];
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

            // if (touchedSD >= d_box_L_AD * d_box_D_AD * d_box_H_AD) {
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

__host__ void chrono::ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC::copyCONSTdata_to_device() {
    // Copy adimensional size of SDs to device
    gpuErrchk(cudaMemcpyToSymbol(d_SD_Ldim_AD, &SD_L_AD, sizeof(d_SD_Ldim_AD)));
    gpuErrchk(cudaMemcpyToSymbol(d_SD_Ddim_AD, &SD_D_AD, sizeof(d_SD_Ddim_AD)));
    gpuErrchk(cudaMemcpyToSymbol(d_SD_Hdim_AD, &SD_H_AD, sizeof(d_SD_Hdim_AD)));
    // Copy global BD size in multiples of SDs to device
    gpuErrchk(cudaMemcpyToSymbol(d_box_L_AD, &nSDs_L_AD, sizeof(d_box_L_AD)));
    gpuErrchk(cudaMemcpyToSymbol(d_box_D_AD, &nSDs_D_AD, sizeof(d_box_D_AD)));
    gpuErrchk(cudaMemcpyToSymbol(d_box_H_AD, &nSDs_H_AD, sizeof(d_box_H_AD)));

    gpuErrchk(
        cudaMemcpyToSymbol(d_monoDisperseSphRadius_AD, &monoDisperseSphRadius_AD, sizeof(d_monoDisperseSphRadius_AD)));
}

// Check number of spheres in each SD and dump relevant info to file
void chrono::ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC::checkSDCounts() {
    unsigned int* sdvals = new unsigned int[nSDs];
    unsigned int* sdSpheres = new unsigned int[MAX_COUNT_OF_DEs_PER_SD * nSDs];
    unsigned int* deCounts = new unsigned int[nDEs];
    cudaMemcpy(sdvals, p_device_SD_NumOf_DEs_Touching, nSDs * sizeof(unsigned int), cudaMemcpyDeviceToHost);
    cudaMemcpy(sdSpheres, p_device_DEs_in_SD_composite, MAX_COUNT_OF_DEs_PER_SD * nSDs * sizeof(unsigned int),
               cudaMemcpyDeviceToHost);

    unsigned int max_count = 0;
    unsigned int sum = 0;
    for (unsigned int i = 0; i < nSDs; i++) {
        // printf("count is %u for SD sd %u \n", sdvals[i], i);
        sum += sdvals[i];
        if (sdvals[i] > max_count)
            max_count = sdvals[i];
    }
    printf("max DEs per SD is %u\n", max_count);
    printf("total sd/de overlaps is %u\n", sum);
    printf("theoretical total is %u\n", MAX_COUNT_OF_DEs_PER_SD * nSDs);
    // Copy over occurences in SDs
    for (unsigned int i = 0; i < MAX_COUNT_OF_DEs_PER_SD * nSDs; i++) {
        // printf("de id is %u, i is %u\n", sdSpheres[i], i);
        deCounts[sdSpheres[i]]++;
    }

    std::ofstream ptFile{"output.csv"};
    ptFile << "x,y,z,nTouched" << std::endl;
    for (unsigned int n = 0; n < nDEs; n++) {
        ptFile << h_X_DE.at(n) << "," << h_Y_DE.at(n) << "," << h_Z_DE.at(n) << "," << deCounts[n] << std::endl;
    }
    delete[] sdvals;
    delete[] sdSpheres;
    delete[] deCounts;
}

__host__ void chrono::ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC::settle(float tEnd) {
    // Come up with the unit of time
    TIME_UNIT = 1. / (1 << SPHERE_TIME_UNIT_FACTOR) *
                sqrt((4. / 3. * M_PI * sphere_radius * sphere_radius * sphere_radius * sphere_density) /
                     (modulusYoung_SPH2SPH > modulusYoung_SPH2WALL ? modulusYoung_SPH2SPH : modulusYoung_SPH2WALL));

    // Once TIME_UNIT is available, set up the AD-ed components of the gravitational acceleration
    set_gravitational_acceleration_AD();

    // Set aside memory for holding data structures worked with. Get some initializations going
    setup_simulation();
    copyCONSTdata_to_device();

    // Seed arrays that are populated by the kernel call
    gpuErrchk(cudaMemset(p_device_SD_NumOf_DEs_Touching, 0, nSDs * sizeof(unsigned int)));
    gpuErrchk(cudaMemset(p_device_DEs_in_SD_composite, 0,
                         MAX_COUNT_OF_DEs_PER_SD * nSDs *
                             sizeof(unsigned int)));  // FUOT: is 0 the right value to set entries in this array to?

    /// Figure our the number of blocks that need to be launched to cover the box
    unsigned int nBlocks = (nDEs + CUDA_THREADS - 1) / CUDA_THREADS;
    primingOperationsRectangularBox<CUDA_THREADS><<<nBlocks, CUDA_THREADS>>>(
        p_d_CM_X, p_d_CM_Y, p_d_CM_Z, p_device_SD_NumOf_DEs_Touching, p_device_DEs_in_SD_composite, nSpheres());
    // printf("checking counts\n");
    // checkSDCounts();
    // printf("counts checked\n");
    // Settling simulation loop.
    unsigned int fakeTimeStep = 5;
    for (unsigned int crntTime = 0; crntTime < tEnd; crntTime += fakeTimeStep) {
        updateVelocities<MAX_COUNT_OF_DEs_PER_SD><<<nSDs, MAX_COUNT_OF_DEs_PER_SD>>>(
            fakeTimeStep, p_d_CM_X, p_d_CM_Y, p_d_CM_Z, p_d_CM_XDOT, p_d_CM_XDOT, p_d_CM_XDOT,
            p_device_SD_NumOf_DEs_Touching, p_device_DEs_in_SD_composite);

        updatePositions<CUDA_THREADS><<<nBlocks, CUDA_THREADS>>>(
            fakeTimeStep, p_d_CM_X, p_d_CM_Y, p_d_CM_Z, p_d_CM_XDOT, p_d_CM_XDOT, p_d_CM_XDOT,
            p_device_SD_NumOf_DEs_Touching, p_device_DEs_in_SD_composite, nSpheres());
    }

    cleanup_simulation();
    return;
}
