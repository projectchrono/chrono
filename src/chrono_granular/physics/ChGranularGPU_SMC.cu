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

__constant__ unsigned int d_monoDisperseSphRadius_SU;  //!< Radius of the sphere, expressed in SU

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
    int sphCenter_X_modified = (d_box_L_SU * d_SD_Ldim_SU) / 2 + sphCenter_X;
    int sphCenter_Y_modified = (d_box_D_SU * d_SD_Ddim_SU) / 2 + sphCenter_Y;
    int sphCenter_Z_modified = (d_box_H_SU * d_SD_Hdim_SU) / 2 + sphCenter_Z;
    int n[3];
    // TODO this doesn't handle if the ball is slightly penetrating the boundary, could result in negative values or end
    // GIDs beyond bounds. We might want to do a check to see if it's outside and set 'valid' accordingly
    // NOTE: This is integer arithmetic to compute the floor. We want to get the first SD below the sphere
    // nx = (xCenter - radius) / wx .
    n[0] = (sphCenter_X_modified - d_monoDisperseSphRadius_SU) / d_SD_Ldim_SU;
    // Same for D and H
    n[1] = (sphCenter_Y_modified - d_monoDisperseSphRadius_SU) / d_SD_Ddim_SU;
    n[2] = (sphCenter_Z_modified - d_monoDisperseSphRadius_SU) / d_SD_Hdim_SU;
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
        unsigned int contact = (d2 < d_monoDisperseSphRadius_SU * d_monoDisperseSphRadius_SU);
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
        unsigned int contact = (d2 < d_monoDisperseSphRadius_SU * d_monoDisperseSphRadius_SU);
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
__device__ void boxWallsEffects(int sphXpos, int sphYpos, int sphZpos, float& Xforce, float& Yforce, float& Zforce) {
    Xforce = 0.f;
    Yforce = 0.f;
    Zforce = 0.f;
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
__global__ void updateVelocities(unsigned int alpha_h_bar,  //!< Value that controls actual step size.
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
                                 unsigned int* spheres_in_SD_composite      //!< Big array that works in conjunction
                                                                            //!< with SD_countsOfSpheresTouching.
) {
    __shared__ int sph_X[MAX_NSPHERES_PER_SD];
    __shared__ int sph_Y[MAX_NSPHERES_PER_SD];
    __shared__ int sph_Z[MAX_NSPHERES_PER_SD];
    __shared__ char bodyB_list[12 * MAX_NSPHERES_PER_SD];  // NOTE: max number of spheres that can kiss a sphere is 12.

    unsigned int spheresTouchingThisSD = SD_countsOfSpheresTouching[blockIdx.x];
    unsigned mySphereID;

    // Bring in data from global into shmem. Only a subset of threads get to do this.
    if (threadIdx.x < spheresTouchingThisSD) {
        mySphereID = spheres_in_SD_composite[blockIdx.x * MAX_NSPHERES_PER_SD + threadIdx.x];
        sph_X[threadIdx.x] = pRawDataX[mySphereID];
        sph_Y[threadIdx.x] = pRawDataY[mySphereID];
        sph_Z[threadIdx.x] = pRawDataZ[mySphereID];
    }

    __syncthreads();  // Needed to make sure data gets in shmem before using it elsewhere

    // Assumes each thread is a body, not the greatest assumption but we can fix that later
    // Note that if we have more threads than bodies, some effort gets wasted. With our current parameters (3/8/18) we
    // have at most 113 DEs per SD. If we have more bodies than threads, we might want to increase the number of threads
    // or decrease the number of DEs per SD
    unsigned int bodyA = threadIdx.x;
    double X_dir_contactForce;
    double Y_dir_contactForce;
    double Z_dir_contactForce;

    // Each body looks at each other body and computes the force that the other body exerts on it
    if (bodyA < spheresTouchingThisSD) {
        double invSphDiameter = 2. * d_monoDisperseSphRadius_SU;
        invSphDiameter = 1. / invSphDiameter;
        double X_dummyVal = sph_X[bodyA] * invSphDiameter;
        double Y_dummyVal = sph_Y[bodyA] * invSphDiameter;
        double Z_dummyVal = sph_Z[bodyA] * invSphDiameter;

        double penetrationProxy;
        unsigned int nCollisions = 0;
        for (unsigned int bodyB = 0; bodyB < spheresTouchingThisSD; bodyB++) {
            // Don't check for collision with self
            if (bodyA == bodyB)
                continue;

            // This avoids computing a square to figure our if collision or not
            X_dir_contactForce = X_dummyVal - sph_X[bodyB] * invSphDiameter;
            Y_dir_contactForce = Y_dummyVal - sph_Y[bodyB] * invSphDiameter;
            Z_dir_contactForce = Z_dummyVal - sph_Z[bodyB] * invSphDiameter;

            penetrationProxy = X_dir_contactForce * X_dir_contactForce;
            penetrationProxy += Y_dir_contactForce * Y_dir_contactForce;
            penetrationProxy += Z_dir_contactForce * Z_dir_contactForce;

            // We have a collision here...
            if (penetrationProxy < 1) {
                bodyB_list[threadIdx.x * 12 + nCollisions] = bodyB;
                nCollisions++;
            }
        }

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
        float bodyA_X_velCorr = 0.f;
        float bodyA_Y_velCorr = 0.f;
        float bodyA_Z_velCorr = 0.f;
        float scalingFactor = alpha_h_bar / (psi_T_dFactor * psi_T_dFactor * psi_h_dFactor);

        for (unsigned int bodyB = 0; bodyB < nCollisions; bodyB++) {
            // Note: this can be accelerated should we decide to go w/ float. Then we can use the CUDA intrinsic:
            // __device__ ​ float rnormf ( int  dim, const float* a)
            // http://docs.nvidia.com/cuda/cuda-math-api/group__CUDA__MATH__SINGLE.html#group__CUDA__MATH__SINGLE
            X_dir_contactForce = X_dummyVal - sph_X[bodyB] * invSphDiameter;
            Y_dir_contactForce = Y_dummyVal - sph_Y[bodyB] * invSphDiameter;
            Z_dir_contactForce = Z_dummyVal - sph_Z[bodyB] * invSphDiameter;

            penetrationProxy = X_dir_contactForce * X_dir_contactForce;
            penetrationProxy += Y_dir_contactForce * Y_dir_contactForce;
            penetrationProxy += Z_dir_contactForce * Z_dir_contactForce;

            penetrationProxy = sqrt(penetrationProxy);
            penetrationProxy = 1 / penetrationProxy;
            penetrationProxy -= 1.;

            X_dir_contactForce *= penetrationProxy;
            Y_dir_contactForce *= penetrationProxy;
            Z_dir_contactForce *= penetrationProxy;

            bodyA_X_velCorr += scalingFactor * X_dir_contactForce;
            bodyA_Y_velCorr += scalingFactor * Y_dir_contactForce;
            bodyA_Z_velCorr += scalingFactor * Z_dir_contactForce;
        }

        // Perhaps this sphere is hitting the wall[s]
        boxWallsEffects(sph_X[bodyA], sph_Y[bodyA], sph_Z[bodyA], bodyA_X_velCorr, bodyA_Y_velCorr, bodyA_Z_velCorr);

        // If the sphere belongs to this SD, add up the gravitational force component.
        // IMPORTANT: Make sure that the sphere belongs to *this* SD, otherwise we'll end up with double counting this
        // force.
        if (true) {
            // CONLAIN: can you help with this test? We add the effect of gravity more times than we need here
            bodyA_X_velCorr += alpha_h_bar * gravAcc_X_d_factor_SU;
            bodyA_Y_velCorr += alpha_h_bar * gravAcc_Y_d_factor_SU;
            bodyA_Z_velCorr += alpha_h_bar * gravAcc_Z_d_factor_SU;
        }

        // We still need to write back atomically to global memory
        atomicAdd(pRawDataX_DOT + mySphereID, (int)bodyA_X_velCorr);
        atomicAdd(pRawDataY_DOT + mySphereID, (int)bodyA_Y_velCorr);
        atomicAdd(pRawDataZ_DOT + mySphereID, (int)bodyA_Z_velCorr);
    }
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

        xSphCenter += pRawDataX[mySphereID];
        ySphCenter += pRawDataY[mySphereID];
        zSphCenter += pRawDataZ[mySphereID];

        pRawDataX[mySphereID] = xSphCenter;
        pRawDataY[mySphereID] = ySphCenter;
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
}

// Check number of spheres in each SD and dump relevant info to file
void chrono::ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC::checkSDCounts(std::string ofile) {
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
        // printf("de id is %d, i is %u\n", sdSpheres[i], i);
        // Check if invalid sphere
        if (-1 == (signed int)sdSpheres[i]) {
            // printf("invalid sphere in sd");
        } else {
            deCounts[sdSpheres[i]]++;
        }
    }

    std::ofstream ptFile{ofile};
    ptFile << "x,y,z,nTouched" << std::endl;
    for (unsigned int n = 0; n < nDEs; n++) {
        ptFile << h_X_DE.at(n) << "," << h_Y_DE.at(n) << "," << h_Z_DE.at(n) << "," << deCounts[n] << std::endl;
    }
    delete[] sdvals;
    delete[] sdSpheres;
    delete[] deCounts;
}

__host__ void chrono::ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC::settle(float tEnd) {
    switch_to_SimUnits();
    generate_DEs();

    // Set aside memory for holding data structures worked with. Get some initializations going
    setup_simulation();
    copyCONSTdata_to_device();

    // Seed arrays that are populated by the kernel call
    const unsigned char allBitsOne = (unsigned char)-1;  // all bits of this variable are 1.
    // Set all the offsets to zero
    gpuErrchk(cudaMemset(p_device_SD_NumOf_DEs_Touching, 0, nSDs * sizeof(unsigned int)));
    // For each SD, all the spheres touching that SD should have their ID be NULL_GRANULAR_ID
    gpuErrchk(
        cudaMemset(p_device_DEs_in_SD_composite, allBitsOne, MAX_COUNT_OF_DEs_PER_SD * nSDs * sizeof(unsigned int)));

    /// Figure our the number of blocks that need to be launched to cover the box
    unsigned int nBlocks = (nDEs + CUDA_THREADS - 1) / CUDA_THREADS;
    primingOperationsRectangularBox<CUDA_THREADS><<<nBlocks, CUDA_THREADS>>>(
        p_d_CM_X, p_d_CM_Y, p_d_CM_Z, p_device_SD_NumOf_DEs_Touching, p_device_DEs_in_SD_composite, nSpheres());
    cudaDeviceSynchronize();

    // printf("checking counts\n");
    checkSDCounts("output.csv");
    // printf("counts checked\n");
    // Settling simulation loop.
    unsigned int stepSize_SU = 8;
    unsigned int tEnd_SU = tEnd / TIME_UNIT;
    for (unsigned int crntTime_SU = 0; crntTime_SU < tEnd; crntTime_SU += stepSize_SU) {
        updateVelocities<MAX_COUNT_OF_DEs_PER_SD><<<nSDs, MAX_COUNT_OF_DEs_PER_SD>>>(
            stepSize_SU, p_d_CM_X, p_d_CM_Y, p_d_CM_Z, p_d_CM_XDOT, p_d_CM_XDOT, p_d_CM_XDOT,
            p_device_SD_NumOf_DEs_Touching, p_device_DEs_in_SD_composite);
        cudaDeviceSynchronize();
        gpuErrchk(cudaMemset(p_device_SD_NumOf_DEs_Touching, 0, nSDs * sizeof(unsigned int)));
        gpuErrchk(cudaMemset(p_device_DEs_in_SD_composite, allBitsOne,
                             MAX_COUNT_OF_DEs_PER_SD * nSDs * sizeof(unsigned int)));
        cudaDeviceSynchronize();

        updatePositions<CUDA_THREADS><<<nBlocks, CUDA_THREADS>>>(
            stepSize_SU, p_d_CM_X, p_d_CM_Y, p_d_CM_Z, p_d_CM_XDOT, p_d_CM_XDOT, p_d_CM_XDOT,
            p_device_SD_NumOf_DEs_Touching, p_device_DEs_in_SD_composite, nSpheres());
        cudaDeviceSynchronize();
    }

    cleanup_simulation();
    return;
}
