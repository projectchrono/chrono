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
// Authors: Dan Negrut
// =============================================================================

#include <cuda.h>
#include <climits>
#include <cstdio>
#include "../../chrono_thirdparty/cub/cub.cuh"
#include "../ChGranularDefines.h"
#include "../chrono_granular/physics/ChGranular.h"
#include "assert.h"

#define BLAH_BLAH_I 0
#define NULL_GRANULAR_ID UINT_MAX

extern __constant__ float3
    xyzOriginBox;  //!< Set of three floats that give the location of the rectangular box in the Global Reference Frame
extern __constant__ float4 eulerParamBox;  //!< Set of four floats that provide the orientation of the rectangular box
                                           //!< in the Global Reference Frame
extern __constant__ dim3
    SD_dims;  //!< Set of three ints that provide the dimension (in multiple of Sphere radia) of a SD
extern __constant__ dim3 RectangularBox_dims;  //!< The dimension of the rectangular box. The 3D box is expressed in
                                               //!< multples of SD, in the X, Y, and Z directions, respectively

// extern __constant__ unsigned int d_monoDisperseSphRadius_AD; // Pulled from the header
// Pulled from header
// extern __constant__ unsigned int d_SD_Ldim_AD;  //!< Ad-ed L-dimension of the SD box
// extern __constant__ unsigned int d_SD_Ddim_AD;  //!< Ad-ed D-dimension of the SD box
// extern __constant__ unsigned int d_SD_Hdim_AD;  //!< Ad-ed H-dimension of the SD box

/// Takes in a sphere's position and inserts into the given int array[8] which subdomains, if any, are touched
/// The array is indexed with the ones bit equal to +/- x, twos bit equal to +/- y, and the fours bit equal to +/- z
/// A bit set to 0 means the lower index, whereas 1 means the higher index (lower + 1)
/// The kernel computes global x, y, and z indices for the bottom-left subdomain and then uses those to figure out which
/// subdomains described in the corresponding 8-SD cube are touched by the sphere. The kernel then converts these
/// indices to indices into the global SD list via the (currently local) conv[3] data structure
/// Should be mostly bug-free, especially away from boundaries
__device__ void figureOutTouchedSD(unsigned int sphCenter_X,
                                   unsigned int sphCenter_Y,
                                   unsigned int sphCenter_Z,
                                   unsigned int* SDs) {
    // if (threadIdx.x == 1) {
    //     printf("kernel touched!\n");
    // }
    // The following variables are pulled in to make the code more legible, although we can move them out of the
    // kernel when we finalize the code and want to speed it up num subdomains, pull from RectangularBox_dims
    const unsigned int NX = RectangularBox_dims.x, NY = RectangularBox_dims.y, NZ = RectangularBox_dims.z;

    // Indices for bottom-left corner (x,y,z)
    unsigned int n[3];
    // TODO this doesn't handle if the ball is slightly penetrating the boundary, could result in negative values or end
    // GIDs beyond bounds. We might want to do a check to see if it's outside and set 'valid' accordingly
    // NOTE: This is integer arithmetic to compute the floor. We want to get the first SD below the sphere
    n[0] = (sphCenter_X - d_monoDisperseSphRadius_AD) / d_SD_Ldim_AD;  // nx = (xCenter - radius) / wx .
    n[1] = (sphCenter_Y - d_monoDisperseSphRadius_AD) / d_SD_Ddim_AD;  // Same for D and H
    n[2] = (sphCenter_Z - d_monoDisperseSphRadius_AD) / d_SD_Hdim_AD;  // Same for D and H
    // Find distance from next box in relevant dir to center, we may be straddling the two
    int d[3];                                        // Store penetrations
    d[0] = (n[0] + 1) * d_SD_Ldim_AD - sphCenter_X;  // dx = (nx + 1)* wx - x
    d[1] = (n[1] + 1) * d_SD_Ddim_AD - sphCenter_Y;
    d[2] = (n[2] + 1) * d_SD_Hdim_AD - sphCenter_Z;

    // Store list of conversions from nx, ny, nz to global subdomain IDs
    // GID = nx * NY * NZ + ny * NZ + nz
    // TODO this could be global
    unsigned int conv[3] = {(NY * NZ), NZ, 1};

    // Calculate whether higher or lower domain owns
    unsigned int own[3];
    // Minimize warp divergence by doing preemptive calculations
    // Use ternaries to provide symmetric code execution, if slightly divergent
    // TODO these used to be ternaries but conditionals return 1 or 0 anyways
    // TODO this is either sneaky or sloppy or clever, depending on how you like conditionals
    own[0] = d[0] < 0;
    own[1] = d[1] < 0;
    own[2] = d[2] < 0;
    // off[i] == 0 => lower subdomain holds center of sphere

    // Whether or not the domain has exclusive control
    unsigned int both[3];
    // If the absolute distance is less than r, both SDs along axis touch it
    both[0] = abs(d[0]) < d_monoDisperseSphRadius_AD;
    both[1] = abs(d[1]) < d_monoDisperseSphRadius_AD;
    both[2] = abs(d[2]) < d_monoDisperseSphRadius_AD;

    // Calculate global indices from locals
    // For each index in SDs
    for (int i = 0; i < 8; i++) {
        SDs[i] = 0;                // Init to 0
        unsigned int valid = 0x1;  // Assume value is true
        for (int e = 0; e < 3; e++) {
            // ones bit is x, twos bit is y, threes bit is z
            // do some cute bit shifting
            // Snag bit at correct place
            unsigned int s = i & (1 << e);
            // s adds an offset to directional index for SDs
            // Increment by global id contrib
            SDs[i] += (n[e] + s) * conv[e];
            if (n[e] != 0) {
            }
            // s == own[e] SDsurns true if the current SD is owner
            valid &= both[e] | (s == own[e]);  // If both touch it or we own it, the result is valid
        }
        // Valid will be 0 or 1 at this point, 0 => SD in SDs[i] isn't touched and the value is invalid
        // SDs[i] *= valid; // invalid is 0
        SDs[i] = (valid ? SDs[i] : NULL_GRANULAR_ID);  //
    }

    // This is how nasty this code would be if I didn't do the bit shifty stuff and the for loops above
    // ret[0] = (n[0] + off[0]) * conv[0] + (n[1] + off[1]) * conv[1] + (n[2] + (1 - off[2])) * conv[2];
    // ret[1] = (n[0] + off[0]) * conv[0] + (n[1] + (1 - off[1])) * conv[1] + (n[2] + off[2]) * conv[2];
    // ret[2] = (n[0] + off[0]) * conv[0] + (n[1] + off[1]) * conv[1] + (n[2] + (1 - off[2])) * conv[2];
    // ret[3] = (n[0] + off[0]) * conv[0] + (n[1] + (1 - off[1])) * conv[1] + (n[2] + (1 - off[2])) * conv[2];
    // ret[4] = (n[0] + (1 - off[0])) * conv[0] + (n[1] + off[1]) * conv[1] + (n[2] + (1 - off[2])) * conv[2];
    // ret[5] = (n[0] + (1 - off[0])) * conv[0] + (n[1] + (1 - off[1])) * conv[1] + (n[2] + off[2]) * conv[2];
    // ret[6] = (n[0] + (1 - off[0])) * conv[0] + (n[1] + off[1]) * conv[1] + (n[2] + (1 - off[2])) * conv[2];
    // ret[7] = (n[0] + (1 - off[0])) * conv[0] + (n[1] + (1 - off[1])) * conv[1] + (n[2] + (1 - off[2])) * conv[2];
}
/**
 * This kernel call prepares information that will be used in a subsequent kernel that performs the actual time
 * stepping.
 *
 * Template arguments:
 *   - CUB_THREADS: the number of threads used in this kernel, comes into play when invoking CUB block collectives
 *   - AMOUNT_SHMEM_KB: amount of shared memory in KB to be set aside; perhaps we should pass this as a kernel call
 * argument
 *
 *
 * Assumptions:
 *   - Granular material is made up of spheres.
 *   - For now, all spheres are of constant radius. The radius of the sphere is 1.f
 *   - The function below assumes the spheres are in a box
 *   - The box has dimensions L x W x H.
 *   - The reference frame associated with the box:
 *       - The x-axis is along the length L of the box
 *       - The y-axis is along the width W of the box
 *       - The z-axis is along the height H of the box
 *   - A sphere cannot touch more than eight SDs; this is reflected in the array "seedData" having size 16.
 *   - The total number of SDs touched by the sphres worked upon in this CUDA block is less than USHRT_MAX. This is
 *       reasonable given that we are not going to have more than 1024 spheres worked upon in a CUDA block
 *
 * Basic idea: use domain decomposition on the rectangular box and have one subdomain be processed by one block.
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
 * box. Usually, there is no sphere in this SD
 *
 *
 *
 */
template <int CUB_THREADS  //!< Number of CUB threads engaged in block-collective CUB operations. Should be a multiple
                           //!< of warp size (32) and less than or equal to min(256, blockDim.x)
          >
__global__ void primingOperationsRectangularBox(
    unsigned int* pRawDataX,                  //!< Pointer to array containing data related to the spheres in the box
    unsigned int* pRawDataY,                  //!< Pointer to array containing data related to the spheres in the box
    unsigned int* pRawDataZ,                  //!< Pointer to array containing data related to the spheres in the box
    unsigned int* SD_countsOfSheresTouching,  //!< The array that for each SD indicates how many spheres touch this SD
    unsigned int* spheres_in_SD_composite,    //!< Big array that works in conjunction with SD_countsOfSheresTouching.
                                              //!< "spheres_in_SD_composite" says which SD contains what spheres
    size_t nSpheres                           //!< Number of spheres in the box
) {
    unsigned int xSphCenter;
    unsigned int ySphCenter;
    unsigned int zSphCenter;

    /// Set aside shared memory
    __shared__ unsigned int offsetInComposite_SphInSD_Array[CUB_THREADS];
    __shared__ bool shMem_head_flags[CUB_THREADS];

    unsigned int touchedSD[1];
    unsigned int mySphereID[1];
    bool head_flags[1];

    // Initiatlizations
    mySphereID[0] = threadIdx.x + blockIdx.x * blockIdx.y * blockIdx.z *
                                      blockDim.x;  // We work with a 1D block structure and a 3D grid structure
    touchedSD[0] = NULL_GRANULAR_ID;               // Important to seed the touchedSD w/ a "no-SD" value
    offsetInComposite_SphInSD_Array[threadIdx.x] =
        NULL_GRANULAR_ID;  // Reflecting that a sphere might belong to an SD in a certain trip "i", see "for" loop

    unsigned int dummyUINT01 = mySphereID[0];
    if (mySphereID[0] < nSpheres) {
        // Coalesced mem access
        xSphCenter = pRawDataX[dummyUINT01];
        ySphCenter = pRawDataY[dummyUINT01];
        zSphCenter = pRawDataZ[dummyUINT01];
    }
    // I moved this function up since we do all at once
    unsigned int SDsTouched[8];
    if (mySphereID[0] < nSpheres)
        figureOutTouchedSD(xSphCenter, ySphCenter, zSphCenter, SDsTouched);

    // Each sphere can at most touch 8 SDs; we'll need to take 8 trips then.
    for (int i = 0; i < 8; i++) {
        // Is there any SD touched at this level? Use a CUB reduce operation to find out
        typedef cub::BlockReduce<unsigned int, CUB_THREADS> BlockReduce;
        __shared__ typename BlockReduce::TempStorage temp_storage;
        dummyUINT01 = BlockReduce(temp_storage).Reduce(touchedSD[0], cub::Min());

        if (dummyUINT01 != NULL_GRANULAR_ID) {
            // Note that there is no thread divergence in this "if" since all threads see the same value (returned by
            // CUB) Processing sphere-in-SD events at level "i". Do a key-value sort to group together the like-SDs
            typedef cub::BlockRadixSort<unsigned int, CUB_THREADS, 1, unsigned int> BlockRadixSortOP;
            __shared__ typename BlockRadixSortOP::TempStorage temp_storage_sort;
            BlockRadixSortOP(temp_storage_sort).Sort(touchedSD, mySphereID);

            // Figure our where each SD starts and ends in the sequence of SDs
            typedef cub::BlockDiscontinuity<unsigned int, CUB_THREADS> Block_Discontinuity;
            __shared__ typename Block_Discontinuity::TempStorage temp_storage_disc;
            Block_Discontinuity(temp_storage_disc).FlagHeads(head_flags, touchedSD, cub::Inequality());

            // Place data in shared memory since it needs to be accessed by other threads
            shMem_head_flags[threadIdx.x] = head_flags[0];
            __syncthreads();

            // Count how many times an SD shows up in conjunction with the collection of CUB_THREADS sphres. There will
            // be some thread divergence here.
            if (head_flags[0]) {
                // This is the beginning of a sequence with a new ID
                dummyUINT01 = 0;
                do {
                    dummyUINT01++;
                } while (threadIdx.x + dummyUINT01 < CUB_THREADS && !(head_flags[threadIdx.x + dummyUINT01]));
                // Overwrite touchedSD[0], it ended its purpose upon call to atomicAdd
                touchedSD[0] = atomicAdd(SD_countsOfSheresTouching + touchedSD[0], dummyUINT01);
                // touchedSD[0] now gives offset in the composite array

                // Next, overwrite mySphereID, this was needed only to make CUB happy; it contains now the length of the
                // streak for this SD
                mySphereID[0] = dummyUINT01;
                for (dummyUINT01 = 0; dummyUINT01 < mySphereID[0]; dummyUINT01++)
                    offsetInComposite_SphInSD_Array[threadIdx.x + dummyUINT01] = touchedSD[0]++;
            }

            __syncthreads();
            spheres_in_SD_composite[threadIdx.x] = offsetInComposite_SphInSD_Array[threadIdx.x];
            // At the beginning of a new trip, for a new "i", start anew
            offsetInComposite_SphInSD_Array[threadIdx.x] = NULL_GRANULAR_ID;
        }  // End of scenarios when this "i" trip had work to do
    }      // End of the eight trips
}

void chrono::ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC::settle(float tEnd) {
#define CUDA_THREADS 128
    setup_simulation();
    /// Figure our the number of blocks that need to be launched to cover the box
    dim3 grid3D_dims(256, 256, 256);
    primingOperationsRectangularBox<CUDA_THREADS><<<grid3D_dims, CUDA_THREADS>>>(
        p_d_CM_X, p_d_CM_Y, p_d_CM_Z, p_device_SD_NumOf_DEs_Touching, p_device_DEs_in_SD_composite, nSpheres());

    return;
}
