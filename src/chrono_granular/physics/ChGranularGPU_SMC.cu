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
#include "chrono_granular/utils/ChGranularUtilities_CUDA.cuh"
//#include "chrono_granular/physics/ChGranularDefines.cuh"

#define NULL_GRANULAR_ID UINT_MAX-1

// extern "C" __constant__ float3
//    xyzOriginBox;  //!< Set of three floats that give the location of the rectangular box in the Global Reference
//    Frame
// extern "C" __constant__ float4 eulerParamBox;  //!< Set of four floats that provide the orientation of the
// rectangular box
//                                           //!< in the Global Reference Frame
// extern "C" __constant__ dim3 RectangularBox_dims;  //!< The dimension of the rectangular box. The 3D box is expressed
// in
//                                               //!< multples of SD, in the X, Y, and Z directions, respectively

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
    // TODO this makes a false assumption about the origin
    n[0] = (sphCenter_X_modified - d_monoDisperseSphRadius_AD) / d_SD_Ldim_AD;
    // Same for D and H
    n[1] = (sphCenter_Y_modified - d_monoDisperseSphRadius_AD) / d_SD_Ddim_AD;
    n[2] = (sphCenter_Z_modified - d_monoDisperseSphRadius_AD) / d_SD_Hdim_AD;
    // Find distance from next box in relevant dir to center, we may be straddling the two
    int d[3];                                                 // Store penetrations
    d[0] = (n[0] + 1) * d_SD_Ldim_AD - sphCenter_X_modified;  // dx = (nx + 1)* wx - x
    d[1] = (n[1] + 1) * d_SD_Ddim_AD - sphCenter_Y_modified;
    d[2] = (n[2] + 1) * d_SD_Hdim_AD - sphCenter_Z_modified;

    // Store list of conversions from nx, ny, nz to global subdomain IDs

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
        // This code should be silent, hopefully
        // check if in bounds, only for debugging
        // if (valid) {
        //     if (n[0] + (i & 0x1) >= d_box_L_AD) {
        //         printf("overflowing xdim %d, %d, %d, %d, %d, d is %d %d %d\n", sphCenter_X, sphCenter_X_modified,
        //                n[0] + (i & 0x1), d_box_L_AD, d_SD_Ldim_AD, d[0], d[1], d[2]);
        //     } else if (n[1] + ((i >> 1) & 0x1) >= d_box_D_AD) {
        //         printf("overflowing ydim %d, %d, %d, %d, %d, d is %d %d %d\n", sphCenter_Y, sphCenter_Y_modified,
        //                n[1] + ((i >> 1) & 0x1), d_box_D_AD, d_SD_Ddim_AD, d[0], d[1], d[2]);
        //     } else if (n[2] + ((i >> 2) & 0x1) >= d_box_H_AD) {
        //         printf("overflowing zdim %d, %d, %d, %d, %d, d is %d %d %d\n", sphCenter_Z, sphCenter_Z_modified,
        //                n[2] + ((i >> 2) & 0x1), d_box_H_AD, d_SD_Hdim_AD, d[0], d[1], d[2]);
        //     }
        //     if (SDs[i] >= d_box_L_AD * d_box_D_AD * d_box_H_AD) {
        //         printf("still overflowing box somehow: i is %x, SD is %d, n is %d %d %d, d is %d %d %d\n", i, SDs[i],
        //                n[0] + (i & 0x1), n[1] + ((i >> 2) & 0x1), n[2] + ((i >> 2) & 0x1), d[0], d[1], d[2]);
        //     }
        // }
    }
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
template <
    int CUB_THREADS>  //!< Number of CUB threads engaged in block-collective CUB operations. Should be a multiple of 32
__global__ void
primingOperationsRectangularBox(
    int* pRawDataX,                           //!< Pointer to array containing data related to the spheres in the box
    int* pRawDataY,                           //!< Pointer to array containing data related to the spheres in the box
    int* pRawDataZ,                           //!< Pointer to array containing data related to the spheres in the box
    unsigned int* SD_countsOfSheresTouching,  //!< The array that for each SD indicates how many spheres touch this SD
    unsigned int* spheres_in_SD_composite,    //!< Big array that works in conjunction with SD_countsOfSheresTouching.
                                              //!< "spheres_in_SD_composite" says which SD contains what spheres
    unsigned int nSpheres                     //!< Number of spheres in the box
) {
    int xSphCenter;
    int ySphCenter;
    int zSphCenter;

    /// Set aside shared memory
    __shared__ unsigned int offsetInComposite_SphInSD_Array[CUB_THREADS];
    __shared__ bool shMem_head_flags[CUB_THREADS];

    typedef cub::BlockReduce<unsigned int, CUB_THREADS> BlockReduce;
    __shared__ typename BlockReduce::TempStorage temp_storage_reduce;

    typedef cub::BlockRadixSort<unsigned int, CUB_THREADS, 1, unsigned int> BlockRadixSortOP;
    __shared__ typename BlockRadixSortOP::TempStorage temp_storage_sort;

    typedef cub::BlockDiscontinuity<unsigned int, CUB_THREADS> Block_Discontinuity;
    __shared__ typename Block_Discontinuity::TempStorage temp_storage_disc;

    unsigned int touchedSD[1];
    unsigned int mySphereID[1];
    bool head_flags[1];

    // Figure out what sphereID this thread will handle. We work with a 1D block structure and a 1D grid structure
    mySphereID[0] = threadIdx.x + blockIdx.x * blockDim.x;

    touchedSD[0] = NULL_GRANULAR_ID;  // Important to seed the touchedSD w/ a "no-SD" value
    offsetInComposite_SphInSD_Array[threadIdx.x] = NULL_GRANULAR_ID;  // Reflecting that a sphere might belong to an SD in a certain trip "i", see "for" loop

    unsigned int SDsTouched[8];
    unsigned int dummyUINT01;
    if (mySphereID[0] < nSpheres) {
        dummyUINT01 = mySphereID[0];
        // Coalesced mem access
        xSphCenter = pRawDataX[dummyUINT01];
        ySphCenter = pRawDataY[dummyUINT01];
        zSphCenter = pRawDataZ[dummyUINT01];

        figureOutTouchedSD(xSphCenter, ySphCenter, zSphCenter, SDsTouched);
    }
    else {
        // this thread doesn't have a corresponding ball; 
        for (dummyUINT01 = 0; dummyUINT01 < 8; dummyUINT01++)
            SDsTouched[dummyUINT01] = NULL_GRANULAR_ID;
    }
//    // TODO this sphere check needs to catch more, I think
//    // I moved this function up since we do all at once
//    unsigned int SDsTouched[8];
//    if (mySphereID[0] < nSpheres)   
//
//    // Each sphere can at most touch 8 SDs; we'll need to take 8 trips then.
//    for (int i = 0; i < 8; i++) {
//        // TODO we need to catch NULL_GRANULAR_ID somewhere around here
//        touchedSD[0] = SDsTouched[i];
//
//        // Amongst all threads, is there any SD touched at this level? Use a CUB reduce operation to find out
//        dummyUINT01 = BlockReduce(temp_storage_reduce).Reduce(touchedSD[0], cub::Min());
//
//        if (dummyUINT01 != NULL_GRANULAR_ID) {
//            // Note that there is no thread divergence in this "if" since all threads see the same value (returned by
//            // CUB) Processing sphere-in-SD events at level "i". Do a key-value sort to group together the like-SDs
//            BlockRadixSortOP(temp_storage_sort).Sort(touchedSD, mySphereID);
//
//            // Figure our where each SD starts and ends in the sequence of SDs
//            Block_Discontinuity(temp_storage_disc).FlagHeads(head_flags, touchedSD, cub::Inequality());
//
//            // Place data in shared memory since it needs to be accessed by other threads
//            shMem_head_flags[threadIdx.x] = head_flags[0];
//            __syncthreads();
//
//            // Count how many times an SD shows up in conjunction with the collection of CUB_THREADS sphres. There will
//            // be some thread divergence here.
//            if (head_flags[0]) {
//                // This is the beginning of a sequence with a new ID
//                dummyUINT01 = 0;
//                do {
//                    dummyUINT01++;
//                } while (threadIdx.x + dummyUINT01 < CUB_THREADS && !(shMem_head_flags[threadIdx.x + dummyUINT01]));
//
//                // if (touchedSD[0] >= d_box_L_AD * d_box_D_AD * d_box_H_AD) {
//                //     printf("invalid SD index %u on thread %u\n", mySphereID[0], touchedSD[0]);
//                // }
//                // TODO this line causes an error later -- I think we're trashing our device heap
//                // We need to check for NULL_GRANULAR_ID
//
//                // Overwrite touchedSD[0], it ended its purpose upon call to atomicAdd
//                touchedSD[0] = atomicAdd(SD_countsOfSheresTouching + touchedSD[0], dummyUINT01);
//                // touchedSD[0] now gives offset in the composite array
//
//                // Next, overwrite mySphereID, this was needed only to make CUB happy; it contains now the length of the
//                // streak for this SD
//                mySphereID[0] = dummyUINT01;
//                for (dummyUINT01 = 0; dummyUINT01 < mySphereID[0]; dummyUINT01++)
//                    offsetInComposite_SphInSD_Array[threadIdx.x + dummyUINT01] = touchedSD[0]++;
//            }
//
//            __syncthreads();
//            spheres_in_SD_composite[threadIdx.x] = offsetInComposite_SphInSD_Array[threadIdx.x];
//            // At the beginning of a new trip, for a new "i", start anew
//            offsetInComposite_SphInSD_Array[threadIdx.x] = NULL_GRANULAR_ID;
//        }  // End of scenarios when this "i" trip had work to do
//    }      // End of the eight trips
}

void chrono::ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC::copyCONSTdata_to_device() {
    // Copy adimensional size of SDs to device
    gpuErrchk(cudaMemcpyToSymbol(d_SD_Ldim_AD, &SD_L_AD, sizeof(d_SD_Ldim_AD)));
    gpuErrchk(cudaMemcpyToSymbol(d_SD_Ddim_AD, &SD_D_AD, sizeof(d_SD_Ddim_AD)));
    gpuErrchk(cudaMemcpyToSymbol(d_SD_Hdim_AD, &SD_H_AD, sizeof(d_SD_Hdim_AD)));
    // Copy global BD size in multiples of SDs to device
    gpuErrchk(cudaMemcpyToSymbol(d_box_L_AD, &nSDs_L_AD, sizeof(d_box_L_AD)));
    gpuErrchk(cudaMemcpyToSymbol(d_box_D_AD, &nSDs_D_AD, sizeof(d_box_D_AD)));
    gpuErrchk(cudaMemcpyToSymbol(d_box_H_AD, &nSDs_H_AD, sizeof(d_box_H_AD)));

    gpuErrchk(cudaMemcpyToSymbol(d_monoDisperseSphRadius_AD, &monoDisperseSphRadius_AD, sizeof(d_monoDisperseSphRadius_AD)));
}

void chrono::ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC::settle(float tEnd) {
#define CUDA_THREADS 128
    // Come up with the unit of time

    TIME_UNIT = 1. / (1 << SPHERE_TIME_UNIT_FACTOR) *
                sqrt((4. / 3. * M_PI * sphere_radius * sphere_radius * sphere_radius * sphere_density) /
                     (modulusYoung_SPH2SPH > modulusYoung_SPH2WALL ? modulusYoung_SPH2SPH : modulusYoung_SPH2WALL));
    setup_simulation();
    copyCONSTdata_to_device();
    /// Figure our the number of blocks that need to be launched to cover the box
    unsigned int nBlocks = (nDEs + CUDA_THREADS - 1) / CUDA_THREADS;
    primingOperationsRectangularBox<CUDA_THREADS><<<nBlocks, CUDA_THREADS>>>
        (p_d_CM_X, p_d_CM_Y, p_d_CM_Z, p_device_SD_NumOf_DEs_Touching, p_device_DEs_in_SD_composite, nSpheres());
    gpuErrchk(cudaPeekAtLastError());
    gpuErrchk(cudaDeviceSynchronize());

    cleanup_simulation();
    return;
}
