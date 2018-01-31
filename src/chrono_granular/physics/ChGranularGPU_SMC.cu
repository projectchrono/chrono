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
#include "../../chrono_thirdparty/cub/cub.cuh"
#include "../ChGranularDefines.h"
#include "../chrono_granular/physics/ChGranular.h"
#include "assert.h"

#define BLAH_BLAH_I 0
#define NULL_GRANULAR_ID UINT_MAX-1 

extern __constant__ float3 xyzOriginBox;       //!< Set of three floats that give the location of the rectangular box in the Global Reference Frame
extern __constant__ float4 eulerParamBox;      //!< Set of four floats that provide the orientation of the rectangular box in the Global Reference Frame
extern __constant__ dim3 SD_dims;              //!< Set of three ints that provide the dimension (in multiple of Sphere radia) of a SD
extern __constant__ dim3 RectangularBox_dims;  //!< The dimension of the rectangular box. The 3D box is expressed in multpiples of SD, in the X, Y, and Z directions, respectively



__device__ unsigned int figureOutTouchedSD(int tripID);

/**
* This kernel call prepares information that will be used in a subsequent kernel that performs the actual time stepping.
*
* Template arguments:
*   - CUB_THREADS: the number of threads used in this kernel, comes into play when invoking CUB block collectives
*   - AMOUNT_SHMEM_KB: amount of shared memory in KB to be set aside; perhaps we should pass this as a kernel call argument
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
* The subdomains are axis-aligned relative to the reference frame associated with the *box*. The origin of the box is at the center of the box.
* The orientation of the box is defined relative to a world inertial reference frame.
*
* Nomenclature:
*   - SD: subdomain.
*   - BD: the big-domain, which is the union of all SDs
*   - NULL_GRANULAR_ID: the equivalent of a non-sphere SD ID, or a non-sphere ID
*
* Notes:
*   - The SD with ID=0 is the catch-all SD. This is the SD in which a sphere ends up if its not inside the rectangular box. Usually, there is no sphere in this SD
*
*
*
*/
template <
    int CUB_THREADS           //!< Number of CUB threads engaged in block-collective CUB operations. Should be a multiple of warp size (32) and less than or equal to min(256, blockDim.x)
>
__global__ void primingOperationsRectangularBox(
    float* pRawDataX,                     //!< Pointer to array containing data related to the spheres in the box
    float* pRawDataY,                     //!< Pointer to array containing data related to the spheres in the box
    float* pRawDataZ,                     //!< Pointer to array containing data related to the spheres in the box
    unsigned int* SD_countsOfSheresTouching,  //!< The array that for each SD indicates how many spheres touch this SD
    unsigned int* spheres_in_SD_composite,    //!< Big array that works in conjunction with SD_countsOfSheresTouching. "spheres_in_SD_composite" says which SD contains what spheres
    size_t nSpheres                           //!< Number of spheres in the box
)
{
    float xSphCenter;
    float ySphCenter;
    float zSphCenter;

    /// Set aside shared memory
    __shared__ unsigned int offsetInComposite_SphInSD_Array[CUB_THREADS];
    __shared__ bool shMem_head_flags[CUB_THREADS];

    unsigned int touchedSD[1];
    unsigned int mySphereID[1];
    bool head_flags[1];


    // Initiatlizations
    mySphereID[0] = threadIdx.x + blockIdx.x * blockIdx.y * blockIdx.z * blockDim.x;  // We work with a 1D block structure and a 3D grid structure   
    touchedSD[0] = NULL_GRANULAR_ID; // Important to seed the touchedSD w/ a "no-SD" value
    offsetInComposite_SphInSD_Array[threadIdx.x] = NULL_GRANULAR_ID; // Reflecting that a sphere might belong to an SD in a certain trip "i", see "for" loop
    
    unsigned int dummyUINT01 = mySphereID[0];
    if (mySphereID[0] < nSpheres) {
        // Coalesced mem access
        xSphCenter = pRawDataX[3 * dummyUINT01];
        ySphCenter = pRawDataY[3 * dummyUINT01];
        zSphCenter = pRawDataZ[3 * dummyUINT01];
    }

    // Each sphere can at most touch 8 SDs; we'll need to take 8 trips then.
    for (int i = 0; i < 8; i++) {
        if (mySphereID[0] < nSpheres)
            touchedSD[0] = figureOutTouchedSD(i);

        // Is there any SD touched at this level? Use a CUB reduce operation to find out
        typedef cub::BlockReduce<unsigned int, CUB_THREADS> BlockReduce;
        __shared__ typename BlockReduce::TempStorage temp_storage;
        dummyUINT01 = BlockReduce(temp_storage).Reduce(touchedSD[0], cub::Min());

        if (dummyUINT01 != NULL_GRANULAR_ID) {
            // Note that there is no thread divergence in this "if" since all threads see the same value (returned by CUB)
            // Processing sphere-in-SD events at level "i". Do a key-value sort to group together the like-SDs
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

            // Count how many times an SD shows up in conjunction with the collection of CUB_THREADS sphres. There will be some thread divergence here.
            if (head_flags[0]) {
                // This is the beginning of a sequence with a new ID
                dummyUINT01 = 0;
                do {
                    dummyUINT01++;
                } while (threadIdx.x + dummyUINT01 < CUB_THREADS && !(head_flags[threadIdx.x + dummyUINT01]));
                // Overwrite touchedSD[0], it ended its purpose upon call to atomicAdd
                touchedSD[0] = atomicAdd(SD_countsOfSheresTouching+touchedSD[0], dummyUINT01);
                // touchedSD[0] now gives offset in the composite array

                // Next, overwrite mySphereID, this was needed only to make CUB happy; it contains now the length of the streak for this SD
                mySphereID[0] = dummyUINT01;
                for (dummyUINT01 = 0; dummyUINT01 < mySphereID[0]; dummyUINT01++)
                    offsetInComposite_SphInSD_Array[threadIdx.x + dummyUINT01] = touchedSD[0]++;
            }

            __syncthreads();
            spheres_in_SD_composite[threadIdx.x] = offsetInComposite_SphInSD_Array[threadIdx.x];
            // At the beginning of a new trip, for a new "i", start anew
            offsetInComposite_SphInSD_Array[threadIdx.x] = NULL_GRANULAR_ID;
        } // End of scenarios when this "i" trip had work to do
    } // End of the eight trips
}




void chrono::ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC::settle(float tEnd) {
    dim3 grid3D_dims(256, 256, 256);
    primingOperationsRectangularBox<128> << <grid3D_dims, 128 >> > (p_d_CM_X, p_d_CM_Y, p_d_CM_Z, p_device_SD_countsOfSheresTouching, p_device_spheres_in_SD_composite, nSpheres());

    return;
}