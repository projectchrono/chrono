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



__device__ dim3 whereSphCenterIs(const float* const sphereXYZ, const float3& xyzOriginBox, const float4& eulerParamBox, const dim3& SDdims, const dim3& RectangularBoxDims)
{
    assert(0);
    return BLAH_BLAH_I;
}

__device__ size_t figureOutTouchedSDs(const dim3&);

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
    unsigned short int CUB_THREADS,           //!< Number of CUB threads engaged in block-collective CUB operations. Should be a multiple of warp size (32) and less than or equal to min(256, blockDim.x)
    unsigned short int AMOUNT_SHMEM_KB        //!< The amount of shared mem to be used (in KB). Asking for lots of shared memory will make the occupancy go down
>
__global__ void primingOperationsRectangularBox(
    float* pRawDataArray,                     //!< Pointer to array containing data related to the spheres in the box
    unsigned int* SD_countsOfSheresTouching,  //!< The array that for each SD indicates how many spheres touch this SD
    unsigned int* spheres_in_SD_composite,    //!< Big array that works in conjunction with SD_countsOfSheresTouching. "spheres_in_SD_composite" says which SD contains what spheres
    size_t nSpheres                           //!< Number of spheres in the box
)
{
    /// Set aside shared memory
    const unsigned int sphere_in_SD_events_UPPERBOUND = ((AMOUNT_SHMEM_KB * 1024) / (2 * sizeof(unsigned int)));
    __shared__ unsigned int shMem_UINT[2 * sphere_in_SD_events_UPPERBOUND];
    const unsigned short strideCUB = sphere_in_SD_events_UPPERBOUND / CUB_THREADS; // this is how many elements of data in ShMem are going to be assigned per lane when doing CUB block collective operations

    // Figure out the memory bounds...
    unsigned int* const touchedSDs = shMem_UINT;
    unsigned int* const sphereID   = shMem_UINT + sphere_in_SD_events_UPPERBOUND;
  
    __shared__ unsigned int seedData[16];
    if (threadIdx.x < 16)
        seedData[threadIdx.x] = NULL_GRANULAR_ID; // needed later to pad array to get the sort to do what's needed
    // I don't think i need a __syncthreads here owing to a later CUB call that hits these 16 lanes <-- DOUBLE CHECK


    // We work with a 1D block structure and a 3D grid structure
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockIdx.y * blockIdx.z * blockDim.x;

    unsigned int N_SDsTouched = 0;                 /// stores the number of SDs touched by this sphere
    unsigned int thisSphereOffset = 0;             /// says where this sphere should deposit info in the SD array "spheres_in_SD_composite"
    float sphereXYZ[3];                            /// the coordinates of this sphere
    dim3 whichSD_SphCenterIsIn;                    /// the XYZ coordinates of the SD that contains the center of this sphere

    if (mySphereID < nSpheres) {
        // Bring over the "center of sphere" information via CUB
        typedef cub::BlockLoad<float, CUB_THREADS, 3, cub::BLOCK_LOAD_WARP_TRANSPOSE> Block_Load;
        // Allocate shared memory for BlockLoad
        __shared__ typename Block_Load::TempStorage temp_storage;
        // Load a segment of consecutive items that are blocked across threads
        BlockLoad(temp_storage).Load(pRawDataArray + 3 * mySphereID, sphereXYZ);

        // Find out which SDs are touched by this sphere
        // NOTE: A sphere might also touch the "catchAll_SD"
        // "catchAll_SD": subdomain that encompasses all the universe except the RectangularBox of interest
        whichSD_SphCenterIsIn = whereSphCenterIs(sphereXYZ, xyzOriginBox, eulerParamBox, SD_dims, RectangularBox_dims);
        N_SDsTouched = figureOutTouchedSDs(whichSD_SphCenterIsIn); /// <--- TAKE CARE; NOT IMPLEMENTED YET
    }


    // Next, do a collective SIMT operation: "inclusive prefix scan". Figure out how much shared memory needs to be available
    unsigned int totalNumberOfSphere_SD_touches;
    typedef cub::BlockScan<unsigned int, CUB_THREADS> BlockScan;
    __shared__ typename BlockScan::TempStorage temp_storage_SCAN;
    BlockScan(temp_storage_SCAN).InclusiveSum(N_SDsTouched, thisSphereOffset, totalNumberOfSphere_SD_touches);
    thisSphereOffset -= N_SDsTouched; // correction necessary given that this was an inclusive scan (necessary to produce "totalNumberOfSphere_SD_touches")

    // Is enough ShMem available? If not, we need to abort and the user should crank up AMOUNT_SHMEM_KB
    if (totalNumberOfSphere_SD_touches > sphere_in_SD_events_UPPERBOUND) 
        asm("trap;"); // Signal an error to the host (general launch failure IIRC); I'll change later to something tamer


    // Start with a clean slate of data in shared memory. Note that dummyUINT cannot be larger than 16, which explains the way "seedData" was set up.
    // The value 16 is tied to the fact that each sphere can touch at the most 8 SDs
    typedef cub::BlockStore<unsigned int, CUB_THREADS, 2*strideCUB, cub::BLOCK_STORE_WARP_TRANSPOSE> BlockStore; // is BLOCK_STORE_WARP_TRANSPOSE what i want?
    __shared__ typename BlockStore::TempStorage temp_storageOP;
    BlockStore(temp_storageOP).Store(shMem_UINT, seedData);

    // Get the actual work done
    unsigned int dummyUINT;
    if (mySphereID < nSpheres) {
        // Find out which SDs are touched by this sphere
        // NOTE: A sphere might also touch the "catchAll_SD"
        // "catchAll_SD": subdomain that encompasses all the universe except the RectangularBox of interest

        // Load the proper value, to be used later for key-value sort
        for (dummyUINT = 0; dummyUINT < N_SDsTouched; dummyUINT++) {
            touchedSDs[thisSphereOffset + dummyUINT] = BLAH_BLAH_I; // <-- TAKE CARE OF THIS; fake news
            sphereID  [thisSphereOffset + dummyUINT] = mySphereID;
        }
    }

    // Do a sort by key, sorting in increasing order; note that the entries that store the 
    // untouchable SD; i.e., NULL_GRANULAR_ID, will migrate to the end of the array
    // The key: the domain touched by this Sphere
    // The value: the sphere ID
    dummyUINT = strideCUB * threadIdx.x;
    typedef cub::BlockRadixSort<unsigned int, CUB_THREADS, strideCUB, unsigned int> BlockRadixSort;
    __shared__ typename BlockRadixSort::TempStorage temp_storage_sort;
    BlockRadixSort(temp_storage_sort).Sort(touchedSDs + dummyUINT, sphereID + dummyUINT);

    // Figure out what each thread needs to check in terms of Heaviside-step in the array of SDs 
    // touched; the CUB_THREADS is promoted to unsigned int here
    dummyUINT = (totalNumberOfSphere_SD_touches + CUB_THREADS - 1) / CUB_THREADS;

    unsigned int startPoint = threadIdx.x * dummyUINT;
    unsigned int endPoint = startPoint + dummyUINT;

    // From spheres with IDs between startPoint to endPoint, figure out in which SD each sphere goes and 
    // do prep work to place it in there. No need to synchronize the block threads since although we
    // write to shared memory, we only work within the startPoint to endPoint bounds. There is some 
    // atomicAdd overhead, but that's going to be cached in L2 --> something to look into in the future
    unsigned int ref_SDid = shMem_UINT[startPoint];
    unsigned int n_repetitions = 0;
    for (unsigned int i = startPoint; i < endPoint; i++) {
        if (ref_SDid == shMem_UINT[i]) {
            // This is guaranteed to be hit for i=startPoint
            n_repetitions++;
        }
        else {
            dummyUINT = atomicAdd(SD_countsOfSheresTouching + ref_SDid, n_repetitions);
            ref_SDid = i - n_repetitions; // using ref_SDid as a dummy; soon to be set to something meaningful
            do {
                // Compute the offset in the SD data array where this sphere should deposit its information.
                // The "this sphere" is the sphere with ID stored in sphereID[i]. Note that we're 
                // overwritting what used to be in shMem_UINT; i.e., which SD this sphere touches, since
                // this information is not needed anymore
                shMem_UINT[ref_SDid++] = dummyUINT++; // exptected to give a coalesced mem access later on
                n_repetitions--;
            } while (n_repetitions > 0);
            ref_SDid = shMem_UINT[i]; // we have a new reference at this point; 
            n_repetitions = 1;        // set to 1 since each SD is hit at least once
        }
    }

    __syncthreads(); // Wating here on all threads to finish before writing the data to global memory.

    if (threadIdx.x < totalNumberOfSphere_SD_touches)
        spheres_in_SD_composite[shMem_UINT[threadIdx.x]] = sphereID[threadIdx.x];

    return;
}




void chrono::ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC::settle(float tEnd) {
    dim3 grid3D_dims(256, 256, 256);
    primingOperationsRectangularBox<256, 32> <<<grid3D_dims, 128 >>>(p_device_GRN_xyz_DE, p_device_SD_countsOfSheresTouching, p_device_spheres_in_SD_composite, nSpheres());

    return;
}