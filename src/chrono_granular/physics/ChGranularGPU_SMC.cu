#include <cuda.h>
#include <climits>
#include "../../chrono_thirdparty/cub/cub.cuh"
#include "../ChGranularDefines.h"
#include "assert.h"

#define BLAH_F 0.f
#define BLAH_I 0
#define NULL_SD UINT_MAX-1 
#define NULL_SphID UINT_MAX-1 

__device__ dim3 whereSphCenterIs(const float* const sphereXYZ, const float3& xyzOriginBox, const float4& eulerParamBox, const ushort3& SDdims, const dim3& RectangularBoxDims)
{
    return BLAH_I;
}

__device__ size_t figureOutTouchedSDs(const dim3& , unsigned int* touchedSDs);

/**
* This kernel call prepares information that will be used in a subsequent kernel that performs the actual time stepping.
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
*   - A sphere cannot touch more than eight SDs
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
*
* Notes:
*   - The SD with ID=0 is the catch-all SD. This is the SD in which a sphere ends up if its not inside the rectangular box. Usually, there is no sphere in this SD
*
*
*/
template<unsigned short int BLOCK_THREADS, unsigned short int SHMEM_UINT, unsigned short int SHMEM_FL>
__global__ void primingOperationsRectangularBox(
    float3 xyzOriginBox,                      //!< Set of three floats that give the location of the rectangular box in the Global Reference Frame
    float4 eulerParamBox,                     //!< Set of four floats that provide the orientation of the rectangular box in the Global Reference Frame
    ushort3 SD_dims,                          //!< Set of three ints that provide the dimension (in multiple of Sphere radia) of a SD
    dim3 RectangularBox_dims,                 //!< The dimension of the rectangular box. The 3D box is expressed in multpiples of SD, in the X, Y, and Z directions, respectively
    float* pRawDataArray,                     //!< Pointer to array containing data related to the spheres in the box
    unsigned int* SD_countsOfSheresTouching,  //!< The array that for each SD indicates how many spheres touch this SD
    unsigned int* spheres_in_SD_composite,    //!< Big array that works in conjunction with SD_countsOfSheresTouching. "spheres_in_SD_composite" says which SD contains what spheres
    size_t nSpheres                           //!< Number of spheres in the box
)
{
    /// Set aside some shared memory (need to look how this gets word aligned)
    __shared__ unsigned int shMem_UINT[SHMEM_UINT];
    __shared__ float shMem_float[SHMEM_FL];


    // We work with a 1D block structure and a 3D grid structure
    unsigned int mySphereID = threadIdx.x + blockIdx.x * blockDim.x * blockDim.y * blockDim.z;

    unsigned short int N_SDsTouched = 0;           /// stores the number of SDs touched by this sphere
    float sphereXYZ[3];                            /// the coordinates of the sphere
    unsigned int touchedSDs[8] = { NULL_SD, NULL_SD, NULL_SD, NULL_SD, NULL_SD, NULL_SD, NULL_SD, NULL_SD };  /// The equivalent of a non-touchable SD
    unsigned int sphereID  [8] = { NULL_SphID, NULL_SphID, NULL_SphID, NULL_SphID, NULL_SphID, NULL_SphID, NULL_SphID, NULL_SphID };  /// The equivalent of a non-existent sphere
    if (mySphereID < nSpheres) {
        // Bring the "center of sphere" information via CUB
        typedef cub::BlockLoad<float, BLOCK_THREADS, 3, cub::BLOCK_LOAD_WARP_TRANSPOSE> BlockLoad;
        // Allocate shared memory for BlockLoad
        __shared__ typename BlockLoad::TempStorage temp_storage;
        // Load a segment of consecutive items that are blocked across threads
        BlockLoad(temp_storage).Load(pRawDataArray + 3*mySphereID, sphereXYZ);

        // Find out which SDs are touched by this sphere
        // NOTE: A sphere might also touch the "catchAll_SD"
        // "catchAll_SD": subdomain that encompasses all the universe except the RectangularBox of interest
        dim3 whichSD_SphCenterIsIn = whereSphCenterIs(sphereXYZ, xyzOriginBox, eulerParamBox, SD_dims, RectangularBox_dims);
        N_SDsTouched = figureOutTouchedSDs(whichSD_SphCenterIsIn, touchedSDs); ///NOT DONE YET

        // Load the proper value, to be used later for key-value sort
        sphereID[0] = mySphereID;
        sphereID[1] = mySphereID;
        sphereID[2] = mySphereID;
        sphereID[3] = mySphereID;
        sphereID[4] = mySphereID;
        sphereID[5] = mySphereID;
        sphereID[6] = mySphereID;
        sphereID[7] = mySphereID;
    }

    __syncthreads();

    // Do a collective SIMT operation: "reduce_op" to figure out how much memory needs to be set aside  
    __shared__ unsigned short int totalNumberOfSphere_SD_touches;
    typedef cub::BlockReduce<unsigned short int, BLOCK_THREADS, cub::BLOCK_REDUCE_RAKING_COMMUTATIVE_ONLY> BlockReduceT;
    __shared__ typename BlockReduceT::TempStorage temp_storage;
    unsigned short int dummy = BlockReduceT(temp_storage).Sum(N_SDsTouched);
    if (threadIdx.x == 0) {
        totalNumberOfSphere_SD_touches = dummy;
    }

    // Do a sort by key, sorting in increasing order; note that the entries that store the 
    // untouchable SD; i.e., NULL_SD, will migrate to the end of the array
    // The key: the domain touched by this Sphere
    // The value: the sphere ID
    typedef cub::BlockRadixSort<unsigned int, BLOCK_THREADS, 8, unsigned int> BlockRadixSort;
    __shared__ typename BlockRadixSort::TempStorage temp_storage_sort;
    BlockRadixSort(temp_storage_sort).Sort(touchedSDs, sphereID);

    // Stitch together in ShMem the unsorted array containing the SDs touched by the sphere handled in this block
    // Specialize BlockStore for a 1D block using BLOCK_THREADS threads each and owning 8 unsigned integer items each
    typedef cub::BlockStore<unsigned int, BLOCK_THREADS, 8, cub::BLOCK_STORE_VECTORIZE > BlockStore; // make sure address is quadword-aligned
    // Allocate shared memory for BlockStore
    __shared__ typename BlockStore::TempStorage temp_storage_stitching;
    BlockStore(temp_storage_stitching).Store(shMem_UINT, touchedSDs);

    // Figure out what each thread needs to check in terms of Heaviside-step in the array of SDs 
    // touched; the BLOCK_THREADS is promoted to unsigned int here
    unsigned int dummyUINT = (totalNumberOfSphere_SD_touches + BLOCK_THREADS - 1) / BLOCK_THREADS;

    unsigned int startPoint = threadIdx.x * dummyUINT;
    unsigned int endPoint   = startPoint + dummyUINT;

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