#include <cuda.h>
#include<stdint.h>
#include "../../chrono_thirdparty/cub/cub.cuh"

#define BLAH_F 0.f
#define BLAH_I 0

using namespace cub;


__device__ dim3 whichSD_SphCenterIs(const float* const sphereXYZ, const float3& xyzOriginBox, const float4& eulerParamBox, const ushort3& SDdims, const dim3& RectangularBoxDims)
{
    return BLAH_I;
}

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
*
* Basic idea: use domain decomposition on the rectangular box and have one subdomain be processed by one block.
* The subdomains are axis-aligned relative to the reference frame associated with the *box*. The origin of the box is at the center of the box.
* The orientation of the box is defined relative to a world inertial reference frame.
*
* Nomenclature:
*   - SD: subdomain.
*   - BD: the big-domain, which is the union of all SDs
*
*/
template<unsigned short int BLOCK_THREADS>
__global__ void primingOperationsRectangularBox(
    float3 xyzOriginBox,        //!< Set of three floats that give the location of the rectangular box in the Global Reference Frame
    float4 eulerParamBox,       //!< Set of four floats that provide the orientation of the rectangular box in the Global Reference Frame
    ushort3 SD_dims,            //!< Set of three ints that provide the dimension (in multiple of Sphere radia) of a SD
    dim3 RectangularBox_dims,   //!< The dimension of the rectangular box. The 3D box is expressed in multpiples of SD, in the X, Y, and Z directions, respectively
    float* pRawDataArray,       //!< Pointer to array containing data related to the spheres in the box
    size_t nSpheres             //!< Number of spheres in the box
)
{
    /// The amount of shared memory is allocated dynamically.  
    extern __shared__ float shMem[];
    const size_t offsetShMem = 1000;     /// This is totally bogus for now; need to revisit

    float* const shMemFLOAT = shMem;                               // constant pointer to float 
    int*   const shMemINT   = (int*) &shMemFLOAT[offsetShMem];     // I wonder whether there are mem alignment issues here

    // We work with a 1D block structure and a 3D grid structure
    size_t mySphereID = threadIdx.x + blockIdx.x * blockDim.x * blockDim.y * blockDim.z;

    size_t N_SDsTouched = 0;     /// stores the number of SDs touched by this sphere
    float sphereXYZ[3];          /// the coordinates of the sphere
    if (mySphereID < nSpheres) {
        // Bring the sphere information and place via CUB
        typedef cub::BlockLoad<float, BLOCK_THREADS, 3, BLOCK_LOAD_WARP_TRANSPOSE> BlockLoad;
        // Allocate shared memory for BlockLoad
        __shared__ typename BlockLoad::TempStorage temp_storage;
        // Load a segment of consecutive items that are blocked across threads
        BlockLoad(temp_storage).Load(pRawDataArray, sphereXYZ);

        // Find out which SDs are touched by this sphere
        // NOTE: A sphere might also touch the "catchAll_SD"
        // "catchAll_SD": subdomain that encompasses all the universe except the RectangularBox of interest
        dim3 whichSD_SphCenterIs = whereSphCenterIs(sphereXYZ, xyzOriginBox, eulerParamBox, SD_dims, RectangularBox_dims);
        

    }

    // Do a collective SIMT operation: "reduce_op" to figure out how much memory needs to be set aside  
    __shared__ size_t totalNumberOfSphere_SD_touches;
    typedef cub::BlockReduce<size_t, BLOCK_THREADS, BLOCK_REDUCE_RAKING_COMMUTATIVE_ONLY> BlockReduceT;
    __shared__ typename BlockReduceT::TempStorage temp_storage;
    size_t dummy = BlockReduceT(temp_storage).Sum(N_SDsTouched); 
    if (threadIdx.x == 0) {
        totalNumberOfSphere_SD_touches = dummy;
    }
    __syncthreads();


    // Do a sort by key
    // The key: the domain touched by this Sphere
    // The value: the offset in ShMem where data for this Sphere is stored

    // Place the sorted data in ShMem

    // Note: totalNumberOfSphere_SD_touches is always greater than zero since 
    // at a minimum there is the catchAllSD that is touched
    if (threadIdx.x < totalNumberOfSphere_SD_touches) {
        if (threadIdx.x == 0) {
            // There is no left term for this guy
        }
        else {
            // Check if this thread is a "SD discontinuity thread", and if so, see how many spheres happen to touch the SD
            // "SD discontinuity thread": a thread which when looking at left, sees an SD that is unlike the SD it has
        }

    }
    __syncthreads(); // needed in preparation for the sort operations

    // Do a collective SIMT operation: sort by key



    // add the spheres to the right SD

    /// 
    return;
}