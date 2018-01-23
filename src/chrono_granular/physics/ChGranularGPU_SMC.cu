#include <cuda.h>
#include<stdint.h>
#include "../../chrono_thirdparty/cub/cub.cuh"
/**
* This kernel call prepres information that will be used in a subsequent kernel that performs the actual time stepping.
* 
* Assumptions:
*   - Granular material is made up of spheres. 
*   - For now, all spheres are of constant radius
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
__global__ void primingOperationsRectangularBox(
    float3 xyzOriginBox,        //!< Set of three floats, that give the location of the rectangular box in the Global Reference Frame
    float4 eulerParamBox,       //!< Set of four floats that provide the orientation of the rectangular box in the Global Reference Frame
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

    uint8_t N_SDsTouched = 0; /// stores the number of SDs touched by this sphere
    if (mySphereID < nSpheres) {
        // Bring the sphere information and place it in shared memory

        // Find out which SDs are touched by this sphere
        // NOTE: A sphere might touch the "catchAllDS"

        // Once N_SDsTouched is computed, copy it into shared memory in preparation for a prefix scan operation

    }

    __syncthreads();

    // Do a collective SIMT operation: "reduce_op" to figure out how much memory needs to be set aside  
    size_t totalNumberOfSphere_SD_touches = 0; // actual CUB reduce call here: 

    // Do a sort by key
    // The key: the domain touched by this Sphere
    // The value: the offset in ShMem where data for this Sphere is stored

    // Place the sorted data in ShMem

    // Note: totalNumberOfSphere_SD_touches is always greater than zero since 
    // at a minimum there is the catchAllDS that is touched
    if (threadIdx.x < totalNumberOfSphere_SD_touches) {
        if (threadIdx.x == 0) {
            // There is no left term for this guy
        }
        else {
            // Check if this thread is a "SD discontinuity thread", and if so, see how many spheres happen to touch thi SD
        }

    }
    __syncthreads(); // needed in preparation for the sort operations

    // Do a collective SIMT operation: sort by key



    // add the spheres to the right SD

    /// 
    return;
}