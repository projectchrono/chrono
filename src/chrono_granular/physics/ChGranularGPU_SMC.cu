#include <cuda.h>
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
* Basic idea: use domain decomposition on the rectangular box and have one subdomain be processed by one block.
* The subdomains are axis-aligned relative to the reference frame associated with the *box*. The origin of the box is at the center of the box.
* The orientation of the box is defined relative to a world inertial reference frame.
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

    // We work with a 1D block structure
    size_t mySphereID = threadIdx.x + blockIdx.x * blockDim.x;

    // Some threads end up doing no work
    if (mySphereID < nSpheres) {
        ;
    }

    /// 
    return;
}