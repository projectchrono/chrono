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
* Basic idea: use domain decomposition and have one subdomain be processed by one block.
* The subdomains are axis-aligned relative to the reference frame associated with the *box*. The origin of the box is at the center of the box
*/
__global__ void primingOperationsRectangularBox(
    float3 xyzOriginBox,        //!< xyzOriginBox a set of three floats, that give the location of the rectangular box in the Global Reference Frame
    float4 eulerParamBox,       //!< set of four floats that provide the orientation of the rectangular box in the Global Reference Frame
    float* pRawDataArray)
{
    /// The amount of shared memory is allocated dynamically.  
    extern __shared__ float shMem[];

    /// The 
    return;
}