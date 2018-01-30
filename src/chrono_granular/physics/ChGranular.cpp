#include <cuda_runtime.h>
#include "ChGranular.h"
#include "chrono_granular/utils/ChGranularUtilities_CUDA.cuh"


chrono::ChGRN_DE_Container::~ChGRN_DE_Container() {
    // Clean up, the host side
    if (pGRN_xyz_DE != nullptr)
        delete[] pGRN_xyz_DE;
    if (pGRN_xyzDOT_DE != nullptr)
        delete[] pGRN_xyzDOT_DE;

    // Clean up, the device side
    if (p_device_GRN_xyz_DE != nullptr)
        cudaFree(pGRN_xyz_DE);
    if (p_device_GRN_xyzDOT_DE != nullptr)
        cudaFree(pGRN_xyzDOT_DE);
}


/** This method sets up the data structures used to perform a simulation.
*
*/
void chrono::ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC::setup_simulation() {

    gpuErrchk( cudaMalloc((void**)& p_device_GRN_xyz_DE, 3*nDEs * sizeof(float)) );

}