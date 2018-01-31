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
    if (p_d_CM_X != nullptr) cudaFree(p_d_CM_X);
    if (p_d_CM_Y != nullptr) cudaFree(p_d_CM_Y);
    if (p_d_CM_Z != nullptr) cudaFree(p_d_CM_Z);

    if (p_d_CM_XDOT != nullptr) cudaFree(p_d_CM_XDOT);
    if (p_d_CM_YDOT != nullptr) cudaFree(p_d_CM_YDOT);
    if (p_d_CM_ZDOT != nullptr) cudaFree(p_d_CM_ZDOT);
}


/** This method sets up the data structures used to perform a simulation.
*
*/
void chrono::ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC::setup_simulation() {

    gpuErrchk(cudaMalloc((void**)& p_d_CM_X, nDEs * sizeof(float)));
    gpuErrchk(cudaMalloc((void**)& p_d_CM_Y, nDEs * sizeof(float)));
    gpuErrchk(cudaMalloc((void**)& p_d_CM_Z, nDEs * sizeof(float)));

}