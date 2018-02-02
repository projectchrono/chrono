#include <cuda_runtime.h>
#include "ChGranular.h"
#include "chrono_granular/utils/ChGranularUtilities_CUDA.cuh"


chrono::ChGRN_DE_Container::~ChGRN_DE_Container() {
    // Clean up, the host side
    if (p_h_xyz_DE != nullptr)
        delete[] p_h_xyz_DE;
    if (p_h_xyzDOT_DE != nullptr)
        delete[] p_h_xyzDOT_DE;

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

    
    gpuErrchk(cudaMalloc((void**)& p_device_SD_NumOf_DEs_Touching, nDEs * sizeof(unsigned int)));

    gpuErrchk(cudaMalloc((void**)& p_device_DEs_in_SD_composite, MAX_COUNT_OF_DEs_PER_SD * nSDs * sizeof(unsigned int)));

}