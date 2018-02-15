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
#include <cuda_runtime.h>
#include "ChGranular.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono_granular/ChGranularDefines.h"
#include "chrono_granular/utils/ChGranularUtilities_CUDA.cuh"

template <int>
__global__ void primingOperationsRectangularBox(int*, int*, int*, unsigned int*, unsigned int*, unsigned int);

chrono::ChGRN_DE_Container::~ChGRN_DE_Container() {
    // Handle position level information
    if (p_d_CM_X != nullptr) {
        gpuErrchk(cudaFree(p_d_CM_X));
        p_d_CM_X = nullptr;
    }
    if (p_d_CM_Y != nullptr) {
        gpuErrchk(cudaFree(p_d_CM_Y));
        p_d_CM_Y = nullptr;
    }
    if (p_d_CM_Z != nullptr) {
        gpuErrchk(cudaFree(p_d_CM_Z));
        p_d_CM_Z = nullptr;
    }

    // Handle velocity level information
    if (p_d_CM_XDOT != nullptr) {
        gpuErrchk(cudaFree(p_d_CM_XDOT));
        p_d_CM_XDOT = nullptr;
    }
    if (p_d_CM_YDOT != nullptr) {
        gpuErrchk(cudaFree(p_d_CM_YDOT));
        p_d_CM_YDOT = nullptr;
    }
    if (p_d_CM_ZDOT != nullptr) {
        gpuErrchk(cudaFree(p_d_CM_ZDOT));
        p_d_CM_ZDOT = nullptr;
    }
}

/** This method sets up the data structures used to perform a simulation.
 *
 */
void chrono::ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC::setup_simulation() {
    partition_BD();

    // set aside device memory to store the position of the CM of the spheres
    gpuErrchk(cudaMalloc((void**)&p_d_CM_X, nDEs * sizeof(int)));
    gpuErrchk(cudaMalloc((void**)&p_d_CM_Y, nDEs * sizeof(int)));
    gpuErrchk(cudaMalloc((void**)&p_d_CM_Z, nDEs * sizeof(int)));
    // TODO this seems to be allocating too much device memory, since we index into it with an SD index
    // Shouldn't it be nSDs * sizeof(unsigned int)
    gpuErrchk(cudaMalloc((void**)&p_device_SD_NumOf_DEs_Touching, nDEs * sizeof(unsigned int)));

    gpuErrchk(cudaMalloc((void**)&p_device_DEs_in_SD_composite, MAX_COUNT_OF_DEs_PER_SD * nSDs * sizeof(unsigned int)));
    gpuErrchk(cudaMemcpy(p_d_CM_X, h_X_DE.data(), nDEs * sizeof(int), cudaMemcpyHostToDevice));
    gpuErrchk(cudaMemcpy(p_d_CM_Y, h_Y_DE.data(), nDEs * sizeof(int), cudaMemcpyHostToDevice));
    gpuErrchk(cudaMemcpy(p_d_CM_Z, h_Z_DE.data(), nDEs * sizeof(int), cudaMemcpyHostToDevice));
}

void chrono::ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC::generate_DEs() {
    // Create the falling balls
    float ball_epsilon = monoDisperseSphRadius_AD / 200.f;  // Margin between balls to ensure no overlap / DEM-splosion

    chrono::utils::HCPSampler<float> sampler(2 * monoDisperseSphRadius_AD + ball_epsilon);  // Add epsilon

    // We need to pass in half-length box here
    ChVector<float> boxCenter(0, 0, 0);
    // We need to subtract off a sphere radius to ensure we don't get put at the edge
    ChVector<float> hdims{float(box_L / (2. * SPACE_UNIT) - monoDisperseSphRadius_AD),
                          float(box_D / (2. * SPACE_UNIT) - monoDisperseSphRadius_AD),
                          float(box_H / (2. * SPACE_UNIT) - monoDisperseSphRadius_AD)};
    std::vector<ChVector<float>> points = sampler.SampleBox(boxCenter, hdims);  // Vector of points

    nDEs = (unsigned int)points.size();
    std::cout << nDEs << " balls added!" << std::endl;
    // Allocate space for new bodies
    h_X_DE.resize(nDEs);
    h_Y_DE.resize(nDEs);
    h_Z_DE.resize(nDEs);
    // Copy from array of structs to 3 arrays
    for (unsigned int i = 0; i < nDEs; i++) {
        auto vec = points.at(i);
        h_X_DE.at(i) = (int)vec.x();
        h_Y_DE.at(i) = (int)vec.y();
        h_Z_DE.at(i) = (int)vec.z();
    }

    h_XDOT_DE.resize(nDEs);
    h_YDOT_DE.resize(nDEs);
    h_ZDOT_DE.resize(nDEs);
}

/** The purpose of this method is to figure out how big a SD is, and how many SDs are going to be necessary
in order to cover the entire BD.
*/
void chrono::ChGRN_DE_MONODISP_SPH_IN_BOX_SMC::partition_BD() {
    double tempDIM = 2. * sphere_radius * AVERAGE_SPHERES_PER_SD_L_DIR;
    unsigned int howMany = (unsigned int)(std::ceil(box_L / tempDIM));
    // work with an even kFac to hit the CM of the box.
    if (howMany & 1)
        howMany++;
    tempDIM = box_L / howMany;
    SD_L_AD = (unsigned int)std::ceil(tempDIM / SPACE_UNIT);
    nSDs_L_AD = howMany;

    tempDIM = 2. * sphere_radius * AVERAGE_SPHERES_PER_SD_D_DIR;
    howMany = (unsigned int)(std::ceil(box_D / tempDIM));
    // work with an even kFac to hit the CM of the box.
    if (howMany & 1)
        howMany++;
    tempDIM = box_D / howMany;
    SD_D_AD = (unsigned int)std::ceil(tempDIM / SPACE_UNIT);
    nSDs_D_AD = howMany;

    tempDIM = 2. * sphere_radius * AVERAGE_SPHERES_PER_SD_H_DIR;
    howMany = (unsigned int)(std::ceil(box_H / tempDIM));
    // work with an even kFac to hit the CM of the box.
    if (howMany & 1)
        howMany++;
    tempDIM = box_H / howMany;
    SD_H_AD = (unsigned int)std::ceil(tempDIM / SPACE_UNIT);
    nSDs_H_AD = howMany;

    nSDs = nSDs_L_AD * nSDs_D_AD * nSDs_H_AD;
}