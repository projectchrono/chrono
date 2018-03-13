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

#include "ChGranular.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono_granular/ChGranularDefines.h"
#include "chrono_granular/utils/ChGranularUtilities_CUDA.cuh"

template <int>
__global__ void primingOperationsRectangularBox(int*, int*, int*, unsigned int*, unsigned int*, unsigned int);

void chrono::ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC::cleanup_simulation() {
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

    // Handle other memory allocated
    if (p_device_SD_NumOf_DEs_Touching != nullptr) {
        gpuErrchk(cudaFree(p_device_SD_NumOf_DEs_Touching));
        p_device_SD_NumOf_DEs_Touching = nullptr;
    }
    if (p_device_DEs_in_SD_composite != nullptr) {
        gpuErrchk(cudaFree(p_device_DEs_in_SD_composite));
        p_device_DEs_in_SD_composite = nullptr;
    }
}

/** This method sets up the data structures used to perform a simulation.
 *
 */
void chrono::ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC::setup_simulation() {
    partition_BD();

    // set aside device memory to store the position of the CM of the spheres
    gpuErrchk(cudaMalloc(&p_d_CM_X, nDEs * sizeof(int)));
    gpuErrchk(cudaMalloc(&p_d_CM_Y, nDEs * sizeof(int)));
    gpuErrchk(cudaMalloc(&p_d_CM_Z, nDEs * sizeof(int)));

    // Set aside memory for velocity information
    gpuErrchk(cudaMalloc(&p_d_CM_XDOT, nDEs * sizeof(int)));
    gpuErrchk(cudaMalloc(&p_d_CM_YDOT, nDEs * sizeof(int)));
    gpuErrchk(cudaMalloc(&p_d_CM_ZDOT, nDEs * sizeof(int)));

    // allocate mem for array saying for each SD how many spheres touch it
    gpuErrchk(cudaMalloc(&p_device_SD_NumOf_DEs_Touching, nSDs * sizeof(unsigned int)));

    // allocate mem for array that for each SD has the list of all spheres touching it; big array
    gpuErrchk(cudaMalloc(&p_device_DEs_in_SD_composite, MAX_COUNT_OF_DEs_PER_SD * nSDs * sizeof(unsigned int)));

    // Copy over initial position information
    gpuErrchk(cudaMemcpy(p_d_CM_X, h_X_DE.data(), nDEs * sizeof(int), cudaMemcpyHostToDevice));
    gpuErrchk(cudaMemcpy(p_d_CM_Y, h_Y_DE.data(), nDEs * sizeof(int), cudaMemcpyHostToDevice));
    gpuErrchk(cudaMemcpy(p_d_CM_Z, h_Z_DE.data(), nDEs * sizeof(int), cudaMemcpyHostToDevice));

    // Set initial velocities to zero
    gpuErrchk(cudaMemset(p_d_CM_XDOT, 0, nDEs * sizeof(int)));
    gpuErrchk(cudaMemset(p_d_CM_YDOT, 0, nDEs * sizeof(int)));
    gpuErrchk(cudaMemset(p_d_CM_ZDOT, 0, nDEs * sizeof(int)));
}

void chrono::ChGRN_MONODISP_SPH_IN_BOX_NOFRIC_SMC::generate_DEs() {
    // Create the falling balls
    float ball_epsilon = monoDisperseSphRadius_SU / 200.f;  // Margin between balls to ensure no overlap / DEM-splosion

    chrono::utils::HCPSampler<float> sampler(2 * monoDisperseSphRadius_SU + ball_epsilon);  // Add epsilon

    // We need to pass in half-length box here
    ChVector<float> boxCenter(0, 0, 0);
    // We need to subtract off a sphere radius to ensure we don't get put at the edge
    ChVector<float> hdims{float(box_L / (2. * LENGTH_UNIT) - monoDisperseSphRadius_SU),
                          float(box_D / (2. * LENGTH_UNIT) - monoDisperseSphRadius_SU),
                          float(box_H / (2. * LENGTH_UNIT) - monoDisperseSphRadius_SU)};
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

/**
This method figures out how big a SD is, and how many SDs are going to be necessary
in order to cover the entire BD.
BD: Bid domain.
SD: Sub-domain.
*/
void chrono::ChGRN_DE_MONODISP_SPH_IN_BOX_SMC::partition_BD() {
    double tempDIM = 2. * sphere_radius * AVERAGE_SPHERES_PER_SD_L_DIR;
    unsigned int howMany = (unsigned int)(std::ceil(box_L / tempDIM));
    // work with an even kFac to hit the CM of the box.
    if (howMany & 1)
        howMany++;
    tempDIM = box_L / howMany;
    SD_L_SU = (unsigned int)std::ceil(tempDIM / LENGTH_UNIT);
    nSDs_L_SU = howMany;

    tempDIM = 2. * sphere_radius * AVERAGE_SPHERES_PER_SD_D_DIR;
    howMany = (unsigned int)(std::ceil(box_D / tempDIM));
    // work with an even kFac to hit the CM of the box.
    if (howMany & 1)
        howMany++;
    tempDIM = box_D / howMany;
    SD_D_SU = (unsigned int)std::ceil(tempDIM / LENGTH_UNIT);
    nSDs_D_SU = howMany;

    tempDIM = 2. * sphere_radius * AVERAGE_SPHERES_PER_SD_H_DIR;
    howMany = (unsigned int)(std::ceil(box_H / tempDIM));
    // work with an even kFac to hit the CM of the box.
    if (howMany & 1)
        howMany++;
    tempDIM = box_H / howMany;
    SD_H_SU = (unsigned int)std::ceil(tempDIM / LENGTH_UNIT);
    nSDs_H_SU = howMany;

    nSDs = nSDs_L_SU * nSDs_D_SU * nSDs_H_SU;
}

/**
This method define the mass, time, length Simulation Units. It also sets several other constants that enter the scaling
of various physical quantities set by the user.
*/
void chrono::ChGRN_DE_MONODISP_SPH_IN_BOX_SMC::switch_to_SimUnits() {
    double massSphere = 4. / 3. * M_PI * sphere_radius * sphere_radius * sphere_radius * sphere_density;
    MASS_UNIT = massSphere;
    K_stiffness = (modulusYoung_SPH2SPH > modulusYoung_SPH2WALL ? modulusYoung_SPH2SPH : modulusYoung_SPH2WALL);
    TIME_UNIT = sqrt(massSphere / (PSI_h*K_stiffness)) / PSI_T;
    
    double magGravAcc = sqrt(X_accGrav*X_accGrav + Y_accGrav*Y_accGrav + Z_accGrav*Z_accGrav);
    LENGTH_UNIT = massSphere * magGravAcc / (PSI_L * K_stiffness);

    monoDisperseSphRadius_SU = sphere_radius / LENGTH_UNIT;
    reciprocal_sphDiam_SU = 1. / (2. * monoDisperseSphRadius_SU);

    float scalingFactor = ((float)PSI_L) / (PSI_T*PSI_T*PSI_h);
    gravAcc_X_factor_SU = scalingFactor * X_accGrav / magGravAcc;
    gravAcc_Y_factor_SU = scalingFactor * Y_accGrav / magGravAcc;
    gravAcc_Z_factor_SU = scalingFactor * Z_accGrav / magGravAcc;
}
