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
// Authors: Dan Negrut, Nic Olsen
// =============================================================================
/*! \file */

#include <cuda.h>
#include <cuda_runtime.h>
#include <cmath>
#include "ChGranular.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono_granular/ChGranularDefines.h"
#include "chrono_granular/utils/ChGranularUtilities_CUDA.cuh"

namespace chrono {
namespace granular {

double ChSystemGranularMonodisperse_SMC_Frictionless::get_max_K() {
    return std::max(YoungModulus_SPH2SPH, YoungModulus_SPH2WALL);
}

void ChSystemGranularMonodisperse_SMC_Frictionless::cleanup_simulation() {
    // deallocate unified device struct
    cudaFree(gran_params);
    // Vectors get deallocated automatically
}

/** This method sets up the data structures used to perform a simulation.
 *
 */
void ChSystemGranularMonodisperse_SMC_Frictionless::setup_simulation() {
    partition_BD();

    // allocate mem for array saying for each SD how many spheres touch it
    // gpuErrchk(cudaMalloc(&SD_NumOf_DEs_Touching, nSDs * sizeof(unsigned int)));
    SD_NumOf_DEs_Touching.resize(nSDs);
    // allocate mem for array that for each SD has the list of all spheres touching it; big array
    // gpuErrchk(cudaMalloc(&DEs_in_SD_composite, MAX_COUNT_OF_DEs_PER_SD * nSDs));
    DEs_in_SD_composite.resize(MAX_COUNT_OF_DEs_PER_SD * nSDs);
}

// Set the bounds to fill in our box
void ChSystemGranularMonodisperse::setFillBounds(float xmin,
                                                 float ymin,
                                                 float zmin,
                                                 float xmax,
                                                 float ymax,
                                                 float zmax) {
    boxFillXmin = xmin;
    boxFillYmin = ymin;
    boxFillZmin = zmin;
    boxFillXmax = xmax;
    boxFillYmax = ymax;
    boxFillZmax = zmax;
}

void ChSystemGranularMonodisperse::generate_DEs() {
    // Create the falling balls
    float ball_epsilon = sphereRadius_SU / 200.f;  // Margin between balls to ensure no overlap / DEM-splosion
    printf("eps is %f, rad is %5f\n", ball_epsilon, sphereRadius_SU * 1.0f);

    chrono::utils::HCPSampler<float> sampler(2.4 * sphereRadius_SU);  // Add epsilon

    // We need to pass in half-length box here

    // generate from bottom to twice the generateDepth
    // average high and low to get midpoint of generation
    float xmid = box_L * (boxFillXmax + boxFillXmin) / (4. * gran_params->LENGTH_UNIT);
    float ymid = box_D * (boxFillYmax + boxFillYmin) / (4. * gran_params->LENGTH_UNIT);
    float zmid = box_H * (boxFillZmax + boxFillZmin) / (4. * gran_params->LENGTH_UNIT);
    // half-spans in each dimension, the difference
    float xlen = abs(box_L * (boxFillXmax - boxFillXmin) / (4. * gran_params->LENGTH_UNIT));
    float ylen = abs(box_D * (boxFillYmax - boxFillYmin) / (4. * gran_params->LENGTH_UNIT));
    float zlen = abs(box_H * (boxFillZmax - boxFillZmin) / (4. * gran_params->LENGTH_UNIT));
    float generateHalfDepth = box_H / (3. * gran_params->LENGTH_UNIT);

    // float generateX = -box_D / (2. * gran_params->LENGTH_UNIT) + generateHalfDepth;
    // float generateY = -box_D / (2. * gran_params->LENGTH_UNIT) + generateHalfDepth;
    // float generateZ = -box_H / (2. * gran_params->LENGTH_UNIT) + generateHalfHeight;
    ChVector<float> boxCenter(xmid, ymid, zmid);
    // We need to subtract off a sphere radius to ensure we don't get put at the edge
    ChVector<float> hdims{xlen - sphereRadius_SU, ylen - sphereRadius_SU, zlen - sphereRadius_SU};
    std::vector<ChVector<float>> points = sampler.SampleBox(boxCenter, hdims);  // Vector of points

    nDEs = (unsigned int)points.size();
    std::cout << nDEs << " balls added!" << std::endl;
    // Allocate space for new bodies
    pos_X.resize(nDEs);
    pos_Y.resize(nDEs);
    pos_Z.resize(nDEs);
    pos_X_dt.resize(nDEs, 0);
    pos_Y_dt.resize(nDEs, 0);
    pos_Z_dt.resize(nDEs, 0);
    pos_X_dt_update.resize(nDEs, 0);
    pos_Y_dt_update.resize(nDEs, 0);
    pos_Z_dt_update.resize(nDEs, 0);
    // Copy from array of structs to 3 arrays
    for (unsigned int i = 0; i < nDEs; i++) {
        auto vec = points.at(i);
        pos_X.at(i) = (int)(vec.x());
        pos_Y.at(i) = (int)(vec.y());
        pos_Z.at(i) = (int)(vec.z());
        // printf("int is %d, float is %f\n", pos_X.at(i), vec.x());
    }
}

/**
This method figures out how big a SD is, and how many SDs are going to be necessary
in order to cover the entire BD.
BD: Bid domain.
SD: Sub-domain.
*/
void ChSystemGranularMonodisperse::partition_BD() {
    double tempDIM = 2. * sphere_radius * AVERAGE_SPHERES_PER_SD_L_DIR;
    unsigned int howMany = (unsigned int)(std::ceil(box_L / tempDIM));
    // work with an even kFac to hit the CM of the box.
    if (howMany & 1)
        howMany++;
    tempDIM = box_L / howMany;
    SD_L_SU = (unsigned int)std::ceil(tempDIM / gran_params->LENGTH_UNIT);
    nSDs_L = howMany;

    tempDIM = 2. * sphere_radius * AVERAGE_SPHERES_PER_SD_D_DIR;
    howMany = (unsigned int)(std::ceil(box_D / tempDIM));
    // work with an even kFac to hit the CM of the box.
    if (howMany & 1)
        howMany++;
    tempDIM = box_D / howMany;
    SD_D_SU = (unsigned int)std::ceil(tempDIM / gran_params->LENGTH_UNIT);
    nSDs_D = howMany;

    tempDIM = 2. * sphere_radius * AVERAGE_SPHERES_PER_SD_H_DIR;
    howMany = (unsigned int)(std::ceil(box_H / tempDIM));
    // work with an even kFac to hit the CM of the box.
    if (howMany & 1)
        howMany++;
    tempDIM = box_H / howMany;
    SD_H_SU = (unsigned int)std::ceil(tempDIM / gran_params->LENGTH_UNIT);
    nSDs_H = howMany;

    nSDs = nSDs_L * nSDs_D * nSDs_H;
    printf("%u Sds as %u, %u, %u\n", nSDs, nSDs_L, nSDs_D, nSDs_H);

    // Place BD frame at bottom-left corner, one half-length in each direction
    // Can change later if desired
    BD_frame_X = -.5 * (nSDs_L * SD_L_SU);
    BD_frame_Y = -.5 * (nSDs_D * SD_D_SU);
    BD_frame_Z = -.5 * (nSDs_H * SD_H_SU);
    // BD starts at rest
    BD_frame_X_dot = 0;
    BD_frame_Y_dot = 0;
    BD_frame_Z_dot = 0;
}

/**
This method defines the mass, time, length Simulation Units. It also sets several other constants that enter the scaling
of various physical quantities set by the user.
*/
void ChSystemGranularMonodisperse_SMC_Frictionless::switch_to_SimUnits() {
    double massSphere = 4. / 3. * M_PI * sphere_radius * sphere_radius * sphere_radius * sphere_density;
    gran_params->MASS_UNIT = massSphere;
    double K_stiffness = get_max_K();
    gran_params->TIME_UNIT = sqrt(massSphere / (PSI_h * K_stiffness)) / PSI_T;

    double magGravAcc = sqrt(X_accGrav * X_accGrav + Y_accGrav * Y_accGrav + Z_accGrav * Z_accGrav);
    gran_params->LENGTH_UNIT = massSphere * magGravAcc / (PSI_L * K_stiffness);

    sphereRadius_SU = sphere_radius / gran_params->LENGTH_UNIT;

    float scalingFactor = ((float)PSI_L) / (PSI_T * PSI_T * PSI_h);
    gravity_X_SU = scalingFactor * X_accGrav / magGravAcc;
    gravity_Y_SU = scalingFactor * Y_accGrav / magGravAcc;
    gravity_Z_SU = scalingFactor * Z_accGrav / magGravAcc;

    /// SU values for normal stiffnesses for S2S and S2W
    scalingFactor = (1.f / (1.f * PSI_T * PSI_T * PSI_h));
    K_n_s2s_SU = scalingFactor * (YoungModulus_SPH2SPH / K_stiffness);
    K_n_s2w_SU = scalingFactor * (YoungModulus_SPH2WALL / K_stiffness);

    // TODO Make this legit, from user input
    Gamma_n_s2s_SU = .005;
    stepSize_SU = determine_stepSize_SU();

    // Handy debug output
    printf("SU gravity is %f, %f, %f\n", gravity_X_SU, gravity_Y_SU, gravity_Z_SU);
    printf("SU mass is %f\n", gran_params->MASS_UNIT);
    printf("SU radius is %u\n", sphereRadius_SU);
}
}  // namespace granular
}  // namespace chrono