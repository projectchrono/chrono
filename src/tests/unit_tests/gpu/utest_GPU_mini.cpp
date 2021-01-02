// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Conlain Kelly
// =============================================================================
// Simple test of the functionality of Chrono::Gpu tools for measuring
// the memory footprint of a system.
// =============================================================================

#include <fstream>
#include <iostream>
#include <string>

#include "chrono/core/ChTimer.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_gpu/api/ChApiGpuChrono.h"
#include "chrono_gpu/physics/ChSystemGpu.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::gpu;

const char* DELIM = "------------------------------------";

// Default values
float sphereRadius = 1.f;
float sphereDensity = 2.50f;
float timeEnd = 0.5f;
float grav_acceleration = -980.f;
float normStiffness_S2S = 5e7f;
float normStiffness_S2W = 5e7f;

float normalDampS2S = 20000.f;
float normalDampS2W = 20000.f;
float adhesion_ratio_s2w = 0.f;
float timestep = 5e-5f;

CHGPU_OUTPUT_MODE write_mode = CHGPU_OUTPUT_MODE::NONE;
CHGPU_VERBOSITY verbose = CHGPU_VERBOSITY::INFO;
float cohesion_ratio = 0;

double run_test(float box_size_X, float box_size_Y, float box_size_Z) {
    // Setup simulation
    ChSystemGpuSMC gpu_sys(sphereRadius, sphereDensity, make_float3(box_size_X, box_size_Y, box_size_Z));
    gpu_sys.set_K_n_SPH2SPH(normStiffness_S2S);
    gpu_sys.set_K_n_SPH2WALL(normStiffness_S2W);
    gpu_sys.set_Gamma_n_SPH2SPH(normalDampS2S);
    gpu_sys.set_Gamma_n_SPH2WALL(normalDampS2W);

    gpu_sys.set_Cohesion_ratio(cohesion_ratio);
    gpu_sys.set_Adhesion_ratio_S2W(adhesion_ratio_s2w);
    gpu_sys.set_gravitational_acceleration(0.f, 0.f, grav_acceleration);
    gpu_sys.setOutputMode(write_mode);

    // Fill the bottom half with material
    chrono::utils::HCPSampler<float> sampler(2.1f * sphereRadius);  // Add epsilon
    ChVector<float> center(0.f, 0.f, -0.25f * box_size_Z);
    ChVector<float> hdims(box_size_X / 2.f - sphereRadius, box_size_X / 2.f - sphereRadius,
                          box_size_Z / 4.f - sphereRadius);
    std::vector<ChVector<float>> body_points = sampler.SampleBox(center, hdims);

    ChGpuSMC_API apiSMC;
    apiSMC.setSystem(&gpu_sys);
    apiSMC.setElemsPositions(body_points);

    gpu_sys.set_BD_Fixed(true);
    gpu_sys.set_friction_mode(CHGPU_FRICTION_MODE::FRICTIONLESS);
    gpu_sys.set_timeIntegrator(CHGPU_TIME_INTEGRATOR::CENTERED_DIFFERENCE);
    gpu_sys.setVerbose(verbose);

    // upward facing plane just above the bottom to capture forces
    float plane_normal[3] = {0, 0, 1};
    float plane_center[3] = {0, 0, -box_size_Z / 2 + 2 * sphereRadius};

    size_t plane_bc_id = gpu_sys.Create_BC_Plane(plane_center, plane_normal, true);

    gpu_sys.set_fixed_stepSize(timestep);
    ChTimer<double> timer;

    timer.start();
    gpu_sys.initialize();
    int fps = 25;

    float frame_step = 1.0f / fps;
    float curr_time = 0;
    int currframe = 0;

    float reaction_forces[3] = {0, 0, 0};

    // Run settling experiments
    while (curr_time < timeEnd) {
        gpu_sys.advance_simulation(frame_step);
        curr_time += frame_step;
        printf("Time: %f\n", curr_time);
    }

    constexpr float F_CGS_TO_SI = 1e-5f;

    bool success = gpu_sys.getBCReactionForces(plane_bc_id, reaction_forces);
    if (!success) {
        printf("ERROR! Get contact forces for plane failed\n");
    } else {
        printf("plane force is (%f, %f, %f) Newtons\n", F_CGS_TO_SI * reaction_forces[0],
               F_CGS_TO_SI * reaction_forces[1], F_CGS_TO_SI * reaction_forces[2]);

        float computed_bottom_force = reaction_forces[2];
        float expected_bottom_force = (float)body_points.size() * (4.f / 3.f) * (float)CH_C_PI * sphereRadius *
                                      sphereRadius * sphereRadius * sphereDensity * grav_acceleration;

        // 1% error allowed, max
        float percent_error = 0.01f;
        printf("Expected bottom force is %f, computed %f\n", expected_bottom_force, computed_bottom_force);
        if (std::abs((expected_bottom_force - computed_bottom_force) / expected_bottom_force) > percent_error) {
            printf("DIFFERENCE IS TOO LARGE!\n");
            exit(1);
        }
    }
    timer.stop();
    return timer.GetTimeSeconds();
}

int main(int argc, char* argv[]) {
    const int gpu_dev_id_active = 0;
    const int gpu_dev_id_other = 1;
    cudaDeviceProp dev_props;

    gpuErrchk(cudaSetDevice(gpu_dev_id_active));

    gpuErrchk(cudaGetDeviceProperties(&dev_props, gpu_dev_id_active));

    printf("%s\nDEVICE PROPERTIES:\n", DELIM);
    printf("\tDevice name: %s\n", dev_props.name);
    printf("\tCompute Capability: %d.%d\n", dev_props.major, dev_props.minor);
    printf("\tcanMapHostMemory: %d\n", dev_props.canMapHostMemory);
    printf("\tunifiedAddressing: %d\n", dev_props.unifiedAddressing);
    printf("\tmanagedMemory: %d\n", dev_props.managedMemory);
    printf("\tpageableMemoryAccess: %d\n", dev_props.pageableMemoryAccess);
    printf("\tconcurrentManagedAccess: %d\n", dev_props.concurrentManagedAccess);
    printf("\tcanUseHostPointerForRegisteredMem: %d\n", dev_props.canUseHostPointerForRegisteredMem);
    printf("\tdirectManagedMemAccessFromHost: %d\n", dev_props.directManagedMemAccessFromHost);

    // int canAccessPeer;

    // gpuErrchk(cudaDeviceCanAccessPeer(&canAccessPeer, gpu_dev_id_active, gpu_dev_id_other));
    // printf("\tCan access peer (GPU 0 -> GPU 1): %d\n", canAccessPeer);

    // gpuErrchk(cudaDeviceCanAccessPeer(&canAccessPeer, gpu_dev_id_active, cudaCpuDeviceId));
    // printf("\tCan access peer (GPU 0 -> CPU): %d\n", canAccessPeer);
    // up to one million bodies
    // double time50k = run_test(100, 100, 100);
    // double time500k = run_test(220, 220, 220);
    double run_time = run_test(70, 70, 70);

    std::cout << "Running mini granular test!" << std::endl;
    // std::cout << "50 thousand bodies took " << time50k << " seconds!" << std::endl;
    // std::cout << "500 thousand bodies took " << time500k << " seconds!" << std::endl;
    std::cout << "1 million bodies took " << run_time << " seconds!" << std::endl;

    std::ofstream ofile(argv[1], std::ofstream::app);
    return 0;
}
