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

#include "chrono_gpu/physics/ChSystemGpu.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::gpu;

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

bool run_test(float box_size_X, float box_size_Y, float box_size_Z) {
    // Setup simulation
    ChSystemGpu gpu_sys(sphereRadius, sphereDensity, make_float3(box_size_X, box_size_Y, box_size_Z));

    gpu_sys.SetKn_SPH2SPH(normStiffness_S2S);
    gpu_sys.SetKn_SPH2WALL(normStiffness_S2W);
    gpu_sys.SetGn_SPH2SPH(normalDampS2S);
    gpu_sys.SetGn_SPH2WALL(normalDampS2W);

    gpu_sys.SetCohesionRatio(cohesion_ratio);
    gpu_sys.SetAdhesionRatio_SPH2WALL(adhesion_ratio_s2w);

    gpu_sys.SetGravitationalAcceleration(ChVector<float>(0.f, 0.f, grav_acceleration));
    gpu_sys.SetParticleOutputMode(write_mode);

    // Fill the bottom half with material
    chrono::utils::HCPSampler<float> sampler(2.1f * sphereRadius);  // Add epsilon
    ChVector<float> center(0.f, 0.f, -0.25f * box_size_Z);
    ChVector<float> hdims(box_size_X / 2.f - sphereRadius, box_size_X / 2.f - sphereRadius,
                          box_size_Z / 4.f - sphereRadius);
    std::vector<ChVector<float>> body_points = sampler.SampleBox(center, hdims);

    gpu_sys.SetParticles(body_points);

    gpu_sys.SetBDFixed(true);
    gpu_sys.SetFrictionMode(CHGPU_FRICTION_MODE::FRICTIONLESS);
    gpu_sys.SetTimeIntegrator(CHGPU_TIME_INTEGRATOR::CENTERED_DIFFERENCE);
    gpu_sys.SetVerbosity(verbose);

    // upward facing plane just above the bottom to capture forces
    ChVector<float> plane_normal(0, 0, 1);
    ChVector<float> plane_center(0, 0, -box_size_Z / 2 + 2 * sphereRadius);

    size_t plane_bc_id = gpu_sys.CreateBCPlane(plane_center, plane_normal, true);

    gpu_sys.SetFixedStepSize(timestep);

    gpu_sys.Initialize();

    int fps = 25;
    float frame_step = 1.0f / fps;
    float curr_time = 0;

    // Run settling experiments
    ChTimer<double> timer;
    timer.start();
    while (curr_time < timeEnd) {
        gpu_sys.AdvanceSimulation(frame_step);
        curr_time += frame_step;
        printf("Time: %f\n", curr_time);
    }
    timer.stop();
    std::cout << "Simulated " << gpu_sys.GetNumParticles() << " particles in " << timer.GetTimeSeconds() << " seconds"
              << std::endl;

    constexpr float F_CGS_TO_SI = 1e-5f;

    ChVector<float> reaction_force;
    if (!gpu_sys.GetBCReactionForces(plane_bc_id, reaction_force)) {
        printf("ERROR! Get contact forces for plane failed\n");
        return false;
    }

    printf("plane force is (%f, %f, %f) Newtons\n",  //
           F_CGS_TO_SI * reaction_force.x(), F_CGS_TO_SI * reaction_force.y(), F_CGS_TO_SI * reaction_force.z());

    float computed_bottom_force = reaction_force.z();
    float expected_bottom_force = (float)body_points.size() * (4.f / 3.f) * (float)CH_C_PI * sphereRadius *
                                  sphereRadius * sphereRadius * sphereDensity * grav_acceleration;

    // 1% error allowed, max
    float percent_error = 0.01f;
    printf("Expected bottom force is %f, computed %f\n", expected_bottom_force, computed_bottom_force);
    if (std::abs((expected_bottom_force - computed_bottom_force) / expected_bottom_force) > percent_error) {
        printf("DIFFERENCE IS TOO LARGE!\n");
        return false;
    }

    return true;
}

int main(int argc, char* argv[]) {
    const int gpu_dev_id_active = 0;
    cudaDeviceProp dev_props;
    gpuErrchk(cudaSetDevice(gpu_dev_id_active));
    gpuErrchk(cudaGetDeviceProperties(&dev_props, gpu_dev_id_active));

    printf("\nDEVICE PROPERTIES:\n");
    printf("\tDevice name: %s\n", dev_props.name);
    printf("\tCompute Capability: %d.%d\n", dev_props.major, dev_props.minor);
    printf("\tcanMapHostMemory: %d\n", dev_props.canMapHostMemory);
    printf("\tunifiedAddressing: %d\n", dev_props.unifiedAddressing);
    printf("\tmanagedMemory: %d\n", dev_props.managedMemory);
    printf("\tpageableMemoryAccess: %d\n", dev_props.pageableMemoryAccess);
    printf("\tconcurrentManagedAccess: %d\n", dev_props.concurrentManagedAccess);
    printf("\tcanUseHostPointerForRegisteredMem: %d\n", dev_props.canUseHostPointerForRegisteredMem);
    printf("\tdirectManagedMemAccessFromHost: %d\n", dev_props.directManagedMemAccessFromHost);

    if (!run_test(70, 70, 70))
        return 1;

    return 0;
}
