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
// Simple test of the functionality of Chrono::Dem tools for measuring
// the memory footprint of a system.
// =============================================================================

#include <fstream>
#include <iostream>
#include <string>

#include "chrono/core/ChTimer.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_dem/physics/ChSystemDem.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::dem;

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

CHDEM_OUTPUT_MODE write_mode = CHDEM_OUTPUT_MODE::NONE;
CHDEM_VERBOSITY verbose = CHDEM_VERBOSITY::INFO;
float cohesion_ratio = 0;

bool run_test(float box_size_X, float box_size_Y, float box_size_Z) {
    // Setup simulation
    ChSystemDem dem_sys(sphereRadius, sphereDensity, make_float3(box_size_X, box_size_Y, box_size_Z));

    dem_sys.SetKn_SPH2SPH(normStiffness_S2S);
    dem_sys.SetKn_SPH2WALL(normStiffness_S2W);
    dem_sys.SetGn_SPH2SPH(normalDampS2S);
    dem_sys.SetGn_SPH2WALL(normalDampS2W);

    dem_sys.SetCohesionRatio(cohesion_ratio);
    dem_sys.SetAdhesionRatio_SPH2WALL(adhesion_ratio_s2w);

    dem_sys.SetGravitationalAcceleration(ChVector3f(0.f, 0.f, grav_acceleration));
    dem_sys.SetParticleOutputMode(write_mode);

    // Fill the bottom half with material
    chrono::utils::HCPSampler<float> sampler(2.1f * sphereRadius);  // Add epsilon
    ChVector3f center(0.f, 0.f, -0.25f * box_size_Z);
    ChVector3f hdims(box_size_X / 2.f - sphereRadius, box_size_X / 2.f - sphereRadius, box_size_Z / 4.f - sphereRadius);
    std::vector<ChVector3f> body_points = sampler.SampleBox(center, hdims);

    dem_sys.SetParticles(body_points);

    dem_sys.SetBDFixed(true);
    dem_sys.SetFrictionMode(CHDEM_FRICTION_MODE::FRICTIONLESS);
    dem_sys.SetTimeIntegrator(CHDEM_TIME_INTEGRATOR::CENTERED_DIFFERENCE);
    dem_sys.SetVerbosity(verbose);

    // upward facing plane just above the bottom to capture forces
    ChVector3f plane_normal(0, 0, 1);
    ChVector3f plane_center(0, 0, -box_size_Z / 2 + 2 * sphereRadius);

    size_t plane_bc_id = dem_sys.CreateBCPlane(plane_center, plane_normal, true);

    dem_sys.SetFixedStepSize(timestep);

    dem_sys.Initialize();

    int fps = 25;
    float frame_step = 1.0f / fps;
    float curr_time = 0;

    // Run settling experiments
    ChTimer<double> timer;
    timer.start();
    while (curr_time < timeEnd) {
        dem_sys.AdvanceSimulation(frame_step);
        curr_time += frame_step;
        printf("Time: %f\n", curr_time);
    }
    timer.stop();
    std::cout << "Simulated " << dem_sys.GetNumParticles() << " particles in " << timer.GetTimeSeconds() << " seconds"
              << std::endl;

    constexpr float F_CGS_TO_SI = 1e-5f;

    ChVector3f reaction_force;
    if (!dem_sys.GetBCReactionForces(plane_bc_id, reaction_force)) {
        printf("ERROR! Get contact forces for plane failed\n");
        return false;
    }

    printf("plane force is (%f, %f, %f) Newtons\n",  //
           F_CGS_TO_SI * reaction_force.x(), F_CGS_TO_SI * reaction_force.y(), F_CGS_TO_SI * reaction_force.z());

    float computed_bottom_force = reaction_force.z();
    float expected_bottom_force = (float)body_points.size() * (4.f / 3.f) * (float)CH_PI * sphereRadius * sphereRadius *
                                  sphereRadius * sphereDensity * grav_acceleration;

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
    const int dem_dev_id_active = 0;
    cudaDeviceProp dev_props;
    demErrchk(cudaSetDevice(dem_dev_id_active));
    demErrchk(cudaGetDeviceProperties(&dev_props, dem_dev_id_active));

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
