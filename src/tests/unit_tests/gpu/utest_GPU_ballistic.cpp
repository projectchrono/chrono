// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Ruochun Zhang, Radu Serban, Jason Zhou
// =============================================================================
// Validation test: high velocity sphere punching through a mesh facet
// =============================================================================

#include "gtest/gtest.h"
#include <cmath>
#include <iostream>

#include "chrono/core/ChGlobal.h"
#include "chrono_gpu/physics/ChSystemGpu.h"

#include "ut_gpu_utils.h"

using namespace chrono;
using namespace chrono::gpu;

TEST(gpuBallistic, check) {
    float radius = 0.5f;
    float density = 7800.0f;
    ChSystemGpuMesh gpu_sys(radius, density, ChVector3f(20.0f, 20.0f, 10.0f));

    // Load in the mesh
    gpu_sys.AddMesh(GetChronoDataPath() + "testing/gpu/one_facet.obj", ChVector3f(0), ChMatrix33<float>(ChVector3f(1)),
                    100.0f);
    gpu_sys.EnableMeshCollision(true);

    // Initialize sphere, with high initial velocity
    float z0 = 4.0f;
    float v0 = 1e4f;
    std::vector<ChVector3f> body_point = {ChVector3f(1.0f, -1.0f, z0)};
    std::vector<ChVector3f> velocity = {ChVector3f(0.0f, 0.0f, -v0)};
    gpu_sys.SetParticles(body_point, velocity);

    gpu_sys.SetPsiFactors(32, 16);

    float g = 980.0f;
    gpu_sys.SetGravitationalAcceleration(ChVector3d(0, 0, -g));

    // Set normal force model
    gpu_sys.SetKn_SPH2SPH(1e11);
    gpu_sys.SetKn_SPH2WALL(1e11);
    gpu_sys.SetKn_SPH2MESH(1e11);
    gpu_sys.SetGn_SPH2SPH(1e4);
    gpu_sys.SetGn_SPH2WALL(1e4);
    gpu_sys.SetGn_SPH2MESH(1e4);

    // Set time integrator
    float step_size = 1e-6f;
    gpu_sys.SetFixedStepSize(step_size);
    gpu_sys.SetTimeIntegrator(CHGPU_TIME_INTEGRATOR::CENTERED_DIFFERENCE);

    gpu_sys.SetBDFixed(true);
    gpu_sys.SetVerbosity(CHGPU_VERBOSITY::QUIET);

    gpu_sys.Initialize();

    // Tolerance for event times
    float tolerance = 1e-4f;
    float hit_time = (std::sqrt(v0 * v0 + 2 * g * (z0 - radius)) - v0) / g;
    std::cout << "Theoretic hit time: " << hit_time << std::endl;

    bool hit = false;
    bool penetrated = false;
    float curr_time = 0;
    while (curr_time < 1) {
        gpu_sys.AdvanceSimulation(step_size);
        curr_time += step_size;

        auto z_pos = gpu_sys.GetParticlePosition(0).z();

        // Check time of hit
        if (!hit && z_pos < radius) {
            std::cout << "Hit at t = " << curr_time << std::endl;
            ASSERT_NEAR(curr_time, hit_time, tolerance);
            hit = true;
        }

        // Check that ball does not bounce
        ASSERT_FALSE(hit && z_pos > radius);

        // Check full penetration
        if (z_pos < -radius) {
            std::cout << "Mesh penetrated at t = " << curr_time << std::endl;
            penetrated = true;
            break;
        }
    }

    ASSERT_TRUE(hit);
    ASSERT_TRUE(penetrated);
}
