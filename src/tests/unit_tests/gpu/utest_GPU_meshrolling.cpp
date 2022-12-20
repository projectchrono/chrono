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
// Authors: Luning Fang, Ruochun Zhang, Jason Zhou
// =============================================================================
// Validation test: one particle rolling on the mesh surface
// This test will check the rolling friction model implemented in chrono::gpu
// When interaction is between Particle vs Mesh Wall
// =============================================================================

#include <cmath>
#include <iostream>
#include <string>
#include "gtest/gtest.h"
#include "unit_testing.h"

#include "chrono/core/ChGlobal.h"
#include "chrono_gpu/physics/ChSystemGpu.h"

using namespace chrono;
using namespace chrono::gpu;

TEST(gpuMeshRolling, check) {
    float radius = 0.5f;
    float density = 6.f;
    float g = 980.f;

    float precision_KE = 1e-3f;
    float precision_pos = 1e-3f;

    float mu_s = 0.2f;
    float mu_r = 0.0008f;

    ChSystemGpuMesh gpu_sys(radius, density, ChVector<float>(20.f, 20.f, 10.f));

    // Load in the mesh
    float mass = 100.f;
    float inertia = (2.0f / 5) * mass * 0.5f * 0.5f;
    gpu_sys.AddMesh(GetChronoDataPath() + "testing/gpu/one_facet.obj", ChVector<float>(0),
                    ChMatrix33<float>(ChVector<float>(1)), mass);
    gpu_sys.EnableMeshCollision(true);

    // assign initial condition for the sphere
    float penetration = pow(mass * abs(-g) / (1e11f), 2.f / 3.f);
    float settled_pos = radius - penetration;

    std::vector<ChVector<float>> body_point = {ChVector<float>(1.f, -1.f, 0.5f)};
    std::vector<ChVector<float>> velocity = {ChVector<float>(1.f, 0.f, 0.f)};
    gpu_sys.SetParticles(body_point, velocity);

    gpu_sys.SetPsiFactors(32, 16);

    // set normal force model
    gpu_sys.SetKn_SPH2SPH(1e11);
    gpu_sys.SetKn_SPH2WALL(1e11);
    gpu_sys.SetKn_SPH2MESH(1e11);
    gpu_sys.SetGn_SPH2SPH(1e4);
    gpu_sys.SetGn_SPH2WALL(1e4);
    gpu_sys.SetGn_SPH2MESH(1e4);

    // set tangential force model
    gpu_sys.SetKt_SPH2SPH(1e7);
    gpu_sys.SetKt_SPH2WALL(1e7);
    gpu_sys.SetKt_SPH2MESH(1e7);
    gpu_sys.SetGt_SPH2SPH(1e4);
    gpu_sys.SetGt_SPH2WALL(1e4);
    gpu_sys.SetGt_SPH2MESH(1e4);
    gpu_sys.SetStaticFrictionCoeff_SPH2SPH(mu_s);
    gpu_sys.SetStaticFrictionCoeff_SPH2WALL(mu_s);
    gpu_sys.SetStaticFrictionCoeff_SPH2MESH(mu_s);
    gpu_sys.SetFrictionMode(CHGPU_FRICTION_MODE::MULTI_STEP);

    // set rolling parameters
    gpu_sys.SetRollingMode(CHGPU_ROLLING_MODE::SCHWARTZ);
    gpu_sys.SetRollingCoeff_SPH2SPH(mu_r);
    gpu_sys.SetRollingCoeff_SPH2WALL(mu_r);
    gpu_sys.SetRollingCoeff_SPH2MESH(mu_r);

    // set gravity
    gpu_sys.SetGravitationalAcceleration(ChVector<>(0.f, 0.f, -g));

    float step_size = 1e-4f;
    float curr_time = 0;
    float end_time = 3.0f;
    float time_start_check = 0.1f;
    bool settled = false;

    // set time integrator
    gpu_sys.SetFixedStepSize(step_size);
    gpu_sys.SetTimeIntegrator(CHGPU_TIME_INTEGRATOR::CENTERED_DIFFERENCE);

    gpu_sys.SetBDFixed(true);
    gpu_sys.Initialize();

    while (curr_time < end_time) {
        gpu_sys.AdvanceSimulation(step_size);
        curr_time += step_size;

        std::cout << "\r" << std::fixed << std::setprecision(6) << curr_time << std::flush;
        if (curr_time > time_start_check) {
            float vel = gpu_sys.GetParticleVelocity(0).Length();
            float omg = gpu_sys.GetParticleAngVelocity(0).Length();
            float KE = 0.5f * mass * vel * vel + 0.5f * inertia * omg * omg;
            std::cout << "\r" << std::fixed << std::setprecision(6) << curr_time << "  " << KE << std::flush;

            // stop simulation if the kinetic energy falls below threshold
            if (KE < precision_KE) {
                settled = true;
                break;
            }
        } else {
            std::cout << "\r" << std::fixed << std::setprecision(6) << curr_time << std::flush;
        }
    }

    // check whether the ball settles
    ASSERT_TRUE(settled);

    // check position x, y ,z components
    ChVector<> end_pos = gpu_sys.GetParticlePosition(0);
    ASSERT_TRUE(end_pos.x() > 1.0f);
    ASSERT_NEAR(end_pos.y(), -1.0f, precision_pos);
    ASSERT_NEAR(end_pos.z(), settled_pos, precision_pos);
}
