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
// Authors: Luning Fang, Radu Serban, Jason Zhou
// =============================================================================
// Pyramid test: two spheres settled on the ground with gap in between, a third
// sphere set on top to form a pyramid. With more friction (rolling/sliding) the
// stack can support the top particle, otherwise the top particle drops
// This test is to verify the rolling friction of the chrono::gpu
// =============================================================================
#include "gtest/gtest.h"
#include <cmath>
#include <iostream>
#include <string>

#include "unit_testing.h"

#include "chrono/core/ChGlobal.h"
#include "chrono/core/ChTimer.h"
#include "chrono_gpu/physics/ChSystemGpu.h"

using namespace chrono;
using namespace chrono::gpu;

class gpuPyramid : public ::testing::Test, public ::testing::WithParamInterface<bool> {
  public:
    gpuPyramid() : hold(GetParam()) {}

  protected:
    bool hold;
};

TEST_P(gpuPyramid, check) {
    // load the gpu system using checkpointing
    std::string settled_filename = GetChronoDataPath() + "testing/gpu/pyramid_checkpoint.dat";
    ChSystemGpu gpu_sys(settled_filename);

    // overwrite fixity to allow the top particle to drop
    std::vector<bool> body_fixity;
    body_fixity.push_back(false);
    body_fixity.push_back(false);
    body_fixity.push_back(false);

    gpu_sys.SetParticleFixed(body_fixity);

    // set sliding friction coefficient
    if (hold) {
        // Case 1: with high rolling friction and sliding friction
        // This top ball should be supported by two balls at the bottom
        std::cout << "Holding pyramid" << std::endl;
        float mu_k = 0.5f;
        gpu_sys.SetStaticFrictionCoeff_SPH2SPH(mu_k);
        gpu_sys.SetStaticFrictionCoeff_SPH2WALL(mu_k);
        float mu_r = 0.2f;
        gpu_sys.SetRollingMode(CHGPU_ROLLING_MODE::SCHWARTZ);
        gpu_sys.SetRollingCoeff_SPH2SPH(mu_r);
        gpu_sys.SetRollingCoeff_SPH2WALL(mu_r);
    } else {
        // Case 2: no rolling resistance, very low sliding friction
        // This top ball will eventually fall
        std::cout << "Collapsing pyramid" << std::endl;
        float mu_k = 0.01f;
        gpu_sys.SetStaticFrictionCoeff_SPH2SPH(mu_k);
        gpu_sys.SetStaticFrictionCoeff_SPH2WALL(mu_k);
        gpu_sys.SetRollingMode(CHGPU_ROLLING_MODE::NO_RESISTANCE);
    }

    // generate the ground for the pyramid
    ChVector3f ground_plate_pos(0.0, 0.0, 0.0);
    ChVector3f ground_plate_normal(0.0, 0.0, 1.0f);
    gpu_sys.CreateBCPlane(ground_plate_pos, ground_plate_normal, true);

    gpu_sys.SetVerbosity(CHGPU_VERBOSITY::QUIET);

    gpu_sys.Initialize();

    // Ball parameters
    float mass = (4.0f / 3) * 3.14f * 0.5f * 0.5f * 0.5f * 1.9f;
    float inertia = (2.0f / 5) * mass * 0.5f * 0.5f;
    float radius = 0.5f;

    // Test precision
    float precision_KE = 1e-7f;
    float precision_pos = 1e-3f;
    float precision_time = 1e-3f;

    // Initial ball position
    auto pos = gpu_sys.GetParticlePosition(2);
    std::cout << "initial pos = " << pos << std::endl;

    // Estimated time to impact
    float g = 9.81f;
    float contact_time = std::sqrt(2 * (pos.z() - (1 + std::sqrt(3)) * radius) / g);
    std::cout << "time to contact (analytical) = " << contact_time << std::endl;

    float step_size = 1e-3f;
    float curr_time = 0;

    ChTimer timer;
    timer.start();

    // Free falling phase
    while (curr_time < 1.1 * contact_time) {
        gpu_sys.AdvanceSimulation(step_size);
        curr_time += step_size;

        if (curr_time < 0.9 * contact_time)
            continue;

        pos = gpu_sys.GetParticlePosition(2);
        std::cout << "\r" << std::fixed << std::setprecision(6) << curr_time << std::flush;
        if (std::abs(pos.z() - (1 + std::sqrt(3)) * radius) < precision_pos)
            break;
    }

    ASSERT_NEAR(curr_time, contact_time, precision_time);
    std::cout << "\rcontact at t = " << curr_time << "  (sim. time = " << timer.GetTimeSeconds() << "s)" << std::endl;

    float vel = gpu_sys.GetParticleVelocity(2).Length();
    float omg = gpu_sys.GetParticleAngVelocity(2).Length();
    float KE = 0.5f * mass * vel * vel + 0.5f * inertia * omg * omg;
    std::cout << "kinetic energy at contact time = " << KE << std::endl;

    // Settling phase
    bool settled = false;
    while (curr_time < 1.5f) {
        gpu_sys.AdvanceSimulation(step_size);
        curr_time += step_size;

        if (curr_time < 2 * contact_time)
            continue;

        vel = gpu_sys.GetParticleVelocity(2).Length();
        omg = gpu_sys.GetParticleAngVelocity(2).Length();
        KE = 0.5f * mass * vel * vel + 0.5f * inertia * omg * omg;
        std::cout << "\r" << std::fixed << std::setprecision(6) << curr_time << "  " << KE << std::flush;
        if (KE < precision_KE) {
            settled = true;
            break;
        }
    }

    ASSERT_TRUE(settled);
    std::cout << "\rsettled at t = " << curr_time << "  (sim. time = " << timer.GetTimeSeconds() << "s)" << std::endl;

    // Check final ball position
    pos = gpu_sys.GetParticlePosition(2);
    std::cout << "final pos = " << pos << std::endl;

    if (hold) {
        ASSERT_NEAR(pos.y(), 0.0f, precision_pos);
        ASSERT_TRUE(pos.z() > 2 * radius);
    } else {
        ASSERT_NEAR(pos.y(), 0.0f, precision_pos);
        ASSERT_NEAR(pos.z(), radius, precision_pos);
    }
}

INSTANTIATE_TEST_SUITE_P(check, gpuPyramid, ::testing::Bool());
