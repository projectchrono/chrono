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
// Authors: Luning Fang, Jason Zhou
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

#include "chrono/core/ChGlobal.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/utils/ChGpuJsonParser.h"
#include "chrono_gpu/ChGpuData.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::gpu;


//// TODO: test also the case where the pyramid does not hold!

TEST(gpuFrictionSliding, check) {
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
    float mu_k = 0.5f;   
    gpu_sys.SetStaticFrictionCoeff_SPH2SPH(mu_k);
    gpu_sys.SetStaticFrictionCoeff_SPH2WALL(mu_k);

    // generate the ground for the pyramid
    ChVector<float> ground_plate_pos(0.0, 0.0, 0.0);
    ChVector<float> ground_plate_normal(0.0, 0.0, 1.0f);
    gpu_sys.CreateBCPlane(ground_plate_pos, ground_plate_normal, true);

    gpu_sys.SetVerbosity(CHGPU_VERBOSITY::QUIET);

    gpu_sys.Initialize();

    // Ball parameters
    float mass = (float)((4.0 / 3) * 3.14 * 0.5 * 0.5 * 0.5 * 1.90986);
    float inertia = (float)((2.0 / 5) * mass * 0.5 * 0.5);
    float radius = 0.5f;

    // Test precision
    float precision_KE = 1e-6f;
    float precision_pos = 1e-5f;

    // Initial ball position
    auto pos = gpu_sys.GetParticlePosition(2);
    std::cout << "\npos = " << pos << std::endl;

    float step_size = 1e-4f;
    float time_start_check = 0.1f;
    float time_end = 1.5f;
    bool settled = false;
    float curr_time = 0;
    while (curr_time < time_end) {
        gpu_sys.AdvanceSimulation(step_size);
        curr_time += step_size;

        if (curr_time > time_start_check) {
            float vel = gpu_sys.GetParticleVelocity(2).Length();
            float omg = gpu_sys.GetParticleAngVelocity(2).Length();
            float KE = 0.5 * mass * vel * vel + 0.5 * inertia * omg * omg;
            std::cout << "\r" << std::fixed << std::setprecision(6) << curr_time << "  " << KE << std::flush;
            if (KE < precision_KE) {
                settled = true;
                break;
            }
        } else {
            std::cout << "\r" << std::fixed << std::setprecision(6) << curr_time << std::flush;
        }
    }

    ASSERT_TRUE(settled);

    // Check final ball position
    pos = gpu_sys.GetParticlePosition(2);
    std::cout << "\npos = " << pos << std::endl;
    ASSERT_NEAR(pos.y(), 0.0f, precision_pos);
    ASSERT_TRUE(pos.z() > radius);
}
