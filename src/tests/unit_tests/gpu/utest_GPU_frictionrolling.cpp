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
// Authors: Jason Zhou
// =============================================================================
// One particle rolling on the boundary surface
// This test will check the rolling friction model implemented in chrono::gpu
// When interaction is between Particle vs Boundary Wall
// =============================================================================

#include "gtest/gtest.h"
#include <cmath>
#include <iostream>
#include <string>

#include "unit_testing.h"

#include "chrono/core/ChGlobal.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/utils/ChGpuJsonParser.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::gpu;

ChGpuSimulationParameters params;

// declare global variables
float precision_vel = 1e-2;
float precision_pos = 1e-2;
float precision_ang = 3e-2;

ChVector<float> end_vel;
ChVector<float> end_pos;
ChVector<float> end_ang_vel;

float settled_pos;
float penetration;
float radius;

int main(int argc, char* argv[]) {
    string json_dir = GetChronoDataPath() + "testing/gpu/utest_GPU_frictionsliding/utest_GPU_frictionsliding.json";
    const char* c_buff = json_dir.c_str();

    // check whether JSON parameters file is valid
    // Parse JSON parameters to the gpu system
    if (ParseJSON(c_buff, params) == false) {
        return 1;
    }

    float mu_s = 0.2;
    float mu_r;
    mu_r = 0.0008;

    float normalStiffness = 10000000;
    int psi_L = 16;

    params.static_friction_coeffS2S = mu_s;
    params.static_friction_coeffS2W = mu_s;
    params.normalStiffS2S = normalStiffness;
    params.normalStiffS2W = normalStiffness;
    params.rolling_friction_coeffS2S = mu_r;
    params.rolling_friction_coeffS2W = mu_r;

    params.psi_L = psi_L;

    // Setup simulation
    ChSystemGpu gpu_sys(params.sphere_radius, params.sphere_density,
                        make_float3(params.box_X, params.box_Y, params.box_Z));

    gpu_sys.SetPsiFactors(params.psi_T, params.psi_L);

    // set normal force model
    gpu_sys.SetKn_SPH2SPH(params.normalStiffS2S);
    gpu_sys.SetKn_SPH2WALL(params.normalStiffS2W);
    gpu_sys.SetGn_SPH2SPH(params.normalDampS2S);
    gpu_sys.SetGn_SPH2WALL(params.normalDampS2W);

    double mass = 4.0 / 3.0 * CH_C_PI * pow(params.sphere_radius, 3) * params.sphere_density;
    std::cout << "mass:  " << mass << std::endl;
    double penetration = pow(mass * abs(params.grav_Z) / params.normalStiffS2S, 2.0 / 3.0);

    double inertia = 2.0 / 5.0 * mass * pow(params.sphere_radius, 2);
    settled_pos = -params.box_Z / 2.0 + params.sphere_radius - penetration;

    printf("settled position is:%e\n", settled_pos);

    // set tangential force model
    gpu_sys.SetKt_SPH2SPH(params.tangentStiffS2S);
    gpu_sys.SetKt_SPH2WALL(params.tangentStiffS2W);
    gpu_sys.SetGt_SPH2SPH(params.tangentDampS2S);
    gpu_sys.SetGt_SPH2WALL(params.tangentDampS2W);
    gpu_sys.SetStaticFrictionCoeff_SPH2SPH(params.static_friction_coeffS2S);
    gpu_sys.SetStaticFrictionCoeff_SPH2WALL(params.static_friction_coeffS2W);
    filesystem::create_directory(filesystem::path(params.output_dir));

    gpu_sys.SetCohesionRatio(params.cohesion_ratio);
    gpu_sys.SetAdhesionRatio_SPH2WALL(params.adhesion_ratio_s2w);
    gpu_sys.SetGravitationalAcceleration(ChVector<>(params.grav_X, params.grav_Y, params.grav_Z));
    gpu_sys.SetOutputMode(params.write_mode);
    gpu_sys.SetOutputFlags(
        CHGPU_OUTPUT_FLAGS::VEL_COMPONENTS | CHGPU_OUTPUT_FLAGS::FIXITY |
        CHGPU_OUTPUT_FLAGS::FORCE_COMPONENTS);  // NOTE: original test used custom FORCE_COMPONENTS output

    gpu_sys.SetFrictionMode(CHGPU_FRICTION_MODE::MULTI_STEP);
    gpu_sys.SetTimeIntegrator(CHGPU_TIME_INTEGRATOR::CHUNG);

    // set rolling friction model
    gpu_sys.SetRollingMode(CHGPU_ROLLING_MODE::SCHWARTZ);

    gpu_sys.SetRollingCoeff_SPH2SPH(params.rolling_friction_coeffS2S);
    gpu_sys.SetRollingCoeff_SPH2WALL(params.rolling_friction_coeffS2W);

    std::vector<ChVector<float>> body_points;
    std::vector<ChVector<float>> velocity;
    std::vector<ChVector<float>> ang_velocity;
    // add only one particle
    body_points.push_back(ChVector<float>(0, 0, settled_pos + 0.01));
    velocity.push_back(ChVector<float>(1.0, 0.0, 0.0));
    ang_velocity.push_back(ChVector<float>(0, 1.0 / params.sphere_radius, 0));

    gpu_sys.SetParticles(body_points, velocity, ang_velocity);
    gpu_sys.SetFixedStepSize(params.step_size);
    gpu_sys.SetRecordingContactInfo(true);

    gpu_sys.SetBDFixed(true);

    gpu_sys.SetVerbosity(params.verbose);
    gpu_sys.Initialize();

    int fps = 200;
    float frame_step = 1.f / fps;
    float curr_time = 0;
    int currframe = 0;
    radius = params.sphere_radius;

    printf("time, ball_px, ball_py, ball_pz, absv, KE, omega.x, omega.y, omega.z\n");
    while (curr_time < params.time_end) {
        // test: get vel of every particle involved
        gpu_sys.AdvanceSimulation(params.step_size);
        curr_time += params.step_size;
        if (gpu_sys.GetParticleVelocity(0).x() < precision_vel) {
            std::cout << "the particle's linear velocity drops below threshold 1e-2 at time = " << curr_time
                      << std::endl;
            break;
        }
    }
    end_vel = gpu_sys.GetParticleVelocity(0);
    end_ang_vel = gpu_sys.GetParticleAngVelocity(0);
    end_pos = gpu_sys.GetParticlePosition(0);

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

// test: check the end velocity and angular velocity
// test that the ball stops rolling
TEST(gpuFrictionSliding, check) {
    // check position y and z component
    ASSERT_NEAR(end_pos.y(), 0, precision_pos);
    ASSERT_NEAR(end_pos.z(), settled_pos, precision_pos);

    // check end velocity
    ASSERT_NEAR(end_vel.x(), 0, precision_vel);
    ASSERT_NEAR(end_vel.y(), 0, precision_vel);
    ASSERT_NEAR(end_vel.z(), 0, precision_vel);

    // check ang velocity
    ASSERT_NEAR(end_ang_vel.x(), 0, precision_ang);
    ASSERT_NEAR(end_ang_vel.y(), 0, precision_ang);
    ASSERT_NEAR(end_ang_vel.z(), 0, precision_ang);
}
