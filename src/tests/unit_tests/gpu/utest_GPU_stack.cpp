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
// Simple stacking unit test consisting of 5 particles
// utilizes gtest infrustructure
// test 1 : check abs velocities of each particle after 3s
// test 2 : check pos of each particle after 3s
// test 3 : comprehensive check - compare the simulation pos results with ground truth
// test 4 : comprehensive check - compare the simulation vel results with ground truth
// =============================================================================

#include "gtest/gtest.h"
#include <cmath>
#include <iostream>
#include <string>

#include "chrono/core/ChGlobal.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/utils/ChGpuJsonParser.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "unit_testing.h"

using namespace chrono;
using namespace chrono::gpu;

ChGpuSimulationParameters params;

// declare global variables
float radius;
float precision_vel = 1e-3;
float precision_pos = 1e-2;

std::vector<float> end_vel;
std::vector<ChVector<float>> end_pos;

float settled_pos;
float penetration;


int main(int argc, char* argv[]) {
    string json_dir = GetChronoDataPath() + "testing/gpu/utest_GPU_stack/utest_GPU_stack.json";

    const char* c_buff = json_dir.c_str();
    // check whether JSON parameters file is valid
    // Parse JSON parameters to the gpu system
    if (ParseJSON(c_buff, params) == false) {
        return 1;
    }

    params.box_X = 100;
    params.box_Y = 100;
    params.box_Z = 100;

    float mu_s = 0.5;
    float mu_r = 0.5;
    float normalStiffness = 10000000;
    int psi_L = 16;

    params.static_friction_coeffS2S = mu_s;
    params.static_friction_coeffS2W = mu_s;
    params.normalStiffS2S = normalStiffness;
    params.normalStiffS2W = normalStiffness;
    params.rolling_friction_coeffS2S = mu_r;
    params.rolling_friction_coeffS2W = mu_r;

    params.psi_L = psi_L;

    // Read particle radius
    radius = params.sphere_radius;
    // Setup simulation
    ChSystemGpu gpu_sys(params.sphere_radius, params.sphere_density,
                        make_float3(params.box_X, params.box_Y, params.box_Z));

    gpu_sys.SetPsiFactors(params.psi_T, params.psi_L);

    // set normal force model
    gpu_sys.SetKn_SPH2SPH(params.normalStiffS2S);
    gpu_sys.SetKn_SPH2WALL(params.normalStiffS2W);
    gpu_sys.SetGn_SPH2SPH(params.normalDampS2S);
    gpu_sys.SetGn_SPH2WALL(params.normalDampS2W);

    float mass = 4.0 / 3.0 * CH_C_PI * pow(params.sphere_radius, 3) * params.sphere_density;
    std::cout << "mass:  " << mass << std::endl;
    penetration = pow(mass * abs(params.grav_Z) / params.normalStiffS2S, 2.0 / 3.0);

    float inertia = 2.0 / 5.0 * mass * pow(params.sphere_radius, 2);
    settled_pos = -params.box_Z / 2.0 + params.sphere_radius - penetration;

    printf("settled position is:%e\n", settled_pos);

    // set tangential force model
    gpu_sys.SetKt_SPH2SPH(params.tangentStiffS2S);
    gpu_sys.SetKt_SPH2WALL(params.tangentStiffS2W);
    gpu_sys.SetGt_SPH2SPH(params.tangentDampS2S);
    gpu_sys.SetGt_SPH2WALL(params.tangentDampS2W);
    gpu_sys.SetStaticFrictionCoeff_SPH2SPH(params.static_friction_coeffS2S);
    gpu_sys.SetStaticFrictionCoeff_SPH2WALL(params.static_friction_coeffS2W);

    gpu_sys.SetCohesionRatio(params.cohesion_ratio);
    gpu_sys.SetAdhesionRatio_SPH2WALL(params.adhesion_ratio_s2w);
    gpu_sys.SetGravitationalAcceleration(ChVector<>(params.grav_X, params.grav_Y, params.grav_Z));
    gpu_sys.SetOutputMode(params.write_mode);
    gpu_sys.SetOutputFlags(
        CHGPU_OUTPUT_FLAGS::VEL_COMPONENTS | CHGPU_OUTPUT_FLAGS::FIXITY |
        CHGPU_OUTPUT_FLAGS::FORCE_COMPONENTS); 

    gpu_sys.SetFrictionMode(CHGPU_FRICTION_MODE::MULTI_STEP);
    gpu_sys.SetTimeIntegrator(CHGPU_TIME_INTEGRATOR::CHUNG);

    // set rolling friction model
    gpu_sys.SetRollingMode(CHGPU_ROLLING_MODE::SCHWARTZ);
    gpu_sys.SetRollingCoeff_SPH2SPH(params.rolling_friction_coeffS2S);
    gpu_sys.SetRollingCoeff_SPH2WALL(params.rolling_friction_coeffS2W);

    std::vector<ChVector<float>> body_points;
    std::vector<ChVector<float>> velocity;
    for (int i = 0; i < 5; i++) {
        body_points.push_back(ChVector<float>(0, 0, settled_pos + params.sphere_radius * 3 * i));
        velocity.push_back(ChVector<float>(0.0, 0.0, 0.0));
    }

    gpu_sys.SetParticlePositions(body_points, velocity);

    gpu_sys.SetFixedStepSize(params.step_size);
    gpu_sys.SetRecordingContactInfo(true);

    gpu_sys.SetBDFixed(true);

    gpu_sys.SetVerbosity(params.verbose);
    gpu_sys.Initialize();

    int fps = 200;
    float frame_step = 1.f / fps;
    float curr_time = 0;
    int currframe = 0;

    printf("time, ball_px, ball_py, ball_pz, absv, KE, omega.x, omega.y, omega.z\n");
    while (curr_time < params.time_end && currframe < 1000) {
        gpu_sys.AdvanceSimulation(frame_step);

        std::cout << "current step: " << curr_time << "  tot: 2" << std::endl;
        curr_time += frame_step;
        currframe++;
    }

    for (int i = 0; i < 5; i++) {
        end_vel.push_back(gpu_sys.GetParticleVelocity(i).Length());
        end_pos.push_back(gpu_sys.GetParticlePosition(i));
    }

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

// test 1 - check end abs velocity of each particle
// the end velocities of all particles should be close to 0, within a barrier
TEST(gpuStack, endVel) {
    for (int i = 0; i < 5; i++) {
        ASSERT_NEAR(end_vel[i], 0, precision_vel);
    }
}

// test 2 - check the end position of each particle
// test pass if all end positions are within a tolerance barrier
TEST(gpuStack, endPos) {
    for (int i = 0; i < 5; i++) {
        ASSERT_NEAR(end_pos[i].x(), 0, precision_pos);
        ASSERT_NEAR(end_pos[i].y(), 0, precision_pos);
        ASSERT_NEAR(end_pos[i].z(), settled_pos + radius * 2 * i - penetration * i, precision_pos);
    }
}
