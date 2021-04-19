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
#include "chrono_gpu/ChGpuDemoUtils.hpp"

using namespace chrono;
using namespace chrono::gpu;

ChGpuSimulationParameters params;

// declare global variables
float target_ke = -1;

float zero_vel_barrier = 1e-3;
float zero_pos_barrier = 2e-2;
float zero_pos_comp_barrier = 1e-3;
float zero_vel_comp_barrier = 2e-2;

float curr_body_vel_1 = -1;
float curr_body_vel_2 = -1;
float curr_body_vel_3 = -1;
float curr_body_vel_4 = -1;
float curr_body_vel_5 = -1;

ChVector<float> curr_body_pos_1;
ChVector<float> curr_body_pos_2;
ChVector<float> curr_body_pos_3;
ChVector<float> curr_body_pos_4;
ChVector<float> curr_body_pos_5;

std::vector<chrono::ChVector<float>> pos_Data;
std::vector<chrono::ChVector<float>> vel_Data;

float settled_pos;
float penetration;

void ShowUsage(std::string name) {
    std::cout << "usage: " + name + " <json_file> <mu_s> <mu_r> <normal stiffness> <psi_L> " << std::endl;
}

int main(int argc, char* argv[]) {
    string json_dir = GetChronoDataPath() + "testing/gpu/utest_GPU_stack/utest_GPU_stack.json";

    const char* c_buff = json_dir.c_str();
    // check whether JSON parameters file is valid
    // Parse JSON parameters to the gpu system
    if (ParseJSON(c_buff, params) == false) {
        ShowUsage(argv[0]);
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
    // filesystem::create_directory(filesystem::path(params.output_dir));

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
        // get velocity of every particle involved
        for (int i = 0; i < 5; i++) {
            pos_Data.push_back(gpu_sys.GetParticlePosition(i));
            vel_Data.push_back(gpu_sys.GetParticleVelocity(i));
        }

        gpu_sys.AdvanceSimulation(frame_step);

        curr_time += frame_step;
        currframe++;
        std::cout << "curr step: " << curr_time << "  tot: 3" << std::endl;
    }

    curr_body_vel_1 = gpu_sys.GetParticleVelocity(0).Length();
    curr_body_vel_2 = gpu_sys.GetParticleVelocity(1).Length();
    curr_body_vel_3 = gpu_sys.GetParticleVelocity(2).Length();
    curr_body_vel_4 = gpu_sys.GetParticleVelocity(3).Length();
    curr_body_vel_5 = gpu_sys.GetParticleVelocity(4).Length();
    curr_body_pos_1 = gpu_sys.GetParticlePosition(0);
    curr_body_pos_2 = gpu_sys.GetParticlePosition(1);
    curr_body_pos_3 = gpu_sys.GetParticlePosition(2);
    curr_body_pos_4 = gpu_sys.GetParticlePosition(3);
    curr_body_pos_5 = gpu_sys.GetParticlePosition(4);

    std::cout << "vel_1:" << curr_body_vel_1 << std::endl;
    std::cout << "vel_2:" << curr_body_vel_2 << std::endl;
    std::cout << "vel_3:" << curr_body_vel_3 << std::endl;
    std::cout << "vel_4:" << curr_body_vel_4 << std::endl;
    std::cout << "vel_5:" << curr_body_vel_5 << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

// test 1 - check end abs velocity of each particle
// the end velocities of all particles should be close to 0, within a barrier
TEST(gpuStack, endVel) {
    float diff_1 = curr_body_vel_1 - 0;
    bool zero_1 = false;
    if (diff_1 < zero_vel_barrier) {
        zero_1 = true;
    }

    float diff_2 = curr_body_vel_2 - 0;
    bool zero_2 = false;
    if (diff_2 < zero_vel_barrier) {
        zero_2 = true;
    }

    float diff_3 = curr_body_vel_3 - 0;
    bool zero_3 = false;
    if (diff_3 < zero_vel_barrier) {
        zero_3 = true;
    }

    float diff_4 = curr_body_vel_4 - 0;
    bool zero_4 = false;
    if (diff_4 < zero_vel_barrier) {
        zero_4 = true;
    }

    float diff_5 = curr_body_vel_5 - 0;
    bool zero_5 = false;
    if (diff_5 < zero_vel_barrier) {
        zero_5 = true;
    }

    EXPECT_EQ(true, zero_1);
    EXPECT_EQ(true, zero_2);
    EXPECT_EQ(true, zero_3);
    EXPECT_EQ(true, zero_4);
    EXPECT_EQ(true, zero_5);
}

// test 2 - check the end position of the bottom 3 particles
// test pass if all end positions are within a tolerance barrier
TEST(gpuStack, endPos) {
    float diff_1_x = abs(curr_body_pos_1.x());
    bool zero_1_x = false;
    if (diff_1_x < zero_pos_barrier) {
        zero_1_x = true;
    }

    float diff_1_y = abs(curr_body_pos_1.y());
    bool zero_1_y = false;
    if (diff_1_y < zero_pos_barrier) {
        zero_1_y = true;
    }

    float diff_1_z = abs(curr_body_pos_1.z() - (settled_pos));
    bool zero_1_z = false;
    if (diff_1_z < zero_pos_barrier) {
        zero_1_z = true;
    }

    float diff_2_x = abs(curr_body_pos_2.x());
    bool zero_2_x = false;
    if (diff_2_x < zero_pos_barrier) {
        zero_2_x = true;
    }

    float diff_2_y = abs(curr_body_pos_2.y());
    bool zero_2_y = false;
    if (diff_2_y < zero_pos_barrier) {
        zero_2_y = true;
    }

    float diff_2_z = abs(curr_body_pos_2.z() - (settled_pos + params.sphere_radius * 2 - penetration));
    bool zero_2_z = false;
    if (diff_2_z < zero_pos_barrier) {
        zero_2_z = true;
    }

    float diff_3_x = abs(curr_body_pos_3.x());
    bool zero_3_x = false;
    if (diff_3_x < zero_pos_barrier) {
        zero_3_x = true;
    }

    float diff_3_y = abs(curr_body_pos_3.y());
    bool zero_3_y = false;
    if (diff_3_y < zero_pos_barrier) {
        zero_3_y = true;
    }

    float diff_3_z = abs(curr_body_pos_3.z() - (settled_pos + params.sphere_radius * 4) - 2 * penetration);
    bool zero_3_z = false;
    if (diff_3_z < zero_pos_barrier) {
        zero_3_z = true;
    }

    EXPECT_EQ(true, zero_1_x);
    EXPECT_EQ(true, zero_1_y);
    EXPECT_EQ(true, zero_1_z);
    EXPECT_EQ(true, zero_2_x);
    EXPECT_EQ(true, zero_2_y);
    EXPECT_EQ(true, zero_2_z);
    EXPECT_EQ(true, zero_3_x);
    EXPECT_EQ(true, zero_3_y);
    EXPECT_EQ(true, zero_3_z);
}

// test 3 - comprehensive position check, compare with ground truth
TEST(granularStack, comprehensivePos) {
    std::vector<chrono::ChVector<float>> ground_truth;
    std::string dir = GetChronoDataPath() + "testing/gpu/utest_GT/stack/stack_groundtruth.csv";

    ground_truth = loadPositionCheckpoint<float>(dir);

    EXPECT_EQ(ground_truth.size(), pos_Data.size());

    for (int i = 0; i < ground_truth.size(); i++) {
        float diff_x = ground_truth[i].x() - pos_Data[i].x();
        float diff_y = ground_truth[i].y() - pos_Data[i].y();
        float diff_z = ground_truth[i].z() - pos_Data[i].z();
        diff_x = abs(diff_x);
        diff_y = abs(diff_y);
        diff_z = abs(diff_z);

        bool comp_res = false;
        // if all x,y,z position components are within zero barrier, set compare result to true
        if ((diff_x < zero_pos_comp_barrier) && (diff_y < zero_pos_comp_barrier) && (diff_z < zero_pos_comp_barrier)) {
            comp_res = true;
        }
        EXPECT_EQ(true, comp_res);
    }
}

// test 4 - comprehensive particle volocities check, compare with ground truth
TEST(gpuStack, comprehensiveVel) {
    std::vector<chrono::ChVector<float>> ground_truth;
    std::string dir = GetChronoDataPath() + "testing/gpu/utest_GT/stack/stack_groundtruth.csv";

    ground_truth = loadVelocityCheckpoint<float>(dir);

    EXPECT_EQ(ground_truth.size(), vel_Data.size());

    for (int i = 0; i < ground_truth.size(); i++) {
        float diff_x = ground_truth[i].x() - vel_Data[i].x();
        float diff_y = ground_truth[i].y() - vel_Data[i].y();
        float diff_z = ground_truth[i].z() - vel_Data[i].z();
        diff_x = abs(diff_x);
        diff_y = abs(diff_y);
        diff_z = abs(diff_z);
        bool comp_res = false;
        // if all x,y,z velocity components are within zero barrier, set compare result to true
        if ((diff_x < zero_vel_comp_barrier) && (diff_y < zero_vel_comp_barrier) && (diff_z < zero_vel_comp_barrier)) {
            comp_res = true;
        }
        EXPECT_EQ(true, comp_res);
    }
}
