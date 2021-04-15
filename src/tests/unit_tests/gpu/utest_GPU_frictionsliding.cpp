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
// Five particles sliding on the box surface
// utilizes gtest infrustructure
// test 1 : check abs velocities of each particle after 5s
// test 2 : comprehensive pos check compared with ground truth
// test 3 : comprehensive vel check compared with ground truth
// =============================================================================

#include "gtest/gtest.h"
#include <cmath>
#include <iostream>
#include <string>
#include <cstdlib>

#ifdef _WIN32
    #include <windows.h>
    #define NATIVE_PWD "CD"
#else
    #define NATIVE_PWD "PWD"
#endif

#include "chrono/core/ChGlobal.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/utils/ChGpuJsonParser.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_gpu/ChGpuDemoUtils.hpp"

using namespace chrono;
using namespace chrono::gpu;

ChGpuSimulationParameters params;
float target_ke = -1;

float zero_vel_barrier = 5e-3;
float zero_pos_barrier = 5e-3;
float zero_pos_comp_barrier = 5e-3;
float zero_vel_comp_barrier = 5e-3;

float curr_body_vel_1;
float curr_body_vel_2;
float curr_body_vel_3;
float curr_body_vel_4;
float curr_body_vel_5;

ChVector<> curr_body_pos_1;
ChVector<> curr_body_pos_2;
ChVector<> curr_body_pos_3;
ChVector<> curr_body_pos_4;
ChVector<> curr_body_pos_5;

std::vector<chrono::ChVector<float>> pos_Data;
std::vector<chrono::ChVector<float>> vel_Data;

void ShowUsage(std::string name) {
    std::cout << "usage: " + name + " <json_file> <mu_s> <mu_r> <normal stiffness> <psi_L> " << std::endl;
}

int main(int argc, char* argv[]) {
    string json_dir = GetChronoDataFile("gpu/utest_GPU_frictionsliding/utest_GPU_frictionsliding.json");

#ifdef _WIN32
    char filemame_dummy[] = "dummy.txt";
    char fullFilename_dummy[MAX_PATH];
    GetFullPathName(filemame_dummy, MAX_PATH, fullFilename_dummy, nullptr);
    string name_str_dummy = fullFilename_dummy;
    if (name_str_dummy.find("bin") == std::string::npos) {
        json_dir = "../../../../bin/Release/" +
                   GetChronoDataFile("gpu/utest_GPU_frictionsliding/utest_GPU_frictionsliding.json");
    }
#endif

    const char* c_buff = json_dir.c_str();

    // Some of the default values are overwritten by user via command line
    if (ParseJSON(c_buff, params) == false) {
        ShowUsage(argv[0]);
        return 1;
    }

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

    double mass = 4.0 / 3.0 * CH_C_PI * pow(params.sphere_radius, 3) * params.sphere_density;
    std::cout << "mass:  " << mass << std::endl;
    double penetration = pow(mass * abs(params.grav_Z) / params.normalStiffS2S, 2.0 / 3.0);

    double inertia = 2.0 / 5.0 * mass * pow(params.sphere_radius, 2);
    double settled_pos = -params.box_Z / 2.0 + params.sphere_radius - penetration;

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

    body_points.push_back(ChVector<float>(0, 0, -3.2));
    velocity.push_back(ChVector<float>(100.0, 40.0, 0.0));

    body_points.push_back(ChVector<float>(1.5, 0, -3.2));
    velocity.push_back(ChVector<float>(40.0, 80.0, 0.0));

    body_points.push_back(ChVector<float>(3, 0, -3.2));
    velocity.push_back(ChVector<float>(90.0, 60.0, 0.0));

    body_points.push_back(ChVector<float>(-1.5, 0, -3.2));
    velocity.push_back(ChVector<float>(100.0, 30.0, 0.0));

    body_points.push_back(ChVector<float>(-3, 0, -3.2));
    velocity.push_back(ChVector<float>(90.0, 70.0, 0.0));

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
        // test: get vel of every particle involved
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

        for (int i = 0; i < 5; i++) {
            chrono::ChVector<float> curr_ball;
            curr_ball.x() = gpu_sys.GetParticlePosition(i).x();
            curr_ball.y() = gpu_sys.GetParticlePosition(i).y();
            curr_ball.z() = gpu_sys.GetParticlePosition(i).z();
            pos_Data.push_back(curr_ball);

            chrono::ChVector<float> curr_vel;
            curr_vel.x() = gpu_sys.GetParticleVelocity(i).x();
            curr_vel.y() = gpu_sys.GetParticleVelocity(i).y();
            curr_vel.z() = gpu_sys.GetParticleVelocity(i).z();
            vel_Data.push_back(curr_vel);
        }

        gpu_sys.AdvanceSimulation(frame_step);

        curr_time += frame_step;

        currframe++;
        std::cout << "curr: " << curr_time << "  tot: 5" << std::endl;
    }
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

// test 1 - check end abs velocity of each particle
TEST(gpuFrictionSLiding, endVel) {
    float diff_1 = curr_body_vel_1 - 0;
    // std::cout << "diff_1:" << diff_1 << std::endl;
    bool zero_1 = false;
    if (diff_1 < zero_vel_barrier) {
        zero_1 = true;
    }

    float diff_2 = curr_body_vel_2 - 0;
    // std::cout << "diff_2:" << diff_2 << std::endl;
    bool zero_2 = false;
    if (diff_2 < zero_vel_barrier) {
        zero_2 = true;
    }

    float diff_3 = curr_body_vel_3 - 0;
    // std::cout << "diff_3:" << diff_3 << std::endl;
    bool zero_3 = false;
    if (diff_3 < zero_vel_barrier) {
        zero_3 = true;
    }

    float diff_4 = curr_body_vel_4 - 0;
    // std::cout << "diff_4:" << diff_4 << std::endl;
    bool zero_4 = false;
    if (diff_4 < zero_vel_barrier) {
        zero_4 = true;
    }

    float diff_5 = curr_body_vel_5 - 0;
    // std::cout << "diff_5:" << diff_5 << std::endl;
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

// test 2 - comprehensive position check, compare with ground truth
TEST(gpuFrictionSLiding, comprehensivePos) {
    std::vector<chrono::ChVector<float>> ground_truth;
    std::string dir = GetChronoDataFile("gpu/utest_GT/frictionsliding/frictionsliding_groundtruth.csv");
#ifdef _WIN32
    char filemame_dummy[] = "dummy.txt";
    char fullFilename_dummy[MAX_PATH];
    GetFullPathName(filemame_dummy, MAX_PATH, fullFilename_dummy, nullptr);
    string name_str_dummy = fullFilename_dummy;
    if (name_str_dummy.find("bin") == std::string::npos) {
        dir = "../../../../bin/Release/" +
              GetChronoDataFile("gpu/utest_GT/frictionsliding/frictionsliding_groundtruth.csv");
    }
#endif
    ground_truth = loadPositionCheckpoint<float>(dir);

    EXPECT_EQ(ground_truth.size(), pos_Data.size());

    int false_count = 0;

    for (int i = 0; i < ground_truth.size(); i++) {
        float diff_x = ground_truth[i].x() - pos_Data[i].x();
        float diff_y = ground_truth[i].y() - pos_Data[i].y();
        float diff_z = ground_truth[i].z() - pos_Data[i].z();
        diff_x = abs(diff_x);
        diff_y = abs(diff_y);
        diff_z = abs(diff_z);
        bool comp_res = false;
        if ((diff_x < zero_pos_comp_barrier) && (diff_y < zero_pos_comp_barrier) && (diff_z < zero_pos_comp_barrier)) {
            comp_res = true;
        }
        if (comp_res == false) {
            false_count++;
        }
    }

    bool res_ind = false;
    if (false_count < 10) {
        res_ind = true;
    }
    EXPECT_EQ(true, res_ind);
}

// test 3 - comprehensive particle volocities check, compare with ground truth
TEST(gpuFrictionSLiding, comprehensiveVel) {
    std::vector<chrono::ChVector<float>> ground_truth;
    std::string dir = GetChronoDataFile("gpu/utest_GT/frictionsliding/frictionsliding_groundtruth.csv");
#ifdef _WIN32
    char filemame_dummy[] = "dummy.txt";
    char fullFilename_dummy[MAX_PATH];
    GetFullPathName(filemame_dummy, MAX_PATH, fullFilename_dummy, nullptr);
    string name_str_dummy = fullFilename_dummy;
    if (name_str_dummy.find("bin") == std::string::npos) {
        dir = "../../../../bin/Release/" +
              GetChronoDataFile("gpu/utest_GT/frictionsliding/frictionsliding_groundtruth.csv");
    }
#endif
    ground_truth = loadVelocityCheckpoint<float>(dir);

    EXPECT_EQ(ground_truth.size(), vel_Data.size());
    int false_count = 0;

    for (int i = 0; i < ground_truth.size(); i++) {
        float diff_x = ground_truth[i].x() - vel_Data[i].x();
        float diff_y = ground_truth[i].y() - vel_Data[i].y();
        float diff_z = ground_truth[i].z() - vel_Data[i].z();
        diff_x = abs(diff_x);
        diff_y = abs(diff_y);
        diff_z = abs(diff_z);
        bool comp_res = false;
        if ((diff_x < zero_vel_comp_barrier) && (diff_y < zero_vel_comp_barrier) && (diff_z < zero_vel_comp_barrier)) {
            comp_res = true;
        }
        if (comp_res == false) {
            false_count++;
        }
    }

    bool res_ind = false;
    if (false_count < 10) {
        res_ind = true;
    }
    EXPECT_EQ(true, res_ind);
}
