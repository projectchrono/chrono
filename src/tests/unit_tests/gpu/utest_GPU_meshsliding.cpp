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
// validation test: sphere rolling on a plane modeled as a planeBC and/or a mesh facet
// Simple stacking unit test consisting of 5 particles
// utilizes gtest infrustructure
// test 1 : comprehensive check - compare the simulation pos results with ground truth
// test 2 : comprehensive check - compare the abs velocity results with ground truth

// =============================================================================

#include <cmath>
#include <iostream>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <limits.h>

#include "chrono/core/ChGlobal.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_gpu/ChGpuData.h"
#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/utils/ChGpuJsonParser.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_gpu/ChGpuDemoUtils.hpp"

#include "gtest/gtest.h"
#include <cstdlib>
// Include definition of chdir(...)
#ifdef _WIN32
    #include <windows.h>
    #define NATIVE_PWD "CD"
#else

    #define NATIVE_PWD "PWD"
#endif

using namespace chrono;
using namespace chrono::gpu;

std::vector<chrono::ChVector<float>> pos_Data;
std::vector<float> absv_Data;
std::vector<chrono::ChVector<float>> ang_vel_Data;

void ShowUsage(std::string name) {
    std::cout << "usage: " + name + " <json_file> \n " << std::endl;
}

int main(int argc, char* argv[]) {
    string json_dir = GetChronoDataFile("gpu/utest_GPU_meshsliding/utest_GPU_meshsliding.json");
#ifdef _WIN32
    char filemame_dummy[] = "dummy.txt";
    char fullFilename_dummy[MAX_PATH];
    GetFullPathName(filemame_dummy, MAX_PATH, fullFilename_dummy, nullptr);
    string name_str_dummy = fullFilename_dummy;
    if (name_str_dummy.find("bin") == std::string::npos) {
        json_dir =
            "../../../../bin/Release/" + GetChronoDataFile("gpu/utest_GPU_meshsliding/utest_GPU_meshsliding.json");
    }
#else
    char pwd[PATH_MAX];
    string name_str_dummy = pwd;
    if (name_str_dummy.find("bin") == std::string::npos) {
        json_dir = "../../../" + GetChronoDataFile("gpu/utest_GPU_meshsliding/utest_GPU_meshsliding.json");
    }
#endif
    const char* c_buff = json_dir.c_str();

    ChGpuSimulationParameters params;

    // Some of the default values are overwritten by user via command line
    if (ParseJSON(c_buff, params) == false) {
        ShowUsage(argv[0]);
        return 1;
    }

    // big domain dimension
    params.box_X = 20.f;
    params.box_Y = 20.f;
    params.box_Z = 10.f;

    ChSystemGpuMesh gpu_sys(params.sphere_radius, params.sphere_density,
                            make_float3(params.box_X, params.box_Y, params.box_Z));

    // load in the mesh
    std::vector<string> mesh_filenames;
    std::string mesh_filename;

    mesh_filename = GetChronoDataFile("gpu/utest_GPU_meshsliding/one_facet.obj");
#ifdef _WIN32
    if (name_str_dummy.find("bin") == std::string::npos) {
        mesh_filename = "../../../../bin/Release/" + GetChronoDataFile("gpu/utest_GPU_meshsliding/one_facet.obj");
    }
#else
    if (name_str_dummy.find("bin") == std::string::npos) {
        mesh_filename = "../../../" + GetChronoDataFile("gpu/utest_GPU_meshsliding/one_facet.obj");
    }
#endif
    mesh_filenames.push_back(mesh_filename);

    std::vector<ChMatrix33<float>> mesh_rotscales;
    std::vector<ChVector<float>> mesh_translations;
    mesh_rotscales.push_back(ChMatrix33<float>(ChVector<float>(1.f, 1.f, 1.f)));
    mesh_translations.push_back(ChVector<>(0, 0, 0));

    std::vector<float> mesh_masses;
    float mass = 100;
    mesh_masses.push_back(mass);

    gpu_sys.AddMeshes(mesh_filenames, mesh_translations, mesh_rotscales, mesh_masses);
    gpu_sys.EnableMeshCollision(true);

    /*
    // create plane BC at the bottom of BD
    float plane_pos[3] = {0.0f, 0.0f, 0.0f};
    float plane_normal[3] = {0.0f, 0.0f, 1.0f};
    size_t plane_bc_id = gpu_sys.Create_BC_Plane(plane_pos, plane_normal, true);
    */

    // assign initial condition for the sphere
    float initialVelo = 1.f;
    std::vector<ChVector<float>> body_point;
    body_point.push_back(ChVector<float>(1.0f, -1.0f, params.sphere_radius));
    std::vector<ChVector<float>> velocity;
    velocity.push_back(ChVector<float>(initialVelo, 0.0f, 0.0f));
    gpu_sys.SetParticlePositions(body_point, velocity);

    gpu_sys.SetPsiFactors(params.psi_T, params.psi_L);

    // set normal force model
    gpu_sys.SetKn_SPH2SPH(params.normalStiffS2S);
    gpu_sys.SetKn_SPH2WALL(params.normalStiffS2W);
    gpu_sys.SetKn_SPH2MESH(params.normalStiffS2M);
    gpu_sys.SetGn_SPH2SPH(params.normalDampS2S);
    gpu_sys.SetGn_SPH2WALL(params.normalDampS2W);
    gpu_sys.SetGn_SPH2MESH(params.normalDampS2M);

    // set tangential force model
    gpu_sys.SetKt_SPH2SPH(params.tangentStiffS2S);
    gpu_sys.SetKt_SPH2WALL(params.tangentStiffS2W);
    gpu_sys.SetKt_SPH2MESH(params.tangentStiffS2M);
    gpu_sys.SetGt_SPH2SPH(params.tangentDampS2S);
    gpu_sys.SetGt_SPH2WALL(params.tangentDampS2W);
    gpu_sys.SetGt_SPH2MESH(params.tangentDampS2M);
    gpu_sys.SetStaticFrictionCoeff_SPH2SPH(params.static_friction_coeffS2S);
    gpu_sys.SetStaticFrictionCoeff_SPH2WALL(params.static_friction_coeffS2W);
    gpu_sys.SetStaticFrictionCoeff_SPH2MESH(params.static_friction_coeffS2M);
    gpu_sys.SetFrictionMode(CHGPU_FRICTION_MODE::MULTI_STEP);

    // set rolling params
    gpu_sys.SetRollingMode(params.rolling_mode);
    gpu_sys.SetRollingCoeff_SPH2SPH(params.rolling_friction_coeffS2S);
    gpu_sys.SetRollingCoeff_SPH2WALL(params.rolling_friction_coeffS2W);
    gpu_sys.SetRollingCoeff_SPH2MESH(params.rolling_friction_coeffS2M);

    // set cohesion and adhesion model
    gpu_sys.SetCohesionRatio(params.cohesion_ratio);
    gpu_sys.SetAdhesionRatio_SPH2WALL(params.adhesion_ratio_s2w);
    gpu_sys.SetAdhesionRatio_SPH2MESH(params.adhesion_ratio_s2m);

    // set gravity
    gpu_sys.SetGravitationalAcceleration(ChVector<>(params.grav_X, params.grav_Y, params.grav_Z));

    // set time integrator
    gpu_sys.SetFixedStepSize(params.step_size);
    gpu_sys.SetTimeIntegrator(CHGPU_TIME_INTEGRATOR::CENTERED_DIFFERENCE);

    gpu_sys.SetOutputMode(params.write_mode);
    filesystem::create_directory(filesystem::path(params.output_dir));

    gpu_sys.SetBDFixed(true);

    gpu_sys.SetVerbosity(params.verbose);
    gpu_sys.Initialize();

    float curr_time = 0;
    ChVector<float> pos;
    ChVector<float> velo;
    ChVector<float> omega;

    unsigned int currframe = 0;
    unsigned int curstep = 0;
    double out_fps = 100;
    float frame_step = 1.f / out_fps;  // Duration of a frame
    unsigned int out_steps = frame_step / params.step_size;
    unsigned int total_frames = (unsigned int)((float)params.time_end * out_fps);
    std::cout << "out_steps " << out_steps << std::endl;

    while (curr_time < params.time_end) {
        gpu_sys.AdvanceSimulation(params.step_size);
        curr_time += params.step_size;

        pos = gpu_sys.GetParticlePosition(0);
        velo = gpu_sys.GetParticleVelocity(0);
        // if frictionless, you can't call getAngVelo
        omega = gpu_sys.GetParticleAngVelocity(0);

        if (curstep % out_steps == 0) {
            char filename[100];
            sprintf(filename, "%s/step%06u", params.output_dir.c_str(), currframe++);
            // gpu_sys.writeFile(std::string(filename));
            // gpu_sys.write_meshes(std::string(filename));

            pos_Data.push_back(gpu_sys.GetParticlePosition(0));
            absv_Data.push_back(gpu_sys.GetParticleVelocity(0).Length());
            ang_vel_Data.push_back(gpu_sys.GetParticleAngVelocity(0));
        }
        std::cout << "time: " << curr_time << " tot: 1" << std::endl;
        curstep++;
    }
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(gpuMeshSliding, comprehensivePos) {
    std::vector<chrono::ChVector<float>> ground_truth;
    std::string dir = GetChronoDataFile("gpu/utest_GT/meshsliding/meshsliding_groundtruth.csv");
#ifdef _WIN32
    char filemame_dummy[] = "dummy.txt";
    char fullFilename_dummy[MAX_PATH];
    GetFullPathName(filemame_dummy, MAX_PATH, fullFilename_dummy, nullptr);
    string name_str_dummy = fullFilename_dummy;
    if (name_str_dummy.find("bin") == std::string::npos) {
        dir = "../../../../bin/Release/" + GetChronoDataFile("gpu/utest_GT/meshsliding/meshsliding_groundtruth.csv");
    }
#else
    char pwd[PATH_MAX];
    string name_str_dummy = pwd;
    if (name_str_dummy.find("bin") == std::string::npos) {
        dir = "../../../" + GetChronoDataFile("gpu/utest_GT/meshsliding/meshsliding_groundtruth.csv");
    }
#endif
    ground_truth = loadPositionCheckpoint<float>(dir);

    EXPECT_EQ(ground_truth.size(), pos_Data.size());

    int false_count = 0;

    for (int i = 0; i < ground_truth.size(); i++) {
        float diff_x = ground_truth[i].x() - pos_Data[i].x();
        float diff_y = ground_truth[i].y() - pos_Data[i].y();
        float diff_z = ground_truth[i].z() - pos_Data[i].z();

        float zero_pos_comp_barrier = 0.001;

        diff_x = abs(diff_x);
        diff_y = abs(diff_y);
        diff_z = abs(diff_z);
        bool comp_res = false;
        if ((float(diff_x / ground_truth[i].x()) < zero_pos_comp_barrier) &&
            (float(diff_y / ground_truth[i].y()) < zero_pos_comp_barrier) &&
            (float(diff_z / ground_truth[i].z()) < zero_pos_comp_barrier)) {
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

TEST(gpuMeshSliding, comprehensiveABSV) {
    std::vector<float> ground_truth;
    std::string dir = GetChronoDataFile("gpu/utest_GT/meshsliding/meshsliding_groundtruth.csv");
#ifdef _WIN32
    char filemame_dummy[] = "dummy.txt";
    char fullFilename_dummy[MAX_PATH];
    GetFullPathName(filemame_dummy, MAX_PATH, fullFilename_dummy, nullptr);
    string name_str_dummy = fullFilename_dummy;
    if (name_str_dummy.find("bin") == std::string::npos) {
        dir = "../../../../bin/Release/" + GetChronoDataFile("gpu/utest_GT/meshsliding/meshsliding_groundtruth.csv");
    }
#else
    char pwd[PATH_MAX];
    string name_str_dummy = pwd;
    if (name_str_dummy.find("bin") == std::string::npos) {
        dir = "../../../" + GetChronoDataFile("gpu/utest_GT/meshsliding/meshsliding_groundtruth.csv");
    }
#endif
    ground_truth = loadColumnCheckpoint(dir, 3);

    EXPECT_EQ(ground_truth.size(), absv_Data.size());

    int false_count = 0;

    for (int i = 0; i < ground_truth.size(); i++) {
        float diff = ground_truth[i] - absv_Data[i];

        float zero_pos_comp_barrier = 0.001;

        diff = abs(diff);
        bool comp_res = false;
        if (float(diff / ground_truth[i]) < zero_pos_comp_barrier) {
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

TEST(gpuMeshSliding, comprehensiveAngularVel) {
    std::vector<float> ground_truth_wx;
    std::vector<float> ground_truth_wy;
    std::vector<float> ground_truth_wz;
    std::string dir = GetChronoDataFile("gpu/utest_GT/meshsliding/meshsliding_groundtruth.csv");
#ifdef _WIN32
    char filemame_dummy[] = "dummy.txt";
    char fullFilename_dummy[MAX_PATH];
    GetFullPathName(filemame_dummy, MAX_PATH, fullFilename_dummy, nullptr);
    string name_str_dummy = fullFilename_dummy;
    if (name_str_dummy.find("bin") == std::string::npos) {
        dir = "../../../../bin/Release/" + GetChronoDataFile("gpu/utest_GT/meshsliding/meshsliding_groundtruth.csv");
    }
#else
    char pwd[PATH_MAX];
    string name_str_dummy = pwd;
    if (name_str_dummy.find("bin") == std::string::npos) {
        dir = "../../../" + GetChronoDataFile("gpu/utest_GT/meshsliding/meshsliding_groundtruth.csv");
    }
#endif
    ground_truth_wx = loadColumnCheckpoint(dir, 4);
    ground_truth_wy = loadColumnCheckpoint(dir, 5);
    ground_truth_wz = loadColumnCheckpoint(dir, 6);

    EXPECT_EQ(ground_truth_wx.size(), ground_truth_wy.size());
    EXPECT_EQ(ground_truth_wy.size(), ground_truth_wz.size());
    EXPECT_EQ(ground_truth_wx.size(), ang_vel_Data.size());

    int false_count = 0;

    for (int i = 0; i < ground_truth_wx.size(); i++) {
        float diff_x = ground_truth_wx[i] - ang_vel_Data[i].x();
        float diff_y = ground_truth_wy[i] - ang_vel_Data[i].y();
        float diff_z = ground_truth_wz[i] - ang_vel_Data[i].z();

        float zero_pos_comp_barrier_ratio = 0.001;
        float zero_pos_comp_barrier = 1e-5;

        diff_x = abs(diff_x);
        diff_y = abs(diff_y);
        diff_z = abs(diff_z);

        bool comp_res = false;
        if (float(diff_x) < zero_pos_comp_barrier && float(diff_y / ground_truth_wy[i]) < zero_pos_comp_barrier_ratio &&
            float(diff_z) < zero_pos_comp_barrier) {
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