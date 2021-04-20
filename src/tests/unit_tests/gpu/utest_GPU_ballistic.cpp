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
// Authors: Ruochun Zhang, Jason Zhou
// =============================================================================
// validation test: high velocity sphere punching through a mesh facet
//
// utilizes gtest infrustructure
// test 1 : check ball hit result/time and penetration results/time
// =============================================================================

#include <cmath>
#include <iostream>
#include <string>

#include "chrono/core/ChGlobal.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_gpu/ChGpuData.h"
#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/utils/ChGpuJsonParser.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "unit_testing.h"

#include "gtest/gtest.h"

using namespace chrono;
using namespace chrono::gpu;

// declare global variables
float hit_time = -1;
float penetration_time = -1;

void ShowUsage(std::string name) {
    std::cout << "usage: " + name + " <json_file> \n " << std::endl;
}

int main(int argc, char* argv[]) {
    string json_dir = GetChronoDataPath() + "testing/gpu/utest_GPU_ballistic/utest_GPU_ballistic.json";
    const char* c_buff = json_dir.c_str();
    ChGpuSimulationParameters params;

    // check whether JSON parameters file is valid
    // Parse JSON parameters to the gpu system
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

    mesh_filename = GetChronoDataPath() + "testing/gpu/utest_GPU_ballistic/one_facet.obj";

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

    // assign initial condition for the sphere
    float initialVelo = 1e4;
    std::vector<ChVector<float>> body_point;
    body_point.push_back(ChVector<float>(1.0f, -1.0f, 4.f));
    std::vector<ChVector<float>> velocity;
    velocity.push_back(ChVector<float>(0.0f, 0.0f, -initialVelo));
    gpu_sys.SetParticlePositions(body_point, velocity);

    gpu_sys.SetPsiFactors(params.psi_T, params.psi_L);

    // set normal force model
    gpu_sys.SetKn_SPH2SPH(params.normalStiffS2S);
    gpu_sys.SetKn_SPH2WALL(params.normalStiffS2W);
    gpu_sys.SetKn_SPH2MESH(params.normalStiffS2M);
    gpu_sys.SetGn_SPH2SPH(params.normalDampS2S);
    gpu_sys.SetGn_SPH2WALL(params.normalDampS2W);
    gpu_sys.SetGn_SPH2MESH(params.normalDampS2M);

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

    unsigned int currframe = 0;
    unsigned int curstep = 0;
    double out_fps = 100000;
    float frame_step = 1.f / out_fps;  // Duration of a frame
    unsigned int out_steps = frame_step / params.step_size;
    unsigned int total_frames = (unsigned int)((float)params.time_end * out_fps);
    std::cout << "out_steps " << out_steps << std::endl;

    bool hit = false;
    while (curr_time < params.time_end) {
        gpu_sys.AdvanceSimulation(params.step_size);
        curr_time += params.step_size;

        pos = gpu_sys.GetParticlePosition(0);
        velo = gpu_sys.GetParticleVelocity(0);

        if (curstep % out_steps == 0) {
            char filename[100];
            sprintf(filename, "%s/step%06u", params.output_dir.c_str(), currframe++);
            // the following lines are used to obtain ground truth data
            // gpu_sys.writeFile(std::string(filename));
            // gpu_sys.write_meshes(std::string(filename));
        }

        if (pos.z() < -params.sphere_radius) {
            printf("Mesh penetration detected! Exiting...");
            penetration_time = curr_time;
            break;

        } else if ((pos.z() < params.sphere_radius) && (!hit)) {
            hit_time = curr_time;
            printf("Recorded a hit.\n");
            hit = true;

        } else if ((pos.z() > params.sphere_radius) && hit) {
            printf("Bounced! Exiting...");
            break;
        }

        curstep++;
    }

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(gpuBallistic, resultTimePointCheck) {
    float diff_hit = hit_time - (float)0.00035;
    float diff_pene = penetration_time - (float)0.00045;

    bool hit_check = true;
    bool pene_check = true;

    float zero_barrier = 1e-5;

    if (diff_hit > zero_barrier) {
        hit_check = false;
    }

    if (diff_pene > zero_barrier) {
        pene_check = false;
    }

    EXPECT_EQ(true, hit_check);
    EXPECT_EQ(true, pene_check);
}
