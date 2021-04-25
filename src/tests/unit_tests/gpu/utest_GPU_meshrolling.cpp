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
// One particle rolling on the mesh surface
// This test will check the rolling friction model implemented in chrono::gpu
// When interaction is between Particle vs Mesh Wall
// =============================================================================

#include <cmath>
#include <iostream>
#include <string>
#include "gtest/gtest.h"
#include "unit_testing.h"

#include "chrono/core/ChGlobal.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_gpu/ChGpuData.h"
#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/utils/ChGpuJsonParser.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::gpu;

// declare global variables
float precision_vel = 1e-2;
float precision_pos = 1e-2;
float precision_ang = 3e-2;

ChVector<float> end_pos;
ChVector<float> end_vel;
ChVector<float> end_ang_vel;

float settled_pos;
float radius;

void ShowUsage(std::string name) {
    std::cout << "usage: " + name + " <json_file> \n " << std::endl;
}

int main(int argc, char* argv[]) {
    string json_dir = GetChronoDataPath() + "testing/gpu/utest_GPU_meshsliding/utest_GPU_meshsliding.json";

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

    float mu_s = 0.2;
    float mu_r;

    mu_r = 0.0008;

    ChSystemGpuMesh gpu_sys(params.sphere_radius, params.sphere_density,
                            make_float3(params.box_X, params.box_Y, params.box_Z));

    // load in the mesh
    std::vector<string> mesh_filenames;
    std::string mesh_filename;

    mesh_filename = GetChronoDataPath() + "testing/gpu/utest_GPU_meshsliding/one_facet.obj";

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
    double penetration = pow(mass * abs(params.grav_Z) / params.normalStiffS2S, 2.0 / 3.0);
    settled_pos = params.sphere_radius - penetration;

    std::vector<ChVector<float>> body_point;
    body_point.push_back(ChVector<float>(1.0f, -1.0f, params.sphere_radius));

    std::vector<ChVector<float>> velocity;
    velocity.push_back(ChVector<float>(1.0f, 0.0f, 0.0f));
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
    gpu_sys.SetStaticFrictionCoeff_SPH2SPH(mu_s);
    gpu_sys.SetStaticFrictionCoeff_SPH2WALL(mu_s);
    gpu_sys.SetStaticFrictionCoeff_SPH2MESH(mu_s);
    gpu_sys.SetFrictionMode(CHGPU_FRICTION_MODE::MULTI_STEP);

    // set rolling params
    gpu_sys.SetRollingMode(CHGPU_ROLLING_MODE::SCHWARTZ);
    gpu_sys.SetRollingCoeff_SPH2SPH(mu_r);
    gpu_sys.SetRollingCoeff_SPH2WALL(mu_r);
    gpu_sys.SetRollingCoeff_SPH2MESH(mu_r);

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
        if (gpu_sys.GetParticleVelocity(0) < precision_vel) {
            std::cout << "the particle's linear velocity drops below threshold 1e-2 at time = " << curr_time
                      << std::endl;
            break;
        }
    }

    end_pos = gpu_sys.GetParticlePosition(0);
    end_vel = gpu_sys.GetParticleVelocity(0);
    end_ang_vel = gpu_sys.GetParticleAngVelocity(0);

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

// test: check the end velocity and angular velocity
// test that the ball stops rolling
TEST(gpuMeshSliding, endPos) {
    // check position y and z component
    EXPECT_NEAR(end_pos.y(), -1, precision_pos);
    EXPECT_NEAR(end_pos.z(), settled_pos, precision_pos);

    // check end velocity
    EXPECT_NEAR(end_vel.x(), 0, precision_vel);
    EXPECT_NEAR(end_vel.y(), 0, precision_vel);
    EXPECT_NEAR(end_vel.z(), 0, precision_vel);

    // check ang velocity
    EXPECT_NEAR(end_ang_vel.x(), 0, precision_ang);
    EXPECT_NEAR(end_ang_vel.y(), 0, precision_ang);
    EXPECT_NEAR(end_ang_vel.z(), 0, precision_ang);
}
