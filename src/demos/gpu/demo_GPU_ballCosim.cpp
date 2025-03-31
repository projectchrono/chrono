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
// Authors: Nic Olsen, Ruochun Zhang
// =============================================================================
// Chrono::Gpu demo using SMC method. A body whose geometry is described by an
// OBJ file is time-integrated in Chrono and interacts with a granular wave tank
// in Chrono::Gpu via the co-simulation framework. The entire simulation consists
// of 2 runs: the settling phase (which outputs a checkpoint file), and a restarted
// phase (which load the checkpoint file and then drop the ball, literally).
// =============================================================================

#include <iostream>
#include <vector>
#include <string>

#include "chrono/core/ChGlobal.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChForce.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/assets/ChVisualShapeSphere.h"

#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/utils/ChGpuJsonParser.h"

#include "chrono_gpu/visualization/ChGpuVisualization.h"
#ifdef CHRONO_OPENGL
    #include "chrono_gpu/visualization/ChGpuVisualizationGL.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::gpu;

// Output frequency
float out_fps = 50;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = true;
float render_fps = 2000;

void runBallDrop(ChSystemGpuMesh& gpu_sys, ChGpuSimulationParameters& params) {
    // Add a ball mesh to the GPU system
    float ball_radius = 20.f;
    float ball_density = params.sphere_density;
    float ball_mass = 4.0f * (float)CH_PI * ball_radius * ball_radius * ball_radius * ball_density / 3.f;
    gpu_sys.AddMesh(GetChronoDataFile("models/sphere.obj"), ChVector3f(0), ChMatrix33<float>(ball_radius), ball_mass);

    // One more thing: we need to manually enable mesh in this run, because we disabled it in the settling phase,
    // let's overload that option.
    gpu_sys.EnableMeshCollision(true);

    gpu_sys.Initialize();
    std::cout << gpu_sys.GetNumMeshes() << " meshes" << std::endl;

    // Create rigid ball_body simulation
    ChSystemSMC sys_ball;
    sys_ball.SetContactForceModel(ChSystemSMC::ContactForceModel::Hooke);
    sys_ball.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
    sys_ball.SetGravitationalAcceleration(ChVector3d(0, 0, -980));

    double inertia = 2.0 / 5.0 * ball_mass * ball_radius * ball_radius;
    ChVector3d ball_initial_pos(0, 0, params.box_Z / 4.0 + ball_radius + 2 * params.sphere_radius);

    auto ball_body = chrono_types::make_shared<ChBody>();
    ball_body->SetMass(ball_mass);
    ball_body->SetInertiaXX(ChVector3d(inertia, inertia, inertia));
    ball_body->SetPos(ball_initial_pos);
    auto sph = chrono_types::make_shared<ChVisualShapeSphere>(ball_radius);
    ball_body->AddVisualShape(sph);
    sys_ball.AddBody(ball_body);

#if !defined(CHRONO_OPENGL)
    render = false;
#endif

    std::shared_ptr<ChGpuVisualization> visGPU;
    if (render) {
#ifdef CHRONO_OPENGL
        visGPU = chrono_types::make_shared<ChGpuVisualizationGL>(&gpu_sys);
        visGPU->SetTitle("Chrono::Gpu ball cosim demo");
        visGPU->AddCamera(ChVector3d(0, -200, 100), ChVector3d(0, 0, 0));
        visGPU->SetCameraMoveScale(1.0f);
        visGPU->Initialize();
#endif
    }

    std::string out_dir = GetChronoOutputPath() + "GPU/";
    filesystem::create_directory(filesystem::path(out_dir));
    out_dir = out_dir + params.output_dir;
    filesystem::create_directory(filesystem::path(out_dir));

    float iteration_step = params.step_size;
    std::cout << "Output at    " << out_fps << " FPS" << std::endl;
    std::cout << "Rendering at " << render_fps << " FPS" << std::endl;

    int sim_frame = 0;
    int render_frame = 0;
    int out_frame = 0;

    // Add force accumulator to the ball body
    auto accumulator_index = ball_body->AddAccumulator();

    clock_t start = std::clock();
    for (double t = 0; t < (double)params.time_end; t += iteration_step) {
        gpu_sys.ApplyMeshMotion(0, ball_body->GetPos(), ball_body->GetRot(), ball_body->GetPosDt(),
                                ball_body->GetAngVelParent());

        ChVector3d ball_force;
        ChVector3d ball_torque;
        gpu_sys.CollectMeshContactForces(0, ball_force, ball_torque);

        ball_body->EmptyAccumulator(accumulator_index);
        ball_body->AccumulateForce(accumulator_index, ball_force, ball_body->GetPos(), false);
        ball_body->AccumulateTorque(accumulator_index, ball_torque, false);

        if (t >= out_frame / out_fps) {
            std::cout << "Output frame " << sim_frame + 1 << std::endl;
            char filename[100];
            char mesh_filename[100];
            sprintf(filename, "%s/step%06d.csv", out_dir.c_str(), sim_frame);
            sprintf(mesh_filename, "%s/step%06d_mesh", out_dir.c_str(), sim_frame);
            gpu_sys.WriteParticleFile(std::string(filename));
            gpu_sys.WriteMeshes(std::string(mesh_filename));

            out_frame++;
        }

        if (render && t >= render_frame / render_fps) {
            if (!visGPU->Render())
                break;
            render_frame++;
        }

        gpu_sys.AdvanceSimulation(iteration_step);
        sys_ball.DoStepDynamics(iteration_step);

        sim_frame++;
    }

    clock_t end = std::clock();
    double total_time = ((double)(end - start)) / CLOCKS_PER_SEC;

    std::cout << "Time: " << total_time << " seconds" << std::endl;
}

int main(int argc, char* argv[]) {
    std::string inputJson = GetChronoDataFile("gpu/ballCosim.json");
    if (argc == 2) {
        inputJson = std::string(argv[1]);
    } else if (argc > 2) {
        std::cout << "Usage:\n./demo_GPU_ballCosim <json_file>" << std::endl;
        return 1;
    }

    ChGpuSimulationParameters params;
    if (!ParseJSON(inputJson, params)) {
        std ::cout << "ERROR: reading input file " << inputJson << std::endl;
        return 1;
    }

    if (params.run_mode != CHGPU_RUN_MODE::FRICTIONLESS && params.run_mode != CHGPU_RUN_MODE::ONE_STEP) {
        std::cout << "ERROR: unknown run_mode specified" << std::endl;
        return 1;
    }

    // Output directory
    std::string out_dir = GetChronoOutputPath() + "GPU/";
    filesystem::create_directory(filesystem::path(out_dir));
    out_dir = out_dir + params.output_dir;
    filesystem::create_directory(filesystem::path(out_dir));

    std::string checkpoint_file = out_dir + "/checkpoint.dat";

    if (params.run_mode == CHGPU_RUN_MODE::ONE_STEP) {
        // This is a restarted run

        // Load checkpoint file.
        // Note that with current version, user defined meshes and boundaries are not stored in the checkpoint file,
        // so they must be manually set later. This behavior will be improved in later patches.
        // Simulation parameters and particle states are all in with this file loaded.
        ChSystemGpuMesh gpu_sys(checkpoint_file);

        // Add a ball through a mesh, whose dynamics are managed by Chrono Core, and run this co-simulation.
        runBallDrop(gpu_sys, params);

        return 0;
    }

    // run_mode = CHGPU_RUN_MODE::FRICTIONLESS, this is a newly started run. We have to set all simulation params.
    ChSystemGpuMesh gpu_sys(params.sphere_radius, params.sphere_density,
                            ChVector3f(params.box_X, params.box_Y, params.box_Z));

    printf(
        "Now run_mode == FRICTIONLESS, this run is particle settling phase.\n"
        "After it is done, you will have a settled bed of granular material.\n"
        "A checkpoint file will be generated in the output directory to store this state.\n"
        "Next, edit the JSON file, change 'run_mode' from 0 (FRICTIONLESS) to 1 (ONE_STEP),\n"
        "then run this demo again to proceed with the ball drop part of this demo.\n\n");

    float iteration_step = params.step_size;
    double fill_bottom = -params.box_Z / 2.0;
    double fill_top = params.box_Z / 4.0;

    chrono::utils::ChPDSampler<float> sampler(2.4f * params.sphere_radius);
    // chrono::utils::ChHCPSampler<float> sampler(2.05 * params.sphere_radius);

    // leave a 4cm margin at edges of sampling
    ChVector3d hdims(params.box_X / 2 - 4.0, params.box_Y / 2 - 4.0, 0);
    ChVector3d center(0, 0, fill_bottom + 2.0 * params.sphere_radius);
    std::vector<ChVector3f> body_points;

    // Shift up for bottom of box
    center.z() += 3 * params.sphere_radius;
    while (center.z() < fill_top) {
        // You can uncomment this line to see a report on particle creation process.
        // std::cout << "Create layer at " << center.z() << std::endl;
        auto points = sampler.SampleBox(center, hdims);
        body_points.insert(body_points.end(), points.begin(), points.end());
        center.z() += 2.05 * params.sphere_radius;
    }
    std::cout << body_points.size() << " particles sampled!" << std::endl;

    gpu_sys.SetParticles(body_points);

    gpu_sys.SetKn_SPH2SPH(params.normalStiffS2S);
    gpu_sys.SetKn_SPH2WALL(params.normalStiffS2W);
    gpu_sys.SetKn_SPH2MESH(params.normalStiffS2M);

    gpu_sys.SetGn_SPH2SPH(params.normalDampS2S);
    gpu_sys.SetGn_SPH2WALL(params.normalDampS2W);
    gpu_sys.SetGn_SPH2MESH(params.normalDampS2M);

    gpu_sys.SetKt_SPH2SPH(params.tangentStiffS2S);
    gpu_sys.SetKt_SPH2WALL(params.tangentStiffS2W);
    gpu_sys.SetKt_SPH2MESH(params.tangentStiffS2M);

    gpu_sys.SetGt_SPH2SPH(params.tangentDampS2S);
    gpu_sys.SetGt_SPH2WALL(params.tangentDampS2W);
    gpu_sys.SetGt_SPH2MESH(params.tangentDampS2M);

    gpu_sys.SetCohesionRatio(params.cohesion_ratio);
    gpu_sys.SetAdhesionRatio_SPH2MESH(params.adhesion_ratio_s2m);
    gpu_sys.SetAdhesionRatio_SPH2WALL(params.adhesion_ratio_s2w);

    gpu_sys.SetGravitationalAcceleration(ChVector3f(params.grav_X, params.grav_Y, params.grav_Z));

    gpu_sys.SetFixedStepSize(params.step_size);
    gpu_sys.SetFrictionMode(CHGPU_FRICTION_MODE::MULTI_STEP);
    gpu_sys.SetTimeIntegrator(CHGPU_TIME_INTEGRATOR::CENTERED_DIFFERENCE);

    gpu_sys.SetStaticFrictionCoeff_SPH2SPH(params.static_friction_coeffS2S);
    gpu_sys.SetStaticFrictionCoeff_SPH2WALL(params.static_friction_coeffS2W);
    gpu_sys.SetStaticFrictionCoeff_SPH2MESH(params.static_friction_coeffS2M);

    // gpu_sys.SetRollingCoeff_SPH2SPH(params.rolling_friction_coeffS2S);
    // gpu_sys.SetRollingCoeff_SPH2WALL(params.rolling_friction_coeffS2W);
    // gpu_sys.SetRollingCoeff_SPH2MESH(params.rolling_friction_coeffS2M);

    gpu_sys.SetParticleOutputMode(params.write_mode);
    gpu_sys.SetVerbosity(params.verbose);
    gpu_sys.SetBDFixed(true);

    // In the settling run we disable the mesh.
    gpu_sys.EnableMeshCollision(false);

    /*
    // We could prescribe the motion of the big box domain. But here in this demo we will not do that.
    std::function<double3(float)> pos_func_wave = [&params](float t) {
        double3 pos = {0, 0, 0};

        double t0 = 0.5;
        double freq = CH_PI / 4;

        if (t > t0) {
            pos.x = 0.1 * params.box_X * std::sin((t - t0) * freq);
        }
        return pos;
    };

    gpu_sys.setBDWallsMotionFunction(pos_func_wave);
    */

    gpu_sys.Initialize();

    int sim_frame = 0;
    int out_frame = 0;

    for (double t = 0; t < (double)params.time_end; t += iteration_step) {
        if (t >= out_frame / out_fps) {
            std::cout << "Output frame " << sim_frame + 1 << std::endl;
            char filename[100];
            sprintf(filename, "%s/step%06d.csv", out_dir.c_str(), sim_frame);
            gpu_sys.WriteParticleFile(std::string(filename));

            out_frame++;
        }

        gpu_sys.AdvanceSimulation(iteration_step);

        sim_frame++;
    }

    // This is settling phase, so output a checkpoint file
    gpu_sys.WriteCheckpointFile(checkpoint_file);

    return 0;
}
