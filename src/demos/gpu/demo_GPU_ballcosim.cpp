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
// Authors: Nic Olsen
// =============================================================================
// Chrono::Gpu demo using SMC method. A body whose geometry is described by an
// OBJ file is time-integrated in Chrono and interacts with a granular wave tank
// in Chrono::Gpu via the co-simulation framework.
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
#include "chrono/assets/ChSphereShape.h"

#include "chrono_gpu/ChGpuData.h"
#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/utils/ChGpuJsonParser.h"
#include "chrono_gpu/utils/ChGpuVisualization.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::gpu;

// Output frequency
float out_fps = 50;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = true;
float render_fps = 2000;

void writeMeshFrames(std::ostringstream& outstream, ChBody& body, std::string obj_name, float mesh_scaling) {
    outstream << obj_name << ",";

    // Get frame position
    ChFrame<> body_frame = body.GetFrame_REF_to_abs();
    ChQuaternion<> rot = body_frame.GetRot();
    ChVector<> pos = body_frame.GetPos();

    // Get basis vectors
    ChVector<> vx = rot.GetXaxis();
    ChVector<> vy = rot.GetYaxis();
    ChVector<> vz = rot.GetZaxis();

    // Output in order
    outstream << pos.x() << ",";
    outstream << pos.y() << ",";
    outstream << pos.z() << ",";
    outstream << vx.x() << ",";
    outstream << vx.y() << ",";
    outstream << vx.z() << ",";
    outstream << vy.x() << ",";
    outstream << vy.y() << ",";
    outstream << vy.z() << ",";
    outstream << vz.x() << ",";
    outstream << vz.y() << ",";
    outstream << vz.z() << ",";
    outstream << mesh_scaling << "," << mesh_scaling << "," << mesh_scaling;
    outstream << "\n";
}

int main(int argc, char* argv[]) {
    ChGpuSimulationParameters params;
    if (argc != 2 || ParseJSON(gpu::GetDataFile(argv[1]), params) == false) {
        std::cout << "Usage:\n./demo_GPU_ballcosim <json_file>" << std::endl;
        return 1;
    }

    float iteration_step = params.step_size;

    ChSystemGpuMesh gpu_sys(params.sphere_radius, params.sphere_density,
                            make_float3(params.box_X, params.box_Y, params.box_Z));

    double fill_bottom = -params.box_Z / 2.0;
    double fill_top = params.box_Z / 4.0;

    chrono::utils::PDSampler<float> sampler(2.4f * params.sphere_radius);
    // chrono::utils::HCPSampler<float> sampler(2.05 * params.sphere_radius);

    // leave a 4cm margin at edges of sampling
    ChVector<> hdims(params.box_X / 2 - 4.0, params.box_Y / 2 - 4.0, 0);
    ChVector<> center(0, 0, fill_bottom + 2.0 * params.sphere_radius);
    std::vector<ChVector<float>> body_points;

    // Shift up for bottom of box
    center.z() += 3 * params.sphere_radius;
    while (center.z() < fill_top) {
        std::cout << "Create layer at " << center.z() << std::endl;
        auto points = sampler.SampleBox(center, hdims);
        body_points.insert(body_points.end(), points.begin(), points.end());
        center.z() += 2.05 * params.sphere_radius;
    }

    gpu_sys.SetParticlePositions(body_points);

    gpu_sys.SetBDFixed(true);
    std::function<double3(float)> pos_func_wave = [&params](float t) {
        double3 pos = {0, 0, 0};

        double t0 = 0.5;
        double freq = CH_C_PI / 4;

        if (t > t0) {
            pos.x = 0.1 * params.box_X * std::sin((t - t0) * freq);
        }
        return pos;
    };

    // gpu_sys.setBDWallsMotionFunction(pos_func_wave);

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

    gpu_sys.SetGravitationalAcceleration(ChVector<float>(params.grav_X, params.grav_Y, params.grav_Z));

    gpu_sys.SetFixedStepSize(params.step_size);
    gpu_sys.SetFrictionMode(CHGPU_FRICTION_MODE::MULTI_STEP);
    gpu_sys.SetTimeIntegrator(CHGPU_TIME_INTEGRATOR::CENTERED_DIFFERENCE);

    gpu_sys.SetStaticFrictionCoeff_SPH2SPH(params.static_friction_coeffS2S);
    gpu_sys.SetStaticFrictionCoeff_SPH2WALL(params.static_friction_coeffS2W);
    gpu_sys.SetStaticFrictionCoeff_SPH2MESH(params.static_friction_coeffS2M);

    // gpu_sys.SetRollingCoeff_SPH2SPH(params.rolling_friction_coeffS2S);
    // gpu_sys.SetRollingCoeff_SPH2WALL(params.rolling_friction_coeffS2W);
    // gpu_sys.SetRollingCoeff_SPH2MESH(params.rolling_friction_coeffS2M);

    // Add a ball mesh to the GPU system
    float ball_radius = 20.f;
    float ball_density = params.sphere_density;
    float ball_mass = 4.0f * (float)CH_C_PI * ball_radius * ball_radius * ball_radius * ball_density / 3.f;
    std::string mesh_filename(GetChronoDataFile("models/sphere.obj"));
    auto ball_mesh_id = gpu_sys.AddMesh(mesh_filename, ChVector<float>(0), ChMatrix33<float>(ball_radius), ball_mass);

    gpu_sys.SetOutputMode(params.write_mode);
    gpu_sys.SetVerbosity(params.verbose);

    gpu_sys.Initialize();
    std::cout << gpu_sys.GetNumMeshes() << " meshes" << std::endl;

    // Create rigid ball_body simulation
    ChSystemSMC sys_ball;
    sys_ball.SetContactForceModel(ChSystemSMC::ContactForceModel::Hooke);
    sys_ball.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
    sys_ball.Set_G_acc(ChVector<>(0, 0, -980));

    double inertia = 2.0 / 5.0 * ball_mass * ball_radius * ball_radius;
    ChVector<> ball_initial_pos(0, 0, fill_top + ball_radius + 2 * params.sphere_radius);

    std::shared_ptr<ChBody> ball_body(sys_ball.NewBody());
    ball_body->SetMass(ball_mass);
    ball_body->SetInertiaXX(ChVector<>(inertia, inertia, inertia));
    ball_body->SetPos(ball_initial_pos);
    auto sph = chrono_types::make_shared<ChSphereShape>();
    sph->GetSphereGeometry().rad = ball_radius;
    ball_body->AddAsset(sph);
    sys_ball.AddBody(ball_body);

    ChGpuVisualization gpu_vis(&gpu_sys, &sys_ball);
    if (render) {
        gpu_vis.SetTitle("Chrono::Gpu ball cosim demo");
        gpu_vis.SetCameraPosition(ChVector<>(0, -200, 100), ChVector<>(0, 0, 0));
        gpu_vis.SetCameraMoveScale(1.0f);
        gpu_vis.Initialize();
    }

    std::string out_dir = GetChronoOutputPath() + "GPU/";
    filesystem::create_directory(filesystem::path(out_dir));
    out_dir = out_dir + params.output_dir;
    filesystem::create_directory(filesystem::path(out_dir));

    std::cout << "Output at    " << out_fps << " FPS" << std::endl;
    std::cout << "Rendering at " << render_fps << " FPS" << std::endl;
    unsigned int out_steps = (unsigned int)(1 / (out_fps * iteration_step));
    unsigned int render_steps = (unsigned int)(1 / (render_fps * iteration_step));
    unsigned int total_frames = (unsigned int)(params.time_end * out_fps);

    int currframe = 0;
    unsigned int curr_step = 0;

    clock_t start = std::clock();
    for (double t = 0; t < (double)params.time_end; t += iteration_step, curr_step++) {
        gpu_sys.ApplyMeshMotion(0, ball_body->GetPos(), ball_body->GetRot(), ball_body->GetPos_dt(),
                                ball_body->GetWvel_par());

        ChVector<> ball_force;
        ChVector<> ball_torque;
        gpu_sys.CollectMeshContactForces(0, ball_force, ball_torque);

        ball_body->Empty_forces_accumulators();
        ball_body->Accumulate_force(ball_force, ball_body->GetPos(), false);
        ball_body->Accumulate_torque(ball_torque, false);

        if (curr_step % out_steps == 0) {
            std::cout << "Output frame " << currframe + 1 << " of " << total_frames << std::endl;
            char filename[100];
            sprintf(filename, "%s/step%06d", out_dir.c_str(), currframe++);
            gpu_sys.WriteFile(std::string(filename));
            gpu_sys.WriteMeshes(std::string(filename));

            /*  // disable meshframes output, for it may be confusing for users dealing with Chrono::Gpu only
            std::string mesh_output = std::string(filename) + "_meshframes.csv";
            std::ofstream meshfile(mesh_output);
            std::ostringstream outstream;
            outstream << "mesh_name,dx,dy,dz,x1,x2,x3,y1,y2,y3,z1,z2,z3,sx,sy,sz\n";
            writeMeshFrames(outstream, *ball_body, mesh_filename, ball_radius);
            meshfile << outstream.str();
            */
        }

        if (render && curr_step % render_steps == 0) {
            if (gpu_vis.Render())
                break;
        }

        gpu_sys.AdvanceSimulation(iteration_step);
        sys_ball.DoStepDynamics(iteration_step);
    }

    clock_t end = std::clock();
    double total_time = ((double)(end - start)) / CLOCKS_PER_SEC;

    std::cout << "Time: " << total_time << " seconds" << std::endl;

    return 0;
}
