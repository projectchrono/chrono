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
// Chrono::Gpu demo using SMC method. A body whose geometry is described
// by an OBJ file is time-integrated in Chrono and interacts with a granular
// wave tank in Chrono::Gpu via the co-simulation framework.
// =============================================================================

#include <iostream>
#include <vector>
#include <string>

#include "chrono/core/ChGlobal.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChForce.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/physics/ChSystemGpu_impl.h"
#include "chrono_gpu/physics/ChSystemGpuMesh_impl.h"
#include "chrono_gpu/utils/ChGpuJsonParser.h"

using namespace chrono;
using namespace chrono::gpu;

void ShowUsage(std::string name) {
    std::cout << "usage: " + name + " <json_file>" << std::endl;
}

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
    if (argc != 2 || ParseJSON(argv[1], params) == false) {
        ShowUsage(argv[0]);
        return 1;
    }

    float iteration_step = params.step_size;

    ChSystemGpuMesh apiSMC_TriMesh(params.sphere_radius, params.sphere_density,
                                   make_float3(params.box_X, params.box_Y, params.box_Z));
    ChSystemGpuMesh_impl& gpu_sys = apiSMC_TriMesh.getSystem();

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

    apiSMC_TriMesh.SetParticlePositions(body_points);

    gpu_sys.set_BD_Fixed(true);
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

    gpu_sys.set_K_n_SPH2SPH(params.normalStiffS2S);
    gpu_sys.set_K_n_SPH2WALL(params.normalStiffS2W);
    gpu_sys.set_K_n_SPH2MESH(params.normalStiffS2M);

    gpu_sys.set_Gamma_n_SPH2SPH(params.normalDampS2S);
    gpu_sys.set_Gamma_n_SPH2WALL(params.normalDampS2W);
    gpu_sys.set_Gamma_n_SPH2MESH(params.normalDampS2M);

    gpu_sys.set_K_t_SPH2SPH(params.tangentStiffS2S);
    gpu_sys.set_K_t_SPH2WALL(params.tangentStiffS2W);
    gpu_sys.set_K_t_SPH2MESH(params.tangentStiffS2M);

    gpu_sys.set_Gamma_t_SPH2SPH(params.tangentDampS2S);
    gpu_sys.set_Gamma_t_SPH2WALL(params.tangentDampS2W);
    gpu_sys.set_Gamma_t_SPH2MESH(params.tangentDampS2M);

    gpu_sys.set_Cohesion_ratio(params.cohesion_ratio);
    gpu_sys.set_Adhesion_ratio_S2M(params.adhesion_ratio_s2m);
    gpu_sys.set_Adhesion_ratio_S2W(params.adhesion_ratio_s2w);
    gpu_sys.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);

    gpu_sys.set_fixed_stepSize(params.step_size);
    gpu_sys.set_friction_mode(CHGPU_FRICTION_MODE::MULTI_STEP);
    gpu_sys.set_timeIntegrator(CHGPU_TIME_INTEGRATOR::CENTERED_DIFFERENCE);
    gpu_sys.set_static_friction_coeff_SPH2SPH(params.static_friction_coeffS2S);
    gpu_sys.set_static_friction_coeff_SPH2WALL(params.static_friction_coeffS2W);
    gpu_sys.set_static_friction_coeff_SPH2MESH(params.static_friction_coeffS2M);

    //gpu_sys.set_rolling_coeff_SPH2SPH(params.rolling_friction_coeffS2S);
    //gpu_sys.set_rolling_coeff_SPH2WALL(params.rolling_friction_coeffS2W);
    //gpu_sys.set_rolling_coeff_SPH2MESH(params.rolling_friction_coeffS2M);

    std::string mesh_filename(GetChronoDataFile("gpu/demo_GPU_ballcosim/sphere.obj"));
    std::vector<string> mesh_filenames(1, mesh_filename);

    std::vector<float3> mesh_translations(1, make_float3(0.f, 0.f, 0.f));

    float ball_radius = 20.f;
    std::vector<ChMatrix33<float>> mesh_rotscales(1, ChMatrix33<float>(ball_radius));

    float ball_density = params.sphere_density;
    float ball_mass = (float)(4.f * CH_C_PI * ball_radius * ball_radius * ball_radius * ball_density / 3.f);
    std::vector<float> mesh_masses(1, ball_mass);

    apiSMC_TriMesh.LoadMeshes(mesh_filenames, mesh_rotscales, mesh_translations, mesh_masses);

    gpu_sys.setOutputMode(params.write_mode);
    gpu_sys.setVerbose(params.verbose);
    filesystem::create_directory(filesystem::path(params.output_dir));

    unsigned int nSoupFamilies = gpu_sys.getNumTriangleFamilies();
    std::cout << nSoupFamilies << " soup families" << std::endl;

    gpu_sys.initialize();

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
    sys_ball.AddBody(ball_body);
    unsigned int out_fps = 50;
    std::cout << "Rendering at " << out_fps << "FPS" << std::endl;

    unsigned int out_steps = (unsigned int)(1.0 / (out_fps * iteration_step));
    unsigned int total_frames = (unsigned int)((float)params.time_end * out_fps);

    int currframe = 0;
    unsigned int curr_step = 0;

    clock_t start = std::clock();
    for (double t = 0; t < (double)params.time_end; t += iteration_step, curr_step++) {
        gpu_sys.applyMeshMotion(0, ball_body->GetPos().data(), ball_body->GetRot().data(),
                                ball_body->GetPos_dt().data(), ball_body->GetWvel_par().data());


        gpu_sys.advance_simulation(iteration_step);
        sys_ball.DoStepDynamics(iteration_step);

        float ball_force[6];
        gpu_sys.collectGeneralizedForcesOnMeshSoup(ball_force);

        ball_body->Empty_forces_accumulators();
        ball_body->Accumulate_force(ChVector<>(ball_force[0], ball_force[1], ball_force[2]), ball_body->GetPos(),
                                    false);
        ball_body->Accumulate_torque(ChVector<>(ball_force[3], ball_force[4], ball_force[5]), false);

        if (curr_step % out_steps == 0) {
            std::cout << "Rendering frame " << currframe << " of " << total_frames << std::endl;
            char filename[100];
            sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
            gpu_sys.writeFile(std::string(filename));
            gpu_sys.write_meshes(std::string(filename));

            /*  // disable meshframes output, for it may be confusing for users dealing with Chrono::Gpu only
            std::string mesh_output = std::string(filename) + "_meshframes.csv";
            std::ofstream meshfile(mesh_output);
            std::ostringstream outstream;
            outstream << "mesh_name,dx,dy,dz,x1,x2,x3,y1,y2,y3,z1,z2,z3,sx,sy,sz\n";
            writeMeshFrames(outstream, *ball_body, mesh_filename, ball_radius);
            meshfile << outstream.str();
            */
        }
    }

    clock_t end = std::clock();
    double total_time = ((double)(end - start)) / CLOCKS_PER_SEC;

    std::cout << "Time: " << total_time << " seconds" << std::endl;

    return 0;
}
