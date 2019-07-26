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
// Chrono::Granular demo using SMC method. A body whose geometry is described
// by an OBJ file is time-integrated in Chrono and interacts with a Granular
// wave tank in Chrono::Granular via the co-simulation framework.
// =============================================================================

#include <iostream>
#include <vector>
#include <string>
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono/core/ChTimer.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChForce.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/physics/ChGranularTriMesh.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_granular/utils/ChGranularJsonParser.h"

using namespace chrono;
using namespace chrono::granular;

using std::cout;
using std::endl;
using std::string;
using std::vector;

void ShowUsage() {
    cout << "usage: ./metrics_GRAN_ballcosim <json_file>" << endl;
}

void writeMeshFrames(std::ostringstream& outstream, ChBody& body, string obj_name, float mesh_scaling) {
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
    sim_param_holder params;
    if (argc != 2 || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }

    double iteration_step = params.step_size;

    // Setup granular simulation
    ChSystemGranularSMC_trimesh gran_sys(params.sphere_radius, params.sphere_density,
                                         make_float3(params.box_X, params.box_Y, params.box_Z));

    double fill_bottom = -params.box_Z / 2.0;
    double fill_top = params.box_Z / 4.0;

    chrono::utils::PDSampler<float> sampler(2.4 * params.sphere_radius);
    // chrono::utils::HCPSampler<float> sampler(2.05 * params.sphere_radius);

    // leave a 4cm margin at edges of sampling
    ChVector<> hdims(params.box_X / 2 - 4.0, params.box_Y / 2 - 4.0, 0);
    ChVector<> center(0, 0, fill_bottom + 2.0 * params.sphere_radius);
    vector<ChVector<float>> body_points;

    // Shift up for bottom of box
    center.z() += 3 * params.sphere_radius;
    while (center.z() < fill_top) {
        cout << "Create layer at " << center.z() << endl;
        auto points = sampler.SampleBox(center, hdims);
        body_points.insert(body_points.end(), points.begin(), points.end());
        center.z() += 2.05 * params.sphere_radius;
    }

    gran_sys.setParticlePositions(body_points);

    gran_sys.set_BD_Fixed(true);
    std::function<double3(float)> pos_func_wave = [&params](float t) {
        double3 pos = {0, 0, 0};

        double t0 = 0.5;
        double freq = M_PI / 4;

        if (t > t0) {
            pos.x = 0.1 * params.box_X * std::sin((t - t0) * freq);
        }
        return pos;
    };

    // gran_sys.setBDWallsMotionFunction(pos_func_wave);

    gran_sys.set_K_n_SPH2SPH(params.normalStiffS2S);
    gran_sys.set_K_n_SPH2WALL(params.normalStiffS2W);
    gran_sys.set_K_n_SPH2MESH(params.normalStiffS2M);

    gran_sys.set_Gamma_n_SPH2SPH(params.normalDampS2S);
    gran_sys.set_Gamma_n_SPH2WALL(params.normalDampS2W);
    gran_sys.set_Gamma_n_SPH2MESH(params.normalDampS2M);

    gran_sys.set_K_t_SPH2SPH(params.tangentStiffS2S);
    gran_sys.set_K_t_SPH2WALL(params.tangentStiffS2W);
    gran_sys.set_K_t_SPH2MESH(params.tangentStiffS2M);

    gran_sys.set_Gamma_t_SPH2SPH(params.tangentDampS2S);
    gran_sys.set_Gamma_t_SPH2WALL(params.tangentDampS2W);
    gran_sys.set_Gamma_t_SPH2MESH(params.tangentDampS2M);

    gran_sys.set_Cohesion_ratio(params.cohesion_ratio);
    gran_sys.set_Adhesion_ratio_S2M(params.adhesion_ratio_s2m);
    gran_sys.set_Adhesion_ratio_S2W(params.adhesion_ratio_s2w);
    gran_sys.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);

    gran_sys.set_fixed_stepSize(params.step_size);
    gran_sys.set_friction_mode(GRAN_FRICTION_MODE::MULTI_STEP);
    gran_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::CENTERED_DIFFERENCE);
    gran_sys.set_static_friction_coeff_SPH2SPH(params.static_friction_coeffS2S);
    gran_sys.set_static_friction_coeff_SPH2WALL(params.static_friction_coeffS2W);
    gran_sys.set_static_friction_coeff_SPH2MESH(params.static_friction_coeffS2M);

    string mesh_filename("data/granular/ballcosim/sphere.obj");
    vector<string> mesh_filenames(1, mesh_filename);

    vector<float3> mesh_translations(1, make_float3(0, 0, 0));

    float ball_radius = 20;
    vector<ChMatrix33<float>> mesh_rotscales(1, ChMatrix33<float>(ball_radius));

    float ball_density = params.sphere_density / 100;
    float ball_mass = 4.0 / 3.0 * CH_C_PI * ball_radius * ball_radius * ball_radius * ball_density;
    vector<float> mesh_masses(1, ball_mass);

    vector<bool> mesh_inflated(1, false);
    vector<float> mesh_inflation_radii(1, 0);

    gran_sys.load_meshes(mesh_filenames, mesh_rotscales, mesh_translations, mesh_masses, mesh_inflated,
                         mesh_inflation_radii);

    gran_sys.setOutputMode(params.write_mode);
    gran_sys.setVerbose(params.verbose);
    filesystem::create_directory(filesystem::path(params.output_dir));

    unsigned int nSoupFamilies = gran_sys.getNumTriangleFamilies();
    cout << nSoupFamilies << " soup families" << endl;
    double* meshPosRot = new double[7 * nSoupFamilies];
    float* meshVel = new float[6 * nSoupFamilies]();

    gran_sys.initialize();

    // Create rigid ball_body simulation
    ChSystemSMC sys_ball;
    sys_ball.SetContactForceModel(ChSystemSMC::ContactForceModel::Hooke);
    sys_ball.SetTimestepperType(ChTimestepper::Type::EULER_EXPLICIT);
    sys_ball.Set_G_acc(ChVector<>(0, 0, -980));

    double inertia = 2.0 / 5.0 * ball_mass * ball_radius * ball_radius;
    ChVector<> ball_initial_pos(0, 0, fill_top + ball_radius + 2 * params.sphere_radius);

    std::shared_ptr<ChBody> ball_body(sys_ball.NewBody());
    ball_body->SetMass(ball_mass);
    ball_body->SetInertiaXX(ChVector<>(inertia, inertia, inertia));
    ball_body->SetPos(ball_initial_pos);
    sys_ball.AddBody(ball_body);
    unsigned int out_fps = 50;
    cout << "Rendering at " << out_fps << "FPS" << endl;

    unsigned int out_steps = 1 / (out_fps * iteration_step);

    int currframe = 0;
    unsigned int curr_step = 0;

    clock_t start = std::clock();
    for (float t = 0; t < params.time_end; t += iteration_step, curr_step++) {
        auto ball_pos = ball_body->GetPos();
        auto ball_rot = ball_body->GetRot();

        auto ball_vel = ball_body->GetPos_dt();
        auto ball_ang_vel = ball_body->GetWvel_loc();
        ball_ang_vel = ball_body->GetRot().GetInverse().Rotate(ball_ang_vel);

        meshPosRot[0] = ball_pos.x();
        meshPosRot[1] = ball_pos.y();
        meshPosRot[2] = ball_pos.z();
        meshPosRot[3] = ball_rot[0];
        meshPosRot[4] = ball_rot[1];
        meshPosRot[5] = ball_rot[2];
        meshPosRot[6] = ball_rot[3];

        meshVel[0] = ball_vel.x();
        meshVel[1] = ball_vel.y();
        meshVel[2] = ball_vel.z();
        meshVel[3] = ball_ang_vel.x();
        meshVel[4] = ball_ang_vel.y();
        meshVel[5] = ball_ang_vel.z();

        gran_sys.meshSoup_applyRigidBodyMotion(meshPosRot, meshVel);

        gran_sys.advance_simulation(iteration_step);
        sys_ball.DoStepDynamics(iteration_step);

        float ball_force[6];
        gran_sys.collectGeneralizedForcesOnMeshSoup(ball_force);

        ball_body->Empty_forces_accumulators();
        ball_body->Accumulate_force(ChVector<>(ball_force[0], ball_force[1], ball_force[2]), ball_pos, false);
        ball_body->Accumulate_torque(ChVector<>(ball_force[3], ball_force[4], ball_force[5]), false);

        if (curr_step % out_steps == 0) {
            cout << "Rendering frame " << currframe << endl;
            char filename[100];
            sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
            gran_sys.writeFile(string(filename));

            string mesh_output = string(filename) + "_meshframes.csv";
            std::ofstream meshfile(mesh_output);
            std::ostringstream outstream;
            outstream << "mesh_name,dx,dy,dz,x1,x2,x3,y1,y2,y3,z1,z2,z3,sx,sy,sz\n";
            writeMeshFrames(outstream, *ball_body, mesh_filename, ball_radius);
            meshfile << outstream.str();
        }
    }

    clock_t end = std::clock();
    double total_time = ((double)(end - start)) / CLOCKS_PER_SEC;

    cout << "Time: " << total_time << " seconds" << endl;

    delete[] meshPosRot;
    delete[] meshVel;

    return 0;
}