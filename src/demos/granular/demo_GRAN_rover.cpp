// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nic Olsen, Conlain Kelly
// =============================================================================
//
// Simplified rover in Chrono::Granular.
// =============================================================================
/*! \file */

#include <iostream>
#include <vector>
#include <string>
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono/core/ChTimer.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChForce.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/physics/ChGranularTriMesh.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"

#include "ChGranular_json_parser.hpp"
#include "ChGranularDemoUtils.hpp"

using namespace chrono;
using namespace chrono::granular;

using std::cout;
using std::endl;

constexpr double METERS_TO_CM = 100;
constexpr double KG_TO_GRAM = 1000;

constexpr double wheel_rad = 0.1 * METERS_TO_CM;
constexpr double wheel_height = 0.1 * METERS_TO_CM;

constexpr double ROVER_MASS_REDUCTION = 1.;

constexpr double wheel_mass = ROVER_MASS_REDUCTION * 3 * KG_TO_GRAM;
constexpr double chassis_mass = ROVER_MASS_REDUCTION * 100 * KG_TO_GRAM;

// distance wheels are in front of / behind chassis COM
constexpr double wheel_offset_x = 0.45 * METERS_TO_CM;
// distance wheels are to left/right of chassis COM
constexpr double wheel_offset_y = 0.35 * METERS_TO_CM;
// distance wheels are below chassis COM
constexpr double wheel_offset_z = -0.35 * METERS_TO_CM;
// distance wheels are in front of / behind chassis COM
constexpr double chassis_length_x = 2.5 * wheel_offset_x;
// distance wheels are to left/right of chassis COM
constexpr double chassis_length_y = 2.5 * wheel_offset_y;
// distance wheels are below chassis COM
constexpr double chassis_length_z = 2.5 * wheel_offset_z;

constexpr float3 wheel_scaling = {wheel_rad * 2, wheel_height, wheel_rad * 2};

constexpr double wheel_inertia_x = (1. / 4.) * wheel_mass * wheel_rad * wheel_rad + (1 / 12.) * wheel_mass;
constexpr double wheel_inertia_y = (1. / 2.) * wheel_mass * wheel_rad * wheel_rad;
constexpr double wheel_inertia_z = wheel_inertia_x;

std::string chassis_filename = "BD_Box.obj";

enum ROVER_BODY_ID { WHEEL_FRONT_LEFT, WHEEL_FRONT_RIGHT, WHEEL_REAR_LEFT, WHEEL_REAR_RIGHT };

std::vector<std::shared_ptr<chrono::ChBody>> wheel_bodies;

std::vector<string> mesh_filenames;
std::vector<float3> mesh_scalings;
std::vector<float> mesh_masses;
std::vector<bool> mesh_inflated;
std::vector<float> mesh_inflation_radii;

// y is height, x and z are radial
// starts as height=1, diameter = 1

std::string wheel_filename = std::string("granular/rover/wheel_scaled.obj");

void ShowUsage() {
    cout << "usage: ./demo_GRAN_rover <json_file>" << endl;
}

void addWheelBody(ChSystemNSC& rover_sys, std::shared_ptr<ChBody> chassis_body, const ChVector<>& wheel_initial_pos) {
    std::shared_ptr<ChBody> wheel_body(rover_sys.NewBody());

    // bool wheel_fixed = true;
    // wheel_body->SetBodyFixed(wheel_fixed);
    wheel_body->SetMass(wheel_mass);
    wheel_body->SetBodyFixed(false);
    // assume it's a cylinder inertially
    wheel_body->SetInertiaXX(ChVector<>(wheel_inertia_x, wheel_inertia_y, wheel_inertia_z));

    printf("Inertia tensor is %f, %f, %f\n", wheel_inertia_x, wheel_inertia_y, wheel_inertia_z);
    wheel_body->SetPos(wheel_initial_pos);
    // wheel_body->SetPos_dt(ChVector<>(50, 0, 0));
    rover_sys.AddBody(wheel_body);

    auto joint = std::make_shared<ChLinkLockRevolute>();
    joint->Initialize(chassis_body, wheel_body, ChCoordsys<>(wheel_initial_pos, Q_from_AngX(CH_C_PI / 2)));
    rover_sys.AddLink(joint);

    auto motor = std::make_shared<ChLinkMotorRotationAngle>();

    motor->Initialize(chassis_body, wheel_body, ChFrame<>(wheel_initial_pos, Q_from_AngX(CH_C_PI / 2)));

    motor->SetMotorFunction(std::make_shared<ChFunction_Ramp>(0, CH_C_PI));
    rover_sys.AddLink(motor);

    mesh_masses.push_back(wheel_mass);
    mesh_inflated.push_back(false);
    mesh_inflation_radii.push_back(0);
    mesh_scalings.push_back(wheel_scaling);
    mesh_filenames.push_back(wheel_filename);
    wheel_bodies.push_back(wheel_body);
}

void writeMeshFrames(std::ostringstream& outstream,
                     std::shared_ptr<ChBody> body,
                     std::string obj_name,
                     float3 mesh_scaling) {
    outstream << obj_name << ",";

    // Get frame position
    ChFrame<> body_frame = body->GetFrame_REF_to_abs();
    ChQuaternion<> rot = body_frame.GetRot();
    ChVector<> pos = body_frame.GetPos();

    // Get basis vectors
    ChVector<> vx = rot.GetXaxis();
    ChVector<> vy = rot.GetYaxis();
    ChVector<> vz = rot.GetZaxis();

    printf("Rot is (%f, %f, %f, %f)\n", rot.e0(), rot.e1(), rot.e2(), rot.e3());
    // normalize basis vectors
    vx = vx / vx.Length();
    vy = vy / vy.Length();
    vz = vz / vz.Length();

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

    // printf("wheel scaling is %f, %f, %f\n", mesh_scaling.x, mesh_scaling.y, mesh_scaling.z);
    outstream << mesh_scaling.x << "," << mesh_scaling.y << "," << mesh_scaling.z;
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
    ChSystemGranular_MonodisperseSMC_trimesh gran_sys(params.sphere_radius, params.sphere_density,
                                                      make_float3(params.box_X, params.box_Y, params.box_Z));

    double fill_bottom = -params.box_Z / 2.0;
    double fill_top = 0;

    // leave a 4cm margin at edges of sampling
    ChVector<> hdims(params.box_X / 2 - 2.0, params.box_Y / 2 - 2.0, abs((fill_bottom - fill_top) / 2.) - 2.0);
    ChVector<> center(0, 0, (fill_bottom + fill_top) / 2.);
    std::vector<ChVector<float>> body_points;

    body_points = PDLayerSampler_BOX<float>(center, hdims, 2. * params.sphere_radius, 1.01);

    std::vector<ChVector<float>> first_points;

    utils::HCPSampler<float> sampler(2.3 * params.sphere_radius);

    double sample_cyl_rad = 10 * params.sphere_radius;

    ChVector<> sample_cyl_center(params.box_X / 4, 0, params.box_Z / 2 - sample_cyl_rad - 3 * params.sphere_radius);
    auto cyl_points =
        sampler.SampleCylinderY(sample_cyl_center, sample_cyl_rad, params.box_Y / 2 - 3 * params.sphere_radius);
    body_points.insert(body_points.end(), cyl_points.begin(), cyl_points.end());

    first_points.push_back(body_points.at(0));
    gran_sys.setParticlePositions(body_points);

    gran_sys.set_BD_Fixed(true);

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
    gran_sys.set_timeStepping(GRAN_TIME_STEPPING::FIXED);
    gran_sys.set_fixed_stepSize(params.step_size);
    gran_sys.set_friction_mode(GRAN_FRICTION_MODE::MULTI_STEP);
    gran_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::CENTERED_DIFFERENCE);
    gran_sys.set_static_friction_coeff(params.static_friction_coeff);

    // Create rigid wheel simulation
    ChSystemNSC rover_sys;

    rover_sys.SetMaxItersSolverSpeed(200);
    if (rover_sys.GetContactMethod() == ChMaterialSurface::NSC) {
        rover_sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    }

    // rover_sys.SetContactForceModel(ChSystemNSC::ContactForceModel::Hooke);
    // rover_sys.SetTimestepperType(ChTimestepper::Type::EULER_EXPLICIT);
    rover_sys.Set_G_acc(ChVector<>(0, 0, -980));
    // rover_sys.Set_G_acc(ChVector<>(0, 0, 0));

    std::shared_ptr<ChBody> chassis_body(rover_sys.NewBody());

    double init_height_boost = params.box_Z + 2 * wheel_rad;
    double init_offset_x = -params.box_X / 4;

    // bool wheel_fixed = true;
    // chassis_body->SetBodyFixed(wheel_fixed);
    chassis_body->SetMass(chassis_mass);
    // assume it's a solid box inertially
    chassis_body->SetInertiaXX(
        ChVector<>((chassis_length_y * chassis_length_y + chassis_length_z * chassis_length_z) * chassis_mass / 12,
                   (chassis_length_x * chassis_length_x + chassis_length_z * chassis_length_z) * chassis_mass / 12,
                   (chassis_length_x * chassis_length_x + chassis_length_y * chassis_length_y) * chassis_mass / 12));
    chassis_body->SetPos(ChVector<>(init_offset_x, 0, init_height_boost));
    rover_sys.AddBody(chassis_body);

    chassis_body->SetBodyFixed(false);

    // NOTE these must happen before the gran system loads meshes!!!
    addWheelBody(rover_sys, chassis_body,
                 chassis_body->GetPos() + ChVector<>(wheel_offset_x, wheel_offset_y, wheel_offset_z));
    addWheelBody(rover_sys, chassis_body,
                 chassis_body->GetPos() + ChVector<>(wheel_offset_x, -wheel_offset_y, wheel_offset_z));
    addWheelBody(rover_sys, chassis_body,
                 chassis_body->GetPos() + ChVector<>(-wheel_offset_x, wheel_offset_y, wheel_offset_z));
    addWheelBody(rover_sys, chassis_body,
                 chassis_body->GetPos() + ChVector<>(-wheel_offset_x, -wheel_offset_y, wheel_offset_z));

    gran_sys.load_meshes(mesh_filenames, mesh_scalings, mesh_masses, mesh_inflated, mesh_inflation_radii);

    gran_sys.setOutputDirectory(params.output_dir);
    gran_sys.setOutputMode(params.write_mode);
    gran_sys.setVerbose(params.verbose);
    filesystem::create_directory(filesystem::path(params.output_dir));

    unsigned int nSoupFamilies = gran_sys.nMeshesInSoup();
    cout << nSoupFamilies << " soup families" << endl;
    double* meshPosRot = new double[7 * nSoupFamilies];
    float* meshVel = new float[6 * nSoupFamilies]();

    gran_sys.initialize();

    unsigned int out_fps = 50;
    cout << "Rendering at " << out_fps << "FPS" << endl;

    unsigned int out_steps = 1 / (out_fps * iteration_step);

    int currframe = 0;
    unsigned int curr_step = 0;

    gran_sys.enableMeshCollision();

    clock_t start = std::clock();
    for (float t = 0; t < params.time_end; t += iteration_step, curr_step++) {
        // if (wheel_fixed && t >= 0.5) {
        //     printf("Setting wheel free!\n");
        //     wheel_fixed = false;
        //     // wheel_body->SetBodyFixed(false);
        //     // gran_sys.enableMeshCollision();
        //     // wheel_pos.z() = 0 + 3 * params.sphere_radius;
        // }
        for (unsigned int i = 0; i < wheel_bodies.size(); i++) {
            auto curr_body = wheel_bodies.at(i);

            auto wheel_pos = curr_body->GetPos();
            auto wheel_rot = curr_body->GetRot();

            auto wheel_vel = curr_body->GetPos_dt();
            auto wheel_ang_vel = curr_body->GetWvel_loc();
            wheel_ang_vel = curr_body->GetRot().GetInverse().Rotate(wheel_ang_vel);

            meshPosRot[7 * i + 0] = wheel_pos.x();
            meshPosRot[7 * i + 1] = wheel_pos.y();
            meshPosRot[7 * i + 2] = wheel_pos.z();
            meshPosRot[7 * i + 3] = wheel_rot[0];
            meshPosRot[7 * i + 4] = wheel_rot[1];
            meshPosRot[7 * i + 5] = wheel_rot[2];
            meshPosRot[7 * i + 6] = wheel_rot[3];

            meshVel[6 * i + 0] = wheel_vel.x();
            meshVel[6 * i + 1] = wheel_vel.y();
            meshVel[6 * i + 2] = wheel_vel.z();
            meshVel[6 * i + 3] = wheel_ang_vel.x();
            meshVel[6 * i + 4] = wheel_ang_vel.y();
            meshVel[6 * i + 5] = wheel_ang_vel.z();
        }

        gran_sys.meshSoup_applyRigidBodyMotion(meshPosRot, meshVel);

        gran_sys.advance_simulation(iteration_step);
        rover_sys.DoStepDynamics(iteration_step);

        float wheel_force[6 * wheel_bodies.size()];
        gran_sys.collectGeneralizedForcesOnMeshSoup(wheel_force);

        for (unsigned int i = 0; i < wheel_bodies.size(); i++) {
            auto curr_body = wheel_bodies.at(i);
            auto wheel_pos = curr_body->GetPos();

            curr_body->Empty_forces_accumulators();
            curr_body->Accumulate_force(
                ChVector<>(wheel_force[6 * i + 0], wheel_force[6 * i + 1], wheel_force[6 * i + 2]), wheel_pos, false);
            curr_body->Accumulate_torque(
                ChVector<>(wheel_force[6 * i + 3], wheel_force[6 * i + 4], wheel_force[6 * i + 5]), false);
        }

        if (curr_step % out_steps == 0) {
            cout << "Rendering frame " << currframe << endl;
            printf("Wheel forces: %f, %f, %f\n", wheel_force[0], wheel_force[1], wheel_force[2]);
            printf("Wheel torques: %f, %f, %f\n", wheel_force[3], wheel_force[4], wheel_force[5]);
            char filename[100];
            sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
            gran_sys.writeFile(std::string(filename));
            std::string mesh_output = std::string(filename) + "_meshframes.csv";
            std::ofstream meshfile(mesh_output);
            std::ostringstream outstream;
            outstream << "mesh_name,dx,dy,dz,x1,x2,x3,y1,y2,y3,z1,z2,z3,sx,sy,sz\n";
            // if the wheel is free, output its mesh, otherwise leave file empty
            // if (!wheel_fixed) {
            for (unsigned int i = 0; i < wheel_bodies.size(); i++) {
                writeMeshFrames(outstream, wheel_bodies.at(i), wheel_filename, wheel_scaling);
            }

            writeMeshFrames(outstream, chassis_body, chassis_filename,
                            {chassis_length_x / 2, chassis_length_y / 2, chassis_length_z / 4});

            meshfile << outstream.str();
            // }
        }
    }

    clock_t end = std::clock();
    double total_time = ((double)(end - start)) / CLOCKS_PER_SEC;

    cout << "Time: " << total_time << " seconds" << endl;

    delete[] meshPosRot;
    delete[] meshVel;

    return 0;
}