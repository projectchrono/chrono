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
// Authors: Nic Olsen
// =============================================================================
/*! \file */

#include <iostream>
#include <string>
#include <cmath>
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/physics/ChGranularTriMesh.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "ChGranular_json_parser.hpp"

using namespace chrono;
using namespace chrono::granular;

using std::cout;
using std::endl;
using std::string;
using std::stof;
using std::stoi;
using std::vector;

void ShowUsage() {
    cout << "usage: ./test_GRAN_repose <json_file>" << endl;
}

int main(int argc, char* argv[]) {
    sim_param_holder params;

    // Some of the default values might be overwritten by user via command line
    if (argc != 2 || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }

    float iteration_step = params.step_size;

    // Setup simulation
    ChSystemGranular_MonodisperseSMC_trimesh m_sys(params.sphere_radius, params.sphere_density);

    m_sys.set_K_n_SPH2SPH(params.normalStiffS2S);
    m_sys.set_K_n_SPH2WALL(params.normalStiffS2W);
    m_sys.set_K_n_SPH2MESH(params.normalStiffS2M);

    m_sys.set_K_t_SPH2SPH(params.tangentStiffS2S);
    m_sys.set_K_t_SPH2WALL(params.tangentStiffS2W);
    m_sys.set_K_t_SPH2MESH(params.tangentStiffS2M);

    m_sys.set_Gamma_n_SPH2SPH(params.normalDampS2S);
    m_sys.set_Gamma_n_SPH2WALL(params.normalDampS2W);
    m_sys.set_Gamma_n_SPH2MESH(params.normalDampS2M);

    m_sys.set_Gamma_t_SPH2SPH(params.tangentDampS2S);
    m_sys.set_Gamma_t_SPH2WALL(params.tangentDampS2W);
    m_sys.set_Gamma_t_SPH2MESH(params.tangentDampS2M);

    m_sys.set_Cohesion_ratio(params.cohesion_ratio);
    m_sys.set_Adhesion_ratio_S2M(params.adhesion_ratio_s2m);
    m_sys.set_Adhesion_ratio_S2W(params.adhesion_ratio_s2w);
    m_sys.set_friction_mode(chrono::granular::GRAN_FRICTION_MODE::MULTI_STEP);

    m_sys.setOutputMode(GRAN_OUTPUT_MODE::CSV);
    m_sys.setOutputDirectory(params.output_dir);
    filesystem::create_directory(filesystem::path(params.output_dir));

    m_sys.set_timeStepping(GRAN_TIME_STEPPING::FIXED);
    m_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::FORWARD_EULER);
    m_sys.set_fixed_stepSize(params.step_size);
    m_sys.set_BD_Fixed(true);
    m_sys.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);

    const float static_friction = 0.5;  // TODO
    m_sys.set_static_friction_coeff(static_friction);
    cout << "Static Friction: " << static_friction << endl;

    const float Bx = params.box_X;
    const float By = Bx;

    const float chamber_height = Bx;  // TODO
    const float fill_height = 50;
    const float extra_height = 0;

    const float Bz = chamber_height + fill_height + extra_height;
    m_sys.setBOXdims(Bx, By, Bz);
    cout << "Box Dims: " << Bx << " " << By << " " << Bz << endl;

    const float chamber_bottom = -Bz / 2.f;
    const float fill_bottom = chamber_bottom + chamber_height;

    float cyl_center[3] = {0, 0, 0};
    const float cyl_rad = Bx / 2.f;
    m_sys.Create_BC_Cyl_Z(cyl_center, cyl_rad, false, false);

    utils::PDSampler<float> sampler(2.5 * params.sphere_radius);
    // utils::HCPSampler<float> sampler(2.01 * params.sphere_radius);
    vector<ChVector<float>> body_points;

    const float fill_radius = Bx / 2.f - 2.f * params.sphere_radius;
    const float fill_top = fill_bottom + fill_height;
    ChVector<float> center(0, 0, fill_bottom);
    center.z() += 2 * params.sphere_radius;
    while (center.z() < fill_top - 2 * params.sphere_radius) {
        auto points = sampler.SampleCylinderZ(center, fill_radius, 0);
        body_points.insert(body_points.end(), points.begin(), points.end());
        center.z() += 2.05 * params.sphere_radius;
    }

    unsigned int n_spheres = body_points.size();
    m_sys.setParticlePositions(body_points);

    vector<string> mesh_filenames;
    vector<float3> mesh_scalings;
    vector<float> mesh_masses;
    const float mass = 10;

    const unsigned int dish_i = 0;
    const unsigned int ledge_i = 1;

    string mesh_filename_dish("granular/repose/repose_dish.obj");
    string mesh_filename_ledge("granular/repose/repose_ledge.obj");

    mesh_filenames.push_back(mesh_filename_dish);
    mesh_filenames.push_back(mesh_filename_ledge);

    const float scale = Bx / 2.6;            // Assumes radius of obj ledge is 1.3
    const float dish_inflation = 0.01 * Bx;  // Small inflation of the dish to close gap
    float3 scaling_dish = make_float3(scale + dish_inflation, scale + dish_inflation, scale);
    float3 scaling_ledge = make_float3(scale, scale, scale);
    mesh_scalings.push_back(scaling_dish);
    mesh_scalings.push_back(scaling_ledge);

    mesh_masses.push_back(mass);
    mesh_masses.push_back(mass);

    std::vector<bool> mesh_inflated;
    mesh_inflated.push_back(false);
    mesh_inflated.push_back(false);

    std::vector<float> mesh_inflation_radii;
    mesh_inflation_radii.push_back(0);
    mesh_inflation_radii.push_back(0);

    m_sys.load_meshes(mesh_filenames, mesh_scalings, mesh_masses, mesh_inflated, mesh_inflation_radii);

    unsigned int nSoupFamilies = 2;
    cout << nSoupFamilies << " soup families" << endl;
    double* meshPosRot = new double[7 * nSoupFamilies];
    float* meshVel = new float[6 * nSoupFamilies]();

    m_sys.initialize();

    const double time_settling = 1;
    const double lower_vel = 2;
    const double lower_dist = 10 * params.sphere_radius + 0.1 * scale;  // Assumes height of dish is 0.1 in obj
    const double time_lowering = lower_dist / lower_vel;

    cout << "Time settling " << time_settling << endl;
    cout << "Lower velocity " << lower_vel << endl;
    cout << "Lower distance " << lower_dist << endl;
    cout << "Time lowering " << time_lowering << endl;

    const double dish_offset = 0.1 * params.sphere_radius;
    const double dish_z = fill_bottom - 0.05 * scale - dish_offset;  // Assumes height of dish is 0.1 in obj
    double ledge_z = fill_bottom;
    const unsigned int pos_per_fam = 7;
    const unsigned int vel_per_fam = 6;

    // Set initial mesh locations for the settling phase
    meshPosRot[dish_i * pos_per_fam + 0] = 0;
    meshPosRot[dish_i * pos_per_fam + 1] = 0;
    meshPosRot[dish_i * pos_per_fam + 2] = dish_z;
    meshPosRot[dish_i * pos_per_fam + 3] = 1;
    meshPosRot[dish_i * pos_per_fam + 4] = 0;
    meshPosRot[dish_i * pos_per_fam + 5] = 0;
    meshPosRot[dish_i * pos_per_fam + 6] = 0;

    meshPosRot[ledge_i * pos_per_fam + 0] = 0;
    meshPosRot[ledge_i * pos_per_fam + 1] = 0;
    meshPosRot[ledge_i * pos_per_fam + 2] = ledge_z;
    meshPosRot[ledge_i * pos_per_fam + 3] = 1;
    meshPosRot[ledge_i * pos_per_fam + 4] = 0;
    meshPosRot[ledge_i * pos_per_fam + 5] = 0;
    meshPosRot[ledge_i * pos_per_fam + 6] = 0;

    m_sys.meshSoup_applyRigidBodyMotion(meshPosRot, meshVel);

    unsigned int currframe = 0;
    double out_fps = 60;
    float frame_step = 1.f / out_fps;  // Duration of a frame
    unsigned int out_steps = frame_step / iteration_step;
    cout << "Writing at " << out_fps << " FPS" << endl;
    cout << "out_steps " << out_steps << endl;

    unsigned int step = 0;
    bool lowered = false;
    bool settled = false;
    cout << "Settling..." << endl;
    for (float t = 0; t < params.time_end; t += iteration_step, step++) {
        if (t >= time_settling && t <= time_settling + time_lowering) {
            if (!settled) {
                cout << "Lowering..." << endl;
                settled = true;
            }
            ledge_z -= iteration_step * lower_vel;
            meshPosRot[ledge_i * pos_per_fam + 2] = ledge_z;
            meshVel[ledge_i * vel_per_fam + 2] = -lower_vel;
            m_sys.meshSoup_applyRigidBodyMotion(meshPosRot, meshVel);
        } else if (t > time_settling + time_lowering && !lowered) {
            lowered = true;
            meshVel[ledge_i * vel_per_fam + 2] = 0;
            m_sys.meshSoup_applyRigidBodyMotion(meshPosRot, meshVel);
        }

        if (step % out_steps == 0) {
            cout << "Rendering frame " << currframe << endl;
            char filename[100];
            sprintf(filename, "%s/step%06u", params.output_dir.c_str(), currframe++);
            m_sys.writeFile(string(filename));
            m_sys.write_meshes(string(filename));
        }

        m_sys.advance_simulation(iteration_step);
    }

    delete[] meshPosRot;

    return 0;
}