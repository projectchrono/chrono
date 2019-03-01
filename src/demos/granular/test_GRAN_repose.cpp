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
    cout << "usage: ./test_GRAN_repose <json_file> <output_dir> <static_friction>" << endl;
}

void writeMeshFrames(std::ostringstream& outstream,
                     const string obj_name,
                     const ChVector<>& pos,
                     const float3 mesh_scaling) {
    outstream << obj_name << ",";

    // Get basis vectors
    ChVector<> vx(1, 0, 0);
    ChVector<> vy(0, 1, 0);
    ChVector<> vz(0, 0, 1);

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
    outstream << mesh_scaling.x << "," << mesh_scaling.y << "," << mesh_scaling.z;
    outstream << "\n";
}

int main(int argc, char* argv[]) {
    sim_param_holder params;

    // Some of the default values might be overwritten by user via command line
    if (argc != 4 || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }

    float iteration_step = params.step_size;
    const float Bx = params.box_X;
    const float By = Bx;

    const float fill_height = 40;
    const float chamber_height = fill_height / 2.f;
    const float extra_height = 0;

    const float Bz = chamber_height + fill_height + extra_height;
    cout << "Box Dims: " << Bx << " " << By << " " << Bz << endl;

    ChSystemGranular_MonodisperseSMC_trimesh m_sys(params.sphere_radius, params.sphere_density,
                                                   make_float3(Bx, By, Bz));

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

    m_sys.setOutputMode(params.write_mode);
    string out_dir(argv[2]);
    m_sys.setOutputDirectory(out_dir);
    filesystem::create_directory(filesystem::path(out_dir));

    m_sys.set_timeStepping(GRAN_TIME_STEPPING::FIXED);
    m_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::CENTERED_DIFFERENCE);
    m_sys.set_fixed_stepSize(params.step_size);
    m_sys.set_BD_Fixed(true);
    m_sys.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);

    float static_friction = std::stof(argv[3]);
    cout << "Static Friction: " << static_friction << endl;
    m_sys.set_static_friction_coeff(static_friction);

    const float chamber_bottom = -Bz / 2.f;
    const float fill_bottom = chamber_bottom + chamber_height;

    float cyl_center[3] = {0, 0, 0};
    const float cyl_rad = Bx / 2.f - params.sphere_radius;
    cout << "Cylinder radius: " << cyl_rad << endl;
    m_sys.Create_BC_Cyl_Z(cyl_center, cyl_rad, false, false);

    utils::PDSampler<float> sampler(2.5 * params.sphere_radius);
    vector<ChVector<float>> body_points;

    const float fill_radius = cyl_rad - 2.f * params.sphere_radius;
    cout << "Fill radius " << fill_radius << endl;

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
    const float dish_height = scale * 0.1;   // Assumes obj height is 0.1
    float3 scaling_dish = make_float3(scale + dish_inflation, scale + dish_inflation, scale);
    float3 scaling_ledge = make_float3(scale, scale, scale);
    mesh_scalings.push_back(scaling_dish);
    mesh_scalings.push_back(scaling_ledge);

    mesh_masses.push_back(mass);
    mesh_masses.push_back(mass);

    vector<bool> mesh_inflated;
    mesh_inflated.push_back(false);
    mesh_inflated.push_back(false);

    vector<float> mesh_inflation_radii;
    mesh_inflation_radii.push_back(0);
    mesh_inflation_radii.push_back(0);

    m_sys.load_meshes(mesh_filenames, mesh_scalings, mesh_masses, mesh_inflated, mesh_inflation_radii);

    unsigned int nSoupFamilies = 2;
    cout << nSoupFamilies << " soup families" << endl;
    double* meshPosRot = new double[7 * nSoupFamilies];
    float* meshVel = new float[6 * nSoupFamilies]();

    m_sys.initialize();

    const double time_settling = 0.5;
    const double lower_vel = 2.0;
    const double lower_dist = 1.0;
    const double time_lowering = lower_dist / lower_vel;

    cout << "Time settling " << time_settling << endl;
    cout << "Lower velocity " << lower_vel << endl;
    cout << "Lower distance " << lower_dist << endl;
    cout << "Time lowering " << time_lowering << endl;

    const double dish_z = fill_bottom - dish_height / 2.0;  // Assumes height of dish is 0.1 in obj
    double ledge_z = fill_bottom - dish_height;

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
    const ChVector<> pos_dish(0, 0, dish_z);
    ChVector<> pos_ledge(0, 0, ledge_z);

    cout << "Settling..." << endl;
    for (float t = 0; t < params.time_end; t += iteration_step, step++) {
        if (t >= time_settling && t <= time_settling + time_lowering) {
            // Lowering phase
            if (!settled) {
                cout << "Lowering..." << endl;
                settled = true;
            }
            ledge_z -= iteration_step * lower_vel;
            meshPosRot[ledge_i * pos_per_fam + 2] = ledge_z;
            pos_ledge.z() = ledge_z;
            meshVel[ledge_i * vel_per_fam + 2] = -lower_vel;
            m_sys.meshSoup_applyRigidBodyMotion(meshPosRot, meshVel);
        } else if (t > time_settling + time_lowering && !lowered) {
            // Draining phase
            lowered = true;
            meshVel[ledge_i * vel_per_fam + 2] = 0;
            m_sys.meshSoup_applyRigidBodyMotion(meshPosRot, meshVel);
        }

        if (step % out_steps == 0) {
            cout << "Rendering frame " << currframe << endl;
            char filename[100];
            sprintf(filename, "%s/step%06u", out_dir.c_str(), currframe++);
            m_sys.writeFile(string(filename));

            string mesh_output = string(filename) + "_meshframes.csv";

            std::ofstream meshfile(mesh_output);
            std::ostringstream outstream;
            outstream << "mesh_name,dx,dy,dz,x1,x2,x3,y1,y2,y3,z1,z2,z3,sx,sy,sz\n";

            writeMeshFrames(outstream, mesh_filename_dish, pos_dish, scaling_dish);
            writeMeshFrames(outstream, mesh_filename_ledge, pos_ledge, scaling_ledge);

            meshfile << outstream.str();
            meshfile.close();
        }

        m_sys.advance_simulation(iteration_step);
    }

    delete[] meshPosRot;

    return 0;
}