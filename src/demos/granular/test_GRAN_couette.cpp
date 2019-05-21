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

enum TEST_MODE { SINGLE = 0, DOUBLE = 1 };

void ShowUsage() {
    cout << "usage: ./test_GRAN_couette <json_file> <output_dir> <test: 0-single drum, 1-double drum> <material_height>"
         << endl;
}

void writeMeshFrames(std::ostringstream& outstream,
                     const string obj_name,
                     const ChVector<>& pos,
                     const float3 mesh_scaling,
                     const double angle = 0.0) {
    outstream << obj_name << ",";

    // Get basis vectors
    auto q = Q_from_AngZ(angle);
    ChVector<> vx(1, 0, 0);
    vx = q.Rotate(vx);
    ChVector<> vy(0, 1, 0);
    vy = q.Rotate(vy);
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
    if (argc != 5 || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }

    TEST_MODE test_mode = (TEST_MODE)stoi(argv[3]);
    switch (test_mode) {
        case SINGLE:
            cout << "Single-Cylinder test" << endl;
            break;
        case DOUBLE:
            cout << "Double-Cylinder test" << endl;
            break;
        default:
            cout << "Invalid test" << endl;
    }
    const double R1 = 6.5;                         // 65 mm
    const double R2 = 9.0;                         // 90 mm
    const double omega = 0.15;                     // 0.15 rad/s
    const double sphere_radius = 0.035 / 2.0;      // 0.35mm diameter
    const float friction = 0.9;                    // Glass static friction
    const double material_height = stof(argv[4]);  // 10,20,30,40,50 mm
    const double extra_cyl_height = 1.0;
    const double padding = sphere_radius * 2.0;
    const double cyl_height = material_height + extra_cyl_height;

    float iteration_step = params.step_size;
    const float Bx = R2 * 2.05;
    const float By = Bx;
    const float Bz = padding + 2.0 * material_height + padding;
    cout << "Box Dims: " << Bx << " " << By << " " << Bz << endl;

    const double cyl_z = cyl_height / 2.0 - Bz / 2.0 + padding;
    const double cyl_bottom = padding - Bz / 2.0;

    const float fill_bottom = cyl_bottom + 2.0 * sphere_radius;
    const float fill_height = 2.0 * material_height;  // TODO tune this

    ChSystemGranularSMC_trimesh m_sys(sphere_radius, params.sphere_density, make_float3(Bx, By, Bz));

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

    m_sys.set_friction_mode(GRAN_FRICTION_MODE::MULTI_STEP);
    m_sys.set_static_friction_coeff_SPH2SPH(friction);
    m_sys.set_static_friction_coeff_SPH2SPH(friction);
    m_sys.set_static_friction_coeff_SPH2MESH(friction);
    m_sys.set_rolling_mode(GRAN_ROLLING_MODE::CONSTANT_TORQUE);
    m_sys.set_rolling_coeff_SPH2SPH(friction);
    m_sys.set_rolling_coeff_SPH2SPH(friction);
    m_sys.set_rolling_coeff_SPH2MESH(friction);

    m_sys.setOutputMode(params.write_mode);

    string output_dir(argv[2]);
    m_sys.setOutputDirectory(output_dir);
    filesystem::create_directory(filesystem::path(output_dir));

    m_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::CENTERED_DIFFERENCE);
    m_sys.set_fixed_stepSize(params.step_size);
    m_sys.set_BD_Fixed(true);
    m_sys.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);

    // utils::PDSampler<float> sampler(2.05 * sphere_radius);
    utils::HCPSampler<float> sampler(2.05 * sphere_radius);
    vector<ChVector<float>> body_points;

    const float fill_radius = R2 - 2.f * sphere_radius - 0.02 * R2;
    const float fill_top = fill_bottom + fill_height;
    cout << "Fill radius " << fill_radius << endl;
    cout << "Fill bottom " << fill_bottom << endl;
    cout << "Fill top " << fill_top << endl;

    ChVector<float> center(0, 0, fill_bottom);
    center.z() += 2 * sphere_radius;
    while (center.z() < fill_top - 2 * sphere_radius) {
        auto points = sampler.SampleCylinderZ(center, fill_radius, 0);
        body_points.insert(body_points.end(), points.begin(), points.end());
        center.z() += 2.05 * sphere_radius;
    }

    vector<ChVector<float>> final_points;
    switch (test_mode) {
        case SINGLE:
            final_points = body_points;
            break;
        case DOUBLE:
            for (auto pos : body_points) {
                if (std::sqrt(pos.x() * pos.x() + pos.y() * pos.y()) > R1 + 2 * sphere_radius + 0.02 * R1) {
                    final_points.push_back(pos);
                }
            }
            break;
    }

    unsigned int n_spheres = final_points.size();
    m_sys.setParticlePositions(final_points);

    vector<string> mesh_filenames;
    vector<float3> mesh_scalings;
    vector<float> mesh_masses;
    vector<bool> mesh_inflated;
    vector<float> mesh_inflation_radii;

    const float mass = 10;

    float cyl_center[3] = {0, 0, 0};
    const float3 inner_scaling = make_float3(R1, R1, cyl_height);
    const float3 outer_scaling = make_float3(R2, R2, cyl_height);

    string inner_filename("granular/couette/inner_cylinder.obj");
    string outer_filename("granular/couette/outer_cylinder.obj");

    if (test_mode == DOUBLE) {
        mesh_filenames.push_back(inner_filename);
        mesh_scalings.push_back(inner_scaling);
        mesh_masses.push_back(mass);
        mesh_inflated.push_back(false);
        mesh_inflation_radii.push_back(0);
    }

    mesh_filenames.push_back(outer_filename);
    mesh_scalings.push_back(outer_scaling);
    mesh_masses.push_back(mass);
    mesh_inflated.push_back(false);
    mesh_inflation_radii.push_back(0);

    m_sys.load_meshes(mesh_filenames, mesh_scalings, mesh_masses, mesh_inflated, mesh_inflation_radii);

    m_sys.initialize();

    const double max_rotation = CH_C_PI / 4.0;  // Run a quarter of a rotation
    const double time_settling = 1.0;
    const double time_spinning = max_rotation / omega;

    cout << "Time settling " << time_settling << endl;
    cout << "Time spinning " << time_spinning << endl;

    unsigned int nSoupFamilies;
    unsigned int outer_i;
    if (test_mode == DOUBLE) {
        nSoupFamilies = 2;
        outer_i = 1;
    } else {
        nSoupFamilies = 1;
        outer_i = 0;
    }
    cout << nSoupFamilies << " soup families" << endl;
    double* meshPosRot = new double[7 * nSoupFamilies];
    float* meshVel = new float[6 * nSoupFamilies]();

    // Set initial mesh locations for the settling phase
    for (int i = 0; i < nSoupFamilies; i++) {
        meshPosRot[7 * i + 0] = 0;
        meshPosRot[7 * i + 1] = 0;
        meshPosRot[7 * i + 2] = 0;
        meshPosRot[7 * i + 3] = 1;
        meshPosRot[7 * i + 4] = 0;
        meshPosRot[7 * i + 5] = 0;
        meshPosRot[7 * i + 6] = 0;
    }
    m_sys.meshSoup_applyRigidBodyMotion(meshPosRot, meshVel);

    unsigned int currframe = 0;
    double out_fps = 100;
    float frame_step = 1.f / out_fps;  // Duration of a frame
    unsigned int out_steps = frame_step / iteration_step;
    cout << "Writing at " << out_fps << " FPS" << endl;
    cout << "out_steps " << out_steps << endl;

    unsigned int step = 0;
    bool settled = false;
    ChVector<> pos_mesh(0, 0, cyl_z);
    cout << "Settling..." << endl;
    for (float t = 0; t < time_settling + time_spinning; t += iteration_step, step++) {
        if (t >= time_settling) {
            // Spinning phase
            if (!settled) {
                cout << "Spinning..." << endl;
                settled = true;
                meshVel[outer_i * 6 + 5] = omega;
            }

            auto q = Q_from_AngZ(omega * (t - time_settling));
            meshPosRot[outer_i * 7 + 3] = q[0];
            meshPosRot[outer_i * 7 + 4] = q[1];
            meshPosRot[outer_i * 7 + 5] = q[2];
            meshPosRot[outer_i * 7 + 6] = q[3];
            m_sys.meshSoup_applyRigidBodyMotion(meshPosRot, meshVel);
        }

        if (step % out_steps == 0) {
            cout << "Rendering frame " << currframe << endl;
            char filename[100];
            sprintf(filename, "%s/step%06u", output_dir.c_str(), currframe++);
            m_sys.writeFile(string(filename), true);
            // m_sys.write_meshes(string(filename));
            string mesh_output = string(filename) + "_meshframes.csv";

            std::ofstream meshfile(mesh_output);
            std::ostringstream outstream;
            outstream << "mesh_name,dx,dy,dz,x1,x2,x3,y1,y2,y3,z1,z2,z3,sx,sy,sz\n";

            if (test_mode == DOUBLE) {
                writeMeshFrames(outstream, inner_filename, pos_mesh, inner_scaling);
            }

            double angle = (settled) ? omega * (t - time_settling) : 0;
            writeMeshFrames(outstream, outer_filename, pos_mesh, outer_scaling, angle);

            meshfile << outstream.str();
            meshfile.close();
        }

        m_sys.advance_simulation(iteration_step);
    }

    delete[] meshPosRot;
    return 0;
}