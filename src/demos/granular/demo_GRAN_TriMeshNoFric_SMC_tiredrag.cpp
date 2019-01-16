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
//
// Chrono::Granular demo using SMC method. A body who's geometry is described
// by a trinagle mesh is initialized under settling granular material. No friction present.
//
// The global reference frame has X to the right, Y into the screen, Z up.
// The global reference frame located in the left lower corner, close to the viewer.
// =============================================================================
/*! \file */

#include <iostream>
#include <string>
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

void ShowUsage() {
    cout << "usage: ./demo_GRAN_TriMeshNoFric_SMC_tiredrag <json_file>" << endl;
}

void rot_func(double t, double rot[4]) {
    double still_time = 0.01;
    double omega = 4;
    auto q = Q_from_AngZ(CH_C_PI_2);
    // TODO figure out rotation
    // if (t >= still_time) {
    //     q += Q_from_AngX(-omega * (t - still_time));
    // }
    rot[0] = q[0];
    rot[1] = q[1];
    rot[2] = q[2];
    rot[3] = q[3];
}

double pos_func_Y(double t, float boxY) {
    double still_time = 0.01;
    double Y_vel = 10;
    if (t < still_time) {
        return -boxY / 4;
    } else {
        return (t - still_time) * Y_vel - boxY / 4;
    }
}

// Remains still for still_time and then begins to move up at Z_vel
double pos_func_Z(double t, float boxZ) {
    double still_time = 0.01;
    double Z_vel = -10;
    if (t < still_time) {
        return 60;
    } else {
        return (t - still_time) * Z_vel + 60;
    }
}

// -----------------------------------------------------------------------------
// Demo for settling a monodisperse collection of shperes in a rectangular box.
// There is no friction. The units are always cm/s/g[L/T/M].
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    float iteration_step = 0.02;

    // Mesh values
    std::vector<string> mesh_filenames;
    string mesh_filename = string("vehicle/hmmwv/hmmwv_tire.obj");

    std::vector<float3> mesh_scalings;
    float3 scaling;
    scaling.x = 30;
    scaling.y = 30;
    scaling.z = 30;
    mesh_scalings.push_back(scaling);

    std::vector<float> mesh_masses;
    float mass = 100;
    mesh_masses.push_back(mass);

    std::vector<bool> mesh_inflated;
    std::vector<float> mesh_inflation_radii;
    mesh_inflated.push_back(false);
    mesh_inflation_radii.push_back(0);


    sim_param_holder params;
    // Some of the default values might be overwritten by user via command line
    if (argc != 2 || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }

    mesh_filenames.push_back(mesh_filename);

    // Setup simulation
    ChSystemGranular_MonodisperseSMC_trimesh m_sys(params.sphere_radius, params.sphere_density);
    m_sys.setPsiFactors(params.psi_T, params.psi_h, params.psi_L);
    m_sys.setBOXdims(params.box_X, params.box_Y, params.box_Z);
    m_sys.set_BD_Fixed(true);
    m_sys.set_K_n_SPH2SPH(params.normalStiffS2S);
    m_sys.set_K_n_SPH2WALL(params.normalStiffS2W);
    m_sys.set_K_n_SPH2MESH(params.normalStiffS2M);
    m_sys.set_Gamma_n_SPH2SPH(params.normalDampS2S);
    m_sys.set_Gamma_n_SPH2MESH(params.normalDampS2M);
    m_sys.set_Cohesion_ratio(params.cohesion_ratio);
    m_sys.set_Adhesion_ratio_S2M(params.adhesion_ratio_s2m);
    m_sys.set_Adhesion_ratio_S2W(params.adhesion_ratio_s2w);
    m_sys.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);
    m_sys.set_timeStepping(GRAN_TIME_STEPPING::FIXED);
    m_sys.set_fixed_stepSize(params.step_size);

    // Fill the bottom half with material
    chrono::utils::HCPSampler<float> sampler(2.4 * params.sphere_radius);  // Add epsilon
    // fill from bottom to 20% up the box
    ChVector<float> center(0, 0, -.15 * params.box_Z);
    ChVector<float> hdims(params.box_X / 2, params.box_X / 2, params.box_Z * .35);
    std::vector<ChVector<float>> body_points = sampler.SampleBox(center, hdims);
    m_sys.setParticlePositions(body_points);

    m_sys.load_meshes(mesh_filenames, mesh_scalings, mesh_masses, mesh_inflated, mesh_inflation_radii);

    /// output preferences
    m_sys.setOutputDirectory(params.output_dir);
    m_sys.setOutputMode(params.write_mode);
    m_sys.setVerbose(params.verbose);
    filesystem::create_directory(filesystem::path(params.output_dir));

    unsigned int nSoupFamilies = m_sys.nMeshesInSoup();
    cout << nSoupFamilies << " soup families" << endl;
    float* genForcesOnMeshSoup = new float[6 * nSoupFamilies];
    double* meshSoupLocOri = new double[7 * nSoupFamilies];
    float* meshVel = new float[6 * nSoupFamilies]();

    m_sys.initialize();
    int currframe = 0;

    // Run a loop that is typical of co-simulation. For instance, the wheeled is moved a bit, which moves the particles.
    // Conversely, the particles impress a force and torque upon the mesh soup
    for (float t = 0; t < params.time_end; t += iteration_step) {
        // Generate next tire location and orientation
        meshSoupLocOri[0] = 0;
        meshSoupLocOri[1] = pos_func_Y(t, params.box_Y);
        meshSoupLocOri[2] = pos_func_Z(t, params.box_Z);

        double rot[4];
        rot_func(t, rot);

        meshSoupLocOri[3] = rot[0];
        meshSoupLocOri[4] = rot[1];
        meshSoupLocOri[5] = rot[2];
        meshSoupLocOri[6] = rot[3];

        m_sys.meshSoup_applyRigidBodyMotion(meshSoupLocOri, meshVel);

        m_sys.advance_simulation(iteration_step);

        cout << "Rendering frame " << currframe << endl;
        char filename[100];
        sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
        m_sys.writeFile(string(filename));
        m_sys.write_meshes(string(filename));
    }

    delete[] genForcesOnMeshSoup;
    delete[] meshSoupLocOri;

    return 0;
}