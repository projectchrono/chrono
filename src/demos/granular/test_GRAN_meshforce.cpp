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
#include "chrono/core/ChFileutils.h"
#include "chrono/utils/ChUtilsSamplers.h"
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

// -----------------------------------------------------------------------------
// Show command line usage
// -----------------------------------------------------------------------------
void ShowUsage() {
    cout << "usage: ./test_GRAN_meshtorque <json_file>" << endl;
}

int main(int argc, char* argv[]) {
    string output_dir = "output";

    float iteration_step = 0.02;

    bool verbose = false;
    float cohesion_ratio = 0;

    sim_param_holder params;

    // Some of the default values might be overwritten by user via command line
    if (argc != 2 || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }

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
    m_sys.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);
    m_sys.set_friction_mode(chrono::granular::GRAN_FRICTION_MODE::SINGLE_STEP);

    m_sys.setOutputMode(GRAN_OUTPUT_MODE::CSV);
    m_sys.setOutputDirectory(params.output_dir);
    ChFileutils::MakeDirectory(output_dir.c_str());

    m_sys.set_timeStepping(GRAN_TIME_STEPPING::FIXED);
    m_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::FORWARD_EULER);
    m_sys.set_fixed_stepSize(params.step_size);

    m_sys.setBOXdims(params.box_X, params.box_Y, params.box_Z);
    m_sys.set_BD_Fixed(true);

    utils::HCPSampler<float> sampler(2.1 * params.sphere_radius);
    auto pos = sampler.SampleBox(ChVector<>(0, 0, 26), ChVector<>(38, 38, 10));

    unsigned int n_spheres = pos.size();
    cout << "Created " << n_spheres << " spheres" << endl;
    double sphere_mass = params.sphere_density * 4.0 * CH_C_PI * params.sphere_radius * params.sphere_radius *
                         params.sphere_radius / 3.0;

    double total_mass = sphere_mass * n_spheres;
    double sphere_weight = sphere_mass * std::abs(params.grav_Z);
    double total_weight = total_mass * std::abs(params.grav_Z);

    m_sys.setParticlePositions(pos);

    // Mesh values
    vector<string> mesh_filenames;
    string mesh_filename("square_box.obj");
    mesh_filenames.push_back(mesh_filename);

    vector<float3> mesh_scalings;
    float3 scaling = make_float3(40, 40, 40);
    mesh_scalings.push_back(scaling);

    std::vector<float> mesh_masses;
    float mass = 1;
    mesh_masses.push_back(mass);

    m_sys.load_meshes(mesh_filenames, mesh_scalings, mesh_masses);

    unsigned int nSoupFamilies = m_sys.nMeshesInSoup();
    cout << nSoupFamilies << " soup families" << endl;
    float* genForcesOnMeshSoup = new float[6 * nSoupFamilies];
    double* meshSoupLocOri = new double[7 * nSoupFamilies];
    float* meshVel = new float[6 * nSoupFamilies]();

    m_sys.initialize();
    unsigned int currframe = 0;

    // Run a loop that is typical of co-simulation. For instance, the wheeled is moved a bit, which moves the
    // particles. Conversely, the particles impress a force and torque upon the mesh soup
    for (float t = 0; t < params.time_end; t += iteration_step) {
        // Triangle remains at the origin
        meshSoupLocOri[0] = 0;
        meshSoupLocOri[1] = 0;
        meshSoupLocOri[2] = 0;
        meshSoupLocOri[3] = 1;
        meshSoupLocOri[4] = 0;
        meshSoupLocOri[5] = 0;
        meshSoupLocOri[6] = 0;

        m_sys.meshSoup_applyRigidBodyMotion(meshSoupLocOri, meshVel);
        cout << "Rendering frame " << currframe << endl;
        char filename[100];
        sprintf(filename, "%s/step%06u", output_dir.c_str(), currframe++);
        m_sys.writeFileUU(string(filename));
        m_sys.write_meshes(string(filename));
        // m_sys.checkSDCounts(std::string(filename) + "SU", true, false);
        float forces[6];
        m_sys.collectGeneralizedForcesOnMeshSoup(forces);
        cout << "force_z: " << forces[2] << "; total weight: " << total_weight << "; sphere weight " << sphere_weight
             << endl;
        cout << "torque: " << forces[3] << ", " << forces[4] << ", " << forces[5] << endl;

        m_sys.advance_simulation(iteration_step);
    }

    delete[] genForcesOnMeshSoup;
    delete[] meshSoupLocOri;

    return 0;
}