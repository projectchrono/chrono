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
// This test compares timing results of three systems:
// (1) Granular-only (2) Granular and a single disabled triangle mesh
// (3) Granular and a single enabled triangle mesh fixed in place
// =============================================================================
// Authors: Nic Olsen
// =============================================================================

#include <iostream>
#include <string>
#include <cmath>
#include <ctime>
#include "chrono_thirdparty/filesystem/path.h"
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
using std::vector;

enum RUN_MODE { GRAN = 0, GRAN_TRI_DISABLED = 1, GRAN_TRI_ENABLED = 2 };

double fill_top;
double block_mass = 1;

void ShowUsage() {
    cout << "usage: ./metrics_GRAN_settling <json_file> <optional: test index for single test>" << endl;
}

void SetupGranSystem(ChSystemGranularSMC& m_sys, sim_param_holder& params) {
    m_sys.set_K_n_SPH2SPH(params.normalStiffS2S);
    m_sys.set_K_n_SPH2WALL(params.normalStiffS2W);
    m_sys.set_K_t_SPH2SPH(params.tangentStiffS2S);
    m_sys.set_K_t_SPH2WALL(params.tangentStiffS2W);
    m_sys.set_Gamma_n_SPH2SPH(params.normalDampS2S);
    m_sys.set_Gamma_n_SPH2WALL(params.normalDampS2W);
    m_sys.set_Gamma_t_SPH2SPH(params.tangentDampS2S);
    m_sys.set_Gamma_t_SPH2WALL(params.tangentDampS2W);

    m_sys.set_Cohesion_ratio(params.cohesion_ratio);
    m_sys.set_Adhesion_ratio_S2W(params.adhesion_ratio_s2w);
    m_sys.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);
    m_sys.set_friction_mode(chrono::granular::GRAN_FRICTION_MODE::MULTI_STEP);

    m_sys.setOutputMode(params.write_mode);
    m_sys.setOutputDirectory(params.output_dir);

    m_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::EXTENDED_TAYLOR);
    m_sys.set_fixed_stepSize(params.step_size);
    m_sys.set_BD_Fixed(true);

    // Fill domain with particles
    vector<ChVector<float>> body_points;
    double epsilon = 0.2 * params.sphere_radius;
    double spacing = 2 * params.sphere_radius + epsilon;

    utils::HCPSampler<float> sampler(spacing);
    double fill_bottom = -params.box_Z / 2 + 2 * spacing;
    fill_top = params.box_Z / 2 - params.sphere_radius - epsilon;
    ChVector<> hdims(params.box_X / 2 - params.sphere_radius - epsilon,
                     params.box_Y / 2 - params.sphere_radius - epsilon, 0);
    for (double z = fill_bottom; z < fill_top; z += spacing) {
        ChVector<> center(0, 0, z);
        auto points = sampler.SampleBox(center, hdims);
        body_points.insert(body_points.end(), points.begin(), points.end());
    }

    cout << "Created " << body_points.size() << " spheres" << endl;

    m_sys.setParticlePositions(body_points);
}

void SetupGranTriSystem(ChSystemGranularSMC_trimesh& m_sys, sim_param_holder& params) {
    SetupGranSystem(m_sys, params);

    m_sys.set_K_n_SPH2MESH(params.normalStiffS2M);
    m_sys.set_K_t_SPH2MESH(params.tangentStiffS2M);
    m_sys.set_Gamma_n_SPH2MESH(params.normalDampS2M);
    m_sys.set_Gamma_t_SPH2MESH(params.tangentDampS2M);
    m_sys.set_Adhesion_ratio_S2M(params.adhesion_ratio_s2m);

    // Mesh values
    vector<string> mesh_filenames;
    string mesh_filename("granular/upward_plane_refined.obj");
    mesh_filenames.push_back(mesh_filename);

    vector<float3> mesh_scalings;
    float3 scaling = make_float3(params.box_X / 2, params.box_Y / 2, 1);
    mesh_scalings.push_back(scaling);

    vector<float> mesh_masses;
    mesh_masses.push_back(block_mass);

    std::vector<bool> mesh_inflated;
    std::vector<float> mesh_inflation_radii;
    mesh_inflated.push_back(false);
    mesh_inflation_radii.push_back(0);

    m_sys.load_meshes(mesh_filenames, mesh_scalings, mesh_masses, mesh_inflated, mesh_inflation_radii);
}

double RunTest(sim_param_holder& params, RUN_MODE run_mode) {
    double out_fps = 50;
    float frame_step = 1.0 / out_fps;

    clock_t start = std::clock();
    switch (run_mode) {
        case RUN_MODE::GRAN: {
            cout << "Running Granular system test..." << endl;
            ChSystemGranularSMC m_sys(params.sphere_radius, params.sphere_density,
                                      make_float3(params.box_X, params.box_Y, params.box_Z));
            SetupGranSystem(m_sys, params);
            filesystem::create_directory(filesystem::path(params.output_dir));
            m_sys.initialize();

            unsigned int currframe = 0;
            for (float t = 0; t < params.time_end; t += frame_step) {
                cout << "Rendering frame " << currframe << endl;
                char filename[100];
                sprintf(filename, "%s/step%06u", params.output_dir.c_str(), currframe++);
                m_sys.writeFile(string(filename));

                m_sys.advance_simulation(frame_step);
            }
            break;
        }
        case RUN_MODE::GRAN_TRI_DISABLED: {
            cout << "Running Granular system with disabled mesh test..." << endl;
            ChSystemGranularSMC_trimesh m_sys(params.sphere_radius, params.sphere_density,
                                              make_float3(params.box_X, params.box_Y, params.box_Z));
            SetupGranTriSystem(m_sys, params);
            m_sys.disableMeshCollision();
            filesystem::create_directory(filesystem::path(params.output_dir));

            unsigned int nSoupFamilies = m_sys.getNumTriangleFamilies();
            cout << nSoupFamilies << " soup families" << endl;
            double* meshSoupLocOri = new double[7 * nSoupFamilies];
            float* meshVel = new float[6 * nSoupFamilies]();

            m_sys.initialize();

            unsigned int currframe = 0;
            for (float t = 0; t < params.time_end; t += frame_step) {
                meshSoupLocOri[0] = 0;
                meshSoupLocOri[1] = 0;
                meshSoupLocOri[2] = -params.box_Z / 2 + 2 * params.sphere_radius;

                meshSoupLocOri[3] = 1;
                meshSoupLocOri[4] = 0;
                meshSoupLocOri[5] = 0;
                meshSoupLocOri[6] = 0;

                m_sys.meshSoup_applyRigidBodyMotion(meshSoupLocOri, meshVel);
                cout << "Rendering frame " << currframe << endl;
                char filename[100];
                sprintf(filename, "%s/step%06u", params.output_dir.c_str(), currframe++);
                m_sys.writeFile(string(filename));
                m_sys.write_meshes(string(filename));

                m_sys.advance_simulation(frame_step);
            }
            delete[] meshSoupLocOri;

            break;
        }
        case RUN_MODE::GRAN_TRI_ENABLED: {
            cout << "Running Granular system with enabled mesh test..." << endl;
            ChSystemGranularSMC_trimesh m_sys(params.sphere_radius, params.sphere_density,
                                              make_float3(params.box_X, params.box_Y, params.box_Z));
            SetupGranTriSystem(m_sys, params);
            m_sys.enableMeshCollision();
            filesystem::create_directory(filesystem::path(params.output_dir));

            unsigned int nSoupFamilies = m_sys.getNumTriangleFamilies();
            cout << nSoupFamilies << " soup families" << endl;
            double* meshSoupLocOri = new double[7 * nSoupFamilies];
            float* meshVel = new float[6 * nSoupFamilies]();

            m_sys.initialize();

            unsigned int currframe = 0;
            for (float t = 0; t < params.time_end; t += frame_step) {
                meshSoupLocOri[0] = 0;
                meshSoupLocOri[1] = 0;
                meshSoupLocOri[2] = -params.box_Z / 2 + 2 * params.sphere_radius;

                meshSoupLocOri[3] = 1;
                meshSoupLocOri[4] = 0;
                meshSoupLocOri[5] = 0;
                meshSoupLocOri[6] = 0;

                m_sys.meshSoup_applyRigidBodyMotion(meshSoupLocOri, meshVel);
                cout << "Rendering frame " << currframe << endl;
                char filename[100];
                sprintf(filename, "%s/step%06u", params.output_dir.c_str(), currframe++);
                m_sys.writeFile(string(filename));
                m_sys.write_meshes(string(filename));

                m_sys.advance_simulation(frame_step);
            }
            delete[] meshSoupLocOri;

            break;
        }
    }
    clock_t end = std::clock();
    double total_time = ((double)(end - start)) / CLOCKS_PER_SEC;

    return total_time;
}

int main(int argc, char* argv[]) {
    sim_param_holder params;

    // Some of the default values might be overwritten by user via command line
    if (!(argc != 2 || argc != 3) || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }
    int single_run = -1;
    if (argc == 3) {
        single_run = std::atoi(argv[2]);
    }

    double time_gran = -1;
    if (argc == 2 || single_run == RUN_MODE::GRAN) {
        time_gran = RunTest(params, RUN_MODE::GRAN);
    }

    double time_tri_disabled = -1;
    if (argc == 2 || single_run == RUN_MODE::GRAN_TRI_DISABLED) {
        time_tri_disabled = RunTest(params, RUN_MODE::GRAN_TRI_DISABLED);
    }

    double time_tri_enabled = -1;
    if (argc == 2 || single_run == RUN_MODE::GRAN_TRI_ENABLED) {
        time_tri_enabled = RunTest(params, RUN_MODE::GRAN_TRI_ENABLED);
    }

    cout << "================== Results ==================" << endl;
    cout << "Granular system: " << time_gran << " seconds" << endl;
    cout << "Granular system with disabled triangles: " << time_tri_disabled << " seconds" << endl;
    cout << "Granular system with enabled triangles: " << time_tri_enabled << " seconds" << endl;

    return 0;
}