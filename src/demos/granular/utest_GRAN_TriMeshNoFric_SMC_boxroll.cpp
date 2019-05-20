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
// Authors: Nic Olsen, Dan Negrut
// =============================================================================
//
// Chrono::Granular demo using SMC method. A body who's geometry is described
// by a trinagle mesh is initialized under settling granular material. No friction present.
//
// The global reference frame has X to the right, Y into the screen, Z up.
// The global reference frame located in the left lower corner, close to the viewer.
// =============================================================================

#include <iostream>
#include <string>
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/physics/ChGranularTriMesh.h"
#include "chrono_thirdparty/SimpleOpt/SimpleOpt.h"

#include "ChGranular_json_parser.hpp"

using namespace chrono;
using namespace chrono::granular;

using std::cout;
using std::endl;
using std::string;
using std::stof;
using std::stoi;
using std::vector;

enum {
    SINGLE_ON_VERTEX = 0,
    SINGLE_TO_CORNER = 1,
    MULTI_TO_CORNER = 2,
    SINGLE_TO_INV_CORNER = 3,
    BOX_FILL = 4,
    SINGLE_ON_LAYER
};

// -----------------------------------------------------------------------------
// Show command line usage
// -----------------------------------------------------------------------------
void ShowUsage() {
    cout << "usage: ./utest_GRAN_TriMeshNoFric_SMC_boxroll <json_file>" << endl;
}

// -----------------------------------------------------------------------------
// Demo for settling a monodisperse collection of shperes in a rectangular box.
// There is no friction. The units are always cm/s/g[L/T/M].
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    string output_dir = "output";

    float fps = 100;

    bool verbose = false;
    float cohesion_ratio = 0;

    sim_param_holder params;

    // Some of the default values might be overwritten by user via command line
    if (argc != 2 || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }

    float iteration_step = 1 / fps;
    // Setup simulation
    ChSystemGranularSMC_trimesh m_sys(params.sphere_radius, params.sphere_density,
                                      make_float3(params.box_X, params.box_Y, params.box_Z));

    m_sys.set_K_n_SPH2SPH(params.normalStiffS2S);
    m_sys.set_K_n_SPH2WALL(params.normalStiffS2W);
    m_sys.set_K_n_SPH2MESH(params.normalStiffS2M);
    m_sys.set_Gamma_n_SPH2SPH(params.normalDampS2S);
    m_sys.set_Gamma_n_SPH2WALL(params.normalDampS2W);
    m_sys.set_Gamma_n_SPH2MESH(params.normalDampS2M);
    m_sys.set_Cohesion_ratio(params.cohesion_ratio);
    m_sys.set_Adhesion_ratio_S2M(params.adhesion_ratio_s2m);
    m_sys.set_Adhesion_ratio_S2W(params.adhesion_ratio_s2w);
    m_sys.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);

    m_sys.setOutputMode(GRAN_OUTPUT_MODE::CSV);
    m_sys.setOutputDirectory(params.output_dir);
    filesystem::create_directory(filesystem::path(output_dir));

    m_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::CHUNG);
    m_sys.set_fixed_stepSize(params.step_size);

    m_sys.set_BD_Fixed(true);

    vector<ChVector<float>> pos;
    switch (params.run_mode) {
        case SINGLE_ON_VERTEX:
            pos.push_back(ChVector<float>(0.f, 0.1f, 3 * params.sphere_radius));
            break;

        case SINGLE_TO_CORNER:
            pos.push_back(ChVector<float>(-5.f, -6.f, params.sphere_radius));
            break;

        case MULTI_TO_CORNER:
            pos.push_back(ChVector<float>(1.f, 1.f, params.sphere_radius));
            pos.push_back(ChVector<float>(2.f, 4.f, params.sphere_radius));
            pos.push_back(ChVector<float>(4.f, 2.f, params.sphere_radius));
            pos.push_back(ChVector<float>(4.f, 5.f, params.sphere_radius));
            break;

        case SINGLE_TO_INV_CORNER:
            pos.push_back(ChVector<float>(10, -10, params.sphere_radius));
            break;

        case BOX_FILL:
            utils::HCPSampler<float> sampler(params.sphere_radius * 2.1);
            ChVector<float> boxCenter(0, 0, params.sphere_radius * 2.1);
            ChVector<float> hdims(9, 9, 0);
            pos = sampler.SampleBox(boxCenter, hdims);
            boxCenter.z() += params.sphere_radius * 2.1;
            pos = sampler.SampleBox(boxCenter, hdims);
            pos.push_back(ChVector<float>(0.f, -5.f, 5.f * params.sphere_radius));
            break;
    }

    m_sys.setParticlePositions(pos);

    switch (params.run_mode) {
        case SINGLE_ON_VERTEX:
        case BOX_FILL:
            m_sys.set_gravitational_acceleration(0.f, 0.f, -980);
            break;

        case SINGLE_TO_CORNER:
            m_sys.set_gravitational_acceleration(-400.f, -400.f, -980);
            break;

        case MULTI_TO_CORNER:
            m_sys.set_gravitational_acceleration(-400.f, -400.f, -980);
            break;

        case SINGLE_TO_INV_CORNER:
            m_sys.set_gravitational_acceleration(-400.f, 400.f, -980);
            break;
    }

    // Mesh values
    vector<string> mesh_filenames;
    string mesh_filename;

    vector<float3> mesh_scalings;
    float3 scaling;

    std::vector<float> mesh_masses;
    float mass;

    switch (params.run_mode) {
        case SINGLE_ON_VERTEX:
        case SINGLE_TO_CORNER:
        case MULTI_TO_CORNER:
            scaling.x = 15;
            scaling.y = 15;
            scaling.z = 10;
            mesh_filename = string("square_box.obj");
            mass = 10;
            break;

        case BOX_FILL:
            scaling.x = 10;
            scaling.y = 10;
            scaling.z = 10;
            mesh_filename = string("square_box.obj");
            mass = 10;
            break;

        case SINGLE_TO_INV_CORNER:
            scaling.x = 15;
            scaling.y = 15;
            scaling.z = 10;
            mesh_filename = string("inverted_corner.obj");
            mass = 10;
            break;
    }

    mesh_scalings.push_back(scaling);
    mesh_filenames.push_back(mesh_filename);
    mesh_masses.push_back(mass);
    std::vector<bool> mesh_inflated;
    std::vector<float> mesh_inflation_radii;
    mesh_inflated.push_back(false);
    mesh_inflation_radii.push_back(0);

    m_sys.load_meshes(mesh_filenames, mesh_scalings, mesh_masses, mesh_inflated, mesh_inflation_radii);

    unsigned int nSoupFamilies = m_sys.getNumTriangleFamilies();
    cout << nSoupFamilies << " soup families" << endl;
    float* genForcesOnMeshSoup = new float[6 * nSoupFamilies];
    double* meshSoupLocOri = new double[7 * nSoupFamilies];
    float* meshVel = new float[6 * nSoupFamilies]();

    m_sys.initialize();
    unsigned int currframe = 0;

    double sphere_mass =
        4. / 3. * CH_C_PI * params.sphere_density * params.sphere_radius * params.sphere_radius * params.sphere_radius;

    double sphere_weight = abs(sphere_mass * params.grav_Z);

    // Run a loop that is typical of co-simulation. For instance, the wheeled is moved a bit, which moves the
    // particles. Conversely, the particles impress a force and torque upon the mesh soup
    for (float t = 0; t < params.time_end; t += iteration_step) {
        float ball_force[6 * nSoupFamilies];
        m_sys.collectGeneralizedForcesOnMeshSoup(ball_force);

        printf("forces are (%f, %f, %f), torques are (%f, %f, %f)\n", ball_force[0], ball_force[1], ball_force[2],
               ball_force[3], ball_force[4], ball_force[5]);
        printf("sphere weight is %f, %f total spheres\n", sphere_weight, sphere_weight * pos.size());
        printf("ratio of z force to sphere weights is %f \n", ball_force[2] / (sphere_weight * pos.size()));
        // Generate next tire location and orientation
        meshSoupLocOri[0] = 0;
        meshSoupLocOri[1] = 0;
        meshSoupLocOri[2] = 0;
        meshSoupLocOri[3] = 1;
        meshSoupLocOri[4] = 0;
        meshSoupLocOri[5] = 0;
        meshSoupLocOri[6] = 0;

        m_sys.meshSoup_applyRigidBodyMotion(meshSoupLocOri, meshVel);  // Apply the mesh orientation data to the mesh
        cout << "Rendering frame " << currframe << endl;
        char filename[100];
        sprintf(filename, "%s/step%06u", output_dir.c_str(), currframe++);
        m_sys.writeFile(string(filename));
        m_sys.write_meshes(string(filename));

        m_sys.advance_simulation(iteration_step);
    }

    delete[] genForcesOnMeshSoup;
    delete[] meshSoupLocOri;

    return 0;
}