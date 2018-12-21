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

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"

#include "ChGranular_json_parser.hpp"

using namespace chrono;
using namespace chrono::granular;

using std::cout;
using std::endl;
using std::string;
using std::vector;

/*
 * Lommen 2014:
 * Settle particles then drop box
 * 18,000 particles
 * Settled for 1 s
 * Poisson Ratio = 0.25
 * Coefficient of Restitution = 0.5
 * Static friction = 0.5
 * Rolling Friction = 0.04
 * Drop height (above material)= 0.2 m
 * Box mass = 50 kg
 * Periodic boundary condition in x and y
 * Time after release = 3 s
 * Varied Shear modulus
 *
 * Assumed:
 * Particle diameter = 4, 8, or 16 mm
 * Density 1500 kg/m3
 * Settled height ~0.25 m
 */

const double time_settle = 1;
double fill_top;

const double block_mass = 50000;  // 50kg
const double drop_height = 20;    // 0.2m

void ShowUsage() {
    cout << "usage: ./test_GRAN_bulkcompress <json_file>" << endl;
}

void SetupGranSystem(ChSystemGranular_MonodisperseSMC_trimesh& m_sys, sim_param_holder& params) {
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

    m_sys.set_timeStepping(GRAN_TIME_STEPPING::FIXED);
    m_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::FORWARD_EULER);
    m_sys.set_fixed_stepSize(params.step_size);
    m_sys.set_BD_Fixed(true);

    m_sys.setBOXdims(params.box_X, params.box_Y, params.box_Z);

    double epsilon = 0.2 * params.sphere_radius;
    double spacing = 2 * params.sphere_radius + epsilon;

    vector<ChVector<float>> body_points;

    // utils::HCPSampler<float> sampler(spacing);
    utils::PDSampler<float> sampler(2 * params.sphere_radius + epsilon);
    double fill_bottom = -params.box_Z / 2 + params.sphere_radius + epsilon;
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

    // Mesh values
    vector<string> mesh_filenames;
    string mesh_filename("granular/downward_square.obj");
    mesh_filenames.push_back(mesh_filename);

    vector<float3> mesh_scalings;
    float3 scaling = make_float3(params.box_X / 2, params.box_Y / 2, 1);
    mesh_scalings.push_back(scaling);

    vector<float> mesh_masses;
    mesh_masses.push_back(block_mass);

    m_sys.load_meshes(mesh_filenames, mesh_scalings, mesh_masses);
}

int main(int argc, char* argv[]) {
    sim_param_holder params;

    if (argc != 2 || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }

    float iteration_step = params.step_size;

    ChSystemGranular_MonodisperseSMC_trimesh m_sys(params.sphere_radius, params.sphere_density);
    SetupGranSystem(m_sys, params);
    ChFileutils::MakeDirectory(params.output_dir.c_str());

    ChSystemSMC ch_sys;
    ch_sys.Set_G_acc(ChVector<>(params.grav_X, params.grav_Y, params.grav_Z));
    auto block = std::make_shared<ChBody>();
    block->SetBodyFixed(true);
    block->SetPos(ChVector<>(0, 0, params.box_Z));
    block->SetMass(block_mass);
    ch_sys.AddBody(block);

    unsigned int nSoupFamilies = m_sys.nMeshesInSoup();
    cout << nSoupFamilies << " soup families" << endl;
    double* meshSoupLocOri = new double[7 * nSoupFamilies];
    float* meshVel = new float[6 * nSoupFamilies]();

    m_sys.initialize();
    unsigned int currframe = 0;
    double out_fps = 100;
    float frame_step = 1.f / out_fps;  // Duration of a frame
    unsigned int out_steps = frame_step / iteration_step;
    cout << "out_steps " << out_steps << endl;

    unsigned int step = 0;
    bool box_released = false;
    m_sys.disableMeshCollision();
    for (float t = 0; t < params.time_end; t += iteration_step, step++) {
        if (t >= time_settle && box_released == false) {
            m_sys.enableMeshCollision();

            block->SetBodyFixed(false);
            double max_z = m_sys.get_max_z();
            block->SetPos(ChVector<>(0, 0, max_z + params.sphere_radius + drop_height));

            box_released = true;
            cout << "Releasing box" << endl;
        }

        meshSoupLocOri[0] = 0;
        meshSoupLocOri[1] = 0;
        meshSoupLocOri[2] = block->GetPos().z();

        meshSoupLocOri[3] = 1;
        meshSoupLocOri[4] = 0;
        meshSoupLocOri[5] = 0;
        meshSoupLocOri[6] = 0;

        m_sys.meshSoup_applyRigidBodyMotion(meshSoupLocOri, meshVel);
        if (step % out_steps == 0) {
            cout << "Rendering frame " << currframe << endl;
            char filename[100];
            sprintf(filename, "%s/step%06u", params.output_dir.c_str(), currframe++);
            m_sys.writeFileUU(string(filename));
            m_sys.write_meshes(string(filename));
            if (box_released) {
                cout << block->GetPos().z() << endl;
            }
        }

        float forces[6];
        m_sys.collectGeneralizedForcesOnMeshSoup(forces);

        ch_sys.DoStepDynamics(iteration_step);
        m_sys.advance_simulation(iteration_step);

        block->Empty_forces_accumulators();
        block->Accumulate_force(ChVector<>(0, 0, forces[2]), block->GetPos(), false);
    }

    delete[] meshSoupLocOri;

    return 0;
}