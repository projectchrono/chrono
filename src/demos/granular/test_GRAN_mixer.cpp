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


#include <iostream>
#include <string>
#include <cmath>
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
using std::stof;
using std::stoi;
using std::vector;

enum MIXER_TYPE {
    INTERNAL = 0,                // Central 4-bladed mixer with cylinder boundary condition
    EXTERNAL_VERTICAL = 1,       // 4 vertical blades on a rotating drum
    EXTERNAL_VERTICAL_GRAV = 2,  // 4 vertical blades on a rotating drum. Drum at 45 deg angle.
    EXTERNAL_ANGLED = 3,         // 4 blades at a 30 deg angle on a rotating drum.
    EXTERNAL_ANGLED_GRAV = 4     // 4 blades at a 30 deg angle on a rotating drum. Drum at 45 deg angle.
};

void ShowUsage() {
    cout << "usage: ./test_GRAN_mixer <json_file> <output_dir>" << endl;
}

int main(int argc, char* argv[]) {
    sim_param_holder params;

    // Some of the default values might be overwritten by user via command line
    if (argc != 3 || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }
    float iteration_step = params.step_size;

    // Setup simulation
    ChSystemGranular_MonodisperseSMC_trimesh m_sys(params.sphere_radius, params.sphere_density,
                                                   make_float3(params.box_X, params.box_Y, params.box_Z));

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

    const float static_friction = 0.9;
    m_sys.set_static_friction_coeff_SPH2SPH(static_friction);
    m_sys.set_static_friction_coeff_SPH2WALL(static_friction);
    m_sys.set_static_friction_coeff_SPH2MESH(static_friction);
    cout << "Static Friction: " << static_friction << endl;

    m_sys.setOutputMode(GRAN_OUTPUT_MODE::CSV);

    string out_dir(argv[2]);
    m_sys.setOutputDirectory(out_dir);
    filesystem::create_directory(filesystem::path(out_dir));

    m_sys.set_timeStepping(GRAN_TIME_STEPPING::FIXED);
    m_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::FORWARD_EULER);
    m_sys.set_fixed_stepSize(params.step_size);
    m_sys.set_BD_Fixed(true);

    const float Bx = params.box_X;
    const float By = Bx;

    const float chamber_height = Bx / 3;  // TODO
    const float fill_height = chamber_height;

    const float Bz = chamber_height + fill_height;
    cout << "Box Dims: " << Bx << " " << By << " " << Bz << endl;

    const float chamber_bottom = -Bz / 2.f;
    const float fill_bottom = chamber_bottom + chamber_height;

    float cyl_center[3] = {0, 0, 0};
    const float cyl_rad = Bx / 2.f;
    m_sys.Create_BC_Cyl_Z(cyl_center, cyl_rad, false, false);

    utils::HCPSampler<float> sampler(2.2 * params.sphere_radius);

    const ChVector<> fill_center(0, 0, fill_bottom + fill_height / 2.f);
    const float fill_radius = Bx / 2.f - 2.f * params.sphere_radius;
    const float fill_htall = fill_height / 2.f - 2.f * params.sphere_radius;
    auto pos = sampler.SampleCylinderZ(fill_center, fill_radius, fill_htall);

    unsigned int n_spheres = pos.size();
    cout << "Created " << n_spheres << " spheres" << endl;

    m_sys.setParticlePositions(pos);
    MIXER_TYPE mixer_type = static_cast<MIXER_TYPE>(params.run_mode);
    float g[3];

    vector<string> mesh_filenames;
    string mesh_filename;
    switch (mixer_type) {
        case MIXER_TYPE::INTERNAL:
            mesh_filename = string("granular/mixer/internal_mixer.obj");
            g[0] = 0;
            g[1] = 0;
            g[2] = -980;
            break;
        case MIXER_TYPE::EXTERNAL_VERTICAL:
            mesh_filename = string("granular/mixer/external_mixer_vertical.obj");
            g[0] = 0;
            g[1] = 0;
            g[2] = -980;
            break;
        case MIXER_TYPE::EXTERNAL_VERTICAL_GRAV:
            mesh_filename = string("granular/mixer/external_mixer_vertical.obj");
            g[0] = -692.9646;
            g[1] = 0;
            g[2] = -692.9646;
            break;
        case MIXER_TYPE::EXTERNAL_ANGLED:
            mesh_filename = string("granular/mixer/external_mixer_angled.obj");
            g[0] = 0;
            g[1] = 0;
            g[2] = -980;
            break;
        case MIXER_TYPE::EXTERNAL_ANGLED_GRAV:
            mesh_filename = string("granular/mixer/external_mixer_angled.obj");
            g[0] = -692.9646;
            g[1] = 0;
            g[2] = -692.9646;
            break;
        default:
            cout << "Invalid mixer type" << endl;
            return 1;
    }

    m_sys.set_gravitational_acceleration(g[0], g[1], g[2]);

    mesh_filenames.push_back(mesh_filename);

    vector<float3> mesh_scalings;
    float scale_xy = Bx / 2.f;
    float scale_z = chamber_height;  // TODO fix this / make switch on mixer_type
    float3 scaling = make_float3(scale_xy, scale_xy, scale_z);
    mesh_scalings.push_back(scaling);

    std::vector<float> mesh_masses;
    float mass = 10;
    mesh_masses.push_back(mass);

    std::vector<bool> mesh_inflated;
    std::vector<float> mesh_inflation_radii;
    mesh_inflated.push_back(false);
    mesh_inflation_radii.push_back(0);

    m_sys.load_meshes(mesh_filenames, mesh_scalings, mesh_masses, mesh_inflated, mesh_inflation_radii);

    unsigned int nSoupFamilies = m_sys.getNumTriangleFamilies();
    cout << nSoupFamilies << " soup families" << endl;
    double* meshSoupLocOri = new double[7 * nSoupFamilies];
    float* meshVel = new float[6 * nSoupFamilies]();

    float rev_per_sec = 0.5f;
    float ang_vel_Z = rev_per_sec * 2 * CH_C_PI;
    meshVel[5] = ang_vel_Z;  // BUG with omegas for mesh

    m_sys.initialize();

    unsigned int currframe = 0;
    double out_fps = 60;
    float frame_step = 1.f / out_fps;  // Duration of a frame
    unsigned int out_steps = frame_step / iteration_step;
    cout << "out_steps " << out_steps << endl;

    unsigned int step = 0;

    for (float t = 0; t < params.time_end; t += iteration_step, step++) {
        meshSoupLocOri[0] = 0;
        meshSoupLocOri[1] = 0;
        meshSoupLocOri[2] = chamber_bottom + chamber_height / 2.0;

        auto q = Q_from_AngZ(t * ang_vel_Z);
        meshSoupLocOri[3] = q[0];
        meshSoupLocOri[4] = q[1];
        meshSoupLocOri[5] = q[2];
        meshSoupLocOri[6] = q[3];

        m_sys.meshSoup_applyRigidBodyMotion(meshSoupLocOri, meshVel);
        if (step % out_steps == 0) {
            cout << "Rendering frame " << currframe << endl;
            char filename[100];
            sprintf(filename, "%s/step%06u", out_dir.c_str(), currframe++);
            m_sys.writeFile(string(filename));
            m_sys.write_meshes(string(filename));
            float forces[6];
            m_sys.collectGeneralizedForcesOnMeshSoup(forces);
            cout << "torque: " << forces[3] << ", " << forces[4] << ", " << forces[5] << endl;
        }

        m_sys.advance_simulation(iteration_step);
    }

    delete[] meshSoupLocOri;

    return 0;
}
