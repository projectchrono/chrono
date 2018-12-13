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
    m_sys.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);
    m_sys.set_friction_mode(chrono::granular::GRAN_FRICTION_MODE::SINGLE_STEP);

    m_sys.setOutputMode(GRAN_OUTPUT_MODE::CSV);
    m_sys.setOutputDirectory(params.output_dir);
    ChFileutils::MakeDirectory(params.output_dir.c_str());

    m_sys.set_timeStepping(GRAN_TIME_STEPPING::FIXED);
    m_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::FORWARD_EULER);
    m_sys.set_fixed_stepSize(params.step_size);
    m_sys.set_BD_Fixed(true);

    const float Bx = params.box_X;
    const float By = Bx;
    float fill_height = 60.f / 4.f;      // height above top of cone that will be filled with particles
    float radius_opening = 4.f;  // Opening at bottom of cone      // TODO vary
    float cone_slope = 1.f;      // TODO vary
    float z_cone_opening = 70.f / 4.f;   // z coord of the cone opening

    float z_cone_tip = z_cone_opening - cone_slope * radius_opening;
    float z_cone_top = z_cone_tip + cone_slope * Bx / std::sqrt(2);

    const float Bz = z_cone_top + fill_height;

    // Offset z values
    z_cone_opening -= Bz / 2.f;
    z_cone_tip -= Bz / 2.f;
    z_cone_top -= Bz / 2.f;

    float cone_tip[3] = {0.f, 0.f, z_cone_tip};  // Hypothetical location of cone tip
    constexpr bool outward_normal = false;       // Inward-colliding cone

    cout << "tip " << cone_tip[0] << " " << cone_tip[1] << " " << cone_tip[2] << endl;
    cout << "slope " << cone_slope << endl;
    cout << "z_cone_top " << z_cone_top << endl;
    m_sys.Create_BC_Cone_Z(cone_tip, cone_slope, z_cone_top, z_cone_opening, outward_normal);

    m_sys.setBOXdims(Bx, By, Bz);
    cout << "Box Dims: " << Bx << " " << By << " " << Bz << endl;
    utils::HCPSampler<float> sampler(2.2 * params.sphere_radius);
    const float z_fill = z_cone_top + fill_height / 2.f;

    auto pos = sampler.SampleBox(ChVector<>(0, 0, z_fill), ChVector<>(Bx / 2.f - 3.f * params.sphere_radius,
                                                                      By / 2.f - 3.f * params.sphere_radius,
                                                                      fill_height / 2.f - 3.f * params.sphere_radius));

    unsigned int n_spheres = pos.size();
    cout << "Created " << n_spheres << " spheres" << endl;

    m_sys.setParticlePositions(pos);

    // Mesh values
    vector<string> mesh_filenames;
    string mesh_filename("granular/Mixer.obj");
    mesh_filenames.push_back(mesh_filename);

    vector<float3> mesh_scalings;
    float scale_xy = Bx / (2 * 3.f) - 4 * params.sphere_radius;
    float3 scaling = make_float3(scale_xy, scale_xy, 10);  // TODO rethink this
    mesh_scalings.push_back(scaling);

    std::vector<float> mesh_masses;
    float mass = 10;  // TODO
    mesh_masses.push_back(mass);

    m_sys.load_meshes(mesh_filenames, mesh_scalings, mesh_masses);

    unsigned int nSoupFamilies = m_sys.nMeshesInSoup();
    cout << nSoupFamilies << " soup families" << endl;
    double* meshSoupLocOri = new double[7 * nSoupFamilies];
    float* meshVel = new float[6 * nSoupFamilies]();

    float rev_per_sec = 0.5f;
    float ang_vel_Z = rev_per_sec * 2 * CH_C_PI;

    // BUG with omegas for mesh
    meshVel[5] = ang_vel_Z;

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
        meshSoupLocOri[2] = -Bz / 2.f + scaling.z / 4 + 4.f * params.sphere_radius;

        auto q = Q_from_AngZ(t * ang_vel_Z);
        meshSoupLocOri[3] = q[0];
        meshSoupLocOri[4] = q[1];
        meshSoupLocOri[5] = q[2];
        meshSoupLocOri[6] = q[3];

        m_sys.meshSoup_applyRigidBodyMotion(meshSoupLocOri, meshVel);
        if (step % out_steps == 0) {
            cout << "Rendering frame " << currframe << endl;
            char filename[100];
            sprintf(filename, "%s/step%06u", params.output_dir.c_str(), currframe++);
            m_sys.writeFileUU(string(filename));
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
