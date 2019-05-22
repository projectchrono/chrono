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
    cout << "usage: ./test_GRAN_mixer <json_file> <output_dir> <run_mode:0-4>" << endl;
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

    if (argc != 4 || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }

    const float Bx = params.box_X;
    const float By = Bx;
    const float chamber_height = Bx / 3;  // TODO
    const float fill_height = chamber_height;
    const float Bz = chamber_height + fill_height;
    cout << "Box Dims: " << Bx << " " << By << " " << Bz << endl;

    float iteration_step = params.step_size;
    string out_dir(argv[2]);
    MIXER_TYPE mixer_type = static_cast<MIXER_TYPE>(stoi(argv[3]));

    ChSystemGranularSMC_trimesh gran_sys(params.sphere_radius, params.sphere_density, make_float3(Bx, By, Bz));

    gran_sys.set_K_n_SPH2SPH(params.normalStiffS2S);
    gran_sys.set_K_n_SPH2WALL(params.normalStiffS2W);
    gran_sys.set_K_n_SPH2MESH(params.normalStiffS2M);

    gran_sys.set_K_t_SPH2SPH(params.tangentStiffS2S);
    gran_sys.set_K_t_SPH2WALL(params.tangentStiffS2W);
    gran_sys.set_K_t_SPH2MESH(params.tangentStiffS2M);

    gran_sys.set_Gamma_n_SPH2SPH(params.normalDampS2S);
    gran_sys.set_Gamma_n_SPH2WALL(params.normalDampS2W);
    gran_sys.set_Gamma_n_SPH2MESH(params.normalDampS2M);

    gran_sys.set_Gamma_t_SPH2SPH(params.tangentDampS2S);
    gran_sys.set_Gamma_t_SPH2WALL(params.tangentDampS2W);
    gran_sys.set_Gamma_t_SPH2MESH(params.tangentDampS2M);

    gran_sys.set_Cohesion_ratio(params.cohesion_ratio);
    gran_sys.set_Adhesion_ratio_S2M(params.adhesion_ratio_s2m);
    gran_sys.set_Adhesion_ratio_S2W(params.adhesion_ratio_s2w);
    gran_sys.set_friction_mode(chrono::granular::GRAN_FRICTION_MODE::MULTI_STEP);

    const float static_friction = 0.9;
    cout << "Static Friction: " << static_friction << endl;
    gran_sys.set_static_friction_coeff_SPH2SPH(static_friction);
    gran_sys.set_static_friction_coeff_SPH2WALL(static_friction);
    gran_sys.set_static_friction_coeff_SPH2MESH(static_friction);

    gran_sys.setOutputMode(params.write_mode);

    gran_sys.setOutputDirectory(out_dir);
    filesystem::create_directory(filesystem::path(out_dir));

    gran_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::CENTERED_DIFFERENCE);
    gran_sys.set_fixed_stepSize(params.step_size);
    gran_sys.set_BD_Fixed(true);

    const float chamber_bottom = -Bz / 2.f;
    const float fill_bottom = chamber_bottom + chamber_height;

    float cyl_center[3] = {0, 0, 0};
    const float cyl_rad = Bx / 2.f;
    gran_sys.Create_BC_Cyl_Z(cyl_center, cyl_rad, false, false);

    utils::HCPSampler<float> sampler(2.1 * params.sphere_radius);
    vector<ChVector<float>> body_points;

    const float fill_radius = Bx / 2.f - 2.f * params.sphere_radius;
    const float fill_top = fill_bottom + fill_height;

    unsigned int n_spheres = body_points.size();
    cout << "Created " << n_spheres << " spheres" << endl;
    cout << "Fill radius " << fill_radius << endl;
    cout << "Fill bottom " << fill_bottom << endl;
    cout << "Fill top " << fill_top << endl;

    ChVector<float> center(0, 0, fill_bottom);
    center.z() += 2 * params.sphere_radius;
    while (center.z() < fill_top - 2 * params.sphere_radius) {
        auto points = sampler.SampleCylinderZ(center, fill_radius, 0);
        body_points.insert(body_points.end(), points.begin(), points.end());
        center.z() += 2.1 * params.sphere_radius;
    }

    gran_sys.setParticlePositions(body_points);

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

    gran_sys.set_gravitational_acceleration(g[0], g[1], g[2]);

    mesh_filenames.push_back(mesh_filename);

    vector<float3> mesh_scalings;
    float scale_xy = Bx / 2.f;
    float scale_z = chamber_height;  // TODO fix this / make switch on mixer_type
    float3 scaling = make_float3(scale_xy, scale_xy, scale_z);
    mesh_scalings.push_back(scaling);

    vector<float> mesh_masses;
    float mixer_mass = 10;
    mesh_masses.push_back(mixer_mass);

    vector<bool> mesh_inflated;
    vector<float> mesh_inflation_radii;
    mesh_inflated.push_back(false);
    mesh_inflation_radii.push_back(0);

    gran_sys.load_meshes(mesh_filenames, mesh_scalings, mesh_masses, mesh_inflated, mesh_inflation_radii);

    unsigned int nSoupFamilies = gran_sys.getNumTriangleFamilies();
    cout << nSoupFamilies << " soup families" << endl;
    double* mesh_pos_rot = new double[7 * nSoupFamilies];
    float* mesh_vel = new float[6 * nSoupFamilies]();

    float rev_per_sec = 0.5f;
    float ang_vel_Z = rev_per_sec * 2 * CH_C_PI;
    mesh_vel[5] = ang_vel_Z;

    gran_sys.initialize();

    unsigned int currframe = 0;
    double out_fps = 60;
    float frame_step = 1.f / out_fps;  // Duration of a frame
    unsigned int out_steps = frame_step / iteration_step;
    cout << "out_steps " << out_steps << endl;

    unsigned int step = 0;

    for (float t = 0; t < params.time_end; t += iteration_step, step++) {
        mesh_pos_rot[0] = 0;
        mesh_pos_rot[1] = 0;
        mesh_pos_rot[2] = chamber_bottom + chamber_height / 2.0;

        auto q = Q_from_AngZ(t * ang_vel_Z);
        mesh_pos_rot[3] = q[0];
        mesh_pos_rot[4] = q[1];
        mesh_pos_rot[5] = q[2];
        mesh_pos_rot[6] = q[3];

        gran_sys.meshSoup_applyRigidBodyMotion(mesh_pos_rot, mesh_vel);
        if (step % out_steps == 0) {
            cout << "Rendering frame " << currframe << endl;
            char filename[100];
            sprintf(filename, "%s/step%06u", out_dir.c_str(), currframe++);
            gran_sys.writeFile(string(filename));
            gran_sys.write_meshes(string(filename));

            string mesh_output = string(filename) + "_meshframes.csv";

            std::ofstream meshfile(mesh_output);
            std::ostringstream outstream;
            outstream << "mesh_name,dx,dy,dz,x1,x2,x3,y1,y2,y3,z1,z2,z3,sx,sy,sz\n";

            double angle = t * ang_vel_Z;
            ChVector<> pos_mesh(mesh_pos_rot[0], mesh_pos_rot[1], mesh_pos_rot[2]);
            writeMeshFrames(outstream, mesh_filename, pos_mesh, scaling, angle);

            meshfile << outstream.str();
            meshfile.close();

            float forces[6];
            gran_sys.collectGeneralizedForcesOnMeshSoup(forces);
            cout << "torque: " << forces[3] << ", " << forces[4] << ", " << forces[5] << endl;
        }

        gran_sys.advance_simulation(iteration_step);
    }

    delete[] mesh_pos_rot;
    delete[] mesh_vel;

    return 0;
}