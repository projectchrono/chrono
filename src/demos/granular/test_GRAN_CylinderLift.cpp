// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nic Olsen
// =============================================================================
// Chrono::Granular simulation forming an angle of repose based on friction and
// cohesion. Granular material is initially settled within a vertically-aligned
// mesh cylinder. The cylinder is then slowly raised to allow the material to
// create a mound of granular material on the box floor.
// =============================================================================

#include <iostream>
#include <string>
#include <cmath>
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/physics/ChGranularTriMesh.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_granular/utils/ChGranularJsonParser.h"

using namespace chrono;
using namespace chrono::granular;

using std::cout;
using std::endl;
using std::string;
using std::stof;
using std::stoi;
using std::vector;

void ShowUsage() {
    cout << "usage: ./test_GRAN_repose <json_file> <output_dir> <static_friction> <rolling_model 0:constant, "
            "1:viscous 2:elastic_plastic 3:none 4:schwartz> <rolling_friction> <spinning_friction> <cohesion_ratio>"
         << endl;
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
    if (argc != 8 || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }

    float iteration_step = params.step_size;
    const float Bx = params.box_X;
    const float By = Bx;

    const float Bz = params.box_Z;
    cout << "Box Dims: " << Bx << " " << By << " " << Bz << endl;

    ChSystemGranularSMC_trimesh m_sys(params.sphere_radius, params.sphere_density, make_float3(Bx, By, Bz));

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

    params.cohesion_ratio = std::stof(argv[7]);
    cout << "Cohesion Ratio: " << params.cohesion_ratio << endl;
    m_sys.set_Cohesion_ratio(params.cohesion_ratio);
    m_sys.set_Adhesion_ratio_S2M(params.adhesion_ratio_s2m);
    m_sys.set_Adhesion_ratio_S2W(params.adhesion_ratio_s2w);

    float static_friction = std::stof(argv[3]);
    cout << "Static Friction: " << static_friction << endl;
    cout << "Expected angle from sliding: " << std::atan(static_friction) << endl;
    // m_sys.set_friction_mode(GRAN_FRICTION_MODE::MULTI_STEP); TODO
    m_sys.set_friction_mode(GRAN_FRICTION_MODE::FRICTIONLESS);
    m_sys.set_static_friction_coeff_SPH2SPH(static_friction);
    m_sys.set_static_friction_coeff_SPH2WALL(static_friction);
    m_sys.set_static_friction_coeff_SPH2MESH(static_friction);

    int rolling_mode = std::atoi(argv[4]);
    float rolling_coeff = std::stof(argv[5]);
    cout << "Rolling resistance coefficient: " << rolling_coeff << endl;
    cout << "Expected angle from rolling: " << std::atan(rolling_coeff) << endl;

    m_sys.set_rolling_coeff_SPH2SPH(rolling_coeff);
    m_sys.set_rolling_coeff_SPH2WALL(rolling_coeff);
    m_sys.set_rolling_coeff_SPH2MESH(rolling_coeff);

    float spinning_coeff = std::stof(argv[6]);
    m_sys.set_spinning_coeff_SPH2SPH(spinning_coeff);
    m_sys.set_spinning_coeff_SPH2WALL(spinning_coeff);
    m_sys.set_spinning_coeff_SPH2MESH(spinning_coeff);

    switch (rolling_mode) {
        case 0:
            m_sys.set_rolling_mode(GRAN_ROLLING_MODE::CONSTANT_TORQUE);
            cout << "Constant torque rolling model" << endl;
            break;
        case 1:
            m_sys.set_rolling_mode(GRAN_ROLLING_MODE::VISCOUS);
            cout << "Viscous rolling model" << endl;
            break;
        case 2:
            m_sys.set_rolling_mode(GRAN_ROLLING_MODE::ELASTIC_PLASTIC);
            cout << "Elasti-plastic rolling model not yet implemented" << endl;
            ShowUsage();
            return 1;
            break;
        case 3:
            m_sys.set_rolling_mode(GRAN_ROLLING_MODE::NO_RESISTANCE);
            cout << "No rolling resistance" << endl;
            break;
        case 4:
            m_sys.set_rolling_mode(GRAN_ROLLING_MODE::SCHWARTZ);
            cout << "Schwartz rolling and spinning friction" << endl;
            cout << "Spinning resistance coefficient: " << spinning_coeff << endl;
            break;
        default:
            cout << "Invalid rolling mode" << endl;
            ShowUsage();
            return 1;
    }

    m_sys.setOutputMode(params.write_mode);
    string out_dir(argv[2]);
    m_sys.setVerbose(params.verbose);
    filesystem::create_directory(filesystem::path(out_dir));

    m_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::CENTERED_DIFFERENCE);
    m_sys.set_fixed_stepSize(params.step_size);
    m_sys.set_BD_Fixed(true);
    m_sys.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);

    const float fill_bottom = -Bz / 2.f;
    const float fill_height = Bz;

    // Cylinder mesh has interior radius 1 and total radius 1.1
    float cyl_center[3] = {0, 0, 0};
    const float3 scaling = make_float3(Bx / 4.f, Bx / 4.f, Bz);
    cout << "Cylinder radius: " << scaling.x << endl;

    utils::PDSampler<float> sampler(2.05 * params.sphere_radius);
    vector<ChVector<float>> body_points;

    const float fill_radius = scaling.x - 2.f * params.sphere_radius;
    const float fill_top = fill_bottom + fill_height;
    cout << "Fill radius " << fill_radius << endl;
    cout << "Fill bottom " << fill_bottom << endl;
    cout << "Fill top " << fill_top << endl;

    ChVector<float> center(0, 0, fill_bottom);
    center.z() += 2 * params.sphere_radius;
    while (center.z() < fill_top - 2 * params.sphere_radius) {
        auto points = sampler.SampleCylinderZ(center, fill_radius, 0);
        body_points.insert(body_points.end(), points.begin(), points.end());
        center.z() += 2.05 * params.sphere_radius;
    }

    unsigned int n_spheres = body_points.size();
    cout << "Adding " << n_spheres << " particles" << endl;
    m_sys.setParticlePositions(body_points);

    vector<string> mesh_filenames;
    vector<float3> mesh_scalings;
    vector<float> mesh_masses;
    const float mass = 10;

    string mesh_filename("data/granular/cylinder_lift/cylinder_refined.obj");
    mesh_filenames.push_back(mesh_filename);
    mesh_scalings.push_back(scaling);
    mesh_masses.push_back(mass);

    vector<bool> mesh_inflated;
    mesh_inflated.push_back(false);
    vector<float> mesh_inflation_radii;
    mesh_inflation_radii.push_back(0);

    m_sys.load_meshes(mesh_filenames, mesh_scalings, mesh_masses, mesh_inflated, mesh_inflation_radii);

    unsigned int nSoupFamilies = 1;
    cout << nSoupFamilies << " soup families" << endl;
    double* meshPosRot = new double[7 * nSoupFamilies];
    float* meshVel = new float[6 * nSoupFamilies]();

    m_sys.initialize();

    const double time_settling = 1.0;
    const double raising_vel = 1.0;
    const double raising_dist = Bz / 2.0;
    const double time_raising = raising_dist / raising_vel;

    cout << "Time settling " << time_settling << endl;
    cout << "Raising velocity " << raising_vel << endl;
    cout << "Raising distance " << raising_dist << endl;
    cout << "Time raising " << time_raising << endl;

    double mesh_z = 0.0;
    double mesh_vz = raising_vel;

    // Set initial mesh locations for the settling phase
    meshPosRot[0] = 0;
    meshPosRot[1] = 0;
    meshPosRot[2] = mesh_z;
    meshPosRot[3] = 1;
    meshPosRot[4] = 0;
    meshPosRot[5] = 0;
    meshPosRot[6] = 0;

    m_sys.meshSoup_applyRigidBodyMotion(meshPosRot, meshVel);

    unsigned int currframe = 0;
    double out_fps = 60;
    float frame_step = 1.f / out_fps;  // Duration of a frame
    unsigned int out_steps = frame_step / iteration_step;
    cout << "Writing at " << out_fps << " FPS" << endl;
    cout << "out_steps " << out_steps << endl;

    unsigned int step = 0;
    bool settled = false;
    ChVector<> pos_mesh(0, 0, mesh_z);

    cout << "Settling..." << endl;
    for (float t = 0; t < time_settling + time_raising; t += iteration_step, step++) {
        if (t >= time_settling) {
            // Raising phase
            if (!settled) {
                cout << "Raising..." << endl;
                settled = true;
            }

            mesh_z += iteration_step * raising_vel;
            meshPosRot[2] = mesh_z;
            pos_mesh.z() = mesh_z;  // For output
            meshVel[2] = mesh_vz;
            m_sys.meshSoup_applyRigidBodyMotion(meshPosRot, meshVel);
        }

        if (step % out_steps == 0) {
            cout << "Rendering frame " << currframe << endl;
            char filename[100];
            sprintf(filename, "%s/step%06u", out_dir.c_str(), currframe++);
            m_sys.writeFile(string(filename));
            // m_sys.write_meshes(string(filename));
            string mesh_output = string(filename) + "_meshframes.csv";

            std::ofstream meshfile(mesh_output);
            std::ostringstream outstream;
            outstream << "mesh_name,dx,dy,dz,x1,x2,x3,y1,y2,y3,z1,z2,z3,sx,sy,sz\n";

            writeMeshFrames(outstream, mesh_filename, pos_mesh, scaling);

            meshfile << outstream.str();
            meshfile.close();
        }

        m_sys.advance_simulation(iteration_step);
    }

    delete[] meshPosRot;
    delete[] meshVel;
    return 0;
}