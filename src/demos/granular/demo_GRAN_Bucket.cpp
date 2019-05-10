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
// Authors: Dan Negrut, Nic Olsen
// =============================================================================
//
// Chrono::Granular demo program using SMC method for frictional contact.
//
// Basic simulation of a settling scenario;
//  - box is rectangular
//

#include <iostream>
#include <string>
#ifdef _WINDOWS
#define _USE_MATH_DEFINES
#include <math.h>
#endif
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_granular/physics/ChGranular.h"
#include "ChGranular_json_parser.hpp"
#include "ChGranularDemoUtils.hpp"
#include "chrono/utils/ChUtilsSamplers.h"

using namespace chrono;
using namespace chrono::granular;
using std::cout;
using std::endl;
using std::string;

// expected number of args for param sweep
constexpr int num_args_full = 5;

int currcapture = 0;
int currframe = 0;

// -----------------------------------------------------------------------------
// Show command line usage
// -----------------------------------------------------------------------------
void ShowUsage() {
    cout << "usage: ./demo_GRAN_Bucket <json_file> [<run_mode> <box_Z> <output_dir>]" << endl;
    cout << "must have either 1 or " << num_args_full - 1 << " arguments" << endl;
}

sim_param_holder params;

std::string box_filename = "BD_Box.obj";

// Take a ChBody and write its
void writeBoxMesh(std::ostringstream& outstream) {
    ChVector<> pos(0, 0, 0);
    // Get basis vectors
    ChVector<> vx(1, 0, 0);
    ChVector<> vy(0, 1, 0);
    ChVector<> vz(0, 0, 1);

    ChVector<> scaling(params.box_X / 2, params.box_Y / 2, params.box_Z / 2);

    // Write the mesh name to find
    outstream << box_filename << ",";
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
    outstream << scaling.x() << ",";
    outstream << scaling.y() << ",";
    outstream << scaling.z();
    outstream << "\n";
}

enum run_mode { FRICTIONLESS = 0, ONE_STEP = 1, MULTI_STEP = 2, FRICLESS_CHUNG = 3, FRICLESS_CD = 4 };

std::vector<size_t> bc_ids;
std::vector<std::string> bc_names;

constexpr float F_CGS_TO_SI = 1e-5;
constexpr float M_CGS_TO_SI = 1e-3;

void writeForcesFile(ChSystemGranular_MonodisperseSMC& settlingExperiment) {
    char forcefile[100];
    sprintf(forcefile, "%s/force%06d.csv", (params.output_dir + "/forces").c_str(), currcapture++);
    printf("force file is %s\n", forcefile);
    std::ofstream ofile(forcefile, std::ios::out);

    std::ostringstream outstrstream;

    outstrstream << "plane_id,fx,fy,fz\n";

    float reaction_forces[3] = {0, 0, 0};
    for (unsigned int i = 0; i < bc_ids.size(); i++) {
        bool success = settlingExperiment.getBCReactionForces(bc_ids[i], reaction_forces);
        if (!success) {
            printf("ERROR! Get contact forces for plane %u failed\n", i);
        } else {
            outstrstream << i << "," << F_CGS_TO_SI * reaction_forces[0] << "," << F_CGS_TO_SI * reaction_forces[1]
                         << "," << F_CGS_TO_SI * reaction_forces[2] << "\n";
            printf("force on plane %u is (%f, %f, %f) Newtons\n", i, F_CGS_TO_SI * reaction_forces[0],
                   F_CGS_TO_SI * reaction_forces[1], F_CGS_TO_SI * reaction_forces[2]);
        }
    }
    // delimiter to make for easier reading
    printf("--------------------------------\n");
    ofile << outstrstream.str();
}

// -----------------------------------------------------------------------------
// Demo for settling a monodisperse collection of shperes in a rectangular box.
// The units are always cm/s/g[L/T/M].
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    // Some of the default values might be overwritten by user via command line
    if (argc < 2 || argc > 2 && argc != num_args_full || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }

    if (argc == num_args_full) {
        params.run_mode = std::atoi(argv[2]);
        params.box_Z = std::atof(argv[3]);
        params.output_dir = std::string(argv[4]);
        printf("new parameters: run_mode is %d, height is %f, %s\n", params.run_mode, params.box_Z,
               params.output_dir.c_str());
    }

    // Setup simulation
    ChSystemGranular_MonodisperseSMC settlingExperiment(params.sphere_radius, params.sphere_density,
                                                        make_float3(params.box_X, params.box_Y, params.box_Z));
    settlingExperiment.setPsiFactors(params.psi_T, params.psi_h, params.psi_L);

    settlingExperiment.set_K_n_SPH2SPH(params.normalStiffS2S);
    settlingExperiment.set_K_n_SPH2WALL(params.normalStiffS2W);
    settlingExperiment.set_Gamma_n_SPH2SPH(params.normalDampS2S);
    settlingExperiment.set_Gamma_n_SPH2WALL(params.normalDampS2W);

    settlingExperiment.set_K_t_SPH2SPH(params.tangentStiffS2S);
    settlingExperiment.set_K_t_SPH2WALL(params.tangentStiffS2W);
    settlingExperiment.set_Gamma_t_SPH2SPH(params.tangentDampS2S);
    settlingExperiment.set_Gamma_t_SPH2WALL(params.tangentDampS2W);
    settlingExperiment.set_static_friction_coeff_SPH2SPH(params.static_friction_coeff);
    settlingExperiment.set_static_friction_coeff_SPH2WALL(params.static_friction_coeff);

    settlingExperiment.set_Cohesion_ratio(params.cohesion_ratio);
    settlingExperiment.set_Adhesion_ratio_S2W(params.adhesion_ratio_s2w);
    settlingExperiment.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);
    settlingExperiment.setOutputDirectory(params.output_dir);
    settlingExperiment.setOutputMode(params.write_mode);

    std::vector<ChVector<float>> body_points;
    // body_points.push_back(ChVector<float>(0., 0., 0. - (params.box_Z / 2.f - 1.01 * params.sphere_radius)));
    // body_points.push_back(ChVector<float>(2., 0., 0. - (params.box_Z / 2.f - 1.01 * params.sphere_radius)));
    // body_points.push_back(ChVector<float>(1., 1.732, 0. - (params.box_Z / 2.f - 1.01 * params.sphere_radius)));
    // body_points.push_back(ChVector<float>(1., .5774, 2.05 - (params.box_Z / 2.f - 1.01 * params.sphere_radius)));
    // body_points.push_back(ChVector<float>(params.box_X / 2.f - 2 * params.sphere_radius,
    //                                       params.box_Y / 2.f - 2 * params.sphere_radius, 3));
    // body_points.push_back(ChVector<float>(-(params.box_X / 2.f - 2 * params.sphere_radius),
    //
    //                                       -(params.box_Y / 2.f - 2 * params.sphere_radius), 3));

    {
        // fill box, layer by layer
        ChVector<> hdims(params.box_X / 2.f - 3, params.box_Y / 2.f - 3, params.box_Z / 2.f - 3);
        ChVector<> center(0, 0, 0);

        // Fill box with bodies
        body_points = PDLayerSampler_BOX<float>(center, hdims, 2. * params.sphere_radius, 1.01);

        // utils::HCPSampler<float> sampler(2.2 * params.sphere_radius);
        // body_points = sampler.SampleBox(center, hdims);
    }
    settlingExperiment.setParticlePositions(body_points);

    // face in upwards
    float bottom_plane_normal_X[3] = {1, 0, 0};
    float top_plane_normal_X[3] = {-1, 0, 0};
    float bottom_plane_normal_Y[3] = {0, 1, 0};
    float top_plane_normal_Y[3] = {0, -1, 0};
    float bottom_plane_normal_Z[3] = {0, 0, 1};
    float top_plane_normal_Z[3] = {0, 0, -1};

    // where to place confining planes
    float box_dist_X = params.box_X / 2.f - 2.f;
    float box_dist_Y = params.box_Y / 2.f - 2.f;
    float box_dist_Z = params.box_Z / 2.f - 2.f;

    // put a plane on the box walls
    float box_bottom_X[3] = {-box_dist_X, 0, 0};
    float box_top_X[3] = {box_dist_X, 0, 0};
    float box_bottom_Y[3] = {0, -box_dist_Y, 0};
    float box_top_Y[3] = {0, box_dist_Y, 0};
    float box_bottom_Z[3] = {0, 0, -box_dist_Z};
    float box_top_Z[3] = {0, 0, box_dist_Z};

    size_t bottom_plane_bc_id_X = settlingExperiment.Create_BC_Plane(box_bottom_X, bottom_plane_normal_X, true);
    size_t top_plane_bc_id_X = settlingExperiment.Create_BC_Plane(box_top_X, top_plane_normal_X, true);
    size_t bottom_plane_bc_id_Y = settlingExperiment.Create_BC_Plane(box_bottom_Y, bottom_plane_normal_Y, true);
    size_t top_plane_bc_id_Y = settlingExperiment.Create_BC_Plane(box_top_Y, top_plane_normal_Y, true);
    size_t bottom_plane_bc_id_Z = settlingExperiment.Create_BC_Plane(box_bottom_Z, bottom_plane_normal_Z, true);
    size_t top_plane_bc_id_Z = settlingExperiment.Create_BC_Plane(box_top_Z, top_plane_normal_Z, true);

    bc_ids.push_back(bottom_plane_bc_id_X);
    bc_ids.push_back(top_plane_bc_id_X);
    bc_ids.push_back(bottom_plane_bc_id_Y);
    bc_ids.push_back(top_plane_bc_id_Y);
    bc_ids.push_back(bottom_plane_bc_id_Z);
    bc_ids.push_back(top_plane_bc_id_Z);

    bc_names.push_back("bottom_plane_bc_X");
    bc_names.push_back("top_plane_bc_X");
    bc_names.push_back("bottom_plane_bc_Y");
    bc_names.push_back("top_plane_bc_Y");
    bc_names.push_back("bottom_plane_bc_Z");
    bc_names.push_back("top_plane_bc_Z");

    settlingExperiment.set_timeIntegrator(GRAN_TIME_INTEGRATOR::FORWARD_EULER);
    settlingExperiment.set_timeStepping(GRAN_TIME_STEPPING::FIXED);

    switch (params.run_mode) {
        case run_mode::MULTI_STEP:
            settlingExperiment.set_friction_mode(GRAN_FRICTION_MODE::MULTI_STEP);
            settlingExperiment.set_timeIntegrator(GRAN_TIME_INTEGRATOR::FORWARD_EULER);
            break;
        case run_mode::ONE_STEP:
            settlingExperiment.set_friction_mode(GRAN_FRICTION_MODE::SINGLE_STEP);
            settlingExperiment.set_timeIntegrator(GRAN_TIME_INTEGRATOR::FORWARD_EULER);
            break;
        case run_mode::FRICLESS_CHUNG:
            settlingExperiment.set_friction_mode(GRAN_FRICTION_MODE::FRICTIONLESS);
            settlingExperiment.set_timeIntegrator(GRAN_TIME_INTEGRATOR::CHUNG);
            break;

        case run_mode::FRICLESS_CD:
            settlingExperiment.set_friction_mode(GRAN_FRICTION_MODE::FRICTIONLESS);
            settlingExperiment.set_timeIntegrator(GRAN_TIME_INTEGRATOR::CENTERED_DIFFERENCE);
            break;

        case run_mode::FRICTIONLESS:
            settlingExperiment.set_friction_mode(GRAN_FRICTION_MODE::FRICTIONLESS);
            settlingExperiment.set_timeIntegrator(GRAN_TIME_INTEGRATOR::FORWARD_EULER);
            break;

        default:
            // fall through to frictionless as default
            settlingExperiment.set_friction_mode(GRAN_FRICTION_MODE::FRICTIONLESS);
    }

    settlingExperiment.set_fixed_stepSize(params.step_size);

    filesystem::create_directory(filesystem::path(params.output_dir));
    filesystem::create_directory(filesystem::path(params.output_dir + "/forces"));

    settlingExperiment.set_BD_Fixed(true);

    settlingExperiment.setVerbose(params.verbose);
    settlingExperiment.initialize();

    // number of times to capture force data per second
    int captures_per_second = 50;
    // number of times to capture force before we capture a frame
    int captures_per_frame = 2;

    // assume we run for at least one frame
    float frame_step = 1. / captures_per_second;
    float curr_time = 0;

    std::cout << "capture step is " << frame_step << std::endl;

    float total_system_mass = 4. / 3. * CH_C_PI * params.sphere_density * params.sphere_radius * params.sphere_radius *
                              params.sphere_radius * body_points.size();
    printf("total system mass is %f kg \n", total_system_mass * M_CGS_TO_SI);

    // write an initial frame
    char filename[100];
    printf("rendering frame %u\n", currframe);
    sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe);
    settlingExperiment.writeFile(std::string(filename));

    // write mesh transforms for ospray renderer
    {
        std::ofstream meshfile{params.output_dir + "/BD_Box_mesh.csv"};
        std::ostringstream outstream;
        outstream << "mesh_name,dx,dy,dz,x1,x2,x3,y1,y2,y3,z1,z2,z3\n";
        writeBoxMesh(outstream);
        meshfile << outstream.str();
    }
    std::cout << "frame step is " << frame_step << std::endl;

    currframe++;

    // Run settling experiments
    while (curr_time < params.time_end) {
        printf("curr time is %f\n", curr_time);
        writeForcesFile(settlingExperiment);
        settlingExperiment.advance_simulation(frame_step);
        curr_time += frame_step;

        // if this frame is a render frame
        if (currcapture % captures_per_frame == 0) {
            printf("rendering frame %u\n", currframe);
            sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
            settlingExperiment.writeFile(std::string(filename));
        }
    }

    return 0;
}
