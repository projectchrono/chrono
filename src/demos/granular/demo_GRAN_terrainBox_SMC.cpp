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
constexpr int num_args_full = 6;

// -----------------------------------------------------------------------------
// Show command line usage
// -----------------------------------------------------------------------------
void ShowUsage() {
    cout << "usage: ./demo_GRAN_DamBreak <json_file> [<radius> <run_mode> <box_width> <output_dir>]" << endl;
    cout << "must have either 1 or " << num_args_full - 1 << " arguments" << endl;
}

enum run_mode { FRICTIONLESS = 0, ONE_STEP = 1, MULTI_STEP = 2 };

// -----------------------------------------------------------------------------
// Demo for settling a monodisperse collection of shperes in a rectangular box.
// The units are always cm/s/g[L/T/M].
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    sim_param_holder params;

    // Some of the default values might be overwritten by user via command line
    if (argc < 2 || argc > 2 && argc != num_args_full || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }

    if (argc == num_args_full) {
        params.sphere_radius = std::atof(argv[2]);
        params.run_mode = std::atoi(argv[3]);
        params.box_Y = std::atof(argv[4]);
        params.box_X = params.box_Y;
        params.output_dir = std::string(argv[5]);
        printf("new parameters: r is %f, run_mode is %d, width is %f, %s\n", params.sphere_radius, params.run_mode,
               params.box_Y, params.output_dir.c_str());
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

    settlingExperiment.set_static_friction_coeff(params.static_friction_coeff);

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
        ChVector<> hdims(params.box_X / 8.f - 2 * params.sphere_radius, params.box_Y / 8.f - 2 * params.sphere_radius,
                         params.box_Z / 8.f - 2 * params.sphere_radius);
        ChVector<> center(0, 0, 0);

        // Fill box with bodies
        body_points = PDLayerSampler_BOX<float>(center, hdims, 2. * params.sphere_radius, 1.01);

        // utils::HCPSampler<float> sampler(2.2 * params.sphere_radius);
        //
        // body_points = sampler.SampleBox(center, hdims);
    }
    std::vector<ChVector<float>> first_points;
    first_points.push_back(body_points.at(0));
    first_points.push_back(body_points.at(body_points.size() / 2));
    first_points.push_back(body_points.at(body_points.size() - 1));
    printf("particle is at %f, %f, %f\n", first_points[0].x(), first_points[0].y(), first_points[0].z());
    printf("particle is at %f, %f, %f\n", first_points[1].x(), first_points[1].y(), first_points[1].z());
    printf("particle is at %f, %f, %f\n", first_points[2].x(), first_points[2].y(), first_points[2].z());
    settlingExperiment.setParticlePositions(body_points);

    switch (params.run_mode) {
        case run_mode::MULTI_STEP:
            settlingExperiment.set_friction_mode(GRAN_FRICTION_MODE::MULTI_STEP);
            settlingExperiment.set_K_t_SPH2SPH(params.tangentStiffS2S);
            settlingExperiment.set_K_t_SPH2WALL(params.tangentStiffS2W);
            settlingExperiment.set_Gamma_t_SPH2SPH(params.tangentDampS2S);
            settlingExperiment.set_Gamma_t_SPH2WALL(params.tangentDampS2W);
            break;
        case run_mode::ONE_STEP:
            settlingExperiment.set_friction_mode(GRAN_FRICTION_MODE::SINGLE_STEP);
            settlingExperiment.set_K_t_SPH2SPH(params.tangentStiffS2S);
            settlingExperiment.set_K_t_SPH2WALL(params.tangentStiffS2W);
            settlingExperiment.set_Gamma_t_SPH2SPH(params.tangentDampS2S);
            settlingExperiment.set_Gamma_t_SPH2WALL(params.tangentDampS2W);
            break;
        case run_mode::FRICTIONLESS:
        default:
            // fall through to frictionless as default
            settlingExperiment.set_friction_mode(GRAN_FRICTION_MODE::FRICTIONLESS);
    }

    settlingExperiment.set_timeStepping(GRAN_TIME_STEPPING::FIXED);
    settlingExperiment.set_ForceModel(GRAN_FORCE_MODEL::HOOKE);
    settlingExperiment.set_timeIntegrator(GRAN_TIME_INTEGRATOR::EXTENDED_TAYLOR);
    settlingExperiment.set_fixed_stepSize(params.step_size);

    filesystem::create_directory(filesystem::path(params.output_dir));

    settlingExperiment.set_BD_Fixed(true);

    settlingExperiment.setVerbose(params.verbose);
    settlingExperiment.initialize();

    int fps = 50;
    // assume we run for at least one frame
    // float frame_step = params.step_size * 100.f;
    float frame_step = 1.f / fps;
    float curr_time = 0;
    int currframe = 0;

    // write an initial frame
    char filename[100];
    sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
    settlingExperiment.writeFile(std::string(filename));

    std::cout << "frame step is " << frame_step << std::endl;

    // Run settling experiments
    while (curr_time < params.time_end) {
        settlingExperiment.advance_simulation(frame_step);
        curr_time += frame_step;
        printf("rendering frame %u\n", currframe);
        sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
        settlingExperiment.writeFile(std::string(filename));
    }

    return 0;
}
