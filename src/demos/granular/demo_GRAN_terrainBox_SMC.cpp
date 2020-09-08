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
// Authors: Conlain Kelly, Nic Olsen
// =============================================================================
// Simple Chrono::Granular settling experiment which allows for sweeping various
// simulation parameters to produce scaling analyses.
// =============================================================================

#include <iostream>
#include <string>
#include <cmath>
#include <chrono>
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_granular/api/ChApiGranularChrono.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/utils/ChGranularJsonParser.h"
#include "ChGranularDemoUtils.hpp"
#include "chrono/utils/ChUtilsSamplers.h"

using namespace chrono;
using namespace chrono::granular;

// expected number of args for param sweep
constexpr int num_args_full = 6;

// -----------------------------------------------------------------------------
// Show command line usage
// -----------------------------------------------------------------------------
void ShowUsage(std::string name) {
    std::cout << "usage: " + name + " <json_file> [<radius> <run_mode> <box_width> <output_dir>]" << std::endl;
    std::cout << "must have either 1 or " << num_args_full - 1 << " arguments" << std::endl;
}

enum run_mode { FRICTIONLESS = 0, ONE_STEP = 1, MULTI_STEP = 2 };

int main(int argc, char* argv[]) {
    sim_param_holder params;

    // Some of the default values are overwritten by user via command line
    if (argc < 2 || (argc > 2 && argc != num_args_full) || ParseJSON(argv[1], params) == false) {
        ShowUsage(argv[0]);
        return 1;
    }

    if (argc == num_args_full) {
        params.sphere_radius = std::stof(argv[2]);
        params.run_mode = std::atoi(argv[3]);
        params.box_Y = std::stof(argv[4]);
        params.box_X = params.box_Y;
        params.output_dir = std::string(argv[5]);
        printf("new parameters: r is %f, run_mode is %d, width is %f, %s\n", params.sphere_radius, params.run_mode,
               params.box_Y, params.output_dir.c_str());
    }

    // Setup simulation
    ChSystemGranularSMC gran_sys(params.sphere_radius, params.sphere_density,
                                 make_float3(params.box_X, params.box_Y, params.box_Z));

    ChGranularSMC_API apiSMC;
    apiSMC.setGranSystem(&gran_sys);

    gran_sys.setPsiFactors(params.psi_T, params.psi_L);

    gran_sys.set_K_n_SPH2SPH(params.normalStiffS2S);
    gran_sys.set_K_n_SPH2WALL(params.normalStiffS2W);
    gran_sys.set_Gamma_n_SPH2SPH(params.normalDampS2S);
    gran_sys.set_Gamma_n_SPH2WALL(params.normalDampS2W);

    gran_sys.set_K_t_SPH2SPH(params.tangentStiffS2S);
    gran_sys.set_K_t_SPH2WALL(params.tangentStiffS2W);
    gran_sys.set_Gamma_t_SPH2SPH(params.tangentDampS2S);
    gran_sys.set_Gamma_t_SPH2WALL(params.tangentDampS2W);

    gran_sys.set_static_friction_coeff_SPH2SPH(params.static_friction_coeffS2S);
    gran_sys.set_static_friction_coeff_SPH2WALL(params.static_friction_coeffS2W);

    gran_sys.set_Cohesion_ratio(params.cohesion_ratio);
    gran_sys.set_Adhesion_ratio_S2W(params.adhesion_ratio_s2w);
    gran_sys.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);
    gran_sys.setOutputMode(params.write_mode);

    gran_sys.set_rolling_mode(GRAN_ROLLING_MODE::NO_RESISTANCE);

    std::vector<ChVector<float>> body_points;

    {
        // fill box, layer by layer
        ChVector<> hdims(params.box_X / 2.f - 2 * params.sphere_radius, params.box_Y / 2.f - 2 * params.sphere_radius,
                         params.box_Z / 2.f - 2 * params.sphere_radius);
        ChVector<> center(0, 0, 0);

        utils::HCPSampler<float> sampler(2.2f * params.sphere_radius);

        body_points = sampler.SampleBox(center, hdims);
    }

    apiSMC.setElemsPositions(body_points);
    std::cout << "Added " << body_points.size() << std::endl;

    switch (params.run_mode) {
        case run_mode::MULTI_STEP:
            gran_sys.set_friction_mode(GRAN_FRICTION_MODE::MULTI_STEP);
            gran_sys.set_K_t_SPH2SPH(params.tangentStiffS2S);
            gran_sys.set_K_t_SPH2WALL(params.tangentStiffS2W);
            gran_sys.set_Gamma_t_SPH2SPH(params.tangentDampS2S);
            gran_sys.set_Gamma_t_SPH2WALL(params.tangentDampS2W);
            break;
        case run_mode::ONE_STEP:
            gran_sys.set_friction_mode(GRAN_FRICTION_MODE::SINGLE_STEP);
            gran_sys.set_K_t_SPH2SPH(params.tangentStiffS2S);
            gran_sys.set_K_t_SPH2WALL(params.tangentStiffS2W);
            gran_sys.set_Gamma_t_SPH2SPH(params.tangentDampS2S);
            gran_sys.set_Gamma_t_SPH2WALL(params.tangentDampS2W);
            break;
        case run_mode::FRICTIONLESS:
            gran_sys.set_friction_mode(GRAN_FRICTION_MODE::FRICTIONLESS);
            break;
        default:
            std::cout << "Invalid run mode" << std::endl;
            return 1;
    }

    gran_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::EXTENDED_TAYLOR);
    gran_sys.set_fixed_stepSize(params.step_size);
    if (params.write_mode != GRAN_OUTPUT_MODE::NONE) {
        filesystem::create_directory(filesystem::path(params.output_dir));
    }
    gran_sys.set_BD_Fixed(true);

    gran_sys.setVerbose(params.verbose);
    gran_sys.initialize();

    int fps = 50;
    float frame_step = 1.f / fps;
    float curr_time = 0;
    int currframe = 0;

    // write an initial frame
    char filename[100];
    sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
    gran_sys.writeFile(std::string(filename));

    std::cout << "frame step is " << frame_step << std::endl;

    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    while (curr_time < params.time_end) {
        gran_sys.advance_simulation(frame_step);
        curr_time += frame_step;
        printf("rendering frame %u\n", currframe);
        sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
        gran_sys.writeFile(std::string(filename));
    }
    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_sec = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
    std::cout << time_sec.count() << " seconds" << std::endl;

    return 0;
}