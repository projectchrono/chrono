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
//  - there is no friction
//
// The global reference frame has X to the right, Y into the screen, Z up.
// The global reference frame located in the left lower corner, close to the viewer.
// =============================================================================

#include <iostream>
#include <string>
#include "chrono/core/ChFileutils.h"
#include "chrono_granular/physics/ChGranular.h"
#include "ChGranular_json_parser.hpp"
#include "ChGranularDemoUtils.hpp"
#include "chrono/utils/ChUtilsSamplers.h"

using namespace chrono;
using namespace chrono::granular;
using std::cout;
using std::endl;
using std::string;

enum { SETTLING = 0, WAVETANK = 1, BOUNCING_PLATE = 2 };

// -----------------------------------------------------------------------------
// Show command line usage
// -----------------------------------------------------------------------------
void ShowUsage() {
    cout << "usage: ./demo_GRAN_BoxSettlNoFirc_SMC <json_file>" << endl;
}

// -----------------------------------------------------------------------------
// Demo for settling a monodisperse collection of shperes in a rectangular box.
// There is no friction. The units are always cm/s/g[L/T/M].
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    GRAN_TIME_STEPPING step_mode = GRAN_TIME_STEPPING::FIXED;
    int run_mode = SETTLING;

    sim_param_holder params;

    // Some of the default values might be overwritten by user via command line
    if (argc != 2 || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }

    // Setup simulation
    ChSystemGranular_MonodisperseSMC settlingExperiment(params.sphere_radius, params.sphere_density);
    settlingExperiment.setBOXdims(params.box_X, params.box_Y, params.box_Z);
    settlingExperiment.set_K_n_SPH2SPH(params.normalStiffS2S);
    settlingExperiment.set_K_n_SPH2WALL(params.normalStiffS2W);
    settlingExperiment.set_Gamma_n_SPH2SPH(params.normalDampS2S);
    settlingExperiment.set_Gamma_n_SPH2WALL(params.normalDampS2W);

    settlingExperiment.set_K_t_SPH2SPH(params.tangentStiffS2S);
    settlingExperiment.set_K_t_SPH2WALL(params.tangentStiffS2W);
    settlingExperiment.set_Gamma_t_SPH2SPH(params.tangentDampS2S);
    settlingExperiment.set_Gamma_t_SPH2WALL(params.tangentDampS2W);

    settlingExperiment.set_Cohesion_ratio(params.cohesion_ratio);
    settlingExperiment.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);
    settlingExperiment.setOutputDirectory(params.output_dir);
    settlingExperiment.setOutputMode(params.write_mode);

    settlingExperiment.set_BD_Fixed(true);

    // Fill box with bodies
    std::vector<ChVector<float>> body_points;

    chrono::utils::PDSampler<float> sampler(2.05 * params.sphere_radius);

    float center_pt[3] = {0.f, 0.f, -2.05f * params.sphere_radius};

    // width we want to fill to
    double fill_width = params.box_Z / 4;
    // height that makes this width above the cone
    double fill_height = fill_width;

    // fill to top
    double fill_top = params.box_Z / 2 - 2.05 * params.sphere_radius;
    double fill_bottom = fill_top + 2.05 * params.sphere_radius - fill_height + center_pt[2];

    printf("width is %f, bot is %f, top is %f, height is %f\n", fill_width, fill_bottom, fill_top, fill_height);
    // fill box, layer by layer
    ChVector<> hdims(fill_width - params.sphere_radius, fill_width - params.sphere_radius, 0);
    ChVector<> center(0, 0, fill_bottom);
    // shift up for bottom of box
    center.z() += 3 * params.sphere_radius;

    while (center.z() < fill_top) {
        std::cout << "Create layer at " << center.z() << std::endl;
        auto points = sampler.SampleCylinderZ(center, fill_width - params.sphere_radius, 0);
        body_points.insert(body_points.end(), points.begin(), points.end());
        center.z() += 2.05 * params.sphere_radius;
    }

    settlingExperiment.setParticlePositions(body_points);

    settlingExperiment.set_timeStepping(GRAN_TIME_STEPPING::FIXED);
    settlingExperiment.set_timeIntegrator(GRAN_TIME_INTEGRATOR::FORWARD_EULER);
    settlingExperiment.set_fixed_stepSize(params.step_size);

    ChFileutils::MakeDirectory(params.output_dir.c_str());

    float cone_slope = 1.0;
    // float hdims[3] = {2.f, 2.f, 2.f};

    settlingExperiment.setVerbose(params.verbose);
    // Finalize settings and initialize for runtime
    settlingExperiment.Create_BC_Cone(center_pt, cone_slope, params.box_Z, center_pt[2] + 10 * params.sphere_radius,
                                      true);
    settlingExperiment.initialize();

    int fps = 100;
    // assume we run for at least one frame
    float frame_step = 1. / fps;
    float curr_time = 0;
    int currframe = 0;

    std::cout << "frame step is " << frame_step << std::endl;

    // Run settling experiments
    while (curr_time < params.time_end) {
        settlingExperiment.advance_simulation(frame_step);
        curr_time += frame_step;
        printf("rendering frame %u\n", currframe);
        char filename[100];
        sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
        settlingExperiment.writeFileUU(std::string(filename));
    }

    return 0;
}
