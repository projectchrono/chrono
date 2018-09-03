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
    GRN_TIME_STEPPING step_mode = GRN_TIME_STEPPING::FIXED;
    int run_mode = SETTLING;

    sim_param_holder params;

    // Some of the default values might be overwritten by user via command line
    if (argc != 2 || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }

    // Setup simulation
    ChSystemGranularMonodisperse_SMC_Frictionless settlingExperiment(params.sphere_radius, params.sphere_density);
    settlingExperiment.setBOXdims(params.box_X, params.box_Y, params.box_Z);
    settlingExperiment.set_K_n_SPH2SPH(params.normalStiffS2S);
    settlingExperiment.set_K_n_SPH2WALL(params.normalStiffS2W);
    settlingExperiment.set_Gamma_n_SPH2SPH(params.normalDampS2S);
    settlingExperiment.set_Cohesion_ratio(params.cohesion_ratio);
    settlingExperiment.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);
    settlingExperiment.setOutputDirectory(params.output_dir);
    settlingExperiment.setOutputMode(params.write_mode);

    settlingExperiment.set_timeStepping(params.step_mode);
    settlingExperiment.set_fixed_stepSize(params.step_size);
    // settlingExperiment.set_max_adaptive_stepSize(1e-4);

    settlingExperiment.setFillBounds(-1.f, 1.f, -1.f, 1.f, -1.f, 1.f);

    ChFileutils::MakeDirectory(params.output_dir.c_str());

    // TODO clean up this API
    // Prescribe a custom position function for the X direction. Note that this MUST be continuous or the simulation
    // will not be stable. The value is in multiples of box half-lengths in that direction, so an x-value of 1 means
    // that the box will be centered at x = box_size_X
    std::function<double(double)> posFunWave = [](double t) {
        // Start oscillating at t = .5s
        double t0 = .5;
        double freq = .1 * M_PI;

        if (t < t0) {
            return -.5;
        } else {
            return (-.5 + .5 * std::sin((t - t0) * freq));
        }
    };
    // Stay centered at origin
    std::function<double(double)> posFunStill = [](double t) { return -.5; };

    std::function<double(double)> posFunZBouncing = [](double t) {
        // Start oscillating at t = .5s
        double t0 = .5;
        double freq = 20 * M_PI;

        if (t < t0) {
            return -.5;
        } else {
            return (-.5 + .01 * std::sin((t - t0) * freq));
        }
    };

    switch (params.run_mode) {
        case SETTLING:
            settlingExperiment.setBDPositionFunction(posFunStill, posFunStill, posFunStill);
            settlingExperiment.set_BD_Fixed(true);
            break;
        case WAVETANK:
            settlingExperiment.setBDPositionFunction(posFunStill, posFunWave, posFunStill);
            settlingExperiment.set_BD_Fixed(false);
            break;
        case BOUNCING_PLATE:
            settlingExperiment.setBDPositionFunction(posFunStill, posFunStill, posFunZBouncing);
            settlingExperiment.set_BD_Fixed(false);
            break;
    }

    settlingExperiment.setVerbose(params.verbose);
    // Finalize settings and initialize for runtime
    settlingExperiment.initialize();

    int fps = 50;
    // assume we run for at least one frame
    float frame_step = 1.0f / fps;
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
        settlingExperiment.checkSDCounts(std::string(filename), true, false);
    }

    return 0;
}
