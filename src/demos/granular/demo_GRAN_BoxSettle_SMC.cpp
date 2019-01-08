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
// The global reference frame has X to the right, Y into the screen, Z up.
// The global reference frame located in the left lower corner, close to the viewer.
// =============================================================================

#include <iostream>
#include <string>
#ifdef _WINDOWS
#define _USE_MATH_DEFINES
#include <math.h>
#endif
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
// The units are always cm/s/g[L/T/M].
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
    settlingExperiment.setPsiFactors(params.psi_T, params.psi_h, params.psi_L);

    settlingExperiment.setBOXdims(params.box_X, params.box_Y, params.box_Z);
    settlingExperiment.set_K_n_SPH2SPH(params.normalStiffS2S);
    settlingExperiment.set_K_n_SPH2WALL(params.normalStiffS2W);
    settlingExperiment.set_Gamma_n_SPH2SPH(params.normalDampS2S);
    settlingExperiment.set_Gamma_n_SPH2WALL(params.normalDampS2W);

    settlingExperiment.set_Cohesion_ratio(params.cohesion_ratio);
    settlingExperiment.set_Adhesion_ratio_S2W(params.adhesion_ratio_s2w);
    settlingExperiment.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);
    settlingExperiment.setOutputDirectory(params.output_dir);
    settlingExperiment.setOutputMode(params.write_mode);

    // fill box, layer by layer
    ChVector<> hdims(params.box_X / 4.f - 1.05 * params.sphere_radius, params.box_Y / 4.f - 1.05 * params.sphere_radius,
                     params.box_Z / 6.f - 1.05 * params.sphere_radius);
    ChVector<> center(0, 0, params.box_Z * 2. / 6.);

    // Fill box with bodies
    std::vector<ChVector<float>> body_points =
        PDLayerSampler_BOX<float>(center, hdims, 2. * params.sphere_radius, 1.05);
    std::vector<ChVector<float>> first_points;
    first_points.push_back(body_points.at(0));
    first_points.push_back(body_points.at(1));

    settlingExperiment.setParticlePositions(body_points);

    settlingExperiment.set_timeStepping(GRAN_TIME_STEPPING::FIXED);
    // settlingExperiment.set_timeIntegrator(GRAN_TIME_INTEGRATOR::CHUNG);
    settlingExperiment.set_timeIntegrator(GRAN_TIME_INTEGRATOR::FORWARD_EULER);
    settlingExperiment.set_friction_mode(GRAN_FRICTION_MODE::SINGLE_STEP);
    settlingExperiment.set_fixed_stepSize(params.step_size);

    ChFileutils::MakeDirectory(params.output_dir.c_str());

    settlingExperiment.set_BD_Fixed(true);

    settlingExperiment.setVerbose(params.verbose);
    // Finalize settings and initialize for runtime
    float point[3] = {0, 0, -params.box_Z * 2.f / 6.f};
    float normal[3] = {0, 0, 1};
    settlingExperiment.Create_BC_Plane(point, normal, false);
    settlingExperiment.initialize();
    // settlingExperiment.Create_BC_AABox(hdims, center_pt, false, false);
    // settlingExperiment.Create_BC_Sphere(center_pt, 3.f, true, false);
    // settlingExperiment.Create_BC_Cone_Z(center_pt, 1, params.box_Z, center_pt[2] + 10 * params.sphere_radius, true,
    // false);

    int fps = 100;
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
        settlingExperiment.writeFileUU(std::string(filename));
    }

    return 0;
}
