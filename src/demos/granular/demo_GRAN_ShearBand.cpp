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
// Authors: Conlain Kelly
// =============================================================================
// Chrono::Granular simulation of a rectangular bed of granular material which
// is first let to settle and then compressed by advancing one of the box walls
// into the material.
// =============================================================================

#include <iostream>
#include <string>
#ifdef _WINDOWS
#define _USE_MATH_DEFINES
#include <math.h>
#endif
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/utils/ChGranularJsonParser.h"
#include "chrono/utils/ChUtilsSamplers.h"

using namespace chrono;
using namespace chrono::granular;
using std::cout;
using std::endl;
using std::string;

// -----------------------------------------------------------------------------
// Show command line usage
// -----------------------------------------------------------------------------
void ShowUsage() {
    cout << "usage: ./demo_GRAN_ShearBand <json_file>" << endl;
}

int main(int argc, char* argv[]) {
    sim_param_holder params;

    // Some of the default values might be overwritten by user via command line
    if (argc != 2 || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }

    // Setup simulation
    ChSystemGranularSMC gran_sys(params.sphere_radius, params.sphere_density,
                                 make_float3(params.box_X, params.box_Y, params.box_Z));
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
    filesystem::create_directory(filesystem::path(params.output_dir));

    // fill box, layer by layer
    ChVector<> hdims(params.box_X / 2.f - 1.2, params.box_Y / 2.f - 1.2, params.box_Z / 10.f - 1.2);
    ChVector<> center(0, 0, -params.box_Z / 2.f + params.box_Z / 10.f);

    // Fill box with bodies
    std::vector<ChVector<float>> body_points =
        utils::PDLayerSampler_BOX<float>(center, hdims, 2. * params.sphere_radius, 1.05);

    gran_sys.setParticlePositions(body_points);

    // Set the position of the BD
    gran_sys.set_BD_Fixed(true);

    gran_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::FORWARD_EULER);
    gran_sys.set_friction_mode(GRAN_FRICTION_MODE::MULTI_STEP);
    gran_sys.set_fixed_stepSize(params.step_size);

    gran_sys.setVerbose(params.verbose);

    // start outside BD by 10 cm
    float plane_pos[3] = {-params.box_X / 2 - 10, 0, 0};
    float plane_normal[3] = {1, 0, 0};

    size_t plane_bc_id = gran_sys.Create_BC_Plane(plane_pos, plane_normal, false);

    // Function prescibing the motion of the advancing plane.
    // Begins outside of the domain.
    std::function<double3(float)> plane_pos_func = [&params](float t) {
        double3 pos = {0, 0, 0};

        // move at 10 cm/s
        constexpr float vel = 10;

        // after 1 second the plane will be at the edge of the BD, and will continue in thereafter
        pos.x = vel * t;

        return pos;
    };

    gran_sys.initialize();

    gran_sys.set_BC_offset_function(plane_bc_id, plane_pos_func);

    int fps = 50;
    // assume we run for at least one frame
    float frame_step = 1.0f / fps;
    float curr_time = 0;
    int currframe = 0;

    char filename[100];
    sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
    gran_sys.writeFile(std::string(filename));

    std::cout << "frame step is " << frame_step << std::endl;

    // Run settling experiments
    while (curr_time < params.time_end) {
        gran_sys.advance_simulation(frame_step);
        curr_time += frame_step;
        printf("rendering frame %u\n", currframe);
        char filename[100];
        sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
        gran_sys.writeFile(std::string(filename));
    }

    return 0;
}