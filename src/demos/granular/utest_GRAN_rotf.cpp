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
#include "chrono_granular/physics/ChGranular.h"
#include "ChGranular_json_parser.hpp"

using namespace chrono;
using namespace chrono::granular;
using std::cout;
using std::endl;
using std::string;

void ShowUsage() {
    cout << "usage: ./demo_GRAN_utest_GRAN_rotf <json_file>" << endl;
}

int main(int argc, char* argv[]) {
    sim_param_holder params;

    // Some of the default values might be overwritten by user via command line
    if (argc != 2 || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }

    ChSystemGranular_MonodisperseSMC gran_sys(params.sphere_radius, params.sphere_density,
                                              make_float3(params.box_X, params.box_Y, params.box_Z));
    gran_sys.setPsiFactors(params.psi_T, params.psi_h, params.psi_L);
    gran_sys.set_K_n_SPH2SPH(params.normalStiffS2S);
    gran_sys.set_K_n_SPH2WALL(params.normalStiffS2W);
    gran_sys.set_Gamma_n_SPH2SPH(params.normalDampS2S);
    gran_sys.set_Gamma_n_SPH2WALL(params.normalDampS2W);

    gran_sys.set_K_t_SPH2SPH(params.tangentStiffS2S);
    gran_sys.set_K_t_SPH2WALL(params.tangentStiffS2W);
    gran_sys.set_Gamma_t_SPH2SPH(params.tangentDampS2S);
    gran_sys.set_Gamma_t_SPH2WALL(params.tangentDampS2W);

    gran_sys.set_Cohesion_ratio(params.cohesion_ratio);
    gran_sys.set_Adhesion_ratio_S2W(params.adhesion_ratio_s2w);
    gran_sys.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);
    gran_sys.setOutputDirectory(params.output_dir);
    gran_sys.setOutputMode(params.write_mode);
    gran_sys.set_static_friction_coeff(0.7);

    gran_sys.set_friction_mode(chrono::granular::GRAN_FRICTION_MODE::MULTI_STEP);
    gran_sys.set_rolling_mode(chrono::granular::GRAN_ROLLING_MODE::NAIVE);

    std::vector<ChVector<float>> points;
    ChVector<float> sphere_pos(-params.box_X / 2 + 2 * params.sphere_radius, 0,
                               -params.box_Z / 2 + 5 * params.sphere_radius);
    points.push_back(sphere_pos);
    gran_sys.setParticlePositions(points);

    float ramp_angle = CH_C_PI / 8;
    // Vector up ramp
    float dx = 1.f;
    float dz = std::tan(ramp_angle);

    // Ramp normal
    dz = dx / dz;
    dx = 1.f;
    float len = std::sqrt(dx * dx + dz * dz);
    dx = dx / len;
    dz = dz / len;

    float plane_n[] = {dx, 0.f, dz};
    ChVector<float> p_n(plane_n[0], plane_n[1], plane_n[2]);
    cout << "p_n" << p_n.x() << " " << p_n.y() << " " << p_n.z() << endl;

    ChVector<float> p_pos = sphere_pos - 1.1f * (float)params.sphere_radius * p_n;
    cout << "p_pos" << p_pos.x() << " " << p_pos.y() << " " << p_pos.z() << endl;

    float plane_pos[] = {p_pos.x(), p_pos.y(), p_pos.z()};

    gran_sys.Create_BC_Plane(plane_pos, plane_n, false);

    gran_sys.set_timeStepping(GRAN_TIME_STEPPING::FIXED);
    gran_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::CENTERED_DIFFERENCE);
    gran_sys.set_fixed_stepSize(params.step_size);
    filesystem::create_directory(filesystem::path(params.output_dir));
    gran_sys.set_BD_Fixed(true);

    gran_sys.setVerbose(params.verbose);

    // Finalize settings and initialize for runtime
    gran_sys.initialize();

    int fps = 100;

    // assume we run for at least one frame
    float frame_step = 1.f / fps;
    float curr_time = 0;
    int currframe = 0;
    cout << "frame step is " << frame_step << endl;

    // Run settling experiments
    while (curr_time < params.time_end) {
        gran_sys.advance_simulation(frame_step);
        curr_time += frame_step;
        printf("rendering frame %u\n", currframe);
        char filename[100];
        sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
        gran_sys.writeFile(string(filename));
    }
    return 0;
}