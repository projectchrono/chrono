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
#include "chrono/core/ChFileutils.h"
#include "chrono_granular/physics/ChGranular.h"
#include "ChGranular_json_parser.hpp"

using namespace chrono;
using namespace chrono::granular;
using std::cout;
using std::endl;
using std::string;

void ShowUsage() {
    cout << "usage: ./demo_GRAN_utest_GRAN_spherefric <json_file>" << endl;
}

// -----------------------------------------------------------------------------
// The units are always cm/s/g[L/T/M].
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    sim_param_holder params;

    // Some of the default values might be overwritten by user via command line
    if (argc != 2 || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }

    // Setup simulation
    ChSystemGranular_MonodisperseSMC m_sys(params.sphere_radius, params.sphere_density);
    m_sys.setBOXdims(params.box_X, params.box_Y, params.box_Z);
    m_sys.set_K_n_SPH2SPH(params.normalStiffS2S);
    m_sys.set_K_n_SPH2WALL(params.normalStiffS2W);
    m_sys.set_Gamma_n_SPH2SPH(params.normalDampS2S);
    m_sys.set_Gamma_n_SPH2WALL(params.normalDampS2W);
    m_sys.set_K_t_SPH2SPH(params.tangentStiffS2S);
    m_sys.set_K_t_SPH2WALL(params.tangentStiffS2W);
    m_sys.set_Gamma_t_SPH2SPH(params.tangentDampS2S);
    m_sys.set_Gamma_t_SPH2WALL(params.tangentDampS2W);
    m_sys.set_Cohesion_ratio(params.cohesion_ratio);
    m_sys.set_Adhesion_ratio_S2W(params.adhesion_ratio_s2w);
    m_sys.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);
    m_sys.setOutputDirectory(params.output_dir);
    m_sys.setOutputMode(params.write_mode);

    m_sys.set_friction_mode(chrono::granular::GRAN_FRICTION_MODE::SINGLE_STEP);
    m_sys.set_contactModel(chrono::granular::GRAN_CONTACT_MODEL::HOOKE);

    std::vector<ChVector<float>> points;
    points.push_back(ChVector<float>(0, 0, -params.box_Z / 2 + params.sphere_radius));
    m_sys.setParticlePositions(points);

    m_sys.set_timeStepping(GRAN_TIME_STEPPING::FIXED);
    m_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::FORWARD_EULER);
    m_sys.set_fixed_stepSize(params.step_size);
    ChFileutils::MakeDirectory(params.output_dir.c_str());
    m_sys.set_BD_Fixed(true);

    m_sys.setVerbose(params.verbose);

    // Finalize settings and initialize for runtime
    m_sys.initialize();

    int fps = 100;

    // assume we run for at least one frame
    float frame_step = 1.f / fps;
    float curr_time = 0;
    int currframe = 0;
    cout << "frame step is " << frame_step << endl;

    // Run settling experiments
    while (curr_time < params.time_end) {
        m_sys.advance_simulation(frame_step);
        curr_time += frame_step;
        printf("rendering frame %u\n", currframe);
        char filename[100];
        sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
        m_sys.writeFileUU(std::string(filename));
    }
    return 0;
}