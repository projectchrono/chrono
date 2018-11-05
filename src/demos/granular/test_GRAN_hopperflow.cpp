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

void ShowUsage() {
    cout << "usage: ./utest_GRAN_Coneflow <json_file>" << endl;
}

int main(int argc, char* argv[]) {
    sim_param_holder params;

    // Some of the default values might be overwritten by user via command line
    if (argc != 2 || ParseJSON(argv[1], params) == false) {
        ShowUsage();
        return 1;
    }

    // Setup simulation parameters
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
    m_sys.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);
    m_sys.setOutputDirectory(params.output_dir);
    m_sys.setOutputMode(params.write_mode);

    m_sys.set_BD_Fixed(true);

    float cone_tip[3] = {0.f, 0.f, 0.f};            // Hypothetical location of cone tip
    float radius_opening = 10;                      // Opening at bottom of cone // TODO vary
    float cone_slope = 1.f;                         // TODO vary
    float opening_z = cone_slope * radius_opening;  // z coord of the cone opening
    float top_z = params.box_Z / 2.f;               // No harm in extending the cone to the top
    constexpr bool outward_normal = true;          // Inward-colliding cone

    m_sys.Create_BC_Cone(cone_tip, cone_slope, top_z, opening_z, outward_normal);

    double particle_mass = 4.0 * CH_C_PI * params.sphere_radius * params.sphere_radius * params.sphere_radius *
                           params.sphere_density / 3.0;

    // Generate constant total mass of particles
    std::vector<ChVector<float>> body_points;
    chrono::utils::PDSampler<float> sampler(2.05 * params.sphere_radius);

    double fill_bottom = opening_z + 2.05 * params.sphere_radius;
    double fill_top = params.box_Z / 2.0 - 2.05 * params.sphere_radius;
    ChVector<> center = {cone_tip[0], cone_tip[1], fill_bottom};
    double total_mass = 0;
    while (center.z() < fill_top) {
        constexpr double fill_height = 0;
        double fill_radius = center.z() / cone_slope - 2.05 * params.sphere_radius;

        auto points = sampler.SampleCylinderZ(center, fill_radius, fill_height);
        body_points.insert(body_points.end(), points.begin(), points.end());

        total_mass = body_points.size() * particle_mass;
        center.z() = center.z() + 2.05 * params.sphere_radius;
    }

    cout << "Number of particles: " << body_points.size() << endl;

    m_sys.setParticlePositions(body_points);

    m_sys.set_timeStepping(GRAN_TIME_STEPPING::FIXED);
    m_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::FORWARD_EULER);
    m_sys.set_fixed_stepSize(params.step_size);

    ChFileutils::MakeDirectory(params.output_dir.c_str());

    m_sys.setVerbose(params.verbose);

    // Finalize settings and initialize for runtime
    m_sys.initialize();

    int fps = 100;
    // assume we run for at least one frame
    float frame_step = 1.f / fps;
    float curr_time = 0;
    int currframe = 0;

    cout << "Frame step is " << frame_step << endl;

    // Run settling experiments
    while (curr_time < params.time_end) {
        m_sys.advance_simulation(frame_step);
        curr_time += frame_step;
        cout << "Rendering frame " << currframe << endl;
        char filename[100];
        std::sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
        m_sys.writeFileUU(string(filename));

        // TODO compute mass flow rate
    }

    return 0;
}
