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
    ChSystemGranular_MonodisperseSMC gran_sys(params.sphere_radius, params.sphere_density);
    gran_sys.setBOXdims(params.box_X, params.box_Y, params.box_Z);
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

    gran_sys.set_BD_Fixed(true);

    // Fill box with bodies
    std::vector<ChVector<float>> body_points;

    // padding in sampler
    float fill_epsilon = 2.02f;
    // padding at top of fill
    float fill_gap = 1.f;

    chrono::utils::PDSampler<float> sampler(fill_epsilon * params.sphere_radius);


    float center_pt[3] = {0.f, 0.f, -2 - params.box_Z / 6.f};

    // width we want to fill to
    float fill_width = params.box_Z / 3.f;
    // height that makes this width above the cone
    float fill_height = fill_width;

    // fill to top
    float fill_top = params.box_Z / 2 - fill_gap;
    float fill_bottom = fill_top - fill_height;

    printf("width is %f, bot is %f, top is %f, height is %f\n", fill_width, fill_bottom, fill_top, fill_height);
    // fill box, layer by layer
    ChVector<> center(0, 0, fill_bottom);
    // shift up for bottom of box
    center.z() += fill_gap;

    while (center.z() < fill_top) {
        std::cout << "Create layer at " << center.z() << std::endl;
        auto points = sampler.SampleCylinderZ(center, fill_width, 0);
        body_points.insert(body_points.end(), points.begin(), points.end());
        center.z() += fill_epsilon * params.sphere_radius;
    }

    gran_sys.setParticlePositions(body_points);

    float sphere_mass = (4.f / 3.f) * params.sphere_density *  params.sphere_radius*  params.sphere_radius*  params.sphere_radius;
    
    printf("%d spheres with mass %f \n", body_points.size(), body_points.size() * sphere_mass);

    gran_sys.set_timeStepping(GRAN_TIME_STEPPING::FIXED);
    // gran_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::CHUNG);
    gran_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::FORWARD_EULER);
    // gran_sys.set_friction_mode(GRAN_FRICTION_MODE::SINGLE_STEP);
    gran_sys.set_ForceModel(GRAN_FORCE_MODEL::HOOKE);
    gran_sys.set_friction_mode(GRAN_FRICTION_MODE::FRICTIONLESS);
    gran_sys.set_fixed_stepSize(params.step_size);

    ChFileutils::MakeDirectory(params.output_dir.c_str());

    float cone_slope = 1.0;

    float cone_offset = 5.f;

    gran_sys.setVerbose(params.verbose   );
    // Finalize settings and initialize for runtime
    gran_sys.Create_BC_Cone_Z(center_pt, cone_slope, params.box_Z, center_pt[2] + cone_offset, false);

    float zvec[3] = {0,0,0};
    float cyl_rad  = fill_width + 8 * params.sphere_radius;

    gran_sys.Create_BC_Cyl_Z(zvec, cyl_rad, false);

    printf("fill radius is %f, cyl radius is %f\n", fill_width, fill_width);


    float plane_center[3] = {0, 0, center_pt[2]  + 2 * cone_slope + cone_slope*cone_offset};
    // face in upwards
    float plane_normal[3] = {0, 0, 1};
    
    printf("center is %f, %f, %f, plane center is is %f, %f, %f\n", center_pt[0], center_pt[1], center_pt[2], plane_center[0],
           plane_center[1], plane_center[2]);
    size_t plane_bc_id = gran_sys.Create_BC_Plane(plane_center, plane_normal);

    gran_sys.initialize();

    int fps =    100;
    // assume we run for at least one frame
    float frame_step =          1. / fps;
    float curr_time = 0;
    int currframe = 0;

    std::cout << "frame step is " << frame_step << std::endl;

    float t_remove_plane = .5;
    bool plane_active = false;

    // Run settling experiments
    while (curr_time < params.time_end) {
        if (!plane_active && curr_time > t_remove_plane) {
            gran_sys.disable_BC_by_ID(plane_bc_id);
        }
        gran_sys.advance_simulation(frame_step);
        curr_time += frame_step;
        printf("rendering frame %u\n", currframe);
        char filename[100];
        sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
        gran_sys.writeFileUU(std::string(filename));
    }

    return 0;
}
