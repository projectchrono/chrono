// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nic Olsen
// =============================================================================
// A column of granular material forms a mound
// =============================================================================

#include <iostream>
#include <string>

#include "ChGranularDemoUtils.hpp"
#include "chrono/core/ChGlobal.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono_granular/ChGranularData.h"
#include "chrono_granular/api/ChApiGranularChrono.h"
#include "chrono_granular/physics/ChGranular.h"
#include "chrono_granular/utils/ChGranularJsonParser.h"
#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::granular;

int num_args = 2;
void ShowUsage(std::string name) {
    std::cout << "usage: " + name + " <json_file> <static_friction> <rolling_friction> <cohesion> <output_dir>"
              << std::endl;
}

int main(int argc, char* argv[]) {
    
    //granular::SetDataPath(std::string(PROJECTS_DATA_DIR) + "granular/");
    sim_param_holder params;

    // Some of the default values might be overwritten by user via command line
    std::cout<<"num_arg: "<<argc<<std::endl;
    if (argc != num_args || ParseJSON(argv[1], params) == false) {
        ShowUsage(argv[0]);
        return 1;
    }

    filesystem::create_directory(filesystem::path(params.output_dir));

    // Setup simulation
    ChSystemGranularSMC gran_sys(params.sphere_radius, params.sphere_density,
                                 make_float3(params.box_X, params.box_Y, params.box_Z));
    gran_sys.set_K_n_SPH2SPH(params.normalStiffS2S);
    gran_sys.set_K_n_SPH2WALL(params.normalStiffS2W);
    gran_sys.set_Gamma_n_SPH2SPH(params.normalDampS2S);
    gran_sys.set_Gamma_n_SPH2WALL(params.normalDampS2W);

    // gran_sys.set_friction_mode(GRAN_FRICTION_MODE::FRICTIONLESS);
    gran_sys.set_friction_mode(GRAN_FRICTION_MODE::MULTI_STEP);
    gran_sys.set_K_t_SPH2SPH(params.tangentStiffS2S);
    gran_sys.set_K_t_SPH2WALL(params.tangentStiffS2W);
    gran_sys.set_Gamma_t_SPH2SPH(params.tangentDampS2S);
    gran_sys.set_Gamma_t_SPH2WALL(params.tangentDampS2W);
    gran_sys.set_static_friction_coeff_SPH2SPH(params.static_friction_coeffS2S);
    gran_sys.set_static_friction_coeff_SPH2WALL(params.static_friction_coeffS2W);

    // gran_sys.set_rolling_mode(GRAN_ROLLING_MODE::NO_RESISTANCE);
    gran_sys.set_rolling_mode(GRAN_ROLLING_MODE::SCHWARTZ);
    gran_sys.set_rolling_coeff_SPH2SPH(params.rolling_friction_coeffS2S);
    gran_sys.set_rolling_coeff_SPH2WALL(params.rolling_friction_coeffS2W);

    gran_sys.set_Cohesion_ratio(params.cohesion_ratio);
    gran_sys.set_Adhesion_ratio_S2W(params.adhesion_ratio_s2w);
    gran_sys.set_gravitational_acceleration(params.grav_X, params.grav_Y, params.grav_Z);
    gran_sys.setOutputMode(params.write_mode);

    gran_sys.set_BD_Fixed(true);

    // padding in sampler
    float fill_epsilon = 2.02f;
    // padding at top of fill
    float drop_height = 0.f;
    float spacing = fill_epsilon * params.sphere_radius;
    chrono::utils::PDSampler<float> sampler(spacing);

    // Fixed points on the bottom for roughness
    float bottom_z = -params.box_Z / 2.f + params.sphere_radius;
    ChVector<> bottom_center(0, 0, bottom_z);
    std::vector<ChVector<float>> roughness_points = sampler.SampleBox(
        bottom_center,
        ChVector<float>(params.box_X / 2.f - params.sphere_radius, params.box_Y / 2.f - params.sphere_radius, 0.f));

    // Create column of material
    std::vector<ChVector<float>> material_points;

    float fill_bottom = bottom_z + spacing;
    float fill_width = 5.f;
    float fill_height = 2.f * fill_width;
    float fill_top = fill_bottom + fill_height;

    ChVector<float> center(0.f, 0.f, fill_bottom + fill_height / 2.f);
    material_points = sampler.SampleCylinderZ(center, fill_width, fill_height / 2.f);

    ChGranularSMC_API apiSMC;
    apiSMC.setGranSystem(&gran_sys);

    std::vector<ChVector<float>> body_points;
    std::vector<bool> body_points_fixed;
    body_points.insert(body_points.end(), roughness_points.begin(), roughness_points.end());
    body_points_fixed.insert(body_points_fixed.end(), roughness_points.size(), true);

    body_points.insert(body_points.end(), material_points.begin(), material_points.end());
    body_points_fixed.insert(body_points_fixed.end(), material_points.size(), false);

    apiSMC.setElemsPositions(body_points);
    gran_sys.setParticleFixed(body_points_fixed);

    std::cout << "Added " << roughness_points.size() << " fixed points" << std::endl;
    std::cout << "Added " << material_points.size() << " material points" << std::endl;

    std::cout << "Actually added " << body_points.size() << std::endl;

    gran_sys.set_timeIntegrator(GRAN_TIME_INTEGRATOR::EXTENDED_TAYLOR);
    gran_sys.set_fixed_stepSize(params.step_size);

    filesystem::create_directory(filesystem::path(params.output_dir));
    gran_sys.setVerbose(params.verbose);
    std::cout<<"verbose: "<<params.verbose<<std::endl;
    gran_sys.setRecordingContactInfo(true);

    gran_sys.initialize();

    int fps = 60;
    float frame_step = 1.f / fps;
    float curr_time = 0.f;
    int currframe = 0;
    unsigned int total_frames = (unsigned int)((float)params.time_end * fps);

    // write an initial frame
    char filename[100];
    sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe);
    gran_sys.writeFile(std::string(filename));

    char contactFilename[100];
    sprintf(contactFilename, "%s/contact%06d", params.output_dir.c_str(), currframe);
    gran_sys.writeContactInfoFile(std::string(contactFilename));

    currframe++;

    std::cout << "frame step is " << frame_step << std::endl;
    while (curr_time < params.time_end) {
        gran_sys.advance_simulation(frame_step);
        curr_time += frame_step;
        printf("rendering frame %u of %u\n", currframe, total_frames);
        sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe);
        gran_sys.writeFile(std::string(filename));

        char contactFilename[100];
        sprintf(contactFilename, "%s/contact%06d", params.output_dir.c_str(), currframe);
        gran_sys.writeContactInfoFile(std::string(contactFilename));

        currframe++;
    }

    return 0;
}
