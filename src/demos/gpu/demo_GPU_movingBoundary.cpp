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
// Chrono::Gpu simulation of a rectangular bed of granular material which
// is first let to settle and then compressed by advancing one of the box walls
// into the material.
// =============================================================================

#include <iostream>
#include <string>
#include <cmath>

#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/physics/ChSystemGpu_impl.h"
#include "chrono_gpu/utils/ChGpuJsonParser.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::gpu;

// -----------------------------------------------------------------------------
// Show command line usage
// -----------------------------------------------------------------------------
void ShowUsage(std::string name) {
    std::cout << "usage: " + name + " <json_file>" << std::endl;
}

int main(int argc, char* argv[]) {
    ChGpuSimulationParameters params;

    // Some of the default values might be overwritten by user via command line
    if (argc != 2 || ParseJSON(argv[1], params) == false) {
        ShowUsage(argv[0]);
        return 1;
    }

    // Setup simulation
    ChSystemGpu apiSMC(params.sphere_radius, params.sphere_density,
                       make_float3(params.box_X, params.box_Y, params.box_Z));
    ChSystemGpu_impl& gpu_sys = apiSMC.getSystem();

    apiSMC.SetPsiFactors(params.psi_T, params.psi_L);

    apiSMC.SetKn_SPH2SPH(params.normalStiffS2S);
    apiSMC.SetKn_SPH2WALL(params.normalStiffS2W);
    apiSMC.SetGn_SPH2SPH(params.normalDampS2S);
    apiSMC.SetGn_SPH2WALL(params.normalDampS2W);

    apiSMC.SetKt_SPH2SPH(params.tangentStiffS2S);
    apiSMC.SetKt_SPH2WALL(params.tangentStiffS2W);
    apiSMC.SetGt_SPH2SPH(params.tangentDampS2S);
    apiSMC.SetGt_SPH2WALL(params.tangentDampS2W);
    apiSMC.SetStaticFrictionCoeff_SPH2SPH(params.static_friction_coeffS2S);
    apiSMC.SetSaticFictionCeff_SPH2WALL(params.static_friction_coeffS2W);

    apiSMC.SetCohesionRatio(params.cohesion_ratio);
    apiSMC.SetAdhesionRatio_SPH2WALL(params.adhesion_ratio_s2w);

    apiSMC.SetGravitationalAcceleration(ChVector<float>(params.grav_X, params.grav_Y, params.grav_Z));
    apiSMC.SetOutputMode(params.write_mode);

    filesystem::create_directory(filesystem::path(params.output_dir));

    // fill box, layer by layer
    ChVector<float> hdims((float)(params.box_X / 2.0 - 1.2), (float)(params.box_Y / 2.0 - 1.2),
                          (float)(params.box_Z / 10.0 - 1.2));
    ChVector<float> center(0.f, 0.f, (float)(-params.box_Z / 2.0 + params.box_Z / 10.0));

    // Fill box with bodies
    std::vector<ChVector<float>> body_points =
        utils::PDLayerSampler_BOX<float>(center, hdims, 2.f * params.sphere_radius, 1.05f);

    apiSMC.SetParticlePositions(body_points);

    // Set the position of the BD
    apiSMC.SetBDFixed(true);

    apiSMC.SetTimeIntegrator(CHGPU_TIME_INTEGRATOR::FORWARD_EULER);
    apiSMC.SetFrictionMode(CHGPU_FRICTION_MODE::MULTI_STEP);
    apiSMC.SetFixedStepSize(params.step_size);

    apiSMC.SetVerbosity(params.verbose);

    // start outside BD by 10 cm
    ChVector<float> plane_pos(-params.box_X / 2 - 10, 0, 0);
    ChVector<float> plane_normal(1, 0, 0);

    size_t plane_bc_id = apiSMC.CreateBCPlane(plane_pos, plane_normal, false);

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

    apiSMC.Initialize();

    apiSMC.SetBCOffsetFunction(plane_bc_id, plane_pos_func);

    int fps = 50;
    // assume we run for at least one frame
    float frame_step = 1.0f / fps;
    float curr_time = 0;
    int currframe = 0;
    unsigned int total_frames = (unsigned int)((float)params.time_end * fps);

    char filename[100];
    sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
    apiSMC.WriteFile(std::string(filename));

    std::cout << "frame step is " << frame_step << std::endl;

    // Run settling experiments
    while (curr_time < params.time_end) {
        apiSMC.AdvanceSimulation(frame_step);
        curr_time += frame_step;
        printf("rendering frame %u of %u\n", currframe, total_frames);
        char filename[100];
        sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
        apiSMC.WriteFile(std::string(filename));
    }

    return 0;
}
