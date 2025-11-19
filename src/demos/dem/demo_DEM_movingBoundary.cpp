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
// Chrono::Dem simulation of a rectangular bed of granular material which is
// first let to settle and then compressed by advancing one of the box walls
// into the material.
// =============================================================================

#include <iostream>
#include <string>
#include <cmath>

#include "chrono/core/ChDataPath.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_dem/physics/ChSystemDem.h"
#include "chrono_dem/utils/ChDemJsonParser.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::dem;

int main(int argc, char* argv[]) {
    std::string inputJson = GetChronoDataFile("dem/movingBoundary.json");
    if (argc == 2) {
        inputJson = std::string(argv[1]);
    } else if (argc > 2) {
        std::cout << "Usage:\n./demo_DEM_movingBoundary <json_file>" << std::endl;
        return 1;
    }

    ChDemSimulationParameters params;
    if (!ParseJSON(inputJson, params)) {
        std ::cout << "ERROR: reading input file " << inputJson << std::endl;
        return 1;
    }

    // Setup simulation. A convenient thing we can do is to move the Big Box Domain by (X/2, Y/2, Z/2) which is done
    // with the fourth constructor param, so the coordinate range we are now working with is (0,0,0) to (X,Y,Z), instead
    // of (-X/2,-Y/2,-Z/2) to (X/2, Y/2, Z/2).
    ChSystemDem dem_sys(params.sphere_radius, params.sphere_density,
                        ChVector3f(params.box_X, params.box_Y, params.box_Z),
                        ChVector3f(params.box_X / 2, params.box_Y / 2, params.box_Z / 2));

    dem_sys.SetPsiFactors(params.psi_T, params.psi_L);

    dem_sys.SetKn_SPH2SPH(params.normalStiffS2S);
    dem_sys.SetKn_SPH2WALL(params.normalStiffS2W);
    dem_sys.SetGn_SPH2SPH(params.normalDampS2S);
    dem_sys.SetGn_SPH2WALL(params.normalDampS2W);

    dem_sys.SetKt_SPH2SPH(params.tangentStiffS2S);
    dem_sys.SetKt_SPH2WALL(params.tangentStiffS2W);
    dem_sys.SetGt_SPH2SPH(params.tangentDampS2S);
    dem_sys.SetGt_SPH2WALL(params.tangentDampS2W);
    dem_sys.SetStaticFrictionCoeff_SPH2SPH(params.static_friction_coeffS2S);
    dem_sys.SetStaticFrictionCoeff_SPH2WALL(params.static_friction_coeffS2W);

    dem_sys.SetCohesionRatio(params.cohesion_ratio);
    dem_sys.SetAdhesionRatio_SPH2WALL(params.adhesion_ratio_s2w);

    dem_sys.SetGravitationalAcceleration(ChVector3f(params.grav_X, params.grav_Y, params.grav_Z));
    dem_sys.SetParticleOutputMode(params.write_mode);

    std::string out_dir = GetChronoOutputPath() + "DEM/";
    filesystem::create_directory(filesystem::path(out_dir));
    out_dir = out_dir + params.output_dir;
    filesystem::create_directory(filesystem::path(out_dir));

    // The box to fill with particles
    ChVector3f hdims((float)(params.box_X / 2.0 - 1.2), (float)(params.box_Y / 2.0 - 1.2),
                     (float)(params.box_Z / 10.0 - 1.2));
    ChVector3f center((float)(params.box_X / 2), (float)(params.box_Y / 2), (float)(params.box_Z / 10.0));

    // Fill box with bodies
    std::vector<ChVector3f> body_points =
        utils::ChPDLayerSamplerBox<float>(center, hdims, 2.f * params.sphere_radius, 1.05f);

    dem_sys.SetParticles(body_points);

    // Set the position of the BD
    dem_sys.SetBDFixed(true);

    dem_sys.SetTimeIntegrator(CHDEM_TIME_INTEGRATOR::FORWARD_EULER);
    dem_sys.SetFrictionMode(CHDEM_FRICTION_MODE::MULTI_STEP);
    dem_sys.SetFixedStepSize(params.step_size);

    dem_sys.SetVerbosity(params.verbose);

    // start outside BD by 10 cm
    ChVector3f plane_pos(-10, 0, 0);
    ChVector3f plane_normal(1, 0, 0);

    size_t plane_bc_id = dem_sys.CreateBCPlane(plane_pos, plane_normal, false);

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

    dem_sys.Initialize();

    dem_sys.SetBCOffsetFunction(plane_bc_id, plane_pos_func);

    int fps = 50;
    // assume we run for at least one frame
    float frame_step = 1.0f / fps;
    float curr_time = 0;
    int currframe = 0;
    unsigned int total_frames = (unsigned int)((float)params.time_end * fps);

    char filename[100];
    sprintf(filename, "%s/step%06d.csv", out_dir.c_str(), currframe++);
    dem_sys.WriteParticleFile(std::string(filename));

    std::cout << "frame step is " << frame_step << std::endl;

    // Run settling experiments
    while (curr_time < params.time_end) {
        dem_sys.AdvanceSimulation(frame_step);
        curr_time += frame_step;
        printf("rendering frame %u of %u\n", currframe, total_frames);
        sprintf(filename, "%s/step%06d.csv", out_dir.c_str(), currframe++);
        dem_sys.WriteParticleFile(std::string(filename));
    }

    return 0;
}
