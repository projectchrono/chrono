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
// Authors: Nic Olsen
// =============================================================================
// Chrono::Dem demo demonstrating some useful utilities for terrain
// initialization. The PDLayerSampler generates material in layers according to
// the Poisson Disk sampler. The terrain is read in from a triangle OBJ mesh
// file and decomposed into spheres which are set as fixed for the simulation.
// Note that the sphere size should be small enough to capture each triangle.
// =============================================================================

#include <iostream>
#include <vector>
#include <string>

#include "chrono/core/ChGlobal.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_dem/physics/ChSystemDem.h"
#include "chrono_dem/utils/ChDemJsonParser.h"
#include "chrono_dem/utils/ChDemSphereDecomp.h"

using namespace chrono;
using namespace chrono::dem;

int main(int argc, char* argv[]) {
    std::string inputJson = GetChronoDataFile("dem/fixedTerrain.json");
    if (argc == 2) {
        inputJson = std::string(argv[1]);
    } else if (argc > 2) {
        std::cout << "Usage:\n./demo_DEM_fixedTerrain <json_file>" << std::endl;
        return 1;
    }

    ChDemSimulationParameters params;
    if (!ParseJSON(inputJson, params)) {
        std ::cout << "ERROR: reading input file " << inputJson << std::endl;
        return 1;
    }

    ChSystemDem dem_sys(params.sphere_radius, params.sphere_density,
                        ChVector3f(params.box_X, params.box_Y, params.box_Z));

    // Add spherically-decomposed underlying terrain.
    std::string objfilename(GetChronoDataFile("models/fixedterrain.obj"));

    ChVector3f scaling(params.box_X / 2, params.box_Y / 2, params.box_Z);
    ChVector3f offset(0, 0, -params.box_Z / 2);

    std::vector<ChVector3f> terrain_points =
        MeshSphericalDecomposition<float>(objfilename, scaling, offset, params.sphere_radius);

    // Add loose granular material
    int layers = 10;  // Approximate number of material layers
    const float boundary_padding = 8.f * params.sphere_radius;
    float fill_bottom = -1e16f;
    for (auto pt : terrain_points) {
        fill_bottom = std::max(fill_bottom, pt.z());
    }
    fill_bottom += 4.f * params.sphere_radius;

    const float fill_height = layers * 2.f * params.sphere_radius;
    float fill_top = std::min(fill_bottom + fill_height, params.box_Z / 2.f - 2.f * params.sphere_radius);

    ChVector3f fill_center(0.f, 0.f, (fill_bottom + fill_top) / 2.f);
    ChVector3f fill_hdims(params.box_X / 2.f - boundary_padding - 2.f * params.sphere_radius,
                          params.box_Y / 2.f - boundary_padding - 2.f * params.sphere_radius,
                          (fill_top - fill_bottom) / 2.f);
    std::vector<ChVector3f> material_points =
        utils::ChPDLayerSamplerBox<float>(fill_center, fill_hdims, 2.f * params.sphere_radius);

    // Vectors of all particle positions and fixities
    std::vector<ChVector3f> body_points;
    std::vector<bool> fixed;

    body_points.insert(body_points.end(), terrain_points.begin(), terrain_points.end());
    fixed.insert(fixed.end(), terrain_points.size(), true);

    body_points.insert(body_points.end(), material_points.begin(), material_points.end());
    fixed.insert(fixed.end(), material_points.size(), false);

    std::cout << "Adding " << body_points.size() << " spheres." << std::endl;
    dem_sys.SetParticles(body_points);
    dem_sys.SetParticleFixed(fixed);

    // Add internal planes to prevent leaking
    {
        ChVector3f plane_pos(-(float)(params.box_X / 2 - boundary_padding), 0, 0);
        ChVector3f plane_normal(1, 0, 0);
        dem_sys.CreateBCPlane(plane_pos, plane_normal, false);
    }
    {
        ChVector3f plane_pos((float)(params.box_X / 2 - boundary_padding), 0, 0);
        ChVector3f plane_normal(-1, 0, 0);
        dem_sys.CreateBCPlane(plane_pos, plane_normal, false);
    }
    {
        ChVector3f plane_pos(0, -(float)(params.box_Y / 2 - boundary_padding), 0);
        ChVector3f plane_normal(0, 1, 0);
        dem_sys.CreateBCPlane(plane_pos, plane_normal, false);
    }
    {
        ChVector3f plane_pos(0, (float)(params.box_Y / 2 - boundary_padding), 0);
        ChVector3f plane_normal(0, -1, 0);
        dem_sys.CreateBCPlane(plane_pos, plane_normal, false);
    }

    dem_sys.SetBDFixed(true);

    dem_sys.SetKn_SPH2SPH(params.normalStiffS2S);
    dem_sys.SetKn_SPH2WALL(params.normalStiffS2W);

    dem_sys.SetGn_SPH2SPH(params.normalDampS2S);
    dem_sys.SetGn_SPH2WALL(params.normalDampS2W);

    dem_sys.SetKt_SPH2SPH(params.tangentStiffS2S);
    dem_sys.SetKt_SPH2WALL(params.tangentStiffS2W);

    dem_sys.SetGt_SPH2SPH(params.tangentDampS2S);
    dem_sys.SetGt_SPH2WALL(params.tangentDampS2W);

    dem_sys.SetCohesionRatio(params.cohesion_ratio);
    dem_sys.SetAdhesionRatio_SPH2WALL(params.adhesion_ratio_s2w);

    dem_sys.SetGravitationalAcceleration(ChVector3f(params.grav_X, params.grav_Y, params.grav_Z));

    dem_sys.SetFixedStepSize(params.step_size);
    dem_sys.SetFrictionMode(params.friction_mode);
    dem_sys.SetTimeIntegrator(params.time_integrator);

    dem_sys.SetStaticFrictionCoeff_SPH2SPH(params.static_friction_coeffS2S);
    dem_sys.SetStaticFrictionCoeff_SPH2WALL(params.static_friction_coeffS2W);

    dem_sys.SetRollingMode(params.rolling_mode);
    dem_sys.SetRollingCoeff_SPH2SPH(params.rolling_friction_coeffS2S);
    dem_sys.SetRollingCoeff_SPH2WALL(params.rolling_friction_coeffS2W);

    dem_sys.SetParticleOutputMode(params.write_mode);
    dem_sys.SetVerbosity(params.verbose);
    dem_sys.SetParticleOutputFlags(CHDEM_OUTPUT_FLAGS::ABSV | CHDEM_OUTPUT_FLAGS::ANG_VEL_COMPONENTS |
                                   CHDEM_OUTPUT_FLAGS::FIXITY);

    // Create data directory
    std::string out_dir = GetChronoOutputPath() + "DEM/";
    filesystem::create_directory(filesystem::path(out_dir));
    out_dir = out_dir + params.output_dir;
    filesystem::create_directory(filesystem::path(out_dir));

    // Finalize initialization of the Chrono::Dem system
    dem_sys.Initialize();

    unsigned int out_fps = 50;
    unsigned int total_frames = (unsigned int)((float)params.time_end * out_fps);
    double frame_step = 1.0 / out_fps;
    int currframe = 0;
    for (double t = 0; t < (double)params.time_end; t += frame_step, currframe++) {
        std::cout << "Rendering frame " << (currframe + 1) << " of " << total_frames << std::endl;
        char filename[100];
        sprintf(filename, "%s/step%06d.csv", out_dir.c_str(), currframe);
        dem_sys.WriteParticleFile(std::string(filename));

        dem_sys.AdvanceSimulation((float)frame_step);
    }

    return 0;
}
