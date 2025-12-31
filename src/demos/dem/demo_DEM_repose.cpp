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
// Authors: Ruochun Zhang
// =============================================================================
// A column of granular material forms a mound after flowing through a funnel
// =============================================================================

#include <iostream>
#include <string>

#include "chrono/core/ChDataPath.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_dem/physics/ChSystemDem.h"
#include "chrono_dem/utils/ChDemJsonParser.h"

#ifdef CHRONO_VSG
    #include "chrono_dem/visualization/ChDemVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::dem;

// Enable/disable run-time visualization
bool render = true;
double render_fps = 2000;

int main(int argc, char* argv[]) {
    std::string inputJson = GetChronoDataFile("dem/repose.json");
    if (argc == 2) {
        inputJson = std::string(argv[1]);
    } else if (argc > 2) {
        std::cout << "Usage:\n./demo_DEM_repose <json_file>" << std::endl;
        return 1;
    }

    ChDemSimulationParameters params;
    if (!ParseJSON(inputJson, params)) {
        std ::cout << "ERROR: reading input file " << inputJson << std::endl;
        return 1;
    }

    std::string out_dir = GetChronoOutputPath() + "DEM/";
    filesystem::create_directory(filesystem::path(out_dir));
    out_dir = out_dir + params.output_dir;
    filesystem::create_directory(filesystem::path(out_dir));

    // Setup simulation
    ChSystemDemMesh dem_sys(params.sphere_radius, params.sphere_density,
                            ChVector3f(params.box_X, params.box_Y, params.box_Z));

    // Insert the funnel
    float funnel_bottom = 0.f;
    dem_sys.AddMesh(GetChronoDataFile("models/funnel.obj"), ChVector3f(0, 0, funnel_bottom), ChMatrix33<float>(0.15f),
                    1e10);
    dem_sys.EnableMeshCollision(true);

    dem_sys.SetKn_SPH2SPH(params.normalStiffS2S);
    dem_sys.SetKn_SPH2WALL(params.normalStiffS2W);
    dem_sys.SetKn_SPH2MESH(params.normalStiffS2M);
    dem_sys.SetGn_SPH2SPH(params.normalDampS2S);
    dem_sys.SetGn_SPH2WALL(params.normalDampS2W);
    dem_sys.SetGn_SPH2MESH(params.normalDampS2M);

    // dem_sys.SetFrictionMode(CHDEM_FRICTION_MODE::FRICTIONLESS);
    dem_sys.SetFrictionMode(CHDEM_FRICTION_MODE::MULTI_STEP);
    dem_sys.SetKt_SPH2SPH(params.tangentStiffS2S);
    dem_sys.SetKt_SPH2WALL(params.tangentStiffS2W);
    dem_sys.SetKt_SPH2MESH(params.tangentStiffS2M);
    dem_sys.SetGt_SPH2SPH(params.tangentDampS2S);
    dem_sys.SetGt_SPH2WALL(params.tangentDampS2W);
    dem_sys.SetGt_SPH2MESH(params.tangentDampS2M);

    dem_sys.SetStaticFrictionCoeff_SPH2SPH(params.static_friction_coeffS2S);
    dem_sys.SetStaticFrictionCoeff_SPH2WALL(params.static_friction_coeffS2W);
    dem_sys.SetStaticFrictionCoeff_SPH2MESH(params.static_friction_coeffS2M);

    dem_sys.SetRollingMode(CHDEM_ROLLING_MODE::SCHWARTZ);

    // In this test, the rolling friction affects the final repose angle
    // by a lot. Test different values, such as 1, to see how it affects.
    dem_sys.SetRollingCoeff_SPH2SPH(params.rolling_friction_coeffS2S);
    dem_sys.SetRollingCoeff_SPH2WALL(params.rolling_friction_coeffS2W);
    dem_sys.SetRollingCoeff_SPH2MESH(params.rolling_friction_coeffS2M);

    // In this test, the cohesion is also influential.
    // You can test different scenarios with much larger cohesion ratio (around 50)
    // to greatly increase the repose angle.
    dem_sys.SetCohesionRatio(params.cohesion_ratio);
    dem_sys.SetAdhesionRatio_SPH2WALL(params.adhesion_ratio_s2w);
    dem_sys.SetGravitationalAcceleration(ChVector3f(params.grav_X, params.grav_Y, params.grav_Z));
    dem_sys.SetParticleOutputMode(params.write_mode);

    dem_sys.SetBDFixed(true);

    // padding in sampler
    float fill_epsilon = 2.02f;
    // padding at top of fill
    float spacing = fill_epsilon * params.sphere_radius;
    chrono::utils::ChPDSampler<float> sampler(spacing);
    chrono::utils::ChHCPSampler<float> HCPsampler(spacing);

    // Create column of material
    std::vector<ChVector3f> material_points;

    float fill_width = 5.f;
    float fill_height = 2.f * fill_width;
    float fill_bottom = funnel_bottom + fill_width + spacing;

    // add granular material particles layer by layer
    ChVector3f center(0, 0, fill_bottom + params.sphere_radius);
    // fill up each layer
    while (center.z() + params.sphere_radius < fill_bottom + fill_height) {
        auto points = sampler.SampleCylinderZ(center, fill_width, 0);
        material_points.insert(material_points.end(), points.begin(), points.end());
        center.z() += 2.02f * params.sphere_radius;
    }

    // Fixed (ground) points on the bottom for roughness
    ChVector3d bottom_center(0, 0, funnel_bottom - 10.f);
    std::vector<ChVector3f> roughness_points = HCPsampler.SampleBox(
        bottom_center,
        ChVector3f(params.box_X / 2.f - params.sphere_radius, params.box_Y / 2.f - params.sphere_radius, 0.f));

    std::vector<ChVector3f> body_points;
    std::vector<bool> body_points_fixed;
    body_points.insert(body_points.end(), roughness_points.begin(), roughness_points.end());
    body_points_fixed.insert(body_points_fixed.end(), roughness_points.size(), true);

    body_points.insert(body_points.end(), material_points.begin(), material_points.end());
    body_points_fixed.insert(body_points_fixed.end(), material_points.size(), false);

    dem_sys.SetParticles(body_points);
    dem_sys.SetParticleFixed(body_points_fixed);

    std::cout << "Added " << material_points.size() << " granular material points" << std::endl;
    std::cout << "Added " << roughness_points.size() << " fixed (ground) points" << std::endl;
    std::cout << "In total, added " << body_points.size() << std::endl;

    dem_sys.SetTimeIntegrator(CHDEM_TIME_INTEGRATOR::EXTENDED_TAYLOR);
    dem_sys.SetFixedStepSize(params.step_size);

    dem_sys.SetVerbosity(params.verbose);
    // std::cout << "verbose: " << static_cast<int>(params.verbose) << std::endl;

    dem_sys.Initialize();

    // Create a run-time visualizer
    std::shared_ptr<ChVisualSystem> vis;

#ifdef CHRONO_VSG
    // DEM plugin
    auto visDEM = chrono_types::make_shared<ChDemVisualizationVSG>(&dem_sys);

    // VSG visual system (attach visDEM as plugin)
    auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
    visVSG->AttachPlugin(visDEM);
    visVSG->SetWindowTitle("Chrono::Dem repose demo");
    visVSG->SetWindowSize(1280, 800);
    visVSG->SetWindowPosition(100, 100);
    visVSG->AddCamera(ChVector3d(0, -30, -10), ChVector3d(0, 0, -20));
    visVSG->SetLightIntensity(0.9f);
    visVSG->SetLightDirection(CH_PI_2, CH_PI / 6);

    visVSG->Initialize();
    vis = visVSG;
#else
    render = false;
#endif

    float step = 1e-3f;
    float time = 0.f;
    int sim_frame = 0;
    int render_frame = 0;

    // write an initial frame
    char filename[100];
    sprintf(filename, "%s/step%06d.csv", out_dir.c_str(), sim_frame);
    dem_sys.WriteParticleFile(std::string(filename));

    char mesh_filename[100];
    sprintf(mesh_filename, "%s/step%06d_mesh", out_dir.c_str(), sim_frame);
    dem_sys.WriteMeshes(std::string(mesh_filename));

    sim_frame++;

    while (time < params.time_end) {
        dem_sys.AdvanceSimulation(step);

        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();
            render_frame++;
        }

        printf("Output frame %u\n", sim_frame);
        sprintf(filename, "%s/step%06d.csv", out_dir.c_str(), sim_frame);
        dem_sys.WriteParticleFile(std::string(filename));
        sprintf(mesh_filename, "%s/step%06d_mesh", out_dir.c_str(), sim_frame);
        dem_sys.WriteMeshes(std::string(mesh_filename));

        float KE = dem_sys.GetParticlesKineticEnergy();
        std::cout << "Total kinetic energy: " << KE << std::endl;
        unsigned int NumStillIn = dem_sys.GetNumParticleAboveZ(funnel_bottom);
        std::cout << "Numer of particles still in funnel: " << NumStillIn << std::endl;

        time += step;
        sim_frame++;
    }

    return 0;
}
