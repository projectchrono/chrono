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
// Chrono::Dem evaluation of several simple mixer designs. Material consisting
// of spherical particles is let to aggitate in a rotating mixer. Metrics on the
// performance of each mixer can be determined in post-processing.
// =============================================================================

#include <cmath>
#include <iostream>
#include <string>

#include "chrono/core/ChGlobal.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"

#include "chrono_dem/physics/ChSystemDem.h"
#include "chrono_dem/utils/ChDemJsonParser.h"

#ifdef CHRONO_VSG
    #include "chrono_dem/visualization/ChDemVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::dem;

// Output frequency
float out_fps = 200;

// Enable/disable run-time visualization
bool render = true;
float render_fps = 2000;

int main(int argc, char* argv[]) {
    std::string inputJson = GetChronoDataFile("dem/mixer.json");
    if (argc == 2) {
        inputJson = std::string(argv[1]);
    } else if (argc > 2) {
        std::cout << "Usage:\n./demo_DEM_mixer <json_file>" << std::endl;
        return 1;
    }

    ChDemSimulationParameters params;
    if (!ParseJSON(inputJson, params)) {
        std ::cout << "ERROR: reading input file " << inputJson << std::endl;
        return 1;
    }

    const float Bx = params.box_X;
    const float By = Bx;
    const float chamber_height = Bx / 3;  // TODO
    const float fill_height = chamber_height;
    const float Bz = chamber_height + fill_height;
    std::cout << "Box Dims: " << Bx << " " << By << " " << Bz << std::endl;

    float iteration_step = params.step_size;

    ChSystemDemMesh dem_sys(params.sphere_radius, params.sphere_density, ChVector3f(Bx, By, Bz));

    dem_sys.SetKn_SPH2SPH(params.normalStiffS2S);
    dem_sys.SetKn_SPH2WALL(params.normalStiffS2W);
    dem_sys.SetKn_SPH2MESH(params.normalStiffS2M);

    dem_sys.SetKt_SPH2SPH(params.tangentStiffS2S);
    dem_sys.SetKt_SPH2WALL(params.tangentStiffS2W);
    dem_sys.SetKt_SPH2MESH(params.tangentStiffS2M);

    dem_sys.SetGn_SPH2SPH(params.normalDampS2S);
    dem_sys.SetGn_SPH2WALL(params.normalDampS2W);
    dem_sys.SetGn_SPH2MESH(params.normalDampS2M);

    dem_sys.SetGt_SPH2SPH(params.tangentDampS2S);
    dem_sys.SetGt_SPH2WALL(params.tangentDampS2W);
    dem_sys.SetGt_SPH2MESH(params.tangentDampS2M);

    dem_sys.SetCohesionRatio(params.cohesion_ratio);
    dem_sys.SetAdhesionRatio_SPH2MESH(params.adhesion_ratio_s2m);
    dem_sys.SetAdhesionRatio_SPH2WALL(params.adhesion_ratio_s2w);
    dem_sys.SetFrictionMode(chrono::dem::CHDEM_FRICTION_MODE::MULTI_STEP);

    dem_sys.SetStaticFrictionCoeff_SPH2SPH(params.static_friction_coeffS2S);
    dem_sys.SetStaticFrictionCoeff_SPH2WALL(params.static_friction_coeffS2W);
    dem_sys.SetStaticFrictionCoeff_SPH2MESH(params.static_friction_coeffS2M);

    dem_sys.SetParticleOutputMode(params.write_mode);

    std::string out_dir = GetChronoOutputPath() + "DEM/";
    filesystem::create_directory(filesystem::path(out_dir));
    out_dir = out_dir + params.output_dir;
    filesystem::create_directory(filesystem::path(out_dir));

    dem_sys.SetTimeIntegrator(CHDEM_TIME_INTEGRATOR::CENTERED_DIFFERENCE);
    dem_sys.SetFixedStepSize(params.step_size);
    dem_sys.SetBDFixed(true);

    const float chamber_bottom = -Bz / 2.f;
    const float fill_bottom = chamber_bottom + chamber_height;

    ChVector3f cyl_center(0, 0, 0);
    const float cyl_rad = Bx / 2.f;
    dem_sys.CreateBCCylinderZ(cyl_center, cyl_rad, false, false);

    utils::ChHCPSampler<float> sampler(2.1f * params.sphere_radius);
    std::vector<ChVector3f> body_points;

    const float fill_radius = Bx / 2.f - 2.f * params.sphere_radius;
    const float fill_top = fill_bottom + fill_height;

    std::cout << "Fill radius " << fill_radius << std::endl;
    std::cout << "Fill bottom " << fill_bottom << std::endl;
    std::cout << "Fill top " << fill_top << std::endl;

    ChVector3f center(0, 0, fill_bottom);
    center.z() += 2 * params.sphere_radius;
    while (center.z() < fill_top - 2 * params.sphere_radius) {
        auto points = sampler.SampleCylinderZ(center, fill_radius, 0);
        body_points.insert(body_points.end(), points.begin(), points.end());
        center.z() += 2.1f * params.sphere_radius;
    }

    dem_sys.SetParticles(body_points);
    dem_sys.SetGravitationalAcceleration(ChVector3f(0, 0, -980));

    // Add the mixer mesh to the DEM system
    float scale_xy = Bx / 2.f;
    float scale_z = chamber_height;
    ChVector3d scaling(scale_xy, scale_xy, scale_z);
    float mixer_mass = 10;
    auto mixer_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    mixer_mesh->LoadWavefrontMesh(GetChronoDataFile("models/mixer/internal_mixer.obj"), true, false);
    mixer_mesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scaling));
    auto mixer_mesh_id = dem_sys.AddMesh(mixer_mesh, mixer_mass);

    float rev_per_sec = 1.f;
    float ang_vel_Z = rev_per_sec * 2 * (float)CH_PI;
    ChVector3d mesh_lin_vel(0);
    ChVector3d mesh_ang_vel(0, 0, ang_vel_Z);

    dem_sys.EnableMeshCollision(true);
    dem_sys.Initialize();

    // Create a run-time visualizer
    std::shared_ptr<ChBody> mixer;
    std::shared_ptr<ChVisualSystem> vis;

#ifdef CHRONO_VSG
    mixer = chrono_types::make_shared<ChBody>();
    auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    trimesh_shape->SetMesh(mixer_mesh);
    mixer->AddVisualShape(trimesh_shape, ChFrame<>());

    // DEM plugin
    auto visDEM = chrono_types::make_shared<ChDemVisualizationVSG>(&dem_sys);
    visDEM->AddProxyBody(mixer);

    // VSG visual system (attach visDEM as plugin)
    auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
    visVSG->AttachPlugin(visDEM);
    visVSG->SetWindowTitle("Chrono::Dem mixer demo");
    visVSG->SetWindowSize(1280, 800);
    visVSG->SetWindowPosition(100, 100);
    visVSG->AddCamera(ChVector3d(0, -100, 75), ChVector3d(0, 0, 0));
    visVSG->SetLightIntensity(0.9f);
    visVSG->SetLightDirection(CH_PI_2, CH_PI / 6);

    visVSG->Initialize();
    vis = visVSG;
#else
    render = false;
#endif

    int sim_frame = 0;
    int render_frame = 0;
    int out_frame = 0;

    for (float t = 0; t < params.time_end; t += iteration_step) {
        ChVector3d mesh_pos(0, 0, chamber_bottom + chamber_height / 2.0);
        ChQuaternion<> mesh_rot = QuatFromAngleZ(t * ang_vel_Z);
        dem_sys.ApplyMeshMotion(mixer_mesh_id, mesh_pos, mesh_rot, mesh_lin_vel, mesh_ang_vel);

        if (t >= out_frame / out_fps) {
            std::cout << "Output at frame " << (sim_frame + 1) << std::endl;
            char filename[100];
            char mesh_filename[100];
            sprintf(filename, "%s/step%06u.csv", out_dir.c_str(), sim_frame);
            sprintf(mesh_filename, "%s/step%06u_mesh", out_dir.c_str(), sim_frame);
            dem_sys.WriteParticleFile(std::string(filename));
            dem_sys.WriteMeshes(std::string(mesh_filename));

            ChVector3d force;
            ChVector3d torque;
            dem_sys.CollectMeshContactForces(0, force, torque);
            std::cout << "torque: " << torque.x() << ", " << torque.y() << ", " << torque.z() << std::endl;

            out_frame++;
        }

        if (render && t >= render_frame / render_fps) {
            mixer->SetPos(mesh_pos);
            mixer->SetRot(mesh_rot);
            if (!vis->Run())
                break;
            vis->Render();
            render_frame++;
        }

        ////std::cout << "Time = " << t << std::endl;
        dem_sys.AdvanceSimulation(iteration_step);
        sim_frame++;
    }

    return 0;
}
