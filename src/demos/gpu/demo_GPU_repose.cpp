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

#include "chrono/core/ChGlobal.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/utils/ChGpuJsonParser.h"

#include "chrono_gpu/visualization/ChGpuVisualization.h"
#ifdef CHRONO_OPENGL
    #include "chrono_gpu/visualization/ChGpuVisualizationGL.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::gpu;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = true;
double render_fps = 100;

int main(int argc, char* argv[]) {
    std::string inputJson = GetChronoDataFile("gpu/repose.json");
    if (argc == 2) {
        inputJson = std::string(argv[1]);
    } else if (argc > 2) {
        std::cout << "Usage:\n./demo_GPU_repose <json_file>" << std::endl;
        return 1;
    }

    ChGpuSimulationParameters params;
    if (!ParseJSON(inputJson, params)) {
        std ::cout << "ERROR: reading input file " << inputJson << std::endl;
        return 1;
    }

    std::string out_dir = GetChronoOutputPath() + "GPU/";
    filesystem::create_directory(filesystem::path(out_dir));
    out_dir = out_dir + params.output_dir;
    filesystem::create_directory(filesystem::path(out_dir));

    // Setup simulation
    ChSystemGpuMesh gpu_sys(params.sphere_radius, params.sphere_density,
                            ChVector3f(params.box_X, params.box_Y, params.box_Z));

    // Insert the funnel
    float funnel_bottom = 0.f;
    gpu_sys.AddMesh(GetChronoDataFile("models/funnel.obj"), ChVector3f(0, 0, funnel_bottom), ChMatrix33<float>(0.15f),
                    1e10);
    gpu_sys.EnableMeshCollision(true);

    gpu_sys.SetKn_SPH2SPH(params.normalStiffS2S);
    gpu_sys.SetKn_SPH2WALL(params.normalStiffS2W);
    gpu_sys.SetKn_SPH2MESH(params.normalStiffS2M);
    gpu_sys.SetGn_SPH2SPH(params.normalDampS2S);
    gpu_sys.SetGn_SPH2WALL(params.normalDampS2W);
    gpu_sys.SetGn_SPH2MESH(params.normalDampS2M);

    // gpu_sys.SetFrictionMode(CHGPU_FRICTION_MODE::FRICTIONLESS);
    gpu_sys.SetFrictionMode(CHGPU_FRICTION_MODE::MULTI_STEP);
    gpu_sys.SetKt_SPH2SPH(params.tangentStiffS2S);
    gpu_sys.SetKt_SPH2WALL(params.tangentStiffS2W);
    gpu_sys.SetKt_SPH2MESH(params.tangentStiffS2M);
    gpu_sys.SetGt_SPH2SPH(params.tangentDampS2S);
    gpu_sys.SetGt_SPH2WALL(params.tangentDampS2W);
    gpu_sys.SetGt_SPH2MESH(params.tangentDampS2M);

    gpu_sys.SetStaticFrictionCoeff_SPH2SPH(params.static_friction_coeffS2S);
    gpu_sys.SetStaticFrictionCoeff_SPH2WALL(params.static_friction_coeffS2W);
    gpu_sys.SetStaticFrictionCoeff_SPH2MESH(params.static_friction_coeffS2M);

    gpu_sys.SetRollingMode(CHGPU_ROLLING_MODE::SCHWARTZ);

    // In this test, the rolling friction affects the final repose angle
    // by a lot. Test different values, such as 1, to see how it affects.
    gpu_sys.SetRollingCoeff_SPH2SPH(params.rolling_friction_coeffS2S);
    gpu_sys.SetRollingCoeff_SPH2WALL(params.rolling_friction_coeffS2W);
    gpu_sys.SetRollingCoeff_SPH2MESH(params.rolling_friction_coeffS2M);

    // In this test, the cohesion is also influential.
    // You can test different scenarios with much larger cohesion ratio (around 50)
    // to greatly increase the repose angle.
    gpu_sys.SetCohesionRatio(params.cohesion_ratio);
    gpu_sys.SetAdhesionRatio_SPH2WALL(params.adhesion_ratio_s2w);
    gpu_sys.SetGravitationalAcceleration(ChVector3f(params.grav_X, params.grav_Y, params.grav_Z));
    gpu_sys.SetParticleOutputMode(params.write_mode);

    gpu_sys.SetBDFixed(true);

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

    gpu_sys.SetParticles(body_points);
    gpu_sys.SetParticleFixed(body_points_fixed);

    std::cout << "Added " << material_points.size() << " granular material points" << std::endl;
    std::cout << "Added " << roughness_points.size() << " fixed (ground) points" << std::endl;
    std::cout << "In total, added " << body_points.size() << std::endl;

    gpu_sys.SetTimeIntegrator(CHGPU_TIME_INTEGRATOR::EXTENDED_TAYLOR);
    gpu_sys.SetFixedStepSize(params.step_size);

    gpu_sys.SetVerbosity(params.verbose);
    // std::cout << "verbose: " << static_cast<int>(params.verbose) << std::endl;

    gpu_sys.Initialize();

#if !defined(CHRONO_OPENGL)
    render = false;
#endif

    std::shared_ptr<ChGpuVisualization> visGPU;
    if (render) {
#ifdef CHRONO_OPENGL
        visGPU = chrono_types::make_shared<ChGpuVisualizationGL>(&gpu_sys);
        visGPU->SetTitle("Chrono::Gpu repose demo");
        visGPU->AddCamera(ChVector3d(0, -30, -10), ChVector3d(0, 0, -20));
        visGPU->SetCameraMoveScale(1.0f);
        visGPU->Initialize();
#endif
    }

    float step = 1e-3;
    float time = 0.f;
    int sim_frame = 0;
    int render_frame = 0;

    // write an initial frame
    char filename[100];
    sprintf(filename, "%s/step%06d.csv", out_dir.c_str(), sim_frame);
    gpu_sys.WriteParticleFile(std::string(filename));

    char mesh_filename[100];
    sprintf(mesh_filename, "%s/step%06d_mesh", out_dir.c_str(), sim_frame);
    gpu_sys.WriteMeshes(std::string(mesh_filename));

    sim_frame++;

    while (time < params.time_end) {
        gpu_sys.AdvanceSimulation(step);

        if (render && time >= render_frame / render_fps) {
            if (!visGPU->Render())
                break;
            render_frame++;
        }

        printf("Output frame %u\n", sim_frame);
        sprintf(filename, "%s/step%06d.csv", out_dir.c_str(), sim_frame);
        gpu_sys.WriteParticleFile(std::string(filename));
        sprintf(mesh_filename, "%s/step%06d_mesh", out_dir.c_str(), sim_frame);
        gpu_sys.WriteMeshes(std::string(mesh_filename));

        float KE = gpu_sys.GetParticlesKineticEnergy();
        std::cout << "Total kinetic energy: " << KE << std::endl;
        unsigned int NumStillIn = gpu_sys.GetNumParticleAboveZ(funnel_bottom);
        std::cout << "Numer of particles still in funnel: " << NumStillIn << std::endl;

        time += step;
        sim_frame++;
    }

    return 0;
}
