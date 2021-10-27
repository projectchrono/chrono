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

#include "chrono/core/ChGlobal.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_gpu/ChGpuData.h"
#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/utils/ChGpuJsonParser.h"
#include "chrono_gpu/utils/ChGpuVisualization.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::gpu;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = true;

int main(int argc, char* argv[]) {
    ChGpuSimulationParameters params;

    // Some of the default values might be overwritten by user via command line
    if (argc != 2 || ParseJSON(gpu::GetDataFile(argv[1]), params) == false) {
        std::cout << "Usage:\n./demo_GPU_repose <json_file>" << std::endl;
        return 1;
    }

    std::string out_dir = GetChronoOutputPath() + "GPU/";
    filesystem::create_directory(filesystem::path(out_dir));
    out_dir = out_dir + params.output_dir;
    filesystem::create_directory(filesystem::path(out_dir));

    // Setup simulation
    ChSystemGpu gpu_sys(params.sphere_radius, params.sphere_density,
                        ChVector<float>(params.box_X, params.box_Y, params.box_Z));

    gpu_sys.SetKn_SPH2SPH(params.normalStiffS2S);
    gpu_sys.SetKn_SPH2WALL(params.normalStiffS2W);
    gpu_sys.SetGn_SPH2SPH(params.normalDampS2S);
    gpu_sys.SetGn_SPH2WALL(params.normalDampS2W);

    // gpu_sys.SetFrictionMode(CHGPU_FRICTION_MODE::FRICTIONLESS);
    gpu_sys.SetFrictionMode(CHGPU_FRICTION_MODE::MULTI_STEP);
    gpu_sys.SetKt_SPH2SPH(params.tangentStiffS2S);
    gpu_sys.SetKt_SPH2WALL(params.tangentStiffS2W);
    gpu_sys.SetGt_SPH2SPH(params.tangentDampS2S);
    gpu_sys.SetGt_SPH2WALL(params.tangentDampS2W);
    gpu_sys.SetStaticFrictionCoeff_SPH2SPH(params.static_friction_coeffS2S);
    gpu_sys.SetStaticFrictionCoeff_SPH2WALL(params.static_friction_coeffS2W);

    // gpu_sys.SetRollingMode(CHGPU_ROLLING_MODE::NO_RESISTANCE);
    gpu_sys.SetRollingMode(CHGPU_ROLLING_MODE::SCHWARTZ);
    gpu_sys.SetRollingCoeff_SPH2SPH(params.rolling_friction_coeffS2S);
    gpu_sys.SetRollingCoeff_SPH2WALL(params.rolling_friction_coeffS2W);

    gpu_sys.SetCohesionRatio(params.cohesion_ratio);
    gpu_sys.SetAdhesionRatio_SPH2WALL(params.adhesion_ratio_s2w);
    gpu_sys.SetGravitationalAcceleration(ChVector<float>(params.grav_X, params.grav_Y, params.grav_Z));
    gpu_sys.SetParticleOutputMode(params.write_mode);

    gpu_sys.SetBDFixed(true);

    // padding in sampler
    float fill_epsilon = 2.02f;
    // padding at top of fill
    ////float drop_height = 0.f;
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
    ////float fill_top = fill_bottom + fill_height;

    ChVector<float> center(0.f, 0.f, fill_bottom + fill_height / 2.f);
    material_points = sampler.SampleCylinderZ(center, fill_width, fill_height / 2.f);

    std::vector<ChVector<float>> body_points;
    std::vector<bool> body_points_fixed;
    body_points.insert(body_points.end(), roughness_points.begin(), roughness_points.end());
    body_points_fixed.insert(body_points_fixed.end(), roughness_points.size(), true);

    body_points.insert(body_points.end(), material_points.begin(), material_points.end());
    body_points_fixed.insert(body_points_fixed.end(), material_points.size(), false);

    gpu_sys.SetParticles(body_points);
    gpu_sys.SetParticleFixed(body_points_fixed);

    std::cout << "Added " << roughness_points.size() << " fixed points" << std::endl;
    std::cout << "Added " << material_points.size() << " material points" << std::endl;

    std::cout << "Actually added " << body_points.size() << std::endl;

    gpu_sys.SetTimeIntegrator(CHGPU_TIME_INTEGRATOR::EXTENDED_TAYLOR);
    gpu_sys.SetFixedStepSize(params.step_size);

    gpu_sys.SetVerbosity(params.verbose);
    std::cout << "verbose: " << static_cast<int>(params.verbose) << std::endl;
    gpu_sys.SetRecordingContactInfo(true);

    gpu_sys.Initialize();

    ChGpuVisualization gpu_vis(&gpu_sys);
    if (render) {
        gpu_vis.SetTitle("Chrono::Gpu repose demo");
        gpu_vis.SetCameraPosition(ChVector<>(0, -30, -10), ChVector<>(0, 0, -20));
        gpu_vis.SetCameraMoveScale(1.0f);
        gpu_vis.Initialize();
    }

    int fps = 60;
    float frame_step = 1.f / fps;
    float curr_time = 0.f;
    int currframe = 0;
    unsigned int total_frames = (unsigned int)((float)params.time_end * fps);

    // write an initial frame
    char filename[100];
    sprintf(filename, "%s/step%06d.csv", out_dir.c_str(), currframe);
    gpu_sys.WriteParticleFile(std::string(filename));

    char contactFilename[100];
    sprintf(contactFilename, "%s/contact%06d.csv", out_dir.c_str(), currframe);
    gpu_sys.WriteContactInfoFile(std::string(contactFilename));

    currframe++;

    std::cout << "frame step is " << frame_step << std::endl;
    while (curr_time < params.time_end) {
        gpu_sys.AdvanceSimulation(frame_step);

        if (render && gpu_vis.Render())
            break;

        printf("Output frame %u of %u\n", currframe, total_frames);
        sprintf(filename, "%s/step%06d.csv", out_dir.c_str(), currframe);
        gpu_sys.WriteParticleFile(std::string(filename));

        sprintf(contactFilename, "%s/contact%06d.csv", out_dir.c_str(), currframe);
        gpu_sys.WriteContactInfoFile(std::string(contactFilename));

        curr_time += frame_step;
        currframe++;
    }

    return 0;
}
