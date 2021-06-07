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
// Authors: Conlain Kelly, Nic Olsen
// =============================================================================
// Simple Chrono::Gpu settling experiment which allows for sweeping various
// simulation parameters to produce scaling analyses.
// =============================================================================

#include <iostream>
#include <string>
#include <cmath>
#include <chrono>

#include "GpuDemoUtils.hpp"

#include "chrono/core/ChGlobal.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_gpu/ChGpuData.h"
#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/utils/ChGpuJsonParser.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::gpu;

// expected number of args for param sweep
constexpr int num_args_full = 6;

// -----------------------------------------------------------------------------
// Show command line usage
// -----------------------------------------------------------------------------
void ShowUsage(std::string name) {
    std::cout << "usage: " + name + " <json_file> [<radius> <run_mode> <box_width> <output_dir>]" << std::endl;
    std::cout << "must have either 1 or " << num_args_full - 1 << " arguments" << std::endl;
}

enum run_mode { FRICTIONLESS = 0, ONE_STEP = 1, MULTI_STEP = 2 };

int main(int argc, char* argv[]) {
    ChGpuSimulationParameters params;

    // Some of the default values are overwritten by user via command line
    if (argc < 2 || (argc > 2 && argc != num_args_full) || ParseJSON(gpu::GetDataFile(argv[1]), params) == false) {
        ShowUsage(argv[0]);
        return 1;
    }

    if (argc == num_args_full) {
        params.sphere_radius = std::stof(argv[2]);
        params.run_mode = std::atoi(argv[3]);
        params.box_Y = std::stof(argv[4]);
        params.box_X = params.box_Y;
        params.output_dir = std::string(argv[5]);
        printf("new parameters: r is %f, run_mode is %d, width is %f, %s\n", params.sphere_radius, params.run_mode,
               params.box_Y, params.output_dir.c_str());
    }

    // Setup simulation
    ChSystemGpu gpu_sys(params.sphere_radius, params.sphere_density,
                        ChVector<float>(params.box_X, params.box_Y, params.box_Z));

    gpu_sys.SetPsiFactors(params.psi_T, params.psi_L);

    gpu_sys.SetKn_SPH2SPH(params.normalStiffS2S);
    gpu_sys.SetKn_SPH2WALL(params.normalStiffS2W);
    gpu_sys.SetGn_SPH2SPH(params.normalDampS2S);
    gpu_sys.SetGn_SPH2WALL(params.normalDampS2W);

    gpu_sys.SetKt_SPH2SPH(params.tangentStiffS2S);
    gpu_sys.SetKt_SPH2WALL(params.tangentStiffS2W);
    gpu_sys.SetGt_SPH2SPH(params.tangentDampS2S);
    gpu_sys.SetGt_SPH2WALL(params.tangentDampS2W);

    gpu_sys.SetStaticFrictionCoeff_SPH2SPH(params.static_friction_coeffS2S);
    gpu_sys.SetStaticFrictionCoeff_SPH2WALL(params.static_friction_coeffS2W);

    gpu_sys.SetCohesionRatio(params.cohesion_ratio);
    gpu_sys.SetAdhesionRatio_SPH2WALL(params.adhesion_ratio_s2w);

    gpu_sys.SetGravitationalAcceleration(ChVector<float>(params.grav_X, params.grav_Y, params.grav_Z));
    gpu_sys.SetParticleOutputMode(params.write_mode);

    gpu_sys.SetRollingMode(CHGPU_ROLLING_MODE::NO_RESISTANCE);

    std::vector<ChVector<float>> body_points;

    {
        // fill box, layer by layer
        ChVector<> hdims(params.box_X / 2.f - 2 * params.sphere_radius, params.box_Y / 2.f - 2 * params.sphere_radius,
                         params.box_Z / 2.f - 2 * params.sphere_radius);
        ChVector<> center(0, 0, 0);

        utils::HCPSampler<float> sampler(2.2f * params.sphere_radius);

        body_points = sampler.SampleBox(center, hdims);
    }

    gpu_sys.SetParticles(body_points);
    std::cout << "Added " << body_points.size() << std::endl;

    switch (params.run_mode) {
        case run_mode::MULTI_STEP:
            gpu_sys.SetFrictionMode(CHGPU_FRICTION_MODE::MULTI_STEP);
            gpu_sys.SetKt_SPH2SPH(params.tangentStiffS2S);
            gpu_sys.SetKt_SPH2WALL(params.tangentStiffS2W);
            gpu_sys.SetGt_SPH2SPH(params.tangentDampS2S);
            gpu_sys.SetGt_SPH2WALL(params.tangentDampS2W);
            break;
        case run_mode::ONE_STEP:
            gpu_sys.SetFrictionMode(CHGPU_FRICTION_MODE::SINGLE_STEP);
            gpu_sys.SetKt_SPH2SPH(params.tangentStiffS2S);
            gpu_sys.SetKt_SPH2WALL(params.tangentStiffS2W);
            gpu_sys.SetGt_SPH2SPH(params.tangentDampS2S);
            gpu_sys.SetGt_SPH2WALL(params.tangentDampS2W);
            break;
        case run_mode::FRICTIONLESS:
            gpu_sys.SetFrictionMode(CHGPU_FRICTION_MODE::FRICTIONLESS);
            break;
        default:
            std::cout << "Invalid run mode" << std::endl;
            return 1;
    }

    gpu_sys.SetTimeIntegrator(CHGPU_TIME_INTEGRATOR::EXTENDED_TAYLOR);
    gpu_sys.SetFixedStepSize(params.step_size);

    std::string out_dir;
    if (params.write_mode != CHGPU_OUTPUT_MODE::NONE) {
        out_dir = GetChronoOutputPath() + "GPU/";
        filesystem::create_directory(filesystem::path(out_dir));
        out_dir = out_dir + params.output_dir;
        filesystem::create_directory(filesystem::path(out_dir));
    }
    gpu_sys.SetBDFixed(true);

    gpu_sys.SetVerbosity(params.verbose);
    gpu_sys.Initialize();

    int fps = 50;
    float frame_step = 1.f / fps;
    float curr_time = 0;
    int currframe = 0;
    unsigned int total_frames = (unsigned int)((float)params.time_end * fps);

    // write an initial frame
    char filename[100];
    sprintf(filename, "%s/step%06d.csv", out_dir.c_str(), currframe++);
    gpu_sys.WriteParticleFile(std::string(filename));

    std::cout << "frame step is " << frame_step << std::endl;

    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    while (curr_time < params.time_end) {
        gpu_sys.AdvanceSimulation(frame_step);
        curr_time += frame_step;
        printf("rendering frame %u of %u\n", currframe, total_frames + 1);
        sprintf(filename, "%s/step%06d.csv", out_dir.c_str(), currframe++);
        gpu_sys.WriteParticleFile(std::string(filename));
    }
    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_sec = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
    std::cout << time_sec.count() << " seconds" << std::endl;

    return 0;
}
