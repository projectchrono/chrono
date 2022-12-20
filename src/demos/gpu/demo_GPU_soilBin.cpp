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

#include "chrono/core/ChGlobal.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/utils/ChGpuJsonParser.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::gpu;

int main(int argc, char* argv[]) {
    std::string inputJson = GetChronoDataFile("gpu/soilBin.json");
    if (argc == 2) {
        inputJson = std::string(argv[1]);
    } else if (argc > 2) {
        std::cout << "Usage:\n./demo_GPU_soilBin <json_file>" << std::endl;
        return 1;
    }

    ChGpuSimulationParameters params;
    if (!ParseJSON(inputJson, params)) {
        std ::cout << "ERROR: reading input file " << inputJson << std::endl;
        return 1;
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
        case CHGPU_RUN_MODE::MULTI_STEP:
            gpu_sys.SetFrictionMode(CHGPU_FRICTION_MODE::MULTI_STEP);
            gpu_sys.SetKt_SPH2SPH(params.tangentStiffS2S);
            gpu_sys.SetKt_SPH2WALL(params.tangentStiffS2W);
            gpu_sys.SetGt_SPH2SPH(params.tangentDampS2S);
            gpu_sys.SetGt_SPH2WALL(params.tangentDampS2W);
            break;
        case CHGPU_RUN_MODE::ONE_STEP:
            gpu_sys.SetFrictionMode(CHGPU_FRICTION_MODE::SINGLE_STEP);
            gpu_sys.SetKt_SPH2SPH(params.tangentStiffS2S);
            gpu_sys.SetKt_SPH2WALL(params.tangentStiffS2W);
            gpu_sys.SetGt_SPH2SPH(params.tangentDampS2S);
            gpu_sys.SetGt_SPH2WALL(params.tangentDampS2W);
            break;
        case CHGPU_RUN_MODE::FRICTIONLESS:
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
