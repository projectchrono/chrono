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
// Authors: Luning Fang
// =============================================================================
// Chrono::Gpu simulation of granular material settled in cylinder first, then
// compressed from a plate on top modelled as a boundary condition
// =============================================================================

#include <iostream>
#include <string>
#include <cmath>

#include "chrono/core/ChGlobal.h"
#include "chrono/core/ChStream.h"
#include "chrono/core/ChVector.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/utils/ChGpuJsonParser.h"

using namespace chrono;
using namespace chrono::gpu;

// unit conversion from cgs to si
float F_CGS_TO_SI = 1e-5f;
float KE_CGS_TO_SI = 1e-7f;
float L_CGS_TO_SI = 1e-2f;

int main(int argc, char* argv[]) {
    std::string inputJson = GetChronoDataFile("gpu/compression.json");
    if (argc == 2) {
        inputJson = std::string(argv[1]);
    } else if (argc > 2) {
        std::cout << "Usage:\n./demo_GPU_compression <json_file>" << std::endl;
        return 1;
    }

    ChGpuSimulationParameters params;
    if (!ParseJSON(inputJson, params)) {
        std ::cout << "ERROR: reading input file " << inputJson << std::endl;
        return 1;
    }

    // Setup simulation, big domain: 10 by 10 by 20
    ChSystemGpu gpu_sys(params.sphere_radius, params.sphere_density,
                        ChVector<float>(params.box_X, params.box_Y, params.box_Z));

    // One thing we can do is to move the Big Box Domain by (X/2, Y/2, Z/2) using SetBDCenter, so the
    // coordinate range we are now working with is (0,0,0) to (X,Y,Z), instead of (-X/2,-Y/2,-Z/2) to (X/2, Y/2, Z/2).
    gpu_sys.SetBDCenter(ChVector<float>(params.box_X / 2, params.box_Y / 2, params.box_Z / 2));

    // creat cylinder boundary of Radius 5 at the center of the box domain
    ChVector<float> cyl_center(params.box_X / 2, params.box_Y / 2, params.box_Z / 2);
    float cyl_rad = std::min(params.box_X, params.box_Y) / 2.0f;
    gpu_sys.CreateBCCylinderZ(cyl_center, cyl_rad, false, true);

    // initialize sampler, set distance between center of spheres as 2.1r
    utils::HCPSampler<float> sampler(2.1f * params.sphere_radius);
    std::vector<ChVector<float>> initialPos;

    // randomize by layer
    ChVector<float> center(params.box_X / 2, params.box_Y / 2, params.sphere_radius);
    // fill up each layer
    while (center.z() + params.sphere_radius < params.box_Z) {
        auto points = sampler.SampleCylinderZ(center, cyl_rad - params.sphere_radius, 0);
        initialPos.insert(initialPos.end(), points.begin(), points.end());
        center.z() += 2.1f * params.sphere_radius;
    }

    size_t numSpheres = initialPos.size();

    // create initial velocity vector
    std::vector<ChVector<float>> initialVelo;
    for (size_t i = 0; i < numSpheres; i++) {
        ChVector<float> velo(-initialPos.at(i).x() / cyl_rad, -initialPos.at(i).x() / cyl_rad, 0.0f);
        initialVelo.push_back(velo);
    }

    // assign initial position and velocity to the granular system
    gpu_sys.SetParticles(initialPos, initialVelo);

    gpu_sys.SetPsiFactors(params.psi_T, params.psi_L);

    // normal force model
    gpu_sys.SetKn_SPH2SPH(params.normalStiffS2S);
    gpu_sys.SetKn_SPH2WALL(params.normalStiffS2W);
    gpu_sys.SetGn_SPH2SPH(params.normalDampS2S);
    gpu_sys.SetGn_SPH2WALL(params.normalDampS2W);

    // assign tangential force model and its parameters
    gpu_sys.SetFrictionMode(CHGPU_FRICTION_MODE::MULTI_STEP);
    gpu_sys.SetKt_SPH2SPH(params.tangentStiffS2S);
    gpu_sys.SetKt_SPH2WALL(params.tangentStiffS2W);
    gpu_sys.SetGt_SPH2SPH(params.tangentDampS2S);
    gpu_sys.SetGt_SPH2WALL(params.tangentDampS2W);

    gpu_sys.SetStaticFrictionCoeff_SPH2SPH(params.static_friction_coeffS2W);
    gpu_sys.SetStaticFrictionCoeff_SPH2WALL(params.static_friction_coeffS2W);

    gpu_sys.SetCohesionRatio(params.cohesion_ratio);
    gpu_sys.SetAdhesionRatio_SPH2WALL(params.adhesion_ratio_s2w);

    gpu_sys.SetGravitationalAcceleration(ChVector<float>(params.grav_X, params.grav_Y, params.grav_Z));
    gpu_sys.SetParticleOutputMode(params.write_mode);

    std::string out_dir = GetChronoOutputPath() + "GPU/";
    filesystem::create_directory(filesystem::path(out_dir));
    out_dir = out_dir + params.output_dir;
    filesystem::create_directory(filesystem::path(out_dir));

    // Set the position of the BD fixed
    gpu_sys.SetBDFixed(true);
    gpu_sys.SetTimeIntegrator(CHGPU_TIME_INTEGRATOR::FORWARD_EULER);
    gpu_sys.SetFixedStepSize(params.step_size);

    gpu_sys.SetVerbosity(params.verbose);

    // create top plane boundary condition with its position and normal
    ChVector<float> topWallPos(params.box_X / 2, params.box_Y / 2, params.box_Z);
    ChVector<float> topWallNrm(0.0f, 0.0f, -1.0f);
    size_t topWall = gpu_sys.CreateBCPlane(topWallPos, topWallNrm, true);

    float topWall_vel;       // top plane moving velocity
    float topWall_offset;    // position offset of the plane when it first starts to move
    float topWall_moveTime;  // time when the plane first starts to move

    // user defined offset position function for top wall
    std::function<double3(float)> topWall_posFunc = [&topWall_offset, &topWall_vel, &topWall_moveTime](float t) {
        double3 pos = {0, 0, 0};
        pos.z = topWall_offset + topWall_vel * (t - topWall_moveTime);
        return pos;
    };

    gpu_sys.SetParticleOutputFlags(ABSV);
    gpu_sys.Initialize();

    // output frames per second
    int fps = 100;
    // assume we run for at least one frame
    float frame_step = 1.0f / fps;
    float curr_time = 0;
    int curr_frame = 0;
    unsigned int total_frames = (unsigned int)(((float)params.time_end - 0.5f) * fps) - 1;
    // initialize values that I want to keep track of
    ChVector<float> plane_reaction_force;
    ChVector<float> platePos;
    int nc;

    // let system run for 0.5 second so the particles can settle
    while (curr_time < 0.5) {
        gpu_sys.AdvanceSimulation(frame_step);
        curr_time += frame_step;
        printf("time = %.4f\n", curr_time);
    }

    // top plate move downward with velocity 1cm/s
    topWall_vel = -1.0f;
    // i would like it to start from the top most sphere
    topWall_offset = (float)gpu_sys.GetMaxParticleZ() + params.sphere_radius - topWallPos[2];
    topWall_moveTime = curr_time;

    // sphere settled now push the plate downward
    gpu_sys.SetBCOffsetFunction(topWall, topWall_posFunc);

    // continue simulation until the end
    while (curr_time < params.time_end) {
        printf("rendering frame: %u of %u, curr_time: %.4f, ", curr_frame + 1, total_frames, curr_time);

        // write position
        char filename[100];
        sprintf(filename, "%s/step%06d.csv", out_dir.c_str(), curr_frame);
        gpu_sys.WriteParticleFile(std::string(filename));
        gpu_sys.AdvanceSimulation(frame_step);

        platePos = gpu_sys.GetBCPlanePosition(topWall);
        std::cout << "top plate pos_z: " << platePos.z() << " cm";

        nc = gpu_sys.GetNumContacts();
        std::cout << ", numContacts: " << nc;

        gpu_sys.GetBCReactionForces(topWall, plane_reaction_force);
        std::cout << ", top plate force: " << plane_reaction_force.z() * F_CGS_TO_SI << " Newton";
        std::cout << "\n";

        curr_frame++;
        curr_time += frame_step;
    }
    return 0;
}
