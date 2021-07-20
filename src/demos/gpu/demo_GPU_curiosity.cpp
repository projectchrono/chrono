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
// Authors: Ruochun
// =============================================================================
// Chrono::Gpu demo to show usage of curiosity models on granular terrain
// =============================================================================

#include <iostream>
#include <vector>
#include <string>
#include <cassert>
#include <cstdlib>
#include <ctime>

#include "chrono_models/robot/curiosity/Curiosity.h"

#include "chrono/core/ChGlobal.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChForce.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/physics/ChParticlesClones.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChLinkDistance.h"

#include "chrono_gpu/ChGpuData.h"
#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/utils/ChGpuJsonParser.h"
#include "chrono_gpu/utils/ChGpuVisualization.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::gpu;
using namespace collision;
using namespace chrono::geometry;
using namespace chrono::curiosity;
// Output frequency
int out_fps = 1;

void ShowUsage(std::string name) {
    std::cout << "usage: " + name + " <json_file>" << std::endl;
}

// Choose Curiosity rover chassis type
Chassis_Type chassis_type = Chassis_Type::FullRover;

int main(int argc, char* argv[]) {
    ChGpuSimulationParameters params;
    if (argc != 2 || ParseJSON(gpu::GetDataFile(argv[1]), params) == false) {
        std::cout << "Usage:\n./demo_GPU_mixer <json_file>" << std::endl;
        return 1;
    }

    float iteration_step = params.step_size;
    double scale_ratio = 100.0;
    double settle_time = 1.0;
    double max_advance_dist = 4.75 * scale_ratio;  // max, reaches the rim of the bin
    // double max_advance_dist = 1.0*scale_ratio; // advance 1m, for timing tests

    ChSystemGpuMesh gpu_sys(params.sphere_radius, params.sphere_density,
                            ChVector<float>(params.box_X, params.box_Y, params.box_Z));

    double road_bottom = -params.box_Z / 2.0 + 2.05 * params.sphere_radius;
    double road_top = -params.box_Z / 2.0 + 0.2 * scale_ratio;
    double pile_top = road_top + 0.8 * scale_ratio;  // high height

    unsigned int num_points = 0;
    // chrono::utils::PDSampler<float> sampler(2.4f * params.sphere_radius);
    chrono::utils::GridSampler<float> sampler(2.05 * params.sphere_radius);
    std::vector<ChVector<float>> body_points;

    ChVector<> road_hdims(params.box_X / 2.0 - 2.05 * params.sphere_radius,
                          params.box_Y / 2.0 - 2.05 * params.sphere_radius, (road_top - road_bottom) / 2.0);
    ChVector<> road_center(0, 0, (road_top + road_bottom) / 2.0 + 0.0 * params.sphere_radius);
    {
        auto points = sampler.SampleBox(road_center, road_hdims);
        body_points.insert(body_points.end(), points.begin(), points.end());
        num_points += points.size();
    }

    ChVector<> hdims(1.0 * scale_ratio - params.sphere_radius, params.box_Y / 2.0 - 2.05 * params.sphere_radius,
                     (pile_top - road_top) / 2.0);
    ChVector<> center(-1.0 * scale_ratio, 0, (pile_top + road_top) / 2.0 + 4.05 * params.sphere_radius);
    {
        auto points = sampler.SampleBox(center, hdims);
        body_points.insert(body_points.end(), points.begin(), points.end());
        num_points += points.size();
    }

    std::cout << "Done creating particles, " << num_points << " of them created." << std::endl;
    gpu_sys.SetParticles(body_points);
    gpu_sys.SetBDFixed(true);

    gpu_sys.SetKn_SPH2SPH(params.normalStiffS2S);
    gpu_sys.SetKn_SPH2WALL(params.normalStiffS2W);
    gpu_sys.SetKn_SPH2MESH(params.normalStiffS2M);

    gpu_sys.SetGn_SPH2SPH(params.normalDampS2S);
    gpu_sys.SetGn_SPH2WALL(params.normalDampS2W);
    gpu_sys.SetGn_SPH2MESH(params.normalDampS2M);

    gpu_sys.SetKt_SPH2SPH(params.tangentStiffS2S);
    gpu_sys.SetKt_SPH2WALL(params.tangentStiffS2W);
    gpu_sys.SetKt_SPH2MESH(params.tangentStiffS2M);

    gpu_sys.SetGt_SPH2SPH(params.tangentDampS2S);
    gpu_sys.SetGt_SPH2WALL(params.tangentDampS2W);
    gpu_sys.SetGt_SPH2MESH(params.tangentDampS2M);

    gpu_sys.SetCohesionRatio(params.cohesion_ratio);
    gpu_sys.SetAdhesionRatio_SPH2MESH(params.adhesion_ratio_s2m);
    gpu_sys.SetAdhesionRatio_SPH2WALL(params.adhesion_ratio_s2w);

    gpu_sys.SetGravitationalAcceleration(ChVector<float>(params.grav_X, params.grav_Y, params.grav_Z));

    gpu_sys.SetFixedStepSize(params.step_size);
    gpu_sys.SetFrictionMode(CHGPU_FRICTION_MODE::MULTI_STEP);
    gpu_sys.SetTimeIntegrator(CHGPU_TIME_INTEGRATOR::CENTERED_DIFFERENCE);

    gpu_sys.SetStaticFrictionCoeff_SPH2SPH(params.static_friction_coeffS2S);
    gpu_sys.SetStaticFrictionCoeff_SPH2WALL(params.static_friction_coeffS2W);
    gpu_sys.SetStaticFrictionCoeff_SPH2MESH(params.static_friction_coeffS2M);

    // gpu_sys.SetRollingCoeff_SPH2SPH(params.rolling_friction_coeffS2S);
    // gpu_sys.SetRollingCoeff_SPH2WALL(params.rolling_friction_coeffS2W);
    // gpu_sys.SetRollingCoeff_SPH2MESH(params.rolling_friction_coeffS2M);

    auto mysurfmaterial = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    double overallPosX = 2.3 * scale_ratio;
    double overallPosY = 0.0;
    double overallPosZ = -params.box_Z / 3.0 - 0.1 * scale_ratio;
    ChSystemSMC curiosity_sys;
    std::vector<string> mesh_filenames;
    std::vector<ChMatrix33<float>> mesh_rotscales;
    std::vector<float> mesh_masses;
    std::vector<float3> mesh_translations;

    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025 * scale_ratio);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.0025 * scale_ratio);

    auto mfloor = chrono_types::make_shared<ChBodyEasyBox>(params.box_X, params.box_Y, 0.02 * scale_ratio, 1000, false,
                                                           true, mysurfmaterial);
    mfloor->SetPos(ChVector<>(0, 0, -params.box_Z / 2.0 - 0.01 * scale_ratio));
    mfloor->SetBodyFixed(true);
    curiosity_sys.Add(mfloor);

    // create rover
    ChQuaternion<> body_rot = ChQuaternion<>(1, 0, 0, 0);
    std::shared_ptr<CuriosityRover> rover = chrono_types::make_shared<CuriosityRover>(
        &curiosity_sys, ChVector<>(overallPosX, overallPosY, overallPosZ), body_rot, mysurfmaterial, chassis_type);
    rover->Initialize();

    gpu_sys.SetParticleOutputMode(params.write_mode);
    gpu_sys.SetVerbosity(params.verbose);
    filesystem::create_directory(filesystem::path(params.output_dir));

    // only 6 wheels go into the simulation meshes list
    for (int i = 0; i < 6; i++) {
        gpu_sys.AddMesh(rover->GetWheelPart((WheelID)i)->GetTrimesh(), rover->GetWheelMass());
    }
    gpu_sys.EnableMeshCollision(false);
    std::cout << "Initializing the GPU system!" << std::endl;
    gpu_sys.Initialize();

    curiosity_sys.SetContactForceModel(ChSystemSMC::ContactForceModel::Hooke);
    curiosity_sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
    curiosity_sys.Set_G_acc(ChVector<>(0, 0, params.grav_Z));

    std::cout << "Output at    " << out_fps << " FPS" << std::endl;
    unsigned int out_steps = (unsigned int)(1.0 / (out_fps * iteration_step));
    unsigned int total_frames = (unsigned int)(params.time_end * out_fps);

    int currframe = 0;
    unsigned int curr_step = 0;
    bool viper_enabled = false;

    clock_t start = std::clock();
    for (double t = 0; t < (double)params.time_end; t += iteration_step, curr_step++) {
        if (!viper_enabled && t > settle_time) {
            gpu_sys.EnableMeshCollision(true);
            viper_enabled = true;
        }
        auto rover_body = rover->GetChassisBody();
        if (overallPosX - rover_body->GetFrame_REF_to_abs().GetPos().x() >= max_advance_dist)
            break;  // if it moved long enough then we stop the simulation

        // 6 wheels
        for (int i = 0; i < 6; i++) {  // Ground (0) and body (1), not interested
            auto wheel_body = rover->GetWheelBody((WheelID)i);
            gpu_sys.ApplyMeshMotion(
                i, wheel_body->GetFrame_REF_to_abs().GetPos(), wheel_body->GetFrame_REF_to_abs().GetRot(),
                wheel_body->GetFrame_REF_to_abs().GetPos_dt(), wheel_body->GetFrame_REF_to_abs().GetWvel_par());

            ChVector<> Body_force;
            ChVector<> Body_torque;
            gpu_sys.CollectMeshContactForces(i, Body_force, Body_torque);

            wheel_body->Empty_forces_accumulators();
            wheel_body->Accumulate_force(Body_force, wheel_body->GetFrame_REF_to_abs().GetPos(), false);
            wheel_body->Accumulate_torque(Body_torque, false);
        }

        if (curr_step % out_steps == 0) {
            std::cout << "Output frame " << currframe + 1 << " of " << total_frames << std::endl;
            char filename[100];
            sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
            gpu_sys.WriteParticleFile(std::string(filename));
        }

        gpu_sys.AdvanceSimulation(iteration_step);
        if (viper_enabled)
            curiosity_sys.DoStepDynamics(iteration_step);
    }

    clock_t end = std::clock();
    double total_time = ((double)(end - start)) / CLOCKS_PER_SEC;

    std::cout << "Time: " << total_time << " seconds" << std::endl;

    return 0;
}
