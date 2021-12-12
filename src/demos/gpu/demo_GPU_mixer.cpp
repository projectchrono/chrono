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
// Chrono::Gpu evaluation of several simple mixer designs. Material consisting
// of spherical particles is let to aggitate in a rotating mixer. Metrics on the
// performance of each mixer can be determined in post-processing.
// =============================================================================

#include <cmath>
#include <iostream>
#include <string>

#include "chrono/core/ChGlobal.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_gpu/ChGpuData.h"
#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/utils/ChGpuJsonParser.h"
#include "chrono_gpu/utils/ChGpuVisualization.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::gpu;

// Output frequency
float out_fps = 200;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = true;
float render_fps = 2000;

int main(int argc, char* argv[]) {
    ChGpuSimulationParameters params;

    if (argc != 2 || ParseJSON(gpu::GetDataFile(argv[1]), params) == false) {
        std::cout << "Usage:\n./demo_GPU_mixer <json_file>" << std::endl;
        return 1;
    }

    const float Bx = params.box_X;
    const float By = Bx;
    const float chamber_height = Bx / 3;  // TODO
    const float fill_height = chamber_height;
    const float Bz = chamber_height + fill_height;
    std::cout << "Box Dims: " << Bx << " " << By << " " << Bz << std::endl;

    float iteration_step = params.step_size;

    ChSystemGpuMesh gpu_sys(params.sphere_radius, params.sphere_density, make_float3(Bx, By, Bz), make_float3((float)0., (float)0., (float)0.));

    gpu_sys.SetKn_SPH2SPH(params.normalStiffS2S);
    gpu_sys.SetKn_SPH2WALL(params.normalStiffS2W);
    gpu_sys.SetKn_SPH2MESH(params.normalStiffS2M);

    gpu_sys.SetKt_SPH2SPH(params.tangentStiffS2S);
    gpu_sys.SetKt_SPH2WALL(params.tangentStiffS2W);
    gpu_sys.SetKt_SPH2MESH(params.tangentStiffS2M);

    gpu_sys.SetGn_SPH2SPH(params.normalDampS2S);
    gpu_sys.SetGn_SPH2WALL(params.normalDampS2W);
    gpu_sys.SetGn_SPH2MESH(params.normalDampS2M);

    gpu_sys.SetGt_SPH2SPH(params.tangentDampS2S);
    gpu_sys.SetGt_SPH2WALL(params.tangentDampS2W);
    gpu_sys.SetGt_SPH2MESH(params.tangentDampS2M);

    gpu_sys.SetCohesionRatio(params.cohesion_ratio);
    gpu_sys.SetAdhesionRatio_SPH2MESH(params.adhesion_ratio_s2m);
    gpu_sys.SetAdhesionRatio_SPH2WALL(params.adhesion_ratio_s2w);
    gpu_sys.SetFrictionMode(chrono::gpu::CHGPU_FRICTION_MODE::MULTI_STEP);

    gpu_sys.SetStaticFrictionCoeff_SPH2SPH(params.static_friction_coeffS2S);
    gpu_sys.SetStaticFrictionCoeff_SPH2WALL(params.static_friction_coeffS2W);
    gpu_sys.SetStaticFrictionCoeff_SPH2MESH(params.static_friction_coeffS2M);

    gpu_sys.SetOutputMode(params.write_mode);

    std::string out_dir = GetChronoOutputPath() + "GPU/";
    filesystem::create_directory(filesystem::path(out_dir));
    out_dir = out_dir + params.output_dir;
    filesystem::create_directory(filesystem::path(out_dir));

    gpu_sys.SetTimeIntegrator(CHGPU_TIME_INTEGRATOR::CENTERED_DIFFERENCE);
    gpu_sys.SetFixedStepSize(params.step_size);
    gpu_sys.SetBDFixed(true);

    const float chamber_bottom = -Bz / 2.f;
    const float fill_bottom = chamber_bottom + chamber_height;

    ChVector<float> cyl_center(0, 0, 0);
    const float cyl_rad = Bx / 2.f;
    gpu_sys.CreateBCCylinderZ(cyl_center, cyl_rad, false, false);

    utils::HCPSampler<float> sampler(2.1f * params.sphere_radius);
    std::vector<ChVector<float>> body_points;

    const float fill_radius = Bx / 2.f - 2.f * params.sphere_radius;
    const float fill_top = fill_bottom + fill_height;

    std::cout << "Created " << body_points.size() << " spheres" << std::endl;
    std::cout << "Fill radius " << fill_radius << std::endl;
    std::cout << "Fill bottom " << fill_bottom << std::endl;
    std::cout << "Fill top " << fill_top << std::endl;

    ChVector<float> center(0, 0, fill_bottom);
    center.z() += 2 * params.sphere_radius;
    while (center.z() < fill_top - 2 * params.sphere_radius) {
        auto points = sampler.SampleCylinderZ(center, fill_radius, 0);
        body_points.insert(body_points.end(), points.begin(), points.end());
        center.z() += 2.1f * params.sphere_radius;
    }

    gpu_sys.SetParticlePositions(body_points);
    gpu_sys.SetGravitationalAcceleration(ChVector<float>(0, 0, -980));

    std::vector<string> mesh_filenames;
    mesh_filenames.push_back(GetChronoDataFile("models/mixer/internal_mixer.obj"));

    std::vector<ChMatrix33<float>> mesh_rotscales;
    std::vector<float3> mesh_translations;

    float scale_xy = Bx / 2.f;
    float scale_z = chamber_height;  // TODO fix this / make switch on mixer_type
    float3 scaling = make_float3(scale_xy, scale_xy, scale_z);
    mesh_rotscales.push_back(ChMatrix33<float>(ChVector<float>(scaling.x, scaling.y, scaling.z)));
    mesh_translations.push_back(make_float3(0, 0, 0));

    std::vector<float> mesh_masses;
    float mixer_mass = 10;
    mesh_masses.push_back(mixer_mass);

    gpu_sys.LoadMeshes(mesh_filenames, mesh_rotscales, mesh_translations, mesh_masses);

    std::cout << gpu_sys.GetNumMeshes() << " meshes" << std::endl;

    float rev_per_sec = 1.f;
    float ang_vel_Z = rev_per_sec * 2 * (float)CH_C_PI;
    ChVector<> mesh_lin_vel(0);
    ChVector<> mesh_ang_vel(0, 0, ang_vel_Z);

    gpu_sys.EnableMeshCollision(true);
    gpu_sys.Initialize();

    ChGpuVisualization gpu_vis(&gpu_sys);
    std::shared_ptr<ChBody> mixer;
    if (render) {
        // Create proxy body for mixer mesh
        mixer = chrono_types::make_shared<ChBody>();
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh->LoadWavefrontMesh(GetChronoDataFile("models/mixer/internal_mixer.obj"));
        trimesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(ChVector<>(scaling.x, scaling.y, scaling.z)));
        trimesh_shape->SetMesh(trimesh);
        mixer->AddAsset(trimesh_shape);
        gpu_vis.AddProxyBody(mixer);

        gpu_vis.SetTitle("Chrono::Gpu mixer demo");
        gpu_vis.SetCameraPosition(ChVector<>(0, -100, 75), ChVector<>(0, 0, 0));
        gpu_vis.SetCameraMoveScale(1.0f);
        gpu_vis.Initialize();
    }

    unsigned int out_steps = (unsigned int)(1.0f / (out_fps * iteration_step));
    unsigned int render_steps = (unsigned int)(1.0 / (render_fps * iteration_step));
    unsigned int total_frames = (unsigned int)(params.time_end * out_fps);
    std::cout << "out_steps " << out_steps << std::endl;

    unsigned int currframe = 0;
    unsigned int step = 0;

    for (float t = 0; t < params.time_end; t += iteration_step, step++) {
        ChVector<> mesh_pos(0, 0, chamber_bottom + chamber_height / 2.0);
        ChQuaternion<> mesh_rot = Q_from_AngZ(t * ang_vel_Z);
        gpu_sys.ApplyMeshMotion(0, mesh_pos, mesh_rot, mesh_lin_vel, mesh_ang_vel);

        if (step % out_steps == 0) {
            std::cout << "Output frame " << (currframe + 1) << " of " << total_frames << std::endl;
            char filename[100];
            sprintf(filename, "%s/step%06u", out_dir.c_str(), currframe++);
            gpu_sys.WriteFile(std::string(filename));
            gpu_sys.WriteMeshes(std::string(filename));

            ChVector<> force;
            ChVector<> torque;
            gpu_sys.CollectMeshContactForces(0, force, torque);
            std::cout << "torque: " << torque.x() << ", " << torque.y() << ", " << torque.z() << std::endl;
        }

        if (render && step % render_steps == 0) {
            mixer->SetPos(mesh_pos);
            mixer->SetRot(mesh_rot);
            if (gpu_vis.Render())
                break;
        }

        gpu_sys.AdvanceSimulation(iteration_step);
    }

    return 0;
}
