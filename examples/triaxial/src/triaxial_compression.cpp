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
// Authors: Wassim Kassem
// =============================================================================
// Simulation of granular material settled in cylinder first, then
// subjected to a triaxial test. Adapted from demo_GPU_compression
// =============================================================================

#include <iostream>
#include <string>
#include <cmath>

#include "chrono/core/ChGlobal.h"
#include "chrono/core/ChStream.h"
#include "chrono/core/ChVector.h"
#include "chrono/physics/ChBody.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_gpu/ChGpuData.h"
#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/utils/ChGpuJsonParser.h"
#include "chrono_gpu/utils/ChGpuVisualization.h"


using namespace chrono;
using namespace chrono::gpu;

// unit conversion from cgs to si
float F_CGS_TO_SI = 1e-5f;
float KE_CGS_TO_SI = 1e-7f;
float L_CGS_TO_SI = 1e-2f;

// Output frequency
float out_fps = 200;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = false;
float render_fps = 2000;

int main(int argc, char* argv[]) {
    ChGpuSimulationParameters params;
    if (argc != 2 || ParseJSON(argv[1], params) == false) {
        std::cout << "Usage:\n./triaxial <json_file>" << std::endl;
        return 1;
    }

    // Setup simulation, big domain: 10 by 10 by 20
    const float Bx = params.box_X;
    const float By = Bx;
    const float chamber_height = Bx / 6;  // TODO
    const float fill_height = chamber_height;
    const float Bz = chamber_height + fill_height;
    std::cout << "Box Dims: " << Bx << " " << By << " " << Bz << std::endl;

    float iteration_step = params.step_size;
    ChSystemGpuMesh gpu_sys(params.sphere_radius, params.sphere_density, make_float3(Bx, By, Bz), make_float3((float)0.,(float)0.,(float)0.));


    // One thing we can do is to move the Big Box Domain by (X/2, Y/2, Z/2) using SetBDCenter, so the
    // coordinate range we are now working with is (0,0,0) to (X,Y,Z), instead of (-X/2,-Y/2,-Z/2) to (X/2, Y/2, Z/2).
    gpu_sys.SetBDCenter(ChVector<float>(Bx / 2, By / 2, Bz / 2));

    // creat cylinder boundary of Radius 5 at the center of the box domain
    ChVector<float> cyl_center(Bx / 2, By / 2, Bz / 2);
    float cyl_rad = std::min(Bx, By) / 8.0f;
    // gpu_sys.CreateBCCylinderZ(cyl_center, cyl_rad, false, true); // original cylinder used to confine particles

    // initialize sampler, set distance between center of spheres as 2.1r
    utils::PDSampler<float> sampler(2.1f * params.sphere_radius);
    std::vector<ChVector<float>> initialPos;

    // randomize by layer
    ChVector<float> center(Bx / 2, By / 2, params.sphere_radius+10); //Fix units for 10 
    // fill up each layer
    while (center.z() + params.sphere_radius < Bz) {
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

    // create initial angular velocity vector
    std::vector<ChVector<float>> initialAnglVelo;
    for (size_t i = 0; i < numSpheres; i++) {
        ChVector<float> velo(0, 0, 0); //TODO
        initialAnglVelo.push_back(velo);
    }

    // assign initial position and velocity to the granular system
    gpu_sys.SetParticlePositions(initialPos, initialVelo, initialAnglVelo);
    gpu_sys.SetPsiFactors(params.psi_T, params.psi_L);

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

    gpu_sys.SetGravitationalAcceleration(ChVector<float>(params.grav_X, params.grav_Y, params.grav_Z));
    
    // output parameters
    gpu_sys.SetOutputMode(params.write_mode);
    gpu_sys.SetOutputFlags(ABSV);
    std::string out_dir = "./";
    filesystem::create_directory(filesystem::path(out_dir));
    out_dir = out_dir + params.output_dir;
    filesystem::create_directory(filesystem::path(out_dir));

    // Set the position of the BD fixed
    gpu_sys.SetBDFixed(true);
    gpu_sys.SetTimeIntegrator(CHGPU_TIME_INTEGRATOR::FORWARD_EULER);
    gpu_sys.SetFixedStepSize(params.step_size);

    gpu_sys.SetVerbosity(params.verbose);

    // create top plane boundary condition with its position and normal
    ChVector<float> topWallPos(Bx / 2, By / 2, Bz);
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

    // add the mixer mesh to the GPU system
    float scale_xy = 1.1 * cyl_rad;
    float scale_z = chamber_height;
    ChVector<> scaling(scale_xy, scale_xy, scale_z);
    float mixer_mass = 10;
    auto cylinder_mesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    cylinder_mesh->LoadWavefrontMesh("./models/open_cylinder_subdivided.obj", true, false);
    cylinder_mesh->Transform(ChVector<>(Bx/2, By/2, Bz/2+2.0), ChMatrix33<>(scaling));
    auto cylinder_mesh_id = gpu_sys.AddMesh(cylinder_mesh, mixer_mass);

    // initialize the system
    gpu_sys.EnableMeshCollision(true);
    gpu_sys.Initialize();

    // visualisation options
    ChGpuVisualization gpu_vis(&gpu_sys);
    std::shared_ptr<ChBody> mixer;
    if (render) {
        // Create proxy body for mixer mesh
        mixer = chrono_types::make_shared<ChBody>();
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(cylinder_mesh);
        mixer->AddAsset(trimesh_shape);
        gpu_vis.AddProxyBody(mixer);

        gpu_vis.SetTitle("Chrono::Gpu mixer demo");
        gpu_vis.SetCameraPosition(ChVector<>(0, -100, 75), ChVector<>(0, 0, 0));
        gpu_vis.SetCameraMoveScale(1.0f);
        gpu_vis.Initialize();
    }

    // assume we run for at least one frame
    float frame_step = 1.0f / out_fps;
    float curr_time = 0;
    int curr_frame = 0;
    
    unsigned int total_frames = (unsigned int)(((float)params.time_end - 0.5f) * out_fps) - 1;
    unsigned int render_steps = (unsigned int)(1 / (render_fps * iteration_step));
    unsigned int step = 0;

    // initialize values that I want to keep track of
    ChVector<float> plane_reaction_force;
    ChVector<float> platePos;
    ChVector<double> cylinder_reaction_force;
    ChVector<double> cylinder_torques;
    int nc;

    
    // let system run for 0.5 second so the particles can settle
    while (curr_time < 1.0) {
        
        char filename[100], filenamemesh[100], filenameforce[100];;
        
        sprintf(filename, "%s/step%06d.csv", out_dir.c_str(), step);
        sprintf(filenamemesh, "%s/mesh%06d.csv", out_dir.c_str(), step);
        sprintf(filenameforce, "%s/forces%06d.csv", out_dir.c_str(), step);

        std::ofstream fcFile(filenameforce, std::ios::out);

        gpu_sys.WriteFile(std::string(filename));
        gpu_sys.WriteMeshes(filenamemesh);

        ChSystemGpuMesh_impl* sys_trimesh = static_cast<ChSystemGpuMesh_impl*>(gpu_sys);

        unsigned int nmeshes = gpu_sys.GetNumMeshes();
        ChVector<> force;  // forces for each mesh
        ChVector<> torque; //torques for each mesh
        
        // Pull forcs
        for (unsigned int imesh = 0; imesh < nmeshes; imesh++) {
            char fforces[100];

            gpu_sys.CollectMeshContactForces(imesh, force, torque);
            force = force * F_CGS_TO_SI;
            sprintf(fforces, "%d, %6f, %6f, %6f \n", imesh, force.x()* F_CGS_TO_SI, force.y()* F_CGS_TO_SI, force.z()* F_CGS_TO_SI);
            fcFile << fforces;

        }

        if (render && step % render_steps == 0) {
            //mixer->SetPos(mesh_pos);
            //mixer->SetRot(mesh_rot);
            if (gpu_vis.Render())
                break;
        }
        gpu_sys.AdvanceSimulation(frame_step);
        curr_time += frame_step;
        printf("time = %.4f\n", curr_time);
        step++;

    }
    return 0;

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
        gpu_sys.WriteFile(std::string(filename));
        gpu_sys.AdvanceSimulation(frame_step);

        platePos = gpu_sys.GetBCPlanePosition(topWall);
        std::cout << "top plate pos_z: " << platePos.z() << " cm";

        nc = gpu_sys.GetNumContacts();
        std::cout << ", numContacts: " << nc;

        gpu_sys.GetBCReactionForces(topWall, plane_reaction_force);
        gpu_sys.CollectMeshContactForces(cylinder_mesh_id, cylinder_reaction_force, cylinder_torques);
        std::cout << ", cylinder force: " << cylinder_reaction_force.Length() * F_CGS_TO_SI << "Newton";
        std::cout << ", top plate force: " << plane_reaction_force.z() * F_CGS_TO_SI << " Newton";
        std::cout << "\n";

        curr_frame++;
        curr_time += frame_step;

    }
    return 0;
}
