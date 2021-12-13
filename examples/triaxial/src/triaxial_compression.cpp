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

    // ===============================================
    // 1. Read json paramater files
    // 2. Create ChSystemGpuMesh object
    // 3. Set simulation parameters
    // ===============================================

    ChGpuSimulationParameters params;

    if (argc != 2 || ParseJSON( argv[1], params) == false) {
        std::cout << "Usage:\n./demo_triaxial.json <json_file>" << std::endl;
        return 1;
    }

    const float Bx = params.box_X;
    const float By = Bx;

    const float chamber_height = Bx / 3;  // TODO
    ChVector<float> cyl_center(0, 0, 0);
    const float cyl_rad = Bx / 5.f;

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

    std::string out_dir = "./";
    filesystem::create_directory(filesystem::path(out_dir));
    out_dir = out_dir + params.output_dir;
    filesystem::create_directory(filesystem::path(out_dir));

    gpu_sys.SetTimeIntegrator(CHGPU_TIME_INTEGRATOR::CENTERED_DIFFERENCE);
    gpu_sys.SetFixedStepSize(params.step_size);
    gpu_sys.SetBDFixed(true);

    // ================================================
    // Read and add the mesh to the simulation
    // ================================================

    const float chamber_bottom = -Bz / 2.f;
    const float fill_bottom = chamber_bottom + chamber_height;

    // gpu_sys.CreateBCCylinderZ(cyl_center, cyl_rad, false, false); // remove boundary condition cylinder

    std::vector<string> mesh_filenames;
    mesh_filenames.push_back(GetChronoDataFile(params.mesh_model));

    std::vector<ChMatrix33<float>> mesh_rotscales;
    std::vector<float3> mesh_translations;

    float scale_xy = cyl_rad;
    float scale_z = chamber_height;  // TODO fix this / make switch on mixer_type
    float3 scaling = make_float3(scale_xy, scale_xy, scale_z);
    mesh_rotscales.push_back(ChMatrix33<float>(ChVector<float>(scaling.x, scaling.y, scaling.z)));
    mesh_translations.push_back(make_float3(0, 0, 0));

    std::vector<float> mesh_masses;
    float mixer_mass = 10;
    mesh_masses.push_back(mixer_mass);

    gpu_sys.LoadMeshes(mesh_filenames, mesh_rotscales, mesh_translations, mesh_masses);

    std::cout << gpu_sys.GetNumMeshes() << " meshes" << std::endl;

    float ang_vel_Z = 0; //TODO: remove
    ChVector<> mesh_lin_vel(0); //TODO: remove
    ChVector<> mesh_ang_vel(0, 0, ang_vel_Z); //TODO: remove

    gpu_sys.EnableMeshCollision(true);    
    
    // ======================================================
    // Add the particles to the sim
    // ======================================================    
    
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

    gpu_sys.Initialize();

    // ===================================================
    //
    // Prepare main loop parameters
    //
    // ===================================================
    
    unsigned int out_steps = (unsigned int)(1.0f / (out_fps * iteration_step));
    unsigned int render_steps = (unsigned int)(1.0 / (render_fps * iteration_step));
    unsigned int total_frames = (unsigned int)(params.time_end * out_fps);
    std::cout << "out_steps " << out_steps << std::endl;

    unsigned int currframe = 0;
    unsigned int step = 0;

    // initialize values that we need to keep track of
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
