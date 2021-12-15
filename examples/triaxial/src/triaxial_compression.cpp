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
#include "chrono/core/ChQuaternion.h"     
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
float out_fps = 100;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
float render_fps = 2000;

// unit conversion from cgs to si
float F_CGS_TO_SI = 1e-5f;
float KE_CGS_TO_SI = 1e-7f;
float L_CGS_TO_SI = 1e-2f;

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
    const float Bz = params.box_Z;

    std::cout << "Box Dims: " << Bx << " " << By << " " << Bz << std::endl;

    float iteration_step = params.step_size;

    ChSystemGpuMesh gpu_sys(params.sphere_radius, 
                            params.sphere_density, 
                            make_float3(Bx, By, Bz), 
                            make_float3((float)0., (float)0., (float)0.));
    
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
    //
    // Read and add the mesh to the simulation
    //
    // ================================================
    ChVector<float> cyl_center(0.0f, 0.0f, 0.0f);
    float cyl_rad = Bx / 2.f;  //std::min(params.box_X, params.box_Y) / 2.0f; //TODO: fix these
    float cyl_hgt = Bz / 1.5f; //params.box_Z / 1.5f; //TODO: fix these
    
    float scale_xy = 2.f*cyl_rad;
    float scale_z = cyl_hgt; 
    float3 scaling = make_float3(scale_xy, scale_xy, scale_z);
    std::vector<float> mesh_masses;
    float mixer_mass = 10;

    std::vector<string> mesh_filenames;
    std::vector<ChMatrix33<float>> mesh_rotscales;
    ChMatrix33<float> mesh_scale(ChVector<float>(scaling.x, scaling.y, scaling.z));
    std::vector<float3> mesh_translations;
    
    for (int i=0; i<120; ++i){
        mesh_filenames.push_back("./models/open_unit_cylinder_slab_120.obj"); // add slice
        ChQuaternion<> quat = Q_from_AngAxis(i*3.f * CH_C_DEG_TO_RAD, VECT_Z); // find quaternion for rotation
        mesh_rotscales.push_back(mesh_scale * ChMatrix33<float>(quat)); // create rotation * scaling matrix and push to vector
        mesh_translations.push_back(make_float3(cyl_center.x(), cyl_center.y(), cyl_center.z())); // push translation
        mesh_masses.push_back(mixer_mass); // push mass
    }

    gpu_sys.LoadMeshes(mesh_filenames, mesh_rotscales, mesh_translations, mesh_masses);
    std::cout << gpu_sys.GetNumMeshes() << " meshes" << std::endl;

    float ang_vel_Z = 0; //TODO: remove
    ChVector<> mesh_lin_vel(0); //TODO: remove
    ChVector<> mesh_ang_vel(0, 0, ang_vel_Z); //TODO: remove
    
    // ======================================================
    //
    // Add the particles to the sim
    //
    // ======================================================    

    // initialize sampler, set distance between center of spheres as 2.1r
    utils::PDSampler<float> sampler(2.1f * params.sphere_radius);
    std::vector<ChVector<float>> initialPos;

    // randomize by layer
    ChVector<float> center(0.0f, 0.0f, 0.0f);
    // fill up each layer
    while (center.z() + params.sphere_radius < cyl_hgt / 2.0f ) {
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

    gpu_sys.SetParticlePositions(initialPos, initialVelo);
    gpu_sys.SetGravitationalAcceleration(ChVector<float>(0, 0, -980));

    // ===================================================
    //
    // Initialize
    //
    // ====================================================

    gpu_sys.EnableMeshCollision(true);    
    gpu_sys.Initialize();
    unsigned int nummeshes = gpu_sys.GetNumMeshes();
    std::cout << nummeshes << " meshes generated!" << std::endl;
    std::cout << "Created " << initialPos.size() << " spheres" << std::endl;
    
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

    float curr_time = 0;

    // initialize values that we need to keep track of
    ChVector<float> plane_reaction_force;
    ChVector<float> platePos;
    ChVector<double> cylinder_reaction_force;
    ChVector<double> cylinder_torques;
    int nc;
    
    char filenamesumforces[100];
    sprintf(filenamesumforces, "%s/sumforces.csv", out_dir.c_str());
    std::ofstream sumfrcsFile(filenamesumforces, std::ios::out);

    // let system run for 0.5 second so the particles can settle
    while ( curr_time < 0.5 ) {
        
        if (step % out_steps == 0){

            char filename[100], filenamemesh[100], filenameforce[100];;
            sprintf(filename, "%s/step%06d", out_dir.c_str(), step);
            sprintf(filenamemesh, "%s/main%06d", out_dir.c_str(), step);

            gpu_sys.WriteFile(std::string(filename));
            gpu_sys.WriteMeshes(filenamemesh);

            // Get sum of forces on cylinder
            unsigned int nmeshes = gpu_sys.GetNumMeshes(); // only 1 mesh 
            ChVector<> force;  // forces for each mesh
            ChVector<> torque; //torques for each mesh
            gpu_sys.CollectMeshContactForces(0, force, torque);
            force = force * F_CGS_TO_SI;
            char fforces[100];
            sprintf(fforces, "%d, %6f, %6f, %6f \n", step, force.x(), force.y(), force.z());            
            sumfrcsFile << fforces;

            // Pull individual mesh forces
            
            //for (unsigned int imesh = 0; imesh < nmeshes; imesh++) {
            //    char fforces[100];

            //    gpu_sys.CollectMeshContactForces(imesh, force, torque);
            //    force = force * F_CGS_TO_SI;
            //    sprintf(fforces, "%d, %6f, %6f, %6f \n", imesh, force.x()* F_CGS_TO_SI, force.y()* F_CGS_TO_SI, force.z()* F_CGS_TO_SI);
            //    fcFile << fforces;
            //}

            printf("time = %.4f\n", curr_time);
        }

        gpu_sys.AdvanceSimulation(iteration_step);
        curr_time += iteration_step;
        step++;

    }
    return 0;
 /* 
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
    */
}
