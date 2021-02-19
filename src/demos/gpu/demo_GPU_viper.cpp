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
// Chrono::Gpu demo using SMC method. A body whose geometry is described
// by an OBJ file is time-integrated in Chrono and interacts with a granular
// wave tank in Chrono::Gpu via the co-simulation framework.
// =============================================================================

#include <iostream>
#include <vector>
#include <string>
#include <cassert>
#include <cstdlib>
#include <ctime>

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

#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/utils/ChGpuJsonParser.h"
#include "chrono_gpu/utils/ChGpuVisualization.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::gpu;
using namespace collision;
using namespace chrono::geometry;
// Output frequency
int out_fps = 20;

void ShowUsage(std::string name) {
    std::cout << "usage: " + name + " <json_file>" << std::endl;
}

void writeMeshVtk(const std::vector<std::shared_ptr<ChBody>>& body_list, std::vector<string> mesh_filenames, std::string vtk_name, int num_body, double scale_ratio) {
    //auto body_list = mphysicalSystem.Get_bodylist();
    int numVertex[num_body];
    int vertexOffset[num_body] = {0};
    int total_f = 0;
    int total_v = 0;

    printf("Writing meshes\n");
    std::ofstream file;
    file.open(vtk_name+"_mesh.vtk");
    file << "# vtk DataFile Version 2.0" << std::endl;
    file << "VTK from simulation" << std::endl;
    file << "ASCII" << std::endl;
    file << "DATASET UNSTRUCTURED_GRID" << std::endl;

    // Prescan the V and F number info
    for (int i=0; i < num_body; i++) {
        auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        mmesh->LoadWavefrontMesh(mesh_filenames[i], false, true);
        mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));
        mmesh->RepairDuplicateVertexes(1e-9);
        int nv = mmesh->getCoordsVertices().size();
        numVertex[i] = nv;
        total_v += nv;
        int nf = mmesh->getIndicesVertexes().size();
        total_f += nf;
    }
    for (int i=1; i < num_body; i++) {
        vertexOffset[i] = vertexOffset[i-1] + numVertex[i-1];
    }

    file << "POINTS " << total_v << " " << "float" << std::endl;
    for (int i=0; i < num_body; i++) {
        auto body = body_list[i+1];
        ChVector<> body_pos = body->GetFrame_REF_to_abs().GetPos();
        ChQuaternion<> body_rot = body->GetFrame_REF_to_abs().GetRot();

        auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        mmesh->LoadWavefrontMesh(mesh_filenames[i], false, true);
        mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));
        mmesh->RepairDuplicateVertexes(1e-9); 
        mmesh->Transform(body_pos, ChMatrix33<>(body_rot));
        
        for (auto& v : mmesh->getCoordsVertices()) { 
            file << v.x() << " " << v.y() << " " << v.z() << std::endl;
        }
    }

    file << "CELLS " << total_f << " " << 4*total_f << std::endl;
    for (int i=0; i < num_body; i++) {
        auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        mmesh->LoadWavefrontMesh(mesh_filenames[i], false, true);
        mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));
        mmesh->RepairDuplicateVertexes(1e-9); 
    
        for (auto& f : mmesh->getIndicesVertexes()) {
            file <<  "3 " << f.x()+vertexOffset[i]  << " " 
                          << f.y()+vertexOffset[i]  << " " 
                          << f.z()+vertexOffset[i]  << std::endl; 
        }    
    }

    file << "CELL_TYPES " << total_f << std::endl;
    for (int i=0; i < num_body; i++) {
        auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        mmesh->LoadWavefrontMesh(mesh_filenames[i], false, true);
        mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));
        mmesh->RepairDuplicateVertexes(1e-9);
        for (auto& f : mmesh->getIndicesVertexes()) {
            file <<  "5 " << std::endl;
        }
    }
    file.close();

}

int main(int argc, char* argv[]) {
    ChGpuSimulationParameters params;
    if (argc != 2 || ParseJSON(argv[1], params) == false) {
        ShowUsage(argv[0]);
        return 1;
    }
    const int motor_type = 1; 
    const double motor_F = 0.666667;
    float iteration_step = params.step_size;

    ChSystemGpuMesh gpu_sys(params.sphere_radius, params.sphere_density,
                            make_float3(params.box_X, params.box_Y, params.box_Z));

    // dummy sys for mesh output only
    ChSystemGpuMesh dummy_sys(params.sphere_radius, params.sphere_density,
                            make_float3(params.box_X, params.box_Y, params.box_Z));

    double road_bottom = -params.box_Z / 2.0;
    double road_top = -params.box_Z / 2.0 + 20.0;
    double pile_top = road_top + 20.0;

    unsigned int num_points = 0;
    //chrono::utils::PDSampler<float> sampler(2.4f * params.sphere_radius);
    chrono::utils::GridSampler<float> sampler(2.05 * params.sphere_radius);
    std::vector<ChVector<float>> body_points;

    // fill road
    /*
    ChVector<> road_hdims(params.box_X/2.0 - 2.0*params.sphere_radius, params.box_Y/2.0 - 2.0*params.sphere_radius, 0);
    ChVector<> road_center(0, 0, road_bottom + 2.0 * params.sphere_radius);
    while (road_center.z() < road_top) {
        std::cout << "Create layer at " << road_center.z() << std::endl;
        auto points = sampler.SampleBox(road_center, road_hdims);
        body_points.insert(body_points.end(), points.begin(), points.end());
        road_center.z() += 2.05 * params.sphere_radius;
    }
    */
    ChVector<> road_hdims(params.box_X/2.0 - 2.0*params.sphere_radius, params.box_Y/2.0 - 2.0*params.sphere_radius, (road_top - road_bottom)/2.0);
    ChVector<> road_center(0, 0, (road_top + road_bottom)/2.0 + 2.05 * params.sphere_radius);
    {
        auto points = sampler.SampleBox(road_center, road_hdims);
        body_points.insert(body_points.end(), points.begin(), points.end());
        num_points += points.size();
    }

    // fill the pile
    /*
    ChVector<> hdims(100.0 - params.sphere_radius, params.box_Y / 2.0 - params.sphere_radius, 0);
    ChVector<> center(0, 0, road_top + 2.0 * params.sphere_radius);
    center.z() += 3 * params.sphere_radius;
    while (center.z() < pile_top) {
        std::cout << "Create layer at " << center.z() << std::endl;
        auto points = sampler.SampleBox(center, hdims);
        body_points.insert(body_points.end(), points.begin(), points.end());
        center.z() += 2.05 * params.sphere_radius;
    }
    */
    ChVector<> hdims(100.0 - params.sphere_radius, params.box_Y / 2.0 - params.sphere_radius, (pile_top - road_top)/2.0);
    ChVector<> center(0, 0, (pile_top + road_top)/2.0 + 2.05 * params.sphere_radius);
    {
        auto points = sampler.SampleBox(center, hdims);
        body_points.insert(body_points.end(), points.begin(), points.end());
        num_points += points.size();
    }

    std::cout << "Done creating particles, " << num_points << " of them created." << std::endl;
    gpu_sys.SetParticlePositions(body_points);
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
    double scale_ratio = 100.0;    
	double overallPosX = 280.0; double overallPosY = 0.0; double overallPosZ = -params.box_Z / 3.0 + 20.0;//-params.box_Z / 3.0 + 1.0 * params.sphere_radius;
    double settle_time = 1.5;
    ChSystemSMC viper_sys;
    std::vector<string> mesh_filenames;
    std::vector<ChMatrix33<float>> mesh_rotscales;
    std::vector<float> mesh_masses;
    std::vector<float3> mesh_translations;

    // only 4 wheels go into the simulation meshes list
    std::vector<string> simul_mesh_filenames;
    std::vector<ChMatrix33<float>> simul_mesh_rotscales;
    std::vector<float> simul_mesh_masses;
    std::vector<float3> simul_mesh_translations;
    
   	collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
	collision::ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

    auto mfloor = chrono_types::make_shared<ChBodyEasyBox>(params.box_X, params.box_Y, 2.0, 1000, false, true, mysurfmaterial);
    mfloor->SetPos(ChVector<>(0, 0, -params.box_Z/2.0-1.0));
    mfloor->SetBodyFixed(true);
    viper_sys.Add(mfloor); 
    {
        // Reading in viper body
        std::string body_filename(GetChronoDataFile("gpu/demo_GPU_Viper/body.obj"));
        mesh_filenames.push_back(body_filename);
        mesh_translations.push_back(make_float3(0.f, 0.f, 0.f));
        mesh_rotscales.push_back(ChMatrix33<float>(scale_ratio));

        double mvol; ChVector<> body_cog; ChMatrix33<> body_inertia;
        auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        mmesh->LoadWavefrontMesh(body_filename, false, true);
        mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));
        mmesh->RepairDuplicateVertexes(1e-9);                   
        mmesh->ComputeMassProperties(true, mvol, body_cog, body_inertia);  
        ChMatrix33<> principal_inertia_rot; ChVector<> principal_I;
        ChInertiaUtils::PrincipalInertia(body_inertia, principal_I, principal_inertia_rot);      
		double mmass = 300*1000;
        double mdensity = mmass / mvol;
        mesh_masses.push_back(mmass);   		
    
        auto Body = chrono_types::make_shared<ChBodyAuxRef>();
        ChQuaternion<> Body_rot = Q_from_Euler123(ChVector<double>(CH_C_PI/2.0, 0.0, CH_C_PI/2.0));
        ChVector<> Body_pos = ChVector<>(overallPosX, 
                                         overallPosY,
                                         overallPosZ);
        ChVector<> Body_vel = ChVector<>(-0.0, 0.0, 0.0);

		Body->SetFrame_COG_to_REF(ChFrame<>(body_cog, principal_inertia_rot));
		Body->SetMass(mmass);
        Body->SetInertiaXX(mdensity * principal_I);
        Body->SetPos_dt(Body_vel);
        Body->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(Body_pos),ChQuaternion<>(Body_rot))); 
		viper_sys.Add(Body);

        Body->SetBodyFixed(false);
        Body->GetCollisionModel()->ClearModel();
        Body->GetCollisionModel()->AddTriangleMesh(mysurfmaterial, mmesh, false, false, VNULL, ChMatrix33<>(1), 0.005);
        Body->GetCollisionModel()->BuildModel();
        Body->SetCollide(false);

        auto masset_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
        masset_mesh->SetMesh(mmesh);
        masset_mesh->SetBackfaceCull(true);
        Body->AddAsset(masset_mesh);
        // end of reading viper body
   	}

    double w_lx = 0.5618 + 0.08; w_lx *= scale_ratio;
    double w_ly = 0.2067 + 0.32 + 0.0831; w_ly *= scale_ratio;
    double w_lz = 0.0; w_lz *= scale_ratio;
    ChVector<> wheel_rel_pos_lf = ChVector<>(-w_lx, -w_ly, w_lz);
    ChVector<> wheel_rel_pos_rf = ChVector<>(-w_lx,  w_ly, w_lz);
    ChVector<> wheel_rel_pos_lb = ChVector<>( w_lx, -w_ly, w_lz);
    ChVector<> wheel_rel_pos_rb = ChVector<>( w_lx,  w_ly, w_lz);
	{
        // Read in wheels
		for (int i = 0; i < 4; i++) {
			// load mesh from obj file
			auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
			//std::string wheel_filename(GetChronoDataFile("gpu/demo_GPU_Viper/curiosity_wheek_sim.obj"));
            std::string wheel_filename(GetChronoDataFile("gpu/demo_GPU_Viper/2500-repair.obj"));
            //std::string wheel_filename(GetChronoDataFile("gpu/demo_GPU_Viper/wheelSimplified.obj"));
            mesh_filenames.push_back(wheel_filename);
        	mesh_translations.push_back(make_float3(0.f, 0.f, 0.f));
        	mesh_rotscales.push_back(ChMatrix33<float>(scale_ratio));
            // wheels used for simulation
        	simul_mesh_filenames.push_back(wheel_filename);
        	simul_mesh_translations.push_back(make_float3(0.f, 0.f, 0.f));
        	simul_mesh_rotscales.push_back(ChMatrix33<float>(scale_ratio));

			mmesh->LoadWavefrontMesh(wheel_filename, false, true);
			// mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(body_rot));       // rotate the mesh if needed
			mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));     // scale to a different size
			mmesh->RepairDuplicateVertexes(1e-9);                                 // if meshes are not watertight                 

			// compute mass inertia from mesh
			double mvol;
			ChVector<> mcog;
			ChMatrix33<> minertia;
			mmesh->ComputeMassProperties(true, mvol, mcog, minertia);
			ChMatrix33<> principal_inertia_rot;
			ChVector<> principal_I;
			ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);
			double mmass = 20*1000;
            double mdensity = mmass / mvol;
            mesh_masses.push_back(mmass);
            // wheels used for simulation
            simul_mesh_masses.push_back(mmass);

			// set the abs orientation, position and velocity
			auto Body = chrono_types::make_shared<ChBodyAuxRef>();
            ChQuaternion<> Body_rot = Q_from_Euler123(ChVector<double>(0, 0, 0));
			//ChQuaternion<> Body_rot = Q_from_Euler123(ChVector<double>(CH_C_PI/2, 0, 0));
            ChVector<> Body_Rel_pos;
			if(i==0){Body_Rel_pos = wheel_rel_pos_lf;}
			if(i==1){Body_Rel_pos = wheel_rel_pos_rf;}
			if(i==2){Body_Rel_pos = wheel_rel_pos_lb;}
			if(i==3){Body_Rel_pos = wheel_rel_pos_rb;}
			ChVector<> Body_pos = ChVector<>(overallPosX, 
											 overallPosY,
											 overallPosZ) + Body_Rel_pos;
			ChVector<> Body_vel = ChVector<>(0.0, 0.0, 0.0);

			// Set the COG coordinates to barycenter, without displacing the REF reference.
			// Make the COG frame a principal frame.
			Body->SetFrame_COG_to_REF(ChFrame<>(mcog, principal_inertia_rot));

			// Set inertia
			Body->SetMass(mmass);
			Body->SetInertiaXX(mdensity * principal_I);
			// Body->SetPos(Body_pos);
			Body->SetPos_dt(Body_vel);
			// Body->SetRot(QUNIT);
			
			// Set the absolute position of the body:
			Body->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(Body_pos),ChQuaternion<>(Body_rot)));                              
			viper_sys.Add(Body);

			Body->SetBodyFixed(false);
			Body->GetCollisionModel()->ClearModel();
			Body->GetCollisionModel()->AddTriangleMesh(mysurfmaterial, mmesh,false, false, VNULL, ChMatrix33<>(1), 0.005);
			Body->GetCollisionModel()->BuildModel();
			Body->SetCollide(true);

			auto masset_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
			masset_mesh->SetMesh(mmesh);
			masset_mesh->SetBackfaceCull(true);
			Body->AddAsset(masset_mesh);
		}
	} // end of reading wheels

    double cr_lx = 0.5618 + 0.08; cr_lx *= scale_ratio;
	double cr_ly = 0.2067; cr_ly *= scale_ratio;
	double cr_lz = 0.0525; cr_lz *= scale_ratio;
    ChVector<> cr_rel_pos_lf_lower = ChVector<>(-cr_lx, -cr_ly, -cr_lz);
    ChVector<> cr_rel_pos_rf_lower = ChVector<>(-cr_lx,  cr_ly, -cr_lz);
    ChVector<> cr_rel_pos_lb_lower = ChVector<>( cr_lx, -cr_ly, -cr_lz);
    ChVector<> cr_rel_pos_rb_lower = ChVector<>( cr_lx,  cr_ly, -cr_lz);
    ChVector<> cr_rel_pos_lf_upper = ChVector<>(-cr_lx, -cr_ly,  cr_lz);
    ChVector<> cr_rel_pos_rf_upper = ChVector<>(-cr_lx,  cr_ly,  cr_lz);
    ChVector<> cr_rel_pos_lb_upper = ChVector<>( cr_lx, -cr_ly,  cr_lz);
    ChVector<> cr_rel_pos_rb_upper = ChVector<>( cr_lx,  cr_ly,  cr_lz);
    { // Read in conecting rods
		for (int i = 0; i < 8; i++) {
			// load mesh from obj file
			auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        	mesh_translations.push_back(make_float3(0.f, 0.f, 0.f));
        	mesh_rotscales.push_back(ChMatrix33<float>(scale_ratio));
			
			if(i < 4){
				std::string rod_filename(GetChronoDataFile("gpu/demo_GPU_Viper/lowerRod.obj"));
			    mmesh->LoadWavefrontMesh(rod_filename, false, true);
            	mesh_filenames.push_back(rod_filename);
			}
			else{
				std::string rod_filename(GetChronoDataFile("gpu/demo_GPU_Viper/upperRod.obj"));
			    mmesh->LoadWavefrontMesh(rod_filename, false, true);
            	mesh_filenames.push_back(rod_filename);
			}
			// mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(body_rot));       // rotate the mesh if needed
			mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));     // scale to a different size
			mmesh->RepairDuplicateVertexes(1e-9);                                 // if meshes are not watertight                 

			// compute mass inertia from mesh
			double mvol;
			ChVector<> mcog;
			ChMatrix33<> minertia;
			mmesh->ComputeMassProperties(true, mvol, mcog, minertia);
			ChMatrix33<> principal_inertia_rot;
			ChVector<> principal_I;
			ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);
			double mmass = 5*1000;
            double mdensity = mmass / mvol;
            mesh_masses.push_back(mmass);

			// set the abs orientation, position and velocity
			auto Body = chrono_types::make_shared<ChBodyAuxRef>();
			ChQuaternion<> Body_rot;
			ChVector<> Body_Rel_pos;
			if(i==0){Body_Rel_pos = cr_rel_pos_lf_lower; Body_rot = Q_from_Euler123(ChVector<double>( CH_C_PI/2, 0, 0));}
			if(i==1){Body_Rel_pos = cr_rel_pos_rf_lower; Body_rot = Q_from_Euler123(ChVector<double>(-CH_C_PI/2, 0, 0));}
			if(i==2){Body_Rel_pos = cr_rel_pos_lb_lower; Body_rot = Q_from_Euler123(ChVector<double>( CH_C_PI/2, 0, 0));}
			if(i==3){Body_Rel_pos = cr_rel_pos_rb_lower; Body_rot = Q_from_Euler123(ChVector<double>(-CH_C_PI/2, 0, 0));}
			if(i==4){Body_Rel_pos = cr_rel_pos_lf_upper; Body_rot = Q_from_Euler123(ChVector<double>( CH_C_PI/2, 0, 0));}
			if(i==5){Body_Rel_pos = cr_rel_pos_rf_upper; Body_rot = Q_from_Euler123(ChVector<double>(-CH_C_PI/2, 0, 0));}
			if(i==6){Body_Rel_pos = cr_rel_pos_lb_upper; Body_rot = Q_from_Euler123(ChVector<double>( CH_C_PI/2, 0, 0));}
			if(i==7){Body_Rel_pos = cr_rel_pos_rb_upper; Body_rot = Q_from_Euler123(ChVector<double>(-CH_C_PI/2, 0, 0));}
			ChVector<> Body_pos = ChVector<>(overallPosX, 
											 overallPosY,
											 overallPosZ) + Body_Rel_pos;
			ChVector<> Body_vel = ChVector<>(0.0, 0.0, 0.0);

			// Set the COG coordinates to barycenter, without displacing the REF reference.
			// Make the COG frame a principal frame.
			Body->SetFrame_COG_to_REF(ChFrame<>(mcog, principal_inertia_rot));

			// Set inertia
			Body->SetMass(mmass);
			Body->SetInertiaXX(mdensity * principal_I);
			// Body->SetPos(Body_pos);
			Body->SetPos_dt(Body_vel);
			// Body->SetRot(QUNIT);
			
			// Set the absolute position of the body:
			Body->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(Body_pos),ChQuaternion<>(Body_rot)));                              
			viper_sys.Add(Body);

			Body->SetBodyFixed(false);
			Body->GetCollisionModel()->ClearModel();
			Body->GetCollisionModel()->AddTriangleMesh(mysurfmaterial, mmesh,false, false, VNULL, ChMatrix33<>(1), 0.005);
			Body->GetCollisionModel()->BuildModel();
			Body->SetCollide(false);

			auto masset_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
			masset_mesh->SetMesh(mmesh);
			masset_mesh->SetBackfaceCull(true);
			Body->AddAsset(masset_mesh);
		}

    } // end of read in connecting rods


	double sr_lx = 0.5618 + 0.08; sr_lx *= scale_ratio;
	double sr_ly = 0.2067 + 0.32 + 0.0831; sr_ly *= scale_ratio;
	double sr_lz = 0.0; sr_lz *= scale_ratio;
    ChVector<> sr_rel_pos_lf = ChVector<>(-sr_lx, -sr_ly, sr_lz);
    ChVector<> sr_rel_pos_rf = ChVector<>(-sr_lx,  sr_ly, sr_lz);
    ChVector<> sr_rel_pos_lb = ChVector<>( sr_lx, -sr_ly, sr_lz);
    ChVector<> sr_rel_pos_rb = ChVector<>( sr_lx,  sr_ly, sr_lz);
    ChVector<> sr_rel_pos_lf_lower = ChVector<>(-sr_lx, -sr_ly, -cr_lz);
    ChVector<> sr_rel_pos_rf_lower = ChVector<>(-sr_lx,  sr_ly, -cr_lz);
    ChVector<> sr_rel_pos_lb_lower = ChVector<>( sr_lx, -sr_ly, -cr_lz);
    ChVector<> sr_rel_pos_rb_lower = ChVector<>( sr_lx,  sr_ly, -cr_lz);
    ChVector<> sr_rel_pos_lf_upper = ChVector<>(-sr_lx, -sr_ly,  cr_lz);
    ChVector<> sr_rel_pos_rf_upper = ChVector<>(-sr_lx,  sr_ly,  cr_lz);
    ChVector<> sr_rel_pos_lb_upper = ChVector<>( sr_lx, -sr_ly,  cr_lz);
    ChVector<> sr_rel_pos_rb_upper = ChVector<>( sr_lx,  sr_ly,  cr_lz);
    {  // read in steering rod on wheels
		for (int i = 0; i < 4; i++) {
			// load mesh from obj file
			auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
			std::string rod_filename(GetChronoDataFile("gpu/demo_GPU_Viper/steeringRod.obj"));
        	mesh_filenames.push_back(rod_filename);
        	mesh_translations.push_back(make_float3(0.f, 0.f, 0.f));
        	mesh_rotscales.push_back(ChMatrix33<float>(scale_ratio));

			mmesh->LoadWavefrontMesh(rod_filename, false, true);
			// mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(body_rot));       // rotate the mesh if needed
			mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));     // scale to a different size
			mmesh->RepairDuplicateVertexes(1e-9);                                 // if meshes are not watertight                 

			// compute mass inertia from mesh
			double mvol;
			ChVector<> mcog;
			ChMatrix33<> minertia;
			mmesh->ComputeMassProperties(true, mvol, mcog, minertia);
			ChMatrix33<> principal_inertia_rot;
			ChVector<> principal_I;
			ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);
            double mmass = 5*1000;
            double mdensity = mmass / mvol;
            mesh_masses.push_back(mmass);

			// set the abs orientation, position and velocity
			auto Body = chrono_types::make_shared<ChBodyAuxRef>();
			ChQuaternion<> Body_rot;
			ChVector<> Body_Rel_pos;
			if(i==0){Body_Rel_pos = sr_rel_pos_lf; Body_rot = Q_from_Euler123(ChVector<double>(CH_C_PI/2, 0, 0)); }
			if(i==1){Body_Rel_pos = sr_rel_pos_rf; Body_rot = Q_from_Euler123(ChVector<double>(CH_C_PI/2, 0, CH_C_PI)); }
			if(i==2){Body_Rel_pos = sr_rel_pos_lb; Body_rot = Q_from_Euler123(ChVector<double>(CH_C_PI/2, 0, 0)); }
			if(i==3){Body_Rel_pos = sr_rel_pos_rb; Body_rot = Q_from_Euler123(ChVector<double>(CH_C_PI/2, 0, CH_C_PI)); }
			ChVector<> Body_pos = ChVector<>(overallPosX, 
											 overallPosY,
											 overallPosZ) + Body_Rel_pos;
			ChVector<> Body_vel = ChVector<>(0.0, 0.0, 0.0);

			// Set the COG coordinates to barycenter, without displacing the REF reference.
			// Make the COG frame a principal frame.
			Body->SetFrame_COG_to_REF(ChFrame<>(mcog, principal_inertia_rot));

			// Set inertia
			Body->SetMass(mmass);
			Body->SetInertiaXX(mdensity * principal_I);
			// Body->SetPos(Body_pos);
			Body->SetPos_dt(Body_vel);
			// Body->SetRot(QUNIT);
			
			// Set the absolute position of the body:
			Body->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(Body_pos),ChQuaternion<>(Body_rot)));                              
			viper_sys.Add(Body);

			Body->SetBodyFixed(false);
			Body->GetCollisionModel()->ClearModel();
			Body->GetCollisionModel()->AddTriangleMesh(mysurfmaterial, mmesh,false, false, VNULL, ChMatrix33<>(1), 0.005);
			Body->GetCollisionModel()->BuildModel();
			Body->SetCollide(false);

			auto masset_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
			masset_mesh->SetMesh(mmesh);
			masset_mesh->SetBackfaceCull(true);
			Body->AddAsset(masset_mesh);
		}

    } // end of read in steering rods

   	
	/// Create joint constraints.
    for (int i = 0; i < 4; i++) {
        // define a rotation of 90 degrees around y (z2x)
        // because all revolute jonts are along Z direction of the joint frame,
        // so we apply the 'z2x' rotation to align the rotation axis with the X axis of the global frame.
        ChQuaternion<> z2x;
        z2x.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(0, 1, 0));

        // pick up relative position of wheel and conecting rod to the body
        ChVector<> Wheel_Rel_pos;
        ChVector<> CR_Rel_pos_Lower;
        ChVector<> CR_Rel_pos_Upper;
        ChVector<> SR_Rel_pos;
        ChVector<> SR_Rel_pos_Lower;
        ChVector<> SR_Rel_pos_Upper;
        if(i==0){
            Wheel_Rel_pos = wheel_rel_pos_lf; 
            CR_Rel_pos_Lower = cr_rel_pos_lf_lower;
            CR_Rel_pos_Upper = cr_rel_pos_lf_upper;
            SR_Rel_pos = sr_rel_pos_lf;
            SR_Rel_pos_Lower = sr_rel_pos_lf_lower;
            SR_Rel_pos_Upper = sr_rel_pos_lf_upper;
        }
        if(i==1){
            Wheel_Rel_pos = wheel_rel_pos_rf; 
            CR_Rel_pos_Lower = cr_rel_pos_rf_lower;
            CR_Rel_pos_Upper = cr_rel_pos_rf_upper;
            SR_Rel_pos = sr_rel_pos_rf;
            SR_Rel_pos_Lower = sr_rel_pos_rf_lower;
            SR_Rel_pos_Upper = sr_rel_pos_rf_upper;
        }
        if(i==2){
            Wheel_Rel_pos = wheel_rel_pos_lb; 
            CR_Rel_pos_Lower = cr_rel_pos_lb_lower;
            CR_Rel_pos_Upper = cr_rel_pos_lb_upper;
            SR_Rel_pos = sr_rel_pos_lb;
            SR_Rel_pos_Lower = sr_rel_pos_lb_lower;
            SR_Rel_pos_Upper = sr_rel_pos_lb_upper;
        }
        if(i==3){
            Wheel_Rel_pos = wheel_rel_pos_rb; 
            CR_Rel_pos_Lower = cr_rel_pos_rb_lower;
            CR_Rel_pos_Upper = cr_rel_pos_rb_upper;
            SR_Rel_pos = sr_rel_pos_rb;
            SR_Rel_pos_Lower = sr_rel_pos_rb_lower;
            SR_Rel_pos_Upper = sr_rel_pos_rb_upper;
        }

        // pick up bodies and create links
        auto ground = viper_sys.Get_bodylist()[0];
        auto body = viper_sys.Get_bodylist()[1];
        auto wheel = viper_sys.Get_bodylist()[i+2];
        auto cr_lower = viper_sys.Get_bodylist()[i+2+4];
        auto cr_upper = viper_sys.Get_bodylist()[i+2+8];
        auto steering_rod = viper_sys.Get_bodylist()[i+2+12];
        ChVector<> Link_pos;
        ChVector<> Rover_Body_pos = ChVector<>(overallPosX, overallPosY, overallPosZ);

        // body-cr_lower Revolute constraint
        auto revo_body_cr_lower = chrono_types::make_shared<ChLinkLockRevolute>(); 
        Link_pos = Rover_Body_pos + CR_Rel_pos_Lower;
        revo_body_cr_lower->Initialize(body, cr_lower, ChCoordsys<>(Link_pos, z2x));
        viper_sys.AddLink(revo_body_cr_lower);

        // body-cr_upper Revolute constraint
        auto revo_body_cr_upper = chrono_types::make_shared<ChLinkLockRevolute>(); 
        Link_pos = Rover_Body_pos + CR_Rel_pos_Upper;
        revo_body_cr_upper->Initialize(body, cr_upper, ChCoordsys<>(Link_pos, z2x));
        viper_sys.AddLink(revo_body_cr_upper);

        // cr_lower-sr_lower Revolute constraint
        auto revo_cr_sr_lower = chrono_types::make_shared<ChLinkLockRevolute>(); 
        Link_pos = Rover_Body_pos + SR_Rel_pos_Lower;
        revo_cr_sr_lower->Initialize(cr_lower, steering_rod, ChCoordsys<>(Link_pos, z2x));
        viper_sys.AddLink(revo_cr_sr_lower);

        // cr_upper-sr_upper Revolute constraint
        auto revo_cr_sr_upper = chrono_types::make_shared<ChLinkLockRevolute>(); 
        Link_pos = Rover_Body_pos + SR_Rel_pos_Upper;
        revo_cr_sr_upper->Initialize(cr_upper, steering_rod, ChCoordsys<>(Link_pos, z2x));
        viper_sys.AddLink(revo_cr_sr_upper);

        // define a rotation of -90 degrees around x (z2y)
        // because all revolute jonts are along Z direction of the joint frame,
        // so we apply the 'z2y' rotation to align the rotation axis with the Y axis of the global frame.
        ChQuaternion<> z2y;
        z2y.Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));
        if(motor_type == 1){
            // sr-wheel Revolute constraint with a motor - Rotation Speed
            auto revo_sr_wheel = chrono_types::make_shared<ChLinkMotorRotationSpeed>(); 
            Link_pos = Rover_Body_pos + Wheel_Rel_pos;
            revo_sr_wheel->Initialize(steering_rod, wheel, ChFrame<>(Link_pos, z2y));
            viper_sys.AddLink(revo_sr_wheel);
            auto my_speed_function = chrono_types::make_shared<ChFunction_Const>(CH_C_PI*motor_F);  // speed w=3.145 rad/sec
            revo_sr_wheel->SetSpeedFunction(my_speed_function);
        }
        else if(motor_type == 2){
            // sr-wheel Revolute constraint with a motor - torque
            std::shared_ptr<ChLinkMotorRotationTorque> link_motor;
            link_motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
            Link_pos = Rover_Body_pos + Wheel_Rel_pos;
            link_motor->Initialize(steering_rod, wheel, ChFrame<>(Link_pos, z2y));
            viper_sys.AddLink(link_motor);
            auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(link_motor->GetTorqueFunction());
            mfun->Set_yconst(motor_F);
        }

        // body-steering_rod spring constraint
        std::shared_ptr<ChLinkTSDA> spring;
        ChVector<> pos1 = Rover_Body_pos + CR_Rel_pos_Upper;
        ChVector<> pos2 = Rover_Body_pos + SR_Rel_pos_Lower;
        spring = chrono_types::make_shared<ChLinkTSDA>();
        spring->Initialize(body, steering_rod, false, pos1, pos2, true, 0.0);
        spring->SetSpringCoefficient(400000.0*scale_ratio*scale_ratio);
        spring->SetDampingCoefficient(10000.0*scale_ratio*scale_ratio);
        viper_sys.AddLink(spring);

	} // end of adding constraints

	gpu_sys.SetOutputMode(params.write_mode);
	gpu_sys.SetVerbosity(params.verbose);
	filesystem::create_directory(filesystem::path(params.output_dir));

	gpu_sys.LoadMeshes(simul_mesh_filenames, simul_mesh_rotscales, simul_mesh_translations, simul_mesh_masses);
    gpu_sys.EnableMeshCollision(false);
    std::cout << "Initializing the GPU system!" << std::endl;
    gpu_sys.Initialize();

    // dummy sys reads all meshes
    std::cout << "Initializing the dummy system!" << std::endl;
    std::vector<ChVector<float>> dummy_points;
    ChVector<float> a_point(0,0,0);
    dummy_points.push_back(a_point);
    dummy_sys.SetParticlePositions(dummy_points);
    dummy_sys.SetOutputMode(params.write_mode);
    dummy_sys.SetVerbosity(params.verbose);
	dummy_sys.LoadMeshes(mesh_filenames, mesh_rotscales, mesh_translations, mesh_masses);
    dummy_sys.EnableMeshCollision(false);
    dummy_sys.SetBDFixed(true);
	dummy_sys.Initialize();

    // Create rigid ball_body simulation
    viper_sys.SetContactForceModel(ChSystemSMC::ContactForceModel::Hooke);
    viper_sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
    viper_sys.Set_G_acc(ChVector<>(0, 0, -980));

    std::cout << "Output at    " << out_fps << " FPS" << std::endl;
    unsigned int out_steps = (unsigned int)(1.0 / (out_fps * iteration_step));
    unsigned int total_frames = (unsigned int)(params.time_end * out_fps);

    int currframe = 0;
    unsigned int curr_step = 0;
    bool viper_enabled = false;

    clock_t start = std::clock();
    for (double t = 0; t < (double)params.time_end; t += iteration_step, curr_step++) {
        if (!viper_enabled && t>settle_time){
            gpu_sys.EnableMeshCollision(true);
            viper_enabled = true;
        }
        auto Body = viper_sys.Get_bodylist();
        int moffset = 2;
        // 4 wheels, to exclude body (number 0)
        for (int i=0; i<4; i++) { // Ground (0) and body (1), not interested
            gpu_sys.ApplyMeshMotion(i, Body[i+moffset]->GetFrame_REF_to_abs().GetPos(), Body[i+moffset]->GetFrame_REF_to_abs().GetRot(),
                                    Body[i+moffset]->GetFrame_REF_to_abs().GetPos_dt(), Body[i+moffset]->GetFrame_REF_to_abs().GetWvel_par());

            ChVector<> Body_force;
            ChVector<> Body_torque;
            gpu_sys.CollectMeshContactForces(i, Body_force, Body_torque);

            Body[i+moffset]->Empty_forces_accumulators();
            Body[i+moffset]->Accumulate_force(Body_force, Body[i+moffset]->GetFrame_REF_to_abs().GetPos(), false);
            Body[i+moffset]->Accumulate_torque(Body_torque, false);
        }
        
        /*
        // all bodies, for mesh output only
        for (int i=0; i<17; i++) {
            dummy_sys.ApplyMeshMotion(i, Body[i+1]->GetFrame_REF_to_abs().GetPos(), Body[i+1]->GetFrame_REF_to_abs().GetRot(),
                                    Body[i+1]->GetFrame_REF_to_abs().GetPos_dt(), Body[i+1]->GetFrame_REF_to_abs().GetWvel_par());
            //ChVector<> Body_force;
            //ChVector<> Body_torque;
            //gpu_sys.CollectMeshContactForces(i, Body_force, Body_torque);
            //Body[i]->Empty_forces_accumulators();
            //Body[i]->Accumulate_force(Body_force, Body[i]->GetFrame_REF_to_abs().GetPos(), false);
            //Body[i]->Accumulate_torque(Body_torque, false);
        }
        */

        if (curr_step % out_steps == 0) {
            std::cout << "Output frame " << currframe + 1 << " of " << total_frames << std::endl;
            char filename[100];
            sprintf(filename, "%s/step%06d", params.output_dir.c_str(), currframe++);
            gpu_sys.WriteFile(std::string(filename));
            //dummy_sys.WriteMeshes(std::string(filename));
            writeMeshVtk(Body, mesh_filenames, std::string(filename), 17, scale_ratio);

            /*  // disable meshframes output, for it may be confusing for users dealing with Chrono::Gpu only
            std::string mesh_output = std::string(filename) + "_meshframes.csv";
            std::ofstream meshfile(mesh_output);
            std::ostringstream outstream;
            outstream << "mesh_name,dx,dy,dz,x1,x2,x3,y1,y2,y3,z1,z2,z3,sx,sy,sz\n";
            writeMeshFrames(outstream, *ball_body, mesh_filename, ball_radius);
            meshfile << outstream.str();
            */
        }

        gpu_sys.AdvanceSimulation(iteration_step);
        if (viper_enabled) viper_sys.DoStepDynamics(iteration_step);
    }

    clock_t end = std::clock();
    double total_time = ((double)(end - start)) / CLOCKS_PER_SEC;

    std::cout << "Time: " << total_time << " seconds" << std::endl;

    return 0;
}
