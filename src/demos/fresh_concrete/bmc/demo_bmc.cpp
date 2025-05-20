// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: BAhar Ayhan
// =============================================================================
//
//   Demo code about
//     - simulating Building Material Cell (BMC) test for fresh concrete
//     - 1st loop : waiting  (0.5sec) 
//     - 2nd loop : Rotating the rod (60sec) 
// =============================================================================

#include "chrono/fresh_concrete/FreshConcreteContact.cpp"

#include "chrono/particlefactory/ChParticleRemover.h"

#include "chrono/collision/ChCollisionSystem.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/filesystem/path.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotionImposed.h"
#include <chrono/physics/ChLinkMate.h>
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/functions/ChFunctionPositionXYZFunctions.h"
#include "chrono/functions/ChFunctionRotationAxis.h"

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChTexture.h"

#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/geometry/ChBox.h"


// Use the main namespace of Chrono, and other chrono namespaces
using namespace chrono;
using namespace chrono::particlefactory;
using namespace chrono::irrlicht;

//
double density_steel=7.8E-9;
//
// Parameters of the bmcRod and the container
std::string bmcRod_obj = "bmcRod.obj";
std::string bmcContainer_obj = "bmcContainer.obj";
// Initial Position and Velocity of the bmcRod and the container
ChVector3d bmcRod_IniPos(0.0, 0.16, 0.05);
ChVector3d bmcRod_IniVel(0.0, 0.0, 0.0);

/////////////////////////////////////////////////////////////////////////////////////////////////////
double calculateKE(ChSystem& sys){
	double KE=0;
	for (auto body:sys.GetBodies()){
		if (body->IsFixed() || body->GetCollisionModel()->GetShapeInstance(0).first->GetType()!=0 )
			continue;
		ChVector3d velo=body->GetFrameCOMToAbs().GetPosDt();
		ChVector3d Wvelo=body->GetFrameCOMToAbs().GetAngVelLocal();
		double velMag=velo.Length();
		double WvelMag=Wvelo.Length();
		KE+=0.5*body->GetMass()*velMag*velMag + 0.5* body->GetInertiaXX()[0]*WvelMag*WvelMag;		
	}
	
	return KE;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<ChBodyAuxRef> CreateBMCrod(ChSystem& sysMBS, std::shared_ptr<ChVisualSystemIrrlicht>& vis, std::shared_ptr<ChBody> ground, std::string current_dir) {
    std::string bmcRod_obj = (current_dir+"/bmcRod.obj").c_str();
    
    // Common contact material
    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial->SetYoungModulus(1e2);
    cmaterial->SetFriction(0.5f);
    cmaterial->SetRestitution(0.0f);
    cmaterial->SetAdhesion(0);
    
    // Create the BMC rod
    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    double scale_ratio = 1;//0.05555556;
    trimesh->LoadWavefrontMesh(bmcRod_obj, false, true);
    trimesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

    // Compute mass inertia from mesh
    double mmass;
    double mdensity = density_steel;
    ChVector3d mcog;
    ChMatrix33<> minertia;
    trimesh->ComputeMassProperties(true, mmass, mcog, minertia);
    ChMatrix33<> principal_inertia_rot;
    ChVector3d principal_I;
    ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);
    mcog = ChVector3d(0.0, 0.0, 0.0);
    std::cout << " minertia " << minertia << " p iner rot " << principal_inertia_rot << std::endl;  
    // Set the abs orientation, position and velocity
    auto bmcRod = chrono_types::make_shared<ChBodyAuxRef>();
    ChQuaternion<> bmcRod_Rot = QUNIT;

    // Set the COG coordinates to barycenter, without displacing the REF reference.
    // Make the COG frame a principal frame.
    bmcRod->SetFrameCOMToRef(ChFrame<>(mcog, principal_inertia_rot));

    // Set inertia
    bmcRod->SetPosDt(bmcRod_IniVel);
    bmcRod->SetAngVelLocal(ChVector3d(0.0, 0.0, 0.0));  // set an initial angular velocity (rad/s)

    // Set the absolute position of the body:
    bmcRod->SetFrameRefToAbs(ChFrame<>(ChVector3d(bmcRod_IniPos), ChQuaternion<>(bmcRod_Rot)));
    sysMBS.AddBody(bmcRod);
    bmcRod->SetFixed(false);
	
	auto bmcRod_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(cmaterial, trimesh, false, false, 0.0002);
    bmcRod->AddCollisionShape(bmcRod_shape);
    bmcRod->EnableCollision(false);
	
	//auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(chassismesh, true, true);
    auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetMutable(false);
    trimesh_shape->SetColor(ChColor(0.2f, 0.32f, 0.48f));
    bmcRod->AddVisualShape(trimesh_shape, bmcRod->GetFrameRefToAbs());
	
	vis->BindItem(bmcRod);
    
    //re-assign mass & inertia
    bmcRod->SetMass(mmass*density_steel);
    bmcRod->SetInertia(principal_I*density_steel);
    std::cout << " principal_I : " << bmcRod->GetInertia() << " mass : " << bmcRod->GetMass() << std::endl;
       
    return bmcRod;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<ChBody> AddBMCContainer(ChSystem& sys, std::shared_ptr<ChContactMaterial> mat, double density, std::string current_dir) {
    // create container 
   
	auto container = chrono_types::make_shared<ChBodyEasyMesh>(
        (current_dir+"/bmcContainer.obj").c_str(),  // file name for OBJ Wavefront mesh
        density,                                         // density of the body
        true,                                         // automatic evaluation of mass, COG position, inertia tensor
        true,                                         // attach visualization asset
        true,                                         // enable the collision detection
        mat,                                     // surface contact material
        1.0  // radius of 'inflating' of mesh (for more robust collision detection)
    );	
	//	
	ChQuaternion<> q=QuatFromAngleAxis(CH_PI_2, VECT_Z);
	container->SetRot(q);
	container->SetPos(ChVector3d(0,0,0));	
	//container->GetVisualShape(0)->GetMaterial(0)->SetOpacity(0.5);
	//container->SetFixed(true);	
    sys.AddBody(container); 
    return container;	
	
}


/////////////////////////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<ChBody> AddBMCLid(ChSystem& sys, std::shared_ptr<ChContactMaterial> mat, double density, std::string current_dir) {
    // create container 
   
	auto lid = chrono_types::make_shared<ChBodyEasyMesh>(
        (current_dir+"/bmc_lid.obj").c_str(),  // file name for OBJ Wavefront mesh
        density,                                         // density of the body
        true,                                         // automatic evaluation of mass, COG position, inertia tensor
        true,                                         // attach visualization asset
        true,                                         // enable the collision detection
        mat,                                     // surface contact material
        1.0  // radius of 'inflating' of mesh (for more robust collision detection)
    );	
	//	
	lid->SetPos(ChVector3d(0,60,0));
	//
	lid->SetFixed(true);	
    sys.AddBody(lid); 
	
    return lid;	
	
}


/////////////////////////////////////////////////////////////////////////////////////////////////////

//------------------------------------------------------------------
// Function to save BMC to Paraview VTK files
//------------------------------------------------------------------
void WritebmcSolidVTK(const std::string& filename,
                   ChTriangleMeshConnected& mesh,
                   const ChFrame<>& frame) {
    std::ofstream outf;
    outf.open(filename);
    outf << "# vtk DataFile Version 2.0" << std::endl;
    outf << "VTK from simulation" << std::endl;
    outf << "ASCII" << std::endl;
    outf << "DATASET UNSTRUCTURED_GRID" << std::endl;
    outf << "POINTS " << mesh.GetCoordsVertices().size() << " "
         << "float" << std::endl;
    for (auto& v : mesh.GetCoordsVertices()) {
        auto w = frame.TransformPointLocalToParent(v);
        outf << w.x() << " " << w.y() << " " << w.z() << std::endl;
    }
    auto nf = mesh.GetIndicesVertexes().size();
    outf << "CELLS " << nf << " " << 4 * nf << std::endl;
    for (auto& f : mesh.GetIndicesVertexes()) {
        outf << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
    }
    outf << "CELL_TYPES " << nf << std::endl;
    for (int i = 0; i < nf; i++) {
        outf << "5 " << std::endl;
    }
    outf.close();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////

void AddFallingItems(ChSystem& sys, std::shared_ptr<ChVisualSystemIrrlicht>& vis, std::string& data_path, std::string& file_name,  double h_layer, double rho) {
    // Shared contact material for falling objects
    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
    mat->SetYoungModulus(1e2); //2e3
    mat->SetFriction(0.5);
    mat->SetRestitution(0.0f);
    mat->SetAdhesion(0);
	
	std::vector<chrono::ChVector3<float>> body_points;
    std::vector<chrono::ChVector3<float>> body_points_new;
	
	std::string nodesFilename=data_path+file_name+"_coords.dat";
	std::ifstream nodesFile(nodesFilename);   
	//
	// Read COG coordinate of bodies
	//	
	try {
        chrono::ChVector3d pos;
       	double x, y, z;       	
		std::string line;
		
		while (std::getline(nodesFile, line)) {			
			//
			if( line[0]!='#') {				
				std::istringstream sline(line);
				sline>> x >> y >> z;
				//std::cout<<"sline"<<x<<"\t"<<y<<"\t"<<z<<std::endl;
				body_points.push_back(ChVector3d(x,y,z)); 
			}
		}
		
    } catch (std::exception myerr) {
        std::cerr << "ERROR opening nodes info file: "<< std::string(nodesFilename) << myerr.what() << std::endl;        
    }
	
	//
	// Read radius info:
	//
	std::string radiusFilename=data_path+file_name+"_radius.dat";
	std::ifstream radiusFile(radiusFilename);   
	//
	std::vector<double> body_radius;	
	
	try {
        double rad;       	
		std::string line;
		
		while (std::getline(radiusFile, line)) {			
			//
			if( line[0]!='#') {				
				std::istringstream sline(line);
				sline>> rad;
				body_radius.push_back(rad); 
			}
		}
		
    } catch (std::exception myerr) {
        std::cerr << "ERROR opening radius info file: "<< std::string(nodesFilename) << myerr.what() << std::endl;        
    }
	

    auto numSpheres = body_points.size();
    //std::cout<<"numSpheres: "<<numSpheres<<std::endl;
    body_points_new.resize(numSpheres);
    int ii=0;
    int i=0;
    for (i=0; i<numSpheres; i++) {
        float x_pos=body_points[i].x();
        float y_pos=body_points[i].y();
        float z_pos=body_points[i].z();
        double y_level=y_pos-0;
        if ((y_pos>60 || y_pos<-27) || (x_pos*x_pos+z_pos*z_pos)>30.*30.)
			continue;
       
            // && y_pos>margin
            body_points_new[ii]=body_points[i];
            ChVector3d pos={x_pos, y_pos, z_pos};            
            
            //AddSphere(sys,pos);
            ii++;
      

	double radius=body_radius[i];
    double mass = rho/2*4/3*pow(radius, 3)*CH_PI;
                
    auto body = chrono_types::make_shared<ChBody>();
    body->SetInertiaXX((2.0 / 5.0) * mass * pow(radius, 2) * ChVector3d(1, 1, 1));
    body->SetMass(mass);
    //body->SetPos(ChVector3d(11 * ix , 50+11*iy, 11 * iz));
    body->SetPos(pos);	
	
	auto sphere_shape = chrono_types::make_shared<ChCollisionShapeSphere>(mat, radius);
    body->AddCollisionShape(sphere_shape);    
    body->EnableCollision(true);   
	
    auto sphereMor = chrono_types::make_shared<ChVisualShapeSphere>(radius);
	//sphereMor->SetTexture(GetChronoDataFile("textures/blue.png"));
	sphereMor->SetColor(ChColor(128.f/255, 128.f/255, 128.f/255));
	sphereMor->SetOpacity(0.25f);
	body->AddVisualShape(sphereMor);

	auto sphereAgg = chrono_types::make_shared<ChVisualShapeSphere>(radius-h_layer);
	//sphereAgg->SetTexture(GetChronoDataFile("textures/pink.png"));
	sphereAgg->SetColor(ChColor(5.f/255, 48.f/255, 173.f/255));	
	body->AddVisualShape(sphereAgg);
	vis->BindItem(body);	
    sys.AddBody(body);
 
    }
}

//////////////////////////  ==== MAIN ====    /////////////////////////////////////////////////////////

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
	SetChronoDataPath(CHRONO_DATA_DIR);
	///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Get the current diectory
    ///
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
	//
    std::string current_dir(argv[0]);
    int pos = current_dir.find_last_of("/\\");
    current_dir=current_dir.substr(0, pos-5); 
	//
    // Create a Chrono system
    ChSystemSMC sys;
	sys.SetGravitationalAcceleration(ChVector3d(0,-9810,0));
	sys.SetNumThreads(16,16,1);
	sys.SetMaxPenetrationRecoverySpeed(100.);
	//
	sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
	//
    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("BMC");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector3d(0, 170, -100));

    // Create the floor:
    auto floor_mat = chrono_types::make_shared<ChContactMaterialSMC>();
	floor_mat->SetYoungModulus(1e2);
    floor_mat->SetFriction(0.5f);
    floor_mat->SetRestitution(0.0f);
    floor_mat->SetAdhesion(0);
	
	auto floorBody = chrono_types::make_shared<ChBody>();
    floorBody->SetPos(ChVector3d(0, -5, 0));
    floorBody->SetFixed(true);
    sys.Add(floorBody);
	
	//
	double density=7.8E-9;
	auto bmcContainer =AddBMCContainer(sys, floor_mat, density, current_dir);
	bmcContainer->GetVisualShape(0)->SetOpacity(0.25f);
    bmcContainer->SetFixed(true);	
	bmcContainer->GetCollisionModel()->SetFamilyGroup(2);
    bmcContainer->GetCollisionModel()->DisallowCollisionsWith(1);
	bmcContainer->EnableCollision(true);
	//	
	auto bmcLid =AddBMCLid(sys, floor_mat, density, current_dir);
	bmcLid->GetCollisionModel()->SetFamilyGroup(2);
    bmcLid->GetCollisionModel()->DisallowCollisionsWith(1);
	bmcLid->EnableCollision(false);
	//	
	//
	auto bmcRod =CreateBMCrod(sys, vis, floorBody, current_dir);
	//auto bmcRod =AddBMCRod(sys, floor_mat, density, current_dir);
	bmcRod->SetPos(ChVector3d(0,-35,0));
	bmcRod->GetCollisionModel()->SetFamilyGroup(2);
    bmcRod->GetCollisionModel()->DisallowCollisionsWith(1);
	bmcRod->EnableCollision(true);
	
	auto rodRefBody = chrono_types::make_shared<ChBody>();
    rodRefBody->SetPos(bmcRod->GetPos());
    rodRefBody->SetFixed(true);
    sys.Add(rodRefBody);

/////////////////////////////////////////////////////////////////////////////////////////////////////	
	//	volume of specimen
	double specimenVol=40.*40.*250.;
	double Vol=specimenVol;
	
/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Concrete properties
	double h_layer=2.5;
	double minD=2;
    double maxD=4;
	double cement=845;
	double WtoC=0.21;
	double AtoC=1.24;
	double rho_c=3150.;
	double rho_w=1000.;
	double vair=0.03;
	double nF=0.5;	
	double va=1.0-cement/rho_c-(WtoC*cement)/rho_w-vair;
	double va0=(1.-pow((minD/maxD),nF))*va;	
	double Va0=va0*Vol;
	double rho=cement*(1.0+WtoC+AtoC)*1E-12;
	double targetmass=Va0*rho;
	double targetVol=specimenVol*2.5;
	//double rho=2.4E-9;
	std::cout<<"targetmass "<<targetmass<<"\n";	
	std::cout<<"rho "<<rho<<"\n";
	std::cout<<"va: "<<va<<" va0 "<<va0<<"\n";
	//
/////////////////////////////////////////////////////////////////////////////////////////////////////
	// Define material parameters of fresh concrete
	//
	float mortar_layer=h_layer; 
	float ENm=1.5E-3;    
	float ENa=100.0*0.001;
	float h=mortar_layer;		
	float alpha=0.25;
	float beta=0.5;
	float np=1.0;
	float sgmTmax=3.0E-4;  
	float sgmTau0=2.0E-5;  
	float kappa0=100.;
	float eta_inf=1.25E-5;

/////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //	Change default contact force algorithm to fresh concrete ones
/////////////////////////////////////////////////////////////////////////////////////////////////////
	
    auto Contact_force_algorithm = chrono_types::make_unique<FCContactForce>();
	auto materialFCM=chrono_types::make_shared<ChMaterialFCM>(ENm, ENa, mortar_layer, 
									alpha, beta, np, sgmTmax, sgmTau0, kappa0, eta_inf);
    materialFCM->Set_flocbeta(0.01);
    materialFCM->Set_flocm(1.0);
    materialFCM->Set_flocTcr(100.);
    materialFCM->Set_lambda_init(2.0);
    materialFCM->Set_ThixOnFlag(false);
    
    Contact_force_algorithm->Set_Material(materialFCM);
    sys.SetContactForceTorqueAlgorithm(std::move(Contact_force_algorithm));
    //
    auto container = chrono_types::make_shared<MyContactContainer>();
    sys.SetContactContainer(container);	
	//
/////////////////////////////////////////////////////////////////////////////////////////////////////	//
	//
    std::string data_path=current_dir;
	std::string file_name="bmc_particle_New4";	
	AddFallingItems(sys, vis, data_path, file_name, h_layer, rho*0.7051*0.966);
	//	
    // Create the remover
    ChParticleRemoverBox remover;
    remover.SetRemoveOutside(true);
    remover.SetBox(ChVector3d(110, 200, 110), ChFrame<>());
	//
/////////////////////////////////////////////////////////////////////////////////////////////////////
    // Bind all existing visual shapes to the visualization system
	//
    vis->AttachSystem(&sys);
/////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Save bmcRod mesh 
    //
	std::string bmcRod_obj = (current_dir+"/bmcRod.obj").c_str();
	std::string bmcContainer_obj = (current_dir+"/bmcContainer.obj").c_str();
	//
    ChTriangleMeshConnected bmcRod_mesh;
    bmcRod_mesh.LoadWavefrontMesh(bmcRod_obj, false, true);
    bmcRod_mesh.RepairDuplicateVertexes(1e-9);
	//
	ChTriangleMeshConnected bmcContainer_mesh;
    bmcContainer_mesh.LoadWavefrontMesh(bmcContainer_obj , false, true);
    bmcContainer_mesh.RepairDuplicateVertexes(1e-9);
    //
	//
/////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// Create the linear motor
/////////////////////////////////////////////////////////////////////////////////////////////////////	
	auto motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor->SetName("engine_bmcRod_bmcShaft");
    motor->Initialize(bmcRod, rodRefBody, ChFrame<>(bmcRod->GetPos(), chrono::QuatFromAngleAxis(CH_PI / 2.0, VECT_X))); //Rotate on Y Axis
	
	auto f_sequence1 = chrono_types::make_shared<ChFunctionSequence>();
    f_sequence1->InsertFunct(chrono_types::make_shared<ChFunctionConst>(0), 0.5, 1, true);
    f_sequence1->InsertFunct(chrono_types::make_shared<ChFunctionRamp>(0,1.0472/60), 60., 1, true);

    motor->SetSpeedFunction(f_sequence1);	
    sys.AddLink(motor);

    // Modify some setting of the physical system for the simulation, if you want
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    //sys.SetSolverMaxIterations(400);
	//
	//
	char filename[200];
	const std::string out_dir = current_dir + "/OUT_BMC/";
	// Create oputput directories
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }   
	//
    FILE *fptr ;
    auto outfilename=out_dir+"history_BMC.txt";
    fptr= fopen(outfilename.c_str(), "w");
    fprintf(fptr,"time\t");
    fprintf(fptr,"x\t y\t z\t");
    fprintf(fptr,"vx\t vy\t vz\t");
    fprintf(fptr,"wx\t wy\t wz\t");
    fprintf(fptr,"Tx\t Ty\t Tz\n");    
	//
	//
	//
	double totalMass=0;
	for (auto body:sys.GetBodies()){
		if (body->IsFixed() || body->GetCollisionModel()->GetShapeInstance(0).first->GetType()!=0 )
			continue;		
		
		totalMass+=body->GetMass();
						
	}
	std::cout<<"Total mass: "<<totalMass<<"  rho: "<<rho<<std::endl;
//////////////////////////////////////////////////////////////////////////////////////
    // Filling loop
//////////////////////////////////////////////////////////////////////////////////////
	std::cout<<"\n 1. phase: Filling the container and waiting \n"<<std::endl;	
//////////////////////////////////////////////////////////////////////////////////////
	//
	int step_num=0;
    double timestep = 1.0E-8;
    double lambda=0;
    double IE;
    while (vis->Run()& sys.GetChTime()<0.5) {		
		if (step_num%10==0){
			timestep=std::min(timestep*=1.05,10.0e-6);
		}
		//vis->Run()&
		vis->BeginScene(true, true, ChColor(185./255,202./255,222./255));
        //vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Continuosly check if some particle must be removed:
        remover.ProcessParticles(sys);

        sys.DoStepDynamics(timestep);
        	
		if (std::fmod(step_num, 400) ==0) {
			sprintf(filename, "%s/bulk_1va_A%06d.vtk", out_dir.c_str(), step_num);
		    WriteParticlesVTK(sys, filename, mortar_layer, true);
		    sprintf(filename, "%s/bulk_1va_M%06d.vtk", out_dir.c_str(), step_num);
		    WriteParticlesVTK(sys, filename, mortar_layer, false);
			//
			sprintf(filename, "%s/bmcRod_fill%06d.vtk", out_dir.c_str(), step_num);			
            WritebmcSolidVTK(filename, bmcRod_mesh, bmcRod->GetFrameRefToAbs());
			//
			sprintf(filename, "%s/bmcCont_fill%06d.vtk", out_dir.c_str(), step_num);				
            WritebmcSolidVTK(filename, bmcContainer_mesh, bmcContainer->GetFrameRefToAbs());
			//			
			double current_time= sys.GetChTime();
			container->IterOnContactList(current_time,IE);					
			double KE=calculateKE(sys);
			std::cout<<" time: "<< current_time << " IE: "<< IE << " KE: "<< KE << "\n"; 
			//
			if (KE<1E-3 & step_num>4000)
					break;
		     
		 }
		 step_num++;
    }
	
	//	
	bmcLid->EnableCollision(true);
	for (auto body:sys.GetBodies()){
		if (body->IsFixed() || body->GetCollisionModel()->GetShapeInstance(0).first->GetType()!=0 )
			continue;
		
		body->SetLimitSpeed(false);				
	}
			
//////////////////////////////////////////////////////////////////////////////////////
    // Rotating loop
//////////////////////////////////////////////////////////////////////////////////////
	std::cout<<"\n 2. phase: Rotating \n"<<std::endl;	
//////////////////////////////////////////////////////////////////////////////////////
    timestep = 1.0E-8;
    while ( vis->Run()& sys.GetChTime()<60.) {
	double current_time=sys.GetChTime();	
		//
		//vis->Run()&
        vis->BeginScene(true, true, ChColor(185./255,202./255,222./255));
        vis->Render();
        vis->EndScene();        
		if (step_num%20==0){
			timestep=std::min(timestep*=1.025,1.0e-5);
		}
        // Continuosly check if some particle must be removed:
        remover.ProcessParticles(sys);        	
     		
        sys.DoStepDynamics(timestep);
				
		if (std::fmod(step_num, 400) ==0) {
			timestep=std::min(timestep*=1.2,10.0e-6);
		    sprintf(filename, "%s/bulk_1va_A%06d.vtk", out_dir.c_str(), step_num);
		    WriteParticlesVTK(sys, filename, mortar_layer, true);
		    sprintf(filename, "%s/bulk_1va_M%06d.vtk", out_dir.c_str(), step_num);
		    WriteParticlesVTK(sys, filename, mortar_layer, false);			 
			//
			sprintf(filename, "%s/bmcRod_fill%06d.vtk", out_dir.c_str(), step_num);			
            WritebmcSolidVTK(filename, bmcRod_mesh, bmcRod->GetFrameRefToAbs());
			//
			sprintf(filename, "%s/bmcCont_fill%06d.vtk", out_dir.c_str(), step_num);				
            WritebmcSolidVTK(filename, bmcContainer_mesh, bmcContainer->GetFrameRefToAbs());
       
        ChVector3d torque = motor->GetMotorTorque();
        ChVector3d w_pos = bmcRod->GetPos();
        //
        ChVector3d angvel = bmcRod->GetAngVelLocal();
		ChVector3d w_vel = motor->GetMotorAngleDt();
        //
        
        double IE;
        container->IterOnContactList(current_time,IE);

        if (true) {
            std::cout << "time: " << sys.GetChTime() ;
            std::cout << "  bmcRod position:         " << w_pos ;
            std::cout << "  bmcRod linear velocity:  " << w_vel ;
            std::cout << "  bmcRod angular velocity: " << angvel ;            
            std::cout << "  bmcRod torque: " << torque << std::endl;
			fprintf(fptr, " %10.6f\t ", sys.GetChTime() );
			fprintf(fptr, " %10.6f %10.6f  %10.6f\t ", w_pos.x(), w_pos.y(), w_pos.z() );
			fprintf(fptr, " %10.6f %10.6f  %10.6f\t ", w_vel.x(), w_vel.y(), w_vel.z() );
			fprintf(fptr, " %10.6f %10.6f  %10.6f\t ", angvel.x(), angvel.y(), angvel.z() );
			fprintf(fptr, " %10.6f %10.6f  %10.6f\n ", torque.x(), torque.y(), torque.z() );
        } 
		   
		 }
		 step_num++;
    }

    return 0;
}
