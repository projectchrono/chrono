///////////////////////////////////////////////////////////////////////////////
//	main.cpp
//	Reads the initializes the particles, either from file or inside the code
//	
//	Related Files: collideSphereSphere.cu, collideSphereSphere.cuh
//	Input File:		initializer.txt (optional: if initialize from file)
//					This file contains the sph particles specifications. The description 
//					reads the number of particles first. The each line provides the 
//					properties of one SPH particl: 
//					position(x,y,z), radius, velocity(x,y,z), mass, \rho, pressure, mu, particle_type(rigid or fluid)
//
//	Created by Arman Pazouki
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <fstream>
#include <string>
#include <limits.h>
#include <vector>
#include <cstdlib> //for RAND_MAX
#include <ctime>
#include <assert.h>

//for memory leak detection, apparently does not work in conjunction with cuda
//#define _CRTDBG_MAP_ALLOC
//#include <stdlib.h>
//#include <crtdbg.h>//just for min

#include "custom_cutil_math.h"
#include "SPHCudaUtils.h"
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/scan.h>
#include "SDKCollisionSystem.cuh" //just for SimParams
#include "collideSphereSphere.cuh"
#include "printToFile.cuh"
#include <algorithm>

#include <omp.h>
//*************************************************************
			//#include "physics/ChBodyEasy.h"
			#include "physics/ChContactContainer.h"
			#include "collision/ChCModelBulletBody.h"
			//#include "core/ChTimer.h"
			//#include "core/ChRealtimeStep.h"
			//#include "assets/ChTexture.h"
			#include "unit_IRRLICHT/ChIrrApp.h"
			#include <cstring>
			#include <fstream>
			#include <sstream>
			#include <time.h>
			#include <cstdlib>
			//#include <map>

			//*************** chrono parallel
			#include <stdio.h>
			#include <vector>
			#include <cmath>

			#include "chrono_parallel/physics/ChSystemParallel.h"
			#include "chrono_parallel/lcp/ChLcpSystemDescriptorParallel.h"

			#include "chrono_utils/ChUtilsCreators.h"  //Arman: why is this
			#include "chrono_utils/ChUtilsInputOutput.h" //Arman: Why is this
			#include "chrono_utils/ChUtilsGenerators.h"

			//***********************************
			// Use the namespace of Chrono

			using namespace chrono;
			using namespace chrono::collision;

			// Use the main namespaces of Irrlicht
			using namespace irr;
			using namespace core;
			using namespace scene;
			using namespace video;
			using namespace io;
			using namespace gui;
			using namespace std;


			#define irrlichtVisualization false
			#if irrlichtVisualization
			shared_ptr<ChIrrApp> application;
			#endif

			#ifdef CHRONO_PARALLEL_HAS_OPENGL2
			#include "chrono_opengl/ChOpenGLWindow.h"
			opengl::ChOpenGLWindow &gl_window = opengl::ChOpenGLWindow::getInstance();
			#endif
//*************************************************************




using namespace std;

SimParams paramsH;
//typedef unsigned int uint;

////************ note: paramsH is zero here. These expressions are wrong
const Real mm = .001;

//&&& some other definitions for boundary and such

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
struct CylinderGeometry {
	Real3 pa3;
	Real3 pb3;
	Real3 center;
	Real r;
	Real h;
};
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void MySeed(double s = time(NULL)) {
	 srand(s);
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
Real myRand() {
	return Real(rand()) / RAND_MAX;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
Real Min(Real a, Real b) {
	return (a < b) ? a : b;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
Real Max(Real a, Real b) {
	return (a < b) ? b : a;
}

void SetArgumentsForMbdFromInput(int argc, char* argv[], int & threads, uint & max_iteration) {
	if (argc > 1) {
		const char* text = argv[1];
    	threads = atoi(text);
	}
	if (argc > 4) {
		const char* text = argv[4];
		max_iteration = atoi(text);
	}
}

void InitializeMbdPhysicalSystem(ChSystemParallelDVI & mphysicalSystem, int argc, char* argv[]) {
	int threads = 2;
	uint max_iteration = 20;//10000;
	SetArgumentsForMbdFromInput(argc, argv, threads, max_iteration);
	//******************** OMP settings **************
	// Set number of threads.
	int max_threads = mphysicalSystem.GetParallelThreadNumber();
	if (threads > max_threads)
	  threads = max_threads;
	mphysicalSystem.SetParallelThreadNumber(threads);
	omp_set_num_threads(threads);
	//************************************************
	double tolerance = 1e-3; // Arman, not used
	double collisionEnvelop = .04 * paramsH.HSML;
	mphysicalSystem.Set_G_acc(ChVector<>(paramsH.gravity.x, paramsH.gravity.y, paramsH.gravity.z));

	// Set solver parameters
	mphysicalSystem.GetSettings()->solver.solver_mode = SLIDING; //NORMAL, SPINNING
	mphysicalSystem.GetSettings()->solver.max_iteration_normal = max_iteration / 3;
	mphysicalSystem.GetSettings()->solver.max_iteration_sliding = max_iteration / 3;
	mphysicalSystem.GetSettings()->solver.max_iteration_spinning = 0;
	mphysicalSystem.GetSettings()->solver.max_iteration_bilateral = max_iteration / 3;
	mphysicalSystem.GetSettings()->solver.tolerance = 0;//tolerance;
	mphysicalSystem.GetSettings()->solver.alpha = 0;  //Arman, find out what is this
	mphysicalSystem.GetSettings()->solver.contact_recovery_speed = paramsH.v_Max;
	mphysicalSystem.ChangeSolverType(APGDREF);  //Arman check this APGD APGDBLAZE
	mphysicalSystem.GetSettings()->collision.narrowphase_algorithm = NARROWPHASE_HYBRID_MPR;

	mphysicalSystem.GetSettings()->collision.collision_envelope = collisionEnvelop;
	mphysicalSystem.GetSettings()->collision.bins_per_axis = mI3(10, 10, 10); //Arman check
}

void create_system_particles(ChSystemParallelDVI& mphysicalSystem) {
	Real3 domainCenter = 0.5 * (paramsH.cMin + paramsH.cMax);
	Real3 ellipsoidSize = .3 * mR3(10 * paramsH.HSML, 5 * paramsH.HSML, 7 * paramsH.HSML);
	ChVector<> size = ChVector<>(ellipsoidSize.x, ellipsoidSize.y, ellipsoidSize.z);

	double density = paramsH.rho0;  // TODO : Arman : Density for now is set to water density
	double muFriction = .1;

	// NOTE: mass properties and shapes are all for sphere
	double volume = utils::CalcSphereVolume(size.x);
	ChVector<> gyration = utils::CalcSphereGyration(size.x).Get_Diag();
	double mass = density * volume;


	//**************** bin and ship
	int bId = 1;

	// Create a common material
	ChSharedPtr<ChMaterialSurface> mat_g(new ChMaterialSurface);
	mat_g->SetFriction(muFriction);
	mat_g->SetCohesion(0);
	mat_g->SetCompliance(0.0);
	mat_g->SetComplianceT(0.0);
	mat_g->SetDampingF(0.2);

	const ChVector<> pos = ChVector<>(domainCenter.x, domainCenter.y, domainCenter.z);
	const ChQuaternion<> rot = ChQuaternion<>(1, 0, 0, 0);

	ChSharedBodyPtr body;
	body = ChSharedBodyPtr(new  ChBody(new ChCollisionModelParallel));
	body->SetMaterialSurface(mat_g);
	body->SetIdentifier(bId);
	body->SetPos(pos);
	body->SetRot(rot);
	body->SetCollide(true);
	body->SetBodyFixed(false);
    body->SetMass(mass);
    body->SetInertiaXX(mass * gyration);

	body->GetCollisionModel()->ClearModel();

	// add collision geometry
//	body->GetCollisionModel()->AddEllipsoid(size.x, size.y, size.z, pos, rot);
//
//	// add asset (for visualization)
//	ChSharedPtr<ChEllipsoidShape> ellipsoid(new ChEllipsoidShape);
//	ellipsoid->GetEllipsoidGeometry().rad = size;
//	ellipsoid->Pos = pos;
//	ellipsoid->Rot = rot;
//
//	body->GetAssets().push_back(ellipsoid);

//	utils::AddCapsuleGeometry(body.get_ptr(), size.x, size.y);		// X
//	utils::AddCylinderGeometry(body.get_ptr(), size.x, size.y);		// O
//	utils::AddConeGeometry(body.get_ptr(), size.x, size.y); 		// X
//	utils::AddBoxGeometry(body.get_ptr(), size);					// O
	utils::AddSphereGeometry(body.get_ptr(), size.x);				// O
//	utils::AddEllipsoidGeometry(body.get_ptr(), size);					// X


	body->GetCollisionModel()->BuildModel();
	mphysicalSystem.AddBody(body);



	// ****************** create boxes around the fluid domain

	ChSharedPtr<ChMaterialSurface> mat(new ChMaterialSurface);
	mat->SetFriction(.5);
	mat->SetDampingF(0.2f);
	int binId = -200;

	ChSharedBodyPtr bin;
	Real3 hdim = paramsH.boxDims;
	double hthick = 1 * paramsH.HSML;

	bin = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
	bin->SetMaterialSurface(mat);
	bin->SetIdentifier(binId);
	bin->SetMass(1);
	bin->SetPos(ChVector<>(domainCenter.x, domainCenter.y, domainCenter.z));
	bin->SetRot(ChQuaternion<>(1, 0, 0, 0));
	bin->SetCollide(true);
	bin->SetBodyFixed(true);
	bin->GetCollisionModel()->ClearModel();

	utils::AddBoxGeometry(bin.get_ptr(), 0.5 * ChVector<>(hdim.x, hdim.y, hthick), ChVector<>(0, 0, -0.5 * hdim.z - 0.5*hthick));	//beginning wall
	utils::AddBoxGeometry(bin.get_ptr(), 0.5 * ChVector<>(hdim.x, hdim.y, hthick), ChVector<>(0, 0, 0.5 * hdim.z + 0.5*hthick));	//end wall
	utils::AddBoxGeometry(bin.get_ptr(), 0.5 * ChVector<>(hthick, hdim.y, hdim.z + 2 * hthick), ChVector<>(-0.5 * hdim.x - 0.5 * hthick, 0, 0));		//side wall
	utils::AddBoxGeometry(bin.get_ptr(), 0.5 * ChVector<>(hthick, hdim.y, hdim.z + 2 * hthick), ChVector<>(0.5 * hdim.x + 0.5 * hthick, 0, 0));	//side wall
	utils::AddBoxGeometry(bin.get_ptr(), 0.5 * ChVector<>(hdim.x + 2 * hthick, hthick, hdim.z + 2 * hthick), ChVector<>(0, -0.5 * hdim.y - 0.5 * hthick, 0));	//bottom wall
	bin->GetCollisionModel()->BuildModel();

	mphysicalSystem.AddBody(bin);
}

void AddSphDataToChSystem(
		ChSystemParallelDVI& mphysicalSystem,
		int & startIndexSph,
		const thrust::host_vector<Real3> & posRadH,
		const thrust::host_vector<Real4> & velMasH,
		const NumberOfObjects & numObjects) {

	Real3 domainCenter = 0.5 * (paramsH.cMin + paramsH.cMax);
	Real3 ellipsoidSize = .3 * mR3(10 * paramsH.HSML, 5 * paramsH.HSML, 7 * paramsH.HSML);
	ChVector<> size = ChVector<>(ellipsoidSize.x, ellipsoidSize.y, ellipsoidSize.z);
	Real rad = paramsH.HSML;
	// NOTE: mass properties and shapes are all for sphere
	double volume = utils::CalcSphereVolume(rad);
	ChVector<> gyration = utils::CalcSphereGyration(rad).Get_Diag();
	double density = paramsH.rho0;
	double mass = density * volume;
	double muFriction = 0;

	//int fId = 0; //fluid id

	// Create a common material
	ChSharedPtr<ChMaterialSurface> mat_g(new ChMaterialSurface);
	mat_g->SetFriction(muFriction);
	mat_g->SetCohesion(0);
	mat_g->SetCompliance(0.0);
	mat_g->SetComplianceT(0.0);
	mat_g->SetDampingF(0.2);

	const ChQuaternion<> rot = ChQuaternion<>(1, 0, 0, 0);


	startIndexSph = mphysicalSystem.GetNbodiesTotal();

	// openmp does not work here
	for (int i = 0; i < numObjects.numAllMarkers; i++) {
		Real3 p3 = posRadH[i];
		Real4 vM4 = velMasH[i];
		ChVector<> pos = ChVector<>(p3.x, p3.y, p3.z);
		ChVector<> vel = ChVector<>(vM4.x, vM4.y, vM4.z);

		ChSharedBodyPtr body;
		body = ChSharedBodyPtr(new  ChBody(new ChCollisionModelParallel));
		body->SetMaterialSurface(mat_g);
		//body->SetIdentifier(fId);
		body->SetPos(pos);
		body->SetRot(rot);
		body->SetCollide(true);
		body->SetBodyFixed(false);
	    body->SetMass(mass);
	    body->SetInertiaXX(mass * gyration);

		body->GetCollisionModel()->ClearModel();

		// add collision geometry
	//	body->GetCollisionModel()->AddEllipsoid(size.x, size.y, size.z, pos, rot);
	//
	//	// add asset (for visualization)
	//	ChSharedPtr<ChEllipsoidShape> ellipsoid(new ChEllipsoidShape);
	//	ellipsoid->GetEllipsoidGeometry().rad = size;
	//	ellipsoid->Pos = pos;
	//	ellipsoid->Rot = rot;
	//
	//	body->GetAssets().push_back(ellipsoid);

	//	utils::AddCapsuleGeometry(body.get_ptr(), size.x, size.y);		// X
	//	utils::AddCylinderGeometry(body.get_ptr(), size.x, size.y);		// O
	//	utils::AddConeGeometry(body.get_ptr(), size.x, size.y); 		// X
	//	utils::AddBoxGeometry(body.get_ptr(), size);					// O
		utils::AddSphereGeometry(body.get_ptr(), size.x);				// O
	//	utils::AddEllipsoidGeometry(body.get_ptr(), size);					// X

		body->GetCollisionModel()->SetFamily(100);
		body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(100);

		body->GetCollisionModel()->BuildModel();
		mphysicalSystem.AddBody(body);
	}
}

void UpdateSphDataInChSystem(
		ChSystemParallelDVI& mphysicalSystem,
		const thrust::host_vector<Real3> & posRadH,
		const thrust::host_vector<Real4> & velMasH,
		const NumberOfObjects & numObjects,
		int  startIndexSph) {

	#pragma omp parallel for
	for (int i = 0; i < numObjects.numAllMarkers; i++) {
		Real3 p3 = posRadH[i];
		Real4 vM4 = velMasH[i];
		ChVector<> pos = ChVector<>(p3.x, p3.y, p3.z);
		ChVector<> vel = ChVector<>(vM4.x, vM4.y, vM4.z);

		int chSystemBodyId = startIndexSph + i;
		vector<ChBody*>::iterator ibody = mphysicalSystem.Get_bodylist()->begin() + chSystemBodyId;
		(*ibody)->SetPos(pos);
		(*ibody)->SetPos(vel);
	}
}

void InitializeChronoGraphics(ChSystemParallelDVI& mphysicalSystem, Real dT) {
	Real3 domainCenter = 0.5 * (paramsH.cMin + paramsH.cMax);
	ChVector<> CameraLocation = ChVector<>(2 * paramsH.cMax.x, 2 * paramsH.cMax.y, 2 * paramsH.cMax.z);
	ChVector<> CameraLookAt = ChVector<>(domainCenter.x, domainCenter.y, domainCenter.z);

#ifdef CHRONO_PARALLEL_HAS_OPENGL2
//   gl_window = opengl::ChOpenGLWindow::getInstance();
   gl_window.Initialize(1280, 720, "mixerDVI", &mphysicalSystem);
   gl_window.SetCamera(CameraLocation, CameraLookAt, ChVector<>(0, 1, 0)); //camera

   // Uncomment the following two lines for the OpenGL manager to automatically
   // run the simulation in an infinite loop.
   //gl_window.StartDrawLoop(time_step);
   //return 0;
#endif

#if irrlichtVisualization
	// Create the Irrlicht visualization (open the Irrlicht device,
	// bind a simple user interface, etc. etc.)
   application = shared_ptr<ChIrrApp>(new ChIrrApp(&mphysicalSystem, L"Bricks test",core::dimension2d<u32>(800,600),false, true));
//	ChIrrApp application(&mphysicalSystem, L"Bricks test",core::dimension2d<u32>(800,600),false, true);


		// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
		ChIrrWizard::add_typical_Logo  (application->GetDevice());
//		ChIrrWizard::add_typical_Sky   (application->GetDevice());
		ChIrrWizard::add_typical_Lights(application->GetDevice(), core::vector3df(14.0f, 44.0f, -18.0f), core::vector3df(-3.0f, 8.0f, 6.0f), 59,  40);
		ChIrrWizard::add_typical_Camera(application->GetDevice(),
				core::vector3df(CameraLocation.x, CameraLocation.y, CameraLocation.z),
				core::vector3df(CameraLookAt.x, CameraLookAt.y, CameraLookAt.z)); //   (7.2,30,0) :  (-3,12,-8)
		// Use this function for adding a ChIrrNodeAsset to all items
		// If you need a finer control on which item really needs a visualization proxy in
		// Irrlicht, just use application->AssetBind(myitem); on a per-item basis.
		application->AssetBindAll();
		// Use this function for 'converting' into Irrlicht meshes the assets
		// into Irrlicht-visualizable meshes
		application->AssetUpdateAll();

		application->SetStepManage(true);
		application->SetTimestep(dT);  					//Arman modify
#endif
}

int DoStepChronoSystem(ChSystemParallelDVI& mphysicalSystem, Real dT) {
	Real3 domainCenter = 0.5 * (paramsH.cMin + paramsH.cMax);
#if irrlichtVisualization
		if ( !(application->GetDevice()->run()) ) return 0;
		application->GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));
		ChIrrTools::drawGrid(application->GetVideoDriver(), 2 * paramsH.HSML, 2 * paramsH.HSML, 50, 50,
			ChCoordsys<>(ChVector<>(domainCenter.x, paramsH.worldOrigin.y, domainCenter.z), Q_from_AngAxis(CH_C_PI/2,VECT_X)), video::SColor(50,90,90,150),true);
		application->DrawAll();
		application->DoStep();
		application->GetVideoDriver()->endScene();
#else
#ifdef CHRONO_PARALLEL_HAS_OPENGL2
		if (gl_window.Active()) {
		 gl_window.DoStepDynamics(dT);
		 gl_window.Render();
		}
#else
		mphysicalSystem.DoStepDynamics(dT);
#endif
#endif
		return 1;
}

void AddChSystemForcesToSphForces(
		thrust::host_vector<Real4>  & derivVelRhoChronoH,
		const thrust::host_vector<Real4> & velMasH2,
		ChSystemParallelDVI& mphysicalSystem,
		const NumberOfObjects & numObjects,
		int startIndexSph,
		Real dT) {
	std::vector<ChBody*>::iterator bodyIter = mphysicalSystem.Get_bodylist()->begin() + startIndexSph;
#pragma omp parallel for
	for (int i = 0; i < numObjects.numAllMarkers; i++) {
//		std::vector<ChBody*>::iterator bodyIterB = bodyIter + i;
		ChVector<> v = ((ChBody*)(*(bodyIter + i)))->GetPos_dt();
		Real3 a3 = (mR3(v.x, v.y, v.z) - mR3(velMasH2[i])) / dT; // f = m * a
		derivVelRhoChronoH[i] += mR4(a3,0);
	}
}

void SavePovFilesMBD(ChSystemParallelDVI& mphysicalSystem, int tStep) {
	// Save PovRay post-processing data.
	const std::string pov_dir = "povray";
	if (tStep == 0) {
		//linux. In windows, it is System instead of system (to invoke a command in the command line)
		system("mkdir -p povray");
		system("rm povray/*.*");
	}
	int stepSave = 50;
	if (tStep % stepSave == 0) {
		char filename[100];
		sprintf(filename, "%s/data_%03d.csv", pov_dir.c_str(), tStep / stepSave + 1);
		utils::WriteBodies(&mphysicalSystem, filename);
	}
}
//*** paramsH.straightChannelBoundaryMax   should be taken care of
Real IsInsideStraightChannel(Real3 posRad) {
	const Real sphR = paramsH.HSML;
	Real penDist1 = 0;
	Real penDist2 = 0;
	Real penDist3 = 0;
	Real largePenet = -5 * sphR; //like a large number. Should be negative (assume large initial penetration)

	if (posRad.z > paramsH.straightChannelBoundaryMax.z) {
		penDist1 = paramsH.straightChannelBoundaryMax.z - posRad.z;
	}
	if (posRad.z < paramsH.straightChannelBoundaryMin.z) {
		penDist1 = posRad.z - paramsH.straightChannelBoundaryMin.z;
	}

	if (posRad.y < paramsH.straightChannelBoundaryMin.y) {
		penDist2 = posRad.y - paramsH.straightChannelBoundaryMin.y;
	}
	if (posRad.y > paramsH.straightChannelBoundaryMax.y) {
		penDist2 = paramsH.straightChannelBoundaryMax.y - posRad.y;
	}

	if (posRad.x < paramsH.straightChannelBoundaryMin.x) {
		penDist3 = posRad.x - paramsH.straightChannelBoundaryMin.x;
	}
	if (posRad.x > paramsH.straightChannelBoundaryMax.x) {
		penDist3 = paramsH.straightChannelBoundaryMax.x - posRad.x;
	}

	if (penDist1 < 0 && penDist2 < 0 && penDist2 < 0) {
		return Min(penDist1, penDist2);
	}
	if (penDist1 < 0)
		return penDist1;
	if (penDist2 < 0)
		return penDist2;
	return -largePenet;
}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
int2 CreateFluidMarkers(
		thrust::host_vector<Real3> & posRadH,
		thrust::host_vector<Real4> & velMasH,
		thrust::host_vector<Real4> & rhoPresMuH,
		thrust::host_vector<uint> & bodyIndex,
		Real & sphMarkerMass) {

	thrust::host_vector<Real3> mPosRadBoundary; //do not set the size here since you are using push back later
	thrust::host_vector<Real4> mVelMasBoundary;
	thrust::host_vector<Real4> mRhoPresMuBoundary;

	int num_FluidMarkers = 0;
	int num_BoundaryMarkers = 0;
	srand(964);
	Real multInitSpace = paramsH.MULT_INITSPACE; //0.9;//0.9;
	Real initSpace0 = paramsH.MULT_INITSPACE * paramsH.HSML;
	int nFX = ceil((paramsH.cMax.x - paramsH.cMin.x) / (initSpace0));
	Real initSpaceX = (paramsH.cMax.x - paramsH.cMin.x) / nFX;
	//printf("orig nFx and nFx %f %f\n", (paramsH.cMax.x - paramsH.cMin.x) / initSpace, ceil ((paramsH.cMax.x - paramsH.cMin.x) / (initSpace)));
	int nFY = ceil((paramsH.cMax.y - paramsH.cMin.y) / (initSpace0));
	Real initSpaceY = (paramsH.cMax.y - paramsH.cMin.y) / nFY;
	int nFZ = ceil((paramsH.cMax.z - paramsH.cMin.z) / (initSpace0));
	Real initSpaceZ = (paramsH.cMax.z - paramsH.cMin.z) / nFZ;
	//printf("&&&&& %f   %f %f %f \n", 1.1 * sphR, initSpaceX, initSpaceY, initSpaceZ);
	printf("nFX Y Z %d %d %d, max distY %f, initSpaceY %f\n", nFX, nFY, nFZ,
			(nFY - 1) * initSpaceY, initSpaceY);
	sphMarkerMass = (initSpaceX * initSpaceY * initSpaceZ) * paramsH.rho0;

	for (int i = 0; i < nFX; i++) {
		for (int j = 0; j < nFY; j++) {
			for (int k = 0; k < nFZ; k++) {
				Real3 posRad;
//					printf("initSpace X, Y, Z %f %f %f \n", initSpaceX, initSpaceY, initSpaceZ);
				posRad =
						paramsH.cMin
								+ mR3(i * initSpaceX, j * initSpaceY, k * initSpaceZ)
								+ mR3(.5 * initSpace0)/* + mR3(sphR) + initSpace * .05 * (Real(rand()) / RAND_MAX)*/;
				if ( 	(posRad.x >  paramsH.straightChannelBoundaryMin.x && posRad.x <  paramsH.straightChannelBoundaryMax.x ) &&
						(posRad.y >  paramsH.straightChannelBoundaryMin.y && posRad.y <  paramsH.straightChannelBoundaryMax.y ) &&
						(posRad.z >  paramsH.straightChannelBoundaryMin.z && posRad.z <  paramsH.straightChannelBoundaryMax.z ) )
				{
					if (i < 0.5 * nFX) {
						num_FluidMarkers++;
						posRadH.push_back(posRad);
						Real3 v3 = mR3(0);
						velMasH.push_back(mR4(v3, sphMarkerMass));
						rhoPresMuH.push_back(mR4(paramsH.rho0, paramsH.BASEPRES, paramsH.mu0, -1));
					}
				} else {
					num_BoundaryMarkers++;
					mPosRadBoundary.push_back(posRad);
					mVelMasBoundary.push_back(mR4(0, 0, 0, sphMarkerMass));
					mRhoPresMuBoundary.push_back(mR4(paramsH.rho0, paramsH.LARGE_PRES, paramsH.mu0, 0));
				}
			}
		}
	}
	int2 num_fluidOrBoundaryMarkers = mI2(num_FluidMarkers, num_BoundaryMarkers);
	// *** copy boundary markers to the end of the markers arrays
	posRadH.resize(
			num_fluidOrBoundaryMarkers.x + num_fluidOrBoundaryMarkers.y);
	velMasH.resize(
			num_fluidOrBoundaryMarkers.x + num_fluidOrBoundaryMarkers.y);
	rhoPresMuH.resize(
			num_fluidOrBoundaryMarkers.x + num_fluidOrBoundaryMarkers.y);

	thrust::copy(mPosRadBoundary.begin(), mPosRadBoundary.end(),
			posRadH.begin() + num_fluidOrBoundaryMarkers.x);
	thrust::copy(mVelMasBoundary.begin(), mVelMasBoundary.end(),
			velMasH.begin() + num_fluidOrBoundaryMarkers.x);
	thrust::copy(mRhoPresMuBoundary.begin(), mRhoPresMuBoundary.end(),
			rhoPresMuH.begin() + num_fluidOrBoundaryMarkers.x);
	// *** end copy
	mPosRadBoundary.clear();
	mVelMasBoundary.clear();
	mRhoPresMuBoundary.clear();

	int numAllMarkers = num_fluidOrBoundaryMarkers.x + num_fluidOrBoundaryMarkers.y;
	bodyIndex.resize(numAllMarkers);
	thrust::fill(bodyIndex.begin(), bodyIndex.end(), 1);
	thrust::exclusive_scan(bodyIndex.begin(), bodyIndex.end(),
			bodyIndex.begin());

	return num_fluidOrBoundaryMarkers;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

void SetupParamsH(SimParams & paramsH) {
	paramsH.sizeScale = 1; //don't change it.
	paramsH.HSML = 0.2;
	paramsH.MULT_INITSPACE = 1.0;
		paramsH.NUM_BOUNDARY_LAYERS = 3;
		paramsH.toleranceZone = paramsH.NUM_BOUNDARY_LAYERS * (paramsH.HSML * paramsH.MULT_INITSPACE);
	paramsH.BASEPRES = 0;//10;
		paramsH.LARGE_PRES = 10000;//paramsH.BASEPRES;//10000;
		paramsH.deltaPress; //** modified below
		paramsH.multViscosity_FSI = 5.0;
	paramsH.gravity = mR3(0, -9.81, 0);//mR3(0);//mR3(0, -9.81, 0);
	paramsH.bodyForce3 = mR3(0,0,0);//mR4(3.2e-3,0,0,0);// mR4(0);;// /*Re = 100 */ //mR4(3.2e-4, 0, 0, 0);/*Re = 100 */
		paramsH.rho0 = 1000;
		paramsH.mu0 = .001;
	paramsH.v_Max = 10;//50e-3;//18e-3;//1.5;//2e-1; /*0.2 for Re = 100 */ //2e-3;
		paramsH.EPS_XSPH = .5f;
	paramsH.dT = 0.0001;//0.1;//.001; //sph alone: .01 for Re 10;
		paramsH.tFinal = 1000;//20 * paramsH.dT; //400
		paramsH.timePause = 0;//.0001 * paramsH.tFinal;//.0001 * paramsH.tFinal; 	// time before applying any bodyforce. Particles move only due to initialization. keep it as small as possible. the time step will be 1/10 * dT.
		paramsH.kdT = 5; // I don't know what is kdT
		paramsH.gammaBB = 0.5;
	// ************
		paramsH.binSize0; // will be changed
		paramsH.rigidRadius; //will be changed
	paramsH.densityReinit = 0; //0: no re-initialization, 1: with initialization
	//****************************************************************************************
	//*** initialize straight channel
	paramsH.straightChannelBoundaryMin = mR3(0, 0, 0); //3D channel
	paramsH.straightChannelBoundaryMax = mR3(3, 2, 3) * paramsH.sizeScale;
	//********************************************************************************************************
	//**  reminiscent of the past******************************************************************************
	paramsH.cMin = mR3(-paramsH.toleranceZone, -paramsH.toleranceZone, -paramsH.toleranceZone);						// 3D channel
	paramsH.cMax = mR3( 3  + paramsH.toleranceZone, 2 + paramsH.toleranceZone,  3 + paramsH.toleranceZone);
	//****************************************************************************************
	//printf("a1  paramsH.cMax.x, y, z %f %f %f,  binSize %f\n", paramsH.cMax.x, paramsH.cMax.y, paramsH.cMax.z, 2 * paramsH.HSML);
	int3 side0 = mI3(
			floor((paramsH.cMax.x - paramsH.cMin.x) / (2 * paramsH.HSML)),
			floor((paramsH.cMax.y - paramsH.cMin.y) / (2 * paramsH.HSML)),
			floor((paramsH.cMax.z - paramsH.cMin.z) / (2 * paramsH.HSML)));
	Real3 binSize3 = mR3((paramsH.cMax.x - paramsH.cMin.x) / side0.x,
			(paramsH.cMax.y - paramsH.cMin.y) / side0.y,
			(paramsH.cMax.z - paramsH.cMin.z) / side0.z);
	paramsH.binSize0 = (binSize3.x > binSize3.y) ? binSize3.x : binSize3.y;
//	paramsH.binSize0 = (paramsH.binSize0 > binSize3.z) ? paramsH.binSize0 : binSize3.z;
	paramsH.binSize0 = binSize3.x; //for effect of distance. Periodic BC in x direction. we do not care about paramsH.cMax y and z.
	paramsH.cMax = paramsH.cMin + paramsH.binSize0 * mR3(side0);
	paramsH.boxDims = paramsH.cMax - paramsH.cMin;
	//************************** modify pressure ***************************
//		paramsH.deltaPress = paramsH.rho0 * paramsH.boxDims * paramsH.bodyForce3;  //did not work as I expected
	paramsH.deltaPress = 0.9 * paramsH.boxDims * paramsH.bodyForce3;

	// modify bin size stuff
	//****************************** bin size adjustement and contact detection stuff *****************************
	int3 SIDE = mI3(int((paramsH.cMax.x - paramsH.cMin.x) / paramsH.binSize0 + .1), int((paramsH.cMax.y - paramsH.cMin.y) / paramsH.binSize0 + .1),
			int((paramsH.cMax.z - paramsH.cMin.z) / paramsH.binSize0 + .1));
	Real mBinSize = paramsH.binSize0; //Best solution in that case may be to change cMax or cMin such that periodic sides be a multiple of binSize
	//**********************************************************************************************************
	paramsH.gridSize = SIDE;
	//paramsH.numCells = SIDE.x * SIDE.y * SIDE.z;
	paramsH.worldOrigin = paramsH.cMin;
	paramsH.cellSize = mR3(mBinSize, mBinSize, mBinSize);

	//***** print numbers
	printf("********************\n paramsH.sizeScale: %f\n paramsH.HSML: %f\n paramsH.bodyForce3: %f %f %f\n paramsH.gravity: %f %f %f\n paramsH.rho0: %e\n paramsH.mu0: %f\n paramsH.v_Max: %f\n paramsH.dT: %e\n paramsH.tFinal: %f\n  paramsH.timePause: %f\n  paramsH.timePauseRigidFlex: %f\n paramsH.densityReinit: %d\n",
			paramsH.sizeScale, paramsH.HSML, paramsH.bodyForce3.x, paramsH.bodyForce3.y, paramsH.bodyForce3.z, paramsH.gravity.x, paramsH.gravity.y, paramsH.gravity.z,
			paramsH.rho0, paramsH.mu0, paramsH.v_Max, paramsH.dT, paramsH.tFinal, paramsH.timePause, paramsH.timePauseRigidFlex, paramsH.densityReinit);
	printf(" paramsH.cMin: %f %f %f, paramsH.cMax: %f %f %f\n binSize: %f\n",
			paramsH.cMin.x, paramsH.cMin.y, paramsH.cMin.z, paramsH.cMax.x,
			paramsH.cMax.y, paramsH.cMax.z, paramsH.binSize0);
	printf(" paramsH.MULT_INITSPACE: %f\n", paramsH.MULT_INITSPACE);
	printf(" paramsH.NUM_BOUNDARY_LAYERS: %d\n paramsH.toleranceZone: %f\n paramsH.NUM_BCE_LAYERS: %d\n paramsH.solidSurfaceAdjust: %f\n", paramsH.NUM_BOUNDARY_LAYERS, paramsH.toleranceZone, paramsH.NUM_BCE_LAYERS, paramsH.solidSurfaceAdjust);
	printf(" paramsH.BASEPRES: %f\n paramsH.LARGE_PRES: %f\n paramsH.deltaPress: %f %f %f\n", paramsH.BASEPRES, paramsH.LARGE_PRES, paramsH.deltaPress.x, paramsH.deltaPress.y, paramsH.deltaPress.z);
	printf(" paramsH.nPeriod: %d\n paramsH.EPS_XSPH: %f\n paramsH.multViscosity_FSI: %f\n paramsH.rigidRadius: %f\n", paramsH.nPeriod, paramsH.EPS_XSPH, paramsH.multViscosity_FSI, paramsH.rigidRadius);
	printf("boxDims: %f, %f, %f\n", paramsH.boxDims.x, paramsH.boxDims.y, paramsH.boxDims.z);
	printf("SIDE: %d, %d, %d\n", paramsH.gridSize.x, paramsH.gridSize.y, paramsH.gridSize.z);
}
//@@@@@@@@@@@@@@@@@@@@@@@@@ set number of objects once for all @@@@@@@@@@@@@@@@@@@@@@22
void SetNumObjects(NumberOfObjects & numObjects, const thrust::host_vector<int3> & referenceArray, int numAllMarkers) {
	numObjects.numFluidMarkers = (referenceArray[0]).y - (referenceArray[0]).x;
	numObjects.numBoundaryMarkers = (referenceArray[1]).y - (referenceArray[1]).x;
	numObjects.numAllMarkers = numAllMarkers;

	numObjects.numRigidBodies = 0;
	numObjects.numRigid_SphMarkers = 0;
	numObjects.numFlex_SphMarkers = 0;
	printf("********************\n numFlexBodies: %d\n numRigidBodies: %d\n numFluidMarkers: %d\n "
			"numBoundaryMarkers: %d\n numRigid_SphMarkers: %d\n numFlex_SphMarkers: %d\n numAllMarkers: %d\n",
			numObjects.numFlexBodies, numObjects.numRigidBodies, numObjects.numFluidMarkers, numObjects.numBoundaryMarkers,
			numObjects.numRigid_SphMarkers, numObjects.numFlex_SphMarkers, numObjects.numAllMarkers);
	printf("********************\n");
}

void ClearArraysH(
	thrust::host_vector<Real3> & posRadH, //do not set the size here since you are using push back later
	thrust::host_vector<Real4> & velMasH,
	thrust::host_vector<Real4> & rhoPresMuH) {

	posRadH.clear();
	velMasH.clear();
	rhoPresMuH.clear();
}
void ClearArraysH(
	thrust::host_vector<Real3> & posRadH, //do not set the size here since you are using push back later
	thrust::host_vector<Real4> & velMasH,
	thrust::host_vector<Real4> & rhoPresMuH,
	thrust::host_vector<uint> & bodyIndex,
	thrust::host_vector<int3> & referenceArray) {

	ClearArraysH(posRadH, velMasH, rhoPresMuH);
	bodyIndex.clear();
	referenceArray.clear();
}

void CopyD2H(
	thrust::host_vector<Real4> & derivVelRhoChronoH,
	const thrust::device_vector<Real4> & derivVelRhoD) {
	assert(derivVelRhoChronoH.size() == derivVelRhoD.size() && "Error! size mismatch host and device" );
	thrust::copy(derivVelRhoD.begin(), derivVelRhoD.end(), derivVelRhoChronoH.begin());
}
void CopyH2D(
	thrust::device_vector<Real4> & derivVelRhoD,
	const thrust::host_vector<Real4> & derivVelRhoChronoH) {
	assert(derivVelRhoChronoH.size() == derivVelRhoD.size() && "Error! size mismatch host and device" );
	thrust::copy(derivVelRhoChronoH.begin(), derivVelRhoChronoH.end(), derivVelRhoD.begin());
}

void CopyD2H(
	thrust::host_vector<Real3> & posRadH, //do not set the size here since you are using push back later
	thrust::host_vector<Real4> & velMasH,
	thrust::host_vector<Real4> & rhoPresMuH,
	const thrust::device_vector<Real3> & posRadD,
	const thrust::device_vector<Real4> & velMasD,
	const thrust::device_vector<Real4> & rhoPresMuD) {
	assert(posRadH.size() == posRadD.size() && "Error! size mismatch host and device" );
	thrust::copy(posRadD.begin(), posRadD.end(), posRadH.begin());
	thrust::copy(velMasD.begin(), velMasD.end(), velMasH.begin());
	thrust::copy(rhoPresMuD.begin(), rhoPresMuD.end(), rhoPresMuH.begin());
}

//void CopyD2D(
//	thrust::host_vector<Real3> & posRadD2, //do not set the size here since you are using push back later
//	thrust::host_vector<Real4> & velMasD2,
//	thrust::host_vector<Real4> & rhoPresMuD2,
//	const thrust::device_vector<Real3> & posRadD,
//	const thrust::device_vector<Real4> & velMasD,
//	const thrust::device_vector<Real4> & rhoPresMuD) {
//	thrust::copy(posRadD.begin(), posRadD.end(), posRadD2.begin());
//	thrust::copy(velMasD.begin(), velMasD.end(), velMasD2.begin());
//	thrust::copy(rhoPresMuD.begin(), rhoPresMuD.end(), rhoPresMuD2.begin());
//}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
int main(int argc, char* argv[]) {
	//****************************************************************************************
	time_t rawtime;
	struct tm * timeinfo;

	//(void) cudaSetDevice(0);

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	printf("Job was submittet at date/time is: %s\n", asctime(timeinfo));

	// ***************************** Create Fluid ********************************************
	//*** Arrays definition
	thrust::host_vector<int3> referenceArray;
	thrust::host_vector<Real3> posRadH; //do not set the size here since you are using push back later
	thrust::host_vector<Real4> velMasH;
	thrust::host_vector<Real4> rhoPresMuH;
	thrust::host_vector<uint> bodyIndex;
	SetupParamsH(paramsH);

	NumberOfObjects numObjects;
	//** default num markers
	int numAllMarkers = 0;

	// initialize fluid particles
	Real sphMarkerMass; // To be initialized in CreateFluidMarkers, and used in other places
	int2 num_fluidOrBoundaryMarkers = CreateFluidMarkers(posRadH, velMasH, rhoPresMuH, bodyIndex, sphMarkerMass);
	referenceArray.push_back(mI3(0, num_fluidOrBoundaryMarkers.x, -1)); //map fluid -1
	numAllMarkers += num_fluidOrBoundaryMarkers.x;
	referenceArray.push_back(
			mI3(numAllMarkers, numAllMarkers + num_fluidOrBoundaryMarkers.y,	0));
	numAllMarkers += num_fluidOrBoundaryMarkers.y;

	// set num objects
	SetNumObjects(numObjects, referenceArray, numAllMarkers);
	assert(posRadH.size() == numObjects.numAllMarkers && "numObjects is not set correctly");
	if (numObjects.numAllMarkers == 0) {
		ClearArraysH(posRadH, velMasH, rhoPresMuH, bodyIndex, referenceArray);
		return 0;
	}
	// ***************************** Create Rigid ********************************************
	ChTimer<double> myTimerTotal;
	ChTimer<double> myTimerStep;
	MySeed(964);

	// Save PovRay post-processing data?
	bool write_povray_data = true;

	myTimerTotal.start();
	ofstream outSimulationInfo;
	outSimulationInfo.open("SimInfo.txt");



	// ***** params
	double dT = 100 * paramsH.dT;
	// ************


	// Create a ChronoENGINE physical system
	ChSystemParallelDVI mphysicalSystem;
	InitializeMbdPhysicalSystem(mphysicalSystem, argc, argv);
	create_system_particles(mphysicalSystem);

	// Add sph data to the physics system
	int startIndexSph = 0;
	AddSphDataToChSystem(mphysicalSystem, startIndexSph, posRadH, velMasH, numObjects);


	// Set gravitational acceleration

	//******************* Irrlicht and driver types **************************
	int numIter = mphysicalSystem.GetSettings()->solver.max_iteration_normal +
			mphysicalSystem.GetSettings()->solver.max_iteration_sliding +
			mphysicalSystem.GetSettings()->solver.max_iteration_spinning +
			mphysicalSystem.GetSettings()->solver.max_iteration_bilateral;
	outSimulationInfo << "****************************************************************************" << endl;
	outSimulationInfo 	<< " dT: " << dT  << " max_iteration: " << numIter <<" muFriction: " << mphysicalSystem.GetParallelThreadNumber() << " number of bodies: " << mphysicalSystem.Get_bodylist()->size() << endl;
	cout			 	<< " dT: " << dT  << " max_iteration: " << numIter <<" muFriction: " << mphysicalSystem.GetParallelThreadNumber() << " number of bodies: " << mphysicalSystem.Get_bodylist()->size() << endl;

	InitializeChronoGraphics(mphysicalSystem, dT);
	//****************************************** Time Loop *************************************

	int counter = -1;
//	while(mphysicalSystem.GetChTime() < timeMove+timePause) //arman modify
	while(false) //arman modify
	{
		myTimerStep.start();
		counter ++;


		int isRunning = DoStepChronoSystem(mphysicalSystem, dT);
		if (isRunning == 0) break;
		printf("*** total number of contacts %d, num bodies %d\n", mphysicalSystem.GetNcontacts(), mphysicalSystem.Get_bodylist()->size());
//		if (write_povray_data) {
//			SavePovFilesMBD(mphysicalSystem, counter);
//		}

		myTimerStep.stop();
		myTimerTotal.stop();
		//****************************************************
	}
	// ***************************************************************************************

	DOUBLEPRECISION ? printf("Double Precision\n") : printf("Single Precision\n");
	printf("********************\n");


	thrust::device_vector<Real3> posRadD = posRadH;
	thrust::device_vector<Real4> velMasD = velMasH;
	thrust::device_vector<Real4> rhoPresMuD = rhoPresMuH;
	thrust::device_vector<uint> bodyIndexD = bodyIndex;
	thrust::device_vector<Real4> derivVelRhoD;
	ResizeMyThrust4(derivVelRhoD, numObjects.numAllMarkers);
//*********************************************** End of Initialization ****************************************
	int stepEnd = int(paramsH.tFinal/paramsH.dT);//1.0e6;//2.4e6;//600000;//2.4e6 * (.02 * paramsH.sizeScale) / currentParamsH.dT ; //1.4e6 * (.02 * paramsH.sizeScale) / currentParamsH.dT ;//0.7e6 * (.02 * paramsH.sizeScale) / currentParamsH.dT ;//0.7e6;//2.5e6; //200000;//10000;//50000;//100000;
	printf("stepEnd %d\n", stepEnd);
	Real realTime = 0;

	SimParams paramsH_B = paramsH;
	paramsH_B.bodyForce3 = mR3(0);
	paramsH_B.gravity = mR3(0);
	paramsH_B.dT = .1 * paramsH.dT;

	printf("\ntimePause %f, numPause %d\n", paramsH.timePause, int(paramsH.timePause/paramsH_B.dT));
	printf("paramsH.timePauseRigidFlex %f, numPauseRigidFlex %d\n\n", paramsH.timePauseRigidFlex, int((paramsH.timePauseRigidFlex-paramsH.timePause)/paramsH.dT + paramsH.timePause/paramsH_B.dT));
	InitSystem(paramsH, numObjects);
	SimParams currentParamsH = paramsH;

	for (int tStep = 0; tStep < stepEnd + 1; tStep++) {
		CpuTimer mCpuTimer;
		mCpuTimer.Start();
		GpuTimer myGpuTimer;
		myGpuTimer.Start();

		PrintToFile(posRadD, velMasD, rhoPresMuD, referenceArray, currentParamsH, realTime, tStep);
		if (realTime <= paramsH.timePause) 	{
			currentParamsH = paramsH_B;
		} else {
			currentParamsH = paramsH;
		}
		InitSystem(currentParamsH, numObjects);

		// ** initialize host mid step data
		thrust::host_vector<Real3> posRadH2(numObjects.numAllMarkers); // Arman: no need for copy
		thrust::host_vector<Real4> velMasH2 = velMasH;
		thrust::host_vector<Real4> rhoPresMuH2(numObjects.numAllMarkers);
		// ** initialize device mid step data
		thrust::device_vector<Real3> posRadD2 = posRadD;
		thrust::device_vector<Real4> velMasD2 = velMasD;
		thrust::device_vector<Real4> rhoPresMuD2 = rhoPresMuD;
		// **
		thrust::device_vector<Real3> vel_XSPH_D;
		ResizeMyThrust3(vel_XSPH_D, numObjects.numAllMarkers);

		FillMyThrust4(derivVelRhoD, mR4(0));
		thrust::host_vector<Real4> derivVelRhoChronoH(numObjects.numAllMarkers);

		ForceSPH(posRadD, velMasD, vel_XSPH_D, rhoPresMuD, bodyIndexD, derivVelRhoD, referenceArray, numObjects, currentParamsH, 0.5 * currentParamsH.dT); //?$ right now, it does not consider paramsH.gravity or other stuff on rigid bodies. they should be applied at rigid body solver
		DoStepChronoSystem(mphysicalSystem, 0.5 * currentParamsH.dT);
		CopyD2H(derivVelRhoChronoH, derivVelRhoD);
		AddChSystemForcesToSphForces(derivVelRhoChronoH, velMasH2, mphysicalSystem, numObjects, startIndexSph, 0.5 * currentParamsH.dT);// assumes velMasH2 constains a copy of velMas in ChSystem right before DoStepDynamics
		CopyH2D(derivVelRhoD, derivVelRhoChronoH);
		UpdateFluid(posRadD2, velMasD2, vel_XSPH_D, rhoPresMuD2, derivVelRhoD, referenceArray, 0.5 * currentParamsH.dT); 	// assumes ...D2 is a copy of ...D
		ApplyBoundarySPH_Markers(posRadD2, rhoPresMuD2, numObjects.numAllMarkers);

		CopyD2H(posRadH2, velMasH2, rhoPresMuH2, posRadD2, velMasD2, rhoPresMuD2);
		UpdateSphDataInChSystem(mphysicalSystem, posRadH2, velMasH2, numObjects, startIndexSph);

		ForceSPH(posRadD2, velMasD2, vel_XSPH_D, rhoPresMuD2, bodyIndexD, derivVelRhoD, referenceArray, numObjects, currentParamsH, currentParamsH.dT); //?$ right now, it does not consider paramsH.gravity or other stuff on rigid bodies. they should be applied at rigid body solver
		DoStepChronoSystem(mphysicalSystem, 0.5 * currentParamsH.dT);
		CopyD2H(derivVelRhoChronoH, derivVelRhoD);
		AddChSystemForcesToSphForces(derivVelRhoChronoH, velMasH2, mphysicalSystem, numObjects, startIndexSph, 0.5 * currentParamsH.dT); // assumes velMasH2 constains a copy of velMas in ChSystem right before DoStepDynamics
		CopyH2D(derivVelRhoD, derivVelRhoChronoH);
		UpdateFluid(posRadD, velMasD, vel_XSPH_D, rhoPresMuD, derivVelRhoD, referenceArray, currentParamsH.dT);
		ApplyBoundarySPH_Markers(posRadD, rhoPresMuD, numObjects.numAllMarkers);

		CopyD2H(posRadH, velMasH, rhoPresMuH, posRadD, velMasD, rhoPresMuD);
		UpdateSphDataInChSystem(mphysicalSystem, posRadH, velMasH, numObjects, startIndexSph);

//		IntegrateSPH(posRadD2, velMasD2, rhoPresMuD2, posRadD, velMasD, vel_XSPH_D, rhoPresMuD, bodyIndexD, derivVelRhoD, referenceArray, numObjects, currentParamsH, 0.5 * currentParamsH.dT);
//		CopyD2H(posRadH2, velMasH2, rhoPresMuH2, posRadD2, velMasD2, rhoPresMuD2);
//		IntegrateSPH(posRadD, velMasD, rhoPresMuD, posRadD2, velMasD2, vel_XSPH_D, rhoPresMuD2, bodyIndexD, derivVelRhoD, referenceArray, numObjects, currentParamsH, currentParamsH.dT);
//		CopyD2H(posRadH, velMasH, rhoPresMuH, posRadD, velMasD, rhoPresMuD);

		ClearArraysH(posRadH2, velMasH2, rhoPresMuH2);
		ClearMyThrustR3(posRadD2);
		ClearMyThrustR4(velMasD2);
		ClearMyThrustR4(rhoPresMuD2);
		ClearMyThrustR3(vel_XSPH_D);


		mCpuTimer.Stop();
		myGpuTimer.Stop();
		if (tStep % 2 == 0) {
			printf("step: %d, realTime: %f, step Time (CUDA): %f, step Time (CPU): %f\n ", tStep, realTime, (Real)myGpuTimer.Elapsed(), 1000 * mCpuTimer.Elapsed());
		}
//		if (write_povray_data) {
//			SavePovFilesMBD(mphysicalSystem, tStep);
//		}
		fflush(stdout);
		realTime += currentParamsH.dT;
	}

	ClearArraysH(posRadH, velMasH, rhoPresMuH, bodyIndex, referenceArray);
	ClearMyThrustR3(posRadD);
	ClearMyThrustR4(velMasD);
	ClearMyThrustR4(rhoPresMuD);
	ClearMyThrustU1(bodyIndexD);
	return 0;
}
