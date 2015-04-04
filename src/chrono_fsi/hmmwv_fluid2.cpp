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

// General Includes
#include <iostream>
#include <fstream>
#include <string>
#include <limits.h>
#include <vector>
#include <ctime>
#include <assert.h>

// SPH includes
#include "MyStructs.cuh" //just for SimParams
#include "collideSphereSphere.cuh"
#include "printToFile.cuh"
#include "custom_cutil_math.h"
#include "SPHCudaUtils.h"

// Chrono Parallel Includes
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/lcp/ChLcpSystemDescriptorParallel.h"

// Chrono Vehicle Include
#include "VehicleProperties.h"
#include "chrono_utils/ChUtilsVehicle.h"
#include "chrono_utils/ChUtilsGeometry.h"
#include "chrono_utils/ChUtilsCreators.h"
#include "chrono_utils/ChUtilsGenerators.h"
#include "chrono_utils/ChUtilsInputOutput.h"

// FSI Interface Includes
#include "hmmwvParams.h"
//#include "BallDropParams.h"
#include "SphInterface.h"
#include "InitializeSphMarkers.h"

// Irrlicht Include
#include "unit_IRRLICHT/ChIrrApp.h"

// Chrono namespaces
using namespace chrono;
using namespace chrono::collision;

using std::cout;
using std::endl;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

//*************************************************************
// Define Graphics
#define irrlichtVisualization false
#if irrlichtVisualization
std::shared_ptr<ChIrrApp> application;
#endif

#ifdef CHRONO_PARALLEL_HAS_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
opengl::ChOpenGLWindow &gl_window = opengl::ChOpenGLWindow::getInstance();
#endif
//*************************************************************
// Define General variables
SimParams paramsH;
//*************************************************************

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
	// Desired number of OpenMP threads (will be clamped to maximum available)
	int threads = 1;
	// Perform dynamic tuning of number of threads?
	bool thread_tuning = true;

//	uint max_iteration = 20;//10000;
	int max_iteration_normal = 0;
	int max_iteration_sliding = 200;
	int max_iteration_spinning = 0;
	int max_iteration_bilateral = 100;

	// SetArgumentsForMbdFromInput(argc, argv, threads, max_iteration);
	//******************** OMP settings **************

	// ----------------------
	// Set number of threads.
	// ----------------------

	int max_threads = mphysicalSystem.GetParallelThreadNumber();
	if (threads > max_threads)
	  threads = max_threads;
	mphysicalSystem.SetParallelThreadNumber(threads);
	omp_set_num_threads(threads);
		cout << "Using " << threads << " threads" << endl;
	//************************************************
	double tolerance = 1e-3; // Arman, move it to paramsH
	double collisionEnvelop = .04 * paramsH.HSML;
	mphysicalSystem.Set_G_acc(ChVector<>(paramsH.gravity.x, paramsH.gravity.y, paramsH.gravity.z));

  // ---------------------
  // Edit system settings.
  // ---------------------

  	mphysicalSystem.GetSettings()->solver.solver_mode = SLIDING; //NORMAL, SPINNING
	mphysicalSystem.GetSettings()->solver.max_iteration_normal = max_iteration_normal; 		// max_iteration / 3
	mphysicalSystem.GetSettings()->solver.max_iteration_sliding = max_iteration_sliding; 	// max_iteration / 3
	mphysicalSystem.GetSettings()->solver.max_iteration_spinning = max_iteration_spinning; 	// 0
	mphysicalSystem.GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;		// max_iteration / 3
	mphysicalSystem.GetSettings()->solver.tolerance = tolerance; // Arman : define it in the params.h file
	mphysicalSystem.GetSettings()->solver.alpha = 0;  //Arman, find out what is this
	mphysicalSystem.GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
	mphysicalSystem.ChangeSolverType(APGD);  //Arman check this APGD APGDBLAZE
	mphysicalSystem.GetSettings()->collision.narrowphase_algorithm = NARROWPHASE_HYBRID_MPR;

	mphysicalSystem.GetSettings()->collision.collision_envelope = collisionEnvelop;
	mphysicalSystem.GetSettings()->collision.bins_per_axis = mI3(10, 10, 10); //Arman check
}

double CreateGranularBed(ChSystem* mphysicalSystem) {
  // Create a material
  ChSharedPtr<ChMaterialSurface> mat_g(new ChMaterialSurface);
  mat_g->SetFriction(mu_g);

  // Create a particle generator and a mixture entirely made out of spheres
  utils::Generator gen(mphysicalSystem);
  utils::MixtureIngredientPtr& m1 = gen.AddMixtureIngredient(utils::SPHERE, 1.0);
  m1->setDefaultMaterialDVI(mat_g);
  m1->setDefaultDensity(rho_g);
  m1->setDefaultSize(r_g);

  // Set starting value for body identifiers
  gen.setBodyIdentifier(Id_g);

  // Create particles in layers until reaching the desired number of particles
  double r = 1.01 * r_g;
  ChVector<> hdims(hdimX - r, hdimY - r, 0);
  ChVector<> center(0, 0, 2 * r);

  while (gen.getTotalNumBodies() < num_particles) {
    gen.createObjectsBox(utils::POISSON_DISK, 2 * r, center, hdims);
    center.z += 2 * r;
  }

  return center.z;
}

void CreateMbdPhysicalSystemObjects(ChSystemParallelDVI& mphysicalSystem) {

// 	// -----------------------------------------
// 	// Create and initialize the vehicle systems
// 	// -----------------------------------------
// 	MyVehicle vehicle(&mphysicalSystem);

// 	// --------------------------------------------------------
// 	// Create the ground body and set contact geometry
// 	// --------------------------------------------------------
// 	double hdimX = 100;
// 	double hdimY = 100;
// 	double hdimZ = 5;

// 	double hdimX3 = hdimX/3;
// //	double hdimY3 = hdimY/3;
// 	hdimX = 3 * hdimX3;
// //	hdimY = 3 * hdimY3;

// 	// basin info
// 	double depth = 10;
// 	double slope = CH_C_PI / 6;
// 	double bottomWidth = hdimX3 - depth / tan(slope); 	// for a 45 degree slope
// 	double inclinedWidth = depth / sin(slope); // for a 45 degree slope

// 	ChSharedPtr<ChBody> ground = ChSharedPtr<ChBody>(new ChBody(new collision::ChCollisionModelParallel));
// 	ground->SetIdentifier(-1);
// 	ground->SetBodyFixed(true);
// 	ground->SetCollide(true);
// 	ground->SetPos(ChVector<>(0, 0, -hdimZ));

// 	ground->GetMaterialSurface()->SetFriction(0.8f);

// 	ground->GetCollisionModel()->ClearModel();
// //	ground->GetCollisionModel()->AddBox(hdimX, hdimY, hdimZ);
// 	utils::AddBoxGeometry(ground.get_ptr(), ChVector<>(hdimX3, hdimY, hdimZ), ChVector<>(-hdimX3, 0, 0));	// beginning third
// 	utils::AddBoxGeometry(ground.get_ptr(), ChVector<>(hdimX3, hdimY, hdimZ), ChVector<>(hdimX3, 0, 0));	// end third
// 	// basin
// 	utils::AddBoxGeometry(ground.get_ptr(), ChVector<>(bottomWidth, hdimY, hdimZ), ChVector<>(0, 0, -depth));	// middle third
// 	utils::AddBoxGeometry(ground.get_ptr(), ChVector<>(inclinedWidth , hdimY, hdimZ),
// 			ChVector<>(-hdimX3 - inclinedWidth * cos(slope), 0, -inclinedWidth*sin(slope)),
// 			Q_from_AngAxis(slope, ChVector(0, 1, 0)));	// middle third
// 	utils::AddBoxGeometry(ground.get_ptr(), ChVector<>(inclinedWidth , hdimY, hdimZ),
// 			ChVector<>(hdimX3 + inclinedWidth * cos(slope), 0, -inclinedWidth*sin(slope)),
// 			Q_from_AngAxis(-slope, ChVector(0, 1, 0)));	// middle third


// 	ground->GetCollisionModel()->BuildModel();

// 	ChSharedPtr<ChBoxShape> box_ground(new ChBoxShape);
// 	box_ground->GetBoxGeometry().Size = ChVector<>(hdimX, hdimY, hdimZ);
// 	ground->AddAsset(box_ground);

// 	mphysicalSystem->AddBody(ground);
















//	// ball drop
//	Real3 domainCenter = 0.5 * (paramsH.cMin + paramsH.cMax);
//	Real3 ellipsoidSize = .3 * mR3(10 * paramsH.HSML, 5 * paramsH.HSML, 7 * paramsH.HSML);
//	ChVector<> size = ChVector<>(ellipsoidSize.x, ellipsoidSize.y, ellipsoidSize.z);
//
//	double density = .5*paramsH.rho0;  // TODO : Arman : Density for now is set to water density
//	double muFriction = .1;
//
//	// NOTE: mass properties and shapes are all for sphere
//	double volume = utils::CalcSphereVolume(size.x);
//	ChVector<> gyration = utils::CalcSphereGyration(size.x).Get_Diag();
//	double mass = density * volume;
//
//
//	//**************** bin and ship
//	// Create a common material
//	ChSharedPtr<ChMaterialSurface> mat_g(new ChMaterialSurface);
//	mat_g->SetFriction(muFriction);
//	mat_g->SetCohesion(0);
//	mat_g->SetCompliance(0.0);
//	mat_g->SetComplianceT(0.0);
//	mat_g->SetDampingF(0.2);
//
//	const ChVector<> pos = ChVector<>(domainCenter.x, domainCenter.y + paramsH.cMax.y, domainCenter.z);
//	const ChQuaternion<> rot = ChQuaternion<>(1, 0, 0, 0);
//
//	ChSharedBodyPtr body;
//	body = ChSharedBodyPtr(new  ChBody(new ChCollisionModelParallel));
//	body->SetMaterialSurface(mat_g);
//	body->SetPos(pos);
//	body->SetRot(rot);
//	body->SetCollide(true);
//	body->SetBodyFixed(false);
//    body->SetMass(mass);
//    body->SetInertiaXX(mass * gyration);
//
//	body->GetCollisionModel()->ClearModel();
//	utils::AddSphereGeometry(body.get_ptr(), size.x);				// O
//	body->GetCollisionModel()->BuildModel();
//	mphysicalSystem.AddBody(body);
//	// ****************** create boxes around the fluid domain
//	ChSharedPtr<ChMaterialSurface> mat(new ChMaterialSurface);
//	mat->SetFriction(.5);
//	mat->SetDampingF(0.2f);
//	int binId = 2;
//
//	ChSharedBodyPtr bin;
//	Real3 hdim = paramsH.boxDims;
//	double hthick = 1 * paramsH.HSML;
//
//	bin = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
//	bin->SetMaterialSurface(mat);
//	bin->SetMass(1);
//	bin->SetPos(ChVector<>(domainCenter.x, domainCenter.y, domainCenter.z));
//	bin->SetRot(ChQuaternion<>(1, 0, 0, 0));
//	bin->SetCollide(true);
//	bin->SetBodyFixed(true);
//
//	bin->GetCollisionModel()->ClearModel();
//	utils::AddBoxGeometry(bin.get_ptr(), 0.5 * ChVector<>(hdim.x + 2 * hthick, hthick, hdim.z + 2 * hthick), ChVector<>(0, -0.5 * hdim.y - 0.5 * hthick, 0));	//bottom wall
//	bin->GetCollisionModel()->BuildModel();
//
//	mphysicalSystem.AddBody(bin);


















	// original hmmwv
  // -------------------
  // Create the terrain.
  // -------------------

  // Ground body
  ChSharedPtr<ChBody> ground = ChSharedPtr<ChBody>(new ChBody(new collision::ChCollisionModelParallel));
  ground->SetIdentifier(-1);
  ground->SetBodyFixed(true);
  ground->SetCollide(true);

  ground->GetMaterialSurface()->SetFriction(mu_g);

  ground->GetCollisionModel()->ClearModel();

  printf("b1\n");

  // Bottom box
  utils::AddBoxGeometry(ground.get_ptr(), ChVector<>(hdimX, hdimY, hthick), ChVector<>(0, 0, -hthick),
                        ChQuaternion<>(1, 0, 0, 0), true);
  if (terrain_type == GRANULAR) {
    // Front box
    utils::AddBoxGeometry(ground.get_ptr(), ChVector<>(hthick, hdimY, hdimZ + hthick),
                          ChVector<>(hdimX + hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), visible_walls);
    // Rear box
    utils::AddBoxGeometry(ground.get_ptr(), ChVector<>(hthick, hdimY, hdimZ + hthick),
                          ChVector<>(-hdimX - hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), visible_walls);
    // Left box
    utils::AddBoxGeometry(ground.get_ptr(), ChVector<>(hdimX, hthick, hdimZ + hthick),
                          ChVector<>(0, hdimY + hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), visible_walls);
    // Right box
    utils::AddBoxGeometry(ground.get_ptr(), ChVector<>(hdimX, hthick, hdimZ + hthick),
                          ChVector<>(0, -hdimY - hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), visible_walls);
  }

  printf("b2\n");
  ground->GetCollisionModel()->BuildModel();

  mphysicalSystem.AddBody(ground);

  // Create the granular material.
  double vertical_offset = 0;

  if (terrain_type == GRANULAR) {
    vertical_offset = CreateGranularBed(&mphysicalSystem);
  }

  printf("b3\n");

  // -----------------------------------------
  // Create and initialize the vehicle system.
  // -----------------------------------------

  utils::VehicleSystem* vehicle;
  utils::TireContactCallback* tire_cb;
  printf("b4\n");


  // Create the vehicle assembly and the callback object for tire contact
  // according to the specified type of tire/wheel.
  switch (wheel_type) {
    case CYLINDRICAL: {

    	  printf("b5\n");



      vehicle = new utils::VehicleSystem(&mphysicalSystem, vehicle_file_cyl, simplepowertrain_file);
      printf("b6\n");
      tire_cb = new MyCylindricalTire();
      printf("b6\n");
    } break;
    case LUGGED: {
      vehicle = new utils::VehicleSystem(&mphysicalSystem, vehicle_file_lug, simplepowertrain_file);
      tire_cb = new MyLuggedTire();
    } break;
  }

  printf("b6\n");
  vehicle->SetTireContactCallback(tire_cb);

  // Set the callback object for driver inputs. Pass the hold time as a delay in
  // generating driver inputs.
  MyDriverInputs driver_cb(time_hold);
  vehicle->SetDriverInputsCallback(&driver_cb);
  printf("b5\n");

  // Initially, fix the chassis (will be released after time_hold).
  vehicle->GetVehicle()->GetChassis()->SetBodyFixed(true);

  // Initialize the vehicle at a height above the terrain.
  vehicle->Initialize(initLoc + ChVector<>(0, 0, vertical_offset), initRot);

  printf("b6\n");

}






void InitializeChronoGraphics(ChSystemParallelDVI& mphysicalSystem) {
	Real3 domainCenter = 0.5 * (paramsH.cMin + paramsH.cMax);
	ChVector<> CameraLocation = ChVector<>(2 * paramsH.cMax.x, 2 * paramsH.cMax.y, 2 * paramsH.cMax.z);
	ChVector<> CameraLookAt = ChVector<>(domainCenter.x, domainCenter.y, domainCenter.z);

#ifdef CHRONO_PARALLEL_HAS_OPENGL
	gl_window.Initialize(1280, 720, "HMMWV", &mphysicalSystem);
	gl_window.SetCamera(ChVector<>(0, -10, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));
	gl_window.SetRenderMode(opengl::WIREFRAME);

   // Uncomment the following two lines for the OpenGL manager to automatically
   // run the simulation in an infinite loop.
   //gl_window.StartDrawLoop(time_step);
   //return 0;
#endif

#if irrlichtVisualization
	// Create the Irrlicht visualization (open the Irrlicht device,
	// bind a simple user interface, etc. etc.)
   application = std::shared_ptr<ChIrrApp>(new ChIrrApp(&mphysicalSystem, L"Bricks test",core::dimension2d<u32>(800,600),false, true));
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
#endif
}

int DoStepChronoSystem(ChSystemParallelDVI& mphysicalSystem, Real dT) {
	Real3 domainCenter = 0.5 * (paramsH.cMin + paramsH.cMax);
#if irrlichtVisualization
		if ( !(application->GetDevice()->run()) ) return 0;
		application->SetTimestep(dT);
		application->GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));
		ChIrrTools::drawGrid(application->GetVideoDriver(), 2 * paramsH.HSML, 2 * paramsH.HSML, 50, 50,
			ChCoordsys<>(ChVector<>(domainCenter.x, paramsH.worldOrigin.y, domainCenter.z), Q_from_AngAxis(CH_C_PI/2,VECT_X)), video::SColor(50,90,90,150),true);
		application->DrawAll();
		application->DoStep();
		application->GetVideoDriver()->endScene();
#else
#ifdef CHRONO_PARALLEL_HAS_OPENGL
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
	int2 num_fluidOrBoundaryMarkers = CreateFluidMarkers(posRadH, velMasH, rhoPresMuH, bodyIndex, paramsH, sphMarkerMass);
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

	// Save PovRay post-processing data?
	bool write_povray_data = true;

	myTimerTotal.start();
	std::ofstream outSimulationInfo;
	outSimulationInfo.open("SimInfo.txt");

	// Create a ChronoENGINE physical system
	ChSystemParallelDVI mphysicalSystem;
	printf("a1\n");
	InitializeMbdPhysicalSystem(mphysicalSystem, argc, argv);
	printf("a2\n");
	CreateMbdPhysicalSystemObjects(mphysicalSystem);
	printf("a3\n");

	// Add sph data to the physics system
	int startIndexSph = 0;
	printf("(1) num bodies %d \n", mphysicalSystem.Get_bodylist()->size());
	AddSphDataToChSystem(mphysicalSystem, startIndexSph, posRadH, velMasH, paramsH, numObjects);
	printf("(2) num bodies %d \n", mphysicalSystem.Get_bodylist()->size());
	// Set gravitational acceleration

	//******************* Irrlicht and driver types **************************
	double dTc = .5 * paramsH.dT;   //Arman: What is this? fix it!
	int numIter = mphysicalSystem.GetSettings()->solver.max_iteration_normal +
			mphysicalSystem.GetSettings()->solver.max_iteration_sliding +
			mphysicalSystem.GetSettings()->solver.max_iteration_spinning +
			mphysicalSystem.GetSettings()->solver.max_iteration_bilateral;
	outSimulationInfo << "****************************************************************************" << std::endl;
	outSimulationInfo 	<< " dT: " << dTc  << " max_iteration: " << numIter <<" muFriction: " << mphysicalSystem.GetParallelThreadNumber() << " number of bodies: " << mphysicalSystem.Get_bodylist()->size() << std::endl;
	std::cout			 	<< " dT: " << dTc  << " max_iteration: " << numIter <<" muFriction: " << mphysicalSystem.GetParallelThreadNumber() << " number of bodies: " << mphysicalSystem.Get_bodylist()->size() << std::endl;

	InitializeChronoGraphics(mphysicalSystem);

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
	paramsH_B.dT = paramsH.dT;

	printf("\ntimePause %f, numPause %d\n", paramsH.timePause, int(paramsH.timePause/paramsH_B.dT));
	printf("paramsH.timePauseRigidFlex %f, numPauseRigidFlex %d\n\n", paramsH.timePauseRigidFlex, int((paramsH.timePauseRigidFlex-paramsH.timePause)/paramsH.dT + paramsH.timePause/paramsH_B.dT));
	InitSystem(paramsH, numObjects);
	SimParams currentParamsH = paramsH;

	for (int tStep = 0; tStep < stepEnd + 1; tStep++) {
		CpuTimer mCpuTimer;
		mCpuTimer.Start();
		GpuTimer myGpuTimer;
		myGpuTimer.Start();

//		CopySys2D(posRadD, mphysicalSystem, numObjects, startIndexSph);
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

		// ****************** RK2: 1/2
		// ForceSPH(posRadD, velMasD, vel_XSPH_D, rhoPresMuD, bodyIndexD, derivVelRhoD, referenceArray, numObjects, currentParamsH, 0.5 * currentParamsH.dT); //?$ right now, it does not consider paramsH.gravity or other stuff on rigid bodies. they should be applied at rigid body solver
		DoStepChronoSystem(mphysicalSystem, 0.5 * currentParamsH.dT);
		// CopyD2H(derivVelRhoChronoH, derivVelRhoD);
		// AddChSystemForcesToSphForces(derivVelRhoChronoH, velMasH2, mphysicalSystem, numObjects, startIndexSph, 0.5 * currentParamsH.dT);// assumes velMasH2 constains a copy of velMas in ChSystem right before DoStepDynamics
		// CopyH2D(derivVelRhoD, derivVelRhoChronoH);
		// UpdateFluid(posRadD2, velMasD2, vel_XSPH_D, rhoPresMuD2, derivVelRhoD, referenceArray, 0.5 * currentParamsH.dT); 	// assumes ...D2 is a copy of ...D
		// ApplyBoundarySPH_Markers(posRadD2, rhoPresMuD2, numObjects.numAllMarkers);

		// CopyD2H(posRadH2, velMasH2, rhoPresMuH2, posRadD2, velMasD2, rhoPresMuD2);
		// UpdateSphDataInChSystem(mphysicalSystem, posRadH2, velMasH2, numObjects, startIndexSph);

		// ****************** RK2: 2/2
		// ForceSPH(posRadD2, velMasD2, vel_XSPH_D, rhoPresMuD2, bodyIndexD, derivVelRhoD, referenceArray, numObjects, currentParamsH, currentParamsH.dT); //?$ right now, it does not consider paramsH.gravity or other stuff on rigid bodies. they should be applied at rigid body solver
		DoStepChronoSystem(mphysicalSystem, 0.5 * currentParamsH.dT);
		// CopyD2H(derivVelRhoChronoH, derivVelRhoD);
		// AddChSystemForcesToSphForces(derivVelRhoChronoH, velMasH2, mphysicalSystem, numObjects, startIndexSph, 0.5 * currentParamsH.dT); // assumes velMasH2 constains a copy of velMas in ChSystem right before DoStepDynamics
		// CopyH2D(derivVelRhoD, derivVelRhoChronoH);
		// UpdateFluid(posRadD, velMasD, vel_XSPH_D, rhoPresMuD, derivVelRhoD, referenceArray, currentParamsH.dT);
		// ApplyBoundarySPH_Markers(posRadD, rhoPresMuD, numObjects.numAllMarkers);

		// CopyD2H(posRadH, velMasH, rhoPresMuH, posRadD, velMasD, rhoPresMuD);
		// UpdateSphDataInChSystem(mphysicalSystem, posRadH, velMasH, numObjects, startIndexSph);

		// ****************** End RK2

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
