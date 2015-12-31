///////////////////////////////////////////////////////////////////////////////
//	main.cpp
//	Reads the initializes the particles, either from file or inside the code
//
//	Related Files: collideSphereSphere.cu, collideSphereSphere.cuh
//	Input File:		initializer.txt (optional: if initialize from file)
//					This file contains the sph particles specifications. The description
//					reads the number of particles first. The each line provides the
//					properties of one SPH particl:
//					position(x,y,z), radius, velocity(x,y,z), mass, \rho, pressure, mu,
// particle_type(rigid
// or fluid)
//
//	Created by Arman Pazouki
///////////////////////////////////////////////////////////////////////////////

// note: this is the original fsi_hmmwv model. uses RK2, an specific coupling, and density re_initializaiton.

// General Includes
#include <iostream>
#include <fstream>
#include <string>
#include <limits.h>
#include <vector>
#include <ctime>
#include <assert.h>
#include <stdlib.h>  // system

// SPH includes
#include "chrono_fsi/MyStructs.cuh"  //just for SimParams
#include "chrono_fsi/collideSphereSphere.cuh"
#include "chrono_fsi/printToFile.cuh"
#include "chrono_fsi/custom_cutil_math.h"
#include "chrono_fsi/SPHCudaUtils.h"
#include "chrono_fsi/checkPointReduced.h"

// Chrono Parallel Includes
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/lcp/ChLcpSystemDescriptorParallel.h"

// Chrono Vehicle Include
#include "chrono_fsi/VehicleExtraProperties.h"
#include "chrono_vehicle/ChVehicleModelData.h"

//#include "chrono_utils/ChUtilsVehicle.h"
#include "utils/ChUtilsGeometry.h"
#include "utils/ChUtilsCreators.h"
#include "utils/ChUtilsGenerators.h"
#include "utils/ChUtilsInputOutput.h"

// Chrono general utils
#include "core/ChFileutils.h"
#include <core/ChTransform.h>  //transform acc from GF to LF for post process

//#include "BallDropParams.h"
#include "chrono_fsi/SphInterface.h"
#include "chrono_fsi/InitializeSphMarkers.h"
#include "chrono_fsi/FSI_Integrate.h"

// FSI Interface Includes
#include "params_test_fsi_invertedPendulum.h"  //SetupParamsH()

#define haveFluid true
#define haveVehicle false

// Chrono namespaces
using namespace chrono;
using namespace chrono::collision;

using std::cout;
using std::endl;

// Define General variables
SimParams paramsH;

// =============================================================================
// Define Graphics
#define irrlichtVisualization false

#if irrlichtVisualization

// Irrlicht Include
#include "unit_IRRLICHT/ChIrrApp.h"

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

std::shared_ptr<ChIrrApp> application;
#endif

// =============================================================================
void SetArgumentsForMbdFromInput(int argc, char* argv[], int& threads,
		int& max_iteration_sliding, int& max_iteration_bilateral) {
	if (argc > 1) {
		const char* text = argv[1];
		threads = atoi(text);
	}
	if (argc > 2) {
		const char* text = argv[2];
		max_iteration_sliding = atoi(text);
	}
	if (argc > 3) {
		const char* text = argv[3];
		max_iteration_bilateral = atoi(text);
	}
}
// =============================================================================
void SetArgumentsForMbdFromInput(int argc, char* argv[], int& threads,
		int& max_iteration_normal, int& max_iteration_sliding,
		int& max_iteration_spinning, int& max_iteration_bilateral) {
	if (argc > 1) {
		const char* text = argv[1];
		threads = atoi(text);
	}
	if (argc > 2) {
		const char* text = argv[2];
		max_iteration_normal = atoi(text);
	}
	if (argc > 3) {
		const char* text = argv[3];
		max_iteration_sliding = atoi(text);
	}
	if (argc > 4) {
		const char* text = argv[4];
		max_iteration_spinning = atoi(text);
	}
	if (argc > 5) {
		const char* text = argv[5];
		max_iteration_bilateral = atoi(text);
	}
}
// =============================================================================

void InitializeMbdPhysicalSystem(ChSystemParallelDVI& mphysicalSystem, int argc,
		char* argv[]) {
	// Desired number of OpenMP threads (will be clamped to maximum available)
	int threads = 1;
	// Perform dynamic tuning of number of threads?
	bool thread_tuning = true;

	//	uint max_iteration = 20;//10000;
	int max_iteration_normal = 0;
	int max_iteration_sliding = 200;
	int max_iteration_spinning = 0;
	int max_iteration_bilateral = 100;

	// ----------------------
	// Set params from input
	// ----------------------

	SetArgumentsForMbdFromInput(argc, argv, threads, max_iteration_sliding,
			max_iteration_bilateral);

	// ----------------------
	// Set number of threads.
	// ----------------------

	//  omp_get_num_procs();
	int max_threads = omp_get_num_procs();
	if (threads > max_threads)
		threads = max_threads;
	mphysicalSystem.SetParallelThreadNumber(threads);
	omp_set_num_threads(threads);
	cout << "Using " << threads << " threads" << endl;

	mphysicalSystem.GetSettings()->perform_thread_tuning = thread_tuning;
	mphysicalSystem.GetSettings()->min_threads = max(1, threads / 2);
	mphysicalSystem.GetSettings()->max_threads = int(3.0 * threads / 2);

	// ---------------------
	// Print the rest of parameters
	// ---------------------

	simParams << endl << " number of threads: " << threads << endl
			<< " max_iteration_normal: " << max_iteration_normal << endl
			<< " max_iteration_sliding: " << max_iteration_sliding << endl
			<< " max_iteration_spinning: " << max_iteration_spinning << endl
			<< " max_iteration_bilateral: " << max_iteration_bilateral << endl
			<< endl;

	// ---------------------
	// Edit mphysicalSystem settings.
	// ---------------------

	double tolerance = 0.1;  // 1e-3;  // Arman, move it to paramsH
	// double collisionEnvelop = 0.04 * paramsH.HSML;
	mphysicalSystem.Set_G_acc(
			ChVector<>(paramsH.gravity.x, paramsH.gravity.y,
					paramsH.gravity.z));

	mphysicalSystem.GetSettings()->solver.solver_mode = SLIDING; // NORMAL, SPINNING
	mphysicalSystem.GetSettings()->solver.max_iteration_normal =
			max_iteration_normal;        // max_iteration / 3
	mphysicalSystem.GetSettings()->solver.max_iteration_sliding =
			max_iteration_sliding;      // max_iteration / 3
	mphysicalSystem.GetSettings()->solver.max_iteration_spinning =
			max_iteration_spinning;    // 0
	mphysicalSystem.GetSettings()->solver.max_iteration_bilateral =
			max_iteration_bilateral;  // max_iteration / 3
	mphysicalSystem.GetSettings()->solver.use_full_inertia_tensor = true;
	mphysicalSystem.GetSettings()->solver.tolerance = tolerance;
	mphysicalSystem.GetSettings()->solver.alpha = 0; // Arman, find out what is this
	mphysicalSystem.GetSettings()->solver.contact_recovery_speed =
			contact_recovery_speed;
	mphysicalSystem.ChangeSolverType(APGD);  // Arman check this APGD APGDBLAZE
	//  mphysicalSystem.GetSettings()->collision.narrowphase_algorithm = NARROWPHASE_HYBRID_MPR;

	//    mphysicalSystem.GetSettings()->collision.collision_envelope = collisionEnvelop;   // global collisionEnvelop
	//    does not work. Maybe due to sph-tire size mismatch
	mphysicalSystem.GetSettings()->collision.bins_per_axis = _make_int3(40, 40,
			40);  // Arman check
}
// =============================================================================

// Arman you still need local position of bce markers
void CreateMbdPhysicalSystemObjects(ChSystemParallelDVI& mphysicalSystem,
		thrust::host_vector<Real3>& posRadH, // do not set the size here since you are using push back later
		thrust::host_vector<Real3>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH,
		thrust::host_vector<uint>& bodyIndex,
		std::vector<ChSharedPtr<ChBody> >& FSI_Bodies,
		thrust::host_vector<::int4>& referenceArray,
		NumberOfObjects& numObjects, const SimParams& paramsH) {
	// Set common material Properties
	mat_g->SetFriction(0.8);
	mat_g->SetCohesion(0);
	mat_g->SetCompliance(0.0);
	mat_g->SetComplianceT(0.0);
	mat_g->SetDampingF(0.2);

	// Ground body
	ChSharedPtr<ChBody> ground = ChSharedPtr<ChBody>(
			new ChBody(new collision::ChCollisionModelParallel));
	ground->SetIdentifier(-1);
	ground->SetBodyFixed(true);
	ground->SetCollide(true);

	ground->SetMaterialSurface(mat_g);

	ground->GetCollisionModel()->ClearModel();

	// Bottom box
	double hdimSide = hdimX / 4.0;
	double midSecDim = hdimX - 2 * hdimSide;

	// basin info
	double phi = CH_C_PI / 2.5; //CH_C_PI / 9;
	double bottomWidth = midSecDim - basinDepth / tan(phi); // for a 45 degree slope
	double bottomBuffer = .4 * bottomWidth;

	double inclinedWidth = 0.5 * basinDepth / sin(phi); // for a 45 degree slope

	double smallBuffer = .7 * hthick;
	double x1I = -midSecDim + inclinedWidth * cos(phi) - hthick * sin(phi)
			- smallBuffer;
	double zI = -inclinedWidth * sin(phi) - hthick * cos(phi);
	double x2I = midSecDim - inclinedWidth * cos(phi) + hthick * sin(phi)
			+ smallBuffer;

	if (!initializeFluidFromFile) {

#if haveFluid
		// basin
		AddBoxBce(posRadH, velMasH, rhoPresMuH, referenceArray, numObjects,
				paramsH, ground,
				ChVector<>(bottomWidth + bottomBuffer, hdimY, hthick),
				ChVector<>(0, 0, -basinDepth - hthick),
				ChQuaternion<>(1, 0, 0, 0));
		// slope 1
		AddBoxBce(posRadH, velMasH, rhoPresMuH, referenceArray, numObjects,
				paramsH, ground,
				ChVector<>(inclinedWidth, hdimY, hthick),
				ChVector<>(x1I, 0, zI),
				Q_from_AngAxis(phi, ChVector<>(0, 1, 0)));

		// slope 2
		AddBoxBce(posRadH, velMasH, rhoPresMuH, referenceArray, numObjects,
				paramsH, ground,
				ChVector<>(inclinedWidth, hdimY, hthick),
				ChVector<>(x2I, 0, zI),
				Q_from_AngAxis(-phi, ChVector<>(0, 1, 0)));

#endif
	}

	ground->GetCollisionModel()->BuildModel();

	mphysicalSystem.AddBody(ground);

	// version 0, create one cylinder // note: rigid body initialization should come after boundary initialization

	// **************************
	// **** Test angular velocity
	// **************************
	//
	double cyl_length1 = 0.2;
	double cyl_radius1 = .55;
	ChVector<> cyl_pos = ChVector<>(0, 0, 0);
	ChQuaternion<> cyl_rot = chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_X);

	CreateCylinderFSI(posRadH, velMasH, rhoPresMuH, referenceArray,
			mphysicalSystem, FSI_Bodies, numObjects, paramsH,
			cyl_radius1, cyl_length1, mat_g, 0.5 * paramsH.rho0, cyl_pos,
			cyl_rot);
	int bid_1 = FSI_Bodies.size();

	double cyl_length2 = 1.0;
	double cyl_radius2 = 0.25;
	CreateCylinderFSI(posRadH, velMasH, rhoPresMuH, referenceArray,
			mphysicalSystem, FSI_Bodies, numObjects, paramsH,
			0.5 * cyl_radius2, cyl_length2, mat_g, 0.5 * paramsH.rho0,
			ChVector<>(0, 0, .5 * (cyl_length2 + 2 * cyl_length1)), cyl_rot);
	int bid_2 = FSI_Bodies.size() - 1;

	double cyl_length3 = 1.0;
	double cyl_radius3 = 0.25;
	CreateCylinderFSI(posRadH, velMasH, rhoPresMuH, referenceArray,
			mphysicalSystem, FSI_Bodies, numObjects, paramsH,
			0.5 * cyl_radius2, cyl_length2, mat_g, 0.5 * paramsH.rho0,
			ChVector<>(0, 0,
					cyl_length2 + .5 * (cyl_length3 + 4 * cyl_length1)),
			cyl_rot);
	int bid_3 = FSI_Bodies.size() - 1;

	// Revolute joint rods
	// ----------------------------
	ChSharedPtr<ChLinkLockRevolute> revolute(new ChLinkLockRevolute);
	revolute->Initialize(FSI_Bodies[bid_2], FSI_Bodies[bid_3],
			ChCoordsys<>(ChVector<>(0, 0, cyl_length2 + cyl_length1),
					chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_X)));
	mphysicalSystem.AddLink(revolute);
}

// =============================================================================

void SavePovFilesMBD(ChSystemParallelDVI& mphysicalSystem, int tStep,
		double mTime) {
	static double exec_time;
	int out_steps = std::ceil((1.0 / paramsH.dT) / out_fps);
	exec_time += mphysicalSystem.GetTimerStep();
	int num_contacts = mphysicalSystem.GetNcontacts();

	static int out_frame = 0;

	// If enabled, output data for PovRay postprocessing.
	if (povray_output && tStep % out_steps == 0) {
		if (tStep / out_steps == 0) {
			const std::string rmCmd = std::string("rm ") + pov_dir_mbd
					+ std::string("/*.dat");
			system(rmCmd.c_str());
		}

		char filename[100];
		sprintf(filename, "%s/data_%03d.dat", pov_dir_mbd.c_str(),
				out_frame + 1);
		utils::WriteShapesPovray(&mphysicalSystem, filename);

		cout << "------------ Output frame:   " << out_frame + 1 << endl;
		cout << "             Sim frame:      " << tStep << endl;
		cout << "             Time:           " << mTime << endl;
		cout << "             Avg. contacts:  " << num_contacts / out_steps
				<< endl;
		cout << "             Execution time: " << exec_time << endl;

		out_frame++;
	}
}

// =============================================================================

void OutputVehicleData(ChSystemParallelDVI& mphysicalSystem, int tStep) {
	std::ofstream outVehicleData;
	const std::string vehicleDataFile = out_dir + "/vehicleData.txt";

	if (tStep == 0) {
		outVehicleData.open(vehicleDataFile);
	} else {
		outVehicleData.open(vehicleDataFile, std::ios::app);
	}
	ChVector<> accLF = ChTransform<>::TransformParentToLocal(
			mVehicle->GetVehicle()->GetChassis()->GetPos_dtdt(), ChVector<>(0),
			mVehicle->GetVehicle()->GetChassis()->GetRot());

	outVehicleData << mphysicalSystem.GetChTime() << ", " <<

	mVehicle->GetVehicle()->GetChassis()->GetPos().x << ", "
			<< mVehicle->GetVehicle()->GetChassis()->GetPos().y << ", "
			<< mVehicle->GetVehicle()->GetChassis()->GetPos().z << ", " <<

			mVehicle->GetVehicle()->GetChassis()->GetPos_dt().x << ", "
			<< mVehicle->GetVehicle()->GetChassis()->GetPos_dt().y << ", "
			<< mVehicle->GetVehicle()->GetChassis()->GetPos_dt().z << ", " <<

			mVehicle->GetVehicle()->GetChassis()->GetPos_dt().Length() << ", "
			<<

			mVehicle->GetVehicle()->GetChassis()->GetPos_dtdt().x << ", "
			<< mVehicle->GetVehicle()->GetChassis()->GetPos_dtdt().y << ", "
			<< mVehicle->GetVehicle()->GetChassis()->GetPos_dtdt().z << ", " <<

			accLF.x << ", " << accLF.y << ", " << accLF.z << ", " <<

			mVehicle->GetPowertrain()->GetMotorTorque() << ", "
			<< mVehicle->GetPowertrain()->GetMotorSpeed() << ", "
			<< mVehicle->GetPowertrain()->GetOutputTorque() << ", "
			<< mVehicle->GetPowertrain()->GetCurrentTransmissionGear() << ", "
			<< mVehicle->GetPowertrain()->GetMaxTorque() << ", "
			<< mVehicle->GetPowertrain()->GetMaxSpeed() << ", " <<

			std::endl;
}
// =============================================================================
void printSimulationParameters() {
	simParams << " time_hold_vehicle: " << time_hold_vehicle << endl
			<< " time_pause_fluid_external_force: "
			<< time_pause_fluid_external_force << endl
			<< " contact_recovery_speed: " << contact_recovery_speed << endl
			<< " maxFlowVelocity " << maxFlowVelocity << endl
			<< " time_step (paramsH.dT): " << paramsH.dT << endl
			<< " time_end: " << time_end << endl;
}

// =============================================================================

int main(int argc, char* argv[]) {
	//****************************************************************************************
	time_t rawtime;
	struct tm* timeinfo;

	//(void) cudaSetDevice(0);

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	//****************************************************************************************
	// Arman take care of this block.
	// Set path to ChronoVehicle data files
	//  vehicle::SetDataPath(CHRONO_VEHICLE_DATA_DIR);
	//  vehicle::SetDataPath("/home/arman/Repos/GitBeta/chrono/src/demos/data/");
	//  SetChronoDataPath(CHRONO_DATA_DIR);

	// --------------------------
	// Create output directories.
	// --------------------------

	if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
		cout << "Error creating directory " << out_dir << endl;
		return 1;
	}

	if (povray_output) {
		if (ChFileutils::MakeDirectory(pov_dir_mbd.c_str()) < 0) {
			cout << "Error creating directory " << pov_dir_mbd << endl;
			return 1;
		}
	}

	if (ChFileutils::MakeDirectory(pov_dir_fluid.c_str()) < 0) {
		cout << "Error creating directory " << pov_dir_fluid << endl;
		return 1;
	}

	//****************************************************************************************
	const std::string simulationParams = out_dir
			+ "/simulation_specific_parameters.txt";
	simParams.open(simulationParams);
	simParams << " Job was submitted at date/time: " << asctime(timeinfo)
			<< endl;
	printSimulationParameters();
	// ***************************** Create Fluid ********************************************
	thrust::host_vector<::int4> referenceArray;
	thrust::host_vector<Real3> posRadH; // do not set the size here since you are using push back later
	thrust::host_vector<Real3> velMasH;
	thrust::host_vector<Real4> rhoPresMuH;
	thrust::host_vector<uint> bodyIndex;

	thrust::host_vector<Real3> pos_ChSystemBackupH;
	thrust::host_vector<Real3> vel_ChSystemBackupH;
	thrust::host_vector<Real3> acc_ChSystemBackupH;
	thrust::host_vector<Real4> quat_ChSystemBackupH;
	thrust::host_vector<Real3> omegaLRF_ChSystemBackupH;
	thrust::host_vector<Real3> omegaAccGRF_ChSystemBackupH;

	std::vector<ChSharedPtr<ChBody> > FSI_Bodies;

	Real sphMarkerMass = 0; // To be initialized in CreateFluidMarkers, and used in other places

	SetupParamsH(paramsH);

	if (initializeFluidFromFile) {
		// call to CheckPointMarkers_Read should be as close to the top as possible
		CheckPointMarkers_Read(initializeFluidFromFile, posRadH, velMasH,
				rhoPresMuH, bodyIndex, referenceArray, paramsH, numObjects);
		if (numObjects.numAllMarkers == 0) {
			ClearArraysH(posRadH, velMasH, rhoPresMuH, bodyIndex,
					referenceArray);
			return 0;
		}
#if !haveFluid
		printf("Error! Initialized from file But haveFluid is false! \n");
		return -1;
#endif
	} else {
#if haveFluid
		//*** default num markers

		int numAllMarkers = 0;

		//*** initialize fluid particles
		::int2 num_fluidOrBoundaryMarkers = CreateFluidMarkers(posRadH, velMasH,
				rhoPresMuH, bodyIndex, paramsH);
		printf("num_fluidOrBoundaryMarkers %d %d \n",
				num_fluidOrBoundaryMarkers.x, num_fluidOrBoundaryMarkers.y);
		referenceArray.push_back(mI4(0, num_fluidOrBoundaryMarkers.x, -1, -1)); // map fluid -1
		numAllMarkers += num_fluidOrBoundaryMarkers.x;
		referenceArray.push_back(
		mI4(numAllMarkers, numAllMarkers + num_fluidOrBoundaryMarkers.y, 0, 0));
		numAllMarkers += num_fluidOrBoundaryMarkers.y;

		//*** set num objects

		SetNumObjects(numObjects, referenceArray, numAllMarkers);
		//        assert(posRadH.size() == numObjects.numAllMarkers && "(1) numObjects is not set correctly");
		if (posRadH.size() != numObjects.numAllMarkers) {
			printf(
					"\n\n\n\n Error! (1) numObjects is not set correctly \n\n\n\n");
			return -1;
		}
		if (numObjects.numAllMarkers == 0) {
			ClearArraysH(posRadH, velMasH, rhoPresMuH, bodyIndex,
					referenceArray);
			std::cout << "No marker to begin with " << numObjects.numAllMarkers
					<< std::endl;
			return 0;
		}
#endif
	}
	// ***************************** Create Rigid ********************************************
	ChSystemParallelDVI mphysicalSystem;
	InitializeMbdPhysicalSystem(mphysicalSystem, argc, argv);

	// This needs to be called after fluid initialization because I am using "numObjects.numBoundaryMarkers" inside it

	CreateMbdPhysicalSystemObjects(mphysicalSystem, posRadH, velMasH,
			rhoPresMuH, bodyIndex, FSI_Bodies, referenceArray, numObjects,
			paramsH);

	// ***************************** Create Interface ********************************************

	//    assert(posRadH.size() == numObjects.numAllMarkers && "(2) numObjects is not set correctly");
	if (posRadH.size() != numObjects.numAllMarkers) {
		printf("\n\n\n\n Error! (2) numObjects is not set correctly \n\n\n\n");
		return -1;
	}

	//*** Add sph data to the physics system

	int startIndexSph = 0;
#if haveFluid
	thrust::device_vector<Real3> posRadD = posRadH;
	thrust::device_vector<Real3> velMasD = velMasH;
	thrust::device_vector<Real4> rhoPresMuD = rhoPresMuH;
	thrust::device_vector<uint> bodyIndexD = bodyIndex;
	thrust::device_vector<Real4> derivVelRhoD;
	ResizeR4(derivVelRhoD, numObjects.numAllMarkers);

	int numFsiBodies = FSI_Bodies.size();
	thrust::device_vector<Real3> posRigid_fsiBodies_D;
	thrust::device_vector<Real4> velMassRigid_fsiBodies_D;
	thrust::device_vector<Real3> accRigid_fsiBodies_D;
	thrust::device_vector<Real4> q_fsiBodies_D;
	thrust::device_vector<Real3> omegaLRF_fsiBodies_D;
	thrust::device_vector<Real3> omegaAccLRF_fsiBodies_D;
	ResizeR3(posRigid_fsiBodies_D, numFsiBodies);
	ResizeR4(velMassRigid_fsiBodies_D, numFsiBodies);
	ResizeR3(accRigid_fsiBodies_D, numFsiBodies);
	ResizeR4(q_fsiBodies_D, numFsiBodies);
	ResizeR3(omegaLRF_fsiBodies_D, numFsiBodies);
	ResizeR3(omegaAccLRF_fsiBodies_D, numFsiBodies);

	thrust::host_vector<Real3> posRigid_fsiBodies_dummyH(numFsiBodies);
	thrust::host_vector<Real4> velMassRigid_fsiBodies_dummyH(numFsiBodies);
	thrust::host_vector<Real3> accRigid_fsiBodies_dummyH(numFsiBodies);
	thrust::host_vector<Real4> q_fsiBodies_dummyH(numFsiBodies);
	thrust::host_vector<Real3> omegaLRF_fsiBodies_dummyH(numFsiBodies);
	thrust::host_vector<Real3> omegaAccLRF_fsiBodies_dummyH(numFsiBodies);



	Copy_fsiBodies_ChSystem_to_FluidSystem(
			posRigid_fsiBodies_D, velMassRigid_fsiBodies_D, accRigid_fsiBodies_D,
			q_fsiBodies_D, omegaLRF_fsiBodies_D, omegaAccLRF_fsiBodies_D,
			posRigid_fsiBodies_dummyH, velMassRigid_fsiBodies_dummyH, accRigid_fsiBodies_dummyH,
			q_fsiBodies_dummyH, omegaLRF_fsiBodies_dummyH, omegaAccLRF_fsiBodies_dummyH,
			FSI_Bodies, mphysicalSystem);

	thrust::device_vector<Real3> posRigid_fsiBodies_D2 = posRigid_fsiBodies_D;
	thrust::device_vector<Real4> velMassRigid_fsiBodies_D2 = velMassRigid_fsiBodies_D;
	thrust::device_vector<Real3> accRigid_fsiBodies_D2 = accRigid_fsiBodies_D;

	thrust::device_vector<Real4> q_fsiBodies_D2 = q_fsiBodies_D;
	thrust::device_vector<Real3> omegaLRF_fsiBodies_D2 = omegaLRF_fsiBodies_D;
	thrust::device_vector<Real3> omegaAccLRF_fsiBodies_D2 = omegaAccLRF_fsiBodies_D;

	thrust::device_vector<Real3> rigid_FSI_ForcesD;
	thrust::device_vector<Real3> rigid_FSI_TorquesD;
	ResizeR3(rigid_FSI_ForcesD, numFsiBodies);
	ResizeR3(rigid_FSI_TorquesD, numFsiBodies);
	// assert
	if ((numObjects.numRigidBodies != numFsiBodies)
			|| (referenceArray.size() - 2 != numFsiBodies)) {
		printf(
				"\n\n\n\n Error! number of fsi bodies (%d) does not match numObjects.numRigidBodies (%d). Size of "
						"reference array: %d \n\n\n\n", numFsiBodies,
				numObjects.numRigidBodies, referenceArray.size());
		return -1;
	}
	ResizeR3(rigid_FSI_ForcesD, numObjects.numRigidBodies);
	ResizeR3(rigid_FSI_TorquesD, numObjects.numRigidBodies);

	thrust::device_vector<uint> rigidIdentifierD;
	ResizeU1(rigidIdentifierD, numObjects.numRigid_SphMarkers);
	thrust::device_vector<Real3> rigidSPH_MeshPos_LRF_D;
	ResizeR3(rigidSPH_MeshPos_LRF_D, numObjects.numRigid_SphMarkers);

	InitSystem(paramsH, numObjects);

	Populate_RigidSPH_MeshPos_LRF(rigidIdentifierD, rigidSPH_MeshPos_LRF_D,
			posRadD, posRigid_fsiBodies_D, q_fsiBodies_D, referenceArray,
			numObjects);

	// ** initialize device mid step data
	thrust::device_vector<Real3> posRadD2 = posRadD;
	thrust::device_vector<Real3> velMasD2 = velMasD;
	thrust::device_vector<Real4> rhoPresMuD2 = rhoPresMuD;
	thrust::device_vector<Real3> vel_XSPH_D;
	ResizeR3(vel_XSPH_D, numObjects.numAllMarkers);
	//    assert(posRadD.size() == numObjects.numAllMarkers && "(3) numObjects is not set correctly");
	if (posRadD.size() != numObjects.numAllMarkers) {
		printf("\n\n\n\n Error! (3) numObjects is not set correctly \n\n\n\n");
		return -1;
	}
#endif
	cout << " -- ChSystem size : " << mphysicalSystem.Get_bodylist()->size()
			<< endl;

	// ***************************** System Initialize ********************************************

	InitializeChronoGraphics(mphysicalSystem);

	double mTime = 0;

	DOUBLEPRECISION ?
			printf("Double Precision\n") : printf("Single Precision\n");

	int stepEnd = int(paramsH.tFinal / paramsH.dT); // 1.0e6;//2.4e6;//600000;//2.4e6 * (.02 * paramsH.sizeScale) /
													// currentParamsH.dT ; //1.4e6 * (.02 * paramsH.sizeScale) /
													// currentParamsH.dT ;//0.7e6 * (.02 * paramsH.sizeScale) /
													// currentParamsH.dT ;//0.7e6;//2.5e6;
													// //200000;//10000;//50000;//100000;
	printf("stepEnd %d\n", stepEnd);
	Real realTime = 0;

	SimParams paramsH_B = paramsH;
	paramsH_B.bodyForce3 = mR3(0);
	paramsH_B.dT = paramsH.dT;

	printf("\ntimePause %f, numPause %d\n", paramsH.timePause,
			int(paramsH.timePause / paramsH_B.dT));
	printf("paramsH.timePauseRigidFlex %f, numPauseRigidFlex %d\n\n",
			paramsH.timePauseRigidFlex,
			int(
					(paramsH.timePauseRigidFlex - paramsH.timePause)
							/ paramsH.dT + paramsH.timePause / paramsH_B.dT));
	//  InitSystem(paramsH, numObjects);
	SimParams currentParamsH = paramsH;

	simParams.close();

	// ******************************************************************************************
	// ******************************************************************************************
	// ******************************************************************************************
	// ******************************************************************************************
	// ***************************** Simulation loop ********************************************

	chrono::ChTimerParallel fsi_timer;
	fsi_timer.AddTimer("total_step_time");
	fsi_timer.AddTimer("fluid_initialization");
	fsi_timer.AddTimer("DoStepDynamics_FSI");
	fsi_timer.AddTimer("DoStepDynamics_ChronoRK2");

	for (int tStep = 0; tStep < stepEnd + 1; tStep++) {
		// -------------------
		// SPH Block
		// -------------------
		fsi_timer.Reset();

#if haveFluid
		CpuTimer mCpuTimer;
		mCpuTimer.Start();
		GpuTimer myGpuTimer;
		myGpuTimer.Start();

		if (realTime <= paramsH.timePause) {
			currentParamsH = paramsH_B;
		} else {
			currentParamsH = paramsH;
		}

		fsi_timer.start("total_step_time");

		//		CopySys2D(posRadD, mphysicalSystem, numObjects, startIndexSph);
		fsi_timer.start("fluid_initialization");

		int out_steps = std::ceil((1.0 / paramsH.dT) / out_fps);
		PrintToFile(posRadD, velMasD, rhoPresMuD, referenceArray,
				currentParamsH, realTime, tStep, out_steps, pov_dir_fluid);

		// ******* slow down the sys.Check point the sys.
		CheckPointMarkers_Write(posRadH, velMasH, rhoPresMuH, bodyIndex,
				referenceArray, paramsH, numObjects, tStep, tStepsCheckPoint);

		//    // freeze sph. check it later
		//    if (fmod(realTime, 0.6) < paramsH.dT && realTime < 1.3) {
		//      FreezeSPH(velMasD, velMasH);
		//    }
		// *******

		fsi_timer.stop("fluid_initialization");
#endif
#if haveFluid
		fsi_timer.start("DoStepDynamics_FSI");

		DoStepDynamics_FSI(mphysicalSystem, mVehicle, posRadD, velMasD,
				vel_XSPH_D, rhoPresMuD,

				posRadD2, velMasD2, rhoPresMuD2,

				derivVelRhoD, rigidIdentifierD, rigidSPH_MeshPos_LRF_D,

				posRigid_fsiBodies_D, velMassRigid_fsiBodies_D, accRigid_fsiBodies_D,
				q_fsiBodies_D, omegaLRF_fsiBodies_D, omegaAccLRF_fsiBodies_D,

				posRigid_fsiBodies_D2, velMassRigid_fsiBodies_D2, accRigid_fsiBodies_D2,
				q_fsiBodies_D2, omegaLRF_fsiBodies_D2, omegaAccLRF_fsiBodies_D2,

				pos_ChSystemBackupH, vel_ChSystemBackupH, acc_ChSystemBackupH,
				quat_ChSystemBackupH, omegaLRF_ChSystemBackupH, omegaAccGRF_ChSystemBackupH,

				posRigid_fsiBodies_dummyH, velMassRigid_fsiBodies_dummyH, accRigid_fsiBodies_dummyH,
				q_fsiBodies_dummyH, omegaLRF_fsiBodies_dummyH, omegaAccLRF_fsiBodies_dummyH,

				rigid_FSI_ForcesD, rigid_FSI_TorquesD,

				bodyIndexD, FSI_Bodies, referenceArray, numObjects, paramsH,
				mTime, time_hold_vehicle, tStep,
				haveVehicle);
		fsi_timer.stop("DoStepDynamics_FSI");
#else
		fsi_timer.start("DoStepDynamics_ChronoRK2");
		DoStepDynamics_ChronoRK2(mphysicalSystem,
				mVehicle,

				pos_ChSystemBackupH,
				vel_ChSystemBackupH,
				acc_ChSystemBackupH,
				quat_ChSystemBackupH,
				omegaLRF_ChSystemBackupH,
				omegaAccGRF_ChSystemBackupH,

				paramsH,
				mTime,
				time_hold_vehicle,
				haveVehicle);
		fsi_timer.stop("DoStepDynamics_ChronoRK2");
#endif
		// -------------------
		// End SPH Block
		// -------------------

		SavePovFilesMBD(mphysicalSystem, tStep, mTime);

//    OutputVehicleData(mphysicalSystem, tStep);

// -------------------
// SPH Block
// -------------------
#if haveFluid
		mCpuTimer.Stop();
		myGpuTimer.Stop();
		if (tStep % 2 == 0) {
			printf(
					"step: %d, realTime: %f, step Time (CUDA): %f, step Time (CPU): %f\n ",
					tStep, realTime, (Real) myGpuTimer.Elapsed(),
					1000 * mCpuTimer.Elapsed());
		}
#endif
		fsi_timer.stop("total_step_time");
		fsi_timer.PrintReport();
		// -------------------
		// End SPH Block
		// -------------------

		fflush(stdout);
		realTime += currentParamsH.dT;

		//        mphysicalSystem.data_manager->system_timer.PrintReport();
	}
	ClearArraysH(posRadH, velMasH, rhoPresMuH, bodyIndex, referenceArray);
	FSI_Bodies.clear();

	pos_ChSystemBackupH.clear();
	vel_ChSystemBackupH.clear();
	acc_ChSystemBackupH.clear();
	quat_ChSystemBackupH.clear();
	omegaLRF_ChSystemBackupH.clear();
	omegaAccGRF_ChSystemBackupH.clear();

// Arman LRF in omegaLRF may need change
#if haveFluid
	ClearMyThrustR3(posRadD);
	ClearMyThrustR3(velMasD);
	ClearMyThrustR4(rhoPresMuD);
	ClearMyThrustU1(bodyIndexD);
	ClearMyThrustR4(derivVelRhoD);
	ClearMyThrustU1(rigidIdentifierD);
	ClearMyThrustR3(rigidSPH_MeshPos_LRF_D);

	ClearMyThrustR3(posRadD2);
	ClearMyThrustR3(velMasD2);
	ClearMyThrustR4(rhoPresMuD2);
	ClearMyThrustR3(vel_XSPH_D);

	ClearMyThrustR3(posRigid_fsiBodies_D);
	ClearMyThrustR4(velMassRigid_fsiBodies_D);
	ClearMyThrustR3(accRigid_fsiBodies_D);
	ClearMyThrustR4(q_fsiBodies_D);
	ClearMyThrustR3(omegaLRF_fsiBodies_D);
	ClearMyThrustR3(omegaAccLRF_fsiBodies_D);

	ClearMyThrustR3(posRigid_fsiBodies_D2);
	ClearMyThrustR4(velMassRigid_fsiBodies_D2);
	ClearMyThrustR3(accRigid_fsiBodies_D2);
	ClearMyThrustR4(q_fsiBodies_D2);
	ClearMyThrustR3(omegaLRF_fsiBodies_D2);
	ClearMyThrustR3(omegaAccLRF_fsiBodies_D2);

	ClearMyThrustR3(rigid_FSI_ForcesD);
	ClearMyThrustR3(rigid_FSI_TorquesD);

	posRigid_fsiBodies_dummyH.clear();
	velMassRigid_fsiBodies_dummyH.clear();
	accRigid_fsiBodies_dummyH.clear();
	q_fsiBodies_dummyH.clear();
	omegaLRF_fsiBodies_dummyH.clear();
	omegaAccLRF_fsiBodies_dummyH.clear();
#endif
	delete mVehicle;
	delete tire_cb;
	delete chassis_cb;
	delete driver_cb;

	return 0;
}
