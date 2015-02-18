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


using namespace std;

SimParams paramsH;
//typedef unsigned int uint;

////************ note: paramsH is zero here. These expressions are wrong
const real_ mm = .001;

//&&& some other definitions for boundary and such

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
struct CylinderGeometry {
	real3 pa3;
	real3 pb3;
	real3 center;
	real_ r;
	real_ h;
};
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
real_ myRand() {
	return real_(rand()) / RAND_MAX;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
real_ Min(real_ a, real_ b) {
	return (a < b) ? a : b;
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
real_ Max(real_ a, real_ b) {
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

void InitializeMbdPhysicalSystem(ChSystemParallelDVI & mphysicalSystem, int threads, uint max_iteration) {
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
	mphysicalSystem.GetSettings()->collision.bins_per_axis = I3(10, 10, 10); //Arman check
}

void create_system_particles(ChSystemParallelDVI& mphysicalSystem, real_ muFriction)
{
	real3 domainCenter = 0.5 * (paramsH.cMin + paramsH.cMax);
	real3 ellipsoidSize = R3(10 * paramsH.HSML, 5 * paramsH.HSML, 7 * paramsH.HSML);
	ChVector<> size = ChVector<>(ellipsoidSize.x, ellipsoidSize.y, ellipsoidSize.z);

	real_ density = paramsH.rho0;  // TODO : Arman : Density for now is set to water density
	real_ volume = utils::CalcEllipsoidVolume(size);
	real_ gyration = utils::CalcEllipsoidGyration(size).Get_Diag();
	real_ mass = density * volume;


	// Generate ice particels
	//(void)CreateIceParticles(mphysicalSystem);




	//**************** bin and ship
	// IDs for the two bodies
	int bId = -200;

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
	body = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
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
	body->GetCollisionModel()->AddEllipsoid(size.x, size.y, size.z, pos, rot);

	// add asset (for visualization)
	ChSharedPtr<ChEllipsoidShape> ellipsoid(new ChEllipsoidShape);
	ellipsoid->GetEllipsoidGeometry().rad = size;
	ellipsoid->Pos = pos;
	ellipsoid->Rot = rot;

	body->GetAssets().push_back(ellipsoid);






	// Create the containing bin (2 x 2 x 1)
	double hthick = .1;
	double hole_width = 1.05 * ship_w;
	double small_wall_Length = 0.5 * (hdim.x - hole_width);

	ChSharedBodyPtr bin;
	bin = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
	bin->SetMaterialSurface(mat);
	bin->SetIdentifier(binId);
	bin->SetMass(1);
	bin->SetPos(ChVector<>(center.x, center.y, center.z));
	bin->SetRot(ChQuaternion<>(1, 0, 0, 0));
	bin->SetCollide(true);
	bin->SetBodyFixed(true);
	bin->GetCollisionModel()->ClearModel();

	utils::AddBoxGeometry(bin.get_ptr(), 0.5 * ChVector<>(hdim.x, hdim.y, hthick), ChVector<>(0, 0, 0.5 * hdim.z + 0.5*hthick));	//end wall
	utils::AddBoxGeometry(bin.get_ptr(), 0.5 * ChVector<>(hthick, hdim.y, hdim.z + 2 * hthick), ChVector<>(-0.5 * hdim.x - 0.5 * hthick, 0, 0));		//side wall
	utils::AddBoxGeometry(bin.get_ptr(), 0.5 * ChVector<>(hthick, hdim.y, hdim.z + 2 * hthick), ChVector<>(0.5 * hdim.x + 0.5 * hthick, 0, 0));	//side wall
	utils::AddBoxGeometry(bin.get_ptr(), 0.5 * ChVector<>(small_wall_Length, hdim.y, hthick), ChVector<>(-0.5 * hdim.x + 0.5*small_wall_Length, 0, -0.5 * hdim.z - 0.5*hthick)); 	//beginning wall 1
	utils::AddBoxGeometry(bin.get_ptr(), 0.5 * ChVector<>(small_wall_Length, hdim.y, hthick), ChVector<>(0.5 * hdim.x - 0.5*small_wall_Length, 0, -0.5 * hdim.z - 0.5*hthick)); //beginning wall 2

	utils::AddBoxGeometry(bin.get_ptr(), 0.5 * ChVector<>(7 * hdim.x, hthick, 7 * hdim.x), ChVector<>(0,-10,0)); //bottom bed
	bin->GetCollisionModel()->BuildModel();

	mphysicalSystem.AddBody(bin);

	//**************** create ship
	double shipMass = rhoPlate * ship_w * ship_y * ship_z;
	double bI1 = 1.0 / 12 * shipMass * (pow(ship_w, 2) + pow(ship_y, 2));
	double bI2 = 1.0 / 12 * shipMass * (pow(ship_y, 2) + pow(ship_z, 2));
	double bI3 = 1.0 / 12 * shipMass * (pow(ship_w, 2) + pow(ship_z, 2));
	printf("mass %f I1 I2 I3 %f %f %f\n", shipMass, bI1, bI2, bI3);

	shipInitialPosZ = boxMin.z - .5 * ship_z;

	shipPtr = ChSharedBodyPtr(new ChBody(new ChCollisionModelParallel));
	shipInitialPos = ChVector<>(center.x,  center.y, shipInitialPosZ);
	shipPtr->SetPos(shipInitialPos);
	shipPtr->SetRot(ChQuaternion<>(1,0,0,0));
	shipPtr->SetMaterialSurface(mat);
	shipPtr->SetPos_dt(ChVector<>(0,0,0));
	shipPtr->SetMass(shipMass);
	shipPtr->SetInertiaXX(ChVector<>(bI2, bI3, bI1));
	shipPtr->SetIdentifier(shipId);
	shipPtr->SetCollide(true);
	shipPtr->SetBodyFixed(false);

	shipPtr->GetCollisionModel()->ClearModel();
//	shipPtr->GetCollisionModel()->SetDefaultSuggestedEnvelope(collisionEnvelop); //envelop is .03 by default
	utils::AddBoxGeometry(shipPtr.get_ptr(), 0.5 * ChVector<>(ship_w, ship_y, ship_z), ChVector<>(0,0,0)); //beginning wall 2. Need "0.5 *" since chronoparallel is apparently different
	shipPtr->GetCollisionModel()->BuildModel();
	mphysicalSystem.Add(shipPtr);
}


//*** paramsH.straightChannelBoundaryMax   should be taken care of
real_ IsInsideStraightChannel(real3 posRad) {
	const real_ sphR = paramsH.HSML;
	real_ penDist1 = 0;
	real_ penDist2 = 0;
	real_ penDist3 = 0;
	real_ largePenet = -5 * sphR; //like a large number. Should be negative (assume large initial penetration)

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
		thrust::host_vector<real3> & posRadH,
		thrust::host_vector<real4> & velMasH,
		thrust::host_vector<real4> & rhoPresMuH,
		thrust::host_vector<uint> & bodyIndex,
		real_ & sphMarkerMass) {

	thrust::host_vector<real3> mPosRadBoundary; //do not set the size here since you are using push back later
	thrust::host_vector<real4> mVelMasBoundary;
	thrust::host_vector<real4> mRhoPresMuBoundary;

	int num_FluidMarkers = 0;
	int num_BoundaryMarkers = 0;
	srand(964);
	real_ multInitSpace = paramsH.MULT_INITSPACE; //0.9;//0.9;
	real_ initSpace0 = paramsH.MULT_INITSPACE * paramsH.HSML;
	int nFX = ceil((paramsH.cMax.x - paramsH.cMin.x) / (initSpace0));
	real_ initSpaceX = (paramsH.cMax.x - paramsH.cMin.x) / nFX;
	//printf("orig nFx and nFx %f %f\n", (paramsH.cMax.x - paramsH.cMin.x) / initSpace, ceil ((paramsH.cMax.x - paramsH.cMin.x) / (initSpace)));
	int nFY = ceil((paramsH.cMax.y - paramsH.cMin.y) / (initSpace0));
	real_ initSpaceY = (paramsH.cMax.y - paramsH.cMin.y) / nFY;
	int nFZ = ceil((paramsH.cMax.z - paramsH.cMin.z) / (initSpace0));
	real_ initSpaceZ = (paramsH.cMax.z - paramsH.cMin.z) / nFZ;
	//printf("&&&&& %f   %f %f %f \n", 1.1 * sphR, initSpaceX, initSpaceY, initSpaceZ);
	printf("nFX Y Z %d %d %d, max distY %f, initSpaceY %f\n", nFX, nFY, nFZ,
			(nFY - 1) * initSpaceY, initSpaceY);
	sphMarkerMass = (initSpaceX * initSpaceY * initSpaceZ) * paramsH.rho0;

	for (int i = 0; i < nFX; i++) {
		for (int j = 0; j < nFY; j++) {
			for (int k = 0; k < nFZ; k++) {
				real3 posRad;
//					printf("initSpace X, Y, Z %f %f %f \n", initSpaceX, initSpaceY, initSpaceZ);
				posRad =
						paramsH.cMin
								+ R3(i * initSpaceX, j * initSpaceY, k * initSpaceZ)
								+ R3(.5 * initSpace0)/* + R3(sphR) + initSpace * .05 * (real_(rand()) / RAND_MAX)*/;
				if ( 	(posRad.x >  paramsH.straightChannelBoundaryMin.x && posRad.x <  paramsH.straightChannelBoundaryMax.x ) &&
						(posRad.y >  paramsH.straightChannelBoundaryMin.y && posRad.y <  paramsH.straightChannelBoundaryMax.y ) &&
						(posRad.z >  paramsH.straightChannelBoundaryMin.z && posRad.z <  paramsH.straightChannelBoundaryMax.z ) )
				{
					if (i < 0.5 * nFX) {
						num_FluidMarkers++;
						posRadH.push_back(posRad);
						real3 v3 = R3(0);
						velMasH.push_back(R4(v3, sphMarkerMass));
						rhoPresMuH.push_back(R4(paramsH.rho0, paramsH.BASEPRES, paramsH.mu0, -1));
					}
				} else {
					num_BoundaryMarkers++;
					mPosRadBoundary.push_back(posRad);
					mVelMasBoundary.push_back(R4(0, 0, 0, sphMarkerMass));
					mRhoPresMuBoundary.push_back(R4(paramsH.rho0, paramsH.LARGE_PRES, paramsH.mu0, 0));
				}
			}
		}
	}
	int2 num_fluidOrBoundaryMarkers = I2(num_FluidMarkers, num_BoundaryMarkers);
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
	paramsH.gravity = R3(0, -9.81, 0);//R3(0);//R3(0, -9.81, 0);
	paramsH.bodyForce3 = R3(0,0,0);//R4(3.2e-3,0,0,0);// R4(0);;// /*Re = 100 */ //R4(3.2e-4, 0, 0, 0);/*Re = 100 */
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
	paramsH.straightChannelBoundaryMin = R3(0, 0, 0); //3D channel
	paramsH.straightChannelBoundaryMax = R3(3, 2, 3) * paramsH.sizeScale;
	//********************************************************************************************************
	//**  reminiscent of the past******************************************************************************
	paramsH.cMin = R3(-paramsH.toleranceZone, -paramsH.toleranceZone, -paramsH.toleranceZone);						// 3D channel
	paramsH.cMax = R3( 3  + paramsH.toleranceZone, 2 + paramsH.toleranceZone,  3 + paramsH.toleranceZone);
	//****************************************************************************************
	//printf("a1  paramsH.cMax.x, y, z %f %f %f,  binSize %f\n", paramsH.cMax.x, paramsH.cMax.y, paramsH.cMax.z, 2 * paramsH.HSML);
	int3 side0 = I3(
			floor((paramsH.cMax.x - paramsH.cMin.x) / (2 * paramsH.HSML)),
			floor((paramsH.cMax.y - paramsH.cMin.y) / (2 * paramsH.HSML)),
			floor((paramsH.cMax.z - paramsH.cMin.z) / (2 * paramsH.HSML)));
	real3 binSize3 = R3((paramsH.cMax.x - paramsH.cMin.x) / side0.x,
			(paramsH.cMax.y - paramsH.cMin.y) / side0.y,
			(paramsH.cMax.z - paramsH.cMin.z) / side0.z);
	paramsH.binSize0 = (binSize3.x > binSize3.y) ? binSize3.x : binSize3.y;
//	paramsH.binSize0 = (paramsH.binSize0 > binSize3.z) ? paramsH.binSize0 : binSize3.z;
	paramsH.binSize0 = binSize3.x; //for effect of distance. Periodic BC in x direction. we do not care about paramsH.cMax y and z.
	paramsH.cMax = paramsH.cMin + paramsH.binSize0 * R3(side0);
	paramsH.boxDims = paramsH.cMax - paramsH.cMin;
	//************************** modify pressure ***************************
//		paramsH.deltaPress = paramsH.rho0 * paramsH.boxDims * paramsH.bodyForce3;  //did not work as I expected
	paramsH.deltaPress = 0.9 * paramsH.boxDims * paramsH.bodyForce3;

	// modify bin size stuff
	//****************************** bin size adjustement and contact detection stuff *****************************
	int3 SIDE = I3(int((paramsH.cMax.x - paramsH.cMin.x) / paramsH.binSize0 + .1), int((paramsH.cMax.y - paramsH.cMin.y) / paramsH.binSize0 + .1),
			int((paramsH.cMax.z - paramsH.cMin.z) / paramsH.binSize0 + .1));
	real_ mBinSize = paramsH.binSize0; //Best solution in that case may be to change cMax or cMin such that periodic sides be a multiple of binSize
	//**********************************************************************************************************
	paramsH.gridSize = SIDE;
	//paramsH.numCells = SIDE.x * SIDE.y * SIDE.z;
	paramsH.worldOrigin = paramsH.cMin;
	paramsH.cellSize = R3(mBinSize, mBinSize, mBinSize);

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
	thrust::host_vector<real3> & posRadH, //do not set the size here since you are using push back later
	thrust::host_vector<real4> & velMasH,
	thrust::host_vector<real4> & rhoPresMuH) {

	posRadH.clear();
	velMasH.clear();
	rhoPresMuH.clear();
}
void ClearArraysH(
	thrust::host_vector<real3> & posRadH, //do not set the size here since you are using push back later
	thrust::host_vector<real4> & velMasH,
	thrust::host_vector<real4> & rhoPresMuH,
	thrust::host_vector<uint> & bodyIndex,
	thrust::host_vector<int3> & referenceArray) {

	ClearArraysH(posRadH, velMasH, rhoPresMuH);
	bodyIndex.clear();
	referenceArray.clear();
}

void CopyD2H(
	thrust::host_vector<real3> & posRadH, //do not set the size here since you are using push back later
	thrust::host_vector<real4> & velMasH,
	thrust::host_vector<real4> & rhoPresMuH,
	const thrust::device_vector<real3> & posRadD,
	const thrust::device_vector<real4> & velMasD,
	const thrust::device_vector<real4> & rhoPresMuD) {
	assert(posRadH.size() == posRadD.size() && "Error! size mismatch host and device" );
	thrust::copy(posRadD.begin(), posRadD.end(), posRadH.begin());
	thrust::copy(velMasD.begin(), velMasD.end(), velMasH.begin());
	thrust::copy(rhoPresMuD.begin(), rhoPresMuD.end(), rhoPresMuH.begin());
}

//void CopyD2D(
//	thrust::host_vector<real3> & posRadD2, //do not set the size here since you are using push back later
//	thrust::host_vector<real4> & velMasD2,
//	thrust::host_vector<real4> & rhoPresMuD2,
//	const thrust::device_vector<real3> & posRadD,
//	const thrust::device_vector<real4> & velMasD,
//	const thrust::device_vector<real4> & rhoPresMuD) {
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
	thrust::host_vector<real3> posRadH; //do not set the size here since you are using push back later
	thrust::host_vector<real4> velMasH;
	thrust::host_vector<real4> rhoPresMuH;
	thrust::host_vector<uint> bodyIndex;
	SetupParamsH(paramsH);

	NumberOfObjects numObjects;
	//** default num markers
	int numAllMarkers = 0;

	// initialize fluid particles
	real_ sphMarkerMass; // To be initialized in CreateFluidMarkers, and used in other places
	int2 num_fluidOrBoundaryMarkers = CreateFluidMarkers(posRadH, velMasH, rhoPresMuH, bodyIndex, sphMarkerMass);
	referenceArray.push_back(I3(0, num_fluidOrBoundaryMarkers.x, -1)); //map fluid -1
	numAllMarkers += num_fluidOrBoundaryMarkers.x;
	referenceArray.push_back(
			I3(numAllMarkers, numAllMarkers + num_fluidOrBoundaryMarkers.y,	0));
	numAllMarkers += num_fluidOrBoundaryMarkers.y;

	// set num objects
	SetNumObjects(numObjects, referenceArray, numAllMarkers);
	if (numObjects.numAllMarkers == 0) {
		ClearArraysH(posRadH, velMasH, rhoPresMuH, bodyIndex, referenceArray);
		return 0;
	}
	// ***************************** Create Rigid ********************************************
	ChTimer<double> myTimerTotal;
	ChTimer<double> myTimerStep;
	int threads = 2;
	uint max_iteration = 1000;//10000;
	MySeed(964);

	// Save PovRay post-processing data?
	bool write_povray_data = true;

	myTimerTotal.start();
	outSimulationInfo.open("SimInfo.txt");

	SetArgumentsForMbdFromInput(argc, argv, threads, max_iteration);


	// ***** params
	double muFriction = .1;
	double dT = paramsH.dT;
	double out_fps = 50;
	// ************


	// Create a ChronoENGINE physical system
	ChSystemParallelDVI mphysicalSystem;
	InitializeMbdPhysicalSystem(mphysicalSystem, threads, max_iteration);

	// Set gravitational acceleration

	//******************* Irrlicht and driver types **************************
#define irrlichtVisualization false
	driveType = ACTUATOR;//KINEMATIC : ACTUATOR
	//************************************************************************
	outSimulationInfo << "****************************************************************************" << endl;
	outSimulationInfo 	<< " dT: " << dT  << " max_iteration: " << max_iteration <<" muFriction: " << muFriction << " threads: " << threads << endl;
	cout				<< " dT: " << dT  << " max_iteration: " << max_iteration <<" muFriction: " << muFriction << " threads: " << threads << endl;

	ofstream outForceData("forceData.txt");

	// Create all the rigid bodies.
	create_system_particles(mphysicalSystem);

#ifdef CHRONO_PARALLEL_HAS_OPENGL2
   opengl::ChOpenGLWindow &gl_window = opengl::ChOpenGLWindow::getInstance();
   gl_window.Initialize(1280, 720, "mixerDVI", &mphysicalSystem);
   gl_window.SetCamera(ChVector<>(-3,12,-8), ChVector<>(7.2, 6, 8.2), ChVector<>(0, 1, 0)); //camera

   // Uncomment the following two lines for the OpenGL manager to automatically
   // run the simulation in an infinite loop.
   //gl_window.StartDrawLoop(time_step);
   //return 0;
#endif

#if irrlichtVisualization
		cout << "@@@@@@@@@@@@@@@@  irrlicht stuff  @@@@@@@@@@@@@@@@" << endl;
		// Create the Irrlicht visualization (open the Irrlicht device,
		// bind a simple user interface, etc. etc.)
		ChIrrApp application(&mphysicalSystem, L"Bricks test",core::dimension2d<u32>(800,600),false, true);
		// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
		ChIrrWizard::add_typical_Logo  (application.GetDevice());
		ChIrrWizard::add_typical_Sky   (application.GetDevice());
		ChIrrWizard::add_typical_Lights(application.GetDevice(), core::vector3df(14.0f, 44.0f, -18.0f), core::vector3df(-3.0f, 8.0f, 6.0f), 59,  40);
		ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0.5,3,7), core::vector3df(2,1,5)); //   (7.2,30,0) :  (-3,12,-8)
		// Use this function for adding a ChIrrNodeAsset to all items
		// If you need a finer control on which item really needs a visualization proxy in
		// Irrlicht, just use application.AssetBind(myitem); on a per-item basis.
		application.AssetBindAll();
		// Use this function for 'converting' into Irrlicht meshes the assets
		// into Irrlicht-visualizable meshes
		application.AssetUpdateAll();

		application.SetStepManage(true);
		application.SetTimestep(dT);  					//Arman modify
		cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@" << endl;

#endif
		outForceData << "[1] time, [2-5] forceContact (x, y, z, magnitude), [6-9] forceActuator (x, y, z, magnitude), [10-13] Ice pressure contact (x, y, z, magnitude), [14-17] Ice pressure actuator (x, y, z, magnitude), [18] shipPos, [19] shipVel, [20] energy, [21] iceThickness, [22] timePerStep, [23] timeElapsed. ## numSpheres" << mphysicalSystem.Get_bodylist()->end() - mphysicalSystem.Get_bodylist()->begin()
				<< " pauseTime: " << timePause<< " setVelocity: "<< shipVelocity << " ship_w: " << ship_w  << endl;
		outForceData.close();
		outSimulationInfo << "Real Time, Compute Time" << endl;

	outSimulationInfo << "***** number of bodies: " << mphysicalSystem.Get_bodylist()->size() << endl;
	bool moveTime = false;
	//****************************************** Time Loop *************************************
	ChSharedPtr<ChFunction_Const> actuator_fun0(new ChFunction_Const(0));
	ChSharedPtr<ChFunction_Ramp> actuator_fun1(new ChFunction_Ramp(shipVelocity * timePause, -shipVelocity)); // function works with general system timer. since the initial force needs to be zero at t=timePause, 0 = x0 + v*t --> x0 = -v*t
	if (driveType == ACTUATOR) {
		Add_Actuator(mphysicalSystem);
	}

	int counter = -1;
	while(mphysicalSystem.GetChTime() < timeMove+timePause) //arman modify
	{
		myTimerStep.start();
		counter ++;
		// ****** include force or motion ********
		switch (driveType) {
		case KINEMATIC:
			(mphysicalSystem.GetChTime() < timePause) ?
				FixShip_Kinematic() :
				MoveShip_Kinematic(mphysicalSystem.GetChTime());
			break;
		case ACTUATOR:
			if (mphysicalSystem.GetChTime() < timePause) {
				MoveShip_Actuator(mphysicalSystem, actuator_fun0.StaticCastTo<ChFunction>(), 0, 0);
			} else {
				MoveShip_Actuator(mphysicalSystem, actuator_fun1.StaticCastTo<ChFunction>(), shipVelocity, 1);
			}
		}
		// ****** end of force or motion *********
#if irrlichtVisualization
		if ( !(application.GetDevice()->run()) ) break;
		application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));
		ChIrrTools::drawGrid(application.GetVideoDriver(), .2,.2, 150,150,
			ChCoordsys<>(ChVector<>(0.5 * hdim.x, boxMin.y, 0.5 * hdim.z),Q_from_AngAxis(CH_C_PI/2,VECT_X)), video::SColor(50,90,90,150),true);
		application.DrawAll();
		application.DoStep();
		application.GetVideoDriver()->endScene();
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

		//******************** ship force*********************
		ChVector<> mForceActuator = ChVector<>(0,0,0);
		ChVector<> mTorqueActuator = ChVector<>(0,0,0);
		ChVector<> icePressureActuator = ChVector<>(0,0,0);
		ChVector<> mTorqueContact;
		ChVector<> mForceContact;


		if (driveType == ACTUATOR) {
			ChSharedPtr<ChLinkLinActuator> actuator;
			actuator = mphysicalSystem.SearchLink("actuator").StaticCastTo<ChLinkLinActuator>();
			mForceActuator = actuator->Get_react_force();
			mTorqueActuator = actuator->Get_react_torque();
			icePressureActuator = mForceActuator / iceThickness / ship_w;
		}
		calc_ship_contact_forces(mphysicalSystem, mForceContact, mTorqueContact);
		ChVector<> icePressureContact = mForceContact / iceThickness / ship_w;

		myTimerStep.stop();
		myTimerTotal.stop();
		//****************************************************
		vector<ChBody*>::iterator ibody = mphysicalSystem.Get_bodylist()->begin();
		double energy = 0;
		while (ibody != mphysicalSystem.Get_bodylist()->end()) {
			create_hydronynamic_force(*ibody, mphysicalSystem, surfaceLoc, false);
			energy += pow((*ibody)->GetPos_dt().Length() , 2);
			ibody++;
		}

		printf("*** total number of contacts %d, num bodies %d\n", mphysicalSystem.GetNcontacts(), mphysicalSystem.Get_bodylist()->size());
		stringstream outDataSS;
		outDataSS << mphysicalSystem.GetChTime() << ", " <<
				mForceContact.x << ", " << mForceContact.y << ", " << mForceContact.z << ", " << mForceContact.Length() << ", " <<
				mForceActuator.x << ", " << mForceActuator.y << ", " << mForceActuator.z << ", " << mForceActuator.Length() << ", " <<
				icePressureContact.x << ", " << icePressureContact.y << ", " << icePressureContact.z << ", " << icePressureContact.Length() << ", " <<
				icePressureActuator.x << ", " << icePressureActuator.y << ", " << icePressureActuator.z << ", " << icePressureActuator.Length() << ", " <<
				shipPtr->GetPos().z << ", " << shipPtr->GetPos_dt().z << ", " << energy << ", " << iceThickness  << ", " << myTimerStep() << ", " << myTimerTotal() << endl;
		ofstream outData("forceData.txt", ios::app);
		outData<<outDataSS.str();
		outData.close();

		double numIter = ((ChLcpSolverParallelDVI*)mphysicalSystem.GetLcpSolverSpeed())->GetTotalIterations();
		outSimulationInfo << "Time: " <<  mphysicalSystem.GetChTime() <<
				" executionTime: " << mphysicalSystem.GetTimerStep() <<
				" Ship pos: " << shipPtr->GetPos().x << ", " << shipPtr->GetPos().y << ", " <<  shipPtr->GetPos().z <<
				" Ship vel: " << shipPtr->GetPos_dt().x << ", " << shipPtr->GetPos_dt().y << ", " <<  shipPtr->GetPos_dt().z <<
				" energy: " << energy <<
				" time per step: " << myTimerStep() <<
				" time elapsed: " << myTimerTotal() <<
				" Ship force: " << mForceContact.x << ", " << mForceContact.y << ", " <<  mForceContact.z <<
				" ice thickness: " << iceThickness <<
				" number of Iteration: " << numIter << endl;
		cout << "Time: " <<  mphysicalSystem.GetChTime() <<
				" executionTime: " << mphysicalSystem.GetTimerStep() <<
				" Ship vel: " << shipPtr->GetPos_dt().x << ", " << shipPtr->GetPos_dt().y << ", " <<  shipPtr->GetPos_dt().z <<
				" energy: " << energy <<
				" time per step: " << myTimerStep() <<
				" time elapsed: " << myTimerTotal() <<
				" Ship force: " << mForceContact.x << ", " << mForceContact.y << ", " <<  mForceContact.z <<
				" ice thickness: " << iceThickness <<
				" number of Iteration: " << numIter << endl;

		// Save PovRay post-processing data.
		const std::string pov_dir = "povray";
		if (counter == 0) {
			//linux. In windows, it is System instead of system (to invoke a command in the command line)
			system("mkdir -p povray");
			system("rm povray/*.*");
		}
		int stepSave = 50;
		if (write_povray_data && counter % stepSave == 0) {
			char filename[100];
			sprintf(filename, "%s/data_%03d.csv", pov_dir.c_str(), counter / stepSave + 1);
			utils::WriteBodies(&mphysicalSystem, filename);
		}
	}

	outForceData.close();

	// ***************************************************************************************

	DOUBLEPRECISION ? printf("Double Precision\n") : printf("Single Precision\n");
	printf("********************\n");


	thrust::device_vector<real3> posRadD = posRadH;
	thrust::device_vector<real4> velMasD = velMasH;
	thrust::device_vector<real4> rhoPresMuD = rhoPresMuH;
	thrust::device_vector<uint> bodyIndexD = bodyIndex;
	thrust::device_vector<real4> derivVelRhoD;
	ResizeMyThrust4(derivVelRhoD, numObjects.numAllMarkers);
//*********************************************** End of Initialization ****************************************
	int stepEnd = int(paramsH.tFinal/paramsH.dT);//1.0e6;//2.4e6;//600000;//2.4e6 * (.02 * paramsH.sizeScale) / currentParamsH.dT ; //1.4e6 * (.02 * paramsH.sizeScale) / currentParamsH.dT ;//0.7e6 * (.02 * paramsH.sizeScale) / currentParamsH.dT ;//0.7e6;//2.5e6; //200000;//10000;//50000;//100000;
	printf("stepEnd %d\n", stepEnd);
	real_ realTime = 0;

	SimParams paramsH_B = paramsH;
	paramsH_B.bodyForce3 = R3(0);
	paramsH_B.gravity = R3(0);
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
		thrust::host_vector<real3> posRadH2 = posRadH;
		thrust::host_vector<real4> velMasH2 = velMasH;
		thrust::host_vector<real4> rhoPresMuH2 = rhoPresMuH;
		// ** initialize device mid step data
		thrust::device_vector<real3> posRadD2 = posRadD;
		thrust::device_vector<real4> velMasD2 = velMasD;
		thrust::device_vector<real4> rhoPresMuD2 = rhoPresMuD;
		// **
		thrust::device_vector<real3> vel_XSPH_D;
		ResizeMyThrust3(vel_XSPH_D, numObjects.numAllMarkers);

		FillMyThrust4(derivVelRhoD, R4(0));

		IntegrateSPH(posRadD2, velMasD2, rhoPresMuD2, posRadD, velMasD, vel_XSPH_D, rhoPresMuD, bodyIndexD, derivVelRhoD, referenceArray, numObjects, currentParamsH, 0.5 * currentParamsH.dT);
		CopyD2H(posRadH2, velMasH2, rhoPresMuH2, posRadD2, velMasD2, rhoPresMuD2);
		IntegrateSPH(posRadD, velMasD, rhoPresMuD, posRadD2, velMasD2, vel_XSPH_D, rhoPresMuD2, bodyIndexD, derivVelRhoD, referenceArray, numObjects, currentParamsH, currentParamsH.dT);
		CopyD2H(posRadH, velMasH, rhoPresMuH, posRadD, velMasD, rhoPresMuD);


		ClearArraysH(posRadH2, velMasH2, rhoPresMuH2);
		ClearMyThrustR3(posRadD2);
		ClearMyThrustR4(velMasD2);
		ClearMyThrustR4(rhoPresMuD2);
		ClearMyThrustR3(vel_XSPH_D);


		mCpuTimer.Stop();
		myGpuTimer.Stop();
		if (tStep % 50 == 0) {
			printf("step: %d, realTime: %f, step Time (CUDA): %f, step Time (CPU): %f\n ", tStep, realTime, (real_)myGpuTimer.Elapsed(), 1000 * mCpuTimer.Elapsed());
		}
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
