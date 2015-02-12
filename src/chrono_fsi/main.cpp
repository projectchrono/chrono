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
int2 CreateFluidMarkers(thrust::host_vector<real3> & mPosRad,
		thrust::host_vector<real4> & mVelMas,
		thrust::host_vector<real4> & mRhoPresMu,
		thrust::host_vector<real3> & mPosRadBoundary,
		thrust::host_vector<real4> & mVelMasBoundary,
		thrust::host_vector<real4> & mRhoPresMuBoundary,
		real_ & sphMarkerMass) {
//	printf("\n\n\nStraightChannelBoundaries: \n   min: %f %f %f\n   max %f %f %f\n\n\n", paramsH.straightChannelBoundaryMin.x, paramsH.straightChannelBoundaryMin.y, paramsH.straightChannelBoundaryMin.z,
//			paramsH.straightChannelBoundaryMax.x, paramsH.straightChannelBoundaryMax.y, paramsH.straightChannelBoundaryMax.z);

	//real2 rad2 = .5 * R2(paramsH.cMax.y - paramsH.cMin.y, paramsH.cMax.z - paramsH.cMin.z);
	//channelRadius = (rad2.x < rad2.y) ? rad2.x : rad2.y;
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
						mPosRad.push_back(posRad);
						real3 v3 = R3(0);
						mVelMas.push_back(R4(v3, sphMarkerMass));
						mRhoPresMu.push_back(R4(paramsH.rho0, paramsH.BASEPRES, paramsH.mu0, -1));
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
	return I2(num_FluidMarkers, num_BoundaryMarkers);
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

void ClearArrays(
	const thrust::host_vector<real3> & mPosRad, //do not set the size here since you are using push back later
	const thrust::host_vector<real4> & mVelMas,
	const thrust::host_vector<real4> & mRhoPresMu,
	const thrust::host_vector<uint> & bodyIndex,
	const thrust::host_vector<int3> & referenceArray) {

	mPosRad.clear();
	mVelMas.clear();
	mRhoPresMu.clear();
	bodyIndex.clear();
	referenceArray.clear()
}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
int main() {
	//****************************************************************************************
	time_t rawtime;
	struct tm * timeinfo;

	//(void) cudaSetDevice(0);

	time(&rawtime);
	timeinfo = localtime(&rawtime);
	printf("Job was submittet at date/time is: %s\n", asctime(timeinfo));

	//*** Arrays definition
	thrust::host_vector<int3> referenceArray;
	thrust::host_vector<real3> mPosRad; //do not set the size here since you are using push back later
	thrust::host_vector<real4> mVelMas;
	thrust::host_vector<real4> mRhoPresMu;
	thrust::host_vector<uint> bodyIndex;

	thrust::host_vector<real3> mPosRadBoundary; //do not set the size here since you are using push back later
	thrust::host_vector<real4> mVelMasBoundary;
	thrust::host_vector<real4> mRhoPresMuBoundary;

	SetupParamsH(paramsH);

	NumberOfObjects numObjects;
	//** default num markers
	int numAllMarkers = 0;

//**********************************************************************
	// initialize fluid particles
	real_ sphMarkerMass; // To be initialized in CreateFluidMarkers, and used in other places
	int2 num_fluidOrBoundaryMarkers = CreateFluidMarkers(mPosRad, mVelMas,
			mRhoPresMu, mPosRadBoundary, mVelMasBoundary,
			mRhoPresMuBoundary, sphMarkerMass);
	referenceArray.push_back(I3(0, num_fluidOrBoundaryMarkers.x, -1)); //map fluid -1
	numAllMarkers += num_fluidOrBoundaryMarkers.x;

	mPosRad.resize(
			num_fluidOrBoundaryMarkers.x + num_fluidOrBoundaryMarkers.y);
	mVelMas.resize(
			num_fluidOrBoundaryMarkers.x + num_fluidOrBoundaryMarkers.y);
	mRhoPresMu.resize(
			num_fluidOrBoundaryMarkers.x + num_fluidOrBoundaryMarkers.y);
	////boundary: type = 0
	//printf("size1 %d, %d , numpart %d, numFluid %d \n", mPosRadBoundary.end() - mPosRadBoundary.begin(), mPosRadBoundary.size(), num_fluidOrBoundaryMarkers.y, num_fluidOrBoundaryMarkers.x);
	thrust::copy(mPosRadBoundary.begin(), mPosRadBoundary.end(),
			mPosRad.begin() + num_fluidOrBoundaryMarkers.x);
	thrust::copy(mVelMasBoundary.begin(), mVelMasBoundary.end(),
			mVelMas.begin() + num_fluidOrBoundaryMarkers.x);
	thrust::copy(mRhoPresMuBoundary.begin(), mRhoPresMuBoundary.end(),
			mRhoPresMu.begin() + num_fluidOrBoundaryMarkers.x);
	mPosRadBoundary.clear();
	mVelMasBoundary.clear();
	mRhoPresMuBoundary.clear();

	bodyIndex.resize(numAllMarkers);
	thrust::fill(bodyIndex.begin(), bodyIndex.end(), 1);
	thrust::exclusive_scan(bodyIndex.begin(), bodyIndex.end(),
			bodyIndex.begin());

	referenceArray.push_back(
			I3(numAllMarkers, numAllMarkers + num_fluidOrBoundaryMarkers.y,	0));
	numAllMarkers += num_fluidOrBoundaryMarkers.y;

	// set num objects
	SetNumObjects(numObjects, referenceArray, numAllMarkers);
	if (numObjects.numAllMarkers == 0) {
		ClearArrays(mPosRad, mVelMas, mRhoPresMu, bodyIndex, referenceArray);
		return 0;
	}

	DOUBLEPRECISION ? printf("Double Precision\n") : printf("Single Precision\n");
	printf("********************\n");


	thrust::device_vector<real3> posRadD	;
	thrust::device_vector<real4> velMasD	;
	thrust::device_vector<real4> rhoPresMuD	;
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


		mCpuTimer.Stop();
		if (tStep % 50 == 0) {
			printf("step: %d, realTime: %f, step Time (CUDA): %f, step Time (CPU): %f\n ", tStep, realTime, time2, 1000 * mCpuTimer.Elapsed());
		}
		fflush(stdout);
		realTime += currentParamsH.dT;
	}





	if (numObjects.numAllMarkers != 0) {
		cudaCollisions(mPosRad, mVelMas, mRhoPresMu, bodyIndex, referenceArray,	paramsH, numObjects);
	}
	ClearArrays(mPosRad, mVelMas, mRhoPresMu, bodyIndex, referenceArray);
	return 0;
}
