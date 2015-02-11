#include "custom_cutil_math.h"
#include "contactForces.cuh"
#include "SPHCudaUtils.h"
#include <thrust/sort.h>
#include <thrust/scan.h>
#include <thrust/reduce.h>
#include <thrust/extrema.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include "SDKCollisionSystem.cuh"
#include "collideSphereSphere.cuh"
#include "FlexibleBodies.cuh"
#include "printToFile.cuh"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <sstream>
#include <fstream>
using namespace std;
//#####################################################################################
#define B_SIZE 128
//#####################################################################################
__constant__ real_ dTD;
__constant__ int2 updatePortionD;

//--------------------------------------------------------------------------------------------------------------------------------
void MapSPH_ToGrid(
		real_ resolution,
		int3 & cartesianGridDims,
		thrust::host_vector<real4> & rho_Pres_CartH,
		thrust::host_vector<real4> & vel_VelMag_CartH,
		thrust::device_vector<real3> & posRadD,
		thrust::device_vector<real4> & velMasD,
		thrust::device_vector<real4> & rhoPresMuD,
		int numAllMarkers,
		SimParams paramsH) {
//	real3* m_dSortedPosRad;
//	real4* m_dSortedVelMas;
//	real4* m_dSortedRhoPreMu;
//	uint* m_dCellStart; // index of start of each cell in sorted list
//	uint* m_dCellEnd; // index of end of cell

	int3 SIDE = paramsH.gridSize;
	uint m_numGridCells = SIDE.x * SIDE.y * SIDE.z; //m_gridSize = SIDE
	//TODO here

	// calculate grid hash
	thrust::device_vector<real3> m_dSortedPosRad(numAllMarkers);
	thrust::device_vector<real4> m_dSortedVelMas(numAllMarkers);
	thrust::device_vector<real4> m_dSortedRhoPreMu(numAllMarkers);

	thrust::device_vector<uint> m_dGridMarkerHash(numAllMarkers);
	thrust::device_vector<uint> m_dGridMarkerIndex(numAllMarkers);

	thrust::device_vector<uint> mapOriginalToSorted(numAllMarkers);

	thrust::device_vector<uint> m_dCellStart(m_numGridCells);
	thrust::device_vector<uint> m_dCellEnd(m_numGridCells);

	// calculate grid hash
	calcHash(m_dGridMarkerHash, m_dGridMarkerIndex, posRadD, numAllMarkers);

	thrust::sort_by_key(m_dGridMarkerHash.begin(), m_dGridMarkerHash.end(), m_dGridMarkerIndex.begin());

	// reorder particle arrays into sorted order and find start and end of each cell
	reorderDataAndFindCellStart(m_dCellStart, m_dCellEnd, m_dSortedPosRad, m_dSortedVelMas, m_dSortedRhoPreMu, m_dGridMarkerHash,
			m_dGridMarkerIndex, mapOriginalToSorted, posRadD, velMasD, rhoPresMuD, numAllMarkers, m_numGridCells);

	//real_ resolution = 8 * paramsH.markerRadius;
	cartesianGridDims = I3(paramsH.boxDims / resolution) + I3(1);
//	printf("^^^ bodDim %f %f %f, GridDim %d %d %d, resolution %f \n", paramsH.boxDims.x, paramsH.boxDims.y, paramsH.boxDims.z, cartesianGridDims.x,
//			cartesianGridDims.y, cartesianGridDims.z, resolution);
	uint cartesianGridSize = cartesianGridDims.x * cartesianGridDims.y * cartesianGridDims.z;
	thrust::device_vector<real4> rho_Pres_CartD(cartesianGridSize);
	thrust::device_vector<real4> vel_VelMag_CartD(cartesianGridSize);

	CalcCartesianData(rho_Pres_CartD, vel_VelMag_CartD, m_dSortedPosRad, m_dSortedVelMas, m_dSortedRhoPreMu,
			m_dGridMarkerIndex, m_dCellStart, m_dCellEnd, cartesianGridSize, cartesianGridDims, resolution);

//	freeArray(m_dSortedPosRad);
//	freeArray(m_dSortedVelMas);
//	freeArray(m_dSortedRhoPreMu);
	m_dSortedPosRad.clear();
	m_dSortedVelMas.clear();

	m_dSortedRhoPreMu.clear();

	m_dGridMarkerHash.clear();
	m_dGridMarkerIndex.clear();
	mapOriginalToSorted.clear();

//	freeArray(m_dCellStart);
//	freeArray(m_dCellEnd);
	m_dCellStart.clear();
	m_dCellEnd.clear();

	rho_Pres_CartH.resize(cartesianGridSize);
	vel_VelMag_CartH.resize(cartesianGridSize);
	thrust::copy(rho_Pres_CartD.begin(), rho_Pres_CartD.end(), rho_Pres_CartH.begin());
	thrust::copy(vel_VelMag_CartD.begin(), vel_VelMag_CartD.end(), vel_VelMag_CartH.begin());

	rho_Pres_CartD.clear();
	vel_VelMag_CartD.clear();
}
//*******************************************************************************************************************************
//builds the neighbors' list of each particle and finds the force on each particle
//calculates the interaction force between 1- fluid-fluid, 2- fluid-solid, 3- solid-fluid particles
//calculates forces from other SPH or solid particles, as wall as boundaries
void ForceSPH(
		thrust::device_vector<real3> & posRadD,
		thrust::device_vector<real4> & velMasD,
		thrust::device_vector<real3> & vel_XSPH_D,
		thrust::device_vector<real4> & rhoPresMuD,

		thrust::device_vector<uint> & bodyIndexD,
		thrust::device_vector<real4> & derivVelRhoD,
		const thrust::host_vector<int3> & referenceArray,
		const NumberOfObjects & numObjects,
		SimParams paramsH,
		real_ dT) {
	// Part1: contact detection #########################################################################################################################
	// grid data for sorting method
//	real3* m_dSortedPosRad;
//	real4* m_dSortedVelMas;
//	real4* m_dSortedRhoPreMu;
//	uint* m_dCellStart; // index of start of each cell in sorted list
//	uint* m_dCellEnd; // index of end of cell

	uint m_numGridCells = paramsH.gridSize.x * paramsH.gridSize.y * paramsH.gridSize.z; //m_gridSize = SIDE
	//TODO here

	int numAllMarkers = numObjects.numAllMarkers;
	// calculate grid hash
	thrust::device_vector<real3> m_dSortedPosRad(numAllMarkers);
	thrust::device_vector<real4> m_dSortedVelMas(numAllMarkers);
	thrust::device_vector<real4> m_dSortedRhoPreMu(numAllMarkers);
	thrust::device_vector<real3> vel_XSPH_Sorted_D(numAllMarkers);

	thrust::device_vector<uint> m_dGridMarkerHash(numAllMarkers);
	thrust::device_vector<uint> m_dGridMarkerIndex(numAllMarkers);

	thrust::device_vector<uint> mapOriginalToSorted(numAllMarkers);

	thrust::device_vector<uint> m_dCellStart(m_numGridCells);
	thrust::device_vector<uint> m_dCellEnd(m_numGridCells);
	// calculate grid hash
	calcHash(m_dGridMarkerHash, m_dGridMarkerIndex, posRadD, numAllMarkers);

//	GpuTimer myT0;
//	myT0.Start();
	thrust::sort_by_key(m_dGridMarkerHash.begin(), m_dGridMarkerHash.end(), m_dGridMarkerIndex.begin());
//	myT0.Stop();
//	real_ t0 = (real_)myT0.Elapsed();
//	printf("(0) ** Sort by key timer %f, array size %d\n", t0, m_dGridMarkerHash.size());


	// reorder particle arrays into sorted order and find start and end of each cell
	reorderDataAndFindCellStart(m_dCellStart, m_dCellEnd, m_dSortedPosRad, m_dSortedVelMas, m_dSortedRhoPreMu, m_dGridMarkerHash,
			m_dGridMarkerIndex, mapOriginalToSorted, posRadD, velMasD, rhoPresMuD, numAllMarkers, m_numGridCells);

	//process collisions
	real3 totalFluidBodyForce3 = paramsH.bodyForce3 + paramsH.gravity;
	thrust::fill(derivVelRhoD.begin(), derivVelRhoD.end(), R4(0)); //initialize derivVelRhoD with zero. necessary
//	GpuTimer myT1;
//	myT1.Start();
	thrust::fill(derivVelRhoD.begin() + referenceArray[0].x, derivVelRhoD.begin() + referenceArray[0].y, R4(totalFluidBodyForce3)); //add body force to fluid particles.
//	myT1.Stop();
//	real_ t1 = (real_)myT1.Elapsed();
//	printf("(1) *** fill timer %f, array size %d\n", t1, referenceArray[0].y - referenceArray[0].x);

	RecalcVelocity_XSPH(vel_XSPH_Sorted_D, m_dSortedPosRad, m_dSortedVelMas, m_dSortedRhoPreMu, m_dGridMarkerIndex, m_dCellStart,
			m_dCellEnd, numAllMarkers, m_numGridCells);

	collide(derivVelRhoD, m_dSortedPosRad, m_dSortedVelMas, vel_XSPH_Sorted_D, m_dSortedRhoPreMu, m_dGridMarkerIndex, m_dCellStart,
			m_dCellEnd, numAllMarkers, m_numGridCells, dT);


	Copy_SortedVelXSPH_To_VelXSPH(vel_XSPH_D, vel_XSPH_Sorted_D, m_dGridMarkerIndex, numAllMarkers);


	// set the pressure and density of BC and BCE markers to those of the nearest fluid marker.
	// I put it here to use the already determined proximity computation
	//********************************************************************************************************************************
//	ProjectDensityPressureToBCandBCE(rhoPresMuD, m_dSortedPosRad, m_dSortedRhoPreMu,
//				m_dGridMarkerIndex, m_dCellStart, m_dCellEnd, numAllMarkers);
	//********************************************************************************************************************************
	//*********************** Calculate MaxStress on Particles ***********************************************************************
	thrust::device_vector<real3> devStressD(numObjects.numRigid_SphMarkers + numObjects.numFlex_SphMarkers);
	thrust::device_vector<real3> volStressD(numObjects.numRigid_SphMarkers + numObjects.numFlex_SphMarkers);
	thrust::device_vector<real4> mainStressD(numObjects.numRigid_SphMarkers + numObjects.numFlex_SphMarkers);
	int numBCE = numObjects.numRigid_SphMarkers + numObjects.numFlex_SphMarkers;
	CalcBCE_Stresses(devStressD, volStressD, mainStressD, m_dSortedPosRad, m_dSortedVelMas, m_dSortedRhoPreMu,
			mapOriginalToSorted, m_dCellStart, m_dCellEnd,numBCE);

	devStressD.clear();
	volStressD.clear();
	mainStressD.clear();
	//********************************************************************************************************************************
	m_dSortedPosRad.clear();
	m_dSortedVelMas.clear();
	m_dSortedRhoPreMu.clear();
	vel_XSPH_Sorted_D.clear();

	m_dGridMarkerHash.clear();
	m_dGridMarkerIndex.clear();

	mapOriginalToSorted.clear();

	m_dCellStart.clear();
	m_dCellEnd.clear();
}
//--------------------------------------------------------------------------------------------------------------------------------
void DensityReinitialization(
		thrust::device_vector<real3> & posRadD,
		thrust::device_vector<real4> & velMasD,
		thrust::device_vector<real4> & rhoPresMuD,
		int numAllMarkers,
		int3 SIDE) {
//	real3* m_dSortedPosRad;
//	real4* m_dSortedVelMas;
//	real4* m_dSortedRhoPreMu;
//	uint* m_dCellStart; // index of start of each cell in sorted list
//	uint* m_dCellEnd; // index of end of cell

	uint m_numGridCells = SIDE.x * SIDE.y * SIDE.z; //m_gridSize = SIDE
	//TODO here

	// calculate grid hash
	thrust::device_vector<real3> m_dSortedPosRad(numAllMarkers);
	thrust::device_vector<real4> m_dSortedVelMas(numAllMarkers);
	thrust::device_vector<real4> m_dSortedRhoPreMu(numAllMarkers);

	thrust::device_vector<uint> m_dGridMarkerHash(numAllMarkers);
	thrust::device_vector<uint> m_dGridMarkerIndex(numAllMarkers);

	thrust::device_vector<uint> mapOriginalToSorted(numAllMarkers);

	thrust::device_vector<uint> m_dCellStart(m_numGridCells);
	thrust::device_vector<uint> m_dCellEnd(m_numGridCells);

	// calculate grid hash
	calcHash(m_dGridMarkerHash, m_dGridMarkerIndex, posRadD, numAllMarkers);

	thrust::sort_by_key(m_dGridMarkerHash.begin(), m_dGridMarkerHash.end(), m_dGridMarkerIndex.begin());

	// reorder particle arrays into sorted order and find start and end of each cell
	reorderDataAndFindCellStart(m_dCellStart, m_dCellEnd, m_dSortedPosRad, m_dSortedVelMas, m_dSortedRhoPreMu, m_dGridMarkerHash,
			m_dGridMarkerIndex, mapOriginalToSorted, posRadD, velMasD, rhoPresMuD, numAllMarkers, m_numGridCells);

	ReCalcDensity(posRadD, velMasD, rhoPresMuD, m_dSortedPosRad, m_dSortedVelMas, m_dSortedRhoPreMu,
			m_dGridMarkerIndex, m_dCellStart, m_dCellEnd, numAllMarkers);

	m_dSortedPosRad.clear();
	m_dSortedVelMas.clear();
	m_dSortedRhoPreMu.clear();

	m_dGridMarkerHash.clear();
	m_dGridMarkerIndex.clear();

	mapOriginalToSorted.clear();

	m_dCellStart.clear();
	m_dCellEnd.clear();
}

//##############################################################################################################################################
// the main function, which updates the particles and implements BC
void cudaCollisions(
		thrust::host_vector<real3> & mPosRad,
		thrust::host_vector<real4> & mVelMas,
		thrust::host_vector<real4> & mRhoPresMu,
		const thrust::host_vector<uint> & bodyIndex,
		const thrust::host_vector<int3> & referenceArray,

		SimParams paramsH,
		NumberOfObjects & numObjects) {
	//****************************** bin size adjustement and contact detection stuff *****************************
	int3 SIDE = I3(int((paramsH.cMax.x - paramsH.cMin.x) / paramsH.binSize0 + .1), int((paramsH.cMax.y - paramsH.cMin.y) / paramsH.binSize0 + .1),
			int((paramsH.cMax.z - paramsH.cMin.z) / paramsH.binSize0 + .1));
	real_ mBinSize = paramsH.binSize0; //Best solution in that case may be to change cMax or cMin such that periodic sides be a multiple of binSize

	printf("SIDE: %d, %d, %d\n", SIDE.x, SIDE.y, SIDE.z);
	//**********************************************************************************************************
	paramsH.gridSize = SIDE;
	//paramsH.numCells = SIDE.x * SIDE.y * SIDE.z;
	paramsH.worldOrigin = paramsH.cMin;
	paramsH.cellSize = R3(mBinSize, mBinSize, mBinSize);
	printf("boxDims: %f, %f, %f\n", paramsH.boxDims.x, paramsH.boxDims.y, paramsH.boxDims.z);

	setParameters(&paramsH, &numObjects);// sets paramsD in SDKCollisionSystem
	setParameters2(&paramsH, &numObjects);// sets paramsD in contactForces
	cutilSafeCall( cudaMemcpyToSymbolAsync(paramsD, &paramsH, sizeof(SimParams))); 	//sets paramsD for this file
	cutilSafeCall( cudaMemcpyToSymbolAsync(numObjectsD, &numObjects, sizeof(NumberOfObjects)));
	//*************************************************************************************************************
	//--------- initialization ---------------
	//cudaError_t dumDevErr = cudaSetDevice(2);
	GpuTimer myTotalTime;
	myTotalTime.Start();
	//printf("cMin.x, y, z, CMAx.x, y, z, binSize %f %f %f , %f %f %f, %f\n", paramsH.cMin.x, paramsH.cMin.y, paramsH.cMin.z, paramsH.cMax.x, paramsH.cMax.y, paramsH.cMax.z, paramsH.binSize0);
	cudaDeviceSetCacheConfig(cudaFuncCachePreferL1);

	thrust::device_vector<real3> posRadD=mPosRad;
	thrust::device_vector<real4> velMasD=mVelMas;
	thrust::device_vector<real4> rhoPresMuD=mRhoPresMu;

	thrust::device_vector<uint> bodyIndexD=bodyIndex;
	thrust::device_vector<real4> derivVelRhoD(numObjects.numAllMarkers);



	//******************************************************************************
	int stepEnd = int(paramsH.tFinal/paramsH.dT);//1.0e6;//2.4e6;//600000;//2.4e6 * (.02 * paramsH.sizeScale) / currentParamsH.dT ; //1.4e6 * (.02 * paramsH.sizeScale) / currentParamsH.dT ;//0.7e6 * (.02 * paramsH.sizeScale) / currentParamsH.dT ;//0.7e6;//2.5e6; //200000;//10000;//50000;//100000;
	printf("stepEnd %d\n", stepEnd);

	real_ delTOrig = paramsH.dT;
	real_ realTime = 0;

	SimParams paramsH_B = paramsH;
	paramsH_B.bodyForce3 = R3(0);
	paramsH_B.gravity = R3(0);
	paramsH_B.dT = .1 * paramsH.dT;

	printf("\ntimePause %f, numPause %d\n", paramsH.timePause, int(paramsH.timePause/paramsH_B.dT));
	printf("paramsH.timePauseRigidFlex %f, numPauseRigidFlex %d\n\n", paramsH.timePauseRigidFlex, int((paramsH.timePauseRigidFlex-paramsH.timePause)/paramsH.dT + paramsH.timePause/paramsH_B.dT));

	SimParams currentParamsH = paramsH;
	for (int tStep = 0; tStep < stepEnd + 1; tStep++) {
		//************************************************
		PrintToFile(posRadD, velMasD, rhoPresMuD, referenceArray, currentParamsH, tStep);
		//************

		GpuTimer myGpuTimer;
		myGpuTimer.Start();

		// Arman timer Added
		CpuTimer mCpuTimer;
		mCpuTimer.Start();

		//***********
		if (realTime <= paramsH.timePause) 	{
			currentParamsH = paramsH_B;
		} else {
			currentParamsH = paramsH;
		}
		//***********

		setParameters(&currentParamsH, &numObjects);// sets paramsD in SDKCollisionSystem
		setParameters2(&currentParamsH, &numObjects);// sets paramsD in contactForces
		cutilSafeCall( cudaMemcpyToSymbolAsync(paramsD, &currentParamsH, sizeof(SimParams))); 	//sets paramsD for this file

		//computations
				//markers
		thrust::device_vector<real3> posRadD2 = posRadD;
		thrust::device_vector<real4> velMasD2 = velMasD;
		thrust::device_vector<real4> rhoPresMuD2 = rhoPresMuD;
		thrust::device_vector<real3> vel_XSPH_D(numObjects.numAllMarkers);


		//******** RK2
		ForceSPH(posRadD, velMasD, vel_XSPH_D, rhoPresMuD, bodyIndexD, derivVelRhoD, referenceArray, numObjects, currentParamsH, 0.5 * currentParamsH.dT); //?$ right now, it does not consider paramsH.gravity or other stuff on rigid bodies. they should be applied at rigid body solver
		UpdateFluid(posRadD2, velMasD2, vel_XSPH_D, rhoPresMuD2, derivVelRhoD, referenceArray, 0.5 * currentParamsH.dT); //assumes ...D2 is a copy of ...D
			//UpdateBoundary(posRadD2, velMasD2, rhoPresMuD2, derivVelRhoD, referenceArray, 0.5 * currentParamsH.dT);		//assumes ...D2 is a copy of ...D
		ApplyBoundarySPH_Markers(posRadD2, rhoPresMuD2, numObjects.numAllMarkers);
		//*****
		ForceSPH(posRadD2, velMasD2, vel_XSPH_D, rhoPresMuD2, bodyIndexD, derivVelRhoD, referenceArray, numObjects, currentParamsH, currentParamsH.dT);
		UpdateFluid(posRadD, velMasD, vel_XSPH_D, rhoPresMuD, derivVelRhoD, referenceArray, currentParamsH.dT);
			//UpdateBoundary(posRadD, velMasD, rhoPresMuD, derivVelRhoD, referenceArray, currentParamsH.dT);
		ApplyBoundarySPH_Markers(posRadD, rhoPresMuD, numObjects.numAllMarkers);
		//************
		posRadD2.clear();
		velMasD2.clear();
		rhoPresMuD2.clear();
		vel_XSPH_D.clear();

		//density re-initialization
//		if ((tStep % 10 == 0) && (paramsH.densityReinit != 0)) {
//			DensityReinitialization(posRadD, velMasD, rhoPresMuD, numObjects.numAllMarkers, SIDE); //does not work for analytical boundaries (non-meshed) and free surfaces
//		}

		myGpuTimer.Stop();
		real_ time2 = (real_)myGpuTimer.Elapsed();

		//cudaDeviceSynchronize();

		//Arman timer Added
		mCpuTimer.Stop();

		if (tStep % 50 == 0) {
			printf("step: %d, realTime: %f, step Time (CUDA): %f, step Time (CPU): %f\n ", tStep, realTime, time2, 1000 * mCpuTimer.Elapsed());

//			// ************ calc and print cartesian data ************************************
//			int3 cartesianGridDims;
//			thrust::host_vector<real4> rho_Pres_CartH(1);
//			thrust::host_vector<real4> vel_VelMag_CartH(1);
//			MapSPH_ToGrid(2 * paramsH.HSML, cartesianGridDims, rho_Pres_CartH, vel_VelMag_CartH,
//					posRadD, velMasD, rhoPresMuD, numObjects.numAllMarkers, paramsH);
//			PrintCartesianData_MidLine(rho_Pres_CartH, vel_VelMag_CartH, cartesianGridDims, paramsH);
//			// *******************************************************************************
		}
		fflush(stdout);

		realTime += currentParamsH.dT;

		//_CrtDumpMemoryLeaks(); //for memory leak detection (msdn suggestion for VS) apparently does not work in conjunction with cuda

	}

	//you may copy back to host
	posRadD.clear();
	velMasD.clear();
	rhoPresMuD.clear();

	bodyIndexD.clear();
	derivVelRhoD.clear();

	myTotalTime.Stop();
	real_ time = (real_)myTotalTime.Elapsed();
	printf("total Time: %f\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n ", time);
}
