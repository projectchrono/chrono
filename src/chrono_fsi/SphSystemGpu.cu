/*
 * SphSystemGpu.cpp
 *
 *  Created on: Jan 22, 2015
 *      Author: arman
 */

#include "SphSystemGpu.h"

SphSystemGpu::SphSystemGpu() {
	ResizeData();
}

SphSystemGpu::SphSystemGpu(SphSet * sphSetH, const SimParams & par, const NumberOfObjects & numObj)  :
	paramsH(par), numObjects(numObj) {
	ResizeData();
	CopySphH2D_External(sphSetH);
}

void SphSystemGpu::CopySphH2D_External(SphSet * sphSetH) {
	ResizeDeviceData_fromHost();

	thrust::copy(sphSetH->posRadH.begin(), sphSetH->posRadH.end(), posRad1D.begin());
	thrust::copy(sphSetH->velMasH.begin(), sphSetH->velMasH.end(), velMas1D.begin());
	thrust::copy(sphSetH->rhoPresMuH.begin(), sphSetH->rhoPresMuH.end(), rhoPresMu1D.begin());
	thrust::copy(sphSetH->markerTypeH.begin(), sphSetH->markerTypeH.end(), markerTypeD.begin());
	thrust::copy(sphSetH->bodyIndexH.begin(), sphSetH->bodyIndexH.end(), bodyIndexD.begin());
}

void SphSystemGpu::CopySphH2D_Internal() {
	CopySphH2D_External((SphSet*)this);
}

void SphSystemGpu::CopySphD2H_Internal() {
	ResizeHostData_fromDevice();

	thrust::copy(posRad1D.begin(), posRad1D.end(), posRadH.begin());
	thrust::copy(velMas1D.begin(), velMas1D.end(), velMasH.begin());
	thrust::copy(rhoPresMu1D.begin(), rhoPresMu1D.end(), rhoPresMuH.begin());
	thrust::copy(markerTypeD.begin(), markerTypeD.end(), markerTypeH.begin());
	thrust::copy(bodyIndexD.begin(), bodyIndexD.end(), bodyIndexH.begin());
}

SphSystemGpu::~SphSystemGpu() {
}

void SphSystemGpu::MacrosAssignPrimary() {
#ifdef posRadD
#undef posRadD
#define posRadD posRad1D
#endif

#ifdef posRadD2
#undef posRadD2
#define posRadD2 posRad2D
#endif

#ifdef velMasD
#undef velMasD
#define velMasD velMas1D
#endif

#ifdef velMasD2
#undef velMasD2
#define velMasD2 velMas2D
#endif

#ifdef rhoPresMuD
#undef rhoPresMuD
#define rhoPresMuD rhoPresMu1D
#endif

#ifdef rhoPresMuD2
#undef rhoPresMuD2
#define rhoPresMuD2 rhoPresMu2D
#endif
}

void SphSystemGpu::MacrosAssignSecondary() {
#ifdef posRadD
#undef posRadD
#define posRadD posRad2D
#endif

#ifdef posRadD2
#undef posRadD2
#define posRadD2 posRad1D
#endif

#ifdef velMasD
#undef velMasD
#define velMasD velMas2D
#endif

#ifdef velMasD2
#undef velMasD2
#define velMasD2 velMas1D
#endif

#ifdef rhoPresMuD
#undef rhoPresMuD
#define rhoPresMuD rhoPresMu2D
#endif

#ifdef rhoPresMuD2
#undef rhoPresMuD2
#define rhoPresMuD2 rhoPresMu1D
#endif
}

SphSystemGpu::ResizeData() {
	int nM = numObjects.numAllMarkers;
	// host data
	posRadH.resize(nM);
	velMasH.resize(nM);
	rhoPresMuH.resize(nM);
	markerTypeH.resize(nM);
	bodyIndexH.resize(nM);

	// device data
	posRad1D.resize(nM);
	velMas1D.resize(nM);
	rhoPresMu1D.resize(nM);
	markerTypeD.resize(nM);
	bodyIndexD.resize(nM);

	vel_XSPH_D.resize(nM);

	// device data intermediate state
	posRad2D.resize(nM);
	velMas2D.resize(nM);
	rhoPresMu2D.resize(nM);

	// device data sorted arrays
	m_dSortedPosRad.resize(nM);
	m_dSortedVelMas.resize(nM);
	m_dSortedRhoPreMu.resize(nM);
	vel_XSPH_Sorted_D.resize(nM);

	// force
	derivVelRhoD(nM);

	// proximity computation data
	m_numGridCells = paramsH.gridSize.x * paramsH.gridSize.y * paramsH.gridSize.z;;
	m_dCellStart.resize(m_numGridCells);
	m_dCellEnd.resize(m_numGridCells);
	m_dGridMarkerHash.resize(nM);
	m_dGridMarkerIndex.resize(nM);
    mapOriginalToSorted.resize(nM);
}


//void SphSystemGpu::ResizeHostData_fromDevice() {
//	int size = posRad1D.size();
//	posRadH.resize(size);
//	velMasH.resize(size);
//	rhoPresMuH.resize(size);
//	markerTypeH.resize(size);
//	bodyIndexH.resize(size);
//}
//
//void SphSystemGpu::ResizeDeviceData_fromHost() {
//	int size = posRadH.size();
//	posRad1D.resize(size);
//	velMas1D.resize(size);
//	rhoPresMu1D.resize(size);
//	markerTypeD.resize(size);
//	bodyIndexD.resize(size);
//}


//*******************************************************************************************************************************
//builds the neighbors' list of each particle and finds the force on each particle
//calculates the interaction force between 1- fluid-fluid, 2- fluid-solid, 3- solid-fluid particles
//calculates forces from other SPH or solid particles, as wall as boundaries
void SphSystemGpu::ForceSPH(
		const thrust::host_vector<int3> & referenceArray,
		real_ & maxStress,
		real_ dT) {
	// Part1: contact detection #########################################################################################################################
	calcHash(U1CAST(m_dGridMarkerHash), U1CAST(m_dGridMarkerIndex), R3CAST(posRadD), numAllMarkers);
	thrust::sort_by_key(m_dGridMarkerHash.begin(), m_dGridMarkerHash.end(), m_dGridMarkerIndex.begin());

	// reorder particle arrays into sorted order and find start and end of each cell
	reorderDataAndFindCellStart(U1CAST(m_dCellStart), U1CAST(m_dCellEnd), R3CAST(m_dSortedPosRad), R4CAST(m_dSortedVelMas), R4CAST(m_dSortedRhoPreMu), U1CAST(m_dGridMarkerHash),
			U1CAST(m_dGridMarkerIndex), U1CAST(mapOriginalToSorted), TCAST(posRadD), R4CAST(velMasD), R4CAST(rhoPresMuD), numAllMarkers, m_numGridCells);

	//process collisions
	real3 totalFluidBodyForce3 = paramsH.bodyForce3 + paramsH.gravity;
	thrust::fill(derivVelRhoD.begin(), derivVelRhoD.end(), R4(0)); //initialize derivVelRhoD with zero. necessary
//	GpuTimer myT1;
//	myT1.Start();
	thrust::fill(derivVelRhoD.begin() + referenceArray[0].x, derivVelRhoD.begin() + referenceArray[0].y, R4(totalFluidBodyForce3)); //add body force to fluid particles.
//	myT1.Stop();
//	real_ t1 = (real_)myT1.Elapsed();
//	printf("(1) *** fill timer %f, array size %d\n", t1, referenceArray[0].y - referenceArray[0].x);

	RecalcVelocity_XSPH(R3CAST(vel_XSPH_Sorted_D), R3CAST(m_dSortedPosRad), R4CAST(m_dSortedVelMas), R4CAST(m_dSortedRhoPreMu), U1CAST(m_dGridMarkerIndex), U1CAST(m_dCellStart),
			U1CAST(m_dCellEnd), numAllMarkers, m_numGridCells);

	collide(R4CAST(derivVelRhoD), R3CAST(m_dSortedPosRad), R4CAST(m_dSortedVelMas), R3CAST(vel_XSPH_Sorted_D), R4CAST(m_dSortedRhoPreMu), R3CAST(posRigidD), I1CAST(rigidIdentifierD), U1CAST(m_dGridMarkerIndex), U1CAST(m_dCellStart),
			U1CAST(m_dCellEnd), numAllMarkers, m_numGridCells, dT);

	uint nBlock_NumSpheres, nThreads_SphMarkers;
	computeGridSize(numAllMarkers, 256, nBlock_NumSpheres, nThreads_SphMarkers);
	Copy_SortedVelXSPH_To_VelXSPH(R3CAST(vel_XSPH_D), R3CAST(vel_XSPH_Sorted_D), U1CAST(m_dGridMarkerIndex), numObjects.numAllMarkers);
	cudaThreadSynchronize();
	CUT_CHECK_ERROR("Kernel execution failed: Copy_SortedVelXSPH_To_VelXSPH");

	// set the pressure and density of BC and BCE markers to those of the nearest fluid marker.
	// I put it here to use the already determined proximity computation
	//********************************************************************************************************************************
//	ProjectDensityPressureToBCandBCE(R4CAST(rhoPresMuD), R3CAST(m_dSortedPosRad), R4CAST(m_dSortedRhoPreMu),
//				U1CAST(m_dGridMarkerIndex), U1CAST(m_dCellStart), U1CAST(m_dCellEnd), numAllMarkers);
	//********************************************************************************************************************************
	//*********************** Calculate MaxStress on Particles ***********************************************************************
	thrust::device_vector<real3> devStressD(numObjects.numRigid_SphMarkers + numObjects.numFlex_SphMarkers);
	thrust::device_vector<real3> volStressD(numObjects.numRigid_SphMarkers + numObjects.numFlex_SphMarkers);
	thrust::device_vector<real4> mainStressD(numObjects.numRigid_SphMarkers + numObjects.numFlex_SphMarkers);
	int numBCE = numObjects.numRigid_SphMarkers + numObjects.numFlex_SphMarkers;
	CalcBCE_Stresses(R3CAST(devStressD), R3CAST(volStressD), R4CAST(mainStressD), R3CAST(m_dSortedPosRad), R4CAST(m_dSortedVelMas), R4CAST(m_dSortedRhoPreMu),
			U1CAST(mapOriginalToSorted), U1CAST(m_dCellStart), U1CAST(m_dCellEnd),numBCE);

	real4 maxStress4 = R4(0);
	if (numBCE > 0) {
		maxStress4 = *(thrust::max_element(mainStressD.begin(), mainStressD.end(), MaxReal4W()));
	}
	maxStress = maxStress4.w;

	devStressD.clear();
	volStressD.clear();
	mainStressD.clear();
	//********************************************************************************************************************************
}






