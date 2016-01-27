// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Arman Pazouki
// =============================================================================
//
// Class for handling time integration in fsi system.//
// =============================================================================
#include "ChFsiForceParallel.h"
using namespace fsi;

//--------------------------------------------------------------------------------------------------------------------------------
// collide a particle against all other particles in a given cell
__device__ Real3 deltaVShare(int3 gridPos, uint index, Real3 posRadA,
		Real3 velMasA, Real4 rhoPresMuA, Real3* sortedPosRad,
		Real3* sortedVelMas, Real4* sortedRhoPreMu, uint* cellStart,
		uint* cellEnd) {
	uint gridHash = calcGridHash(gridPos);
	// get start of bucket for this cell
	Real3 deltaV = mR3(0.0f);

	uint startIndex = FETCH(cellStart, gridHash);
	if (startIndex != 0xffffffff) {  // cell is not empty
		// iterate over particles in this cell
		uint endIndex = FETCH(cellEnd, gridHash);

		for (uint j = startIndex; j < endIndex; j++) {
			if (j != index) {  // check not colliding with self
				Real3 posRadB = FETCH(sortedPosRad, j);
				Real3 dist3 = Distance(posRadA, posRadB);
				Real d = length(dist3);
				if (d > RESOLUTION_LENGTH_MULT * paramsD.HSML)
					continue;
				Real4 rhoPresMuB = FETCH(sortedRhoPreMu, j);
				if (rhoPresMuB.w > -.1)
					continue; //# B must be fluid (A was checked originally and it is fluid at this point), accoring to
				// colagrossi (2003), the other phase (i.e. rigid) should not be considered)
				Real multRho = 2.0f / (rhoPresMuA.x + rhoPresMuB.x);
				Real3 velMasB = FETCH(sortedVelMas, j);
				deltaV += paramsD.markerMass * (velMasB - velMasA) * W3(d)
						* multRho;
			}
		}
	}
	return deltaV;
}
//--------------------------------------------------------------------------------------------------------------------------------
// collide a particle against all other particles in a given cell
__device__ Real4 collideCell(int3 gridPos, uint index, Real3 posRadA,
		Real3 velMasA, Real3 vel_XSPH_A, Real4 rhoPresMuA, Real3* sortedPosRad,
		Real3* sortedVelMas, Real3* vel_XSPH_Sorted_D, Real4* sortedRhoPreMu,
		Real3* velMas_ModifiedBCE, Real4* rhoPreMu_ModifiedBCE, uint* gridMarkerIndex,
		uint* cellStart, uint* cellEnd) {

	uint gridHash = calcGridHash(gridPos);
	// get start of bucket for this cell
	Real4 derivVelRho = mR4(0);

	uint startIndex = FETCH(cellStart, gridHash);
	if (startIndex == 0xffffffff) { // cell is not empty
		return derivVelRho;
	}
	// iterate over particles in this cell
	uint endIndex = FETCH(cellEnd, gridHash);

	for (uint j = startIndex; j < endIndex; j++) {
		if (j != index) {  // check not colliding with self
			Real3 posRadB = FETCH(sortedPosRad, j);
			Real3 dist3Alpha = posRadA - posRadB;
//			Real3 dist3 = Distance(posRadA, posRadB);
			Real3 dist3 = Modify_Local_PosB(posRadB, posRadA);
			Real d = length(dist3);
			if (d > RESOLUTION_LENGTH_MULT * paramsD.HSML)
				continue;

			Real4 rhoPresMuB = FETCH(sortedRhoPreMu, j);
//			// old version. When rigid-rigid contact used to be handled within fluid
//			if ((fabs(rhoPresMuB.w - rhoPresMuA.w) < .1)
//					&& rhoPresMuA.w > -.1) {
//				continue;
//			}
			if (rhoPresMuA.w > -.1 && rhoPresMuB.w > -.1) { // no rigid-rigid force
				continue;
			}

			modifyPressure(rhoPresMuB, dist3Alpha);
			Real3 velMasB = FETCH(sortedVelMas, j);
			if (rhoPresMuB.w > -.1) {
				int bceIndexB = gridMarkerIndex[j] - (numObjectsD.numFluidMarkers);
				if (!(bceIndexB >= 0 && bceIndexB < numObjectsD.numBoundaryMarkers + numObjectsD.numRigid_SphMarkers)) {
					printf("Error! bceIndex out of bound, collideD !\n");
				}
				rhoPresMuB = rhoPreMu_ModifiedBCE[bceIndexB];
				velMasB = velMas_ModifiedBCE[bceIndexB];
			}
			Real multViscosit = 1;
			Real4 derivVelRhoAB = mR4(0.0f);
			Real3 vel_XSPH_B = FETCH(vel_XSPH_Sorted_D, j);
			derivVelRhoAB = DifVelocityRho(dist3, d, posRadA, posRadB, velMasA, vel_XSPH_A,
					velMasB, vel_XSPH_B, rhoPresMuA, rhoPresMuB,
					multViscosit);
			derivVelRho += derivVelRhoAB;
		}
	}

	// ff1
	//	if (rhoPresMuA.w > 0) printf("force value %f %f %f\n", 1e20*derivV.x, 1e20*derivV.y, 1e20*derivV.z);
	return derivVelRho;
} 
//--------------------------------------------------------------------------------------------------------------------------------
// calculate marker acceleration, required in ADAMI
__global__ void calcBceAcceleration_kernel(
		Real3* bceAcc,
		Real4* q_fsiBodies_D,
		Real3* accRigid_fsiBodies_D,
		Real3* omegaVelLRF_fsiBodies_D,
		Real3* omegaAccLRF_fsiBodies_D,
		Real3* rigidSPH_MeshPos_LRF_D,
		const uint* rigidIdentifierD) {
	uint bceIndex = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;
	if (bceIndex >= numObjectsD.numRigid_SphMarkers) {
		return;
	}

	int rigidBodyIndex = rigidIdentifierD[bceIndex];
	Real3 acc3 = accRigid_fsiBodies_D[rigidBodyIndex]; // linear acceleration (CM)

	Real4 q4 = q_fsiBodies_D[rigidBodyIndex];
	Real3 a1, a2, a3;
	RotationMatirixFromQuaternion(a1, a2, a3, q4);
	Real3 wVel3 = omegaVelLRF_fsiBodies_D[rigidBodyIndex];
	Real3 rigidSPH_MeshPos_LRF = rigidSPH_MeshPos_LRF_D[bceIndex];
	Real3 wVelCrossS = cross(wVel3, rigidSPH_MeshPos_LRF);
	Real3 wVelCrossWVelCrossS = cross(wVel3, wVelCrossS);
	acc3 += dot(a1, wVelCrossWVelCrossS), dot(a2, wVelCrossWVelCrossS), dot(a3,
			wVelCrossWVelCrossS); 						// centrigugal acceleration

	Real3 wAcc3 = omegaAccLRF_fsiBodies_D[rigidBodyIndex];
	Real3 wAccCrossS = cross(wAcc3, rigidSPH_MeshPos_LRF);
	acc3 += dot(a1, wAccCrossS), dot(a2, wAccCrossS), dot(a3,
			wAccCrossS); 								// tangential acceleration

//	printf("linear acc %f %f %f point acc %f %f %f \n", accRigid3.x, accRigid3.y, accRigid3.z, acc3.x, acc3.y, acc3.z);
	bceAcc[bceIndex] = acc3;
}

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void newVel_XSPH_D(Real3* vel_XSPH_Sorted_D,  // output: new velocity
		Real3* sortedPosRad,       // input: sorted positions
		Real3* sortedVelMas,       // input: sorted velocities
		Real4* sortedRhoPreMu, uint* gridMarkerIndex, // input: sorted particle indices
		uint* cellStart, uint* cellEnd, uint numAllMarkers, volatile bool *isErrorD) {
	uint index = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;
	if (index >= numAllMarkers)
		return;

	// read particle data from sorted arrays

	Real4 rhoPreMuA = FETCH(sortedRhoPreMu, index);
	Real3 velMasA = FETCH(sortedVelMas, index);
	if (rhoPreMuA.w > -0.1) { // v_XSPH is calculated only for fluid markers. Keep unchanged if not fluid.
		vel_XSPH_Sorted_D[index] = velMasA;
		return;
	}

	Real3 posRadA = FETCH(sortedPosRad, index);
	Real3 deltaV = mR3(0);

	// get address in grid
	int3 gridPos = calcGridPos(posRadA);

	/// if (gridPos.x == paramsD.gridSize.x-1) printf("****aha %d %d\n", gridPos.x, paramsD.gridSize.x);

	// examine neighbouring cells
	for (int z = -1; z <= 1; z++) {
		for (int y = -1; y <= 1; y++) {
			for (int x = -1; x <= 1; x++) {
				int3 neighbourPos = gridPos + mI3(x, y, z);
				deltaV += deltaVShare(neighbourPos, index, posRadA, velMasA,
						rhoPreMuA, sortedPosRad, sortedVelMas, sortedRhoPreMu,
						cellStart, cellEnd);
			}
		}
	}
	//   // write new velocity back to original unsorted location
	// sortedVel_XSPH[index] = velMasA + paramsD.EPS_XSPH * deltaV;

	// write new velocity back to original unsorted location
	// uint originalIndex = gridMarkerIndex[index];
	Real3 vXSPH = velMasA + paramsD.EPS_XSPH * deltaV;
	if (!(isfinite(vXSPH.x) && isfinite(vXSPH.y)
			&& isfinite(vXSPH.z))) {
		printf("Error! particle vXSPH is NAN: thrown from SDKCollisionSystem.cu, newVel_XSPH_D !\n");
		*isErrorD = true;
	}
	vel_XSPH_Sorted_D[index] = vXSPH;
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void collideD(Real4* sortedDerivVelRho_fsi_D,  // output: new velocity
		Real3* sortedPosRad,  // input: sorted positions
		Real3* sortedVelMas,  // input: sorted velocities
		Real3* vel_XSPH_Sorted_D, Real4* sortedRhoPreMu,
		Real3* velMas_ModifiedBCE, Real4* rhoPreMu_ModifiedBCE, uint* gridMarkerIndex,
		uint* cellStart, uint* cellEnd, uint numAllMarkers, volatile bool *isErrorD) {

	uint index = __mul24(blockIdx.x, blockDim.x) + threadIdx.x;
	if (index >= numAllMarkers)
		return;

	// read particle data from sorted arrays
	Real3 posRadA = FETCH(sortedPosRad, index);
	Real3 velMasA = FETCH(sortedVelMas, index);
	Real4 rhoPreMuA = FETCH(sortedRhoPreMu, index);


	// *** comment these couple of lines since we don't want the force on the rigid (or boundary) be influenced by ADAMi
	// *** method since it would cause large forces. ADAMI method is used only to calculate forces on the fluid markers (A)
	// *** near the boundary or rigid (B).
//	if (rhoPreMuA.w > -.1) {
//		int bceIndex = gridMarkerIndex[index] - (numObjectsD.numFluidMarkers);
//		if (!(bceIndex >= 0 && bceIndex < numObjectsD.numBoundaryMarkers + numObjectsD.numRigid_SphMarkers)) {
//			printf("Error! bceIndex out of bound, collideD !\n");
//			*isErrorD = true;
//		}
//		rhoPreMuA = rhoPreMu_ModifiedBCE[bceIndex];
//		velMasA = velMas_ModifiedBCE[bceIndex];
//	}

//	uint originalIndex = gridMarkerIndex[index];
	Real3 vel_XSPH_A = vel_XSPH_Sorted_D[index];
	Real4 derivVelRho = sortedDerivVelRho_fsi_D[index];

	// get address in grid
	int3 gridPos = calcGridPos(posRadA);

	// examine neighbouring cells
	for (int x = -1; x <= 1; x++) {
		for (int y = -1; y <= 1; y++) {
			for (int z = -1; z <= 1; z++) {
				derivVelRho += collideCell(gridPos + mI3(x, y, z), index,
						posRadA, velMasA, vel_XSPH_A, rhoPreMuA, sortedPosRad,
						sortedVelMas, vel_XSPH_Sorted_D, sortedRhoPreMu,
						velMas_ModifiedBCE, rhoPreMu_ModifiedBCE, gridMarkerIndex,
						cellStart, cellEnd);
			}
		}
	}

	// write new velocity back to original unsorted location
	// *** let's tweak a little bit :)
	if (!(isfinite(derivVelRho.x) && isfinite(derivVelRho.y)
			&& isfinite(derivVelRho.z) )) {
		printf("Error! particle derivVel is NAN: thrown from SDKCollisionSystem.cu, collideD !\n");
		*isErrorD = true;
	}
	if (!(isfinite(derivVelRho.w))) {
		printf("Error! particle derivRho is NAN: thrown from SDKCollisionSystem.cu, collideD !\n");
		*isErrorD = true;
	}
	sortedDerivVelRho_fsi_D[index] = derivVelRho;
}
//--------------------------------------------------------------------------------------------------------------------------------
// use invasive to avoid one extra copy. However, keep in mind that sorted is changed.
void ChFsiForceParallel::CopySortedToOriginal_Invasive_R3(thrust::device_vector<Real3>& original,
		thrust::device_vector<Real3>& sorted,
		const thrust::device_vector<uint>& gridMarkerIndex) {
	thrust::device_vector<uint> dummyMarkerIndex = gridMarkerIndex;
	thrust::sort_by_key(dummyMarkerIndex.begin(), dummyMarkerIndex.end(),
			sorted.begin());
	dummyMarkerIndex.clear();
	thrust::copy(sorted.begin(), sorted.end(), original.begin());
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChFsiForceParallel::CopySortedToOriginal_NonInvasive_R3(thrust::device_vector<Real3>& original,
		thrust::device_vector<Real3>& sorted,
		const thrust::device_vector<uint>& gridMarkerIndex) {
	thrust::device_vector<Real3> dummySorted = sorted;
	CopySortedToOriginal_Invasive_R3(original, dummySorted, gridMarkerIndex);
}
//--------------------------------------------------------------------------------------------------------------------------------
// use invasive to avoid one extra copy. However, keep in mind that sorted is changed.
void ChFsiForceParallel::CopySortedToOriginal_Invasive_R4(thrust::device_vector<Real4>& original,
		thrust::device_vector<Real4>& sorted,
		const thrust::device_vector<uint>& gridMarkerIndex) {
	thrust::device_vector<uint> dummyMarkerIndex = gridMarkerIndex;
	thrust::sort_by_key(dummyMarkerIndex.begin(), dummyMarkerIndex.end(),
			sorted.begin());
	dummyMarkerIndex.clear();
	thrust::copy(sorted.begin(), sorted.end(), original.begin());
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChFsiForceParallel::CopySortedToOriginal_NonInvasive_R4(thrust::device_vector<Real4>& original,
		thrust::device_vector<Real4>& sorted,
		const thrust::device_vector<uint>& gridMarkerIndex) {
	thrust::device_vector<Real4> dummySorted = sorted;
	CopySortedToOriginal_Invasive_R4(original, dummySorted, gridMarkerIndex);
}
//--------------------------------------------------------------------------------------------------------------------------------

void ChFsiForceParallel::ChFsiForceParallel(
	SphMarkerDataD * otherSortedSphMarkersD,
	ProximityDataD * otherMarkersProximityD,
	FsiGeneralData * otherFsiGeneralData,
	SimParams* otherParamsH, 
	NumberOfObjects* otherNumObjects)
: sortedSphMarkersD(otherSortedSphMarkersD), markersProximityD(otherMarkersProximityD), fsiGeneralData(otherFsiGeneralData), 
paramsH(otherParamsH), numObjects(otherNumObjects) {

	fsiCollisionSystem = new ChCollisionSystemFsi(sortedSphMarkersD, markersProximityD, paramsH, numObjects);
	this->setParameters(paramsH, numObjects);

	sphMarkersD = NULL;
	fsiBodiesD = NULL;
}

//--------------------------------------------------------------------------------------------------------------------------------

void ChFsiForceParallel::CalcBceAcceleration(
		thrust::device_vector<Real3>& bceAcc,
		const thrust::device_vector<Real4>& q_fsiBodies_D,
		const thrust::device_vector<Real3>& accRigid_fsiBodies_D,
		const thrust::device_vector<Real3>& omegaVelLRF_fsiBodies_D,
		const thrust::device_vector<Real3>& omegaAccLRF_fsiBodies_D,
		const thrust::device_vector<Real3>& rigidSPH_MeshPos_LRF_D,
		const thrust::device_vector<uint>& rigidIdentifierD,
		int numRigid_SphMarkers) {

	// thread per particle
	uint numThreads, numBlocks;
	computeGridSize(numRigid_SphMarkers, 64, numBlocks, numThreads);

	calcBceAcceleration_kernel<<<numBlocks, numThreads>>>(mR3CAST(bceAcc),
			mR4CAST(q_fsiBodies_D), mR3CAST(accRigid_fsiBodies_D), mR3CAST(omegaVelLRF_fsiBodies_D), mR3CAST(omegaAccLRF_fsiBodies_D),
			mR3CAST(rigidSPH_MeshPos_LRF_D), U1CAST(rigidIdentifierD));

	cudaThreadSynchronize();
	cudaCheckError();
}
//--------------------------------------------------------------------------------------------------------------------------------

void ChFsiForceParallel::ModifyBceVelocity() {
	// modify BCE velocity and pressure
	int numRigidAndBoundaryMarkers = fsiGeneralData->referenceArray[2 + numObjects.numRigidBodies - 1].y - fsiGeneralData->referenceArray[0].y;
	if ((numObjects.numBoundaryMarkers + numObjects.numRigid_SphMarkers) != numRigidAndBoundaryMarkers) {
		throw std::runtime_error ("Error! number of rigid and boundary markers are saved incorrectly. Thrown from ModifyBceVelocity!\n");
	}
	if (!(velMas_ModifiedBCE.size() == numRigidAndBoundaryMarkers && rhoPreMu_ModifiedBCE.size() == numRigidAndBoundaryMarkers)) {
		throw std::runtime_error ("Error! size error velMas_ModifiedBCE and rhoPreMu_ModifiedBCE. Thrown from ModifyBceVelocity!\n");
	}
	int2 updatePortion = mI2(fsiGeneralData->referenceArray[0].y, fsiGeneralData->referenceArray[2 + numObjects.numRigidBodies - 1].y);
	if (paramsH.bceType == ADAMI) {
		thrust::device_vector<Real3> bceAcc(numObjects.numRigid_SphMarkers);
		if (numObjects.numRigid_SphMarkers > 0) {
			CalcBceAcceleration(bceAcc, fsiBodiesD->q_fsiBodies_D, fsiBodiesD->accRigid_fsiBodies_D, fsiBodiesD->omegaVelLRF_fsiBodies_D,
					fsiBodiesD->omegaAccLRF_fsiBodies_D, fsiGeneralData->rigidSPH_MeshPos_LRF_D, fsiGeneralData->rigidIdentifierD, numObjects.numRigid_SphMarkers);
		}
		RecalcSortedVelocityPressure_BCE(velMas_ModifiedBCE, rhoPreMu_ModifiedBCE,
				sortedSphMarkersD->posRadD, sortedSphMarkersD->velMasD, sortedSphMarkersD->rhoPresMuD, rhoPresMuD->cellStartD, rhoPresMuD->cellEndD, 
				rhoPresMuD->mapOriginalToSorted, bceAcc, updatePortion);
		bceAcc.clear();
	} else {
		thrust::copy(sphMarkersD->velMasD.begin() + updatePortion.x, sphMarkersD->velMasD.begin() + updatePortion.y, velMas_ModifiedBCE.begin());
		thrust::copy(sphMarkersD->rhoPresMuD.begin() + updatePortion.x, sphMarkersD->rhoPresMuD.begin() + updatePortion.y, rhoPreMu_ModifiedBCE.begin());
	}

}
//--------------------------------------------------------------------------------------------------------------------------------
void ChFsiForceParallel::RecalcVelocity_XSPH(thrust::device_vector<Real3>& vel_XSPH_Sorted_D,
		thrust::device_vector<Real3>& sortedPosRad,
		thrust::device_vector<Real3>& sortedVelMas,
		thrust::device_vector<Real4>& sortedRhoPreMu,
		thrust::device_vector<uint>& gridMarkerIndex,
		thrust::device_vector<uint>& cellStart,
		thrust::device_vector<uint>& cellEnd, uint numAllMarkers,
		uint numCells) {

	bool *isErrorH, *isErrorD;
	isErrorH = (bool *)malloc(sizeof(bool));
	cudaMalloc((void**) &isErrorD, sizeof(bool));
	*isErrorH = false;
	cudaMemcpy(isErrorD, isErrorH, sizeof(bool), cudaMemcpyHostToDevice);
	//------------------------------------------------------------------------
	/* thread per particle */
	uint numThreads, numBlocks;
	computeGridSize(numAllMarkers, 64, numBlocks, numThreads);

	/* Execute the kernel */
	newVel_XSPH_D<<<numBlocks, numThreads>>>(mR3CAST(vel_XSPH_Sorted_D),
			mR3CAST(sortedPosRad), mR3CAST(sortedVelMas),
			mR4CAST(sortedRhoPreMu), U1CAST(gridMarkerIndex), U1CAST(cellStart),
			U1CAST(cellEnd), numAllMarkers, isErrorD);

	cudaThreadSynchronize();
	cudaCheckError();
	//------------------------------------------------------------------------
	cudaMemcpy(isErrorH, isErrorD, sizeof(bool), cudaMemcpyDeviceToHost);
	if (*isErrorH == true) {
		throw std::runtime_error ("Error! program crashed in  newVel_XSPH_D!\n");
	}
	cudaFree(isErrorD);
	free(isErrorH);
}
//--------------------------------------------------------------------------------------------------------------------------------

void ChFsiForceParallel::CalculateXSPH_velocity() {	
	/* Calculate vel_XSPH */
	if (vel_XSPH_Sorted_D.size() != numAllMarkers) {
		throw std::runtime_error ("Error! size error vel_XSPH_Sorted_D Thrown from CalculateXSPH_velocity!\n");
	}
	RecalcVelocity_XSPH(vel_XSPH_Sorted_D, sortedSphMarkersD->posRadD, sortedSphMarkersD->velMasD,
			sortedSphMarkersD->rhoPresMuD, rhoPresMuD->gridMarkerIndexD, rhoPresMuD->cellStartD, rhoPresMuD->cellEndD,
			paramsH.numAllMarkers, paramsH.m_numGridCells);

	/* Collide */
	/* Initialize derivVelRhoD with zero. NECESSARY. */


}

//--------------------------------------------------------------------------------------------------------------------------------

/**
 * @brief Wrapper function for collide
 * @details
 * 		See SDKCollisionSystem.cuh for informaton on collide
 */
void ChFsiForceParallel::collide(thrust::device_vector<Real4>& sortedDerivVelRho_fsi_D,
		thrust::device_vector<Real3>& sortedPosRad,
		thrust::device_vector<Real3>& sortedVelMas,
		thrust::device_vector<Real3>& vel_XSPH_Sorted_D,
		thrust::device_vector<Real4>& sortedRhoPreMu,
		thrust::device_vector<Real3>& velMas_ModifiedBCE,
		thrust::device_vector<Real4>& rhoPreMu_ModifiedBCE,

		thrust::device_vector<uint>& gridMarkerIndex,
		thrust::device_vector<uint>& cellStart,
		thrust::device_vector<uint>& cellEnd, uint numAllMarkers, uint numCells) {

	bool *isErrorH, *isErrorD;
	isErrorH = (bool *)malloc(sizeof(bool));
	cudaMalloc((void**) &isErrorD, sizeof(bool));
	*isErrorH = false;
	cudaMemcpy(isErrorD, isErrorH, sizeof(bool), cudaMemcpyHostToDevice);
	//------------------------------------------------------------------------
	// thread per particle
	uint numThreads, numBlocks;
	computeGridSize(numAllMarkers, 64, numBlocks, numThreads);

	// execute the kernel
	collideD<<<numBlocks, numThreads>>>(mR4CAST(sortedDerivVelRho_fsi_D),
			mR3CAST(sortedPosRad), mR3CAST(sortedVelMas),
			mR3CAST(vel_XSPH_Sorted_D), mR4CAST(sortedRhoPreMu),
			mR3CAST(velMas_ModifiedBCE), mR4CAST(rhoPreMu_ModifiedBCE), U1CAST(gridMarkerIndex),
			U1CAST(cellStart), U1CAST(cellEnd),
			numAllMarkers, isErrorD);

	cudaThreadSynchronize();
	cudaCheckError();
	//------------------------------------------------------------------------
	cudaMemcpy(isErrorH, isErrorD, sizeof(bool), cudaMemcpyDeviceToHost);
	if (*isErrorH == true) {
		throw std::runtime_error ("Error! program crashed in  collideD!\n");
	}
	cudaFree(isErrorD);
	free(isErrorH);


//					// unroll sorted index to have the location of original particles in the sorted arrays
//					thrust::device_vector<uint> dummyIndex = gridMarkerIndex;
//					thrust::sort_by_key(dummyIndex.begin(), dummyIndex.end(),
//							derivVelRhoD.begin());
//					dummyIndex.clear();

}
//--------------------------------------------------------------------------------------------------------------------------------

void ChFsiForceParallel::CollideWrapper() {
	thrust::device_vector<Real4> m_dSortedDerivVelRho_fsi_D(numAllMarkers); // Store Rho, Pressure, Mu of each particle in the device memory
	thrust::fill(m_dSortedDerivVelRho_fsi_D.begin(), m_dSortedDerivVelRho_fsi_D.end(), mR4(0));

	collide(m_dSortedDerivVelRho_fsi_D, sortedSphMarkersD->posRadD, sortedSphMarkersD->velMasD, vel_XSPH_Sorted_D,
			sortedSphMarkersD->rhoPresMuD, velMas_ModifiedBCE, rhoPreMu_ModifiedBCE, rhoPresMuD->gridMarkerIndexD, 
			rhoPresMuD->cellStartD, rhoPresMuD->cellEndD,
			paramsH.numAllMarkers, paramsH.m_numGridCells);

	CopySortedToOriginal_Invasive_R3(fsiGeneralData->vel_XSPH_D, vel_XSPH_Sorted_D, rhoPresMuD->gridMarkerIndexD);
	CopySortedToOriginal_Invasive_R4(fsiGeneralData->derivVelRhoD, m_dSortedDerivVelRho_fsi_D, rhoPresMuD->gridMarkerIndexD);

	m_dSortedDerivVelRho_fsi_D.clear();
	// vel_XSPH_Sorted_D.clear();
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChFsiForceParallel::AddGravityToFluid() {
	// add gravity to fluid markers
	/* Add outside forces. Don't add gravity to rigids, BCE, and boundaries, it is added in ChSystem */
	Real3 totalFluidBodyForce3 = paramsH.bodyForce3 + paramsH.gravity;
	thrust::device_vector<Real4> bodyForceD(numAllMarkers);
	thrust::fill(bodyForceD.begin(), bodyForceD.end(), mR4(totalFluidBodyForce3));
	thrust::transform(fsiGeneralData->derivVelRhoD.begin() + fsiGeneralData->referenceArray[0].x, fsiGeneralData->derivVelRhoD.begin() + fsiGeneralData->referenceArray[0].y,
			bodyForceD.begin(), fsiGeneralData->derivVelRhoD.begin() + fsiGeneralData->referenceArray[0].x, thrust::plus<Real4>());
	bodyForceD.clear();
}
//--------------------------------------------------------------------------------------------------------------------------------

void ChFsiForceParallel::ForceSPH(
		SphMarkerDataD * otherSphMarkersD,
		FsiBodiesDataD * otherFsiBodiesD) {
	// Arman: Change this function by getting in the arrays of the current stage: useful for RK2. array pointers need to be private members
	sphMarkersD = otherSphMarkersD;
	fsiBodiesD = otherFsiBodiesD;

	fsiCollisionSystem->ArrangeData(sphMarkersD);
	ModifyBceVelocity();
	CalculateXSPH_velocity();
	CollideWrapper();
	AddGravityToFluid();
}