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
// Base class for processing sph force in fsi system.//
// =============================================================================
#include "ChFsiForceParallel.h"
using namespace fsi;
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
	int numRigidAndBoundaryMarkers = fsiData->referenceArray[2 + numObjects.numRigidBodies - 1].y - fsiData->referenceArray[0].y;
	if ((numObjects.numBoundaryMarkers + numObjects.numRigid_SphMarkers) != numRigidAndBoundaryMarkers) {
		throw std::runtime_error ("Error! number of rigid and boundary markers are saved incorrectly. Thrown from ModifyBceVelocity!\n");
	}
	int2 updatePortion = mI2(referenceArray[0].y, referenceArray[2 + numObjects.numRigidBodies - 1].y);
	fsiData->velMas_ModifiedBCE.resize(numRigidAndBoundaryMarkers);
	fsiData->rhoPreMu_ModifiedBCE.resize(numRigidAndBoundaryMarkers);
	if (paramsH.bceType == ADAMI) {
		thrust::device_vector<Real3> bceAcc(numObjects.numRigid_SphMarkers);
		if (numObjects.numRigid_SphMarkers > 0) {
			CalcBceAcceleration(bceAcc, fsiData->q_fsiBodies_D, fsiData->accRigid_fsiBodies_D, fsiData->omegaVelLRF_fsiBodies_D,
					fsiData->omegaAccLRF_fsiBodies_D, fsiData->rigidSPH_MeshPos_LRF_D, fsiData->rigidIdentifierD, numObjects.numRigid_SphMarkers);
		}
		RecalcSortedVelocityPressure_BCE(fsiData->velMas_ModifiedBCE, fsiData->rhoPreMu_ModifiedBCE,
				fsiData->m_dSortedPosRad, fsiData->m_dSortedVelMas, fsiData->m_dSortedRhoPreMu, fsiData->m_dCellStart, fsiData->m_dCellEnd, 
				fsiData->mapOriginalToSorted, bceAcc, updatePortion);
		bceAcc.clear();
	} else {
		thrust::copy(fsiData->velMasD.begin() + updatePortion.x, fsiData->velMasD.begin() + updatePortion.y, fsiData->velMas_ModifiedBCE.begin());
		thrust::copy(fsiData->rhoPresMuD.begin() + updatePortion.x, fsiData->rhoPresMuD.begin() + updatePortion.y, fsiData->rhoPreMu_ModifiedBCE.begin());
	}

}

void ChFsiForceParallel::ForceSPH() {
	fsiCollisionSystem->ArrangeData();

}