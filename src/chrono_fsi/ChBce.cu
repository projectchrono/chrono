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
// Base class for processing bce forces in fsi system.//
// =============================================================================

#include "chrono_fsi/ChBce.cuh" //for FsiGeneralData

namespace chrono {
namespace fsi {

//--------------------------------------------------------------------------------------------------------------------------------
// updates the rigid body particles
__global__ void UpdateRigidMarkersPositionVelocityD(Real3* posRadD, Real3* velMasD,
		const Real3* rigidSPH_MeshPos_LRF_D, const uint* rigidIdentifierD,
		Real3* posRigidD, Real4* velMassRigidD, Real3* omegaLRF_D, Real4* qD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numObjectsD.numRigid_SphMarkers) {
		return;
	}
	uint rigidMarkerIndex = index + numObjectsD.startRigidMarkers; // updatePortion = [start, end] index of the update portion
	int rigidBodyIndex = rigidIdentifierD[index];

	Real4 q4 = qD[rigidBodyIndex];
	Real3 a1, a2, a3;
	RotationMatirixFromQuaternion(a1, a2, a3, q4);

	Real3 rigidSPH_MeshPos_LRF = rigidSPH_MeshPos_LRF_D[index];

	// position
	Real3 p_Rigid = posRigidD[rigidBodyIndex];
	posRadD[rigidMarkerIndex] = p_Rigid
			+ mR3(dot(a1, rigidSPH_MeshPos_LRF), dot(a2, rigidSPH_MeshPos_LRF),
					dot(a3, rigidSPH_MeshPos_LRF));

	// velocity
	Real4 vM_Rigid = velMassRigidD[rigidBodyIndex];
	Real3 omega3 = omegaLRF_D[rigidBodyIndex];
	Real3 omegaCrossS = cross(omega3, rigidSPH_MeshPos_LRF);
	velMasD[rigidMarkerIndex] =
	mR3(vM_Rigid) + dot(a1, omegaCrossS), dot(a2, omegaCrossS), dot(a3,
			omegaCrossS);
}

//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Calc_Rigid_FSI_ForcesD(Real3* rigid_FSI_ForcesD,
		Real4* totalSurfaceInteractionRigid4) {
	uint rigidSphereA = blockIdx.x * blockDim.x + threadIdx.x;
	if (rigidSphereA >= numObjectsD.numRigidBodies) {
		return;
	}
	Real3 force3 = paramsD.markerMass
			* mR3(totalSurfaceInteractionRigid4[rigidSphereA]);
	rigid_FSI_ForcesD[rigidSphereA] = force3;
}
//--------------------------------------------------------------------------------------------------------------------------------
__global__ void Calc_Markers_TorquesD(Real3* torqueMarkersD,
		Real4* derivVelRhoD, Real3* posRadD, uint* rigidIdentifierD,
		Real3* posRigidD) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= numObjectsD.numRigid_SphMarkers) {
		return;
	}
	uint rigidMarkerIndex = index + numObjectsD.startRigidMarkers;
	Real3 dist3 = Distance(posRadD[rigidMarkerIndex],
			posRigidD[rigidIdentifierD[index]]);
	torqueMarkersD[index] = paramsD.markerMass
			* cross(dist3, mR3(derivVelRhoD[rigidMarkerIndex])); // paramsD.markerMass is multiplied to convert
																 // from SPH acceleration to force
}
//--------------------------------------------------------------------------------------------------------------------------------
ChBce::ChBce(FsiGeneralData* otherFsiGeneralData,
	SimParams* otherParamsH, 
	NumberOfObjects* otherNumObjects) :
			 fsiGeneralData(otherFsiGeneralData), paramsH(otherParamsH), numObjects(otherNumObjects) {
	this->setParameters(paramsH, numObjects);
	totalSurfaceInteractionRigid4.resize(numObjects.numRigidBodies);
	dummyIdentify.resize(numObjects.numRigidBodies);
	torqueMarkersD.resize(numObjects.numRigid_SphMarkers);
}
//--------------------------------------------------------------------------------------------------------------------------------
// applies the time step to the current quantities and saves the new values into variable with the same name and '2' and
// the end
// precondition: for the first step of RK2, all variables with '2' at the end have the values the same as those without
// '2' at the end.
void ChBce::Rigid_Forces_Torques(
		SphMarkerDataD* sphMarkersD,
		FsiBodiesDataD* fsiBodiesD) {
	// Arman: InitSystem has to be called before this point to set the number of objects

	if (numObjects.numRigidBodies == 0) {
		return;
	}
	//################################################### make force and torque arrays
	//####### Force (Acceleration)
	if (totalSurfaceInteractionRigid4.size() != numObjects.numRigidBodies ||
	 dummyIdentify.size() != numObjects.numRigidBodies ||
	 torqueMarkersD.size() != numObjects.numRigidBodies ) {
		throw std::runtime_error ("Error! wrong size: totalSurfaceInteractionRigid4 or torqueMarkersD or dummyIdentify. Thrown from Rigid_Forces_Torques!\n");
	}

	thrust::fill(totalSurfaceInteractionRigid4.begin(), totalSurfaceInteractionRigid4.end(), mR4(0));
	thrust::fill(torqueMarkersD.begin(), torqueMarkersD.end(), mR3(0));

	thrust::equal_to<uint> binary_pred;

	//** forces on BCE markers of each rigid body are accumulated at center. "totalSurfaceInteractionRigid4" is got built.
	(void) thrust::reduce_by_key(fsiGeneralData->rigidIdentifierD.begin(),
			fsiGeneralData->rigidIdentifierD.end(),
			fsiGeneralData->derivVelRhoD.begin() + numObjects.startRigidMarkers,
			dummyIdentify.begin(), totalSurfaceInteractionRigid4.begin(),
			binary_pred, thrust::plus<Real4>());
	thrust::fill(fsiGeneralData->rigid_FSI_ForcesD.begin(), fsiGeneralData->rigid_FSI_ForcesD.end(), mR3(0));

	uint nBlock_UpdateRigid;
	uint nThreads_rigidParticles;
	computeGridSize(numObjects.numRigidBodies, 128, nBlock_UpdateRigid,
			nThreads_rigidParticles);

	//** accumulated BCE forces at center are transformed to acceleration of rigid body "rigid_FSI_ForcesD".
	//"rigid_FSI_ForcesD" gets built.
	Calc_Rigid_FSI_ForcesD<<<nBlock_UpdateRigid, nThreads_rigidParticles>>>(
			mR3CAST(fsiGeneralData->rigid_FSI_ForcesD), mR4CAST(totalSurfaceInteractionRigid4));
	cudaThreadSynchronize();
	cudaCheckError();

	//####### Torque
	uint nBlocks_numRigid_SphMarkers;
	uint nThreads_SphMarkers;
	computeGridSize(numObjects.numRigid_SphMarkers, 256,
			nBlocks_numRigid_SphMarkers, nThreads_SphMarkers);
	

	//** the current position of the rigid, 'posRigidD', is used to calculate the moment of BCE acceleration at the rigid
	//*** body center (i.e. torque/mass). "torqueMarkersD" gets built.
	Calc_Markers_TorquesD<<<nBlocks_numRigid_SphMarkers, nThreads_SphMarkers>>>(
			mR3CAST(torqueMarkersD), mR4CAST(fsiGeneralData->derivVelRhoD), mR3CAST(sphMarkersD->posRadD),
			U1CAST(fsiGeneralData->rigidIdentifierD), mR3CAST(fsiBodiesD->posRigidD));
	cudaThreadSynchronize();
	cudaCheckError();

	(void) thrust::reduce_by_key(fsiGeneralData->rigidIdentifierD.begin(),
			fsiGeneralData->rigidIdentifierD.end(), torqueMarkersD.begin(),
			dummyIdentify.begin(), fsiBodiesD->rigid_FSI_TorquesD.begin(), binary_pred,
			thrust::plus<Real3>());
}
//--------------------------------------------------------------------------------------------------------------------------------
void ChBce::UpdateRigidMarkersPositionVelocity(
	SphMarkerDataD* sphMarkersD,
	FsiBodiesDataD* fsiBodiesD) {

	if (numObjects.numRigidBodies == 0) {
		return;
	}

	uint nBlocks_numRigid_SphMarkers;
	uint nThreads_SphMarkers;
	computeGridSize(numObjects.numRigid_SphMarkers, 256,
			nBlocks_numRigid_SphMarkers, nThreads_SphMarkers);

	// Arman: InitSystem has to be called before this lunch to set numObjectsD

	//################################################### update BCE markers position
	//** "posRadD2"/"velMasD2" associated to BCE markers are updated based on new rigid body (position,
	// orientation)/(velocity, angular velocity)
	UpdateRigidMarkersPositionVelocityD<<<nBlocks_numRigid_SphMarkers,
			nThreads_SphMarkers>>>(mR3CAST(sphMarkersD->posRadD), mR3CAST(sphMarkersD->velMasD),
			mR3CAST(fsiGeneralData->rigidSPH_MeshPos_LRF_D), U1CAST(fsiGeneralData->rigidIdentifierD),
			mR3CAST(fsiBodiesD->posRigid_fsiBodies_D), mR4CAST(fsiBodiesD->velMassRigid_fsiBodies_D), 
			mR3CAST(fsiBodiesD->omegaVelLRF_fsiBodies_D), mR4CAST(fsiBodiesD->q_fsiBodies_D));
	cudaThreadSynchronize();
	cudaCheckError();
}

} // end namespace fsi
} // end namespace chrono