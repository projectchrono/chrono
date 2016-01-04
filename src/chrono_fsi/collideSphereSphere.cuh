///////////////////////////////////////////////////////////////////////////////
//	collideSphereSphere.cuh
//	header file implements kernels and functions for fluid force calculation and update, rigids, and bce
//
//	Created by Arman Pazouki
#ifndef COLLIDESPHERESPHERE_CUH
#define COLLIDESPHERESPHERE_CUH

#include <thrust/device_vector.h>

#include "chrono_fsi/MyStructs.cuh"  //just for SimParams
/**
 * @brief InitSystem
 * @details
 * 			Initializes paramsD and numObjectsD in collideSphereSphere.cu and SDKCollisionSystem.cu
 * 			These two are exactly the same struct as paramsH and numObjects but they are stored in
 * 			the device memory.
 *
 * @param paramsH Parameters that will be used in the system
 * @param numObjects [description]
 */
void InitSystem(SimParams paramsH, NumberOfObjects numObjects);

/**
 * @brief ResizeMyThrust, ResizeMyThrust3, ResizeMyThrust4
 * @details
 * 			This function needs to exist because resizing thrust vectors can only happen in .cu
 * 			files. So in order to resize a thrust vector in a .cpp file, this file will need to
 * 			call a ResizeMyThrust function
 * @param mThrustVec Vector to resize
 * @param mSize Size to resize
 */
void ResizeMyThrust3(thrust::device_vector<Real3>& mThrustVec, int mSize);
void ResizeMyThrust4(thrust::device_vector<Real4>& mThrustVec, int mSize);

/**
 * @brief FillMyThrust, FillMyThrust4
 * @details
 * 			Same explanation as ResizeMyThrust.
 * @param mThrustVec Vector to resize
 * @param v Value to fill thrust vector with.
 */
void FillMyThrust4(thrust::device_vector<Real4>& mThrustVec, Real4 v);

/**
 * @brief ClearMyThrust, ClearMyThrust3, ClearMyThrust4, ClearMyThrustU
 * @details
 * 			Same explanation as ResizeMyThrust.
 * @param mThrustVec Vector to clear
 */
void ClearMyThrustR3(thrust::device_vector<Real3>& mThrustVec);
void ClearMyThrustR4(thrust::device_vector<Real4>& mThrustVec);
void ClearMyThrustU1(thrust::device_vector<uint>& mThrustVec);
void PushBackR3(thrust::device_vector<Real3>& mThrustVec, Real3 a3);
void PushBackR4(thrust::device_vector<Real4>& mThrustVec, Real4 a4);
void ResizeR3(thrust::device_vector<Real3>& mThrustVec, int size);
void ResizeR4(thrust::device_vector<Real4>& mThrustVec, int size);
void ResizeU1(thrust::device_vector<uint>& mThrustVec, int size);

void MakeRigidIdentifier(thrust::device_vector<uint>& rigidIdentifierD,
		int numRigidBodies, int startRigidMarkers,
		const thrust::host_vector<int4>& referenceArray);

void Populate_RigidSPH_MeshPos_LRF(
		thrust::device_vector<uint>& rigidIdentifierD,
		thrust::device_vector<Real3>& rigidSPH_MeshPos_LRF_D,
		const thrust::device_vector<Real3>& posRadD,
		const thrust::device_vector<Real3>& posRigidD,
		const thrust::device_vector<Real4>& qD,
		const thrust::host_vector<int4>& referenceArray,
		const NumberOfObjects& numObjects);

void Rigid_Forces_Torques(thrust::device_vector<Real3>& rigid_FSI_ForcesD,
		thrust::device_vector<Real3>& rigid_FSI_TorquesD,

		const thrust::device_vector<Real3>& posRadD,
		const thrust::device_vector<Real3>& posRigidD,

		const thrust::device_vector<Real4>& derivVelRhoD,
		const thrust::device_vector<uint>& rigidIdentifierD,

		const NumberOfObjects& numObjects);

void UpdateRigidMarkersPositionVelocity(thrust::device_vector<Real3>& posRadD,
		thrust::device_vector<Real3>& velMasD,
		const thrust::device_vector<Real3>& rigidSPH_MeshPos_LRF_D,
		const thrust::device_vector<uint>& rigidIdentifierD,
		const thrust::device_vector<Real3>& posRigidD,
		const thrust::device_vector<Real4>& qD,
		const thrust::device_vector<Real4>& velMassRigidD,
		const thrust::device_vector<Real3>& omegaLRF_D,
		NumberOfObjects numObjects);

void UpdateRigidMarkersPositionVelocity(thrust::device_vector<Real3>& posRadD,
		thrust::device_vector<Real3>& velMasD,
		const thrust::device_vector<Real3>& rigidSPH_MeshPos_LRF_D,
		const thrust::device_vector<uint>& rigidIdentifierD,
		const thrust::device_vector<Real3>& posRigidD,
		const thrust::device_vector<Real4>& qD,
		const thrust::device_vector<Real4>& velMassRigidD,
		const thrust::device_vector<Real3>& omegaLRF_D,
		NumberOfObjects numObjects,
		SimParams paramsH);

/**
 * @brief Calculates the force on each particles
 * @details
 * 			Algorithm:
 *          1. Build neighbor list of each particle. These are the particles that are within the
 *          interaction radius (HSML).
 *          2. Calculate interaction force between:
 *          	- fluid-fluid
 *          	- fluid-solid
 *          	- solid-fluid
 *          3. Calculates forces from other SPH or solid particles as well as boundaries.
 *
 * @param &posRadD
 * @param &velMasD
 * @param &vel_XSPH_D
 * @param &rhoPresMuD
 * @param &bodyIndexD
 * @param &derivVelRhoD
 * @param &referenceArray
 * @param &numObjects
 * @param paramsH These are the simulation parameters which were set in the initialization part.
 * @param dT Time step
 */
void ForceSPH(thrust::device_vector<Real3>& posRadD,
		thrust::device_vector<Real3>& velMasD,
		thrust::device_vector<Real3>& vel_XSPH_D,
		thrust::device_vector<Real4>& rhoPresMuD,

		thrust::device_vector<uint>& bodyIndexD,
		thrust::device_vector<Real4>& derivVelRhoD,
		const thrust::host_vector<int4>& referenceArray,
		const NumberOfObjects& numObjects, SimParams paramsH,
		BceVersion bceType, Real dT);

void ForceSPH_LF(thrust::device_vector<Real3>& posRadD,
		thrust::device_vector<Real3>& velMasD,
		thrust::device_vector<Real4>& rhoPresMuD,

		thrust::device_vector<uint>& bodyIndexD,
		thrust::device_vector<Real4>& derivVelRhoD,
		const thrust::host_vector<int4>& referenceArray,
		const NumberOfObjects& numObjects, SimParams paramsH,
		BceVersion bceType, Real dT);

void DensityReinitialization(thrust::device_vector<Real3>& posRadD,
		thrust::device_vector<Real3>& velMasD,
		thrust::device_vector<Real4>& rhoPresMuD, int numAllMarkers, int3 SIDE);

void IntegrateSPH(thrust::device_vector<Real4>& derivVelRhoD,
		thrust::device_vector<Real3>& posRadD2, thrust::device_vector<Real3>& velMasD2,
		thrust::device_vector<Real4>& rhoPresMuD2,

		thrust::device_vector<Real3>& posRadD,
		thrust::device_vector<Real3>& velMasD,
		thrust::device_vector<Real3>& vel_XSPH_D,
		thrust::device_vector<Real4>& rhoPresMuD,

		thrust::device_vector<uint>& bodyIndexD,
		const thrust::host_vector<int4>& referenceArray,
		const NumberOfObjects& numObjects, SimParams currentParamsH, Real dT);

void cudaCollisions(thrust::host_vector<Real3>& mPosRad,
		thrust::host_vector<Real3>& mVelMas,
		thrust::host_vector<Real4>& mRhoPresMu,
		const thrust::host_vector<uint>& bodyIndex,
		const thrust::host_vector<int4>& referenceArray,

		SimParams paramsH, NumberOfObjects numObjects);

#endif
