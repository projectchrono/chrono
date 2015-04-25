#ifndef COLLIDESPHERESPHERE_CUH
#define COLLIDESPHERESPHERE_CUH

#include <thrust/device_vector.h>

#include "MyStructs.cuh" //just for SimParams

void InitSystem(
		SimParams paramsH,
		NumberOfObjects numObjects);
//******
void ResizeMyThrust3(thrust::device_vector<Real3> & mThrustVec, int mSize);
void ResizeMyThrust4(thrust::device_vector<Real4> & mThrustVec, int mSize);
//******
void FillMyThrust4(thrust::device_vector<Real4> & mThrustVec, Real4 v);
//******
void ClearMyThrustR3(thrust::device_vector<Real3> & mThrustVec);
void ClearMyThrustR4(thrust::device_vector<Real4> & mThrustVec);
void ClearMyThrustU1(thrust::device_vector<uint> & mThrustVec);
//******

void ForceSPH(
		thrust::device_vector<Real3> & posRadD,
		thrust::device_vector<Real4> & velMasD,
		thrust::device_vector<Real3> & vel_XSPH_D,
		thrust::device_vector<Real4> & rhoPresMuD,

		thrust::device_vector<uint> & bodyIndexD,
		thrust::device_vector<Real4> & derivVelRhoD,
		const thrust::host_vector<int3> & referenceArray,
		const NumberOfObjects & numObjects,
		SimParams paramsH,
		BceVersion bceType,
		Real dT);

void IntegrateSPH(
		thrust::device_vector<Real3> & posRadD2,
		thrust::device_vector<Real4> & velMasD2,
		thrust::device_vector<Real4> & rhoPresMuD2,

		thrust::device_vector<Real3> & posRadD,
		thrust::device_vector<Real4> & velMasD,
		thrust::device_vector<Real3> & vel_XSPH_D,
		thrust::device_vector<Real4> & rhoPresMuD,

		thrust::device_vector<uint> & bodyIndexD,
		thrust::device_vector<Real4> & derivVelRhoD,
		const thrust::host_vector<int3> & referenceArray,
		const NumberOfObjects & numObjects,
		SimParams currentParamsH,
		Real dT);

void cudaCollisions(
		thrust::host_vector<Real3> & mPosRad,
		thrust::host_vector<Real4> & mVelMas,
		thrust::host_vector<Real4> & mRhoPresMu,
		const thrust::host_vector<uint> & bodyIndex,
		const thrust::host_vector<int3> & referenceArray,

		SimParams paramsH,
		NumberOfObjects numObjects);

#endif
