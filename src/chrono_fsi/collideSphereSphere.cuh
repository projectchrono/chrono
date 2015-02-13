#ifndef COLLIDESPHERESPHERE_CUH
#define COLLIDESPHERESPHERE_CUH

#include "SDKCollisionSystem.cuh" //just for SimParams

void InitSystem(
		SimParams paramsH,
		NumberOfObjects numObjects);
//******
template<typename T>
void ResizeMyThrust(thrust::device_vector<T> & mThrustVec, int mSize);

void ResizeMyThrust3(thrust::device_vector<real3> & mThrustVec, int mSize);
void ResizeMyThrust4(thrust::device_vector<real4> & mThrustVec, int mSize);
//******
template<typename T>
void FillMyThrust(thrust::device_vector<T> & mThrustVec, T v);
void FillMyThrust4(thrust::device_vector<real4> & mThrustVec, real4 v);


//******
template<typename T>
void ClearMyThrust(thrust::device_vector<T> & mThrustVec);

void ClearMyThrust3(thrust::device_vector<real3> & mThrustVec);
void ClearMyThrust4(thrust::device_vector<real4> & mThrustVec);
void ClearMyThrustU(thrust::device_vector<uint> & mThrustVec);
//******

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
		real_ dT);

void IntegrateSPH(
		thrust::device_vector<real3> & posRadD2,
		thrust::device_vector<real4> & velMasD2,
		thrust::device_vector<real4> & rhoPresMuD2,

		thrust::device_vector<real3> & posRadD,
		thrust::device_vector<real4> & velMasD,
		thrust::device_vector<real3> & vel_XSPH_D,
		thrust::device_vector<real4> & rhoPresMuD,

		thrust::device_vector<uint> & bodyIndexD,
		thrust::device_vector<real4> & derivVelRhoD,
		const thrust::host_vector<int3> & referenceArray,
		const NumberOfObjects & numObjects,
		SimParams currentParamsH,
		real_ dT);

void cudaCollisions(
		thrust::host_vector<real3> & mPosRad,
		thrust::host_vector<real4> & mVelMas,
		thrust::host_vector<real4> & mRhoPresMu,
		const thrust::host_vector<uint> & bodyIndex,
		const thrust::host_vector<int3> & referenceArray,

		SimParams paramsH,
		NumberOfObjects numObjects);

#endif
