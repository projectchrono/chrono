#ifndef COLLIDESPHERESPHERE_CUH
#define COLLIDESPHERESPHERE_CUH

#include "SDKCollisionSystem.cuh" //just for SimParams

void InitSystem(
		SimParams paramsH,
		NumberOfObjects numObjects);

void cudaCollisions(
		thrust::host_vector<real3> & mPosRad,
		thrust::host_vector<real4> & mVelMas,
		thrust::host_vector<real4> & mRhoPresMu,
		const thrust::host_vector<uint> & bodyIndex,
		const thrust::host_vector<int3> & referenceArray,

		SimParams paramsH,
		NumberOfObjects numObjects);

#endif
