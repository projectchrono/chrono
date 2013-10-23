#ifndef PRINTTOFILE_CUH
#define PRINTTOFILE_CUH
#include "custom_cutil_math.h"
#include "SPHCudaUtils.h"
#include "SDKCollisionSystem.cuh"
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

//--------------------------------------------------------------------------------------------------------------------------------
void PrintToFile(
		thrust::device_vector<real3> & posRadD,
		thrust::device_vector<real4> & velMasD,
		thrust::device_vector<real4> & rhoPresMuD,
		const thrust::host_vector<int2> & referenceArray,
		const thrust::device_vector<int> & rigidIdentifierD,
		thrust::device_vector<real3> & posRigidD,
		thrust::device_vector<real3> & posRigidCumulativeD,
		thrust::device_vector<real4> & velMassRigidD,
		thrust::device_vector<real4> & qD1,
		thrust::device_vector<real3> & AD1,
		thrust::device_vector<real3> & AD2,
		thrust::device_vector<real3> & AD3,
		thrust::device_vector<real3> & omegaLRF_D,
		real3 cMax,
		real3 cMin,
		SimParams paramsH,
		real_ delT,
		int tStep,
		real_ channelRadius,
		real2 channelCenterYZ);

void PrintToFileDistribution(
		thrust::device_vector<int> & distributionD,
		real_ channelRadius,
		int numberOfSections,
		int tStep);


#endif
