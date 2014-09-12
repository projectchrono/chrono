#ifndef PRINTTOFILE_CUH
#define PRINTTOFILE_CUH
#include "custom_cutil_math.h"
#include "SPHCudaUtils.h"
#include "SDKCollisionSystem.cuh"
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

//--------------------------------------------------------------------------------------------------------------------------------
void printMaxStress(char * fileName, real_ maxStress, int tStep);
//--------------------------------------------------------------------------------------------------------------------------------
void PrintToFile(
		const thrust::device_vector<real3> & posRadD,
		const thrust::device_vector<real4> & velMasD,
		const thrust::device_vector<real4> & rhoPresMuD,
		const thrust::host_vector<int3> & referenceArray,
		const thrust::device_vector<int> & rigidIdentifierD,
		const thrust::device_vector<real3> & posRigidD,
		const thrust::device_vector<real3> & posRigidCumulativeD,
		const thrust::device_vector<real4> & velMassRigidD,
		const thrust::device_vector<real4> & qD1,
		const thrust::device_vector<real3> & AD1,
		const thrust::device_vector<real3> & AD2,
		const thrust::device_vector<real3> & AD3,
		const thrust::device_vector<real3> & omegaLRF_D,

		const thrust::device_vector<real3> & ANCF_NodesD,
		const thrust::device_vector<real3> & ANCF_SlopesD,
		const thrust::device_vector<real3> & ANCF_NodesVelD,
		const thrust::device_vector<real3> & ANCF_SlopesVelD,
		const thrust::device_vector<int2> & ANCF_ReferenceArrayNodesOnBeamsD,

		const SimParams paramsH,
		const real_ realTime,
		int tStep,
		const real_ channelRadius,
		const real2 channelCenterYZ,
		const int numRigidBodies,
		const int numFlexBodies);

void PrintToFileDistribution(
		thrust::device_vector<int> & distributionD,
		real_ channelRadius,
		int numberOfSections,
		int tStep);


#endif
