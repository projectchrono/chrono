#ifndef PRINTTOFILE_CUH
#define PRINTTOFILE_CUH
#include "custom_cutil_math.h"
#include "SPHCudaUtils.h"
#include "SDKCollisionSystem.cuh"
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

//--------------------------------------------------------------------------------------------------------------------------------
void PrintCartesianData_MidLine(
		const thrust::host_vector<real4> & rho_Pres_CartH,
		const thrust::host_vector<real4> & vel_VelMag_CartH,
		const int3 & cartesianGridDims,
		const SimParams & paramsH);
//--------------------------------------------------------------------------------------------------------------------------------
void PrintToFile(
		const thrust::device_vector<real3> & posRadD,
		const thrust::device_vector<real4> & velMasD,
		const thrust::device_vector<real4> & rhoPresMuD,
		const thrust::host_vector<int3> & referenceArray,
		const SimParams paramsH,
		real_ realTime,
		int tStep);
#endif
