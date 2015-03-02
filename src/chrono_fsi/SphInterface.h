/*
 * SphInterface.h
 *
 *  Created on: Mar 2, 2015
 *      Author: arman
 */

#ifndef SPHINTERFACE_H_
#define SPHINTERFACE_H_

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include "custom_cutil_math.h"
#include "SPHCudaUtils.h"
#include "SDKCollisionSystem.cuh" //just for SimParams

#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/lcp/ChLcpSystemDescriptorParallel.h"
#include "chrono_utils/ChUtilsCreators.h"  //Arman: why is this
#include "chrono_utils/ChUtilsInputOutput.h" //Arman: Why is this
#include "chrono_utils/ChUtilsGenerators.h"
#include <vector>

using namespace std;
using namespace chrono;
using namespace chrono::collision;



void SetupParamsH(SimParams & paramsH);

void AddSphDataToChSystem(
		ChSystemParallelDVI& mphysicalSystem,
		int & startIndexSph,
		const thrust::host_vector<Real3> & posRadH,
		const thrust::host_vector<Real4> & velMasH,
		const SimParams & paramsH,
		const NumberOfObjects & numObjects);

void UpdateSphDataInChSystem(
		ChSystemParallelDVI& mphysicalSystem,
		const thrust::host_vector<Real3> & posRadH,
		const thrust::host_vector<Real4> & velMasH,
		const NumberOfObjects & numObjects,
		int  startIndexSph);

void AddChSystemForcesToSphForces(
		thrust::host_vector<Real4>  & derivVelRhoChronoH,
		const thrust::host_vector<Real4> & velMasH2,
		ChSystemParallelDVI& mphysicalSystem,
		const NumberOfObjects & numObjects,
		int startIndexSph,
		Real dT);

void ClearArraysH(
	thrust::host_vector<Real3> & posRadH, //do not set the size here since you are using push back later
	thrust::host_vector<Real4> & velMasH,
	thrust::host_vector<Real4> & rhoPresMuH);

void ClearArraysH(
	thrust::host_vector<Real3> & posRadH, //do not set the size here since you are using push back later
	thrust::host_vector<Real4> & velMasH,
	thrust::host_vector<Real4> & rhoPresMuH,
	thrust::host_vector<uint> & bodyIndex,
	thrust::host_vector<int3> & referenceArray);

void CopyD2H(
	thrust::host_vector<Real4> & derivVelRhoChronoH,
	const thrust::device_vector<Real4> & derivVelRhoD);

void CopyH2D(
	thrust::device_vector<Real4> & derivVelRhoD,
	const thrust::host_vector<Real4> & derivVelRhoChronoH);

void CopySys2D(
	thrust::device_vector<Real3> & posRadD,
	ChSystemParallelDVI& mphysicalSystem,
	const NumberOfObjects & numObjects,
	int startIndexSph);

void CopyD2H(
	thrust::host_vector<Real3> & posRadH, //do not set the size here since you are using push back later
	thrust::host_vector<Real4> & velMasH,
	thrust::host_vector<Real4> & rhoPresMuH,
	const thrust::device_vector<Real3> & posRadD,
	const thrust::device_vector<Real4> & velMasD,
	const thrust::device_vector<Real4> & rhoPresMuD);

#endif /* SPHINTERFACE_H_ */
