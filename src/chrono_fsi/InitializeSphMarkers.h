/*
 * InitializeFluidSystem.h
 *
 *  Created on: Mar 2, 2015
 *      Author: arman
 */

#ifndef INITIALIZESPHMARKERS_H_
#define INITIALIZESPHMARKERS_H_

#include <thrust/host_vector.h>
#include "SDKCollisionSystem.cuh" // and real 4 stuff

int2 CreateFluidMarkers(
		thrust::host_vector<Real3> & posRadH,
		thrust::host_vector<Real4> & velMasH,
		thrust::host_vector<Real4> & rhoPresMuH,
		thrust::host_vector<uint> & bodyIndex,
		const SimParams & paramsH,
		Real & sphMarkerMass);

void SetNumObjects(
		NumberOfObjects & numObjects,
		const thrust::host_vector<int3> & referenceArray,
		int numAllMarkers);

#endif /* INITIALIZESPHMARKERS_H_ */
