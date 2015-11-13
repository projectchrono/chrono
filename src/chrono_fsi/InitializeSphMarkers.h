/*
 * InitializeFluidSystem.h
 *
 *  Created on: Mar 2, 2015
 *      Author: Arman Pazouki
 */

#ifndef INITIALIZESPHMARKERS_H_
#define INITIALIZESPHMARKERS_H_

#include <thrust/host_vector.h>
#include "chrono_fsi/custom_cutil_math.h"
#include "chrono_fsi/SPHCudaUtils.h"
#include "chrono_fsi/MyStructs.cuh"

#include "chrono_parallel/physics/ChSystemParallel.h"

/**
 * @brief Create the fluid markers
 * @details
 * 		This function is a model specific function. Depends on what kind of container your fluid
 * 		will be in. Currently it sets up a box of fluid and this is dropped into a container
 */
int2 CreateFluidMarkers(thrust::host_vector<Real3>& posRadH,
                        thrust::host_vector<Real4>& velMasH,
                        thrust::host_vector<Real4>& rhoPresMuH,
                        thrust::host_vector<uint>& bodyIndex,
                        const SimParams& paramsH,
                        Real& sphMarkerMass);

void AddBoxBceToChSystemAndSPH(
    chrono::ChBody* body,
    const chrono::ChVector<>& size,
    const chrono::ChVector<>& pos,
    const chrono::ChQuaternion<>& rot,
    bool visualization,

    thrust::host_vector<Real3>& posRadH,  // do not set the size here since you are using push back later
    thrust::host_vector<Real4>& velMasH,
    thrust::host_vector<Real4>& rhoPresMuH,
    thrust::host_vector<uint>& bodyIndex,
    thrust::host_vector< ::int4>& referenceArray,
    NumberOfObjects& numObjects,
    const SimParams& paramsH,
    Real sphMarkerMass);

void AddSphereBceToChSystemAndSPH(chrono::ChSystemParallelDVI& mphysicalSystem,
		Real radius, const chrono::ChVector<>& pos,
		const chrono::ChQuaternion<>& rot,
		thrust::host_vector<Real3>& posRadH, // do not set the size here since you are using push back later
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH,
		thrust::host_vector<::int4>& referenceArray,
		std::vector<chrono::ChSharedPtr<chrono::ChBody> >& FSI_Bodies,
		NumberOfObjects& numObjects, Real sphMarkerMass,
		const SimParams& paramsH);

void AddCylinderBceToChSystemAndSPH(
		chrono::ChSystemParallelDVI& mphysicalSystem,
    Real radius,
    Real height,
    const chrono::ChVector<>& pos,
    const chrono::ChQuaternion<>& rot,
    thrust::host_vector<Real3>& posRadH,  // do not set the size here since you are using push back later
    thrust::host_vector<Real4>& velMasH,
    thrust::host_vector<Real4>& rhoPresMuH,
    thrust::host_vector< ::int4>& referenceArray,
    std::vector<chrono::ChSharedPtr<chrono::ChBody> >& FSI_Bodies,
    NumberOfObjects& numObjects,
    Real sphMarkerMass,
    const SimParams& paramsH);

void AddBCE2FluidSystem_FromFile(
    thrust::host_vector<Real3>& posRadH,  // do not set the size here since you are using push back later
    thrust::host_vector<Real4>& velMasH,
    thrust::host_vector<Real4>& rhoPresMuH,
    thrust::host_vector< ::int4>& referenceArray,
    NumberOfObjects& numObjects,
    Real sphMarkerMass,
    const SimParams& paramsH,
    chrono::ChSharedPtr<chrono::ChBody> body,
    std::string dataPath);


/**
 * @brief Set the number of objects (rigid and flexible)
 * @details [long description]
 *
 * @param numObjects Reference where the objects will be stored in
 * @param referenceArray referenceArray[0].y = number of fluid markers,
 *                       referenceArray[1].x = number of fluid markers,
 *                       referenceArray[1].y = Total number of markers,
 * @param numAllMarkers Total number of markers (fluid + boundary)
 */
void SetNumObjects(NumberOfObjects& numObjects, const thrust::host_vector<int4>& referenceArray, int numAllMarkers);

#endif /* INITIALIZESPHMARKERS_H_ */
