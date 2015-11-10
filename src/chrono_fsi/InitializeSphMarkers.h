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

void CreateBCE_On_Box(thrust::host_vector<Real3>& posRadBCE,
                      thrust::host_vector<Real4>& velMasBCE,
                      thrust::host_vector<Real4>& rhoPresMuBCE,
                      const SimParams& paramsH,
                      const Real& sphMarkerMass,
                      const chrono::ChVector<>& size,
                      const chrono::ChVector<>& pos = chrono::ChVector<>(0, 0, 0),
                      const chrono::ChQuaternion<>& rot = chrono::ChQuaternion<>(1, 0, 0, 0),
                      int face = 12);

void LoadBCE_fromFile(
    thrust::host_vector<Real3>& posRadBCE,  // do not set the size here since you are using push back later
    std::string fileName);
//void LoadBCE_fromFile(
//    thrust::host_vector<Real3>& posRadH,  // do not set the size here since you are using push back later
//    thrust::host_vector<Real4>& velMasH,
//    thrust::host_vector<Real4>& rhoPresMuH,
//    thrust::host_vector< ::int3>& referenceArray,
//    NumberOfObjects& numObjects,
//    Real sphMarkerMass,
//    std::string fileName);

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
void SetNumObjects(NumberOfObjects& numObjects, const thrust::host_vector<int3>& referenceArray, int numAllMarkers);

#endif /* INITIALIZESPHMARKERS_H_ */
