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
 * @brief Set the number of objects (rigid and flexible)
 * @details [long description]
 *
 * @param numObjects Reference where the objects will be stored in
 * @param referenceArray referenceArray[0].y = number of fluid markers,
 *                       referenceArray[1].x = number of fluid markers,
 *                       referenceArray[1].y = Total number of markers,
 * @param numAllMarkers Total number of markers (fluid + boundary)
 */
void SetNumObjects(NumberOfObjects& numObjects,
		const thrust::host_vector<int4>& referenceArray, int numAllMarkers);

/**
 * @brief Create the fluid markers
 * @details
 * 		This function is a model specific function. Depends on what kind of container your fluid
 * 		will be in. Currently it sets up a box of fluid and this is dropped into a container
 */
int2 CreateFluidMarkers(thrust::host_vector<Real3>& posRadH,
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH,
		thrust::host_vector<uint>& bodyIndex, SimParams& paramsH);

void CreateBceGlobalMarkersFromBceLocalPos(thrust::host_vector<Real3>& posRadH,
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH,
		thrust::host_vector<::int4>& referenceArray,
		NumberOfObjects& numObjects,
		const thrust::host_vector<Real3>& posRadBCE,
		const SimParams& paramsH, chrono::ChSharedPtr<chrono::ChBody> body,
		chrono::ChVector<> collisionShapeRelativePos = chrono::ChVector<>(0),
		chrono::ChQuaternion<> collisionShapeRelativeRot = chrono::QUNIT,
		bool isSolid = true);

void CreateBceGlobalMarkersFromBceLocalPosBoundary(
		thrust::host_vector<Real3>& posRadH,
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH,
		thrust::host_vector<::int4>& referenceArray,
		NumberOfObjects& numObjects,
		const thrust::host_vector<Real3>& posRadBCE,
		const SimParams& paramsH, chrono::ChSharedPtr<chrono::ChBody> body,
		chrono::ChVector<> collisionShapeRelativePos = chrono::ChVector<>(0),
		chrono::ChQuaternion<> collisionShapeRelativeRot = chrono::QUNIT);

void AddSphereBce(
		thrust::host_vector<Real3>& posRadH, // do not set the size here since you are using push back later
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH,
		thrust::host_vector<::int4>& referenceArray,
		NumberOfObjects& numObjects,
		const SimParams& paramsH, chrono::ChSharedPtr<chrono::ChBody> body,
		Real radius, chrono::ChVector<> relPos = chrono::ChVector<>(0, 0, 0),
		chrono::ChQuaternion<> relRot = chrono::ChQuaternion<>(1, 0, 0, 0));

void AddCylinderBce(
		thrust::host_vector<Real3>& posRadH, // do not set the size here since you are using push back later
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH,
		thrust::host_vector<::int4>& referenceArray,
		NumberOfObjects& numObjects,
		const SimParams& paramsH, chrono::ChSharedPtr<chrono::ChBody> body,
		Real radius, Real height, chrono::ChVector<> relPos =
				chrono::ChVector<>(0, 0, 0), chrono::ChQuaternion<> relRot =
				chrono::ChQuaternion<>(1, 0, 0, 0));

void AddBoxBce(
		thrust::host_vector<Real3>& posRadH, // do not set the size here since you are using push back later
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH,
		thrust::host_vector<::int4>& referenceArray,
		NumberOfObjects& numObjects,
		const SimParams& paramsH, chrono::ChSharedPtr<chrono::ChBody> body,
		const chrono::ChVector<>& size, chrono::ChVector<> relPos =
				chrono::ChVector<>(0, 0, 0), chrono::ChQuaternion<> relRot =
				chrono::ChQuaternion<>(1, 0, 0, 0));

void AddBCE_FromFile(
		thrust::host_vector<Real3>& posRadH, // do not set the size here since you are using push back later
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH,
		thrust::host_vector<::int4>& referenceArray,
		NumberOfObjects& numObjects,
		const SimParams& paramsH, chrono::ChSharedPtr<chrono::ChBody> body,
		std::string dataPath);

void CreateSphereFSI(
		thrust::host_vector<Real3>& posRadH, // do not set the size here since you are using push back later
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH,
		thrust::host_vector<::int4>& referenceArray,
		chrono::ChSystem& mphysicalSystem,
		std::vector<chrono::ChSharedPtr<chrono::ChBody> >& FSI_Bodies,
		NumberOfObjects& numObjects,
		const SimParams& paramsH,
		Real radius,
		chrono::ChSharedPtr<chrono::ChMaterialSurface> mat_prop,
		Real density,
		chrono::ChVector<> pos = chrono::ChVector<>(0));

void CreateCylinderFSI(
		thrust::host_vector<Real3>& posRadH, // do not set the size here since you are using push back later
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH,
		thrust::host_vector<::int4>& referenceArray,
		chrono::ChSystem& mphysicalSystem,
		std::vector<chrono::ChSharedPtr<chrono::ChBody> >& FSI_Bodies,
		NumberOfObjects& numObjects,
		const SimParams& paramsH,
		Real radius,
		Real length,
		chrono::ChSharedPtr<chrono::ChMaterialSurface> mat_prop,
		Real density,
		chrono::ChVector<> pos = chrono::ChVector<>(0),
		chrono::ChQuaternion<> rot = chrono::QUNIT);

void CreateBoxFSI(
		thrust::host_vector<Real3>& posRadH, // do not set the size here since you are using push back later
		thrust::host_vector<Real4>& velMasH,
		thrust::host_vector<Real4>& rhoPresMuH,
		thrust::host_vector<::int4>& referenceArray,
		chrono::ChSystem& mphysicalSystem,
		std::vector<chrono::ChSharedPtr<chrono::ChBody> >& FSI_Bodies,
		NumberOfObjects& numObjects,
		const SimParams& paramsH,
		const chrono::ChVector<>& hsize,
		chrono::ChSharedPtr<chrono::ChMaterialSurface> mat_prop,
		Real density,
		chrono::ChVector<> pos = chrono::ChVector<>(0),
		chrono::ChQuaternion<> rot = chrono::QUNIT);

#endif /* INITIALIZESPHMARKERS_H_ */
