// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Arman Pazouki
// =============================================================================
//
// Utility class for generating BCE markers.//
// =============================================================================

#ifndef CH_UTILSGENERATORFSI_CUH
#define CH_UTILSGENERATORFSI_CUH

#include "chrono_parallel/physics/ChSystemParallel.h"


#include "chrono_fsi/ChUtilsGeneratorBce.h"
#include "chrono_fsi/ChFsiDataManager.cuh"

namespace chrono{
namespace fsi {
namespace utils {

	chrono::ChVector<> TransformBCEToCOG(chrono::ChBody* body, const chrono::ChVector<> & pos);
	chrono::ChVector<> TransformBCEToCOG(chrono::ChBody* body, const Real3 & pos3);

	void CreateBceGlobalMarkersFromBceLocalPos(
		SphMarkerDataH& sphMarkersH,
		FsiGeneralData& fsiGeneralData,
		NumberOfObjects* numObjects,
		SimParams* paramsH,
		const thrust::host_vector<Real3>& posRadBCE,
		chrono::ChSharedPtr<chrono::ChBody> body,
		chrono::ChVector<> collisionShapeRelativePos = chrono::ChVector<>(0),
		chrono::ChQuaternion<> collisionShapeRelativeRot =  chrono::QUNIT,
		bool isSolid = true);

	void CreateBceGlobalMarkersFromBceLocalPosBoundary(
		SphMarkerDataH& sphMarkersH,
		FsiGeneralData& fsiGeneralData,
		NumberOfObjects* numObjects,
		SimParams* paramsH,
		const thrust::host_vector<Real3>& posRadBCE,
		chrono::ChSharedPtr<chrono::ChBody> body,
		chrono::ChVector<> collisionShapeRelativePos,
		chrono::ChQuaternion<> collisionShapeRelativeRot);

	void AddSphereBce(
		SphMarkerDataH& sphMarkersH,
		FsiGeneralData& fsiGeneralData,
		NumberOfObjects* numObjects,
		SimParams* paramsH,
		chrono::ChSharedPtr<chrono::ChBody> body, 
		chrono::ChVector<> relPos,
		chrono::ChQuaternion<> relRot,
		Real radius);

	void AddCylinderBce(
		SphMarkerDataH& sphMarkersH,
		FsiGeneralData& fsiGeneralData,
		NumberOfObjects* numObjects,
		SimParams* paramsH,
		chrono::ChSharedPtr<chrono::ChBody> body, 
		chrono::ChVector<> relPos,
		chrono::ChQuaternion<> relRot,
		Real radius, 
		Real height);

	void AddBoxBce(
		SphMarkerDataH& sphMarkersH,
		FsiGeneralData& fsiGeneralData,
		NumberOfObjects* numObjects,
		SimParams* paramsH,
		chrono::ChSharedPtr<chrono::ChBody> body, 
		chrono::ChVector<> relPos,
		chrono::ChQuaternion<> relRot,
		const chrono::ChVector<>& size);

	void AddBCE_FromFile(
		SphMarkerDataH& sphMarkersH,
		FsiGeneralData& fsiGeneralData,
		NumberOfObjects* numObjects,
		SimParams* paramsH,
		chrono::ChSharedPtr<chrono::ChBody> body,
		std::string dataPath);

	void CreateSphereFSI(
		SphMarkerDataH& sphMarkersH,
		FsiGeneralData& fsiGeneralData,
		std::vector<chrono::ChSharedPtr<chrono::ChBody> > * fsiBodeisPtr,
		chrono::ChSystem& mphysicalSystem,
		NumberOfObjects* numObjects,
		SimParams* paramsH,
		chrono::ChSharedPtr<chrono::ChMaterialSurface> mat_prop,
		Real density,
		chrono::ChVector<> pos,
		Real radius);

	void CreateCylinderFSI(
		SphMarkerDataH& sphMarkersH,
		FsiGeneralData& fsiGeneralData,
		std::vector<chrono::ChSharedPtr<chrono::ChBody> > * fsiBodeisPtr,
		chrono::ChSystem& mphysicalSystem,
		NumberOfObjects* numObjects,
		SimParams* paramsH,
		chrono::ChSharedPtr<chrono::ChMaterialSurface> mat_prop,
		Real density,
		chrono::ChVector<> pos,
		chrono::ChQuaternion<> rot,
		Real radius,
		Real length);

	void CreateBoxFSI(
		SphMarkerDataH& sphMarkersH,
		FsiGeneralData& fsiGeneralData,
		std::vector<chrono::ChSharedPtr<chrono::ChBody> > * fsiBodeisPtr,
		chrono::ChSystem& mphysicalSystem,
		NumberOfObjects* numObjects,
		SimParams* paramsH,
		chrono::ChSharedPtr<chrono::ChMaterialSurface> mat_prop,
		Real density,
		chrono::ChVector<> pos,
		chrono::ChQuaternion<> rot,
		const chrono::ChVector<>& hsize);


} // end namespace utils
} // end namespace fsi
} // end namespace chrono

#endif
