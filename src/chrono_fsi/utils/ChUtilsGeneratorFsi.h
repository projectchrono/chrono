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

#include "chrono_fsi/ChFsiDataManager.cuh"
#include "chrono_fsi/utils/ChUtilsGeneratorBce.h"

namespace chrono {
namespace fsi {
namespace utils {

chrono::ChVector<> TransformBCEToCOG(chrono::ChBody *body,
                                     const chrono::ChVector<> &pos);
chrono::ChVector<> TransformBCEToCOG(chrono::ChBody *body, const Real3 &pos3);

CH_FSI_API void CreateBceGlobalMarkersFromBceLocalPos(
    ChFsiDataManager *fsiData, SimParams *paramsH,
    const thrust::host_vector<Real3> &posRadBCE,
    std::shared_ptr<chrono::ChBody> body,
    chrono::ChVector<> collisionShapeRelativePos = chrono::ChVector<>(0),
    chrono::ChQuaternion<> collisionShapeRelativeRot = chrono::QUNIT,
    bool isSolid = true);

CH_FSI_API void CreateBceGlobalMarkersFromBceLocalPosBoundary(
    ChFsiDataManager *fsiData, SimParams *paramsH,
    const thrust::host_vector<Real3> &posRadBCE,
    std::shared_ptr<chrono::ChBody> body,
    chrono::ChVector<> collisionShapeRelativePos,
    chrono::ChQuaternion<> collisionShapeRelativeRot);

CH_FSI_API void AddSphereBce(ChFsiDataManager *fsiData, SimParams *paramsH,
                             std::shared_ptr<chrono::ChBody> body,
                             chrono::ChVector<> relPos,
                             chrono::ChQuaternion<> relRot, Real radius);

CH_FSI_API void AddCylinderBce(ChFsiDataManager *fsiData, SimParams *paramsH,
                               std::shared_ptr<chrono::ChBody> body,
                               chrono::ChVector<> relPos,
                               chrono::ChQuaternion<> relRot, Real radius,
                               Real height);

CH_FSI_API void AddBoxBce(ChFsiDataManager *fsiData, SimParams *paramsH,
                          std::shared_ptr<chrono::ChBody> body,
                          chrono::ChVector<> relPos,
                          chrono::ChQuaternion<> relRot,
                          const chrono::ChVector<> &size,
			  int plane=12);


CH_FSI_API void AddBCE_FromFile(ChFsiDataManager *fsiData, SimParams *paramsH,
                                std::shared_ptr<chrono::ChBody> body,
                                std::string dataPath);

CH_FSI_API void
CreateSphereFSI(ChFsiDataManager *fsiData, chrono::ChSystem &mphysicalSystem,
                std::vector<std::shared_ptr<chrono::ChBody>> *fsiBodeisPtr,
                SimParams *paramsH,
                std::shared_ptr<chrono::ChMaterialSurfaceNSC> mat_prop,
                Real density, chrono::ChVector<> pos, Real radius);

CH_FSI_API void
CreateCylinderFSI(ChFsiDataManager *fsiData, chrono::ChSystem &mphysicalSystem,
                  std::vector<std::shared_ptr<chrono::ChBody>> *fsiBodeisPtr,
                  SimParams *paramsH,
                  std::shared_ptr<chrono::ChMaterialSurfaceNSC> mat_prop,
                  Real density, chrono::ChVector<> pos,
                  chrono::ChQuaternion<> rot, Real radius, Real length);

CH_FSI_API void
CreateBoxFSI(ChFsiDataManager *fsiData, chrono::ChSystem &mphysicalSystem,
             std::vector<std::shared_ptr<chrono::ChBody>> *fsiBodeisPtr,
             SimParams *paramsH,
             std::shared_ptr<chrono::ChMaterialSurfaceNSC> mat_prop, Real density,
             chrono::ChVector<> pos, chrono::ChQuaternion<> rot,
             const chrono::ChVector<> &hsize);

} // end namespace utils
} // end namespace fsi
} // end namespace chrono

#endif
