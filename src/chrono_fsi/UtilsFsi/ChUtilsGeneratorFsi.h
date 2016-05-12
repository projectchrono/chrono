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

#include "chrono_fsi/UtilsFsi/ChUtilsGeneratorBce.h"
#include "chrono_fsi/ChFsiDataManager.cuh"

namespace chrono {
namespace fsi {
namespace utils {

chrono::ChVector<> TransformBCEToCOG(chrono::ChBody* body, const chrono::ChVector<>& pos);
chrono::ChVector<> TransformBCEToCOG(chrono::ChBody* body, const Real3& pos3);

void CreateBceGlobalMarkersFromBceLocalPos(ChFsiDataManager* fsiData,
                                           SimParams* paramsH,
                                           const thrust::host_vector<Real3>& posRadBCE,
                                           std::shared_ptr<chrono::ChBody> body,
                                           chrono::ChVector<> collisionShapeRelativePos = chrono::ChVector<>(0),
                                           chrono::ChQuaternion<> collisionShapeRelativeRot = chrono::QUNIT,
                                           bool isSolid = true);

void CreateBceGlobalMarkersFromBceLocalPosBoundary(ChFsiDataManager* fsiData,
                                                   SimParams* paramsH,
                                                   const thrust::host_vector<Real3>& posRadBCE,
                                                   std::shared_ptr<chrono::ChBody> body,
                                                   chrono::ChVector<> collisionShapeRelativePos,
                                                   chrono::ChQuaternion<> collisionShapeRelativeRot);

void AddSphereBce(ChFsiDataManager* fsiData,
                  SimParams* paramsH,
                  std::shared_ptr<chrono::ChBody> body,
                  chrono::ChVector<> relPos,
                  chrono::ChQuaternion<> relRot,
                  Real radius);

void AddCylinderBce(ChFsiDataManager* fsiData,
                    SimParams* paramsH,
                    std::shared_ptr<chrono::ChBody> body,
                    chrono::ChVector<> relPos,
                    chrono::ChQuaternion<> relRot,
                    Real radius,
                    Real height);

void AddBoxBce(ChFsiDataManager* fsiData,
               SimParams* paramsH,
               std::shared_ptr<chrono::ChBody> body,
               chrono::ChVector<> relPos,
               chrono::ChQuaternion<> relRot,
               const chrono::ChVector<>& size);

void AddBCE_FromFile(ChFsiDataManager* fsiData,
                     SimParams* paramsH,
                     std::shared_ptr<chrono::ChBody> body,
                     std::string dataPath);

void CreateSphereFSI(ChFsiDataManager* fsiData,
                     chrono::ChSystem& mphysicalSystem,
                     std::vector<std::shared_ptr<chrono::ChBody> >* fsiBodeisPtr,
                     SimParams* paramsH,
                     std::shared_ptr<chrono::ChMaterialSurface> mat_prop,
                     Real density,
                     chrono::ChVector<> pos,
                     Real radius);

void CreateCylinderFSI(ChFsiDataManager* fsiData,
                       chrono::ChSystem& mphysicalSystem,
                       std::vector<std::shared_ptr<chrono::ChBody> >* fsiBodeisPtr,
                       SimParams* paramsH,
                       std::shared_ptr<chrono::ChMaterialSurface> mat_prop,
                       Real density,
                       chrono::ChVector<> pos,
                       chrono::ChQuaternion<> rot,
                       Real radius,
                       Real length);

void CreateBoxFSI(ChFsiDataManager* fsiData,
                  chrono::ChSystem& mphysicalSystem,
                  std::vector<std::shared_ptr<chrono::ChBody> >* fsiBodeisPtr,
                  SimParams* paramsH,
                  std::shared_ptr<chrono::ChMaterialSurface> mat_prop,
                  Real density,
                  chrono::ChVector<> pos,
                  chrono::ChQuaternion<> rot,
                  const chrono::ChVector<>& hsize);

}  // end namespace utils
}  // end namespace fsi
}  // end namespace chrono

#endif
