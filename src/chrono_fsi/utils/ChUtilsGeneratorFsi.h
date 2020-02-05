// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Arman Pazouki, Milad Rakhsha
// =============================================================================
//
// Utility class for generating BCE markers.//
// =============================================================================

#ifndef CH_UTILSGENERATORFSI_CUH
#define CH_UTILSGENERATORFSI_CUH

#include "chrono/ChConfig.h"
#include "chrono_fsi/ChFsiDataManager.cuh"
#include "chrono_fsi/utils/ChUtilsGeneratorBce.h"

namespace chrono {

// Forward declaration
namespace fea {
class ChElementCableANCF;
class ChElementShellANCF;
class ChNodeFEAxyzD;
class ChMesh;
}  // namespace fea

namespace fsi {
namespace utils {

chrono::ChVector<> TransformBCEToCOG(chrono::ChBody* body, const chrono::ChVector<>& pos);
chrono::ChVector<> TransformBCEToCOG(chrono::ChBody* body, const Real3& pos3);

CH_FSI_API void FinalizeDomainCreating(SimParams* paramsH);

CH_FSI_API void CreateBceGlobalMarkersFromBceLocalPos(
    ChFsiDataManager* fsiData,
    SimParams* paramsH,
    const thrust::host_vector<Real4>& posRadBCE,
    std::shared_ptr<chrono::ChBody> body,
    chrono::ChVector<> collisionShapeRelativePos = chrono::ChVector<>(0),
    chrono::ChQuaternion<> collisionShapeRelativeRot = chrono::QUNIT,
    bool isSolid = true,
    bool add_to_fluid_helpers = false);

CH_FSI_API void CreateBceGlobalMarkersFromBceLocalPosBoundary(ChFsiDataManager* fsiData,
                                                              SimParams* paramsH,
                                                              const thrust::host_vector<Real4>& posRadBCE,
                                                              std::shared_ptr<chrono::ChBody> body,
                                                              chrono::ChVector<> collisionShapeRelativePos,
                                                              chrono::ChQuaternion<> collisionShapeRelativeRot,
                                                              bool isSolid = false);

CH_FSI_API void AddSphereBce(ChFsiDataManager* fsiData,
                             SimParams* paramsH,
                             std::shared_ptr<chrono::ChBody> body,
                             chrono::ChVector<> relPos,
                             chrono::ChQuaternion<> relRot,
                             Real radius);

CH_FSI_API void AddCylinderBce(ChFsiDataManager* fsiData,
                               SimParams* paramsH,
                               std::shared_ptr<chrono::ChBody> body,
                               chrono::ChVector<> relPos,
                               chrono::ChQuaternion<> relRot,
                               Real radius,
                               Real height,
                               Real kernel_h,
                               bool cartesian = true);
CH_FSI_API void AddCylinderSurfaceBce(ChFsiDataManager* fsiData,
                                      SimParams* paramsH,
                                      std::shared_ptr<ChBody> body,
                                      ChVector<> relPos,
                                      ChQuaternion<> relRot,
                                      Real radius,
                                      Real height,
                                      Real kernel_h);
CH_FSI_API void AddSphereSurfaceBce(ChFsiDataManager* fsiData,
                                    SimParams* paramsH,
                                    std::shared_ptr<ChBody> body,
                                    ChVector<> relPos,
                                    ChQuaternion<> relRot,
                                    Real radius,
                                    Real kernel_h);
CH_FSI_API void AddBoxBce(ChFsiDataManager* fsiData,
                          SimParams* paramsH,
                          std::shared_ptr<chrono::ChBody> body,
                          chrono::ChVector<> relPos,
                          chrono::ChQuaternion<> relRot,
                          const chrono::ChVector<>& size,
                          int plane = 12,
                          bool isSolid = false);

CH_FSI_API void AddBCE_FromFile(ChFsiDataManager* fsiData,
                                SimParams* paramsH,
                                std::shared_ptr<chrono::ChBody> body,
                                std::string dataPath,
                                chrono::ChVector<> collisionShapeRelativePos = chrono::ChVector<>(0),
                                chrono::ChQuaternion<> collisionShapeRelativeRot = chrono::QUNIT,
                                double scale = 1.);

CH_FSI_API void CreateSphereFSI(ChFsiDataManager* fsiData,
                                chrono::ChSystem& mphysicalSystem,
                                std::vector<std::shared_ptr<chrono::ChBody>>* fsiBodeisPtr,
                                SimParams* paramsH,
                                std::shared_ptr<chrono::ChMaterialSurfaceNSC> mat_prop,
                                Real density,
                                chrono::ChVector<> pos,
                                Real radius);

CH_FSI_API void CreateCylinderFSI(ChFsiDataManager* fsiData,
                                  chrono::ChSystem& mphysicalSystem,
                                  std::vector<std::shared_ptr<chrono::ChBody>>* fsiBodeisPtr,
                                  SimParams* paramsH,
                                  std::shared_ptr<chrono::ChMaterialSurfaceSMC> mat_prop,
                                  Real density,
                                  chrono::ChVector<> pos,
                                  chrono::ChQuaternion<> rot,
                                  Real radius,
                                  Real length);

CH_FSI_API void CreateBoxFSI(ChFsiDataManager* fsiData,
                             chrono::ChSystem& mphysicalSystem,
                             std::vector<std::shared_ptr<chrono::ChBody>>* fsiBodeisPtr,
                             SimParams* paramsH,
                             std::shared_ptr<chrono::ChMaterialSurface> mat_prop,
                             Real density,
                             chrono::ChVector<> pos,
                             chrono::ChQuaternion<> rot,
                             const chrono::ChVector<>& hsize);

CH_FSI_API void AddBCE_ShellANCF(ChFsiDataManager* fsiData,
                                 SimParams* paramsH,
                                 std::vector<std::shared_ptr<chrono::fea::ChElementShellANCF>>* fsiShellsPtr,
                                 std::shared_ptr<chrono::fea::ChMesh> my_mesh,
                                 bool multiLayer = true,
                                 bool removeMiddleLayer = false,
                                 int SIDE = -2);

CH_FSI_API void AddBCE_ShellFromMesh(ChFsiDataManager* fsiData,
                                     SimParams* paramsH,
                                     std::vector<std::shared_ptr<chrono::fea::ChElementShellANCF>>* fsiShellsPtr,
                                     std::vector<std::shared_ptr<chrono::fea::ChNodeFEAxyzD>>* fsiNodesPtr,
                                     std::shared_ptr<chrono::fea::ChMesh> my_mesh,
                                     std::vector<std::vector<int>> elementsNodes,
                                     std::vector<std::vector<int>> NodeNeighborElement,
                                     bool multiLayer = true,
                                     bool removeMiddleLayer = false,
                                     int SIDE = -2);

CH_FSI_API void AddBCE_FromMesh(ChFsiDataManager* fsiData,
                                SimParams* paramsH,
                                std::shared_ptr<chrono::fea::ChMesh> my_mesh,
                                std::vector<std::shared_ptr<chrono::fea::ChNodeFEAxyzD>>* fsiNodesPtr,
                                std::vector<std::shared_ptr<chrono::fea::ChElementCableANCF>>* fsiCablesPtr,
                                std::vector<std::shared_ptr<chrono::fea::ChElementShellANCF>>* fsiShellsPtr,
                                std::vector<std::vector<int>> NodeNeighborElement,
                                std::vector<std::vector<int>> _1D_elementsNodes,
                                std::vector<std::vector<int>> _2D_elementsNodes,
                                bool add1DElem,
                                bool add2DElem,
                                bool multiLayer,
                                bool removeMiddleLayer,
                                int SIDE,
                                int SIDE2D = 2,
                                double kernel_h = 0);

}  // end namespace utils
}  // end namespace fsi
}  // end namespace chrono

#endif
