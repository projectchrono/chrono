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

chrono::ChVector<> TransformBCEToCOG(std::shared_ptr<ChBody> body, const chrono::ChVector<>& pos);
chrono::ChVector<> TransformBCEToCOG(std::shared_ptr<ChBody> body, const Real3& pos3);

CH_FSI_API void FinalizeDomain(std::shared_ptr<SimParams> paramsH);

CH_FSI_API void CreateBceGlobalMarkersFromBceLocalPos(
    std::shared_ptr<ChFsiDataManager> fsiData,
    std::shared_ptr<SimParams> paramsH,
    const thrust::host_vector<Real4>& posRadBCE,
    std::shared_ptr<ChBody> body,
    chrono::ChVector<> collisionShapeRelativePos = chrono::ChVector<>(0),
    chrono::ChQuaternion<> collisionShapeRelativeRot = chrono::QUNIT,
    bool isSolid = true,
    bool add_to_fluid_helpers = false,
    bool add_to_previous_object = false);

CH_FSI_API void CreateBceGlobalMarkersFromBceLocalPosBoundary(std::shared_ptr<ChFsiDataManager> fsiData,
                                                              std::shared_ptr<SimParams> paramsH,
                                                              const thrust::host_vector<Real4>& posRadBCE,
                                                              std::shared_ptr<ChBody> body,
                                                              chrono::ChVector<> collisionShapeRelativePos,
                                                              chrono::ChQuaternion<> collisionShapeRelativeRot,
                                                              bool isSolid = false,
                                                              bool add_to_previous = true);

CH_FSI_API void AddSphereBce(std::shared_ptr<ChFsiDataManager> fsiData,
                             std::shared_ptr<SimParams> paramsH,
                             std::shared_ptr<ChBody> body,
                             chrono::ChVector<> relPos,
                             chrono::ChQuaternion<> relRot,
                             Real radius);

CH_FSI_API void AddCylinderBce(std::shared_ptr<ChFsiDataManager> fsiData,
                               std::shared_ptr<SimParams> paramsH,
                               std::shared_ptr<ChBody> body,
                               chrono::ChVector<> relPos,
                               chrono::ChQuaternion<> relRot,
                               Real radius,
                               Real height,
                               Real kernel_h,
                               bool cartesian = true);
CH_FSI_API void AddConeBce(std::shared_ptr<ChFsiDataManager> fsiData,
                           std::shared_ptr<SimParams> paramsH,
                           std::shared_ptr<ChBody> body,
                           chrono::ChVector<> relPos,
                           chrono::ChQuaternion<> relRot,
                           Real radius,
                           Real height,
                           Real kernel_h,
                           bool cartesian = true);
CH_FSI_API void AddCylinderSurfaceBce(std::shared_ptr<ChFsiDataManager> fsiData,
                                      std::shared_ptr<SimParams> paramsH,
                                      std::shared_ptr<ChBody> body,
                                      ChVector<> relPos,
                                      ChQuaternion<> relRot,
                                      Real radius,
                                      Real height,
                                      Real kernel_h);
CH_FSI_API void AddSphereSurfaceBce(std::shared_ptr<ChFsiDataManager> fsiData,
                                    std::shared_ptr<SimParams> paramsH,
                                    std::shared_ptr<ChBody> body,
                                    ChVector<> relPos,
                                    ChQuaternion<> relRot,
                                    Real radius,
                                    Real kernel_h);
CH_FSI_API void AddBoxBce(std::shared_ptr<ChFsiDataManager> fsiData,
                          std::shared_ptr<SimParams> paramsH,
                          std::shared_ptr<ChBody> body,
                          chrono::ChVector<> relPos,
                          chrono::ChQuaternion<> relRot,
                          const chrono::ChVector<>& size,
                          int plane = 12,
                          bool isSolid = false,
                          bool add_to_previous = false);

CH_FSI_API void AddBCE_FromPoints(std::shared_ptr<ChFsiDataManager> fsiData,
                                  std::shared_ptr<SimParams> paramsH,
                                  std::shared_ptr<ChBody> body,
                                  const std::vector<chrono::ChVector<>>& points,
                                  chrono::ChVector<> collisionShapeRelativePos = chrono::ChVector<>(0),
                                  chrono::ChQuaternion<> collisionShapeRelativeRot = chrono::QUNIT);

CH_FSI_API void AddBCE_FromFile(std::shared_ptr<ChFsiDataManager> fsiData,
                                std::shared_ptr<SimParams> paramsH,
                                std::shared_ptr<ChBody> body,
                                std::string dataPath,
                                chrono::ChVector<> collisionShapeRelativePos = chrono::ChVector<>(0),
                                chrono::ChQuaternion<> collisionShapeRelativeRot = chrono::QUNIT,
                                double scale = 1.0,
                                bool isSolid = true);

CH_FSI_API void CreateSphereFSI(std::shared_ptr<ChFsiDataManager> fsiData,
                                chrono::ChSystem& mphysicalSystem,
                                std::vector<std::shared_ptr<ChBody>>& fsiBodeis,
                                std::shared_ptr<SimParams> paramsH,
                                std::shared_ptr<chrono::ChMaterialSurface> mat_prop,
                                Real density,
                                chrono::ChVector<> pos,
                                Real radius);

CH_FSI_API void CreateCylinderFSI(std::shared_ptr<ChFsiDataManager> fsiData,
                                  chrono::ChSystem& mphysicalSystem,
                                  std::vector<std::shared_ptr<ChBody>>& fsiBodeis,
                                  std::shared_ptr<SimParams> paramsH,
                                  std::shared_ptr<chrono::ChMaterialSurface> mat_prop,
                                  Real density,
                                  chrono::ChVector<> pos,
                                  chrono::ChQuaternion<> rot,
                                  Real radius,
                                  Real length);

CH_FSI_API void CreateBoxFSI(std::shared_ptr<ChFsiDataManager> fsiData,
                             chrono::ChSystem& mphysicalSystem,
                             std::vector<std::shared_ptr<ChBody>>& fsiBodeis,
                             std::shared_ptr<SimParams> paramsH,
                             std::shared_ptr<chrono::ChMaterialSurface> mat_prop,
                             Real density,
                             chrono::ChVector<> pos,
                             chrono::ChQuaternion<> rot,
                             const chrono::ChVector<>& hsize);

CH_FSI_API void AddBCE_ShellANCF(std::shared_ptr<ChFsiDataManager> fsiData,
                                 std::shared_ptr<SimParams> paramsH,
                                 std::vector<std::shared_ptr<fea::ChElementShellANCF>>& fsiShells,
                                 std::shared_ptr<fea::ChMesh> my_mesh,
                                 bool multiLayer = true,
                                 bool removeMiddleLayer = false,
                                 int SIDE = -2);

CH_FSI_API void AddBCE_ShellFromMesh(std::shared_ptr<ChFsiDataManager> fsiData,
                                     std::shared_ptr<SimParams> paramsH,
                                     std::vector<std::shared_ptr<fea::ChElementShellANCF>>& fsiShells,
                                     std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>>& fsiNodes,
                                     std::shared_ptr<fea::ChMesh> my_mesh,
                                     const std::vector<std::vector<int>>& elementsNodes,
                                     const std::vector<std::vector<int>>& NodeNeighborElement,
                                     bool multiLayer = true,
                                     bool removeMiddleLayer = false,
                                     int SIDE = -2);

CH_FSI_API void AddBCE_FromMesh(std::shared_ptr<ChFsiDataManager> fsiData,
                                std::shared_ptr<SimParams> paramsH,
                                std::shared_ptr<fea::ChMesh> my_mesh,
                                std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>>& fsiNodes,
                                std::vector<std::shared_ptr<fea::ChElementCableANCF>>& fsiCables,
                                std::vector<std::shared_ptr<fea::ChElementShellANCF>>& fsiShells,
                                const std::vector<std::vector<int>>& NodeNeighborElement,
                                const std::vector<std::vector<int>>& _1D_elementsNodes,
                                const std::vector<std::vector<int>>& _2D_elementsNodes,
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
