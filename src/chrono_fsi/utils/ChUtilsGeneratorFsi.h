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
// Author: Arman Pazouki, Milad Rakhsha, Wei Hu
// =============================================================================
//
// Utility class for generating BCE markers.
// =============================================================================

#ifndef CH_FSI_UTILSGENERATOR_CUH
#define CH_FSI_UTILSGENERATOR_CUH

#include "chrono/ChConfig.h"
#include "chrono_fsi/ChSystemFsi_impl.cuh"
#include "chrono_fsi/utils/ChUtilsGeneratorBce.h"

namespace chrono {

// Forward declarations
namespace fea {
class ChElementCableANCF;
class ChElementShellANCF_3423;
class ChNodeFEAxyzD;
class ChMesh;
}  // namespace fea

namespace fsi {
namespace utils {

/// @addtogroup fsi_utils
/// @{

/// This utility function converts a given position and orientation, specified
/// with respect to a body's reference frame, into a frame defined with respect
/// to the body's centroidal frame. Note that by default, a body's reference
/// frame is the centroidal frame. This is not true for a ChBodyAuxRef.
ChVector<> TransformBCEToCOG(std::shared_ptr<ChBody> body, const ChVector<>& pos);

/// This utility function converts a given position and orientation, specified
/// with respect to a body's reference frame, into a frame defined with respect
/// to the body's centroidal frame. Note that by default, a body's reference
/// frame is the centroidal frame. This is not true for a ChBodyAuxRef.
ChVector<> TransformBCEToCOG(std::shared_ptr<ChBody> body, const Real3& pos3);

/// Set subdomains to find neighbor particles faster.
CH_FSI_API void FinalizeDomain(std::shared_ptr<SimParams> paramsH);

/// Create BCE particles from the local position on a body.
CH_FSI_API void CreateBceGlobalMarkersFromBceLocalPos(std::shared_ptr<ChSystemFsi_impl> fsiSystem,
                                                      std::shared_ptr<SimParams> paramsH,
                                                      const thrust::host_vector<Real4>& posRadBCE,
                                                      std::shared_ptr<ChBody> body,
                                                      const ChVector<>& collisionShapeRelativePos = ChVector<>(0),
                                                      const ChQuaternion<>& collisionShapeRelativeRot = QUNIT,
                                                      bool isSolid = true,
                                                      bool add_to_fluid_helpers = false,
                                                      bool add_to_previous_object = false);

/// Create BCE particles from the local position on a boundary.
CH_FSI_API void CreateBceGlobalMarkersFromBceLocalPosBoundary(std::shared_ptr<ChSystemFsi_impl> fsiSystem,
                                                              std::shared_ptr<SimParams> paramsH,
                                                              const thrust::host_vector<Real4>& posRadBCE,
                                                              std::shared_ptr<ChBody> body,
                                                              const ChVector<>& collisionShapeRelativePos,
                                                              const ChQuaternion<>& collisionShapeRelativeRot,
                                                              bool isSolid = false,
                                                              bool add_to_previous = true);

/// Add BCE particles genetrated from a sphere.
CH_FSI_API void AddSphereBce(std::shared_ptr<ChSystemFsi_impl> fsiSystem,
                             std::shared_ptr<SimParams> paramsH,
                             std::shared_ptr<ChBody> body,
                             const ChVector<>& relPos,
                             const ChQuaternion<>& relRot,
                             Real radius);

/// Add BCE particles genetrated from a cylinder.
CH_FSI_API void AddCylinderBce(std::shared_ptr<ChSystemFsi_impl> fsiSystem,
                               std::shared_ptr<SimParams> paramsH,
                               std::shared_ptr<ChBody> body,
                               const ChVector<>& relPos,
                               const ChQuaternion<>& relRot,
                               Real radius,
                               Real height,
                               Real kernel_h,
                               bool cartesian = true);

/// Add BCE particles genetrated from a cone.
CH_FSI_API void AddConeBce(std::shared_ptr<ChSystemFsi_impl> fsiSystem,
                           std::shared_ptr<SimParams> paramsH,
                           std::shared_ptr<ChBody> body,
                           const ChVector<>& relPos,
                           const ChQuaternion<>& relRot,
                           Real radius,
                           Real height,
                           Real kernel_h,
                           bool cartesian = true);

/// Add BCE particles genetrated from the surface of a cylinder.
CH_FSI_API void AddCylinderSurfaceBce(std::shared_ptr<ChSystemFsi_impl> fsiSystem,
                                      std::shared_ptr<SimParams> paramsH,
                                      std::shared_ptr<ChBody> body,
                                      const ChVector<>& relPos,
                                      const ChQuaternion<>& relRot,
                                      Real radius,
                                      Real height,
                                      Real kernel_h);

/// Add BCE particles genetrated from the surface of a sphere.
CH_FSI_API void AddSphereSurfaceBce(std::shared_ptr<ChSystemFsi_impl> fsiSystem,
                                    std::shared_ptr<SimParams> paramsH,
                                    std::shared_ptr<ChBody> body,
                                    const ChVector<>& relPos,
                                    const ChQuaternion<>& relRot,
                                    Real radius,
                                    Real kernel_h);

/// Add BCE particles genetrated from a box.
CH_FSI_API void AddBoxBce(std::shared_ptr<ChSystemFsi_impl> fsiSystem,
                          std::shared_ptr<SimParams> paramsH,
                          std::shared_ptr<ChBody> body,
                          const ChVector<>& relPos,
                          const ChQuaternion<>& relRot,
                          const ChVector<>& size,
                          int plane = 12,
                          bool isSolid = false,
                          bool add_to_previous = false);

/// Add BCE particles genetrated from point information.
CH_FSI_API void AddBCE_FromPoints(std::shared_ptr<ChSystemFsi_impl> fsiSystem,
                                  std::shared_ptr<SimParams> paramsH,
                                  std::shared_ptr<ChBody> body,
                                  const std::vector<ChVector<>>& points,
                                  const ChVector<>& collisionShapeRelativePos = ChVector<>(0),
                                  const ChQuaternion<>& collisionShapeRelativeRot = QUNIT);

/// Add BCE particles loaded from a file.
CH_FSI_API void AddBCE_FromFile(std::shared_ptr<ChSystemFsi_impl> fsiSystem,
                                std::shared_ptr<SimParams> paramsH,
                                std::shared_ptr<ChBody> body,
                                std::string dataPath,
                                const ChVector<>& collisionShapeRelativePos = ChVector<>(0),
                                const ChQuaternion<>& collisionShapeRelativeRot = QUNIT,
                                double scale = 1.0,
                                bool isSolid = true);

/// Create an FSI body for a sphere.
CH_FSI_API void CreateSphereFSI(std::shared_ptr<ChSystemFsi_impl> fsiSystem,
                                ChSystem& mphysicalSystem,
                                std::vector<std::shared_ptr<ChBody>>& fsiBodies,
                                std::shared_ptr<SimParams> paramsH,
                                std::shared_ptr<ChMaterialSurface> mat_prop,
                                Real density,
                                const ChVector<>& pos,
                                Real radius);

/// Create an FSI body for a cylinder.
CH_FSI_API void CreateCylinderFSI(std::shared_ptr<ChSystemFsi_impl> fsiSystem,
                                  ChSystem& mphysicalSystem,
                                  std::vector<std::shared_ptr<ChBody>>& fsiBodies,
                                  std::shared_ptr<SimParams> paramsH,
                                  std::shared_ptr<ChMaterialSurface> mat_prop,
                                  Real density,
                                  const ChVector<>& pos,
                                  const ChQuaternion<>& rot,
                                  Real radius,
                                  Real length);

/// Create an FSI body for a box.
CH_FSI_API void CreateBoxFSI(std::shared_ptr<ChSystemFsi_impl> fsiSystem,
                             ChSystem& mphysicalSystem,
                             std::vector<std::shared_ptr<ChBody>>& fsiBodies,
                             std::shared_ptr<SimParams> paramsH,
                             std::shared_ptr<ChMaterialSurface> mat_prop,
                             Real density,
                             const ChVector<>& pos,
                             const ChQuaternion<>& rot,
                             const ChVector<>& hsize);

/// Add BCE particles genetrated from ANCF shell elements.
CH_FSI_API void AddBCE_ShellANCF(std::shared_ptr<ChSystemFsi_impl> fsiSystem,
                                 std::shared_ptr<SimParams> paramsH,
                                 std::vector<std::shared_ptr<fea::ChElementShellANCF_3423>>& fsiShells,
                                 std::shared_ptr<fea::ChMesh> my_mesh,
                                 bool multiLayer = true,
                                 bool removeMiddleLayer = false,
                                 int SIDE = -2);

/// Add BCE particles genetrated from shell elements.
CH_FSI_API void AddBCE_ShellFromMesh(std::shared_ptr<ChSystemFsi_impl> fsiSystem,
                                     std::shared_ptr<SimParams> paramsH,
                                     std::vector<std::shared_ptr<fea::ChElementShellANCF_3423>>& fsiShells,
                                     std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>>& fsiNodes,
                                     std::shared_ptr<fea::ChMesh> my_mesh,
                                     const std::vector<std::vector<int>>& elementsNodes,
                                     const std::vector<std::vector<int>>& NodeNeighborElement,
                                     bool multiLayer = true,
                                     bool removeMiddleLayer = false,
                                     int SIDE = -2);

/// Add BCE particles genetrated from mesh.
CH_FSI_API void AddBCE_FromMesh(std::shared_ptr<ChSystemFsi_impl> fsiSystem,
                                std::shared_ptr<SimParams> paramsH,
                                std::shared_ptr<fea::ChMesh> my_mesh,
                                std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>>& fsiNodes,
                                std::vector<std::shared_ptr<fea::ChElementCableANCF>>& fsiCables,
                                std::vector<std::shared_ptr<fea::ChElementShellANCF_3423>>& fsiShells,
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

/// @} fsi_utils

}  // end namespace utils
}  // end namespace fsi
}  // end namespace chrono

#endif
