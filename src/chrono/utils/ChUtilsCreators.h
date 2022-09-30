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
// Authors: Radu Serban, Arman Pazouki
// =============================================================================
//
// Utility functions to facilitate adding contact and visualization geometry to
// a body:
//  - AddSphereGeometry
//  - AddEllipsoidGeometry
//  - AddBoxGeometry
//  - AddCapsuleGeometry
//  - AddCylinderGeometry
//  - AddConeGeometry
//  - AddTriangleMeshGeometry
//  - AddRoundedBoxGeometry
//  - AddRoundedCylinderGeometry
//  - AddTorusGeometry
//
// =============================================================================

#ifndef CH_UTILS_CREATORS_H
#define CH_UTILS_CREATORS_H

#include <cmath>
#include <string>
#include <vector>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChMaterialSurfaceNSC.h"
#include "chrono/physics/ChMaterialSurfaceSMC.h"
#include "chrono/physics/ChSystem.h"

#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono/collision/ChConvexDecomposition.h"

namespace chrono {
namespace utils {

/// @addtogroup chrono_utils
/// @{

/// Add a sphere collision shape and optionally a corresponding visualization asset to the specified body.
ChApi void AddSphereGeometry(ChBody* body,
                             std::shared_ptr<ChMaterialSurface> material,
                             double radius,
                             const ChVector<>& pos = ChVector<>(0, 0, 0),
                             const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
                             bool visualization = true,
                             std::shared_ptr<ChVisualMaterial> vis_material = ChVisualMaterial::Default());

/// Add an ellipsoid collision shape and optionally a corresponding visualization asset to the specified body.
ChApi void AddEllipsoidGeometry(ChBody* body,
                                std::shared_ptr<ChMaterialSurface> material,
                                const ChVector<>& size,
                                const ChVector<>& pos = ChVector<>(0, 0, 0),
                                const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
                                bool visualization = true,
                                std::shared_ptr<ChVisualMaterial> vis_material = ChVisualMaterial::Default());

/// Add a box collision shape and optionally a corresponding visualization asset to the specified body.
ChApi void AddBoxGeometry(ChBody* body,
                          std::shared_ptr<ChMaterialSurface> material,
                          const ChVector<>& size,
                          const ChVector<>& pos = ChVector<>(0, 0, 0),
                          const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
                          bool visualization = true,
                          std::shared_ptr<ChVisualMaterial> vis_material = ChVisualMaterial::Default());

/// Add a bisphere collision shape and optionally a corresponding visualization asset to the specified body.
ChApi void AddBiSphereGeometry(ChBody* body,
                               std::shared_ptr<ChMaterialSurface> material,
                               double radius,
                               double cDist,
                               const ChVector<>& pos = ChVector<>(0, 0, 0),
                               const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
                               bool visualization = true,
                               std::shared_ptr<ChVisualMaterial> vis_material = ChVisualMaterial::Default());

/// Add a box capsule shape and optionally a corresponding visualization asset to the specified body.
ChApi void AddCapsuleGeometry(ChBody* body,
                              std::shared_ptr<ChMaterialSurface> material,
                              double radius,
                              double hlen,
                              const ChVector<>& pos = ChVector<>(0, 0, 0),
                              const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
                              bool visualization = true,
                              std::shared_ptr<ChVisualMaterial> vis_material = ChVisualMaterial::Default());

/// Add a box cylinder shape and optionally a corresponding visualization asset to the specified body.
ChApi void AddCylinderGeometry(ChBody* body,
                               std::shared_ptr<ChMaterialSurface> material,
                               double radius,
                               double hlen,
                               const ChVector<>& pos = ChVector<>(0, 0, 0),
                               const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
                               bool visualization = true,
                               std::shared_ptr<ChVisualMaterial> vis_material = ChVisualMaterial::Default());

/// Add a box cone shape and optionally a corresponding visualization asset to the specified body.
ChApi void AddConeGeometry(ChBody* body,
                           std::shared_ptr<ChMaterialSurface> material,
                           double radius,
                           double height,
                           const ChVector<>& pos = ChVector<>(0, 0, 0),
                           const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
                           bool visualization = true,
                           std::shared_ptr<ChVisualMaterial> vis_material = ChVisualMaterial::Default());

/// Add a triangular mesh collision shape and optionally a corresponding visualization asset to the specified body.
ChApi bool AddTriangleMeshGeometry(ChBody* body,
                                   std::shared_ptr<ChMaterialSurface> material,
                                   const std::string& obj_filename,
                                   const std::string& name,
                                   const ChVector<>& pos = ChVector<>(0, 0, 0),
                                   const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
                                   bool visualization = true,
                                   std::shared_ptr<ChVisualMaterial> vis_material = ChVisualMaterial::Default());

/// Add convex hull collision shapes and optionally a corresponding visualization asset to the specified body.
ChApi bool AddTriangleMeshConvexDecomposition(
    ChBody* body,
    std::shared_ptr<ChMaterialSurface> material,
    const std::string& obj_filename,
    const std::string& name,
    const ChVector<>& pos = ChVector<>(0, 0, 0),
    const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
    float skin_thickness = 0.0f,
    bool use_original_asset = true,
    std::shared_ptr<ChVisualMaterial> vis_material = ChVisualMaterial::Default());

/// Add convex hull collision shapes and optionally a corresponding visualization asset to the specified body.
ChApi bool AddTriangleMeshConvexDecompositionV2(
    ChBody* body,
    std::shared_ptr<ChMaterialSurface> material,
    const std::string& obj_filename,
    const std::string& name,
    const ChVector<>& pos = ChVector<>(0, 0, 0),
    const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
    bool use_original_asset = true,
    std::shared_ptr<ChVisualMaterial> vis_material = ChVisualMaterial::Default());

/// Add convex hull collision shapes and optionally a corresponding visualization asset to the specified body.
ChApi bool AddTriangleMeshConvexDecompositionSplit(ChSystem* system,
                                                   std::shared_ptr<ChMaterialSurface> material,
                                                   const std::string& obj_filename,
                                                   const std::string& name,
                                                   const ChVector<>& pos,
                                                   const ChQuaternion<>& rot,
                                                   double total_mass);

/// Add a triangle collision shape and optionally a corresponding visualization asset to the specified body.
ChApi void AddTriangleGeometry(ChBody* body,
                               std::shared_ptr<ChMaterialSurface> material,
                               const ChVector<>& vertA,
                               const ChVector<>& vertB,
                               const ChVector<>& vertC,
                               const std::string& name,
                               const ChVector<>& pos = ChVector<>(0, 0, 0),
                               const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
                               bool visualization = true,
                               std::shared_ptr<ChVisualMaterial> vis_material = ChVisualMaterial::Default());

/// Add a rounded box (sphere-swept box) collision shape and optionally a corresponding visualization asset to the
/// specified body.
ChApi void AddRoundedBoxGeometry(ChBody* body,
                                 std::shared_ptr<ChMaterialSurface> material,
                                 const ChVector<>& size,
                                 double srad,
                                 const ChVector<>& pos = ChVector<>(0, 0, 0),
                                 const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
                                 bool visualization = true,
                                 std::shared_ptr<ChVisualMaterial> vis_material = ChVisualMaterial::Default());

/// Add a rounded cylinder (sphere-swept cylinder) collision shape and optionally a corresponding visualization asset
/// to the specified body.
ChApi void AddRoundedCylinderGeometry(ChBody* body,
                                      std::shared_ptr<ChMaterialSurface> material,
                                      double radius,
                                      double hlen,
                                      double srad,
                                      const ChVector<>& pos = ChVector<>(0, 0, 0),
                                      const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
                                      bool visualization = true,
                                      std::shared_ptr<ChVisualMaterial> vis_material = ChVisualMaterial::Default());

/// Add a torus collision shape (compound object created with capsules) and optionally a corresponding visualization
/// asset to the specified body.
ChApi void AddTorusGeometry(ChBody* body,
                            std::shared_ptr<ChMaterialSurface> material,
                            double radius,
                            double thickness,
                            int segments = 20,
                            int angle = 360,
                            const ChVector<>& pos = ChVector<>(0, 0, 0),
                            const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
                            bool visualization = true,
                            std::shared_ptr<ChVisualMaterial> vis_material = ChVisualMaterial::Default());

/// Add collision shapes representing a box container of specified dimensions to the given body.
/// The center of the container bottom face is at the origin of the given frame and the the container is aligned
/// with the frame axes. The container walls are constructed with specified thickness
/// 
/// The 'faces' input vector specifies which faces of the container are to be created: for each
/// direction, a value of -1 indicates the face in the negative direction, a value of +1 indicates the face in the
/// positive direction, and a value of 2 indicates both faces. Setting a value of 0 does not create container faces
/// in that direction.
ChApi void AddBoxContainer(std::shared_ptr<ChBody> body,
                           std::shared_ptr<ChMaterialSurface> material,
                           const ChFrame<>& frame,
                           const ChVector<>& size,
                           double thickness,
                           const ChVector<int> faces,
                           bool visualization = true,
                           std::shared_ptr<ChVisualMaterial> vis_material = ChVisualMaterial::Default());

/// Create a fixed body with contact and asset geometry representing a box with 5 walls (no top).
ChApi std::shared_ptr<ChBody> CreateBoxContainer(ChSystem* system,
                                                 int id,
                                                 std::shared_ptr<ChMaterialSurface> mat,
                                                 const ChVector<>& hdim,
                                                 double hthick,
                                                 const ChVector<>& pos = ChVector<>(0, 0, 0),
                                                 const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
                                                 bool collide = true,
                                                 bool y_up = false,
                                                 bool overlap = true,
                                                 bool closed = false);

/// Create a cylindrical container body with contact and asset geometry representing a cylindrical container modeled
/// with boxes. The container is aligned with the z direction. The position refers to the center of the bottom inner
/// circle. Only half of the cylinder is visualized.
ChApi std::shared_ptr<ChBody> CreateCylindricalContainerFromBoxes(
    ChSystem* system,
    int id,
    std::shared_ptr<ChMaterialSurface> mat,
    const ChVector<>& hdim,
    double hthick,
    int numBoxes,
    const ChVector<>& pos = ChVector<>(0, 0, 0),
    const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
    bool collide = true,
    bool overlap = true,
    bool closed = false,
    bool isBoxBase = true,
    bool partialVisualization = true);

/// Load an object from a Wavefront OBJ file and generate its convex decomposition.
ChApi bool LoadConvexMesh(const std::string& file_name,
                          geometry::ChTriangleMeshConnected& convex_mesh,
                          collision::ChConvexDecompositionHACDv2& convex_shape,
                          const ChVector<>& pos = ChVector<>(0, 0, 0),
                          const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
                          int hacd_maxhullcount = 1024,
                          int hacd_maxhullmerge = 256,
                          int hacd_maxhullvertexes = 64,
                          float hacd_concavity = 0.01f,
                          float hacd_smallclusterthreshold = 0.0f,
                          float hacd_fusetolerance = 1e-6f);

/// Given a path to an obj file, loads the obj assuming that the individual objects in the obj are convex hulls, useful
/// when loading a precomputed set of convex hulls. The output of this function is used with AddConvexCollisionModel
ChApi bool LoadConvexHulls(const std::string& file_name,
                           geometry::ChTriangleMeshConnected& convex_mesh,
                           std::vector<std::vector<ChVector<double>>>& convex_hulls);

/// Given a convex mesh and its decomposition add it to a ChBody use_original_asset can be used to specify if the mesh
/// or the convex decomp should be used for visualization
ChApi void AddConvexCollisionModel(std::shared_ptr<ChBody> body,
                                   std::shared_ptr<ChMaterialSurface> material,
                                   std::shared_ptr<geometry::ChTriangleMeshConnected> convex_mesh,
                                   collision::ChConvexDecompositionHACDv2& convex_shape,
                                   const ChVector<>& pos = ChVector<>(0, 0, 0),
                                   const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
                                   bool use_original_asset = true,
                                   std::shared_ptr<ChVisualMaterial> vis_material = ChVisualMaterial::Default());

/// Add a convex mesh to an object based on a set of points.
/// This version will use the triangle mesh to set the visualization geometry.
ChApi void AddConvexCollisionModel(std::shared_ptr<ChBody> body,
                                   std::shared_ptr<ChMaterialSurface> material,
                                   std::shared_ptr<geometry::ChTriangleMeshConnected> convex_mesh,
                                   std::vector<std::vector<ChVector<double>>>& convex_hulls,
                                   const ChVector<>& pos = ChVector<>(0, 0, 0),
                                   const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
                                   std::shared_ptr<ChVisualMaterial> vis_material = ChVisualMaterial::Default());

/// @} chrono_utils

}  // end namespace utils
}  // end namespace chrono

#endif
