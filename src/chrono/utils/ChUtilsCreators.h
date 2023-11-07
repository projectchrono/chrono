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
ChApi void AddSphereGeometry(
    ChBody* body,                                                         ///< associated body
    ChMaterialSurfaceSharedPtr material,                                  ///< contact material
    double radius,                                                        ///< sphere radius
    const ChVector<>& pos = ChVector<>(0, 0, 0),                          ///< position on body
    const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),               ///< orientation wrt body frame
    bool visualization = true,                                            ///< create visualization shape
    ChVisualMaterialSharedPtr vis_material = ChVisualMaterial::Default()  ///< visualization material
);

/// Add an ellipsoid collision shape and optionally a corresponding visualization asset to the specified body.
ChApi void AddEllipsoidGeometry(
    ChBody* body,                                                         ///< associated body
    ChMaterialSurfaceSharedPtr material,                                  ///< contact material
    const ChVector<>& axes,                                               ///< elipsoid axes
    const ChVector<>& pos = ChVector<>(0, 0, 0),                          ///< position on body
    const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),               ///< orientation wrt body frame
    bool visualization = true,                                            ///< create visualization shape
    ChVisualMaterialSharedPtr vis_material = ChVisualMaterial::Default()  ///< visualization material
);

/// Add a box collision shape and optionally a corresponding visualization asset to the specified body.
ChApi void AddBoxGeometry(
    ChBody* body,                                                         ///< associated body
    ChMaterialSurfaceSharedPtr material,                                  ///< contact material
    const ChVector<>& size,                                               ///< box side lengths
    const ChVector<>& pos = ChVector<>(0, 0, 0),                          ///< position on body
    const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),               ///< orientation wrt body frame
    bool visualization = true,                                            ///< create visualization shape
    ChVisualMaterialSharedPtr vis_material = ChVisualMaterial::Default()  ///< visualization material
);

/// Add a bisphere collision shape and optionally a corresponding visualization asset to the specified body.
ChApi void AddBiSphereGeometry(
    ChBody* body,                                                         ///< associated body
    ChMaterialSurfaceSharedPtr material,                                  ///< contact material
    double radius,                                                        ///< sphere radius
    double cDist,                                                         ///< distance between centers
    const ChVector<>& pos = ChVector<>(0, 0, 0),                          ///< position on body
    const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),               ///< orientation wrt body frame
    bool visualization = true,                                            ///< create visualization shape
    ChVisualMaterialSharedPtr vis_material = ChVisualMaterial::Default()  ///< visualization material
);

/// Add a capsule shape and optionally a corresponding visualization asset to the specified body.
ChApi void AddCapsuleGeometry(
    ChBody* body,                                                         ///< associated body
    ChMaterialSurfaceSharedPtr material,                                  ///< contact material
    double radius,                                                        ///< capsule radius
    double height,                                                        ///< capsule height (cylindrical portion)
    const ChVector<>& pos = ChVector<>(0, 0, 0),                          ///< position on body
    const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),               ///< orientation wrt body frame
    bool visualization = true,                                            ///< create visualization shape
    ChVisualMaterialSharedPtr vis_material = ChVisualMaterial::Default()  ///< visualization material
);

/// Add a cylinder shape and optionally a corresponding visualization asset to the specified body.
ChApi void AddCylinderGeometry(
    ChBody* body,                                                         ///< associated body
    ChMaterialSurfaceSharedPtr material,                                  ///< contact material
    double radius,                                                        ///< cylinder radius
    double height,                                                        ///< cylinder height
    const ChVector<>& pos = ChVector<>(0, 0, 0),                          ///< position on body
    const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),               ///< orientation wrt body frame
    bool visualization = true,                                            ///< create visualization shape
    ChVisualMaterialSharedPtr vis_material = ChVisualMaterial::Default()  ///< visualization material
);

/// Add a cylinder shape and optionally a corresponding visualization asset to the specified body.
ChApi void AddCylinderGeometry(
    ChBody* body,                                                         ///< associated body
    ChMaterialSurfaceSharedPtr material,                                  ///< contact material
    double radius,                                                        ///< cylinder radius
    const ChVector<>& p1,                                                 ///< first end point
    const ChVector<>& p2,                                                 ///< second end point
    bool visualization = true,                                            ///< create visualization shape
    ChVisualMaterialSharedPtr vis_material = ChVisualMaterial::Default()  ///< visualization material
);

/// Add a cone shape and optionally a corresponding visualization asset to the specified body.
ChApi void AddConeGeometry(
    ChBody* body,                                                         ///< associated body
    ChMaterialSurfaceSharedPtr material,                                  ///< contact material
    double radius,                                                        ///< cone radius
    double height,                                                        ///< cone height
    const ChVector<>& pos = ChVector<>(0, 0, 0),                          ///< position on body
    const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),               ///< orientation wrt body frame
    bool visualization = true,                                            ///< create visualization shape
    ChVisualMaterialSharedPtr vis_material = ChVisualMaterial::Default()  ///< visualization material
);

/// Add a triangular mesh collision shape and optionally a corresponding visualization asset to the specified body.
ChApi bool AddTriangleMeshGeometry(
    ChBody* body,                                                         ///< associated body
    ChMaterialSurfaceSharedPtr material,                                  ///< contact material
    const std::string& obj_filename,                                      ///< name of Wavefront OBJ file
    const std::string& name,                                              ///< mesh name
    const ChVector<>& pos = ChVector<>(0, 0, 0),                          ///< position on body
    const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),               ///< orientation wrt body frame
    bool visualization = true,                                            ///< create visualization shape
    ChVisualMaterialSharedPtr vis_material = ChVisualMaterial::Default()  ///< visualization material
);

/// Add convex hull collision shapes and optionally a corresponding visualization asset to the specified body.
ChApi bool AddTriangleMeshConvexDecomposition(ChBody* body,
                                              ChMaterialSurfaceSharedPtr material,
                                              const std::string& obj_filename,
                                              const std::string& name,
                                              const ChVector<>& pos = ChVector<>(0, 0, 0),
                                              const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
                                              float skin_thickness = 0.0f,
                                              bool use_original_asset = true,
                                              ChVisualMaterialSharedPtr vis_material = ChVisualMaterial::Default());

/// Add convex hull collision shapes and optionally a corresponding visualization asset to the specified body.
ChApi bool AddTriangleMeshConvexDecompositionV2(ChBody* body,
                                                ChMaterialSurfaceSharedPtr material,
                                                const std::string& obj_filename,
                                                const std::string& name,
                                                const ChVector<>& pos = ChVector<>(0, 0, 0),
                                                const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
                                                bool use_original_asset = true,
                                                ChVisualMaterialSharedPtr vis_material = ChVisualMaterial::Default());

/// Add convex hull collision shapes and optionally a corresponding visualization asset to the specified body.
ChApi bool AddTriangleMeshConvexDecompositionSplit(ChSystem* system,
                                                   ChMaterialSurfaceSharedPtr material,
                                                   const std::string& obj_filename,
                                                   const std::string& name,
                                                   const ChVector<>& pos,
                                                   const ChQuaternion<>& rot,
                                                   double total_mass);

/// Add a triangle collision shape and optionally a corresponding visualization asset to the specified body.
ChApi void AddTriangleGeometry(
    ChBody* body,                                                         ///< associated body
    ChMaterialSurfaceSharedPtr material,                                  ///< contact material
    const ChVector<>& vertA,                                              ///< first triangle vertex
    const ChVector<>& vertB,                                              ///< second triangle vertex
    const ChVector<>& vertC,                                              ///< third triangle vertex
    const std::string& name,                                              ///< name of generated mesh
    const ChVector<>& pos = ChVector<>(0, 0, 0),                          ///< position on body
    const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),               ///< orientation wrt body frame
    bool visualization = true,                                            ///< create visualization shape
    ChVisualMaterialSharedPtr vis_material = ChVisualMaterial::Default()  ///< visualization material
);

/// Add a rounded box (sphere-swept box) collision shape and optionally a corresponding visualization asset to the
/// specified body.
ChApi void AddRoundedBoxGeometry(
    ChBody* body,                                                         ///< associated body
    ChMaterialSurfaceSharedPtr material,                                  ///< contact material
    const ChVector<>& size,                                               ///< box side lengths
    double srad,                                                          ///< radius of sweeping sphere
    const ChVector<>& pos = ChVector<>(0, 0, 0),                          ///< position on body
    const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),               ///< orientation wrt body frame
    bool visualization = true,                                            ///< create visualization shape
    ChVisualMaterialSharedPtr vis_material = ChVisualMaterial::Default()  ///< visualization material
);

/// Add a rounded cylinder (sphere-swept cylinder) collision shape and optionally a corresponding visualization asset
/// to the specified body.
ChApi void AddRoundedCylinderGeometry(
    ChBody* body,                                                         ///< associated body
    ChMaterialSurfaceSharedPtr material,                                  ///< contact material
    double radius,                                                        ///< cylinder radius
    double height,                                                        ///< cylinder radius
    double srad,                                                          ///< radius of sweeping sphere
    const ChVector<>& pos = ChVector<>(0, 0, 0),                          ///< position on body
    const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),               ///< orientation wrt body frame
    bool visualization = true,                                            ///< create visualization shape
    ChVisualMaterialSharedPtr vis_material = ChVisualMaterial::Default()  ///< visualization material
);

/// Add a torus collision shape (compound object created with capsules) and optionally a corresponding visualization
/// asset to the specified body.
ChApi void AddTorusGeometry(ChBody* body,
                            ChMaterialSurfaceSharedPtr material,
                            double radius,
                            double thickness,
                            int segments = 20,
                            int angle = 360,
                            const ChVector<>& pos = ChVector<>(0, 0, 0),
                            const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
                            bool visualization = true,
                            ChVisualMaterialSharedPtr vis_material = ChVisualMaterial::Default());

/// Add collision shapes representing a box container of specified dimensions to the given body.
/// The center of the box volume is at the origin of the given frame and the the container is aligned with the frame
/// axes. The container walls are constructed with the specified thickness.
/// The 'faces' input vector specifies which faces of the container are to be created: for each direction, a value of -1
/// indicates the face in the negative direction, a value of +1 indicates the face in the positive direction, and a
/// value of 2 indicates both faces. Setting a value of 0 does not create container faces in that direction.
ChApi void AddBoxContainer(
    std::shared_ptr<ChBody> body,                                         ///< associated body
    ChMaterialSurfaceSharedPtr material,                                  ///< contact material
    const ChFrame<>& frame,                                               ///< position and orientation wrt body frame
    const ChVector<>& size,                                               ///< interior container dimensions
    double thickness,                                                     ///< wall thickness
    const ChVector<int> faces,                                            ///< indices of container faces to be created
    bool visualization = true,                                            ///< create visualization shape
    ChVisualMaterialSharedPtr vis_material = ChVisualMaterial::Default()  ///< visualization material
);

/// Create a fixed body with collision and visualization geometry representing a box volume.
/// The center of the bottom box wall is at the body origin.
ChApi std::shared_ptr<ChBody> CreateBoxContainer(
    ChSystem* system,                                        ///< containing system
    int id,                                                  ///< body identifier
    ChMaterialSurfaceSharedPtr mat,                          ///< contact material
    const ChVector<>& size,                                  ///< interior container dimensions
    double thickness,                                        ///< wall thickness
    const ChVector<>& pos = ChVector<>(0, 0, 0),             ///< body position
    const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),  ///< body orientation
    bool collide = true,                                     ///< enable collision
    bool overlap = true,                                     ///< include overlap at container edges
    bool closed = false                                      ///< create top wall
);

/// Create a fixed body with collision and visualization representing a cylindrical volume.
/// The volume geometry is modeled using boxes. The container is aligned with the z direction with the center of the
/// bottom at the body origin. Optionally, the side boxes in the first quadrant are not visualized.
ChApi std::shared_ptr<ChBody> CreateCylindricalContainerFromBoxes(
    ChSystem* system,                                        ///< containing system
    int id,                                                  ///< body identifier
    ChMaterialSurfaceSharedPtr mat,                          ///< contact material
    double radius,                                           ///< container inner radius
    double height,                                           ///< container inner height
    double thickness,                                        ///< wall thickness
    int numBoxes,                                            ///< number of circumference boxes
    const ChVector<>& pos = ChVector<>(0, 0, 0),             ///< body position
    const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),  ///< body orientation
    bool collide = true,                                     ///< enable collision
    bool overlap = true,                                     ///< include overlap at box edges
    bool closed = false,                                     ///< create top
    bool isBoxBase = true,                                   ///< use a box or a cylinder for bases
    bool partialVisualization = true                         ///< visualize only half of the boxes
);

/// Load an object from a Wavefront OBJ file and generate its convex decomposition.
ChApi bool LoadConvexMesh(const std::string& file_name,
                          geometry::ChTriangleMeshConnected& convex_mesh,
                          ChConvexDecompositionHACDv2& convex_shape,
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
                                   ChMaterialSurfaceSharedPtr material,
                                   std::shared_ptr<geometry::ChTriangleMeshConnected> convex_mesh,
                                   ChConvexDecompositionHACDv2& convex_shape,
                                   const ChVector<>& pos = ChVector<>(0, 0, 0),
                                   const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
                                   bool use_original_asset = true,
                                   ChVisualMaterialSharedPtr vis_material = ChVisualMaterial::Default());

/// Add a convex mesh to an object based on a set of points.
/// This version will use the triangle mesh to set the visualization geometry.
ChApi void AddConvexCollisionModel(std::shared_ptr<ChBody> body,
                                   ChMaterialSurfaceSharedPtr material,
                                   std::shared_ptr<geometry::ChTriangleMeshConnected> convex_mesh,
                                   std::vector<std::vector<ChVector<double>>>& convex_hulls,
                                   const ChVector<>& pos = ChVector<>(0, 0, 0),
                                   const ChQuaternion<>& rot = ChQuaternion<>(1, 0, 0, 0),
                                   ChVisualMaterialSharedPtr vis_material = ChVisualMaterial::Default());

/// @} chrono_utils

}  // end namespace utils
}  // end namespace chrono

#endif
