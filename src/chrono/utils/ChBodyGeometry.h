// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Geometry definition for a body collision and visualization models
//
// =============================================================================

#ifndef CH_BODY_GEOMETRY_H
#define CH_BODY_GEOMETRY_H

#include <string>
#include <vector>

#include "chrono/core/ChApiCE.h"
#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChGeometry.h"

namespace chrono {
namespace utils {

/// @addtogroup chrono_utils
/// @{

/// Utility class defining geometry (visualization and collision) and contact materials for a rigid body.
/// Holds vectors of primitive shapes (any one of which may be empty) and a list of contact materials.
/// Each shape defines its position and orientation relative to the parent body, geometric dimensions, and an index into
/// the list of contact materials.
class ChApi ChBodyGeometry {
  public:
    /// Body visualization mode.
    enum class VisualizationType {
        NONE,        ///< no visualization
        PRIMITIVES,  ///< use primitve shapes (create primitive ChVisualShape objects)
        MESH,        ///< use meshes (create a ChVisualShapeTriangleMesh)
        MODEL_FILE,  ///< use a data file (create a ChVisualShapeModelFile)
        COLLISION    ///< visualize collision shapes
    };

    ChBodyGeometry();

    /// Box shape for visualization and/or collision.
    struct ChApi BoxShape {
        BoxShape(const ChVector3d& pos, const ChQuaternion<>& rot, const ChVector3d& dims, int matID = -1);
        ChVector3d pos;      ///< center position relative to body
        ChQuaternion<> rot;  ///< orientation relative to body
        ChVector3d dims;     ///< box dimensions
        int matID;           ///< index in contact material list
    };

    /// Sphere shape for visualization and/or collision.
    struct ChApi SphereShape {
        SphereShape(const ChVector3d& pos, double radius, int matID = -1);
        ChVector3d pos;  ///< center position relative to body
        double radius;   ///< sphere radius
        int matID;       ///< index in contact material list
    };

    /// Cylinder shape for visualization and/or collision.
    struct ChApi CylinderShape {
        CylinderShape(const ChVector3d& pos, const ChVector3d& axis, double radius, double length, int matID = -1);
        CylinderShape(const ChVector3d& pos, const ChQuaternion<>& rot, double radius, double length, int matID = -1);
        ChVector3d pos;      ///< center position relative to body
        ChQuaternion<> rot;  ///< orientation relative to body
        double radius;       ///< cylinder radius
        double length;       ///< cylinder length
        int matID;           ///< index in contact material list
    };

    /// Line shape for visualization.
    struct ChApi LineShape {
        LineShape(const ChVector3d& pos, const ChQuaternion<>& rot, std::shared_ptr<ChLine> line);
        ChVector3d pos;                ///< position relative to body
        ChQuaternion<> rot;            ///< orientation relative to body
        std::shared_ptr<ChLine> line;  ///< line data
    };

    /// Convex hulls shape for collision.
    struct ChApi ConvexHullsShape {
        ConvexHullsShape(const std::string& filename, int matID = -1);
        std::vector<std::vector<ChVector3d>> hulls;  ///< convex hulls in group
        int matID;                                   ///< index in contact material list
    };

    /// Tri-mesh shape for collision.
    struct ChApi TrimeshShape {
        TrimeshShape(const ChVector3d& pos, const std::string& filename, double scale, double radius = 0, int matID = -1);
        TrimeshShape(const ChVector3d& pos,
                     std::shared_ptr<ChTriangleMeshConnected> trimesh,
                     double radius = 0,
                     int matID = -1);
        std::shared_ptr<ChTriangleMeshConnected> trimesh;  ///< triangular mesh
        double radius;                                     ///< radius of sweeping sphere
        ChVector3d pos;                                    ///< position relative to body
        int matID;                                         ///< index in contact material list
    };

    std::vector<ChContactMaterialData> materials;  ///< list of contact materials
    std::vector<BoxShape> coll_boxes;              ///< list of collision boxes
    std::vector<SphereShape> coll_spheres;         ///< list of collision spheres
    std::vector<CylinderShape> coll_cylinders;     ///< list of collision cylinders
    std::vector<ConvexHullsShape> coll_hulls;      ///< list of collision convex hulls
    std::vector<TrimeshShape> coll_meshes;         ///< list of collision trimeshes

    std::vector<BoxShape> vis_boxes;           ///< list of visualization boxes
    std::vector<SphereShape> vis_spheres;      ///< list of visualization spheres
    std::vector<CylinderShape> vis_cylinders;  ///< list of visualization cylinders
    std::vector<LineShape> vis_lines;          ///< list of visualization lines

    ChColor color_boxes;      ///< visualization color
    ChColor color_spheres;    ///< visualization color
    ChColor color_cylinders;  ///< visualization color

    std::string vis_mesh_file;  ///< name of Wavefront OBJ file with visualization mesh

    /// Create visualization assets for the specified body.
    void CreateVisualizationAssets(std::shared_ptr<ChBody> body, VisualizationType vis = VisualizationType::MODEL_FILE);

    /// Create collision shapes for the specified body.
    void CreateCollisionShapes(std::shared_ptr<ChBody> body, int collision_family, ChContactMethod contact_method);

    /// Utility function for adding a cylinder visualization shape defined by the end points and a radius.
    /// This function adds the visualization shape to the body's visual model and returns the shape.
    static std::shared_ptr<ChVisualShape> AddVisualizationCylinder(std::shared_ptr<ChBody> body,
                                                                   const ChVector3d& p1,
                                                                   const ChVector3d& p2,
                                                                   double radius,
                                                                   ChVisualMaterialSharedPtr mat = nullptr);

    /// Calculate axis-aligned bounding box of all collision shapes.
    ChAABB CalculateAABB();
};

/// Utility class defining visualization geometry for a TSDA.
/// Holds vectors of segment and spring visualization shapes.
class ChApi ChTSDAGeometry {
  public:
    ChTSDAGeometry();

    /// Segment shape for TSDA visualization.
    struct ChApi SegmentShape {
        SegmentShape() {}
    };

    /// Spring shape for TSDA visualization.
    struct ChApi SpringShape {
        SpringShape(double radius, int resolution, double turns);
        double radius;
        double turns;
        int resolution;
    };

    std::shared_ptr<SegmentShape> vis_segment;  ///< visualization segment
    std::shared_ptr<SpringShape> vis_spring;    ///< visualization spring
    ChColor color;                              ///< visualization color

    /// Create visualization assets for the specified TSDA.
    void CreateVisualizationAssets(std::shared_ptr<ChLinkTSDA> tsda);
};

/// @} chrono_utils

}  // end namespace utils
}  // end namespace chrono

#endif
