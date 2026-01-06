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

#include "chrono/geometry/ChBox.h"
#include "chrono/geometry/ChSphere.h"
#include "chrono/geometry/ChCone.h"
#include "chrono/geometry/ChCylinder.h"
#include "chrono/geometry/ChLine.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono/physics/ChContactMaterial.h"

#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChGeometry.h"

namespace chrono {

class ChBody;
class ChLinkTSDA;

/// Body visualization mode.
enum class VisualizationType {
    NONE,        ///< no visualization
    PRIMITIVES,  ///< use primitve shapes (create primitive ChVisualShape objects)
    MESH,        ///< use a mesh data file (create a ChVisualShapeModelFile)
    COLLISION    ///< visualize collision shapes
};

namespace utils {

/// @addtogroup chrono_utils
/// @{

/// Utility class defining geometry (visualization and collision) and contact materials for a rigid body.
/// Holds vectors of primitive shapes (any one of which may be empty) and a list of contact materials.
/// Each shape defines its position and orientation relative to the parent body, geometric dimensions, and an index into
/// the list of contact materials.
class ChApi ChBodyGeometry {
  public:
    ChBodyGeometry();

    /// Box shape for visualization and/or collision.
    struct ChApi BoxShape {
        BoxShape() = default;
        BoxShape(const ChVector3d& pos, const ChQuaternion<>& rot, const ChVector3d& dims, int matID = -1);
        BoxShape(const ChVector3d& pos, const ChQuaternion<>& rot, const ChBox& box, int matID = -1);
        ChVector3d pos;      ///< center position relative to body
        ChQuaternion<> rot;  ///< orientation relative to body
        ChVector3d dims;     ///< box dimensions
        int matID;           ///< index in contact material list
        ChColor color;       ///< visualization color
    };

    /// Sphere shape for visualization and/or collision.
    struct ChApi SphereShape {
        SphereShape() = default;
        SphereShape(const ChVector3d& pos, double radius, int matID = -1);
        SphereShape(const ChVector3d& pos, const ChSphere& sphere, int matID = -1);
        ChVector3d pos;  ///< center position relative to body
        double radius;   ///< sphere radius
        int matID;       ///< index in contact material list
        ChColor color;   ///< visualization color
    };

    /// Cylinder shape for visualization and/or collision.
    struct ChApi CylinderShape {
        CylinderShape() = default;
        CylinderShape(const ChVector3d& pos, const ChVector3d& axis, double radius, double length, int matID = -1);
        CylinderShape(const ChVector3d& pos, const ChQuaternion<>& rot, double radius, double length, int matID = -1);
        CylinderShape(const ChVector3d& pos, const ChQuaternion<>& rot, const ChCylinder& cylinder, int matID = -1);
        ChVector3d pos;      ///< center position relative to body
        ChQuaternion<> rot;  ///< orientation relative to body
        double radius;       ///< cylinder radius
        double length;       ///< cylinder length
        int matID;           ///< index in contact material list
        ChColor color;       ///< visualization color
    };

    /// Cone shape for visualization and/or collision.
    struct ChApi ConeShape {
        ConeShape() = default;
        ConeShape(const ChVector3d& pos, const ChVector3d& axis, double radius, double length, int matID = -1);
        ConeShape(const ChVector3d& pos, const ChQuaternion<>& rot, double radius, double length, int matID = -1);
        ConeShape(const ChVector3d& pos, const ChQuaternion<>& rot, const ChCone& cone, int matID = -1);
        ChVector3d pos;      ///< center position relative to body
        ChQuaternion<> rot;  ///< orientation relative to body
        double radius;       ///< cone radius
        double length;       ///< cone length
        int matID;           ///< index in contact material list
        ChColor color;       ///< visualization color
    };

    /// Tri-mesh shape for visualization and/or collision.
    struct ChApi TrimeshShape {
        TrimeshShape() = default;
        TrimeshShape(const ChVector3d& pos,
                     const ChQuaternion<>& rot, 
                     const std::string& filename,
                     double scale = 1,
                     double radius = 0,
                     int matID = -1);
        TrimeshShape(const ChVector3d& pos,
                     const ChQuaternion<>& rot, 
                     std::shared_ptr<ChTriangleMeshConnected> trimesh,
                     double scale = 1,
                     double radius = 0,
                     int matID = -1);

        TrimeshShape(const ChVector3d& pos,
                     const ChQuaternion<>& rot,
                     const std::string& filename,
                     const ChVector3d& interior_point,
                     double scale = 1,
                     double radius = 0,
                     int matID = -1);
        TrimeshShape(const ChVector3d& pos,
                     const ChQuaternion<>& rot,
                     std::shared_ptr<ChTriangleMeshConnected> trimesh,
                     const ChVector3d& interior_point,
                     double scale = 1,
                     double radius = 0,
                     int matID = -1);

        std::shared_ptr<ChTriangleMeshConnected> trimesh;  ///< triangular mesh
        ChVector3d int_point;                              ///< location of a point inside the mesh
        double radius;                                     ///< radius of sweeping sphere
        int matID;                                         ///< index in contact material list
        ChColor color;                                     ///< visualization color
        bool is_mutable;                                   ///< true if mesh is deformable
    };

    /// Line shape for visualization.
    struct ChApi LineShape {
        LineShape() = default; 
        LineShape(const ChVector3d& pos, const ChQuaternion<>& rot, std::shared_ptr<ChLine> line);
        ChVector3d pos;                ///< position relative to body
        ChQuaternion<> rot;            ///< orientation relative to body
        std::shared_ptr<ChLine> line;  ///< line data
    };

    /// Convex hulls shape for collision.
    struct ChApi ConvexHullsShape {
        ConvexHullsShape() = default;
        ConvexHullsShape(const std::string& filename, int matID = -1);
        std::vector<std::vector<ChVector3d>> hulls;  ///< convex hulls in group
        int matID;                                   ///< index in contact material list
        bool is_mutable;                             ///< true if hull is deformable
    };

    /// Create visualization assets for the specified body.
    void CreateVisualizationAssets(std::shared_ptr<ChBody> body, VisualizationType vis = VisualizationType::PRIMITIVES);

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

    /// Indicate whether or not collision shapes are defined.
    bool HasCollision() const;

    /// Indicate whether or not visualization primitives are defined.
    bool HasVisualizationPrimitives() const;

    /// Indicate whether or not a visualization mesh is defined.
    bool HasVisualizationMesh() const;

    static std::string GetVisualizationTypeAsString(VisualizationType type); 

  public:
    std::vector<ChContactMaterialData> materials;  ///< list of contact materials
    std::vector<BoxShape> coll_boxes;              ///< list of collision boxes
    std::vector<SphereShape> coll_spheres;         ///< list of collision spheres
    std::vector<CylinderShape> coll_cylinders;     ///< list of collision cylinders
    std::vector<ConeShape> coll_cones;             ///< list of cone cylinders
    std::vector<TrimeshShape> coll_meshes;         ///< list of collision trimeshes
    std::vector<ConvexHullsShape> coll_hulls;      ///< list of collision convex hulls

    std::vector<BoxShape> vis_boxes;           ///< list of visualization boxes
    std::vector<SphereShape> vis_spheres;      ///< list of visualization spheres
    std::vector<CylinderShape> vis_cylinders;  ///< list of visualization cylinders
    std::vector<ConeShape> vis_cones;          ///< list of visualization cones
    std::vector<TrimeshShape> vis_meshes;      ///< list of collision trimeshes
    std::vector<LineShape> vis_lines;          ///< list of visualization lines

    ChColor color_boxes;      ///< default visualization color for box primitives
    ChColor color_spheres;    ///< default visualization color for sphere primitives
    ChColor color_cylinders;  ///< default visualization color for cylinder primitives
    ChColor color_cones;      ///< default visualization color for cone primitives
    ChColor color_meshes;     ///< default visualization color for mesh primitives

    std::string vis_model_file;  ///< name of file with visualization mesh
};

/// Utility class defining visualization geometry for a TSDA.
/// Holds vectors of segment and spring visualization shapes.
class ChApi ChTSDAGeometry {
  public:
    ChTSDAGeometry();

    /// Segment shape for TSDA visualization.
    struct ChApi SegmentShape{SegmentShape(){}};

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
