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
// Authors: Radu Serban
// =============================================================================
//
// Geometry definition for vehicle collision and visualization models
//
// =============================================================================

#ifndef CH_VEHICLE_GEOMETRY_H
#define CH_VEHICLE_GEOMETRY_H

#include <string>
#include <vector>

#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChVisualShape.h"
#include "chrono/geometry/ChGeometry.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

/// Utility class defining geometry (visualization and collision) and contact materials for a rigid vehicle body.
/// Holds vectors of primitive shapes (any one of which may be empty) and a list of contact materials.
/// Each shape defines its position and orientation relative to the parent body, geometric dimensions, and an index into
/// the list of contact materials.
class CH_VEHICLE_API ChVehicleGeometry {
  public:
    ChVehicleGeometry();

    /// Box shape for visualization and/or collision.
    struct CH_VEHICLE_API BoxShape {
        BoxShape(const ChVector<>& pos, const ChQuaternion<>& rot, const ChVector<>& dims, int matID = -1);
        ChVector<> m_pos;      ///< center position relative to body
        ChQuaternion<> m_rot;  ///< orientation relative to body
        ChVector<> m_dims;     ///< box dimensions
        int m_matID;           ///< index in contact material list
    };

    /// Sphere shape for visualization and/or collision.
    struct CH_VEHICLE_API SphereShape {
        SphereShape(const ChVector<>& pos, double radius, int matID = -1);
        ChVector<> m_pos;  ///< center position relative to body
        double m_radius;   ///< sphere radius
        int m_matID;       ///< index in contact material list
    };

    /// Cylinder shape for visualization and/or collision.
    struct CH_VEHICLE_API CylinderShape {
        CylinderShape(const ChVector<>& pos, const ChVector<>& axis, double radius, double length, int matID = -1);
        CylinderShape(const ChVector<>& pos, const ChQuaternion<>& rot, double radius, double length, int matID = -1);
        ChVector<> m_pos;      ///< center position relative to body
        ChQuaternion<> m_rot;  ///< orientation relative to body
        double m_radius;       ///< cylinder radius
        double m_length;       ///< cylinder length
        int m_matID;           ///< index in contact material list
    };

    /// Line shape for visualization.
    struct CH_VEHICLE_API LineShape {
        LineShape(const ChVector<>& pos, const ChQuaternion<>& rot, std::shared_ptr<geometry::ChLine> line);
        ChVector<> m_pos;                          ///< position relative to body
        ChQuaternion<> m_rot;                      ///< orientation relative to body
        std::shared_ptr<geometry::ChLine> m_line;  ///< line data
    };

    /// Convex hulls shape for collision.
    struct CH_VEHICLE_API ConvexHullsShape {
        ConvexHullsShape(const std::string& filename, int matID = -1);
        std::vector<std::vector<ChVector<>>> m_hulls;  ///< convex hulls in group
        int m_matID;                                   ///< index in contact material list
    };

    /// Tri-mesh shape for collision.
    struct CH_VEHICLE_API TrimeshShape {
        TrimeshShape(const ChVector<>& pos, const std::string& filename, double radius, int matID = -1);
        TrimeshShape(const ChVector<>& pos,
                     std::shared_ptr<geometry::ChTriangleMeshConnected> trimesh,
                     double radius,
                     int matID = -1);
        std::shared_ptr<geometry::ChTriangleMeshConnected> m_trimesh;  ///< triangular mesh
        double m_radius;                                               ///< radius of sweeping sphere
        ChVector<> m_pos;                                              ///< position relative to body
        int m_matID;                                                   ///< index in contact material list
    };

    bool m_has_collision;                            ///< true if body has a collision model
    std::vector<ChContactMaterialData> m_materials;  ///< list of contact materials
    std::vector<BoxShape> m_coll_boxes;              ///< list of collision boxes
    std::vector<SphereShape> m_coll_spheres;         ///< list of collision spheres
    std::vector<CylinderShape> m_coll_cylinders;     ///< list of collision cylinders
    std::vector<ConvexHullsShape> m_coll_hulls;      ///< list of collision convex hulls
    std::vector<TrimeshShape> m_coll_meshes;         ///< list of collision trimeshes

    bool m_has_primitives;                       ///< true if the body uses visualization primitives
    std::vector<BoxShape> m_vis_boxes;           ///< list of visualization boxes
    std::vector<SphereShape> m_vis_spheres;      ///< list of visualization spheres
    std::vector<CylinderShape> m_vis_cylinders;  ///< list of visualization cylinders
    std::vector<LineShape> m_vis_lines;          ///< list of visualization lines

    bool m_has_colors;          ///< true if primitive colors were provided
    ChColor m_color_boxes;      ///< visualization color
    ChColor m_color_spheres;    ///< visualization color
    ChColor m_color_cylinders;  ///< visualization color

    bool m_has_obj;               ///< true if the body uses visualization from an OBJ
    bool m_has_mesh;              ///< true if the body uses a visualization mesh
    std::string m_vis_mesh_file;  ///< name of Wavefront OBJ file with visualization mesh

    /// Create visualization assets for the specified body.
    void CreateVisualizationAssets(std::shared_ptr<ChBody> body,
                                   VisualizationType vis,
                                   bool visualize_collision = false);

    /// Create collision shapes for the specified body.
    void CreateCollisionShapes(std::shared_ptr<ChBody> body, int collision_family, ChContactMethod contact_method);

    /// Utility function for adding a cylinder visualization shape defined by the end points and a radius.
    /// This function adds the visualization shape to the body's visual model and returns the shape.
    static std::shared_ptr<ChVisualShape> AddVisualizationCylinder(std::shared_ptr<ChBody> body,
                                                                   const ChVector<>& p1,
                                                                   const ChVector<>& p2,
                                                                   double radius,
                                                                   ChVisualMaterialSharedPtr mat = nullptr);

    /// Calculate axis-aligned bounding box of all collision shapes.
    geometry::ChAABB CalculateAABB();
};

/// Utility class defining visualization geometry for a vehicle TSDA.
/// Holds vectors of segment and spring visualization shapes.
class CH_VEHICLE_API ChTSDAGeometry {
  public:
    ChTSDAGeometry();

    /// Segment shape for TSDA visualization.
    struct CH_VEHICLE_API SegmentShape {
        SegmentShape() {}
    };

    /// Spring shape for TSDA visualization.
    struct CH_VEHICLE_API SpringShape {
        SpringShape(double radius, int resolution, double turns);
        double m_radius;
        double m_turns;
        int m_resolution;
    };

    std::shared_ptr<SegmentShape> m_vis_segment;  ///< visualization segment
    std::shared_ptr<SpringShape> m_vis_spring;    ///< visualization spring
    bool m_has_color;                             ///< true if visualization color was provided
    ChColor m_color;                              ///< visualization color

    /// Create visualization assets for the specified TSDA.
    void CreateVisualizationAssets(std::shared_ptr<ChLinkTSDA> tsda, VisualizationType vis);
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
