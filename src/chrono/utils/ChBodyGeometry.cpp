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

#include <limits>

#include "chrono/core/ChGlobal.h"

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeCone.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChBodyGeometry.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace utils {

ChBodyGeometry::ChBodyGeometry()
    : color_boxes(ChColor(0.75f, 0.75f, 0.75f)),
      color_spheres(ChColor(0.75f, 0.75f, 0.75f)),
      color_cylinders(ChColor(0.75f, 0.75f, 0.75f)),
      color_cones(ChColor(0.75f, 0.75f, 0.75f)),
      color_meshes(ChColor(0.75f, 0.75f, 0.75f)) {}

ChBodyGeometry::BoxShape::BoxShape(const ChVector3d& pos, const ChQuaternion<>& rot, const ChVector3d& dims, int matID)
    : pos(pos), rot(rot), dims(dims), matID(matID), color({-1, -1, -1}) {}

ChBodyGeometry::BoxShape::BoxShape(const ChVector3d& pos, const ChQuaternion<>& rot, const ChBox& box, int matID)
    : pos(pos), rot(rot), dims(box.GetLengths()), matID(matID), color({-1, -1, -1}) {}

ChBodyGeometry::SphereShape::SphereShape(const ChVector3d& pos, double radius, int matID)
    : pos(pos), radius(radius), matID(matID), color({-1, -1, -1}) {}

ChBodyGeometry::SphereShape::SphereShape(const ChVector3d& pos, const ChSphere& sphere, int matID)
    : pos(pos), radius(sphere.GetRadius()), matID(matID), color({-1, -1, -1}) {}

ChBodyGeometry::CylinderShape::CylinderShape(const ChVector3d& pos,
                                             const ChQuaternion<>& rot,
                                             double radius,
                                             double length,
                                             int matID)
    : pos(pos), rot(rot), radius(radius), length(length), matID(matID), color({-1, -1, -1}) {}

ChBodyGeometry::CylinderShape::CylinderShape(const ChVector3d& pos,
                                             const ChVector3d& axis,
                                             double radius,
                                             double length,
                                             int matID)
    : pos(pos), radius(radius), length(length), matID(matID), color({-1, -1, -1}) {
    ChMatrix33<> rot_mat;
    rot_mat.SetFromAxisX(axis);
    rot = rot_mat.GetQuaternion() * QuatFromAngleY(CH_PI_2);
}

ChBodyGeometry::CylinderShape::CylinderShape(const ChVector3d& pos,
                                             const ChQuaternion<>& rot,
                                             const ChCylinder& cylinder,
                                             int matID)
    : pos(pos),
      rot(rot),
      radius(cylinder.GetRadius()),
      length(cylinder.GetHeight()),
      matID(matID),
      color({-1, -1, -1}) {}

ChBodyGeometry::ConeShape::ConeShape(const ChVector3d& pos,
                                     const ChQuaternion<>& rot,
                                     double radius,
                                     double length,
                                     int matID)
    : pos(pos), rot(rot), radius(radius), length(length), matID(matID), color({-1, -1, -1}) {}

ChBodyGeometry::ConeShape::ConeShape(const ChVector3d& pos,
                                     const ChVector3d& axis,
                                     double radius,
                                     double length,
                                     int matID)
    : pos(pos), radius(radius), length(length), matID(matID), color({-1, -1, -1}) {
    ChMatrix33<> rot_mat;
    rot_mat.SetFromAxisX(axis);
    rot = rot_mat.GetQuaternion() * QuatFromAngleY(CH_PI_2);
}

ChBodyGeometry::ConeShape::ConeShape(const ChVector3d& pos, const ChQuaternion<>& rot, const ChCone& cone, int matID)
    : pos(pos), rot(rot), radius(cone.GetRadius()), length(cone.GetHeight()), matID(matID), color({-1, -1, -1}) {}

ChBodyGeometry::LineShape::LineShape(const ChVector3d& pos, const ChQuaternion<>& rot, std::shared_ptr<ChLine> line)
    : pos(pos), rot(rot), line(line) {}

ChBodyGeometry::ConvexHullsShape::ConvexHullsShape(const std::string& filename, int matID) : matID(matID) {
    ChTriangleMeshConnected mesh;
    utils::LoadConvexHulls(filename, mesh, hulls);
}

ChBodyGeometry::TrimeshShape::TrimeshShape(const ChVector3d& pos,
                                           const ChQuaternion<>& rot,
                                           const std::string& filename,
                                           double scale,
                                           double radius,
                                           int matID)
    : TrimeshShape(pos,
                   rot,
                   ChTriangleMeshConnected::CreateFromWavefrontFile(filename, true, true),
                   VNULL,
                   scale,
                   radius,
                   matID) {}

ChBodyGeometry::TrimeshShape::TrimeshShape(const ChVector3d& pos,
                                           const ChQuaternion<>& rot,
                                           std::shared_ptr<ChTriangleMeshConnected> trimesh,
                                           double scale,
                                           double radius,
                                           int matID)
    : TrimeshShape(pos, rot, trimesh, VNULL, scale, radius, matID) {}

ChBodyGeometry::TrimeshShape::TrimeshShape(const ChVector3d& pos,
                                           const ChQuaternion<>& rot,
                                           const std::string& filename,
                                           const ChVector3d& interior_point,
                                           double scale,
                                           double radius,
                                           int matID)
    : TrimeshShape(pos,
                   rot,
                   ChTriangleMeshConnected::CreateFromWavefrontFile(filename, true, true),
                   interior_point,
                   scale,
                   radius,
                   matID) {}

ChBodyGeometry::TrimeshShape::TrimeshShape(const ChVector3d& pos,
                                           const ChQuaternion<>& rot,
                                           std::shared_ptr<ChTriangleMeshConnected> trimesh,
                                           const ChVector3d& interior_point,
                                           double scale,
                                           double radius,
                                           int matID)
    : trimesh(trimesh), radius(radius), matID(matID), int_point(interior_point), color({-1, -1, -1}) {
    ChMatrix33d R(rot);

    // Transform mesh vertices
    for (auto& v : trimesh->GetCoordsVertices()) {
        v *= scale;       // scale
        v = pos + R * v;  // rotate and translate
    }

    // Transform interior point
    int_point *= scale;
    int_point = pos + R * int_point;
}

std::shared_ptr<ChVisualShape> ChBodyGeometry::AddVisualizationCylinder(std::shared_ptr<ChBody> body,
                                                                        const ChVector3d& p1,
                                                                        const ChVector3d& p2,
                                                                        double radius,
                                                                        ChVisualMaterialSharedPtr mat) {
    ChLineSegment seg(p1, p2);
    auto cyl = chrono_types::make_shared<ChVisualShapeCylinder>(radius, seg.GetLength());
    if (mat)
        cyl->AddMaterial(mat);
    body->AddVisualShape(cyl, seg.GetFrame());
    return cyl;
}

void ChBodyGeometry::CreateVisualizationAssets(std::shared_ptr<ChBody> body, VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    // Create a visual model if one doesn't exist
    if (!body->GetVisualModel()) {
        auto model = chrono_types::make_shared<ChVisualModel>();
        body->AddVisualModel(model);
    }

    // Create default diffuse color materials for primitive shapes
    auto box_mat = chrono_types::make_shared<ChVisualMaterial>();
    auto sph_mat = chrono_types::make_shared<ChVisualMaterial>();
    auto cyl_mat = chrono_types::make_shared<ChVisualMaterial>();
    auto cone_mat = chrono_types::make_shared<ChVisualMaterial>();
    auto mesh_mat = chrono_types::make_shared<ChVisualMaterial>();

    box_mat->SetDiffuseColor({color_boxes.R, color_boxes.G, color_boxes.B});
    sph_mat->SetDiffuseColor({color_spheres.R, color_spheres.G, color_spheres.B});
    cyl_mat->SetDiffuseColor({color_cylinders.R, color_cylinders.G, color_cylinders.B});
    cone_mat->SetDiffuseColor({color_cones.R, color_cones.G, color_cones.B});
    mesh_mat->SetDiffuseColor({color_meshes.R, color_meshes.G, color_meshes.B});

    // Use the collision shapes
    if (vis == VisualizationType::COLLISION) {
        for (auto& sphere : coll_spheres) {
            auto sphere_shape = chrono_types::make_shared<ChVisualShapeSphere>(sphere.radius);
            if (sphere.color.R < 0 || sphere.color.G < 0 || sphere.color.B < 0)
                sphere_shape->AddMaterial(sph_mat);
            else
                sphere_shape->SetColor(sphere.color);
            body->AddVisualShape(sphere_shape, ChFrame<>(sphere.pos));
        }

        for (auto& box : coll_boxes) {
            auto box_shape = chrono_types::make_shared<ChVisualShapeBox>(box.dims);
            if (box.color.R < 0 || box.color.G < 0 || box.color.B < 0)
                box_shape->AddMaterial(box_mat);
            else
                box_shape->SetColor(box.color);
            body->AddVisualShape(box_shape, ChFrame<>(box.pos, box.rot));
        }

        for (auto& cyl : coll_cylinders) {
            auto cyl_shape = chrono_types::make_shared<ChVisualShapeCylinder>(cyl.radius, cyl.length);
            if (cyl.color.R < 0 || cyl.color.G < 0 || cyl.color.B < 0)
                cyl_shape->AddMaterial(cyl_mat);
            else
                cyl_shape->SetColor(cyl.color);
            body->AddVisualShape(cyl_shape, ChFrame<>(cyl.pos, cyl.rot));
        }

        for (auto& cone : coll_cones) {
            auto cone_shape = chrono_types::make_shared<ChVisualShapeCone>(cone.radius, cone.length);
            if (cone.color.R < 0 || cone.color.G < 0 || cone.color.B < 0)
                cone_shape->AddMaterial(cone_mat);
            else
                cone_shape->SetColor(cone.color);
            body->AddVisualShape(cone_shape, ChFrame<>(cone.pos, cone.rot));
        }

        for (auto& mesh : coll_meshes) {
            auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            trimesh_shape->SetMesh(mesh.trimesh);
            trimesh_shape->SetMutable(false);
            if (mesh.color.R < 0 || mesh.color.G < 0 || mesh.color.B < 0)
                trimesh_shape->AddMaterial(mesh_mat);
            else
                trimesh_shape->SetColor(mesh.color);
            body->AddVisualShape(trimesh_shape, ChFrame<>());
        }

        return;
    }

    // Use a model file if provided
    if (vis == VisualizationType::MESH && !vis_model_file.empty()) {
        auto obj_shape = chrono_types::make_shared<ChVisualShapeModelFile>();
        obj_shape->SetFilename(vis_model_file);
        body->AddVisualShape(obj_shape, ChFrame<>());
        return;
    }

    // If no model file specified, default to primitives
    for (auto& sphere : vis_spheres) {
        auto sphere_shape = chrono_types::make_shared<ChVisualShapeSphere>(sphere.radius);
        if (sphere.color.R < 0 || sphere.color.G < 0 || sphere.color.B < 0)
            sphere_shape->AddMaterial(sph_mat);
        else
            sphere_shape->SetColor(sphere.color);
        body->AddVisualShape(sphere_shape, ChFrame<>(sphere.pos));
    }

    for (auto& box : vis_boxes) {
        auto box_shape = chrono_types::make_shared<ChVisualShapeBox>(box.dims);
        if (box.color.R < 0 || box.color.G < 0 || box.color.B < 0)
            box_shape->AddMaterial(box_mat);
        else
            box_shape->SetColor(box.color);
        body->AddVisualShape(box_shape, ChFrame<>(box.pos, box.rot));
    }

    for (auto& cyl : vis_cylinders) {
        auto cyl_shape = chrono_types::make_shared<ChVisualShapeCylinder>(cyl.radius, cyl.length);
        if (cyl.color.R < 0 || cyl.color.G < 0 || cyl.color.B < 0)
            cyl_shape->AddMaterial(cyl_mat);
        else
            cyl_shape->SetColor(cyl.color);
        body->AddVisualShape(cyl_shape, ChFrame<>(cyl.pos, cyl.rot));
    }

    for (auto& cone : vis_cones) {
        auto cone_shape = chrono_types::make_shared<ChVisualShapeCone>(cone.radius, cone.length);
        if (cone.color.R < 0 || cone.color.G < 0 || cone.color.B < 0)
            cone_shape->AddMaterial(cone_mat);
        else
            cone_shape->SetColor(cone.color);
        body->AddVisualShape(cone_shape, ChFrame<>(cone.pos, cone.rot));
    }

    for (auto& line : vis_lines) {
        auto line_shape = chrono_types::make_shared<ChVisualShapeLine>();
        line_shape->SetLineGeometry(line.line);
        body->AddVisualShape(line_shape, ChFrame<>(line.pos, line.rot));
    }

    for (auto& mesh : vis_meshes) {
        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(mesh.trimesh);
        trimesh_shape->SetMutable(false);
        if (mesh.color.R < 0 || mesh.color.G < 0 || mesh.color.B < 0)
            trimesh_shape->AddMaterial(mesh_mat);
        else
            trimesh_shape->SetColor(mesh.color);
        body->AddVisualShape(trimesh_shape, ChFrame<>());
    }

    return;
}

void ChBodyGeometry::CreateCollisionShapes(std::shared_ptr<ChBody> body,
                                           int collision_family,
                                           ChContactMethod contact_method) {
    std::vector<std::shared_ptr<ChContactMaterial>> cmaterials;
    for (const auto& minfo : materials) {
        cmaterials.push_back(minfo.CreateMaterial(contact_method));
    }

    body->EnableCollision(true);

    for (auto& sphere : coll_spheres) {
        assert(cmaterials[sphere.matID]);
        auto shape = chrono_types::make_shared<ChCollisionShapeSphere>(cmaterials[sphere.matID], sphere.radius);
        body->AddCollisionShape(shape, ChFrame<>(sphere.pos, QUNIT));
    }
    for (auto& box : coll_boxes) {
        assert(cmaterials[box.matID]);
        auto shape = chrono_types::make_shared<ChCollisionShapeBox>(cmaterials[box.matID], box.dims.x(), box.dims.y(),
                                                                    box.dims.z());
        body->AddCollisionShape(shape, ChFrame<>(box.pos, box.rot));
    }
    for (auto& cyl : coll_cylinders) {
        assert(cmaterials[cyl.matID]);
        auto shape = chrono_types::make_shared<ChCollisionShapeCylinder>(cmaterials[cyl.matID], cyl.radius, cyl.length);
        body->AddCollisionShape(shape, ChFrame<>(cyl.pos, cyl.rot));
    }
    for (auto& cone : coll_cones) {
        assert(cmaterials[cone.matID]);
        auto shape = chrono_types::make_shared<ChCollisionShapeCone>(cmaterials[cone.matID], cone.radius, cone.length);
        body->AddCollisionShape(shape, ChFrame<>(cone.pos, cone.rot));
    }
    for (auto& hulls_group : coll_hulls) {
        assert(cmaterials[hulls_group.matID]);
        for (const auto& hull : hulls_group.hulls) {
            auto shape = chrono_types::make_shared<ChCollisionShapeConvexHull>(cmaterials[hulls_group.matID], hull);
            body->AddCollisionShape(shape);
        }
    }
    for (auto& mesh : coll_meshes) {
        assert(cmaterials[mesh.matID]);
        auto shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(cmaterials[mesh.matID], mesh.trimesh,
                                                                             false, false, mesh.radius);
        body->AddCollisionShape(shape);
    }

    body->GetCollisionModel()->SetFamily(collision_family);
}

ChAABB ChBodyGeometry::CalculateAABB() {
    ChVector3d amin(+std::numeric_limits<double>::max());
    ChVector3d amax(-std::numeric_limits<double>::max());

    for (const auto& box : coll_boxes) {
        ChMatrix33<> A(box.rot);
        A.cwiseAbs();
        auto tmp = A * (box.dims / 2);

        amin = Vmin(amin, box.pos - tmp);
        amax = Vmax(amax, box.pos + tmp);
    }

    for (const auto& sph : coll_spheres) {
        amin = Vmin(amin, sph.pos - sph.radius);
        amax = Vmin(amax, sph.pos + sph.radius);
    }

    for (const auto& cyl : coll_cylinders) {
        auto axis = cyl.rot.GetAxisZ();
        auto p1 = cyl.pos - (cyl.length / 2) * axis;
        auto p2 = cyl.pos + (cyl.length / 2) * axis;
        auto e2 = ChVector3d(1.0) - axis * axis;
        e2.Abs();
        ChVector3d e(std::sqrt(e2.x()), std::sqrt(e2.y()), std::sqrt(e2.z()));

        amin = Vmin(amin, Vmin(p1, p2) - cyl.radius * e);
        amax = Vmax(amax, Vmax(p1, p2) + cyl.radius * e);
    }

    for (const auto& cone : coll_cones) {
        //// TODO - for now, use the AABB of the circumscribed cylinder
        auto axis = cone.rot.GetAxisZ();
        auto p1 = cone.pos - (cone.length / 2) * axis;
        auto p2 = cone.pos + (cone.length / 2) * axis;
        auto e2 = ChVector3d(1.0) - axis * axis;
        e2.Abs();
        ChVector3d e(std::sqrt(e2.x()), std::sqrt(e2.y()), std::sqrt(e2.z()));

        amin = Vmin(amin, Vmin(p1, p2) - cone.radius * e);
        amax = Vmax(amax, Vmax(p1, p2) + cone.radius * e);
    }

    for (const auto& hulls_group : coll_hulls) {
        for (const auto& hull : hulls_group.hulls) {
            for (const auto& v : hull) {
                amin = Vmin(amin, v);
                amax = Vmax(amax, v);
            }
        }
    }

    for (const auto& mesh : coll_meshes) {
        auto bbox = mesh.trimesh->GetBoundingBox();
        amin = Vmin(amin, bbox.min);
        amax = Vmax(amax, bbox.max);
    }

    return ChAABB(amin, amax);
}

bool ChBodyGeometry::HasCollision() const {
    bool empty = coll_boxes.empty() && coll_spheres.empty() && coll_cylinders.empty() && coll_cones.empty() &&
                 coll_hulls.empty() && coll_meshes.empty();
    return !empty;
}

bool ChBodyGeometry::HasVisualizationPrimitives() const {
    bool empty =
        vis_boxes.empty() && vis_spheres.empty() && vis_cylinders.empty() && vis_cones.empty() && vis_lines.empty();
    return !empty;
}

bool ChBodyGeometry::HasVisualizationMesh() const {
    return !vis_model_file.empty();
}

std::string ChBodyGeometry::GetVisualizationTypeAsString(VisualizationType type) {
    switch (type) {
        case VisualizationType::NONE:
            return "NONE";
        case VisualizationType::PRIMITIVES:
            return "PRIMITIVES";
        case VisualizationType::MESH:
            return "MESH";
        case VisualizationType::COLLISION:
            return "COLLISION";
    }

    return "NONE";
}

// -----------------------------------------------------------------------------

ChTSDAGeometry::ChTSDAGeometry() : color(ChColor(1.0f, 1.0f, 1.0f)), vis_segment(nullptr), vis_spring(nullptr) {}

ChTSDAGeometry::SpringShape::SpringShape(double radius, int resolution, double turns)
    : radius(radius), turns(turns), resolution(resolution) {}

void ChTSDAGeometry::CreateVisualizationAssets(std::shared_ptr<ChLinkTSDA> tsda) {
    if (!tsda->GetVisualModel()) {
        auto model = chrono_types::make_shared<ChVisualModel>();
        tsda->AddVisualModel(model);
    }

    auto mat = chrono_types::make_shared<ChVisualMaterial>();
    mat->SetDiffuseColor(color);

    if (vis_spring) {
        auto spring_shape = chrono_types::make_shared<ChVisualShapeSpring>(vis_spring->radius, vis_spring->resolution,
                                                                           vis_spring->turns);
        spring_shape->AddMaterial(mat);
        tsda->AddVisualShape(spring_shape);
    }

    if (vis_segment) {
        auto segment_shape = chrono_types::make_shared<ChVisualShapeSegment>();
        segment_shape->AddMaterial(mat);
        tsda->AddVisualShape(segment_shape);
    }
}

}  // namespace utils
}  // end namespace chrono
