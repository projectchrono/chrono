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
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChBodyGeometry.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace utils {

ChBodyGeometry::ChBodyGeometry()
    : color_boxes(ChColor(0.5f, 0.5f, 0.5f)),
      color_spheres(ChColor(0.5f, 0.5f, 0.5f)),
      color_cylinders(ChColor(0.5f, 0.5f, 0.5f)) {}

ChBodyGeometry::BoxShape::BoxShape(const ChVector3d& pos, const ChQuaternion<>& rot, const ChVector3d& dims, int matID)
    : pos(pos), rot(rot), dims(dims), matID(matID) {}

ChBodyGeometry::SphereShape::SphereShape(const ChVector3d& pos, double radius, int matID)
    : pos(pos), radius(radius), matID(matID) {}

ChBodyGeometry::CylinderShape::CylinderShape(const ChVector3d& pos,
                                             const ChQuaternion<>& rot,
                                             double radius,
                                             double length,
                                             int matID)
    : pos(pos), rot(rot), radius(radius), length(length), matID(matID) {}

ChBodyGeometry::CylinderShape::CylinderShape(const ChVector3d& pos,
                                             const ChVector3d& axis,
                                             double radius,
                                             double length,
                                             int matID)
    : pos(pos), radius(radius), length(length), matID(matID) {
    ChMatrix33<> rot;
    rot.SetFromAxisX(axis);
    rot = rot.GetQuaternion() * QuatFromAngleY(-CH_PI_2);
}

ChBodyGeometry::LineShape::LineShape(const ChVector3d& pos, const ChQuaternion<>& rot, std::shared_ptr<ChLine> line)
    : pos(pos), rot(rot), line(line) {}

ChBodyGeometry::ConvexHullsShape::ConvexHullsShape(const std::string& filename, int matID) : matID(matID) {
    ChTriangleMeshConnected mesh;
    utils::LoadConvexHulls(GetChronoDataFile(filename), mesh, hulls);
}

ChBodyGeometry::TrimeshShape::TrimeshShape(const ChVector3d& pos,
                                           const std::string& filename,
                                           double scale,
                                           double radius,
                                           int matID)
    : radius(radius), pos(pos), matID(matID) {
    trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(filename), true, false);
    for (auto& v : trimesh->GetCoordsVertices()) {
        v *= scale;
    }
}

ChBodyGeometry::TrimeshShape::TrimeshShape(const ChVector3d& pos,
                                           std::shared_ptr<ChTriangleMeshConnected> trimesh,
                                           double radius,
                                           int matID)
    : trimesh(trimesh), radius(radius), pos(pos), matID(matID) {}

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

    if (!body->GetVisualModel()) {
        auto model = chrono_types::make_shared<ChVisualModel>();
        body->AddVisualModel(model);
    }

    if (vis == VisualizationType::COLLISION) {
        auto coll_mat = chrono_types::make_shared<ChVisualMaterial>();
        coll_mat->SetDiffuseColor({0.8f, 0.56f, 0.0f});

        for (auto& sphere : coll_spheres) {
            auto sphere_shape = chrono_types::make_shared<ChVisualShapeSphere>(sphere.radius);
            sphere_shape->AddMaterial(coll_mat);
            body->AddVisualShape(sphere_shape, ChFrame<>(sphere.pos));
        }

        for (auto& box : coll_boxes) {
            auto box_shape = chrono_types::make_shared<ChVisualShapeBox>(box.dims);
            box_shape->AddMaterial(coll_mat);
            body->AddVisualShape(box_shape, ChFrame<>(box.pos, box.rot));
        }

        for (auto& cyl : coll_cylinders) {
            auto cyl_shape = chrono_types::make_shared<ChVisualShapeCylinder>(cyl.radius, cyl.length);
            cyl_shape->AddMaterial(coll_mat);
            body->AddVisualShape(cyl_shape, ChFrame<>(cyl.pos, cyl.rot));
        }

        for (auto& mesh : coll_meshes) {
            auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            trimesh_shape->SetMesh(mesh.trimesh);
            trimesh_shape->SetMutable(false);
            trimesh_shape->AddMaterial(coll_mat);
            body->AddVisualShape(trimesh_shape, ChFrame<>());
        }

        return;
    }

    if (vis == VisualizationType::MODEL_FILE && !vis_mesh_file.empty()) {
        auto obj_shape = chrono_types::make_shared<ChVisualShapeModelFile>();
        obj_shape->SetFilename(GetChronoDataFile(vis_mesh_file));
        body->AddVisualShape(obj_shape, ChFrame<>());
        return;
    }

    if (vis == VisualizationType::MESH && !vis_mesh_file.empty()) {
        auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(GetChronoDataFile(vis_mesh_file), true, true);
        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(vis_mesh_file).stem());
        trimesh_shape->SetMutable(false);
        body->AddVisualShape(trimesh_shape, ChFrame<>());
        return;
    }

    // If no mesh specified, default to primitives
    auto box_mat = chrono_types::make_shared<ChVisualMaterial>();
    auto sph_mat = chrono_types::make_shared<ChVisualMaterial>();
    auto cyl_mat = chrono_types::make_shared<ChVisualMaterial>();

    box_mat->SetDiffuseColor({color_boxes.R, color_boxes.G, color_boxes.B});
    sph_mat->SetDiffuseColor({color_spheres.R, color_spheres.G, color_spheres.B});
    cyl_mat->SetDiffuseColor({color_cylinders.R, color_cylinders.G, color_cylinders.B});

    for (auto& sphere : vis_spheres) {
        auto sphere_shape = chrono_types::make_shared<ChVisualShapeSphere>(sphere.radius);
        sphere_shape->AddMaterial(sph_mat);
        body->AddVisualShape(sphere_shape, ChFrame<>(sphere.pos));
    }

    for (auto& box : vis_boxes) {
        auto box_shape = chrono_types::make_shared<ChVisualShapeBox>(box.dims);
        box_shape->AddMaterial(box_mat);
        body->AddVisualShape(box_shape, ChFrame<>(box.pos, box.rot));
    }

    for (auto& cyl : vis_cylinders) {
        auto cyl_shape = chrono_types::make_shared<ChVisualShapeCylinder>(cyl.radius, cyl.length);
        cyl_shape->AddMaterial(cyl_mat);
        body->AddVisualShape(cyl_shape, ChFrame<>(cyl.pos, cyl.rot));
    }

    for (auto& line : vis_lines) {
        auto line_shape = chrono_types::make_shared<ChVisualShapeLine>();
        line_shape->SetLineGeometry(line.line);
        body->AddVisualShape(line_shape, ChFrame<>(line.pos, line.rot));
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
    for (auto& hulls_group : coll_hulls) {
        assert(cmaterials[hulls_group.matID]);
        for (const auto& hull : hulls_group.hulls) {
            auto shape = chrono_types::make_shared<ChCollisionShapeConvexHull>(cmaterials[hulls_group.matID], hull);
            body->AddCollisionShape(shape);
        }
    }
    for (auto& mesh : coll_meshes) {
        assert(cmaterials[mesh.matID]);
        // Hack: explicitly offset vertices
        for (auto& v : mesh.trimesh->m_vertices)
            v += mesh.pos;
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
        auto axis = cyl.rot.GetAxisY();
        auto p1 = cyl.pos - (cyl.length / 2) * axis;
        auto p2 = cyl.pos + (cyl.length / 2) * axis;
        auto e2 = ChVector3d(1.0) - axis * axis;
        ChVector3d e(std::sqrt(e2.x()), std::sqrt(e2.y()), std::sqrt(e2.z()));

        amin = Vmin(amin, p1 - cyl.radius * e);
        amax = Vmax(amax, p2 + cyl.radius * e);
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
