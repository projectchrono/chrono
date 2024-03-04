// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
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

#include <limits>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChVehicleGeometry.h"

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_thirdparty/filesystem/path.h"


namespace chrono {
namespace vehicle {

ChVehicleGeometry::ChVehicleGeometry()
    : m_has_primitives(false), m_has_obj(false), m_has_mesh(false), m_has_collision(false), m_has_colors(false) {}

ChVehicleGeometry::BoxShape::BoxShape(const ChVector<>& pos,
                                      const ChQuaternion<>& rot,
                                      const ChVector<>& dims,
                                      int matID)
    : m_pos(pos), m_rot(rot), m_dims(dims), m_matID(matID) {}

ChVehicleGeometry::SphereShape::SphereShape(const ChVector<>& pos, double radius, int matID)
    : m_pos(pos), m_radius(radius), m_matID(matID) {}

ChVehicleGeometry::CylinderShape::CylinderShape(const ChVector<>& pos,
                                                const ChQuaternion<>& rot,
                                                double radius,
                                                double length,
                                                int matID)
    : m_pos(pos), m_rot(rot), m_radius(radius), m_length(length), m_matID(matID) {}

ChVehicleGeometry::CylinderShape::CylinderShape(const ChVector<>& pos,
                                                const ChVector<>& axis,
                                                double radius,
                                                double length,
                                                int matID)
    : m_pos(pos), m_radius(radius), m_length(length), m_matID(matID) {
    ChMatrix33<> rot;
    rot.Set_A_Xdir(axis);
    m_rot = rot.Get_A_quaternion() * Q_from_AngY(-CH_C_PI_2);
}

ChVehicleGeometry::LineShape::LineShape(const ChVector<>& pos,
                                        const ChQuaternion<>& rot,
                                        std::shared_ptr<geometry::ChLine> line)
    : m_pos(pos), m_rot(rot), m_line(line) {}

ChVehicleGeometry::ConvexHullsShape::ConvexHullsShape(const std::string& filename, int matID) : m_matID(matID) {
    geometry::ChTriangleMeshConnected mesh;
    utils::LoadConvexHulls(vehicle::GetDataFile(filename), mesh, m_hulls);
}

ChVehicleGeometry::TrimeshShape::TrimeshShape(const ChVector<>& pos,
                                              const std::string& filename,
                                              double radius,
                                              int matID)
    : m_radius(radius), m_pos(pos), m_matID(matID) {
    m_trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(vehicle::GetDataFile(filename), true, false);
}

ChVehicleGeometry::TrimeshShape::TrimeshShape(const ChVector<>& pos,
                                              std::shared_ptr<geometry::ChTriangleMeshConnected> trimesh,
                                              double radius,
                                              int matID)
    : m_trimesh(trimesh), m_radius(radius), m_pos(pos), m_matID(matID) {}

std::shared_ptr<ChVisualShape> ChVehicleGeometry::AddVisualizationCylinder(std::shared_ptr<ChBody> body,
                                                                           const ChVector<>& p1,
                                                                           const ChVector<>& p2,
                                                                           double radius,
                                                                           ChVisualMaterialSharedPtr mat) {
    geometry::ChLineSegment seg(p1, p2);
    auto cyl = chrono_types::make_shared<ChVisualShapeCylinder>(radius, seg.GetLength());
    if (mat)
        cyl->AddMaterial(mat);
    body->AddVisualShape(cyl, seg.GetFrame());
    return cyl;
}

void ChVehicleGeometry::CreateVisualizationAssets(std::shared_ptr<ChBody> body,
                                                  VisualizationType vis,
                                                  bool visualize_collision) {
    if (vis == VisualizationType::NONE)
        return;

    if (!body->GetVisualModel()) {
        auto model = chrono_types::make_shared<ChVisualModel>();
        body->AddVisualModel(model);
    }

    if (visualize_collision) {
        for (auto& sphere : m_coll_spheres) {
            auto sphere_shape = chrono_types::make_shared<ChVisualShapeSphere>(sphere.m_radius);
            body->AddVisualShape(sphere_shape, ChFrame<>(sphere.m_pos));
        }

        for (auto& box : m_coll_boxes) {
            auto box_shape = chrono_types::make_shared<ChVisualShapeBox>(box.m_dims);
            body->AddVisualShape(box_shape, ChFrame<>(box.m_pos, box.m_rot));
        }

        for (auto& cyl : m_coll_cylinders) {
            auto cyl_shape = chrono_types::make_shared<ChVisualShapeCylinder>(cyl.m_radius, cyl.m_length);
            body->AddVisualShape(cyl_shape, ChFrame<>(cyl.m_pos, cyl.m_rot));
        }

        for (auto& mesh : m_coll_meshes) {
            auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            trimesh_shape->SetMesh(mesh.m_trimesh);
            trimesh_shape->SetMutable(false);
            body->AddVisualShape(trimesh_shape, ChFrame<>());
        }

        return;
    }

    if (vis == VisualizationType::MESH && m_has_obj) {
        auto obj_shape = chrono_types::make_shared<ChVisualShapeModelFile>();
        obj_shape->SetFilename(vehicle::GetDataFile(m_vis_mesh_file));
        body->AddVisualShape(obj_shape, ChFrame<>());
        return;
    }

    if (vis == VisualizationType::MESH && m_has_mesh) {
        auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(vehicle::GetDataFile(m_vis_mesh_file),
                                                                                  true, true);
        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(m_vis_mesh_file).stem());
        trimesh_shape->SetMutable(false);
        body->AddVisualShape(trimesh_shape, ChFrame<>());
        return;
    }

    // If no mesh specified, default to primitives
    if (m_has_primitives) {
        if (!m_has_colors) {
            m_color_boxes = ChColor(0.5f, 0.5f, 0.5f);
            m_color_spheres = ChColor(0.5f, 0.5f, 0.5f);
            m_color_cylinders = ChColor(0.5f, 0.5f, 0.5f);
        }

        auto box_mat = chrono_types::make_shared<ChVisualMaterial>();
        auto sph_mat = chrono_types::make_shared<ChVisualMaterial>();
        auto cyl_mat = chrono_types::make_shared<ChVisualMaterial>();

        box_mat->SetDiffuseColor({m_color_boxes.R, m_color_boxes.G, m_color_boxes.B});
        sph_mat->SetDiffuseColor({m_color_spheres.R, m_color_spheres.G, m_color_spheres.B});
        cyl_mat->SetDiffuseColor({m_color_cylinders.R, m_color_cylinders.G, m_color_cylinders.B});

        for (auto& sphere : m_vis_spheres) {
            auto sphere_shape = chrono_types::make_shared<ChVisualShapeSphere>(sphere.m_radius);
            sphere_shape->AddMaterial(sph_mat);
            body->AddVisualShape(sphere_shape, ChFrame<>(sphere.m_pos));
        }

        for (auto& box : m_vis_boxes) {
            auto box_shape = chrono_types::make_shared<ChVisualShapeBox>(box.m_dims);
            box_shape->AddMaterial(box_mat);
            body->AddVisualShape(box_shape, ChFrame<>(box.m_pos, box.m_rot));
        }

        for (auto& cyl : m_vis_cylinders) {
            auto cyl_shape = chrono_types::make_shared<ChVisualShapeCylinder>(cyl.m_radius, cyl.m_length);
            cyl_shape->AddMaterial(cyl_mat);
            body->AddVisualShape(cyl_shape, ChFrame<>(cyl.m_pos, cyl.m_rot));
        }

        for (auto& line : m_vis_lines) {
            auto line_shape = chrono_types::make_shared<ChVisualShapeLine>();
            line_shape->SetLineGeometry(line.m_line);
            body->AddVisualShape(line_shape, ChFrame<>(line.m_pos, line.m_rot));
        }

        return;
    }
}

void ChVehicleGeometry::CreateCollisionShapes(std::shared_ptr<ChBody> body,
                                              int collision_family,
                                              ChContactMethod contact_method) {
    std::vector<std::shared_ptr<ChMaterialSurface>> materials;
    for (const auto& minfo : m_materials) {
        materials.push_back(minfo.CreateMaterial(contact_method));
    }

    body->SetCollide(true);

    for (auto& sphere : m_coll_spheres) {
        assert(materials[sphere.m_matID]);
        auto shape = chrono_types::make_shared<ChCollisionShapeSphere>(materials[sphere.m_matID], sphere.m_radius);
        body->AddCollisionShape(shape, ChFrame<>(sphere.m_pos, QUNIT));
    }
    for (auto& box : m_coll_boxes) {
        assert(materials[box.m_matID]);
        auto shape = chrono_types::make_shared<ChCollisionShapeBox>(materials[box.m_matID], box.m_dims.x(),
                                                                    box.m_dims.y(), box.m_dims.z());
        body->AddCollisionShape(shape, ChFrame<>(box.m_pos, box.m_rot));
    }
    for (auto& cyl : m_coll_cylinders) {
        assert(materials[cyl.m_matID]);
        auto shape =
            chrono_types::make_shared<ChCollisionShapeCylinder>(materials[cyl.m_matID], cyl.m_radius, cyl.m_length);
        body->AddCollisionShape(shape, ChFrame<>(cyl.m_pos, cyl.m_rot));
    }
    for (auto& hulls_group : m_coll_hulls) {
        assert(materials[hulls_group.m_matID]);
        for (const auto& hull : hulls_group.m_hulls) {
            auto shape = chrono_types::make_shared<ChCollisionShapeConvexHull>(materials[hulls_group.m_matID], hull);
            body->AddCollisionShape(shape);
        }
    }
    for (auto& mesh : m_coll_meshes) {
        assert(materials[mesh.m_matID]);
        // Hack: explicitly offset vertices
        for (auto& v : mesh.m_trimesh->m_vertices)
            v += mesh.m_pos;
        auto shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(materials[mesh.m_matID], mesh.m_trimesh,
                                                                             false, false, mesh.m_radius);
        body->AddCollisionShape(shape);
    }

    body->GetCollisionModel()->SetFamily(collision_family);
}

geometry::ChAABB ChVehicleGeometry::CalculateAABB() {
    ChVector<> amin(+std::numeric_limits<double>::max());
    ChVector<> amax(-std::numeric_limits<double>::max());

    for (const auto& box : m_coll_boxes) {
        ChMatrix33<> A(box.m_rot);
        A.cwiseAbs();
        auto tmp = A * (box.m_dims / 2);

        amin = Vmin(amin, box.m_pos - tmp);
        amax = Vmax(amax, box.m_pos + tmp);
    }

    for (const auto& sph : m_coll_spheres) {
        amin = Vmin(amin, sph.m_pos - sph.m_radius);
        amax = Vmin(amax, sph.m_pos + sph.m_radius);
    }

    for (const auto& cyl : m_coll_cylinders) {
        auto axis = cyl.m_rot.GetYaxis();
        auto p1 = cyl.m_pos - (cyl.m_length / 2) * axis;
        auto p2 = cyl.m_pos + (cyl.m_length / 2) * axis;
        auto e2 = ChVector<>(1.0) - axis * axis;
        ChVector<> e(std::sqrt(e2.x()), std::sqrt(e2.y()), std::sqrt(e2.z()));

        amin = Vmin(amin, p1 - cyl.m_radius * e);
        amax = Vmax(amax, p2 + cyl.m_radius * e);
    }

    for (const auto& hulls_group : m_coll_hulls) {
        for (const auto& hull : hulls_group.m_hulls) {
            for (const auto& v : hull) {
                amin = Vmin(amin, v);
                amax = Vmax(amax, v);
            }
        }
    }

    for (const auto& mesh : m_coll_meshes) {
        auto bbox = mesh.m_trimesh->GetBoundingBox();
        amin = Vmin(amin, bbox.min);
        amax = Vmax(amax, bbox.max);
    }

    return geometry::ChAABB(amin, amax);
}

// -----------------------------------------------------------------------------

ChTSDAGeometry::ChTSDAGeometry() : m_has_color(false), m_vis_segment(nullptr), m_vis_spring(nullptr) {}

ChTSDAGeometry::SpringShape::SpringShape(double radius, int resolution, double turns)
    : m_radius(radius), m_turns(turns), m_resolution(resolution) {}

void ChTSDAGeometry::CreateVisualizationAssets(std::shared_ptr<ChLinkTSDA> tsda, VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    if (!tsda->GetVisualModel()) {
        auto model = chrono_types::make_shared<ChVisualModel>();
        tsda->AddVisualModel(model);
    }

    if (!m_has_color) {
        m_color = ChColor(1.0f, 1.0f, 1.0f);
    }
    auto mat = chrono_types::make_shared<ChVisualMaterial>();
    mat->SetDiffuseColor(m_color);

    if (m_vis_spring) {
        auto spring_shape = chrono_types::make_shared<ChVisualShapeSpring>(
            m_vis_spring->m_radius, m_vis_spring->m_resolution, m_vis_spring->m_turns);
        spring_shape->AddMaterial(mat);
        tsda->AddVisualShape(spring_shape);
    }

    if (m_vis_segment) {
        auto segment_shape = chrono_types::make_shared<ChVisualShapeSegment>();
        segment_shape->AddMaterial(mat);
        tsda->AddVisualShape(segment_shape);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
