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
// Various utility classes for vehicle subsystems.
//
// =============================================================================

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChSubsysDefs.h"

#include "chrono/assets/ChAssetLevel.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {

ChVehicleGeometry::ChVehicleGeometry()
    : m_has_primitives(false), m_has_mesh(false), m_has_collision(false), m_has_colors(false) {}

void ChVehicleGeometry::AddVisualizationAssets(std::shared_ptr<ChBody> body, VisualizationType vis) {
    if (!body->GetVisualModel()) {
        auto model = chrono_types::make_shared<ChVisualModel>();
        body->AddVisualModel(model);
    }

    if (vis == VisualizationType::MESH && m_has_mesh) {
        auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(vehicle::GetDataFile(m_vis_mesh_file),
                                                                                  true, true);
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(m_vis_mesh_file).stem());
        trimesh_shape->SetMutable(false);
        body->AddAsset(trimesh_shape);
        body->GetVisualModel()->AddShape(trimesh_shape, ChFrame<>());
        return;
    }

    if (vis == VisualizationType::PRIMITIVES && m_has_primitives) {
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

        auto sphere_level = chrono_types::make_shared<ChAssetLevel>();
        for (auto& sphere : m_vis_spheres) {
            auto sphere_shape = chrono_types::make_shared<ChSphereShape>();
            sphere_shape->GetSphereGeometry().rad = sphere.m_radius;
            sphere_shape->Pos = sphere.m_pos;
                body->AddAsset(sphere_shape);
            sphere_shape->AddMaterial(sph_mat);
            body->GetVisualModel()->AddShape(sphere_shape, ChFrame<>(sphere.m_pos));

        }

        auto box_level = chrono_types::make_shared<ChAssetLevel>();
        for (auto& box : m_vis_boxes) {
            auto box_shape = chrono_types::make_shared<ChBoxShape>();
            box_shape->GetBoxGeometry().SetLengths(box.m_dims);
            box_shape->Pos = box.m_pos;
            box_shape->Rot = box.m_rot;
                body->AddAsset(box_shape);
            box_shape->AddMaterial(box_mat);
            body->GetVisualModel()->AddShape(box_shape, ChFrame<>(box.m_pos, box.m_rot));
        }

        auto cyl_level = chrono_types::make_shared<ChAssetLevel>();
        for (auto& cyl : m_vis_cylinders) {
            auto cyl_shape = chrono_types::make_shared<ChCylinderShape>();
            cyl_shape->GetCylinderGeometry().rad = cyl.m_radius;
            cyl_shape->GetCylinderGeometry().p1 = ChVector<>(0, cyl.m_length / 2, 0);
            cyl_shape->GetCylinderGeometry().p2 = ChVector<>(0, -cyl.m_length / 2, 0);
            cyl_shape->Pos = cyl.m_pos;
            cyl_shape->Rot = cyl.m_rot;
                body->AddAsset(cyl_shape);
            cyl_shape->AddMaterial(cyl_mat);
                body->GetVisualModel()->AddShape(cyl_shape, ChFrame<>(cyl.m_pos, cyl.m_rot));
        }

        return;
    }
}

void ChVehicleGeometry::AddCollisionShapes(std::shared_ptr<ChBody> body, int collision_family) {
    body->SetCollide(true);

    body->GetCollisionModel()->ClearModel();

    body->GetCollisionModel()->SetFamily(collision_family);

    for (auto& sphere : m_coll_spheres) {
        assert(m_materials[sphere.m_matID] &&
               m_materials[sphere.m_matID]->GetContactMethod() == body->GetSystem()->GetContactMethod());
        body->GetCollisionModel()->AddSphere(m_materials[sphere.m_matID], sphere.m_radius, sphere.m_pos);
    }
    for (auto& box : m_coll_boxes) {
        assert(m_materials[box.m_matID] &&
               m_materials[box.m_matID]->GetContactMethod() == body->GetSystem()->GetContactMethod());
        ChVector<> hdims = box.m_dims / 2;
        body->GetCollisionModel()->AddBox(m_materials[box.m_matID], hdims.x(), hdims.y(), hdims.z(), box.m_pos,
                                          box.m_rot);
    }
    for (auto& cyl : m_coll_cylinders) {
        assert(m_materials[cyl.m_matID] &&
               m_materials[cyl.m_matID]->GetContactMethod() == body->GetSystem()->GetContactMethod());
        body->GetCollisionModel()->AddCylinder(m_materials[cyl.m_matID], cyl.m_radius, cyl.m_radius, cyl.m_length / 2,
                                               cyl.m_pos, cyl.m_rot);
    }
    for (auto& hulls_group : m_coll_hulls) {
        assert(m_materials[hulls_group.m_matID] &&
               m_materials[hulls_group.m_matID]->GetContactMethod() == body->GetSystem()->GetContactMethod());
        geometry::ChTriangleMeshConnected mesh;
        std::vector<std::vector<ChVector<>>> hulls;
        utils::LoadConvexHulls(vehicle::GetDataFile(hulls_group.m_filename), mesh, hulls);
        for (int c = 0; c < hulls.size(); c++) {
            body->GetCollisionModel()->AddConvexHull(m_materials[hulls_group.m_matID], hulls[c]);
        }
    }
    for (auto& mesh : m_coll_meshes) {
        auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(mesh.m_filename, true, false);
        // Hack: explicitly offset vertices
        for (auto& v : trimesh->m_vertices)
            v += mesh.m_pos;
        body->GetCollisionModel()->AddTriangleMesh(m_materials[mesh.m_matID], trimesh, false, false, ChVector<>(0),
                                                   ChMatrix33<>(1), mesh.m_radius);
    }

    body->GetCollisionModel()->BuildModel();
}

}  // end namespace vehicle
}  // end namespace chrono
