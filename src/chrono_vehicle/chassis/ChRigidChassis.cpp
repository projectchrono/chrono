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
// Template for a rigid-body chassis vehicle subsystem.
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCylinderShape.h"

#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/chassis/ChRigidChassis.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChRigidChassis::ChRigidChassis(const std::string& name, bool fixed)
    : ChChassis(name, fixed), m_has_primitives(false), m_has_mesh(false), m_has_collision(false) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRigidChassis::Initialize(ChSystem* system,
                                const ChCoordsys<>& chassisPos,
                                double chassisFwdVel,
                                int collision_family) {
    // Invoke the base class method to construct the frame body.
    ChChassis::Initialize(system, chassisPos, chassisFwdVel);

    // If collision shapes were defined, create the contact geometry and enable contact
    // for the chassis's rigid body.
    // NOTE: setting the collision family is deferred to the containing vehicle system
    // (which can also disable contact between the chassis and certain vehicle subsystems).
    if (m_has_collision) {
        auto contact_method = system->GetContactMethod();
        CreateContactMaterials(contact_method);

        m_body->SetCollide(true);

        m_body->GetCollisionModel()->ClearModel();

        m_body->GetCollisionModel()->SetFamily(collision_family);

        for (auto sphere : m_coll_spheres) {
            assert(m_materials[sphere.m_matID] && m_materials[sphere.m_matID]->GetContactMethod() == contact_method);
            m_body->GetCollisionModel()->AddSphere(m_materials[sphere.m_matID], sphere.m_radius, sphere.m_pos);
        }
        for (auto box : m_coll_boxes) {
            assert(m_materials[box.m_matID] && m_materials[box.m_matID]->GetContactMethod() == contact_method);
            ChVector<> hdims = box.m_dims / 2;
            m_body->GetCollisionModel()->AddBox(m_materials[box.m_matID], hdims.x(), hdims.y(), hdims.z(), box.m_pos,
                                                box.m_rot);
        }
        for (auto cyl : m_coll_cylinders) {
            assert(m_materials[cyl.m_matID] && m_materials[cyl.m_matID]->GetContactMethod() == contact_method);
            m_body->GetCollisionModel()->AddCylinder(m_materials[cyl.m_matID], cyl.m_radius, cyl.m_radius,
                                                     cyl.m_length / 2,
                                                     cyl.m_pos, cyl.m_rot);
        }
        for (auto hulls_group : m_coll_hulls) {
            assert(m_materials[hulls_group.m_matID] &&
                   m_materials[hulls_group.m_matID]->GetContactMethod() == contact_method);
            geometry::ChTriangleMeshConnected mesh;
            std::vector<std::vector<ChVector<>>> hulls;
            utils::LoadConvexHulls(vehicle::GetDataFile(hulls_group.m_filename), mesh, hulls);
            for (int c = 0; c < hulls.size(); c++) {
                m_body->GetCollisionModel()->AddConvexHull(m_materials[hulls_group.m_matID], hulls[c]);
            }
        }
        m_body->GetCollisionModel()->BuildModel();
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRigidChassis::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    if (vis == VisualizationType::MESH && m_has_mesh) {
        auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        trimesh->LoadWavefrontMesh(vehicle::GetDataFile(m_vis_mesh_file), false, false);
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(filesystem::path(m_vis_mesh_file).stem());
        trimesh_shape->SetStatic(true);
        m_body->AddAsset(trimesh_shape);
        return;
    }

    if (vis == VisualizationType::PRIMITIVES && m_has_primitives) {
        for (auto sphere : m_vis_spheres) {
            auto sphere_shape = chrono_types::make_shared<ChSphereShape>();
            sphere_shape->GetSphereGeometry().rad = sphere.m_radius;
            sphere_shape->Pos = sphere.m_pos;
            m_body->AddAsset(sphere_shape);
        }

        for (auto box : m_vis_boxes) {
            auto box_shape = chrono_types::make_shared<ChBoxShape>();
            box_shape->GetBoxGeometry().SetLengths(box.m_dims);
            box_shape->Pos = box.m_pos;
            box_shape->Rot = box.m_rot;
            m_body->AddAsset(box_shape);
        }

        for (auto cyl : m_vis_cylinders) {
            auto cyl_shape = chrono_types::make_shared<ChCylinderShape>();
            cyl_shape->GetCylinderGeometry().rad = cyl.m_radius;
            cyl_shape->GetCylinderGeometry().p1 = ChVector<>(0, cyl.m_length / 2, 0);
            cyl_shape->GetCylinderGeometry().p2 = ChVector<>(0, -cyl.m_length / 2, 0);
            cyl_shape->Pos = cyl.m_pos;
            cyl_shape->Rot = cyl.m_rot;
            m_body->AddAsset(cyl_shape);
        }

        return;
    }

    auto sphere_shape = chrono_types::make_shared<ChSphereShape>();
    sphere_shape->GetSphereGeometry().rad = 0.1;
    sphere_shape->Pos = GetLocalPosCOM();
    m_body->AddAsset(sphere_shape);
}

void ChRigidChassis::RemoveVisualizationAssets() {
    m_body->GetAssets().clear();
}

void ChRigidChassis::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_body);
    ChPart::ExportBodyList(jsonDocument, bodies);

    ChPart::ExportMarkerList(jsonDocument, m_markers);
}

void ChRigidChassis::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBodyAuxRef>> bodies;
    bodies.push_back(m_body);
    database.WriteAuxRefBodies(bodies);

    database.WriteMarkers(m_markers);
}

}  // end namespace vehicle
}  // end namespace chrono
