// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/chassis/ChRigidChassis.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChRigidChassis::ChRigidChassis(const std::string& name, bool fixed)
    : ChChassis(name, fixed), m_has_primitives(false), m_has_mesh(false) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRigidChassis::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    if (vis == VisualizationType::MESH && m_has_mesh) {
        geometry::ChTriangleMeshConnected trimesh;
        trimesh.LoadWavefrontMesh(vehicle::GetDataFile(m_vis_mesh_file), false, false);
        auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(m_vis_mesh_name);
        m_body->AddAsset(trimesh_shape);
        return;
    }

    if (vis == VisualizationType::PRIMITIVES && m_has_primitives) {
        for (auto sphere : m_vis_spheres) {
            auto sphere_shape = std::make_shared<ChSphereShape>();
            sphere_shape->GetSphereGeometry().rad = sphere.m_radius;
            sphere_shape->Pos = sphere.m_pos;
            m_body->AddAsset(sphere_shape);
        }

        for (auto box : m_vis_boxes) {
            auto box_shape = std::make_shared<ChBoxShape>();
            box_shape->GetBoxGeometry().SetLengths(box.m_dims);
            box_shape->Pos = box.m_pos;
            box_shape->Rot = box.m_rot;
            m_body->AddAsset(box_shape);
        }

        for (auto cyl : m_vis_cylinders) {
            auto cyl_shape = std::make_shared<ChCylinderShape>();
            ChVector<> p1 = cyl.m_pos + cyl.m_rot.Rotate(ChVector<>(cyl.m_length / 2, 0, 0));
            ChVector<> p2 = cyl.m_pos + cyl.m_rot.Rotate(ChVector<>(-cyl.m_length / 2, 0, 0));
            cyl_shape->GetCylinderGeometry().rad = cyl.m_radius;
            cyl_shape->GetCylinderGeometry().p1 = p1;
            cyl_shape->GetCylinderGeometry().p2 = p2;
            m_body->AddAsset(cyl_shape);
        }

        return;
    }

    auto sphere_shape = std::make_shared<ChSphereShape>();
    sphere_shape->GetSphereGeometry().rad = 0.1;
    sphere_shape->Pos = GetLocalPosCOM();
    m_body->AddAsset(sphere_shape);
}

void ChRigidChassis::RemoveVisualizationAssets() {
    m_body->GetAssets().clear();
}

}  // end namespace vehicle
}  // end namespace chrono
