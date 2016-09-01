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
// Template for a rigid tire
//
// =============================================================================

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/physics/ChGlobal.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChRigidTire.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChRigidTire::ChRigidTire(const std::string& name)
    : ChTire(name),
      m_use_mesh(false),
      m_friction(0.6f),
      m_restitution(0.1f),
      m_young_modulus(2e5f),
      m_poisson_ratio(0.3f),
      m_kn(2e5f),
      m_gn(40),
      m_kt(2e5f),
      m_gt(20) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRigidTire::SetMeshFilename(const std::string& mesh_file, double sweep_sphere_radius) {
    m_use_mesh = true;
    m_mesh_file = mesh_file;
    m_sweep_sphere_radius = sweep_sphere_radius;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRigidTire::SetContactMaterial(float friction_coefficient,
                                     float restitution_coefficient,
                                     float young_modulus,
                                     float poisson_ratio,
                                     float kn,
                                     float gn,
                                     float kt,
                                     float gt) {
    m_friction = friction_coefficient;
    m_restitution = restitution_coefficient;
    m_young_modulus = young_modulus;
    m_poisson_ratio = poisson_ratio;
    m_kn = kn;
    m_gn = gn;
    m_kt = kt;
    m_gt = gt;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRigidTire::Initialize(std::shared_ptr<ChBody> wheel, VehicleSide side) {
    ChTire::Initialize(wheel, side);

    wheel->SetCollide(true);

    if (m_use_mesh) {
        // Mesh contact & visualization shape
        geometry::ChTriangleMeshConnected trimesh;
        trimesh.LoadWavefrontMesh(m_mesh_file, true, false);

        wheel->GetCollisionModel()->ClearModel();
        wheel->GetCollisionModel()->AddTriangleMesh(trimesh, false, false, ChVector<>(0), ChMatrix33<>(1),
                                                    m_sweep_sphere_radius);
        wheel->GetCollisionModel()->BuildModel();

        if (m_vis_enabled) {
            auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
            trimesh_shape->SetMesh(trimesh);
            trimesh_shape->SetName(m_name);
            wheel->AddAsset(trimesh_shape);
        }
    } else {
        // Cylinder contact & visualization shape
        wheel->GetCollisionModel()->ClearModel();
        wheel->GetCollisionModel()->AddCylinder(GetRadius(), GetRadius(), GetWidth() / 2);
        wheel->GetCollisionModel()->BuildModel();

        if (m_vis_enabled) {
            auto cyl = std::make_shared<ChCylinderShape>();
            cyl->GetCylinderGeometry().rad = GetRadius();
            cyl->GetCylinderGeometry().p1 = ChVector<>(0, GetWidth() / 2, 0);
            cyl->GetCylinderGeometry().p2 = ChVector<>(0, -GetWidth() / 2, 0);
            wheel->AddAsset(cyl);

            auto tex = std::make_shared<ChTexture>();
            tex->SetTextureFilename(GetChronoDataFile("greenwhite.png"));
            wheel->AddAsset(tex);
        }
    }

    switch (wheel->GetContactMethod()) {
        case ChMaterialSurfaceBase::DVI:
            wheel->GetMaterialSurface()->SetFriction(m_friction);
            wheel->GetMaterialSurface()->SetRestitution(m_restitution);
            break;
        case ChMaterialSurfaceBase::DEM:
            wheel->GetMaterialSurfaceDEM()->SetFriction(m_friction);
            wheel->GetMaterialSurfaceDEM()->SetRestitution(m_restitution);
            wheel->GetMaterialSurfaceDEM()->SetYoungModulus(m_young_modulus);
            wheel->GetMaterialSurfaceDEM()->SetPoissonRatio(m_poisson_ratio);
            wheel->GetMaterialSurfaceDEM()->SetKn(m_kn);
            wheel->GetMaterialSurfaceDEM()->SetGn(m_gn);
            wheel->GetMaterialSurfaceDEM()->SetKt(m_kt);
            wheel->GetMaterialSurfaceDEM()->SetGt(m_gt);
            break;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
TireForce ChRigidTire::GetTireForce(bool cosim) const {
    TireForce tire_force;

    // If the tire is simulated together with the associated vehicle, return zero
    // force and moment. In this case, the tire forces are automatically applied
    // to the associated wheel through Chrono's frictional contact system.
    if (!cosim) {
      tire_force.force = ChVector<>(0, 0, 0);
      tire_force.point = ChVector<>(0, 0, 0);
      tire_force.moment = ChVector<>(0, 0, 0);

      return tire_force;
    }

    // If the tire is co-simulated, calculate and return the resultant of the
    // contact forces acting on the tire.

    //// TODO
    tire_force.force = ChVector<>(0, 0, 0);
    tire_force.point = ChVector<>(0, 0, 0);
    tire_force.moment = ChVector<>(0, 0, 0);

    return tire_force;
}

}  // end namespace vehicle
}  // end namespace chrono
