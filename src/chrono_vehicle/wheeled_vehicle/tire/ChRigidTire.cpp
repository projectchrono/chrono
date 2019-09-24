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
// Template for a rigid tire
//
// =============================================================================

#include <algorithm>

#include "chrono/core/ChGlobal.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChContactContainer.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChRigidTire.h"

#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChRigidTire::ChRigidTire(const std::string& name) : ChTire(name), m_use_contact_mesh(false), m_trimesh(nullptr) {}

ChRigidTire::~ChRigidTire() {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRigidTire::SetMeshFilename(const std::string& mesh_file, double sweep_sphere_radius) {
    m_use_contact_mesh = true;
    m_contact_meshFile = mesh_file;
    m_sweep_sphere_radius = sweep_sphere_radius;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRigidTire::Initialize(std::shared_ptr<ChWheel> wheel) {
    ChTire::Initialize(wheel);

    auto wheel_body = wheel->GetSpindle();

    wheel_body->SetCollide(true);

    wheel_body->GetCollisionModel()->ClearModel();

    wheel_body->GetCollisionModel()->SetFamily(WheeledCollisionFamily::TIRES);

    if (m_use_contact_mesh) {
        // Mesh contact
        m_trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        m_trimesh->LoadWavefrontMesh(m_contact_meshFile, true, false);

        //// RADU
        // Hack to deal with current limitation: cannot set offset on a trimesh collision shape!
        double offset = GetOffset();
        if (std::abs(offset) > 1e-3) {
            for (int i = 0; i < m_trimesh->m_vertices.size(); i++)
                m_trimesh->m_vertices[i].y() += offset;
        }

        wheel_body->GetCollisionModel()->AddTriangleMesh(m_trimesh, false, false, ChVector<>(0), ChMatrix33<>(1),
                                                         m_sweep_sphere_radius);
    } else {
        // Cylinder contact
        wheel_body->GetCollisionModel()->AddCylinder(GetRadius(), GetRadius(), GetWidth() / 2,
                                                     ChVector<>(0, GetOffset(), 0));
    }

    wheel_body->GetCollisionModel()->BuildModel();

    switch (wheel_body->GetContactMethod()) {
        case ChMaterialSurface::NSC:
            wheel_body->GetMaterialSurfaceNSC()->SetFriction(m_friction);
            wheel_body->GetMaterialSurfaceNSC()->SetRestitution(m_restitution);
            break;
        case ChMaterialSurface::SMC:
            wheel_body->GetMaterialSurfaceSMC()->SetFriction(m_friction);
            wheel_body->GetMaterialSurfaceSMC()->SetRestitution(m_restitution);
            wheel_body->GetMaterialSurfaceSMC()->SetYoungModulus(m_young_modulus);
            wheel_body->GetMaterialSurfaceSMC()->SetPoissonRatio(m_poisson_ratio);
            wheel_body->GetMaterialSurfaceSMC()->SetKn(m_kn);
            wheel_body->GetMaterialSurfaceSMC()->SetGn(m_gn);
            wheel_body->GetMaterialSurfaceSMC()->SetKt(m_kt);
            wheel_body->GetMaterialSurfaceSMC()->SetGt(m_gt);
            break;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRigidTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    m_cyl_shape = chrono_types::make_shared<ChCylinderShape>();
    m_cyl_shape->GetCylinderGeometry().rad = GetRadius();
    m_cyl_shape->GetCylinderGeometry().p1 = ChVector<>(0, GetOffset() + GetWidth() / 2, 0);
    m_cyl_shape->GetCylinderGeometry().p2 = ChVector<>(0, GetOffset() - GetWidth() / 2, 0);
    m_wheel->GetSpindle()->AddAsset(m_cyl_shape);

    m_texture = chrono_types::make_shared<ChTexture>();
    m_texture->SetTextureFilename(GetChronoDataFile("greenwhite.png"));
    m_wheel->GetSpindle()->AddAsset(m_texture);
}

void ChRigidTire::RemoveVisualizationAssets() {
    // Make sure we only remove the assets added by ChRigidTire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    auto& assets = m_wheel->GetSpindle()->GetAssets();
    {
        auto it = std::find(assets.begin(), assets.end(), m_cyl_shape);
        if (it != assets.end())
            assets.erase(it);
    }
    {
        auto it = std::find(assets.begin(), assets.end(), m_texture);
        if (it != assets.end())
            assets.erase(it);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

// Callback class to process contacts on a rigid tire.
// Accumulate contact forces and torques on the associated wheel body.
// Express them in the global frame, as applied to the wheel center.
class RigidTireContactReporter : public ChContactContainer::ReportContactCallback {
  public:
    RigidTireContactReporter(std::shared_ptr<ChBody> body) : m_body(body) {}

    // Accumulated force, expressed in global frame, applied to wheel center.
    const ChVector<>& GetAccumulatedForce() const { return m_force; }

    // Accumulated torque, expressed in global frame.
    const ChVector<>& GetAccumulatedTorque() const { return m_torque; }

  private:
    virtual bool OnReportContact(const ChVector<>& pA,
                                 const ChVector<>& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const double& eff_radius,
                                 const ChVector<>& rforce,
                                 const ChVector<>& rtorque,
                                 ChContactable* modA,
                                 ChContactable* modB) override {
        // Filter contacts that involve the tire body.
        if (modA == m_body.get() || modB == m_body.get()) {
            // Express current contact force and torque in global frame
            ChVector<> force = plane_coord * rforce;
            ChVector<> torque = plane_coord * rtorque;
            // Wheel center in global frame
            const ChVector<>& center = m_body->GetPos();
            // Accumulate
            m_force += force;
            m_torque += torque + Vcross(Vsub(pA, center), force);
        }

        return true;
    }

    std::shared_ptr<ChBody> m_body;
    ChVector<> m_force;
    ChVector<> m_torque;
};

TerrainForce ChRigidTire::GetTireForce() const {
    // A ChRigidTire always returns zero force and moment since tire forces are automatically applied
    // to the associated wheel through Chrono's frictional contact system.
    TerrainForce tire_force;
    tire_force.point = m_wheel->GetPos();
    tire_force.force = ChVector<>(0, 0, 0);
    tire_force.moment = ChVector<>(0, 0, 0);

    return tire_force;
}

TerrainForce ChRigidTire::ReportTireForce(ChTerrain* terrain) const {
    // If interacting with an SCM terrain, interrogate the terrain system
    // for the cumulative force on the associated rigid body.
    if (auto scm = dynamic_cast<SCMDeformableTerrain*>(terrain)) {
        return scm->GetContactForce(m_wheel->GetSpindle());
    }

    // Otherwise, calculate and return the resultant of the contact forces acting on the tire.
    // The resulting tire force and moment are expressed in global frame, as applied at the center
    // of the associated spindle body.

    TerrainForce tire_force;
    tire_force.point = m_wheel->GetSpindle()->GetPos();
    tire_force.force = m_wheel->GetSpindle()->GetContactForce();
    tire_force.moment = m_wheel->GetSpindle()->GetContactTorque();

    // Approach using the RigidTireContactReporter does not work in Chrono::Parallel
    // since contact forces and torques passed to OnReportContact are always zero.
    /*
    RigidTireContactReporter reporter(m_wheel);
    m_wheel->GetSpindle()->GetSystem()->GetContactContainer()->ReportAllContacts(&reporter);
    TerrainForce tire_force;
    tire_force.point = m_wheel->GetSpindle()->GetPos();
    tire_force.force = reporter.GetAccumulatedForce();
    tire_force.moment = reporter.GetAccumulatedTorque();
    */

    return tire_force;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
unsigned int ChRigidTire::GetNumVertices() const {
    assert(m_use_contact_mesh);
    return static_cast<unsigned int>(m_trimesh->getCoordsVertices().size());
}

unsigned int ChRigidTire::GetNumTriangles() const {
    assert(m_use_contact_mesh);
    return static_cast<unsigned int>(m_trimesh->getIndicesVertexes().size());
}

const std::vector<ChVector<int>>& ChRigidTire::GetMeshConnectivity() const {
    assert(m_use_contact_mesh);
    return m_trimesh->getIndicesVertexes();
}

const std::vector<ChVector<>>& ChRigidTire::GetMeshVertices() const {
    assert(m_use_contact_mesh);
    return m_trimesh->getCoordsVertices();
}

const std::vector<ChVector<>>& ChRigidTire::GetMeshNormals() const {
    assert(m_use_contact_mesh);
    return m_trimesh->getCoordsNormals();
}

void ChRigidTire::GetMeshVertexStates(std::vector<ChVector<>>& pos, std::vector<ChVector<>>& vel) const {
    assert(m_use_contact_mesh);
    auto vertices = m_trimesh->getCoordsVertices();

    for (size_t i = 0; i < vertices.size(); ++i) {
        pos.push_back(m_wheel->GetSpindle()->TransformPointLocalToParent(vertices[i]));
        vel.push_back(m_wheel->GetSpindle()->PointSpeedLocalToParent(vertices[i]));
    }
}

}  // end namespace vehicle
}  // end namespace chrono
