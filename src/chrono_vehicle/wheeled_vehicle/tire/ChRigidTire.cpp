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

#include <algorithm>

#include "chrono/physics/ChGlobal.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChContactContainerBase.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChRigidTire.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChRigidTire::ChRigidTire(const std::string& name)
    : ChTire(name),
      m_use_contact_mesh(false),
      m_trimesh(nullptr),
      m_friction(0.7f),
      m_restitution(0.1f),
      m_young_modulus(2e5f),
      m_poisson_ratio(0.3f),
      m_kn(2e5f),
      m_gn(40),
      m_kt(2e5f),
      m_gt(20) {}

ChRigidTire::~ChRigidTire() {
    delete m_trimesh;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRigidTire::SetMeshFilename(const std::string& mesh_file, double sweep_sphere_radius) {
    m_use_contact_mesh = true;
    m_contact_meshFile = mesh_file;
    m_sweep_sphere_radius = sweep_sphere_radius;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRigidTire::SetContactMaterialProperties(float young_modulus, float poisson_ratio) {
    m_young_modulus = young_modulus;
    m_poisson_ratio = poisson_ratio;
}

void ChRigidTire::SetContactMaterialCoefficients(float kn, float gn, float kt, float gt) {
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

    if (m_use_contact_mesh) {
        // Mesh contact
        m_trimesh = new geometry::ChTriangleMeshConnected;
        m_trimesh->LoadWavefrontMesh(m_contact_meshFile, true, false);

        wheel->GetCollisionModel()->ClearModel();
        wheel->GetCollisionModel()->AddTriangleMesh(*m_trimesh, false, false, ChVector<>(0), ChMatrix33<>(1),
                                                    m_sweep_sphere_radius);
        wheel->GetCollisionModel()->BuildModel();
    } else {
        // Cylinder contact
        wheel->GetCollisionModel()->ClearModel();
        wheel->GetCollisionModel()->AddCylinder(GetRadius(), GetRadius(), GetWidth() / 2);
        wheel->GetCollisionModel()->BuildModel();
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
void ChRigidTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    m_cyl_shape = std::make_shared<ChCylinderShape>();
    m_cyl_shape->GetCylinderGeometry().rad = GetRadius();
    m_cyl_shape->GetCylinderGeometry().p1 = ChVector<>(0, GetWidth() / 2, 0);
    m_cyl_shape->GetCylinderGeometry().p2 = ChVector<>(0, -GetWidth() / 2, 0);
    m_wheel->AddAsset(m_cyl_shape);

    m_texture = std::make_shared<ChTexture>();
    m_texture->SetTextureFilename(GetChronoDataFile("greenwhite.png"));
    m_wheel->AddAsset(m_texture);
}

void ChRigidTire::RemoveVisualizationAssets() {
    // Make sure we only remove the assets added by ChRigidTire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    {
        auto it = std::find(m_wheel->GetAssets().begin(), m_wheel->GetAssets().end(), m_cyl_shape);
        if (it != m_wheel->GetAssets().end())
            m_wheel->GetAssets().erase(it);
    }
    {
        auto it = std::find(m_wheel->GetAssets().begin(), m_wheel->GetAssets().end(), m_texture);
        if (it != m_wheel->GetAssets().end())
            m_wheel->GetAssets().erase(it);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

// Callback class to process contacts on a rigid tire.
// Accumulate contact forces and torques on the associated wheel body.
// Express them in the global frame, as applied to the wheel center.
class RigidTireContactReporter : public ChReportContactCallback {
  public:
    RigidTireContactReporter(std::shared_ptr<ChBody> body) : m_body(body) {}

    // Accumulated force, expressed in global frame, applied to wheel center.
    const ChVector<>& GetAccumulatedForce() const { return m_force; }

    // Accumulated torque, expressed in global frame.
    const ChVector<>& GetAccumulatedTorque() const { return m_torque; }

  private:
    virtual bool ReportContactCallback(const ChVector<>& pA,
                                       const ChVector<>& pB,
                                       const ChMatrix33<>& plane_coord,
                                       const double& distance,
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

TireForce ChRigidTire::GetTireForce(bool cosim) const {
    TireForce tire_force;

    // If the tire is simulated together with the associated vehicle, return zero
    // force and moment. In this case, the tire forces are automatically applied
    // to the associated wheel through Chrono's frictional contact system.
    if (!cosim) {
        tire_force.point = ChVector<>(0, 0, 0);
        tire_force.force = ChVector<>(0, 0, 0);
        tire_force.moment = ChVector<>(0, 0, 0);

        return tire_force;
    }

    // If the tire is co-simulated, calculate and return the resultant of the
    // contact forces acting on the tire.  The resulting tire force and moment
    // are expressed in global frame, as applied at the center of the associated
    // wheel body.
    RigidTireContactReporter reporter(m_wheel);
    
    m_wheel->GetSystem()->GetContactContainer()->ReportAllContacts(&reporter);

    tire_force.point = m_wheel->GetPos();
    tire_force.force = reporter.GetAccumulatedForce();
    tire_force.moment = reporter.GetAccumulatedTorque();

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
        pos.push_back(m_wheel->TransformPointLocalToParent(vertices[i]));
        vel.push_back(m_wheel->PointSpeedLocalToParent(vertices[i]));
    }
}

}  // end namespace vehicle
}  // end namespace chrono
