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

#include "chrono_vehicle/terrain/SCMTerrain.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
ChRigidTire::ChRigidTire(const std::string& name) : ChTire(name), m_use_contact_mesh(false), m_trimesh(nullptr) {}

ChRigidTire::~ChRigidTire() {}

// -----------------------------------------------------------------------------
void ChRigidTire::SetMeshFilename(const std::string& mesh_file, double sweep_sphere_radius) {
    m_use_contact_mesh = true;
    m_contact_meshFile = mesh_file;
    m_sweep_sphere_radius = sweep_sphere_radius;
}

// -----------------------------------------------------------------------------
void ChRigidTire::Initialize(std::shared_ptr<ChWheel> wheel) {
    ChTire::Initialize(wheel);

    auto wheel_body = wheel->GetSpindle();

    CreateContactMaterial(wheel_body->GetSystem()->GetContactMethod());
    assert(m_material && m_material->GetContactMethod() == wheel_body->GetSystem()->GetContactMethod());

    wheel_body->EnableCollision(true);

    if (m_use_contact_mesh) {
        // Mesh contact
        m_trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(m_contact_meshFile, true, false);

        //// RADU
        // Hack to deal with current limitation: cannot set offset on a trimesh collision shape!
        double offset = GetOffset();
        if (std::abs(offset) > 1e-3) {
            for (int i = 0; i < m_trimesh->m_vertices.size(); i++)
                m_trimesh->m_vertices[i].y() += offset;
        }
        auto ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(m_material, m_trimesh, false, false,
                                                                                m_sweep_sphere_radius);
        wheel_body->AddCollisionShape(ct_shape);
    } else {
        // Cylinder contact
        auto ct_shape = chrono_types::make_shared<ChCollisionShapeCylinder>(m_material, GetRadius(), GetWidth());
        wheel_body->AddCollisionShape(ct_shape, ChFrame<>(ChVector3d(0, 0, GetOffset()), QuatFromAngleX(CH_PI_2)));
    }

    wheel_body->GetCollisionModel()->SetFamily(WheeledCollisionFamily::TIRE);
}

void ChRigidTire::Synchronize(double time, const ChTerrain& terrain) {
    WheelState wheel_state = m_wheel->GetState();

    // Calculate tire reference frame
    ChCoordsys<> tire_frame;
    double depth;
    float mu;
    ChTire::DiscTerrainCollision1pt(terrain, wheel_state.pos, wheel_state.rot.GetAxisY(), GetRadius(), tire_frame,
                                    depth, mu);

    // Calculate tire kinematics
    CalculateKinematics(wheel_state, tire_frame);
}

void ChRigidTire::InitializeInertiaProperties() {
    m_mass = GetTireMass();
    m_inertia.setZero();
    m_inertia.diagonal() = GetTireInertia().eigen();
    m_com = ChFrame<>();
}

void ChRigidTire::UpdateInertiaProperties() {
    auto spindle = m_wheel->GetSpindle();
    m_xform = ChFrame<>(spindle->TransformPointLocalToParent(ChVector3d(0, GetOffset(), 0)), spindle->GetRot());
}

double ChRigidTire::GetAddedMass() const {
    return GetTireMass();
}

ChVector3d ChRigidTire::GetAddedInertia() const {
    return GetTireInertia();
}

// -----------------------------------------------------------------------------
void ChRigidTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    m_cyl_shape = utils::ChBodyGeometry::AddVisualizationCylinder(m_wheel->GetSpindle(),                           //
                                                                  ChVector3d(0, GetOffset() + GetWidth() / 2, 0),  //
                                                                  ChVector3d(0, GetOffset() - GetWidth() / 2, 0),  //
                                                                  GetRadius());
    m_cyl_shape->SetTexture(GetChronoDataFile("textures/greenwhite.png"));
}

void ChRigidTire::RemoveVisualizationAssets() {
    // Make sure we only remove the assets added by ChRigidTire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets to the same body (the
    // spindle/wheel).
    ChPart::RemoveVisualizationAsset(m_wheel->GetSpindle(), m_cyl_shape);
}

// -----------------------------------------------------------------------------
// Callback class to process contacts on a rigid tire.
// Accumulate contact forces and torques on the associated wheel body.
// Express them in the global frame, as applied to the wheel center.
class RigidTireContactReporter : public ChContactContainer::ReportContactCallback {
  public:
    RigidTireContactReporter(std::shared_ptr<ChBody> body) : m_body(body) {}

    // Accumulated force, expressed in global frame, applied to wheel center.
    const ChVector3d& GetAccumulatedForce() const { return m_force; }

    // Accumulated torque, expressed in global frame.
    const ChVector3d& GetAccumulatedTorque() const { return m_torque; }

  private:
    virtual bool OnReportContact(const ChVector3d& pA,
                                 const ChVector3d& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const double& eff_radius,
                                 const ChVector3d& rforce,
                                 const ChVector3d& rtorque,
                                 ChContactable* modA,
                                 ChContactable* modB) override {
        // Filter contacts that involve the tire body.
        if (modA == m_body.get() || modB == m_body.get()) {
            // Express current contact force and torque in global frame
            ChVector3d force = plane_coord * rforce;
            ChVector3d torque = plane_coord * rtorque;
            // Wheel center in global frame
            const ChVector3d& center = m_body->GetPos();
            // Accumulate
            m_force += force;
            m_torque += torque + Vcross(Vsub(pA, center), force);
        }

        return true;
    }

    std::shared_ptr<ChBody> m_body;
    ChVector3d m_force;
    ChVector3d m_torque;
};

TerrainForce ChRigidTire::GetTireForce() const {
    // A ChRigidTire always returns zero force and moment since tire forces are automatically applied
    // to the associated wheel through Chrono's frictional contact system.
    TerrainForce tire_force;
    tire_force.point = m_wheel->GetPos();
    tire_force.force = ChVector3d(0, 0, 0);
    tire_force.moment = ChVector3d(0, 0, 0);

    return tire_force;
}

TerrainForce ChRigidTire::ReportTireForce(ChTerrain* terrain) const {
    // If interacting with an SCM terrain, interrogate the terrain system
    // for the cumulative force on the associated rigid body.
    if (auto scm = dynamic_cast<SCMTerrain*>(terrain)) {
        ChVector3d force;
        ChVector3d torque;
        scm->GetContactForceBody(m_wheel->GetSpindle(), force, torque);

        TerrainForce tire_force;
        tire_force.point = m_wheel->GetSpindle()->GetPos();
        tire_force.force = force;
        tire_force.moment = torque;

        return tire_force;
    }

    // Otherwise, calculate and return the resultant of the contact forces acting on the tire.
    // The resulting tire force and moment are expressed in global frame, as applied at the center
    // of the associated spindle body.

    TerrainForce tire_force;
    tire_force.point = m_wheel->GetSpindle()->GetPos();
    tire_force.force = m_wheel->GetSpindle()->GetContactForce();
    tire_force.moment = m_wheel->GetSpindle()->GetContactTorque();

    // Approach using the RigidTireContactReporter does not work in Chrono::Multicore
    // since contact forces and torques passed to OnReportContact are always zero.
    /*
    auto reporter = chrono_types::make_shared<RigidTireContactReporter>(m_wheel);
    m_wheel->GetSpindle()->GetSystem()->GetContactContainer()->ReportAllContacts(&reporter);
    TerrainForce tire_force;
    tire_force.point = m_wheel->GetSpindle()->GetPos();
    tire_force.force = reporter->GetAccumulatedForce();
    tire_force.moment = reporter->GetAccumulatedTorque();
    */

    return tire_force;
}

TerrainForce ChRigidTire::ReportTireForceLocal(ChTerrain* terrain, ChCoordsys<>& tire_frame) const {
    std::cerr << "ChRigidTire::ReportTireForceLocal not implemented." << std::endl;
    throw std::runtime_error("ChRigidTire::ReportTireForceLocal not implemented.");
}

// -----------------------------------------------------------------------------
std::shared_ptr<ChTriangleMeshConnected> ChRigidTire::GetContactMesh() const {
    assert(m_use_contact_mesh);
    return m_trimesh;
}

void ChRigidTire::GetMeshVertexStates(std::vector<ChVector3d>& pos, std::vector<ChVector3d>& vel) const {
    assert(m_use_contact_mesh);
    auto vertices = m_trimesh->GetCoordsVertices();

    for (size_t i = 0; i < vertices.size(); ++i) {
        pos.push_back(m_wheel->GetSpindle()->TransformPointLocalToParent(vertices[i]));
        vel.push_back(m_wheel->GetSpindle()->PointSpeedLocalToParent(vertices[i]));
    }
}

}  // end namespace vehicle
}  // end namespace chrono
