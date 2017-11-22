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
// Base class for a deformable tire (i.e. modeled with an FEA mesh)
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"

namespace chrono {
namespace vehicle {

using namespace chrono::fea;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChDeformableTire::ChDeformableTire(const std::string& name)
    : ChTire(name),
      m_pressure_enabled(true),
      m_contact_enabled(true),
      m_connection_enabled(true),
      m_contact_type(NODE_CLOUD),
      m_contact_node_radius(0.001),
      m_contact_face_thickness(0.0),
      m_young_modulus(2e5f),
      m_poisson_ratio(0.3f),
      m_friction(0.6f),
      m_restitution(0.1f),
      m_kn(2e5),
      m_kt(2e5),
      m_gn(40),
      m_gt(20),
      m_pressure(-1) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDeformableTire::SetContactMaterialProperties(float young_modulus,
                                                    float poisson_ratio) {
    m_young_modulus = young_modulus;
    m_poisson_ratio = poisson_ratio;
}

void ChDeformableTire::SetContactMaterialCoefficients(float kn, float gn, float kt, float gt) {
    m_kn = kn;
    m_gn = gn;
    m_kt = kt;
    m_gt = gt;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDeformableTire::Initialize(std::shared_ptr<ChBody> wheel, VehicleSide side) {
    ChTire::Initialize(wheel, side);

    ChSystemSMC* system = dynamic_cast<ChSystemSMC*>(wheel->GetSystem());
    assert(system);

    // Create the tire mesh
    m_mesh = std::make_shared<ChMesh>();
    system->Add(m_mesh);

    // Create the FEA nodes and elements
    CreateMesh(*(wheel.get()), side);

    // Create a load container
    m_load_container = std::make_shared<ChLoadContainer>();
    system->Add(m_load_container);

    // Enable tire pressure
    if (m_pressure_enabled) {
        // If pressure was not explicitly specified, fall back to the default value.
        if (m_pressure < 0)
            m_pressure = GetDefaultPressure();

        // Let the derived class create the pressure load and add it to the load container.
        CreatePressureLoad();
    }

    // Create the contact material
    m_contact_mat = std::make_shared<ChMaterialSurfaceSMC>();
    m_contact_mat->SetFriction(m_friction);
    m_contact_mat->SetRestitution(m_restitution);
    m_contact_mat->SetYoungModulus(m_young_modulus);
    m_contact_mat->SetPoissonRatio(m_poisson_ratio);
    m_contact_mat->SetKn(m_kn);
    m_contact_mat->SetGn(m_gn);
    m_contact_mat->SetKt(m_kt);
    m_contact_mat->SetGt(m_gt);

    // Enable tire contact
    if (m_contact_enabled) {
        // Let the derived class create the contact surface and add it to the mesh.
        CreateContactSurface();
    }

    // Enable tire connection to rim
    if (m_connection_enabled) {
        // Let the derived class create the constraints and add them to the system.
        CreateRimConnections(wheel);
    }
}

void ChDeformableTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    m_visualization = std::make_shared<ChVisualizationFEAmesh>(*(m_mesh.get()));
    m_visualization->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    m_visualization->SetColorscaleMinMax(0.0, 1);
    m_visualization->SetSmoothFaces(true);
    m_mesh->AddAsset(m_visualization);
}

void ChDeformableTire::RemoveVisualizationAssets() {
    m_mesh->GetAssets().clear();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
std::shared_ptr<ChContactSurface> ChDeformableTire::GetContactSurface() const {
    if (m_contact_enabled) {
        return m_mesh->GetContactSurface(0);
    }

    std::shared_ptr<ChContactSurface> empty;
    return empty;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChDeformableTire::GetTireMass() const {
    double mass;
    ChVector<> com;
    ChMatrix33<> inertia;

    m_mesh->ComputeMassProperties(mass, com, inertia);
    return mass;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
TerrainForce ChDeformableTire::GetTireForce() const {
    auto body_frame = m_connections[0]->GetConstrainedBodyFrame();
    TerrainForce tire_force;
    tire_force.point = body_frame->GetPos();
    tire_force.force = ChVector<>(0, 0, 0);
    tire_force.moment = ChVector<>(0, 0, 0);
    return tire_force;
}

TerrainForce ChDeformableTire::ReportTireForce(ChTerrain* terrain) const {
    auto body_frame = m_connections[0]->GetConstrainedBodyFrame();
    TerrainForce tire_force;
    tire_force.point = body_frame->GetPos();
    tire_force.force = ChVector<>(0, 0, 0);
    tire_force.moment = ChVector<>(0, 0, 0);

    // Calculate and return the resultant of all reaction forces and torques in the
    // tire-wheel connections, as applied at the wheel body center of mass.
    // These encapsulate the tire-terrain interaction forces and the inertia of the tire itself.
    ChVector<> force;
    ChVector<> moment;
    for (size_t ic = 0; ic < m_connections.size(); ic++) {
        ChCoordsys<> csys = m_connections[ic]->GetLinkAbsoluteCoords();
        ChVector<> react = csys.TransformDirectionLocalToParent(m_connections[ic]->GetReactionOnBody());
        body_frame->To_abs_forcetorque(react, csys.pos, false, force, moment);
        tire_force.force += force;
        tire_force.moment += moment;
    }

    for (size_t ic = 0; ic < m_connectionsD.size(); ic++) {
        ChCoordsys<> csys = m_connectionsD[ic]->GetLinkAbsoluteCoords();
        ChVector<> react = csys.TransformDirectionLocalToParent(m_connectionsD[ic]->GetReactionOnBody());
        body_frame->To_abs_torque(react, false, moment);
        tire_force.moment += moment;
    }

    for (size_t ic = 0; ic < m_connectionsF.size(); ic++) {
        ChCoordsys<> csys = m_connectionsF[ic]->GetLinkAbsoluteCoords();
        ChVector<> react = csys.TransformDirectionLocalToParent(m_connectionsF[ic]->Get_react_force());
        body_frame->To_abs_forcetorque(react, csys.pos, false, force, moment);
        tire_force.force += force;
        tire_force.moment += moment;
        ChVector<> reactMoment = csys.TransformDirectionLocalToParent(m_connectionsF[ic]->Get_react_torque());
        tire_force.moment += reactMoment;
    }

    return tire_force;
}

}  // end namespace vehicle
}  // end namespace chrono
