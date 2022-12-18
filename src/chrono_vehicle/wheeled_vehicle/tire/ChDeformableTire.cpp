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

//// RADU TODO
//// extend this and derived classes to allow use in a double-wheel setup.
//// in particular, check how the tire FEA mesh is attached to the rim.

#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"

namespace chrono {
namespace vehicle {

using namespace chrono::fea;

// -----------------------------------------------------------------------------
ChDeformableTire::ChDeformableTire(const std::string& name)
    : ChTire(name),
      m_connection_enabled(true),
      m_pressure_enabled(true),
      m_contact_enabled(true),
      m_pressure(-1),
      m_contact_type(ContactSurfaceType::NODE_CLOUD),
      m_contact_node_radius(0.001),
      m_contact_face_thickness(0.0) {}

ChDeformableTire::~ChDeformableTire() {
    auto sys = m_mesh->GetSystem();
    if (sys) {
        sys->Remove(m_mesh);
        sys->Remove(m_load_container);
        for (size_t i = 0; i < m_connections.size(); i++) {
            sys->Remove(m_connections[i]);
        }
        for (size_t i = 0; i < m_connectionsD.size(); i++) {
            sys->Remove(m_connectionsD[i]);
        }
        for (size_t i = 0; i < m_connectionsF.size(); i++) {
            sys->Remove(m_connectionsF[i]);
        }
    }
}

// -----------------------------------------------------------------------------
void ChDeformableTire::Initialize(std::shared_ptr<ChWheel> wheel) {
    ChTire::Initialize(wheel);

    ChSystemSMC* system = dynamic_cast<ChSystemSMC*>(wheel->GetSpindle()->GetSystem());
    assert(system);

    // Create the tire mesh
    m_mesh = chrono_types::make_shared<ChMesh>();
    system->Add(m_mesh);

    // Create the FEA nodes and elements
    CreateMesh(*(wheel->GetSpindle().get()), wheel->GetSide());

    // Create a load container
    m_load_container = chrono_types::make_shared<ChLoadContainer>();
    system->Add(m_load_container);

    // Enable tire pressure
    if (m_pressure_enabled) {
        // If pressure was not explicitly specified, fall back to the default value.
        if (m_pressure < 0)
            m_pressure = GetDefaultPressure();

        // Let the derived class create the pressure load and add it to the load container.
        CreatePressureLoad();
    }

    // Enable tire contact
    if (m_contact_enabled) {
        // Let the derived class create the contact surface and add it to the mesh.
        CreateContactMaterial();
        assert(m_contact_mat && m_contact_mat->GetContactMethod() == ChContactMethod::SMC);
        CreateContactSurface();
    }

    // Enable tire connection to rim
    if (m_connection_enabled) {
        // Let the derived class create the constraints and add them to the system.
        CreateRimConnections(wheel->GetSpindle());
    }
}

void ChDeformableTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    m_visualization = chrono_types::make_shared<ChVisualShapeFEA>(m_mesh);
    m_visualization->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    m_visualization->SetColorscaleMinMax(0.0, 1);
    m_visualization->SetSmoothFaces(true);
    m_mesh->AddVisualShapeFEA(m_visualization);
}

void ChDeformableTire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_mesh);
}

// -----------------------------------------------------------------------------
std::shared_ptr<ChContactSurface> ChDeformableTire::GetContactSurface() const {
    if (m_contact_enabled) {
        return m_mesh->GetContactSurface(0);
    }

    std::shared_ptr<ChContactSurface> empty;
    return empty;
}

// -----------------------------------------------------------------------------
void ChDeformableTire::InitializeInertiaProperties() {
    ChVector<> com;
    m_mesh->ComputeMassProperties(m_mass, com, m_inertia);
    m_com = ChFrame<>(com, QUNIT);
}

void ChDeformableTire::UpdateInertiaProperties() {
    InitializeInertiaProperties();

    auto spindle = m_wheel->GetSpindle();
    m_xform = ChFrame<>(spindle->TransformPointLocalToParent(ChVector<>(0, GetOffset(), 0)), spindle->GetRot());
}

double ChDeformableTire::GetTireMass() const {
    return m_mass;
}

ChVector<> ChDeformableTire::GetTireInertia() const {
    return m_inertia.diagonal();
}

// -----------------------------------------------------------------------------
TerrainForce ChDeformableTire::GetTireForce() const {
    TerrainForce tire_force;
    tire_force.point = m_wheel->GetPos();
    tire_force.force = ChVector<>(0, 0, 0);
    tire_force.moment = ChVector<>(0, 0, 0);
    return tire_force;
}

TerrainForce ChDeformableTire::ReportTireForce(ChTerrain* terrain) const {
    TerrainForce tire_force;
    tire_force.point = m_wheel->GetPos();
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
        m_wheel->GetSpindle()->To_abs_forcetorque(react, csys.pos, false, force, moment);
        tire_force.force += force;
        tire_force.moment += moment;
    }

    for (size_t ic = 0; ic < m_connectionsD.size(); ic++) {
        ChCoordsys<> csys = m_connectionsD[ic]->GetLinkAbsoluteCoords();
        moment = csys.TransformDirectionLocalToParent(m_connectionsD[ic]->GetReactionOnBody());
        tire_force.moment += moment;
    }

    for (size_t ic = 0; ic < m_connectionsF.size(); ic++) {
        ChCoordsys<> csys = m_connectionsF[ic]->GetLinkAbsoluteCoords();
        ChVector<> react = csys.TransformDirectionLocalToParent(m_connectionsF[ic]->Get_react_force());
        m_wheel->GetSpindle()->To_abs_forcetorque(react, csys.pos, false, force, moment);
        tire_force.force += force;
        tire_force.moment += moment;
        ChVector<> reactMoment = csys.TransformDirectionLocalToParent(m_connectionsF[ic]->Get_react_torque());
        tire_force.moment += reactMoment;
    }

    return tire_force;
}

}  // end namespace vehicle
}  // end namespace chrono
