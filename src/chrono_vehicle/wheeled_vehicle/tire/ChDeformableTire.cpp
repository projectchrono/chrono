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
    : ChTire(name), m_connection_enabled(true), m_pressure_enabled(true), m_contact_enabled(true) {}

ChDeformableTire::~ChDeformableTire() {
    if (!m_initialized)
        return;

    auto sys = m_mesh->GetSystem();
    if (!sys)
        return;

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
        if (m_pressure <= 0)
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

    InitializeInertiaProperties();
}

void ChDeformableTire::AddVisualShapeFEA(std::shared_ptr<ChVisualShapeFEA> shape) {
    m_visFEA.push_back(shape);
}

void ChDeformableTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    // If no FEA visualization shape was provided, create a single one with speed coloring
    if (m_visFEA.empty()) {
        auto visFEA = chrono_types::make_shared<ChVisualShapeFEA>();
        visFEA->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
        visFEA->SetShellResolution(3);
        visFEA->SetWireframe(false);
        visFEA->SetColorscaleMinMax(0.0, 5.0);
        visFEA->SetSmoothFaces(true);
        m_mesh->AddVisualShapeFEA(visFEA);

        return;
    }

    // Attach the requested FEA visual shapes
    for (const auto& v : m_visFEA) {
        m_mesh->AddVisualShapeFEA(v);
    }
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
    ChVector3d com;
    m_mesh->ComputeMassProperties(m_mass, com, m_inertia);
    m_com = ChFrame<>(com, QUNIT);
}

void ChDeformableTire::UpdateInertiaProperties() {
    InitializeInertiaProperties();

    auto spindle = m_wheel->GetSpindle();
    m_xform = ChFrame<>(spindle->TransformPointLocalToParent(ChVector3d(0, GetOffset(), 0)), spindle->GetRot());
}

double ChDeformableTire::GetTireMass() const {
    return m_mass;
}

ChVector3d ChDeformableTire::GetTireInertia() const {
    return m_inertia.diagonal();
}

// -----------------------------------------------------------------------------
TerrainForce ChDeformableTire::GetTireForce() const {
    TerrainForce tire_force;
    tire_force.point = m_wheel->GetPos();
    tire_force.force = ChVector3d(0, 0, 0);
    tire_force.moment = ChVector3d(0, 0, 0);
    return tire_force;
}

TerrainForce ChDeformableTire::ReportTireForce(ChTerrain* terrain) const {
    TerrainForce tire_force;
    tire_force.point = m_wheel->GetPos();
    tire_force.force = ChVector3d(0, 0, 0);
    tire_force.moment = ChVector3d(0, 0, 0);

    // Calculate and return the resultant of all reaction forces and torques in the
    // tire-wheel connections, as applied at the wheel body center of mass.
    // These encapsulate the tire-terrain interaction forces and the inertia of the tire itself.
    for (size_t ic = 0; ic < m_connections.size(); ic++) {
        ChCoordsysd csys = m_connections[ic]->GetFrameNodeAbs().GetCoordsys();
        ChVector3d react = csys.TransformDirectionLocalToParent(m_connections[ic]->GetReactionOnBody());
        ChWrenchd w = m_wheel->GetSpindle()->AppliedForceParentToWrenchParent(react, csys.pos);
        tire_force.force += w.force;
        tire_force.moment += w.torque;
    }

    for (size_t ic = 0; ic < m_connectionsD.size(); ic++) {
        ChCoordsysd csys = m_connectionsD[ic]->GetFrameNodeAbs().GetCoordsys();
        ChVector3d torque = csys.TransformDirectionLocalToParent(m_connectionsD[ic]->GetReactionOnBody());
        tire_force.moment += torque;
    }

    for (size_t ic = 0; ic < m_connectionsF.size(); ic++) {
        ChCoordsysd csys = m_connectionsF[ic]->GetFrame2Abs().GetCoordsys();
        ChWrenchd reaction = m_connectionsF[ic]->GetReaction2();
        ChVector3d react = csys.TransformDirectionLocalToParent(reaction.force);
        ChWrenchd w = m_wheel->GetSpindle()->AppliedForceParentToWrenchParent(react, csys.pos);
        tire_force.force += w.force;
        tire_force.moment += w.torque;
        ChVector3d reactMoment = csys.TransformDirectionLocalToParent(reaction.torque);
        tire_force.moment += reactMoment;
    }

    return tire_force;
}

TerrainForce ChDeformableTire::ReportTireForceLocal(ChTerrain* terrain, ChCoordsys<>& tire_frame) const {
    std::cerr << "ChDeformableTire::ReportTireForceLocal not implemented." << std::endl;
    throw std::runtime_error("ChDeformableTire::ReportTireForceLocal not implemented.");
}

}  // end namespace vehicle
}  // end namespace chrono
