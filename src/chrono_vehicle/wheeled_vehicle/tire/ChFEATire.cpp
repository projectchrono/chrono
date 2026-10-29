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
// Base class for a deformable tire modeled with an FEA mesh
//
// =============================================================================

//// RADU TODO
//// extend this and derived classes to allow use in a double-wheel setup.
//// in particular, check how the tire FEA mesh is attached to the rim.

#include "chrono_vehicle/wheeled_vehicle/tire/ChFEATire.h"

namespace chrono {
namespace vehicle {

using namespace chrono::fea;

// -----------------------------------------------------------------------------
ChFEATire::ChFEATire(const std::string& name) : ChDeformableTire(name), m_connection_enabled(true) {}

ChFEATire::~ChFEATire() {
    if (!IsInitialized() || !m_mesh)
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
void ChFEATire::Initialize(std::shared_ptr<ChWheel> wheel) {
    ChDeformableTire::Initialize(wheel);

    ChSystemSMC* system = dynamic_cast<ChSystemSMC*>(wheel->GetSpindle()->GetSystem());
    assert(system);

    // Create the tire mesh
    m_mesh = chrono_types::make_shared<ChMesh>();
    m_mesh->SetTag(m_obj_tag);
    system->Add(m_mesh);

    // Create the FEA nodes and elements
    CreateMesh(*(wheel->GetSpindle().get()), wheel->GetSide());

    // Create a load container
    m_load_container = chrono_types::make_shared<ChLoadContainer>();
    system->Add(m_load_container);

    // Enable tire pressure
    if (m_pressure_enabled) {
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

void ChFEATire::AddVisualShapeFEA(std::shared_ptr<ChVisualShapeFEA> shape) {
    m_visFEA.push_back(shape);
}

void ChFEATire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    // If no FEA visualization shape was provided, create a single one with speed coloring
    if (m_visFEA.empty()) {
        auto visFEA = chrono_types::make_shared<ChVisualShapeFEA>();
        visFEA->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
        visFEA->SetShellResolution(3);
        visFEA->SetWireframe(false);
        visFEA->SetColormapRange(0.0, 5.0);
        visFEA->SetSmoothFaces(true);
        m_mesh->AddVisualShapeFEA(visFEA);

        return;
    }

    // Attach the requested FEA visual shapes
    for (const auto& v : m_visFEA) {
        m_mesh->AddVisualShapeFEA(v);
    }
}

void ChFEATire::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_mesh);
}

// -----------------------------------------------------------------------------
std::shared_ptr<ChContactSurface> ChFEATire::GetContactSurface() const {
    if (m_contact_enabled && m_mesh) {
        return m_mesh->GetContactSurface(0);
    }

    std::shared_ptr<ChContactSurface> empty;
    return empty;
}

std::shared_ptr<ChContactSurfaceMesh> ChFEATire::GetSurfaceMesh() const {
    if (!m_mesh)
        return nullptr;

    // Use the contact surface, if it is a triangle mesh
    if (auto contact_surf = std::dynamic_pointer_cast<ChContactSurfaceMesh>(GetContactSurface()))
        return contact_surf;

    // Otherwise, create a surface mesh from the boundary faces of the FEA mesh (not added to the mesh, so that it does
    // not participate in collision)
    if (!m_surface_mesh) {
        auto mat = m_contact_mat ? m_contact_mat : chrono_types::make_shared<ChContactMaterialSMC>();
        m_surface_mesh = chrono_types::make_shared<ChContactSurfaceMesh>(mat);
        m_surface_mesh->AddFacesFromBoundary(*m_mesh, 0, true, false, false);
    }

    return m_surface_mesh;
}

// -----------------------------------------------------------------------------
void ChFEATire::InitializeInertiaProperties() {
    ChVector3d com;
    m_mesh->ComputeMassProperties(m_mass, com, m_inertia);
    m_com = ChFrame<>(com, QUNIT);
}

void ChFEATire::UpdateInertiaProperties() {
    InitializeInertiaProperties();

    auto spindle = m_wheel->GetSpindle();
    m_xform = ChFrame<>(spindle->TransformPointLocalToParent(ChVector3d(0, GetOffset(), 0)), spindle->GetRot());
}

// -----------------------------------------------------------------------------
TerrainForce ChFEATire::ReportTireForce(ChTerrain* terrain) const {
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

}  // end namespace vehicle
}  // end namespace chrono
