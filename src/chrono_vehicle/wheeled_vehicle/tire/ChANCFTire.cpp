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
// Template for a deformable ANCF tire
//
// =============================================================================

#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChSystemDEM.h"

#include "chrono_fea/ChContactSurfaceMesh.h"
#include "chrono_fea/ChContactSurfaceNodeCloud.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChANCFTire.h"

namespace chrono {
namespace vehicle {

using namespace chrono::fea;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChANCFTire::ChANCFTire(const std::string& name)
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
      m_pressure(-1) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChANCFTire::SetContactMaterial(float friction_coefficient,
                                    float restitution_coefficient,
                                    float young_modulus,
                                    float poisson_ratio) {
    m_friction = friction_coefficient;
    m_restitution = restitution_coefficient;
    m_young_modulus = young_modulus;
    m_poisson_ratio = poisson_ratio;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChANCFTire::Initialize(std::shared_ptr<ChBody> wheel, VehicleSide side) {
    ChSystemDEM* system = dynamic_cast<ChSystemDEM*>(wheel->GetSystem());
    assert(system);

    // Create the tire mesh
    m_mesh = std::make_shared<ChMesh>();
    system->Add(m_mesh);

    // Create the FEA nodes and elements
    CreateMesh(m_mesh, *(wheel.get()), side);

    // Create a load container
    auto load_container = std::make_shared<ChLoadContainer>();
    system->Add(load_container);

    if (m_pressure_enabled) {
        // If pressure was not explicitly specified, fall back to the
        // default value.
        if (m_pressure < 0)
            m_pressure = GetDefaultPressure();

        // Create a pressure load for each element in the mesh.  Note that we set a
        // negative pressure (i.e. internal pressure, acting opposite to the surface normal)
        for (unsigned int ie = 0; ie < m_mesh->GetNelements(); ie++) {
            auto load = std::make_shared<ChLoad<ChLoaderPressure>>(
                std::static_pointer_cast<ChElementShellANCF>(m_mesh->GetElement(ie)));
            load->loader.SetPressure(-m_pressure);
            load->loader.SetStiff(false);          //// TODO:  user control?
            load->loader.SetIntegrationPoints(2);  //// TODO:  user control?
            load_container->Add(load);
        }
    }

    if (m_contact_enabled) {
        // Create the contact material
        auto contact_mat = std::make_shared<ChMaterialSurfaceDEM>();
        contact_mat->SetYoungModulus(m_young_modulus);
        contact_mat->SetFriction(m_friction);
        contact_mat->SetRestitution(m_restitution);
        contact_mat->SetPoissonRatio(m_poisson_ratio);

        // Create the contact surface
        switch (m_contact_type) {
            case NODE_CLOUD: {
                auto contact_surf = std::make_shared<ChContactSurfaceNodeCloud>();
                m_mesh->AddContactSurface(contact_surf);
                contact_surf->AddAllNodes(m_contact_node_radius);
                contact_surf->SetMaterialSurface(contact_mat);
                break;
            }
            case TRIANGLE_MESH: {
                auto contact_surf = std::make_shared<ChContactSurfaceMesh>();
                m_mesh->AddContactSurface(contact_surf);
                contact_surf->AddFacesFromBoundary(m_contact_face_thickness);
                contact_surf->SetMaterialSurface(contact_mat);
                break;
            }
        }
    }

    if (m_connection_enabled) {
        // Connect nodes to rim
        auto nodes = GetConnectedNodes(m_mesh);

        for (size_t in = 0; in < nodes.size(); ++in) {
            auto linkP = std::make_shared<ChLinkPointFrame>();
            linkP->Initialize(nodes[in], wheel);
            system->Add(linkP);

            auto linkD = std::make_shared<ChLinkDirFrame>();
            linkD->Initialize(nodes[in], wheel);
            linkD->SetDirectionInAbsoluteCoords(nodes[in]->GetD());
            system->Add(linkD);
        }
    }

    // Attach mesh visualization
    ////auto visualizationW = std::make_shared<ChVisualizationFEAmesh>(*(m_mesh.get()));
    ////visualizationW->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    ////visualizationW->SetWireframe(true);
    ////m_mesh->AddAsset(visualizationW);

    auto visualizationS = std::make_shared<ChVisualizationFEAmesh>(*(m_mesh.get()));
    visualizationS->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    visualizationS->SetColorscaleMinMax(0.0, 1);
    visualizationS->SetSmoothFaces(true);
    m_mesh->AddAsset(visualizationS);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChANCFTire::GetMass() const {
    double mass = 0;
    for (unsigned int ie = 0; ie < m_mesh->GetNelements(); ++ie) {
        auto element = std::static_pointer_cast<ChElementShellANCF>(m_mesh->GetElement(ie));
        //// TODO
    }
    return mass;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
TireForce ChANCFTire::GetTireForce() const {
    TireForce tire_force;

    tire_force.force = ChVector<>(0, 0, 0);
    tire_force.point = ChVector<>(0, 0, 0);
    tire_force.moment = ChVector<>(0, 0, 0);

    return tire_force;
}

}  // end namespace vehicle
}  // end namespace chrono
