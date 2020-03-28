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
// Template for a deformable ANCF tire
//
// =============================================================================

#include "chrono/fea/ChElementShellANCF.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChANCFTire.h"

namespace chrono {
namespace vehicle {

using namespace chrono::fea;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChANCFTire::ChANCFTire(const std::string& name) : ChDeformableTire(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChANCFTire::CreatePressureLoad() {
    // Create a pressure load for each element in the mesh.  Note that we set a
    // negative pressure (i.e. internal pressure, acting opposite to the surface normal)
    for (unsigned int ie = 0; ie < m_mesh->GetNelements(); ie++) {
        if (auto mshell = std::dynamic_pointer_cast<ChElementShellANCF>(m_mesh->GetElement(ie))) {
            auto load = chrono_types::make_shared<ChLoad<ChLoaderPressure>>(mshell);
            load->loader.SetPressure(-m_pressure);
            load->loader.SetStiff(false);          //// TODO:  user control?
            load->loader.SetIntegrationPoints(2);  //// TODO:  user control?
            m_load_container->Add(load);
        }
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChANCFTire::CreateContactSurface() {
    switch (m_contact_type) {
        case NODE_CLOUD: {
            auto contact_surf = chrono_types::make_shared<ChContactSurfaceNodeCloud>(m_contact_mat);
            m_mesh->AddContactSurface(contact_surf);
            contact_surf->AddAllNodes(m_contact_node_radius);
            break;
        }
        case TRIANGLE_MESH: {
            auto contact_surf = chrono_types::make_shared<ChContactSurfaceMesh>(m_contact_mat);
            m_mesh->AddContactSurface(contact_surf);
            contact_surf->AddFacesFromBoundary(m_contact_face_thickness, false);
            break;
        }
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChANCFTire::CreateRimConnections(std::shared_ptr<ChBody> wheel) {
    auto nodes = GetConnectedNodes();

    m_connections.resize(nodes.size());
    m_connectionsD.resize(nodes.size());

    for (size_t in = 0; in < nodes.size(); ++in) {
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzD>(nodes[in]);
        m_connections[in] = chrono_types::make_shared<ChLinkPointFrame>();
        m_connections[in]->Initialize(node, wheel);
        wheel->GetSystem()->Add(m_connections[in]);

        m_connectionsD[in] = chrono_types::make_shared<ChLinkDirFrame>();
        m_connectionsD[in]->Initialize(node, wheel);
        m_connectionsD[in]->SetDirectionInAbsoluteCoords(node->GetD());
        wheel->GetSystem()->Add(m_connectionsD[in]);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
