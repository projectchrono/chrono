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
// Authors: Alessandro Tasora
// =============================================================================
//
// Template for a deformable tire based on Reissner-Mindlin 4 nodes shells
//
// =============================================================================

#include "chrono/fea/ChElementShellReissner4.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChReissnerTire.h"

namespace chrono {
namespace vehicle {

using namespace chrono::fea;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChReissnerTire::ChReissnerTire(const std::string& name) : ChDeformableTire(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChReissnerTire::CreatePressureLoad() {
    // Create a pressure load for each element in the mesh.  Note that we set a
    // negative pressure (i.e. internal pressure, acting opposite to the surface normal)
    for (unsigned int ie = 0; ie < m_mesh->GetNelements(); ie++) {
        if (auto mshell = std::dynamic_pointer_cast<ChElementShellReissner4>(m_mesh->GetElement(ie))) {
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
void ChReissnerTire::CreateContactSurface() {
    switch (m_contact_type) {
        case NODE_CLOUD: {
            auto contact_surf = chrono_types::make_shared<ChContactSurfaceNodeCloud>();
            m_mesh->AddContactSurface(contact_surf);
            contact_surf->AddAllNodes(m_contact_node_radius);
            contact_surf->SetMaterialSurface(m_contact_mat);
            break;
        }
        case TRIANGLE_MESH: {
            auto contact_surf = chrono_types::make_shared<ChContactSurfaceMesh>();
            m_mesh->AddContactSurface(contact_surf);
            contact_surf->AddFacesFromBoundary(m_contact_face_thickness, false);
            contact_surf->SetMaterialSurface(m_contact_mat);
            break;
        }
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChReissnerTire::CreateRimConnections(std::shared_ptr<ChBody> wheel) {
    auto nodes = GetConnectedNodes();

    m_connectionsF.resize(nodes.size());

    for (size_t in = 0; in < nodes.size(); ++in) {
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzrot>(nodes[in]);
        m_connectionsF[in] = chrono_types::make_shared<ChLinkMateFix>();
        m_connectionsF[in]->Initialize(node, wheel);
        wheel->GetSystem()->Add(m_connectionsF[in]);
    }

}

}  // end namespace vehicle
}  // end namespace chrono
