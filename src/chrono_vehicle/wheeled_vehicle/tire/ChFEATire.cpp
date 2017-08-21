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
// Template for a deformable co-rotational FEA tire
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/tire/ChFEATire.h"

namespace chrono {
namespace vehicle {

using namespace chrono::fea;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChFEATire::ChFEATire(const std::string& name) : ChDeformableTire(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChFEATire::CreatePressureLoad() {
    // Get the list of internal nodes and create the internal mesh surface.
    auto nodes = GetInternalNodes();
    auto surface = std::make_shared<ChMeshSurface>();
    m_mesh->AddMeshSurface(surface);
    surface->AddFacesFromNodeSet(nodes);

    // Create a pressure load for each element in the mesh surface.  Note that we set a
    // positive pressure (i.e. internal pressure, acting opposite to the surface normal)
    for (unsigned int ie = 0; ie < surface->GetFacesList().size(); ie++) {
        auto load = std::make_shared<ChLoad<ChLoaderPressure>>(surface->GetFacesList()[ie]);
        load->loader.SetPressure(m_pressure);
        load->loader.SetStiff(false);
        m_load_container->Add(load);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChFEATire::CreateContactSurface() {
    switch (m_contact_type) {
        case NODE_CLOUD: {
            auto contact_surf = std::make_shared<ChContactSurfaceNodeCloud>();
            m_mesh->AddContactSurface(contact_surf);
            contact_surf->AddAllNodes(m_contact_node_radius);
            contact_surf->SetMaterialSurface(m_contact_mat);
            break;
        }
        case TRIANGLE_MESH: {
            auto contact_surf = std::make_shared<ChContactSurfaceMesh>();
            m_mesh->AddContactSurface(contact_surf);
            contact_surf->AddFacesFromBoundary();
            contact_surf->SetMaterialSurface(m_contact_mat);
            break;
        }
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChFEATire::CreateRimConnections(std::shared_ptr<ChBody> wheel) {
    auto nodes = GetConnectedNodes();

    m_connections.resize(nodes.size());

    for (size_t in = 0; in < nodes.size(); ++in) {
        m_connections[in] = std::make_shared<ChLinkPointFrame>();
        m_connections[in]->Initialize(std::dynamic_pointer_cast<ChNodeFEAxyz>(nodes[in]), wheel);
        wheel->GetSystem()->Add(m_connections[in]);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
