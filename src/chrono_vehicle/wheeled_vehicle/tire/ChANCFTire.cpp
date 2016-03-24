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

#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChContactSurfaceMesh.h"
#include "chrono_fea/ChContactSurfaceNodeCloud.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"

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
      m_use_mat_props(true),
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
void ChANCFTire::SetContactMaterialProperties(float friction_coefficient,
                                              float restitution_coefficient,
                                              float young_modulus,
                                              float poisson_ratio) {
    m_use_mat_props = true;

    m_friction = friction_coefficient;
    m_restitution = restitution_coefficient;
    m_young_modulus = young_modulus;
    m_poisson_ratio = poisson_ratio;
}

void ChANCFTire::SetContactMaterialCoefficients(float kn, float gn, float kt, float gt) {
    m_use_mat_props = false;

    m_kn = kn;
    m_gn = gn;
    m_kt = kt;
    m_gt = gt;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChANCFTire::Initialize(std::shared_ptr<ChBody> wheel, VehicleSide side) {
    ChTire::Initialize(wheel, side);

    ChSystemDEM* system = dynamic_cast<ChSystemDEM*>(wheel->GetSystem());
    assert(system);

    // Create the tire mesh
    m_mesh = std::make_shared<ChMesh>();
    system->Add(m_mesh);

    // Create the FEA nodes and elements
    CreateMesh(*(wheel.get()), side);

    // Create a load container
    auto load_container = std::make_shared<ChLoadContainer>();
    system->Add(load_container);

    // Enable tire pressure
    if (m_pressure_enabled) {
        // If pressure was not explicitly specified, fall back to the default value.
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

    // Create the contact material
    m_contact_mat = std::make_shared<ChMaterialSurfaceDEM>();
    if (m_use_mat_props) {
        m_contact_mat->SetYoungModulus(m_young_modulus);
        m_contact_mat->SetFriction(m_friction);
        m_contact_mat->SetRestitution(m_restitution);
        m_contact_mat->SetPoissonRatio(m_poisson_ratio);

        system->UseMaterialProperties(true);
    } else {
        m_contact_mat->SetKn(m_kn);
        m_contact_mat->SetGn(m_gn);
        m_contact_mat->SetKt(m_kt);
        m_contact_mat->SetGt(m_gt);

        system->UseMaterialProperties(false);
    }

    // Enable tire contact
    if (m_contact_enabled) {
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
                contact_surf->AddFacesFromBoundary(m_contact_face_thickness);
                contact_surf->SetMaterialSurface(m_contact_mat);
                break;
            }
        }
    }

    // Enable tire connection to rim
    if (m_connection_enabled) {
        auto nodes = GetConnectedNodes();

        m_connections.resize(nodes.size());
        m_connectionsD.resize(nodes.size());

        for (size_t in = 0; in < nodes.size(); ++in) {
            auto node = std::dynamic_pointer_cast<ChNodeFEAxyzD>(nodes[in]);
            m_connections[in] = std::make_shared<ChLinkPointFrame>();
            m_connections[in]->Initialize(node, wheel);
            system->Add(m_connections[in]);

            m_connectionsD[in] = std::make_shared<ChLinkDirFrame>();
            m_connectionsD[in]->Initialize(node, wheel);
            m_connectionsD[in]->SetDirectionInAbsoluteCoords(node->GetD());
            system->Add(m_connectionsD[in]);
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
    double mass;
    ChVector<> com;
    ChMatrix33<> inertia;

    m_mesh->ComputeMassProperties(mass, com, inertia);
    return mass;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
TireForce ChANCFTire::GetTireForce(bool cosim) const {
    TireForce tire_force;

    // If the tire is simulated together with the associated vehicle, return zero
    // force and moment. In this case, the tire forces are implicitly applied to
    // the wheel body through the tire-wheel connections.
    // Also return zero forces if the tire is not connected to the wheel.
    if (!cosim || m_connections.size() == 0) {
        tire_force.force = ChVector<>(0, 0, 0);
        tire_force.point = ChVector<>(0, 0, 0);
        tire_force.moment = ChVector<>(0, 0, 0);

        return tire_force;
    }

    // If the tire is co-simulated, calculate and return the resultant of all
    // reaction forces in the tire-wheel connections.  This encapsulated the
    // tire-terrain interaction forces and the weight of the tire itself.
    auto body_frame = m_connections[0]->GetConstrainedBodyFrame();

    ChVector<> force;
    for (size_t ic = 0; ic < m_connections.size(); ic++) {
        force += m_connections[ic]->GetReactionOnBody();
    }

    for (size_t ic = 0; ic < m_connectionsD.size(); ic++) {
        force += m_connectionsD[ic]->GetReactionOnBody();
    }

    // Calculate and return the resultant force and moment at the center of the
    // wheel body.
    tire_force.point = body_frame->GetPos();
    body_frame->To_abs_forcetorque(force, ChVector<>(0, 0, 0), 1, tire_force.force, tire_force.moment);

    return tire_force;
}

}  // end namespace vehicle
}  // end namespace chrono
