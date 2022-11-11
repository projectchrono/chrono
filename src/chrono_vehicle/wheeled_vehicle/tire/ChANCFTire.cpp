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

#include "chrono/core/ChCubicSpline.h"
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChElementShellANCF_3833.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChANCFTire.h"

namespace chrono {
namespace vehicle {

using namespace chrono::fea;

ChANCFTire::ChANCFTire(const std::string& name) : ChDeformableTire(name) {}

void ChANCFTire::CreatePressureLoad() {
    // Create a pressure load for each element in the mesh.  Note that we set a
    // negative pressure (i.e. internal pressure, acting opposite to the surface normal)
    for (unsigned int ie = 0; ie < m_mesh->GetNelements(); ie++) {
        if (auto mshell = std::dynamic_pointer_cast<ChElementShellANCF_3423>(m_mesh->GetElement(ie))) {
            auto load = chrono_types::make_shared<ChLoad<ChLoaderPressure>>(mshell);
            load->loader.SetPressure(-m_pressure);
            load->loader.SetStiff(false);          //// TODO:  user control?
            load->loader.SetIntegrationPoints(2);  //// TODO:  user control?
            m_load_container->Add(load);
        }
    }
}

void ChANCFTire::CreateContactSurface() {
    switch (m_contact_type) {
        case ContactSurfaceType::NODE_CLOUD: {
            auto contact_surf = chrono_types::make_shared<ChContactSurfaceNodeCloud>(m_contact_mat);
            m_mesh->AddContactSurface(contact_surf);
            contact_surf->AddAllNodes(m_contact_node_radius);
            break;
        }
        case ContactSurfaceType::TRIANGLE_MESH: {
            auto contact_surf = chrono_types::make_shared<ChContactSurfaceMesh>(m_contact_mat);
            m_mesh->AddContactSurface(contact_surf);
            contact_surf->AddFacesFromBoundary(m_contact_face_thickness, false);
            break;
        }
    }
}

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

// -----------------------------------------------------------------------------

std::vector<std::shared_ptr<fea::ChNodeFEAbase>> ChANCFTire::CreateMeshANCF4(const Profile& profile,
                                                                             const Section& bead,
                                                                             const Section& sidewall,
                                                                             const Section& tread,
                                                                             int div_circumference,
                                                                             double rim_radius,
                                                                             double damping,
                                                                             std::shared_ptr<fea::ChMesh> mesh,
                                                                             const ChFrameMoving<>& wheel_frame) {
    // Create piece-wise cubic spline approximation of the tire profile.
    //   x - radial direction
    //   y - transversal direction
    ChCubicSpline splineX(profile.t, profile.x);
    ChCubicSpline splineY(profile.t, profile.y);

    // Total number of mesh divisions across width
    int div_width = 2 * (bead.num_divs + sidewall.num_divs + tread.num_divs);

    // Create the mesh nodes.
    // The nodes are first created in the wheel local frame, assuming Y as the tire axis,
    // and are then transformed to the global frame.
    for (int i = 0; i < div_circumference; i++) {
        double phi = (CH_C_2PI * i) / div_circumference;
        ChVector<> nrm(-std::sin(phi), 0, std::cos(phi));

        for (int j = 0; j <= div_width; j++) {
            double t_prf = double(j) / div_width;
            double x_prf, xp_prf, xpp_prf;
            double y_prf, yp_prf, ypp_prf;
            splineX.Evaluate(t_prf, x_prf, xp_prf, xpp_prf);
            splineY.Evaluate(t_prf, y_prf, yp_prf, ypp_prf);

            // Node position with respect to rim center
            double x = (rim_radius + x_prf) * std::cos(phi);
            double y = y_prf;
            double z = (rim_radius + x_prf) * std::sin(phi);
            // Node position in global frame (actual coordinate values)
            ChVector<> loc = wheel_frame.TransformPointLocalToParent(ChVector<>(x, y, z));

            // Node direction
            ChVector<> tan_prf(std::cos(phi) * xp_prf, yp_prf, std::sin(phi) * xp_prf);
            ChVector<> nrm_prf = Vcross(tan_prf, nrm).GetNormalized();
            ChVector<> dir = wheel_frame.TransformDirectionLocalToParent(nrm_prf);

            auto node = chrono_types::make_shared<ChNodeFEAxyzD>(loc, dir);

            // Node velocity
            ChVector<> vel = wheel_frame.PointSpeedLocalToParent(ChVector<>(x, y, z));
            node->SetPos_dt(vel);
            node->SetMass(0);
            mesh->AddNode(node);
        }
    }

    // Create the ANCF shell elements
    for (int i = 0; i < div_circumference; i++) {
        for (int j = 0; j < div_width; j++) {
            // Adjacent nodes
            int inode0, inode1, inode2, inode3;
            inode1 = j + i * (div_width + 1);
            inode2 = j + 1 + i * (div_width + 1);
            if (i == div_circumference - 1) {
                inode0 = j;
                inode3 = j + 1;
            } else {
                inode0 = j + (i + 1) * (div_width + 1);
                inode3 = j + 1 + (i + 1) * (div_width + 1);
            }

            auto node0 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(inode0));
            auto node1 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(inode1));
            auto node2 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(inode2));
            auto node3 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(inode3));

            // Create the element and set its nodes.
            auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
            element->SetNodes(node0, node1, node2, node3);

            // Element dimensions
            double len_circumference =
                0.5 * ((node1->GetPos() - node0->GetPos()).Length() + (node3->GetPos() - node2->GetPos()).Length());
            double len_width =
                0.5 * ((node2->GetPos() - node1->GetPos()).Length() + (node3->GetPos() - node0->GetPos()).Length());

            element->SetDimensions(len_circumference, len_width);

            // Figure out the section for this element
            int b1 = bead.num_divs;
            int b2 = div_width - bead.num_divs;
            int s1 = b1 + sidewall.num_divs;
            int s2 = b2 - sidewall.num_divs;
            if (j < b1 || j >= b2) {  // Bead section
                for (int im = 0; im < bead.num_layers; im++) {
                    element->AddLayer(bead.thickness[im], CH_C_DEG_TO_RAD * bead.angle[im], bead.mat[im]);
                }
            } else if (j < s1 || j >= s2) {  // Sidewall section
                for (int im = 0; im < sidewall.num_layers; im++) {
                    element->AddLayer(sidewall.thickness[im], CH_C_DEG_TO_RAD * sidewall.angle[im], sidewall.mat[im]);
                }
            } else {  // Tread section
                for (int im = 0; im < tread.num_layers; im++) {
                    element->AddLayer(tread.thickness[im], CH_C_DEG_TO_RAD * tread.angle[im], tread.mat[im]);
                }
            }

            // Set other element properties
            element->SetAlphaDamp(damping);

            // Add element to mesh
            mesh->AddElement(element);
        }
    }

    // Generate the list of nodes connected to the rim
    std::vector<std::shared_ptr<fea::ChNodeFEAbase>> nodes;

    for (int i = 0; i < div_circumference; i++) {
        for (int j = 0; j <= div_width; j++) {
            int index = j + i * (div_width + 1);
            if (index % (div_width + 1) == 0) {
                nodes.push_back(std::dynamic_pointer_cast<ChNodeFEAbase>(mesh->GetNode(index)));
                nodes.push_back(std::dynamic_pointer_cast<ChNodeFEAbase>(mesh->GetNode(index + div_width)));
            }
        }
    }

    return nodes;
}

std::vector<std::shared_ptr<fea::ChNodeFEAbase>> ChANCFTire::CreateMeshANCF8(const Profile& profile,
                                                                             const Section& bead,
                                                                             const Section& sidewall,
                                                                             const Section& tread,
                                                                             int div_circumference,
                                                                             double rim_radius,
                                                                             double damping,
                                                                             std::shared_ptr<fea::ChMesh> mesh,
                                                                             const ChFrameMoving<>& wheel_frame) {
    // Create piece-wise cubic spline approximation of the tire profile.
    //   x - radial direction
    //   y - transversal direction
    ChCubicSpline splineX(profile.t, profile.x);
    ChCubicSpline splineY(profile.t, profile.y);

    // Total number of mesh divisions across width
    int div_width = 2 * (bead.num_divs + sidewall.num_divs + tread.num_divs);

    // Create the mesh nodes.
    // The nodes are first created in the wheel local frame, assuming Y as the tire axis,
    // and are then transformed to the global frame.
    double dphi = CH_C_2PI / (2 * div_circumference);
    double dt = 1.0 / (2 * div_width);

    ChVector<> curv(0, 0, 0);

    for (int i = 0; i < 2 * div_circumference; i++) {
        double phi = i * dphi;
        ChVector<> nrm(-std::sin(phi), 0, std::cos(phi));

        for (int j = 0; j <= 2 * div_width; j++) {
            double t_prf = j * dt;
            double x_prf, xp_prf, xpp_prf;
            double y_prf, yp_prf, ypp_prf;
            splineX.Evaluate(t_prf, x_prf, xp_prf, xpp_prf);
            splineY.Evaluate(t_prf, y_prf, yp_prf, ypp_prf);

            // Node position with respect to rim center
            double x = (rim_radius + x_prf) * std::cos(phi);
            double y = y_prf;
            double z = (rim_radius + x_prf) * std::sin(phi);
            // Node position in global frame (actual coordinate values)
            ChVector<> loc = wheel_frame.TransformPointLocalToParent(ChVector<>(x, y, z));

            // Node direction
            ChVector<> tan_prf(std::cos(phi) * xp_prf, yp_prf, std::sin(phi) * xp_prf);
            ChVector<> nrm_prf = Vcross(tan_prf, nrm).GetNormalized();
            ChVector<> dir = wheel_frame.TransformDirectionLocalToParent(nrm_prf);

            auto node = chrono_types::make_shared<ChNodeFEAxyzDD>(loc, dir, curv);

            // Node velocity
            ChVector<> vel = wheel_frame.PointSpeedLocalToParent(ChVector<>(x, y, z));
            node->SetPos_dt(vel);
            node->SetMass(0);
            mesh->AddNode(node);
        }
    }

    // Create the ANCF shell elements
    // The node numbering is in ccw fashion as in the following scheme:
    //         v
    //         ^
    // D o-----G-----o C
    //   |     |     |
    // --H-----+-----F-> u
    //   |     |     |
    // A o-----E-----o B
    //
    // Simultaneously collect nodes connected to the rim.

    int NW = 2 * div_width + 1;
    std::vector<std::shared_ptr<fea::ChNodeFEAbase>> nodes;

    for (int i = 0; i < div_circumference; i++) {
        for (int j = 0; j < div_width; j++) {
            // Indices of adjacent nodes
            int iA = (2 * i + 0) * NW + 2 * j + 0;
            int iE = (2 * i + 0) * NW + 2 * j + 1;
            int iB = (2 * i + 0) * NW + 2 * j + 2;

            int iH = (2 * i + 1) * NW + 2 * j + 0;
            int iF = (2 * i + 1) * NW + 2 * j + 2;

            int iD = (2 * i + 2) * NW + 2 * j + 0;
            int iG = (2 * i + 2) * NW + 2 * j + 1;
            int iC = (2 * i + 2) * NW + 2 * j + 2;

            if (i == div_circumference - 1) {
                iD = 2 * j + 0;
                iG = 2 * j + 1;
                iC = 2 * j + 2;
            }

            auto nodeA = std::dynamic_pointer_cast<ChNodeFEAxyzDD>(mesh->GetNode(iA));
            auto nodeB = std::dynamic_pointer_cast<ChNodeFEAxyzDD>(mesh->GetNode(iB));
            auto nodeC = std::dynamic_pointer_cast<ChNodeFEAxyzDD>(mesh->GetNode(iC));
            auto nodeD = std::dynamic_pointer_cast<ChNodeFEAxyzDD>(mesh->GetNode(iD));

            auto nodeE = std::dynamic_pointer_cast<ChNodeFEAxyzDD>(mesh->GetNode(iE));
            auto nodeF = std::dynamic_pointer_cast<ChNodeFEAxyzDD>(mesh->GetNode(iF));
            auto nodeG = std::dynamic_pointer_cast<ChNodeFEAxyzDD>(mesh->GetNode(iG));
            auto nodeH = std::dynamic_pointer_cast<ChNodeFEAxyzDD>(mesh->GetNode(iH));

            if (j == 0) {
                nodes.push_back(nodeA);
                nodes.push_back(nodeH);
            } else if (j == div_width - 1) {
                nodes.push_back(nodeB);
                nodes.push_back(nodeF);
            }

            // Create the element and set its nodes.
            auto element = chrono_types::make_shared<ChElementShellANCF_3833>();
            element->SetNodes(nodeA, nodeB, nodeC, nodeD, nodeE, nodeF, nodeG, nodeH);

            // Element dimensions
            double len_circumference = (nodeG->GetPos() - nodeE->GetPos()).Length();
            double len_width = (nodeF->GetPos() - nodeH->GetPos()).Length();
            element->SetDimensions(len_circumference, len_width);

            // Figure out the section for this element
            int b1 = bead.num_divs;
            int b2 = div_width - bead.num_divs;
            int s1 = b1 + sidewall.num_divs;
            int s2 = b2 - sidewall.num_divs;
            if (j < b1 || j >= b2) {
                // Bead section
                for (int k = 0; k < bead.num_layers; k++)
                    element->AddLayer(bead.thickness[k], CH_C_DEG_TO_RAD * bead.angle[k], bead.mat[k]);
            } else if (j < s1 || j >= s2) {
                // Sidewall section
                for (int k = 0; k < sidewall.num_layers; k++)
                    element->AddLayer(sidewall.thickness[k], CH_C_DEG_TO_RAD * sidewall.angle[k], sidewall.mat[k]);
            } else {
                // Tread section
                for (int k = 0; k < tread.num_layers; k++)
                    element->AddLayer(tread.thickness[k], CH_C_DEG_TO_RAD * tread.angle[k], tread.mat[k]);
            }

            // Set other element properties
            element->SetAlphaDamp(damping);

            // Add element to mesh
            mesh->AddElement(element);
        }
    }

    return nodes;
}

}  // end namespace vehicle
}  // end namespace chrono
