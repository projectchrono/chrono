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
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// Base class for a continuous band track shoe using an ANCFshell-based web
// (template definition).
//
// =============================================================================

#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/physics/ChGlobal.h"

#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadsBody.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeBandANCF.h"

#include "chrono_fea/ChContactSurfaceMesh.h"
#include "chrono_fea/ChContactSurfaceNodeCloud.h"
#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChElementShellANCF_8.h"
#include "chrono_fea/ChLinkDirFrame.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChNodeFEAbase.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"

#define USE_ANCF_4
//#define USE_ANCF_8

using namespace chrono::fea;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

ChTrackShoeBandANCF::ChTrackShoeBandANCF(const std::string& name) : ChTrackShoeBand(name) {}

void ChTrackShoeBandANCF::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                     const ChVector<>& location,
                                     const ChQuaternion<>& rotation) {
    // Initialize base class (create tread body)
    ChTrackShoeBand::Initialize(chassis, location, rotation);

    // Express the tread body location and orientation in global frame.
    ChVector<> loc = chassis->TransformPointLocalToParent(location);
    ChQuaternion<> rot = chassis->GetRot() * rotation;
    ChVector<> xdir = rot.GetXaxis();
    ChVector<> ydir = rot.GetYaxis();
    ChVector<> zdir = rot.GetZaxis();

    // Reference point for creating the mesh nodes for this track shoe
    ChVector<> seg_loc = loc + (0.5 * GetToothBaseLength()) * xdir - (0.5 * GetBeltWidth()) * ydir;

    // Get starting index for mesh nodes contributed by the track shoe
    assert(m_web_mesh);
    m_starting_node_index = m_web_mesh->GetNnodes();

#ifdef USE_ANCF_4
    int N_x = m_num_elements_length + 1;
    int N_y = m_num_elements_width + 1;

    double dx = GetWebLength() / m_num_elements_length;
    double dy = GetBeltWidth() / m_num_elements_width;

    double dz_steel = 0.05 * 25.4 / 1000.0;
    double dz_rubber = (GetWebThickness() - dz_steel) / 2;

    // Create an orthotropic material.
    // All layers for all elements share the same material.
    double rho_rubber = 1.1e3;
    ChVector<> E_rubber(0.01e9, 0.01e9, 0.01e9);
    ChVector<> nu_rubber(0.3, 0.3, 0.3);
    // ChVector<> G(0.0003e9, 0.0003e9, 0.0003e9);
    ChVector<> G_rubber = E_rubber / (2 * (1 + .49));
    auto mat_rubber = std::make_shared<ChMaterialShellANCF>(rho_rubber, E_rubber, nu_rubber, G_rubber);

    // Create an orthotropic material.
    // All layers for all elements share the same material.
    double rho_steel = 7900.0;
    ChVector<> E_steel(210e9, 210e9, 210e9);
    ChVector<> nu_steel(0.3, 0.3, 0.3);
    // ChVector<> G(0.0003e9, 0.0003e9, 0.0003e9);
    ChVector<> G_steel = E_steel / (2 * (1 + .3));
    auto mat_steel = std::make_shared<ChMaterialShellANCF>(rho_steel, E_steel, nu_steel, G_steel);

    // Create and add the nodes
    for (int x_idx = 0; x_idx < N_x; x_idx++) {
        for (int y_idx = 0; y_idx < N_y; y_idx++) {
            // Node location
            auto node_loc = seg_loc + x_idx * dx * xdir + y_idx * dy * ydir;

            // Node direction
            auto node_dir = zdir;

            // Create the node
            auto node = std::make_shared<ChNodeFEAxyzD>(node_loc, node_dir);

            node->SetMass(0);

            // Add node to mesh
            m_web_mesh->AddNode(node);
        }
    }

    // Create the elements
    for (int x_idx = 0; x_idx < m_num_elements_length; x_idx++) {
        for (int y_idx = 0; y_idx < m_num_elements_width; y_idx++) {
            // Adjacent nodes
            unsigned int node0 = m_starting_node_index + y_idx + x_idx * N_y;
            unsigned int node1 = m_starting_node_index + y_idx + (x_idx + 1) * N_y;
            unsigned int node2 = m_starting_node_index + (y_idx + 1) + (x_idx + 1) * N_y;
            unsigned int node3 = m_starting_node_index + (y_idx + 1) + x_idx * N_y;

            // Create the element and set its nodes.
            auto element = std::make_shared<ChElementShellANCF>();
            element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_web_mesh->GetNode(node0)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_web_mesh->GetNode(node1)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_web_mesh->GetNode(node2)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_web_mesh->GetNode(node3)));

            // Set element dimensions
            element->SetDimensions(dx, dy);

            // Add a single layers with a fiber angle of 0 degrees.
            element->AddLayer(dz_rubber, 0 * CH_C_DEG_TO_RAD, mat_rubber);
            element->AddLayer(dz_steel, 0 * CH_C_DEG_TO_RAD, mat_steel);
            element->AddLayer(dz_rubber, 0 * CH_C_DEG_TO_RAD, mat_rubber);

            // Set other element properties
            element->SetAlphaDamp(0.05);   // Structural damping for this element
            element->SetGravityOn(false);  // turn internal gravitational force calculation off

            // Add element to mesh
            m_web_mesh->AddElement(element);
        }
    }

#endif

#ifdef USE_ANCF_8
    int N_x_edge = 2 * m_num_elements_length + 1;
    int N_y_edge = 2 * m_num_elements_width + 1;
    int N_x_mid = m_num_elements_length + 1;
    int N_y_mid = m_num_elements_width + 1;

    double dx = GetWebLength() / (2 * m_num_elements_length);
    double dy = GetBeltWidth() / (2 * m_num_elements_width);

    double dz_steel = 0.05 * 25.4 / 1000.0;
    double dz_rubber = (GetWebThickness() - dz_steel) / 2;

    // Create an orthotropic material.
    // All layers for all elements share the same material.
    double rho_rubber = 1.1e3;
    ChVector<> E_rubber(0.01e9, 0.01e9, 0.01e9);
    ChVector<> nu_rubber(0.3, 0.3, 0.3);
    // ChVector<> G(0.0003e9, 0.0003e9, 0.0003e9);
    ChVector<> G_rubber = E_rubber / (2 * (1 + .49));
    auto mat_rubber = std::make_shared<ChMaterialShellANCF>(rho_rubber, E_rubber, nu_rubber, G_rubber);

    // Create an orthotropic material.
    // All layers for all elements share the same material.
    double rho_steel = 7900.0;
    ChVector<> E_steel(210e9, 210e9, 210e9);
    ChVector<> nu_steel(0.3, 0.3, 0.3);
    // ChVector<> G(0.0003e9, 0.0003e9, 0.0003e9);
    ChVector<> G_steel = E_steel / (2 * (1 + .3));
    auto mat_steel = std::make_shared<ChMaterialShellANCF>(rho_steel, E_steel, nu_steel, G_steel);

    // Create and add the nodes
    for (int x_idx = 0; x_idx < N_x_edge; x_idx++) {
        for (int y_idx = 0; y_idx < N_y_edge; y_idx++) {
            if ((x_idx % 2 == 1) && (y_idx % 2 == 1))
                continue;

            // Node location
            auto node_loc = seg_loc + x_idx * dx * xdir + y_idx * dy * ydir;

            // Node direction
            auto node_dir = zdir;

            // Node direction derivative
            auto node_curv = ChVector<>(0.0, 0.0, 0.0);

            // Create the node
            auto node = std::make_shared<ChNodeFEAxyzDD>(node_loc, node_dir, node_curv);

            node->SetMass(0);

            // Add node to mesh
            m_web_mesh->AddNode(node);
        }
    }

    // Create the elements
    for (int x_idx = 0; x_idx < m_num_elements_length; x_idx++) {
        for (int y_idx = 0; y_idx < m_num_elements_width; y_idx++) {
            // Adjacent nodes
            /// The node numbering is in ccw fashion as in the following scheme:
            ///         v
            ///         ^
            /// D o-----G-----o C
            ///   |     |     |
            /// --H-----+-----F-> u
            ///   |     |     |
            /// A o-----E-----o B

            unsigned int node0 = m_starting_node_index + 2 * y_idx + x_idx * (N_y_edge + N_y_mid);
            unsigned int node1 = m_starting_node_index + 2 * y_idx + (x_idx + 1) * (N_y_edge + N_y_mid);
            unsigned int node2 = m_starting_node_index + 2 * (y_idx + 1) + (x_idx + 1) * (N_y_edge + N_y_mid);
            unsigned int node3 = m_starting_node_index + 2 * (y_idx + 1) + x_idx * (N_y_edge + N_y_mid);

            unsigned int node4 = m_starting_node_index + 1 * y_idx + x_idx * (N_y_edge + N_y_mid) + N_y_edge;
            unsigned int node5 = m_starting_node_index + 2 * y_idx + (x_idx + 1) * (N_y_edge + N_y_mid) + 1;
            unsigned int node6 = m_starting_node_index + 1 * y_idx + x_idx * (N_y_edge + N_y_mid) + N_y_edge + 1;
            unsigned int node7 = m_starting_node_index + 2 * y_idx + x_idx * (N_y_edge + N_y_mid) + 1;

            // Create the element and set its nodes.
            auto element = std::make_shared<ChElementShellANCF_8>();
            element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzDD>(m_web_mesh->GetNode(node0)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzDD>(m_web_mesh->GetNode(node1)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzDD>(m_web_mesh->GetNode(node2)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzDD>(m_web_mesh->GetNode(node3)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzDD>(m_web_mesh->GetNode(node4)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzDD>(m_web_mesh->GetNode(node5)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzDD>(m_web_mesh->GetNode(node6)),
                              std::dynamic_pointer_cast<ChNodeFEAxyzDD>(m_web_mesh->GetNode(node7)));

            // Set element dimensions
            element->SetDimensions(2 * dx, 2 * dy);

            // Add a single layers with a fiber angle of 0 degrees.
            element->AddLayer(dz_rubber, 0 * CH_C_DEG_TO_RAD, mat_rubber);
            element->AddLayer(dz_steel, 0 * CH_C_DEG_TO_RAD, mat_steel);
            element->AddLayer(dz_rubber, 0 * CH_C_DEG_TO_RAD, mat_rubber);

            // Set other element properties
            element->SetAlphaDamp(0.05);   // Structural damping for this element
            element->SetGravityOn(false);  // turn internal gravitational force calculation off

            // Add element to mesh
            m_web_mesh->AddElement(element);
        }
    }

#endif
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeBandANCF::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                     const std::vector<ChCoordsys<>>& component_pos) {
    // Check the number of provided locations and orientations.
    assert(component_pos.size() == 2);

    // Initialize at origin.
    Initialize(chassis, VNULL, QUNIT);

    // Overwrite absolute body locations and orientations.
    m_shoe->SetPos(chassis->TransformPointLocalToParent(component_pos[0].pos));
    m_shoe->SetRot(chassis->GetRot() * component_pos[0].rot);

    // Overwrite absolute node locations and orientations.

    auto rot = chassis->GetRot() * component_pos[1].rot;
    ChVector<> xdir = rot.GetXaxis();
    ChVector<> ydir = rot.GetYaxis();
    ChVector<> zdir = rot.GetZaxis();

    ChVector<> seg_loc = chassis->TransformPointLocalToParent(component_pos[1].pos) - (0.5 * GetWebLength()) * xdir -
                         (0.5 * GetBeltWidth()) * ydir;

#ifdef USE_ANCF_4
    int N_x = m_num_elements_length + 1;
    int N_y = m_num_elements_width + 1;

    double dx = GetWebLength() / m_num_elements_length;
    double dy = GetBeltWidth() / m_num_elements_width;

    // Move the nodes on the mesh to the correct location
    for (int x_idx = 0; x_idx < N_x; x_idx++) {
        for (int y_idx = 0; y_idx < N_y; y_idx++) {
            // Node location
            auto node_loc = seg_loc + x_idx * dx * xdir + y_idx * dy * ydir;

            // Node direction
            auto node_dir = zdir;

            // Create the node
            int node_idx = m_starting_node_index + y_idx + x_idx * N_y;
            auto node = std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_web_mesh->GetNode(node_idx));

            node->SetPos(node_loc);
            node->SetD(node_dir);
        }
    }
#endif

#ifdef USE_ANCF_8

    int N_x_edge = 2 * m_num_elements_length + 1;
    int N_y_edge = 2 * m_num_elements_width + 1;
    int N_x_mid = m_num_elements_length + 1;
    int N_y_mid = m_num_elements_width + 1;

    double dx = GetWebLength() / (2 * m_num_elements_length);
    double dy = GetBeltWidth() / (2 * m_num_elements_width);

    // Move the nodes on the mesh to the correct location
    int node_idx = m_starting_node_index;
    for (int x_idx = 0; x_idx < N_x_edge; x_idx++) {
        for (int y_idx = 0; y_idx < N_y_edge; y_idx++) {
            if ((x_idx % 2 == 1) && (y_idx % 2 == 1))
                continue;

            // Node location
            auto node_loc = seg_loc + x_idx * dx * xdir + y_idx * dy * ydir;

            // Node direction
            auto node_dir = zdir;

            // Create the node
            auto node = std::dynamic_pointer_cast<ChNodeFEAxyzDD>(m_web_mesh->GetNode(node_idx));

            node->SetPos(node_loc);
            node->SetD(node_dir);
            node_idx++;
        }
    }
#endif
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeBandANCF::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    AddShoeVisualization();
}

void ChTrackShoeBandANCF::RemoveVisualizationAssets() {
    m_shoe->GetAssets().clear();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeBandANCF::Connect(std::shared_ptr<ChTrackShoe> next) {
    ChSystem* system = m_shoe->GetSystem();
    ChVector<> loc_cur_shoe = m_shoe->TransformPointLocalToParent(ChVector<>(GetToothBaseLength() / 2, 0, 0));
    ChQuaternion<> rot_cur_shoe = m_shoe->GetRot();
    ChVector<> loc_next_shoe =
        next->GetShoeBody()->TransformPointLocalToParent(ChVector<>(-GetToothBaseLength() / 2, 0, 0));
    ;
    ChQuaternion<> rot_next_shoe = next->GetShoeBody()->GetRot();

#ifdef USE_ANCF_4
    int N_x = m_num_elements_length + 1;
    int N_y = m_num_elements_width + 1;

    // Change the gradient on the web boundary nodes that will connect to the current shoe body
    // and then connect those web nodes to the show tread body
    for (int y_idx = 0; y_idx < N_y; y_idx++) {
        int node_idx = m_starting_node_index + y_idx + 0 * N_y;
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_web_mesh->GetNode(node_idx));

        node->SetD(rot_cur_shoe.GetZaxis());

        auto constraintxyz = std::make_shared<ChLinkPointFrame>();
        constraintxyz->Initialize(node, m_shoe);
        system->Add(constraintxyz);

        auto constraintD = std::make_shared<ChLinkDirFrame>();
        constraintD->Initialize(node, m_shoe);
        system->Add(constraintD);
    }

    // Change the gradient on the boundary nodes that will connect to the second fixed body
    // and then connect those nodes to the body
    for (int y_idx = 0; y_idx < N_y; y_idx++) {
        int node_idx = m_starting_node_index + y_idx + m_num_elements_length * N_y;
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_web_mesh->GetNode(node_idx));

        node->SetD(rot_next_shoe.GetZaxis());

        auto constraintxyz = std::make_shared<ChLinkPointFrame>();
        constraintxyz->Initialize(node, next->GetShoeBody());
        system->Add(constraintxyz);

        auto constraintD = std::make_shared<ChLinkDirFrame>();
        constraintD->Initialize(node, next->GetShoeBody());
        system->Add(constraintD);
    }
#endif

#ifdef USE_ANCF_8
    int N_x_edge = 2 * m_num_elements_length + 1;
    int N_y_edge = 2 * m_num_elements_width + 1;
    int N_x_mid = m_num_elements_length + 1;
    int N_y_mid = m_num_elements_width + 1;

    double dx = GetWebLength() / (2 * m_num_elements_length);
    double dy = GetBeltWidth() / (2 * m_num_elements_width);

    // Change the gradient on the web boundary nodes that will connect to the current shoe body
    // and then connect those web nodes to the show tread body
    for (int y_idx = 0; y_idx < N_y_edge; y_idx++) {
        int node_idx = m_starting_node_index + y_idx;
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzDD>(m_web_mesh->GetNode(node_idx));

        node->SetD(rot_cur_shoe.GetZaxis());

        auto constraintxyz = std::make_shared<ChLinkPointFrame>();
        constraintxyz->Initialize(node, m_shoe);
        system->Add(constraintxyz);

        auto constraintD = std::make_shared<ChLinkDirFrame>();
        constraintD->Initialize(node, m_shoe);
        system->Add(constraintD);
    }

    // Change the gradient on the boundary nodes that will connect to the second fixed body
    // and then connect those nodes to the body
    for (int y_idx = 0; y_idx < N_y_edge; y_idx++) {
        int node_idx = m_starting_node_index + y_idx + m_num_elements_length * (N_y_edge + N_y_mid);
        auto node = std::dynamic_pointer_cast<ChNodeFEAxyzDD>(m_web_mesh->GetNode(node_idx));

        node->SetD(rot_next_shoe.GetZaxis());

        auto constraintxyz = std::make_shared<ChLinkPointFrame>();
        constraintxyz->Initialize(node, next->GetShoeBody());
        system->Add(constraintxyz);

        auto constraintD = std::make_shared<ChLinkDirFrame>();
        constraintD->Initialize(node, next->GetShoeBody());
        system->Add(constraintD);
    }
#endif
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChTrackShoeBandANCF::SetWebMesh(std::shared_ptr<ChMesh> mesh) {
    m_web_mesh = mesh;
}

}  // end namespace vehicle
}  // end namespace chrono
