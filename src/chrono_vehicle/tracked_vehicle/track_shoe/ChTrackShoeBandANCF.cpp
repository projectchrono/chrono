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

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChDataPath.h"

#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadsBody.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/track_shoe/ChTrackShoeBandANCF.h"

#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChElementShellANCF_3833.h"
#include "chrono/fea/ChLinkNodeSlopeFrame.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChNodeFEAbase.h"
#include "chrono/assets/ChVisualShapeFEA.h"

using namespace chrono::fea;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

ChTrackShoeBandANCF::ChTrackShoeBandANCF(const std::string& name, ElementType element_type, bool constrain_curvature)
    : ChTrackShoeBand(name), m_element_type(element_type), m_constrain_curvature(constrain_curvature) {}

ChTrackShoeBandANCF::~ChTrackShoeBandANCF() {
    if (!m_connections[0])
        return;

    auto sys = m_connections[0]->GetSystem();
    if (sys) {
        for (auto c : m_connections)
            sys->Remove(c);
    }
}

// -----------------------------------------------------------------------------
void ChTrackShoeBandANCF::SetWebMesh(std::shared_ptr<fea::ChMesh> mesh) {
    m_web_mesh = mesh;
}

void ChTrackShoeBandANCF::SetWebMeshProperties(std::shared_ptr<fea::ChMaterialShellANCF> rubber_mat,
                                               std::shared_ptr<fea::ChMaterialShellANCF> steel_mat,
                                               double angle_1,
                                               double angle_2,
                                               double angle_3,
                                               double alpha) {
    m_rubber_mat = rubber_mat;
    m_steel_mat = steel_mat;
    m_angle_1 = angle_1;
    m_angle_2 = angle_2;
    m_angle_3 = angle_3;
    m_alpha = alpha;
}

// -----------------------------------------------------------------------------
void ChTrackShoeBandANCF::Construct(std::shared_ptr<ChChassis> chassis,
                                    const ChVector3d& location,
                                    const ChQuaternion<>& rotation) {
    // Invoke base class (construct m_shoe body)
    ChTrackShoeBand::Construct(chassis, location, rotation);

    // Express the tread body location and orientation in global frame.
    auto chassis_body = chassis->GetBody();
    ChVector3d loc = chassis_body->TransformPointLocalToParent(location);
    ChQuaternion<> rot = chassis_body->GetRot() * rotation;
    ChVector3d xdir = rot.GetAxisX();
    ChVector3d ydir = rot.GetAxisY();
    ChVector3d zdir = rot.GetAxisZ();

    // Reference point for creating the mesh nodes for this track shoe
    ChVector3d seg_loc = loc + (0.5 * GetToothBaseLength()) * xdir - (0.5 * GetBeltWidth()) * ydir;

    // Get starting index for mesh nodes contributed by the track shoe
    assert(m_web_mesh);
    m_starting_node_index = m_web_mesh->GetNumNodes();

    int num_elements_length = GetNumElementsLength();
    int num_elements_width = GetNumElementsWidth();

    switch (m_element_type) {
        case ElementType::ANCF_4: {
            int N_x = num_elements_length + 1;
            int N_y = num_elements_width + 1;

            double dx = GetWebLength() / num_elements_length;
            double dy = GetBeltWidth() / num_elements_width;

            double dz_steel = GetSteelLayerThickness();
            double dz_rubber = (GetWebThickness() - dz_steel) / 2;

            // Create and add the nodes
            for (int x_idx = 0; x_idx < N_x; x_idx++) {
                for (int y_idx = 0; y_idx < N_y; y_idx++) {
                    // Node location
                    auto node_loc = seg_loc + x_idx * dx * xdir + y_idx * dy * ydir;
                    // Node direction
                    auto node_dir = zdir;
                    // Create the node
                    auto node = chrono_types::make_shared<ChNodeFEAxyzD>(node_loc, node_dir);
                    // No additional mass assigned to nodes
                    node->SetMass(0);
                    // Add node to mesh
                    m_web_mesh->AddNode(node);
                }
            }

            // Create the elements
            for (int x_idx = 0; x_idx < num_elements_length; x_idx++) {
                for (int y_idx = 0; y_idx < num_elements_width; y_idx++) {
                    // Adjacent nodes
                    unsigned int node0 = m_starting_node_index + y_idx + x_idx * N_y;
                    unsigned int node1 = m_starting_node_index + y_idx + (x_idx + 1) * N_y;
                    unsigned int node2 = m_starting_node_index + (y_idx + 1) + (x_idx + 1) * N_y;
                    unsigned int node3 = m_starting_node_index + (y_idx + 1) + x_idx * N_y;

                    // Create the element and set its nodes.
                    auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
                    element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_web_mesh->GetNode(node0)),
                                      std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_web_mesh->GetNode(node1)),
                                      std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_web_mesh->GetNode(node2)),
                                      std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_web_mesh->GetNode(node3)));

                    // Set element dimensions
                    element->SetDimensions(dx, dy);

                    // Add a single layers with a fiber angle of 0 degrees.
                    element->AddLayer(dz_rubber, m_angle_1, m_rubber_mat);
                    element->AddLayer(dz_steel, m_angle_2, m_steel_mat);
                    element->AddLayer(dz_rubber, m_angle_3, m_rubber_mat);

                    // Set other element properties
                    element->SetAlphaDamp(m_alpha);

                    // Add element to mesh
                    m_web_mesh->AddElement(element);
                }
            }

            break;
        }
        case ElementType::ANCF_8: {
            int N_x_edge = 2 * num_elements_length + 1;
            int N_y_edge = 2 * num_elements_width + 1;
            int N_y_mid = num_elements_width + 1;

            double dx = GetWebLength() / (2 * num_elements_length);
            double dy = GetBeltWidth() / (2 * num_elements_width);

            double dz_steel = GetSteelLayerThickness();
            double dz_rubber = (GetWebThickness() - dz_steel) / 2;

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
                    auto node_curv = ChVector3d(0.0, 0.0, 0.0);
                    // Create the node
                    auto node = chrono_types::make_shared<ChNodeFEAxyzDD>(node_loc, node_dir, node_curv);
                    // No additional mass assigned to nodes
                    node->SetMass(0);
                    // Add node to mesh
                    m_web_mesh->AddNode(node);
                }
            }

            // Create the elements
            for (int x_idx = 0; x_idx < num_elements_length; x_idx++) {
                for (int y_idx = 0; y_idx < num_elements_width; y_idx++) {
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
                    unsigned int node6 =
                        m_starting_node_index + 1 * y_idx + x_idx * (N_y_edge + N_y_mid) + N_y_edge + 1;
                    unsigned int node7 = m_starting_node_index + 2 * y_idx + x_idx * (N_y_edge + N_y_mid) + 1;

                    // Create the element and set its nodes.
                    auto element = chrono_types::make_shared<ChElementShellANCF_3833>();
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
                    element->AddLayer(dz_rubber, m_angle_1, m_rubber_mat);
                    element->AddLayer(dz_steel, m_angle_2, m_steel_mat);
                    element->AddLayer(dz_rubber, m_angle_3, m_rubber_mat);

                    // Set other element properties
                    element->SetAlphaDamp(m_alpha);

                    // Add element to mesh
                    m_web_mesh->AddElement(element);
                }
            }

            break;
        }
    }  // end switch
}

// -----------------------------------------------------------------------------
void ChTrackShoeBandANCF::Initialize(std::shared_ptr<ChChassis> chassis,
                                     const std::vector<ChCoordsys<>>& component_pos) {
    // Check the number of provided locations and orientations.
    assert(component_pos.size() == 2);

    // Initialize at origin.
    ChTrackShoe::Initialize(chassis, VNULL, QUNIT);

    auto chassis_body = chassis->GetBody();

    // Overwrite absolute body locations and orientations.
    m_shoe->SetPos(chassis_body->TransformPointLocalToParent(component_pos[0].pos));
    m_shoe->SetRot(chassis_body->GetRot() * component_pos[0].rot);

    // Overwrite absolute node locations and orientations.

    auto rot = chassis_body->GetRot() * component_pos[1].rot;
    ChVector3d xdir = rot.GetAxisX();
    ChVector3d ydir = rot.GetAxisY();
    ChVector3d zdir = rot.GetAxisZ();

    ChVector3d seg_loc = chassis_body->TransformPointLocalToParent(component_pos[1].pos) -
                         (0.5 * GetWebLength()) * xdir -
                         (0.5 * GetBeltWidth()) * ydir;

    int num_elements_length = GetNumElementsLength();
    int num_elements_width = GetNumElementsWidth();

    switch (m_element_type) {
        case ElementType::ANCF_4: {
            int N_x = num_elements_length + 1;
            int N_y = num_elements_width + 1;

            double dx = GetWebLength() / num_elements_length;
            double dy = GetBeltWidth() / num_elements_width;

            // Move the nodes on the mesh to the correct location
            for (int x_idx = 0; x_idx < N_x; x_idx++) {
                for (int y_idx = 0; y_idx < N_y; y_idx++) {
                    // Node location
                    auto node_loc = seg_loc + x_idx * dx * xdir + y_idx * dy * ydir;
                    // Node direction
                    auto node_dir = zdir;
                    // Get the node
                    int node_idx = m_starting_node_index + y_idx + x_idx * N_y;
                    auto node = std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_web_mesh->GetNode(node_idx));
                    // Overwrite node position and direction
                    node->SetPos(node_loc);
                    node->SetSlope1(node_dir);
                }
            }

            break;
        }
        case ElementType::ANCF_8: {
            int N_x_edge = 2 * num_elements_length + 1;
            int N_y_edge = 2 * num_elements_width + 1;

            double dx = GetWebLength() / (2 * num_elements_length);
            double dy = GetBeltWidth() / (2 * num_elements_width);

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
                    // Get the node
                    auto node = std::dynamic_pointer_cast<ChNodeFEAxyzDD>(m_web_mesh->GetNode(node_idx));
                    // Overwrite node position and direction
                    node->SetPos(node_loc);
                    node->SetSlope1(node_dir);
                    node_idx++;
                }
            }

            break;
        }
    }  // end switch
}

void ChTrackShoeBandANCF::InitializeInertiaProperties() {
    m_mass = GetTreadMass() + GetWebMass();
}

void ChTrackShoeBandANCF::UpdateInertiaProperties() {
    m_xform = m_shoe->GetFrameRefToAbs();

    // Calculate web mesh inertia properties
    double mesh_mass;
    ChVector3d mesh_com;
    ChMatrix33<> mesh_inertia;
    m_web_mesh->ComputeMassProperties(mesh_mass, mesh_com, mesh_inertia);

    // Calculate COM and inertia expressed in global frame
    CompositeInertia composite;
    composite.AddComponent(m_shoe->GetFrameCOMToAbs(), m_shoe->GetMass(), m_shoe->GetInertia());
    composite.AddComponent(ChFrame<>(mesh_com, QUNIT), mesh_mass, mesh_inertia);

    // Express COM and inertia in subsystem reference frame
    m_com.SetPos(m_xform.TransformPointParentToLocal(composite.GetCOM()));
    m_com.SetRot(QUNIT);

    m_inertia = m_xform.GetRotMat().transpose() * composite.GetInertia() * m_xform.GetRotMat();
}

// -----------------------------------------------------------------------------
void ChTrackShoeBandANCF::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    AddShoeVisualization();
}

void ChTrackShoeBandANCF::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_shoe);
}

// -----------------------------------------------------------------------------
void ChTrackShoeBandANCF::Connect(std::shared_ptr<ChTrackShoe> next,
                                  ChTrackAssembly* assembly,
                                  ChChassis* chassis,
                                  bool ccw) {
    ChSystem* system = m_shoe->GetSystem();
    ChQuaternion<> rot_cur_shoe = m_shoe->GetRot();
    ChQuaternion<> rot_next_shoe = next->GetShoeBody()->GetRot();

    int num_elements_length = GetNumElementsLength();
    int num_elements_width = GetNumElementsWidth();

    switch (m_element_type) {
        case ElementType::ANCF_4: {
            int N_y = num_elements_width + 1;

            // Change the gradient on the web boundary nodes that will connect to the current shoe body
            // and then connect those web nodes to the show tread body
            for (int y_idx = 0; y_idx < N_y; y_idx++) {
                int node_idx = m_starting_node_index + y_idx + 0 * N_y;
                auto node = std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_web_mesh->GetNode(node_idx));

                node->SetSlope1(rot_cur_shoe.GetAxisZ());

                auto constraintxyz = chrono_types::make_shared<ChLinkNodeFrame>();
                constraintxyz->Initialize(node, m_shoe);
                system->Add(constraintxyz);
                m_connections.push_back(constraintxyz);

                auto constraintD = chrono_types::make_shared<ChLinkNodeSlopeFrame>();
                constraintD->Initialize(node, m_shoe);
                system->Add(constraintD);
                m_connections.push_back(constraintD);
            }

            // Change the gradient on the boundary nodes that will connect to the second fixed body
            // and then connect those nodes to the body
            for (int y_idx = 0; y_idx < N_y; y_idx++) {
                int node_idx = m_starting_node_index + y_idx + num_elements_length * N_y;
                auto node = std::dynamic_pointer_cast<ChNodeFEAxyzD>(m_web_mesh->GetNode(node_idx));

                node->SetSlope1(rot_next_shoe.GetAxisZ());

                auto constraintxyz = chrono_types::make_shared<ChLinkNodeFrame>();
                constraintxyz->Initialize(node, next->GetShoeBody());
                system->Add(constraintxyz);
                m_connections.push_back(constraintxyz);

                auto constraintD = chrono_types::make_shared<ChLinkNodeSlopeFrame>();
                constraintD->Initialize(node, next->GetShoeBody());
                system->Add(constraintD);
                m_connections.push_back(constraintD);
            }

            break;
        }
        case ElementType::ANCF_8: {
            int N_y_edge = 2 * num_elements_width + 1;
            int N_y_mid = num_elements_width + 1;

            ////double dx = GetWebLength() / (2 * num_elements_length);
            ////double dy = GetBeltWidth() / (2 * num_elements_width);

            // Change the gradient on the web boundary nodes that will connect to the current shoe body
            // and then connect those web nodes to the show tread body
            for (int y_idx = 0; y_idx < N_y_edge; y_idx++) {
                int node_idx = m_starting_node_index + y_idx;
                auto node = std::dynamic_pointer_cast<ChNodeFEAxyzDD>(m_web_mesh->GetNode(node_idx));

                node->SetSlope1(rot_cur_shoe.GetAxisZ());

                auto constraintxyz = chrono_types::make_shared<ChLinkNodeFrame>();
                constraintxyz->Initialize(node, m_shoe);
                system->Add(constraintxyz);
                m_connections.push_back(constraintxyz);

                auto constraintD = chrono_types::make_shared<ChLinkNodeSlopeFrame>();
                constraintD->Initialize(node, m_shoe);
                system->Add(constraintD);
                m_connections.push_back(constraintD);

                node->SetSlope2Fixed(m_constrain_curvature);
            }

            // Change the gradient on the boundary nodes that will connect to the second fixed body
            // and then connect those nodes to the body
            for (int y_idx = 0; y_idx < N_y_edge; y_idx++) {
                int node_idx = m_starting_node_index + y_idx + num_elements_length * (N_y_edge + N_y_mid);
                auto node = std::dynamic_pointer_cast<ChNodeFEAxyzDD>(m_web_mesh->GetNode(node_idx));

                node->SetSlope1(rot_next_shoe.GetAxisZ());

                auto constraintxyz = chrono_types::make_shared<ChLinkNodeFrame>();
                constraintxyz->Initialize(node, next->GetShoeBody());
                system->Add(constraintxyz);
                m_connections.push_back(constraintxyz);

                auto constraintD = chrono_types::make_shared<ChLinkNodeSlopeFrame>();
                constraintD->Initialize(node, next->GetShoeBody());
                system->Add(constraintD);
                m_connections.push_back(constraintD);

                node->SetSlope2Fixed(m_constrain_curvature);
            }

            break;
        }
    }  // end switch
}

ChVector3d ChTrackShoeBandANCF::GetTension() const {
    return m_connections[0]->GetReaction2().force;
}

// -----------------------------------------------------------------------------
void ChTrackShoeBandANCF::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_shoe);
    ExportBodyList(jsonDocument, bodies);
}

void ChTrackShoeBandANCF::Output(ChOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_shoe);
    database.WriteBodies(bodies);
}

}  // end namespace vehicle
}  // namespace chrono
