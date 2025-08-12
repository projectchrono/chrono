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
//
//  Models using the ANCF gradient-deficient cable element
//
// =============================================================================

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono/fea/ChElementCableANCF.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/assets/ChVisualShapeFEA.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChLinkNodeSlopeFrame.h"

using namespace chrono;
using namespace chrono::fea;

// ----------------------------------------------------------------------------
// Model1: A beam composed of a single ANCF cable element, with one end fixed
//         and a constant force applied at the other node. A rigid body is
//         connected to node2.
// ----------------------------------------------------------------------------
class Model1 {
  public:
    Model1(ChSystem& system, std::shared_ptr<ChMesh> mesh) {
        double beam_L = 0.1;
        double beam_diameter = 0.015;

        // Create a section, i.e. thickness and material properties
        // for beams. This will be shared among some beams.
        auto section = chrono_types::make_shared<ChBeamSectionCable>();
        section->SetDiameter(beam_diameter);
        section->SetYoungModulus(0.01e9);
        section->SetRayleighDamping(0.000);

        // Create the nodes
        auto node1 = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector3d(0, 0, -0.2), ChVector3d(1, 0, 0));
        auto node2 = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector3d(beam_L, 0, -0.2), ChVector3d(1, 0, 0));
        mesh->AddNode(node1);
        mesh->AddNode(node2);

        // Create the element
        auto element1 = chrono_types::make_shared<ChElementCableANCF>();
        element1->SetNodes(node1, node2);
        element1->SetSection(section);
        mesh->AddElement(element1);

        // Fix node1 and apply a force to node2
        node1->SetFixed(true);
        node2->SetForce(ChVector3d(0, 3, 0));

        // Add a rigid body connected to the end of the beam
        body = chrono_types::make_shared<ChBodyEasyBox>(0.1, 0.02, 0.02, 1000);
        body->SetPos(node2->GetPos() + ChVector3d(0.05, 0, 0));
        system.Add(body);

        // Fix position and direction of node2 to the body
        auto constraint_pos = chrono_types::make_shared<ChLinkNodeFrame>();
        constraint_pos->Initialize(node2, body);
        system.Add(constraint_pos);

        auto constraint_dir = chrono_types::make_shared<ChLinkNodeSlopeFrame>();
        constraint_dir->Initialize(node2, body);
        constraint_dir->SetDirectionInAbsoluteCoords(ChVector3d(1, 0, 0));
        system.Add(constraint_dir);
    }

    // Print position of end body
    void PrintBodyPosition() {
        std::cout << "Time: " << body->GetChTime() << std::endl;
        std::cout << "  " << body->GetPos() << std::endl;
    }

  private:
    std::shared_ptr<ChBodyEasyBox> body;
};

// ----------------------------------------------------------------------------
// Model2: A beam composed of 10 ANCF beam element, with one end hinged to
//         ground, moving under gravity alone.
// This model demonstrates the use of the utility class ChBuilderCableANCF.
// ----------------------------------------------------------------------------
class Model2 {
  public:
    Model2(ChSystem& system, std::shared_ptr<ChMesh> mesh) {
        // Create ground body
        auto ground = chrono_types::make_shared<ChBody>();
        ground->SetFixed(true);

        // Create a section, i.e. thickness and material properties for beams.
        auto section = chrono_types::make_shared<ChBeamSectionCable>();
        section->SetDiameter(0.015);
        section->SetYoungModulus(0.01e9);
        section->SetRayleighDamping(0.000);

        // Use BuildBeam to create a beam from a point to another
        ChBuilderCableANCF builder;
        builder.BuildBeam(mesh,                       // container FEA mesh
                          section,                    // cable section
                          10,                         // number of cable elements
                          ChVector3d(0, 0, -0.1),     // point A (beginning of beam)
                          ChVector3d(0.5, 0, -0.1));  // point B (end of beam)

        // Apply a force on the first node of the beam
        builder.GetLastBeamNodes().front()->SetForce(ChVector3d(1.0, 0, 0));

        // Spherical or cantilever constraint at last node of the beam
        ////{
        ////    auto constraint_hinge = chrono_types::make_shared<ChLinkNodeFrame>();
        ////    constraint_hinge->Initialize(builder.GetLastBeamNodes().back(), ground);
        ////    system.Add(constraint_hinge);
        ////}
        {
            builder.GetLastBeamNodes().back()->SetFixed(true);
        }
    }
};

// ----------------------------------------------------------------------------
// Model3: A set of beam elements with connected bodies, each with different
//         number of ANCF cable elements.
// ----------------------------------------------------------------------------

class Model3 {
  public:
    Model3(ChSystem& system, std::shared_ptr<ChMesh> mesh, int n_chains = 6) : bodies(n_chains) {
        auto section = chrono_types::make_shared<ChBeamSectionCable>();
        section->SetDiameter(0.015);
        section->SetYoungModulus(0.01e9);
        section->SetRayleighDamping(0.000);

        auto ground = chrono_types::make_shared<ChBody>();
        ground->SetFixed(true);

        for (int j = 0; j < n_chains; ++j) {
            ChBuilderCableANCF builder;

            // Use BuildBeam to create a beam from a point to another
            builder.BuildBeam(mesh,                                   // container FEA mesh
                              section,                                // cable section
                              1 + j,                                  // number of cable elements
                              ChVector3d(0, 0, -0.1 * j),             // point A (beginning of beam)
                              ChVector3d(0.1 + 0.1 * j, 0, -0.1 * j)  // point B (end of beam)
            );

            builder.GetLastBeamNodes().back()->SetForce(ChVector3d(0, -0.2, 0));

            auto constraint_hinge = chrono_types::make_shared<ChLinkNodeFrame>();
            constraint_hinge->Initialize(builder.GetLastBeamNodes().front(), ground);
            system.Add(constraint_hinge);

            auto msphere = chrono_types::make_shared<ChVisualShapeSphere>(0.02);
            constraint_hinge->AddVisualShape(msphere);

            // Create and connect the intermediate body
            auto mid_body = chrono_types::make_shared<ChBodyEasyBox>(0.2, 0.04, 0.04, 1000);
            mid_body->SetPos(builder.GetLastBeamNodes().back()->GetPos() + ChVector3d(0.1, 0, 0));
            system.Add(mid_body);

            auto constraint_pos = chrono_types::make_shared<ChLinkNodeFrame>();
            constraint_pos->Initialize(builder.GetLastBeamNodes().back(), mid_body);
            system.Add(constraint_pos);

            auto constraint_dir = chrono_types::make_shared<ChLinkNodeSlopeFrame>();
            constraint_dir->Initialize(builder.GetLastBeamNodes().back(), mid_body);
            constraint_dir->SetDirectionInAbsoluteCoords(ChVector3d(1, 0, 0));
            system.Add(constraint_dir);

            // Create a second beam
            builder.BuildBeam(
                mesh,                                                   // container FEA mesh
                section,                                                // cable section
                1 + (n_chains - j),                                     // number of cable elements
                ChVector3d(mid_body->GetPos().x() + 0.1, 0, -0.1 * j),  // point A (beginning of beam)
                ChVector3d(mid_body->GetPos().x() + 0.1 + 0.1 * (n_chains - j), 0, -0.1 * j)  // point B (end of beam)
            );

            auto constraint_pos2 = chrono_types::make_shared<ChLinkNodeFrame>();
            constraint_pos2->Initialize(builder.GetLastBeamNodes().front(), mid_body);
            system.Add(constraint_pos2);

            auto constraint_dir2 = chrono_types::make_shared<ChLinkNodeSlopeFrame>();
            constraint_dir2->Initialize(builder.GetLastBeamNodes().front(), mid_body);
            constraint_dir2->SetDirectionInAbsoluteCoords(ChVector3d(1, 0, 0));
            system.Add(constraint_dir2);

            // Create and connect the end body
            bodies[j] = chrono_types::make_shared<ChBodyEasyBox>(0.2, 0.04, 0.04, 1000);
            bodies[j]->SetPos(builder.GetLastBeamNodes().back()->GetPos() + ChVector3d(0.1, 0, 0));
            system.Add(bodies[j]);

            auto constraint_pos3 = chrono_types::make_shared<ChLinkNodeFrame>();
            constraint_pos3->Initialize(builder.GetLastBeamNodes().back(), bodies[j]);
            system.Add(constraint_pos3);

            auto constraint_dir3 = chrono_types::make_shared<ChLinkNodeSlopeFrame>();
            constraint_dir3->Initialize(builder.GetLastBeamNodes().back(), bodies[j]);
            constraint_dir3->SetDirectionInAbsoluteCoords(ChVector3d(1, 0, 0));
            system.Add(constraint_dir3);
        }
    }

    // Print positions of end bodies in each chain
    void PrintBodyPositions() {
        std::cout << "Time: " << bodies[0]->GetChTime() << std::endl;
        for (auto body : bodies) {
            std::cout << "  " << body->GetPos() << std::endl;
        }
    }

  private:
    std::vector<std::shared_ptr<ChBodyEasyBox>> bodies;  // end bodies
};
