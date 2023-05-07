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
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChLinkDirFrame.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

using namespace irr;

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
        auto msection_cable = chrono_types::make_shared<ChBeamSectionCable>();
        msection_cable->SetDiameter(beam_diameter);
        msection_cable->SetYoungModulus(0.01e9);
        msection_cable->SetBeamRaleyghDamping(0.000);

        // Create the nodes
        auto hnodeancf1 = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(0, 0, -0.2), ChVector<>(1, 0, 0));
        auto hnodeancf2 = chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(beam_L, 0, -0.2), ChVector<>(1, 0, 0));

        mesh->AddNode(hnodeancf1);
        mesh->AddNode(hnodeancf2);

        // Create the element

        auto belementancf1 = chrono_types::make_shared<ChElementCableANCF>();

        belementancf1->SetNodes(hnodeancf1, hnodeancf2);
        belementancf1->SetSection(msection_cable);

        mesh->AddElement(belementancf1);

        // Apply a force or a torque to a node:
        hnodeancf2->SetForce(ChVector<>(0, 3, 0));

        hnodeancf1->SetFixed(true);

        // Add a rigid body connected to the end of the beam:

        body = chrono_types::make_shared<ChBodyEasyBox>(0.1, 0.02, 0.02, 1000);
        body->SetPos(hnodeancf2->GetPos() + ChVector<>(0.05, 0, 0));
        system.Add(body);

        auto constraint_pos = chrono_types::make_shared<ChLinkPointFrame>();
        constraint_pos->Initialize(hnodeancf2, body);
        system.Add(constraint_pos);

        auto constraint_dir = chrono_types::make_shared<ChLinkDirFrame>();
        constraint_dir->Initialize(hnodeancf2, body);
        constraint_dir->SetDirectionInAbsoluteCoords(ChVector<>(1, 0, 0));
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
        // Create a section, i.e. thickness and material properties
        // for beams. This will be shared among some beams.

        auto msection_cable2 = chrono_types::make_shared<ChBeamSectionCable>();
        msection_cable2->SetDiameter(0.015);
        msection_cable2->SetYoungModulus(0.01e9);
        msection_cable2->SetBeamRaleyghDamping(0.000);

        // This ChBuilderCableANCF helper object is very useful because it will
        // subdivide 'beams' into sequences of finite elements of beam type, ex.
        // one 'beam' could be made of 5 FEM elements of ChElementBeamANCF_3333 class.
        // If new nodes are needed, it will create them for you.
        ChBuilderCableANCF builder;

        // Now, simply use BuildBeam to create a beam from a point to another:
        builder.BuildBeam(mesh,                    // the mesh where to put the created nodes and elements
                          msection_cable2,         // the ChBeamSectionCable to use for the ChElementBeamANCF_3333 elements
                          10,                      // the number of ChElementBeamANCF_3333 to create
                          ChVector<>(0, 0, -0.1),  // the 'A' point in space (beginning of beam)
                          ChVector<>(0.5, 0, -0.1));  // the 'B' point in space (end of beam)

        // After having used BuildBeam(), you can retrieve the nodes used for the beam,
        // For example say you want to fix both pos and dir of A end and apply a force to the B end:
        // builder.GetLastBeamNodes().back()->SetFixed(true);
        builder.GetLastBeamNodes().front()->SetForce(ChVector<>(0, -0.2, 0));

        // For instance, now retrieve the A end and add a constraint to
        // block the position only of that node:
        auto mtruss = chrono_types::make_shared<ChBody>();
        mtruss->SetBodyFixed(true);

        auto constraint_hinge = chrono_types::make_shared<ChLinkPointFrame>();
        constraint_hinge->Initialize(builder.GetLastBeamNodes().back(), mtruss);
        system.Add(constraint_hinge);
    }
};

// ----------------------------------------------------------------------------
// Model3: A set of beam elements with connected bodies, each with different
//         number of ANCF cable elements.
// ----------------------------------------------------------------------------

class Model3 {
  public:
    Model3(ChSystem& system, std::shared_ptr<ChMesh> mesh, int n_chains = 6) : bodies(n_chains) {
        auto msection_cable2 = chrono_types::make_shared<ChBeamSectionCable>();
        msection_cable2->SetDiameter(0.015);
        msection_cable2->SetYoungModulus(0.01e9);
        msection_cable2->SetBeamRaleyghDamping(0.000);

        auto mtruss = chrono_types::make_shared<ChBody>();
        mtruss->SetBodyFixed(true);

        for (int j = 0; j < n_chains; ++j) {
            ChBuilderCableANCF builder;

            // Now, simply use BuildBeam to create a beam from a point to another:
            builder.BuildBeam(mesh,             // the mesh where to put the created nodes and elements
                              msection_cable2,  // ChBeamSectionCable to use for the ChElementBeamANCF_3333 elements
                              1 + j,            // number of ChElementBeamANCF_3333 to create
                              ChVector<>(0, 0, -0.1 * j),             // point A (beginning of beam)
                              ChVector<>(0.1 + 0.1 * j, 0, -0.1 * j)  // point B (end of beam)
            );

            builder.GetLastBeamNodes().back()->SetForce(ChVector<>(0, -0.2, 0));

            auto constraint_hinge = chrono_types::make_shared<ChLinkPointFrame>();
            constraint_hinge->Initialize(builder.GetLastBeamNodes().front(), mtruss);
            system.Add(constraint_hinge);

            auto msphere = chrono_types::make_shared<ChSphereShape>(0.02);
            constraint_hinge->AddVisualShape(msphere);

            // make a box and connect it
            auto mbox = chrono_types::make_shared<ChBodyEasyBox>(0.2, 0.04, 0.04, 1000);
            mbox->SetPos(builder.GetLastBeamNodes().back()->GetPos() + ChVector<>(0.1, 0, 0));
            system.Add(mbox);

            auto constraint_pos = chrono_types::make_shared<ChLinkPointFrame>();
            constraint_pos->Initialize(builder.GetLastBeamNodes().back(), mbox);
            system.Add(constraint_pos);

            auto constraint_dir = chrono_types::make_shared<ChLinkDirFrame>();
            constraint_dir->Initialize(builder.GetLastBeamNodes().back(), mbox);
            constraint_dir->SetDirectionInAbsoluteCoords(ChVector<>(1, 0, 0));
            system.Add(constraint_dir);

            // make another beam
            builder.BuildBeam(
                mesh,                // mesh where to put the created nodes and elements
                msection_cable2,     // ChBeamSectionCable to use for the ChElementBeamANCF_3333 elements
                1 + (n_chains - j),  // number of ChElementBeamANCF_3333 to create
                ChVector<>(mbox->GetPos().x() + 0.1, 0, -0.1 * j),                        // point A (beginning of beam)
                ChVector<>(mbox->GetPos().x() + 0.1 + 0.1 * (n_chains - j), 0, -0.1 * j)  // point B (end of beam)
            );

            auto constraint_pos2 = chrono_types::make_shared<ChLinkPointFrame>();
            constraint_pos2->Initialize(builder.GetLastBeamNodes().front(), mbox);
            system.Add(constraint_pos2);

            auto constraint_dir2 = chrono_types::make_shared<ChLinkDirFrame>();
            constraint_dir2->Initialize(builder.GetLastBeamNodes().front(), mbox);
            constraint_dir2->SetDirectionInAbsoluteCoords(ChVector<>(1, 0, 0));
            system.Add(constraint_dir2);

            // make a box and connect it
            bodies[j] = chrono_types::make_shared<ChBodyEasyBox>(0.2, 0.04, 0.04, 1000);
            bodies[j]->SetPos(builder.GetLastBeamNodes().back()->GetPos() + ChVector<>(0.1, 0, 0));
            system.Add(bodies[j]);

            auto constraint_pos3 = chrono_types::make_shared<ChLinkPointFrame>();
            constraint_pos3->Initialize(builder.GetLastBeamNodes().back(), bodies[j]);
            system.Add(constraint_pos3);

            auto constraint_dir3 = chrono_types::make_shared<ChLinkDirFrame>();
            constraint_dir3->Initialize(builder.GetLastBeamNodes().back(), bodies[j]);
            constraint_dir3->SetDirectionInAbsoluteCoords(ChVector<>(1, 0, 0));
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
