//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

//
//  Models using the ANCF gradient-deficient cable element
//

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_fea/ChElementBeamANCF.h"
#include "chrono_fea/ChBuilderBeam.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChLinkDirFrame.h"

using namespace chrono;
using namespace fea;
using namespace irr;

// ----------------------------------------------------------------------------
// Model1: A beam composed of a single ANCF cable element, with one end fixed
//         and a constant force applied at the other node. A rigid body is
//         connected to node2.
// ----------------------------------------------------------------------------
void model1(ChSystem& system, ChSharedPtr<ChMesh> mesh) {
    double beam_L = 0.1;
    double beam_diameter = 0.015;

    // Create a section, i.e. thickness and material properties
    // for beams. This will be shared among some beams.

    ChSharedPtr<ChBeamSectionCable> msection_cable(new ChBeamSectionCable);
    msection_cable->SetDiameter(beam_diameter);
    msection_cable->SetYoungModulus(0.01e9);
    msection_cable->SetBeamRaleyghDamping(0.000);

    // Create the nodes

    ChSharedPtr<ChNodeFEAxyzD> hnodeancf1(new ChNodeFEAxyzD(ChVector<>(0, 0, -0.2), ChVector<>(1, 0, 0)));
    ChSharedPtr<ChNodeFEAxyzD> hnodeancf2(new ChNodeFEAxyzD(ChVector<>(beam_L, 0, -0.2), ChVector<>(1, 0, 0)));

    mesh->AddNode(hnodeancf1);
    mesh->AddNode(hnodeancf2);

    // Create the element

    ChSharedPtr<ChElementBeamANCF> belementancf1(new ChElementBeamANCF);

    belementancf1->SetNodes(hnodeancf1, hnodeancf2);
    belementancf1->SetSection(msection_cable);

    mesh->AddElement(belementancf1);

    // Apply a force or a torque to a node:
    hnodeancf2->SetForce(ChVector<>(0, 3, 0));

    hnodeancf1->SetFixed(true);

    // Add a rigid body connected to the end of the beam:

    ChSharedPtr<ChBodyEasyBox> mbox(new ChBodyEasyBox(0.1, 0.02, 0.02, 1000));
    mbox->SetPos(hnodeancf2->GetPos() + ChVector<>(0.05, 0, 0));
    system.Add(mbox);

    ChSharedPtr<ChLinkPointFrame> constraint_pos(new ChLinkPointFrame);
    constraint_pos->Initialize(hnodeancf2, mbox);
    system.Add(constraint_pos);

    ChSharedPtr<ChLinkDirFrame> constraint_dir(new ChLinkDirFrame);
    constraint_dir->Initialize(hnodeancf2, mbox);
    constraint_dir->SetDirectionInAbsoluteCoords(ChVector<>(1, 0, 0));
    system.Add(constraint_dir);
}

// ----------------------------------------------------------------------------
// Model2: A beam composed of 10 ANCF cable element, with one end hinged to
//         ground, moving under gravity alone.
// This model demonstrates the use of the utility class ChBuilderBeamANCF.
// ----------------------------------------------------------------------------
void model2(ChSystem& system, ChSharedPtr<ChMesh> mesh) {
    // Create a section, i.e. thickness and material properties
    // for beams. This will be shared among some beams.

    ChSharedPtr<ChBeamSectionCable> msection_cable2(new ChBeamSectionCable);
    msection_cable2->SetDiameter(0.015);
    msection_cable2->SetYoungModulus(0.01e9);
    msection_cable2->SetBeamRaleyghDamping(0.000);

    // Shortcut!
    // This ChBuilderBeamANCF helper object is very useful because it will
    // subdivide 'beams' into sequences of finite elements of beam type, ex.
    // one 'beam' could be made of 5 FEM elements of ChElementBeamANCF class.
    // If new nodes are needed, it will create them for you.
    ChBuilderBeamANCF builder;

    // Now, simply use BuildBeam to create a beam from a point to another:
    builder.BuildBeam(mesh,                       // the mesh where to put the created nodes and elements
                      msection_cable2,            // the ChBeamSectionCable to use for the ChElementBeamANCF elements
                      10,                         // the number of ChElementBeamANCF to create
                      ChVector<>(0, 0, -0.1),     // the 'A' point in space (beginning of beam)
                      ChVector<>(0.5, 0, -0.1));  // the 'B' point in space (end of beam)

    // After having used BuildBeam(), you can retrieve the nodes used for the beam,
    // For example say you want to fix both pos and dir of A end and apply a force to the B end:
    // builder.GetLastBeamNodes().back()->SetFixed(true);
    builder.GetLastBeamNodes().front()->SetForce(ChVector<>(0, -0.2, 0));

    // For instance, now retrieve the A end and add a constraint to
    // block the position only of that node:
    ChSharedPtr<ChBody> mtruss(new ChBody);
    mtruss->SetBodyFixed(true);

    ChSharedPtr<ChLinkPointFrame> constraint_hinge(new ChLinkPointFrame);
    constraint_hinge->Initialize(builder.GetLastBeamNodes().back(), mtruss);
    system.Add(constraint_hinge);
}

// ----------------------------------------------------------------------------
// Model3: A set of beam elements with connected bodies, each with different
//         number of ANCF cable elements.
// ----------------------------------------------------------------------------
void model3(ChSystem& system, ChSharedPtr<ChMesh> mesh) {
  // TEST 3
  // Add ANCF CABLE BEAMS making chains with bodies
  //

  ChSharedPtr<ChBeamSectionCable> msection_cable2(new ChBeamSectionCable);
  msection_cable2->SetDiameter(0.015);
  msection_cable2->SetYoungModulus(0.01e9);
  msection_cable2->SetBeamRaleyghDamping(0.000);

  ChSharedPtr<ChBody> mtruss(new ChBody);
  mtruss->SetBodyFixed(true);

  for (int j = 0; j < 6; ++j) {
    ChBuilderBeamANCF builder;

    // Now, simply use BuildBeam to create a beam from a point to another:
    builder.BuildBeam(mesh,                        // the mesh where to put the created nodes and elements
      msection_cable2,             // ChBeamSectionCable to use for the ChElementBeamANCF elements
      1 + j,                       // number of ChElementBeamANCF to create
      ChVector<>(0, 0, -0.1 * j),  // point A (beginning of beam)
      ChVector<>(0.1 + 0.1 * j, 0, -0.1 * j)  // point B (end of beam)
      );

    builder.GetLastBeamNodes().back()->SetForce(ChVector<>(0, -0.2, 0));

    ChSharedPtr<ChLinkPointFrame> constraint_hinge(new ChLinkPointFrame);
    constraint_hinge->Initialize(builder.GetLastBeamNodes().front(), mtruss);
    system.Add(constraint_hinge);

    ChSharedPtr<ChSphereShape> msphere(new ChSphereShape);
    msphere->GetSphereGeometry().rad = 0.02;
    constraint_hinge->AddAsset(msphere);

    // make a box and connect it
    ChSharedPtr<ChBodyEasyBox> mbox(new ChBodyEasyBox(0.2, 0.04, 0.04, 1000));
    mbox->SetPos(builder.GetLastBeamNodes().back()->GetPos() + ChVector<>(0.1, 0, 0));
    system.Add(mbox);

    ChSharedPtr<ChLinkPointFrame> constraint_pos(new ChLinkPointFrame);
    constraint_pos->Initialize(builder.GetLastBeamNodes().back(), mbox);
    system.Add(constraint_pos);

    ChSharedPtr<ChLinkDirFrame> constraint_dir(new ChLinkDirFrame);
    constraint_dir->Initialize(builder.GetLastBeamNodes().back(), mbox);
    constraint_dir->SetDirectionInAbsoluteCoords(ChVector<>(1, 0, 0));
    system.Add(constraint_dir);

    // make another beam
    builder.BuildBeam(mesh,             // mesh where to put the created nodes and elements
      msection_cable2,  // ChBeamSectionCable to use for the ChElementBeamANCF elements
      1 + (6 - j),      // number of ChElementBeamANCF to create
      ChVector<>(mbox->GetPos().x + 0.1, 0, -0.1 * j),  // point A (beginning of beam)
      ChVector<>(mbox->GetPos().x + 0.1 + 0.1 * (6 - j), 0, -0.1 * j)  // point B (end of beam)
      );

    ChSharedPtr<ChLinkPointFrame> constraint_pos2(new ChLinkPointFrame);
    constraint_pos2->Initialize(builder.GetLastBeamNodes().front(), mbox);
    system.Add(constraint_pos2);

    ChSharedPtr<ChLinkDirFrame> constraint_dir2(new ChLinkDirFrame);
    constraint_dir2->Initialize(builder.GetLastBeamNodes().front(), mbox);
    constraint_dir2->SetDirectionInAbsoluteCoords(ChVector<>(1, 0, 0));
    system.Add(constraint_dir2);

    // make a box and connect it
    ChSharedPtr<ChBodyEasyBox> mbox2(new ChBodyEasyBox(0.2, 0.04, 0.04, 1000));
    mbox2->SetPos(builder.GetLastBeamNodes().back()->GetPos() + ChVector<>(0.1, 0, 0));
    system.Add(mbox2);

    ChSharedPtr<ChLinkPointFrame> constraint_pos3(new ChLinkPointFrame);
    constraint_pos3->Initialize(builder.GetLastBeamNodes().back(), mbox2);
    system.Add(constraint_pos3);

    ChSharedPtr<ChLinkDirFrame> constraint_dir3(new ChLinkDirFrame);
    constraint_dir3->Initialize(builder.GetLastBeamNodes().back(), mbox2);
    constraint_dir3->SetDirectionInAbsoluteCoords(ChVector<>(1, 0, 0));
    system.Add(constraint_dir3);
  }
}
