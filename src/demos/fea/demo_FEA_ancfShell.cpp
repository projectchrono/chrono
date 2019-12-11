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
// Authors: Milad Rakhsha, Radu Serban
// =============================================================================
//
// Demo on using ANCF shell elements
//
// =============================================================================

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/fea/ChElementShellANCF.h"
#include "chrono/fea/ChLinkDirFrame.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

using namespace irr;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    double time_step = 1e-3;

    ChSystemSMC my_system;
    my_system.Set_G_acc(ChVector<>(0, 0, -9.8));

    // Create the Irrlicht visualization (open the Irrlicht device, bind a simple user interface, etc.)
    ChIrrApp application(&my_system, L"ANCF Shells", core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(-0.4f, -0.3f, 0.0f),  // camera location
                                 core::vector3df(0.0f, 0.5f, -0.1f));  // "look at" location

    GetLog() << "-----------------------------------------------------------\n";
    GetLog() << "-----------------------------------------------------------\n";
    GetLog() << "     ANCF Shell Elements demo with implicit integration    \n";
    GetLog() << "-----------------------------------------------------------\n";

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();
    int numFlexBody = 1;
    // Geometry of the plate
    double plate_lenght_x = 1;
    double plate_lenght_y = 0.1;
    double plate_lenght_z = 0.01;
    // Specification of the mesh
    int numDiv_x = 10;
    int numDiv_y = 4;
    int N_x = numDiv_x + 1;
    int N_y = numDiv_y + 1;
    // Number of elements in the z direction is considered as 1
    int TotalNumElements = numDiv_x * numDiv_y;
    int TotalNumNodes = N_x * N_y;
    // For uniform mesh
    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z;

    // Create and add the nodes
    for (int i = 0; i < TotalNumNodes; i++) {
        // Node location
        double loc_x = (i % N_x) * dx;
        double loc_y = (i / N_x) % N_y * dy;
        double loc_z = 0;

        // Node direction
        double dir_x = 0;
        double dir_y = 0;
        double dir_z = 1;

        // Create the node
        auto node =
            chrono_types::make_shared<ChNodeFEAxyzD>(ChVector<>(loc_x, loc_y, loc_z), ChVector<>(dir_x, dir_y, dir_z));

        node->SetMass(0);

        // Fix all nodes along the axis X=0
        if (i % (numDiv_x + 1) == 0)
            node->SetFixed(true);

        // Add node to mesh
        my_mesh->AddNode(node);
    }

    // Get a handle to the tip node.
    auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(TotalNumNodes - 1));

    // Create an orthotropic material.
    // All layers for all elements share the same material.
    double rho = 500;
    ChVector<> E(2.1e7, 2.1e7, 2.1e7);
    ChVector<> nu(0.3, 0.3, 0.3);
    ChVector<> G(8.0769231e6, 8.0769231e6, 8.0769231e6);
    auto mat = chrono_types::make_shared<ChMaterialShellANCF>(rho, E, nu, G);

    // Create the elements
    for (int i = 0; i < TotalNumElements; i++) {
        // Adjacent nodes
        int node0 = (i / numDiv_x) * N_x + i % numDiv_x;
        int node1 = (i / numDiv_x) * N_x + i % numDiv_x + 1;
        int node2 = (i / numDiv_x) * N_x + i % numDiv_x + 1 + N_x;
        int node3 = (i / numDiv_x) * N_x + i % numDiv_x + N_x;

        // Create the element and set its nodes.
        auto element = chrono_types::make_shared<ChElementShellANCF>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(my_mesh->GetNode(node3)));

        // Set element dimensions
        element->SetDimensions(dx, dy);

        // Add a single layers with a fiber angle of 0 degrees.
        element->AddLayer(dz, 0 * CH_C_DEG_TO_RAD, mat);

        // Set other element properties
        element->SetAlphaDamp(0.0);    // Structural damping for this element
        element->SetGravityOn(false);  // turn internal gravitational force calculation off

        // Add element to mesh
        my_mesh->AddElement(element);
    }

    // Add the mesh to the system
    my_system.Add(my_mesh);

    // -------------------------------------
    // Options for visualization in irrlicht
    // -------------------------------------

    auto visualizemeshA = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    visualizemeshA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    visualizemeshA->SetColorscaleMinMax(0.0, 5.50);
    visualizemeshA->SetShrinkElements(true, 0.85);
    visualizemeshA->SetSmoothFaces(true);
    my_mesh->AddAsset(visualizemeshA);

    auto visualizemeshB = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    visualizemeshB->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    visualizemeshB->SetWireframe(true);
    visualizemeshB->SetDrawInUndeformedReference(true);
    my_mesh->AddAsset(visualizemeshB);

    auto visualizemeshC = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    visualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    visualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    visualizemeshC->SetSymbolsThickness(0.004);
    my_mesh->AddAsset(visualizemeshC);

    auto visualizemeshD = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    visualizemeshD->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_ELEM_TENS_STRAIN);
    visualizemeshD->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    visualizemeshD->SetSymbolsScale(1);
    visualizemeshD->SetColorscaleMinMax(-0.5, 5);
    visualizemeshD->SetZbufferHide(false);
    my_mesh->AddAsset(visualizemeshD);

    application.AssetBindAll();
    application.AssetUpdateAll();

    // ----------------------------------
    // Perform a dynamic time integration
    // ----------------------------------

    // Set up solver
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    my_system.SetSolver(solver);
    solver->SetMaxIterations(100);
    solver->SetTolerance(1e-10);
    solver->EnableDiagonalPreconditioner(true);

    // Set up integrator
    auto stepper = chrono_types::make_shared<ChTimestepperHHT>(&my_system);
    my_system.SetTimestepper(stepper);
    // Alternative way of changing the integrator:
    ////my_system.SetTimestepperType(ChTimestepper::Type::HHT);
    ////auto stepper = std::static_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());

    stepper->SetAlpha(-0.2);
    stepper->SetMaxiters(5);
    stepper->SetAbsTolerances(1e-5);
    stepper->SetMode(ChTimestepperHHT::POSITION);
    stepper->SetScaling(true);
    stepper->SetStepControl(true);
    stepper->SetMinStepSize(1e-4);
    ////stepper->SetVerbose(true);

    // Simulation loop

    application.SetTimestep(0.01);

    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        application.DoStep();
        application.EndScene();
    }

    return 0;
}
