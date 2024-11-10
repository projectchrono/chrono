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
#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChLinkNodeSlopeFrame.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "FEAvisualization.h"

using namespace chrono;
using namespace chrono::fea;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.8));

    sys.SetNumThreads(std::min(4, ChOMP::GetNumProcs()), 0, 1);

    std::cout << "-----------------------------------------------------------\n";
    std::cout << "-----------------------------------------------------------\n";
    std::cout << "     ANCF Shell Elements demo with implicit integration    \n";
    std::cout << "-----------------------------------------------------------\n";

    // Create a mesh, that is a container for groups of elements and their referenced nodes.
    auto mesh = chrono_types::make_shared<ChMesh>();
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
            chrono_types::make_shared<ChNodeFEAxyzD>(ChVector3d(loc_x, loc_y, loc_z), ChVector3d(dir_x, dir_y, dir_z));

        node->SetMass(0);

        // Fix all nodes along the axis X=0
        if (i % (numDiv_x + 1) == 0)
            node->SetFixed(true);

        // Add node to mesh
        mesh->AddNode(node);
    }

    // Get a handle to the tip node.
    auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(TotalNumNodes - 1));

    // Create an orthotropic material.
    // All layers for all elements share the same material.
    double rho = 500;
    ChVector3d E(2.1e7, 2.1e7, 2.1e7);
    ChVector3d nu(0.3, 0.3, 0.3);
    ChVector3d G(8.0769231e6, 8.0769231e6, 8.0769231e6);
    auto mat = chrono_types::make_shared<ChMaterialShellANCF>(rho, E, nu, G);

    // Create the elements
    for (int i = 0; i < TotalNumElements; i++) {
        // Adjacent nodes
        int node0 = (i / numDiv_x) * N_x + i % numDiv_x;
        int node1 = (i / numDiv_x) * N_x + i % numDiv_x + 1;
        int node2 = (i / numDiv_x) * N_x + i % numDiv_x + 1 + N_x;
        int node3 = (i / numDiv_x) * N_x + i % numDiv_x + N_x;

        // Create the element and set its nodes.
        auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node3)));

        // Set element dimensions
        element->SetDimensions(dx, dy);

        // Add a single layers with a fiber angle of 0 degrees.
        element->AddLayer(dz, 0 * CH_DEG_TO_RAD, mat);

        // Set other element properties
        element->SetAlphaDamp(0.0);  // Structural damping for this element

        // Add element to mesh
        mesh->AddElement(element);
    }

    // Add the mesh to the system
    sys.Add(mesh);

    // -------------------------------------
    // Options for visualization in irrlicht
    // -------------------------------------

    auto visualizemeshA = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    visualizemeshA->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    visualizemeshA->SetColorscaleMinMax(0.0, 5.50);
    visualizemeshA->SetShrinkElements(true, 0.85);
    visualizemeshA->SetSmoothFaces(true);
    mesh->AddVisualShapeFEA(visualizemeshA);

    auto visualizemeshB = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    visualizemeshB->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    visualizemeshB->SetWireframe(true);
    visualizemeshB->SetDrawInUndeformedReference(true);
    mesh->AddVisualShapeFEA(visualizemeshB);

    auto visualizemeshC = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    visualizemeshC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    visualizemeshC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    visualizemeshC->SetSymbolsThickness(0.004);
    mesh->AddVisualShapeFEA(visualizemeshC);

    auto visualizemeshD = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    visualizemeshD->SetFEMglyphType(ChVisualShapeFEA::GlyphType::ELEM_TENS_STRAIN);
    visualizemeshD->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    visualizemeshD->SetSymbolsScale(1);
    visualizemeshD->SetColorscaleMinMax(-0.5, 5);
    visualizemeshD->SetZbufferHide(false);
    mesh->AddVisualShapeFEA(visualizemeshD);

    // Create the run-time visualization system
    auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Z, sys, "ANCF Shells 3423",
                                         ChVector3d(0.4, 0.3, 0.1), ChVector3d(0.0, 0.0, -0.1));

    // ----------------------------------
    // Perform a dynamic time integration
    // ----------------------------------

    // Set up solver
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(100);
    solver->SetTolerance(1e-10);
    solver->EnableDiagonalPreconditioner(true);

    // Set up integrator
    auto stepper = chrono_types::make_shared<ChTimestepperHHT>(&sys);
    sys.SetTimestepper(stepper);
    // Alternative way of changing the integrator:
    ////sys.SetTimestepperType(ChTimestepper::Type::HHT);
    ////auto stepper = std::static_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());

    stepper->SetAlpha(-0.2);
    stepper->SetMaxIters(5);
    stepper->SetAbsTolerances(1e-2);
    stepper->SetStepControl(true);
    stepper->SetMinStepSize(1e-4);
    ////stepper->SetVerbose(true);

    // Simulation loop

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(0.01);
    }

    return 0;
}
