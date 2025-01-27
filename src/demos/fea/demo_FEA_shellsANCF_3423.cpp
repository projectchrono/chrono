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

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChMesh.h"

#include "demos/SetChronoSolver.h"
#include "demos/fea/FEAvisualization.h"

using namespace chrono;
using namespace chrono::fea;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create Chrono system
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.8));

    // Solver and integrator settings
    sys.SetNumThreads(std::min(4, ChOMP::GetNumProcs()), 0, 1);

    double step_size = 1e-2;
    auto solver_type = ChSolver::Type::MINRES;
    auto integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    SetChronoSolver(sys, solver_type, integrator_type);

    if (auto ls_it = std::dynamic_pointer_cast<ChIterativeSolverLS>(sys.GetSolver())) {
        ls_it->SetMaxIterations(100);
        ls_it->SetTolerance(1e-10);
        ls_it->EnableDiagonalPreconditioner(true);
    }

    if (auto hht = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper())) {
        hht->SetAlpha(-0.2);
        hht->SetMaxIters(5);
        hht->SetAbsTolerances(1e-2);
        hht->SetStepControl(true);
        hht->SetMinStepSize(1e-4);
        ////hht->SetVerbose(true);
    }

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

    // Get a handle to the tip node
    auto nodetip = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(TotalNumNodes - 1));

    // Create an orthotropic material; all layers for all elements share the same material
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

        // Create the element and set its nodes
        auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
        element->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node0)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node1)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node2)),
                          std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node3)));

        // Set element dimensions
        element->SetDimensions(dx, dy);

        // Add a single layers with a fiber angle of 0 degrees
        element->AddLayer(dz, 0 * CH_DEG_TO_RAD, mat);

        // Set other element properties
        element->SetAlphaDamp(0.0);  // Structural damping for this element

        // Add element to mesh
        mesh->AddElement(element);
    }

    // Add the mesh to the system
    sys.Add(mesh);

    // Options for FEA visualization

    auto shapeA = chrono_types::make_shared<ChVisualShapeFEA>();
    shapeA->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    shapeA->SetColorscaleMinMax(0.0, 5.50);
    shapeA->SetShrinkElements(true, 0.85);
    shapeA->SetSmoothFaces(true);
    mesh->AddVisualShapeFEA(shapeA);

    auto shapeB = chrono_types::make_shared<ChVisualShapeFEA>();
    shapeB->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    shapeB->SetWireframe(true);
    shapeB->SetDrawInUndeformedReference(true);
    mesh->AddVisualShapeFEA(shapeB);

    auto shapeC = chrono_types::make_shared<ChVisualShapeFEA>();
    shapeC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    shapeC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    shapeC->SetSymbolsThickness(0.004);
    mesh->AddVisualShapeFEA(shapeC);

    auto shapeD = chrono_types::make_shared<ChVisualShapeFEA>();
    shapeD->SetFEMglyphType(ChVisualShapeFEA::GlyphType::ELEM_TENS_STRAIN);
    shapeD->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    shapeD->SetSymbolsScale(1);
    shapeD->SetColorscaleMinMax(-0.5, 5);
    shapeD->SetZbufferHide(false);
    mesh->AddVisualShapeFEA(shapeD);

    // Create the run-time visualization system
    auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Z, sys, "ANCF Shells 3423",
                                         ChVector3d(0.4, 0.3, 0.1), ChVector3d(0.0, 0.0, -0.1));

    // Simulation loop
    ChRealtimeStepTimer realtime_timer;
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(step_size);
        realtime_timer.Spin(step_size);
    }

    return 0;
}
