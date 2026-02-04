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
#include "chrono/fea/ChElementShellANCF_3443.h"
#include "chrono/fea/ChMesh.h"

#include "demos/SetChronoSolver.h"
#include "demos/fea/FEAvisualization.h"

using namespace chrono;
using namespace chrono::fea;

using std::cout;
using std::cerr;
using std::endl;

// -----------------------------------------------------------------------------

// Visualization system type
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// FEA shell element type
enum class ANCFShellElementType { ANCF_3423, ANCF_3443 };

// -----------------------------------------------------------------------------

void ConstructPlate(ANCFShellElementType type, ChSystem& sys, const ChVector3d& loc) {
    double plate_lenght_x = 1;
    double plate_lenght_y = 0.1;
    double plate_lenght_z = 0.01;

    int numDiv_x = 10;
    int numDiv_y = 4;
    int N_x = numDiv_x + 1;
    int N_y = numDiv_y + 1;

    int TotalNumElements = numDiv_x * numDiv_y;
    int TotalNumNodes = N_x * N_y;

    double dx = plate_lenght_x / numDiv_x;
    double dy = plate_lenght_y / numDiv_y;
    double dz = plate_lenght_z;

    // Creat the FEA mesh
    auto mesh = chrono_types::make_shared<ChMesh>();

    // Create and add the nodes
    ChVector3d n_dirX(1, 0, 0);
    ChVector3d n_dirY(0, 1, 0);
    ChVector3d n_dirZ(0, 0, 1);
    for (int i = 0; i < TotalNumNodes; i++) {
        double loc_x = (i % N_x) * dx;
        double loc_y = (i / N_x) % N_y * dy;
        double loc_z = 0;
        ChVector3d n_loc = loc + ChVector3d(loc_x, loc_y, loc_z);

        std::shared_ptr<ChNodeFEAxyz> node;
        switch (type) {
            case ANCFShellElementType::ANCF_3423:
                node = chrono_types::make_shared<ChNodeFEAxyzD>(n_loc, n_dirZ);
                break;
            case ANCFShellElementType::ANCF_3443:
                node = chrono_types::make_shared<ChNodeFEAxyzDDD>(n_loc, n_dirX, n_dirY, n_dirZ);
                break;
        }
        node->SetMass(0);

        // Fix all nodes along the axis X=0
        if (i % (numDiv_x + 1) == 0)
            node->SetFixed(true);

        mesh->AddNode(node);
    }

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

        std::shared_ptr<ChElementBase> element;
        switch (type) {
            case ANCFShellElementType::ANCF_3423: {
                auto e = chrono_types::make_shared<ChElementShellANCF_3423>();
                e->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node0)),
                            std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node1)),
                            std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node2)),
                            std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(node3)));

                e->SetDimensions(dx, dy);                 // element dimensions
                e->AddLayer(dz, 0 * CH_DEG_TO_RAD, mat);  // single layer with a fiber angle of 0 degrees
                e->SetAlphaDamp(0.0);                     // structural damping
                element = e;
                break;
            }
            case ANCFShellElementType::ANCF_3443: {
                auto e = chrono_types::make_shared<ChElementShellANCF_3443>();
                e->SetNodes(std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(mesh->GetNode(node0)),
                            std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(mesh->GetNode(node1)),
                            std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(mesh->GetNode(node2)),
                            std::dynamic_pointer_cast<ChNodeFEAxyzDDD>(mesh->GetNode(node3)));
                e->SetDimensions(dx, dy);                 // element dimensions
                e->AddLayer(dz, 0 * CH_DEG_TO_RAD, mat);  // single layer with a fiber angle of 0 degrees
                e->SetAlphaDamp(0.0);                     // structural damping
                element = e;
                break;
            }
        }

        mesh->AddElement(element);
    }

    sys.Add(mesh);

    // FEA mesh visualization
    auto shapeA = chrono_types::make_shared<ChVisualShapeFEA>();
    shapeA->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
    shapeA->SetColormapRange(0.0, 5.50);
    shapeA->SetShrinkElements(true, 0.85);
    shapeA->SetSmoothFaces(true);
    shapeA->SetShellResolution(4);
    mesh->AddVisualShapeFEA(shapeA);

    auto shapeB = chrono_types::make_shared<ChVisualShapeFEA>();
    shapeB->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
    shapeB->SetWireframe(true);
    shapeB->SetShellResolution(4);
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
    shapeD->SetColormapRange(-0.5, 5);
    shapeD->SetZbufferHide(false);
    mesh->AddVisualShapeFEA(shapeD);
}

// -----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create Chrono system
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.8));

    // Solver and integrator settings
    sys.SetNumThreads(std::min(4, ChOMP::GetNumProcs()), 0, 1);

    double step_size = 1e-3;
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

    // Create the FEA mesh and construct the two plates
    ConstructPlate(ANCFShellElementType::ANCF_3423, sys, ChVector3d(0, -0.15, 0));
    ConstructPlate(ANCFShellElementType::ANCF_3443, sys, ChVector3d(0, +0.05, 0));

    // Create the run-time visualization system
    auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Z, sys, "ANCF Shells 3423",
                                         ChVector3d(0.3, 0.3, 0.0), ChVector3d(0.0, -0.1, -0.2));

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
