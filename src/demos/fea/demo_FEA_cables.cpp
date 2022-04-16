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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// FEA for 3D beams of 'cable' type (ANCF gradient-deficient beams)
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/timestepper/ChTimestepper.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "FEAcables.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

using namespace irr;

// Select solver type (SPARSE_QR, SPARSE_LU, or MINRES).
ChSolver::Type solver_type = ChSolver::Type::SPARSE_QR;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono::Engine physical system
    ChSystemSMC sys;

    // Create a mesh, that is a container for groups of elements and
    // their referenced nodes.
    auto mesh = chrono_types::make_shared<ChMesh>();

    // Create one of the available models (defined in FEAcables.h)
    ////auto model = Model1(sys, mesh);
    ////auto model = Model2(sys, mesh);
    auto model = Model3(sys, mesh);

    // Remember to add the mesh to the system!
    sys.Add(mesh);

    // Visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChTriangleMeshShape asset that is internally managed) by
    // setting  proper coordinates and vertex colors as in the FEM elements. Such triangle mesh can be rendered by
    // Irrlicht or POVray or whatever postprocessor that can handle a colored ChTriangleMeshShape).
    auto vis_beam_A = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    vis_beam_A->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MZ);
    vis_beam_A->SetColorscaleMinMax(-0.4, 0.4);
    vis_beam_A->SetSmoothFaces(true);
    vis_beam_A->SetWireframe(false);
    mesh->AddVisualShapeFEA(vis_beam_A);

    auto vis_beam_B = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    vis_beam_B->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    vis_beam_B->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    vis_beam_B->SetSymbolsThickness(0.006);
    vis_beam_B->SetSymbolsScale(0.01);
    vis_beam_B->SetZbufferHide(false);
    mesh->AddVisualShapeFEA(vis_beam_B);

    // Set solver and solver settings
    switch (solver_type) {
        case ChSolver::Type::SPARSE_QR: {
            std::cout << "Using SparseQR solver" << std::endl;
            auto solver = chrono_types::make_shared<ChSolverSparseQR>();
            sys.SetSolver(solver);
            solver->UseSparsityPatternLearner(true);
            solver->LockSparsityPattern(true);
            solver->SetVerbose(false);
            break;
        }
        case ChSolver::Type::SPARSE_LU: {
            std::cout << "Using SparseLU solver" << std::endl;
            auto solver = chrono_types::make_shared<ChSolverSparseLU>();
            sys.SetSolver(solver);
            solver->UseSparsityPatternLearner(true);
            solver->LockSparsityPattern(true);
            solver->SetVerbose(false);
            break;
        }
        case ChSolver::Type::MINRES: {
            std::cout << "Using MINRES solver" << std::endl;
            auto solver = chrono_types::make_shared<ChSolverMINRES>();
            sys.SetSolver(solver);
            solver->SetMaxIterations(200);
            solver->SetTolerance(1e-10);
            solver->EnableDiagonalPreconditioner(true);
            solver->EnableWarmStart(true);  // IMPORTANT for convergence when using EULER_IMPLICIT_LINEARIZED
            solver->SetVerbose(false);
            break;
        }
        default: {
            std::cout << "Solver type not supported." << std::endl;
            break;
        }
    }

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Cables FEM");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector<>(0, 0.6, -1.0));
    sys.SetVisualSystem(vis);

    // Set integrator
    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    sys.SetSolverForceTolerance(1e-13);

    while (vis->Run()) {
        vis->BeginScene();
        vis->DrawAll();
        vis->EndScene();
        sys.DoStepDynamics(0.01);
    }

    return 0;
}
