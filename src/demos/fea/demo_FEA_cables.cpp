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
#include "chrono/input_output/ChWriterCSV.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "FEAvisualization.h"
#include "FEAcables.h"

using namespace chrono;
using namespace chrono::fea;

// Select run-time visualization
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Set integration step size
double step = 1e-3;

// Select solver type (SPARSE_QR, SPARSE_LU, or MINRES).
ChSolver::Type solver_type = ChSolver::Type::SPARSE_QR;

// Create output file with node positions and directions
bool output = false;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create a Chrono physical system
    ChSystemSMC sys;

    sys.SetNumThreads(std::min(4, ChOMP::GetNumProcs()), 0, 1);

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
    // This will automatically update a triangle mesh (a ChVisualShapeTriangleMesh asset that is internally managed) by
    // setting  proper coordinates and vertex colors as in the FEM elements. Such triangle mesh can be rendered by
    // Irrlicht or POVray or whatever postprocessor that can handle a colored ChVisualShapeTriangleMesh).

    ChColormap::Type colormap_type = ChColormap::Type::JET;
    ChVector2d colormap_range(-0.01, 0.01);

    auto vis_beam_A = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_beam_A->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MZ);
    vis_beam_A->SetColormap(colormap_type);
    vis_beam_A->SetColormapRange(colormap_range);
    vis_beam_A->SetSmoothFaces(true);
    vis_beam_A->SetWireframe(false);
    mesh->AddVisualShapeFEA(vis_beam_A);

    auto vis_beam_B = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_beam_B->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    vis_beam_B->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    vis_beam_B->SetSymbolsThickness(0.006);
    vis_beam_B->SetSymbolsScale(0.01);
    vis_beam_B->SetZbufferHide(false);
    mesh->AddVisualShapeFEA(vis_beam_B);

    // Create output directory
    std::string out_dir = GetChronoOutputPath() + "FEA_cables/";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    ChWriterCSV csv(" ");
    csv << mesh->GetNumNodes() << "\n" << std::endl;

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
            solver->SetTolerance(1e-14);
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

    // Create the run-time visualization system
    auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Y, sys, "Cables FEM",         //
                                         ChVector3d(-0.4, -0.15, -0.9), ChVector3d(0, -0.4, -0.3),  //
                                         true, "Mz (Nm)", colormap_range, colormap_type);

    // Set integrator
    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    while (vis->Run()) {
        if (output) {
            csv << sys.GetChTime() << std::endl;
            for (const auto& node : mesh->GetNodes()) {
                auto nodeD = std::dynamic_pointer_cast<ChNodeFEAxyzD>(node);
                if (!nodeD)
                    continue;
                csv << nodeD->GetPos() << "    " << nodeD->GetSlope1() << std::endl;
            }
            csv << std::endl;
        }

        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(step);
    }

    if (output) {
        csv.WriteToFile(out_dir + "/output.dat");
    }

    return 0;
}
