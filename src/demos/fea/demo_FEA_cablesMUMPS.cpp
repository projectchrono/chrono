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
// Authors: Dario Mangoni, Radu Serban
// =============================================================================
//
// FEA for 3D beams of 'cable' type (ANCF gradient-deficient beams)
//       uses the Chrono MUMPS module
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono_mumps/ChSolverMumps.h"

#include "FEAvisualization.h"
#include "FEAcables.h"

using namespace chrono;
using namespace fea;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org" << std::endl
              << "Chrono version: " << CHRONO_VERSION << std::endl
              << std::endl;

    // Create a Chrono physical system
    ChSystemSMC sys;

    // Create a mesh, that is a container for groups of elements and
    // their referenced nodes.
    auto mesh = chrono_types::make_shared<ChMesh>();

    // Create the model (defined in FEAcables.h)
    auto model = Model3(sys, mesh);

    // Remember to add the mesh to the system!
    sys.Add(mesh);

    // Visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChVisualShapeTriangleMesh asset that is internally managed) by
    // setting  proper coordinates and vertex colors as in the FEM elements. Such triangle mesh can be rendered by
    // Irrlicht or POVray or whatever postprocessor that can handle a colored ChVisualShapeTriangleMesh).
    auto vis_beam_A = chrono_types::make_shared<ChVisualShapeFEA>();
    vis_beam_A->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MZ);
    vis_beam_A->SetColormapRange(-0.4, 0.4);
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

    // Configure MUMPS solver.
    // For this simple and relatively small problem, use of the sparsity pattern learner may introduce additional
    // overhead (if the sparsity pattern is not locked).
    auto mumps_solver = chrono_types::make_shared<ChSolverMumps>(1);
    mumps_solver->UseSparsityPatternLearner(true);
    mumps_solver->LockSparsityPattern(true);
    sys.SetSolver(mumps_solver);

    // Set integrator
    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
    
    // Create the run-time visualization system
    auto vis = CreateVisualizationSystem(vis_type, CameraVerticalDir::Y, sys, "Cables FEM (MUMPS)", ChVector3d(0, 0.6, -1.0));

    // Simulation loop
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(0.01);
        ////model.PrintBodyPositions();
    }

    return 0;
}
