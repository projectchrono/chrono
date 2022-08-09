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
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "FEAcables.h"

using namespace chrono;
using namespace fea;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono::Engine physical system
    ChSystemSMC sys;

    // Create a mesh, that is a container for groups of elements and
    // their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create the model (defined in FEAcables.h)
    auto model = Model3(sys, my_mesh);

    // Remember to add the mesh to the system!
    sys.Add(my_mesh);

    // Visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChTriangleMeshShape
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colors as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a colored ChTriangleMeshShape).

    auto mvisualizebeamA = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizebeamA->SetFEMdataType(ChVisualShapeFEA::DataType::ELEM_BEAM_MZ);
    mvisualizebeamA->SetColorscaleMinMax(-0.4, 0.4);
    mvisualizebeamA->SetSmoothFaces(true);
    mvisualizebeamA->SetWireframe(false);
    my_mesh->AddVisualShapeFEA(mvisualizebeamA);

    auto mvisualizebeamC = chrono_types::make_shared<ChVisualShapeFEA>(my_mesh);
    mvisualizebeamC->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
    mvisualizebeamC->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    mvisualizebeamC->SetSymbolsThickness(0.006);
    mvisualizebeamC->SetSymbolsScale(0.01);
    mvisualizebeamC->SetZbufferHide(false);
    my_mesh->AddVisualShapeFEA(mvisualizebeamC);


    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Cables FEM (MUMPS)");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector<>(0.0, 0.6, -1.0));
    vis->AttachSystem(&sys);

    // Configure MKL solver.
    // For this simple and relatively small problem, use of the sparsity pattern learner may introduce additional
    // overhead (if the sparsity pattern is not locked).
    auto mkl_solver = chrono_types::make_shared<ChSolverMumps>();
    mkl_solver->UseSparsityPatternLearner(false);
    mkl_solver->LockSparsityPattern(false);
    sys.SetSolver(mkl_solver);

    sys.Update();

    // Set integrator
    sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

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
