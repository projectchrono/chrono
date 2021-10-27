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
//       uses the Chrono MKL module
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#include "chrono_irrlicht/ChIrrApp.h"

#include "FEAcables.h"

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono::Engine physical system
    ChSystemSMC my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Cables FEM (MKL)", core::dimension2d<u32>(800, 600));

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0.f, 0.6f, -1.f));

    // Create a mesh, that is a container for groups of elements and
    // their referenced nodes.
    auto my_mesh = chrono_types::make_shared<ChMesh>();

    // Create the model (defined in FEAcables.h)
    auto model = Model3(my_system, my_mesh);

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // ==Asset== attach a visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChTriangleMeshShape
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colors as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a colored ChTriangleMeshShape).
    // Do not forget AddAsset() at the end!

    auto mvisualizebeamA = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_MZ);
    mvisualizebeamA->SetColorscaleMinMax(-0.4, 0.4);
    mvisualizebeamA->SetSmoothFaces(true);
    mvisualizebeamA->SetWireframe(false);
    my_mesh->AddAsset(mvisualizebeamA);

    auto mvisualizebeamC = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
    mvisualizebeamC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizebeamC->SetSymbolsThickness(0.006);
    mvisualizebeamC->SetSymbolsScale(0.01);
    mvisualizebeamC->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizebeamC);

    // ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
    // in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
    // If you need a finer control on which item really needs a visualization proxy in
    // Irrlicht, just use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
    // that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!
    application.AssetUpdateAll();

    // Configure PardisoMKL solver.
    // For this simple and relatively small problem, use of the sparsity pattern learner may introduce additional
    // overhead (if the sparsity pattern is not locked).
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    mkl_solver->UseSparsityPatternLearner(false);
    mkl_solver->LockSparsityPattern(false);
    mkl_solver->SetVerbose(false);
    my_system.SetSolver(mkl_solver);

    my_system.Update();

    // Set integrator
    my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    // SIMULATION LOOP
    application.SetTimestep(0.01);

    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        application.DoStep();
        application.EndScene();
        ////model.PrintBodyPositions();
    }

    return 0;
}
