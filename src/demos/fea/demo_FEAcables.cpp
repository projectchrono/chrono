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
//   Demo code about
//
//     - FEA for 3D beams of 'cable' type (ANCF gradient-deficient beams)

#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/lcp/ChLcpIterativePMINRES.h"
#include "chrono/lcp/ChLcpIterativeMINRES.h"
#include "chrono_irrlicht/ChIrrApp.h"

#include "FEAcables.h"

using namespace chrono;
using namespace fea;
using namespace irr;

int main(int argc, char* argv[]) {
    // Create a Chrono::Engine physical system
    ChSystem my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Cables FEM", core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0.f, 0.6f, -1.f));

    // Create a mesh, that is a container for groups of elements and
    // their referenced nodes.
    ChSharedPtr<ChMesh> my_mesh(new ChMesh);

    // Create one of the available models (defined in FEAcables.h)
    // model1(my_system, my_mesh);
    // model2(my_system, my_mesh);
    model3(my_system, my_mesh);

    // This is necessary in order to precompute the stiffness matrices for all
    // inserted elements in mesh
    my_mesh->SetupInitial();

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // ==Asset== attach a visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChTriangleMeshShape
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colours as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a coloured ChTriangleMeshShape).
    // Do not forget AddAsset() at the end!

    ChSharedPtr<ChVisualizationFEAmesh> mvisualizebeamA(new ChVisualizationFEAmesh(*(my_mesh.get_ptr())));
    mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_MZ);
    mvisualizebeamA->SetColorscaleMinMax(-0.4, 0.4);
    mvisualizebeamA->SetSmoothFaces(true);
    mvisualizebeamA->SetWireframe(false);
    my_mesh->AddAsset(mvisualizebeamA);

    ChSharedPtr<ChVisualizationFEAmesh> mvisualizebeamC(new ChVisualizationFEAmesh(*(my_mesh.get_ptr())));
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

    // Change solver settings
    my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_MINRES);  // <- NEEDED THIS OR ::LCP_SIMPLEX because other
                                                                 // solvers can't handle stiffness matrices
    my_system.SetIterLCPwarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
    my_system.SetIterLCPmaxItersSpeed(200);
    my_system.SetIterLCPmaxItersStab(200);
    my_system.SetTolForce(1e-13);
    chrono::ChLcpIterativeMINRES* msolver = (chrono::ChLcpIterativeMINRES*)my_system.GetLcpSolverSpeed();
    msolver->SetVerbose(false);
    msolver->SetDiagonalPreconditioning(true);

    // Change type of integrator:
    my_system.SetIntegrationType(chrono::ChSystem::INT_EULER_IMPLICIT_LINEARIZED);  // fast, less precise
    // my_system.SetIntegrationType(chrono::ChSystem::INT_HHT);  // precise,slower, might iterate each step

    // if later you want to change integrator settings:
    if (ChSharedPtr<ChTimestepperHHT> mystepper = my_system.GetTimestepper().DynamicCastTo<ChTimestepperHHT>()) {
        mystepper->SetAlpha(-0.2);
        mystepper->SetMaxiters(2);
        mystepper->SetTolerance(1e-6);
    }

    //
    // THE SOFT-REAL-TIME CYCLE
    //
    application.SetTimestep(0.01);

    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        application.DoStep();
        application.EndScene();
    }

    return 0;
}
