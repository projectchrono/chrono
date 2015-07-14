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
//     - contacts in FEA


#include "physics/ChSystem.h"
#include "physics/ChSystemDEM.h"
#include "physics/ChBodyEasy.h"
#include "lcp/ChLcpIterativeMINRES.h"
#include "unit_FEA/ChElementTetra_4.h"
#include "unit_FEA/ChMesh.h"
#include "unit_FEA/ChVisualizationFEAmesh.h"
#include "unit_IRRLICHT/ChIrrApp.h"


using namespace chrono;
using namespace fea;
using namespace irr;

int main(int argc, char* argv[]) {

    // Create a Chrono::Engine physical system
    ChSystemDEM my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"FEA contacts", core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, (f32)0.6, -1));
    application.SetContactsDrawMode(irr::ChIrrTools::CONTACT_DISTANCES);

    //
    // Create the system
    //

    ChSharedPtr<ChMaterialSurfaceDEM> mysurfmaterial (new ChMaterialSurfaceDEM);

    // Create a floor
    ChSharedPtr<ChBodyEasyBox> mfloor (new ChBodyEasyBox(3,0.1,3,2700, true));
    mfloor->SetBodyFixed(true);
    mfloor->SetMaterialSurface(mysurfmaterial);
    my_system.Add(mfloor);

    ChSharedPtr<ChBodyEasyBox> mcube (new ChBodyEasyBox(0.1,0.1,0.1,2700, true));
    mcube->SetPos(ChVector<>(1,0.5,1));
    mcube->SetMaterialSurface(mysurfmaterial);
    my_system.Add(mcube);

    ChSharedPtr<ChBodyEasySphere> msphere (new ChBodyEasySphere(0.1,2700, true));
    msphere->SetPos(ChVector<>(1.2,0.5,1));
    msphere->SetMaterialSurface(mysurfmaterial);
    my_system.Add(msphere);

   
    // Create a mesh, that is a container for groups
    // of FEA elements and their referenced nodes.
    ChSharedPtr<ChMesh> my_mesh(new ChMesh);

    // Create a material, that must be assigned to each element,
    // and set its parameters
    ChSharedPtr<ChContinuumElastic> mmaterial(new ChContinuumElastic);
    mmaterial->Set_E(0.01e9);  // rubber 0.01e9, steel 200e9
    mmaterial->Set_v(0.3);
    mmaterial->Set_RayleighDampingK(0.001);
    mmaterial->Set_density(1000);

    // Create a tetahedron
    ChSharedPtr<ChNodeFEAxyz> mnode1(new ChNodeFEAxyz(ChVector<>(0, 1, 0)));
    ChSharedPtr<ChNodeFEAxyz> mnode2(new ChNodeFEAxyz(ChVector<>(0, 1, 1)));
    ChSharedPtr<ChNodeFEAxyz> mnode3(new ChNodeFEAxyz(ChVector<>(0, 2, 0)));
    ChSharedPtr<ChNodeFEAxyz> mnode4(new ChNodeFEAxyz(ChVector<>(1, 1, 0)));

    my_mesh->AddNode(mnode1);
    my_mesh->AddNode(mnode2);
    my_mesh->AddNode(mnode3);
    my_mesh->AddNode(mnode4);

    mnode1->SetForce(ChVector<>(0,-15,0));

    // Create the tetrahedron element, and assign
    // nodes and material
    ChSharedPtr<ChElementTetra_4> melement1(new ChElementTetra_4);
    melement1->SetNodes(mnode1, mnode2, mnode3, mnode4);
    melement1->SetMaterial(mmaterial);

    // Remember to add elements to the mesh!
    my_mesh->AddElement(melement1);

    // This is necessary in order to precompute the
    // stiffness matrices for all inserted elements in mesh
    my_mesh->SetupInitial();
  

    // Create the contact surfaces
    ChSharedPtr<ChContactSurfaceNodeCloud> mcontactsurf (new ChContactSurfaceNodeCloud);
    mcontactsurf->AddNode(mnode1);
    mcontactsurf->AddNode(mnode2);
    mcontactsurf->AddNode(mnode3);
    mcontactsurf->AddNode(mnode4);
    
    mcontactsurf->SetMaterialSurface(mysurfmaterial);

    my_mesh->AddContactSurface(mcontactsurf);


    //
    // Final touches..
    //

    // This is necessary in order to precompute the
    // stiffness matrices for all inserted elements in mesh
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

    ChSharedPtr<ChVisualizationFEAmesh> mvisualizemesh(new ChVisualizationFEAmesh(*(my_mesh.get_ptr())));
    mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizemesh); 

    ChSharedPtr<ChVisualizationFEAmesh> mvisualizemeshC(new ChVisualizationFEAmesh(*(my_mesh.get_ptr())));
    mvisualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    mvisualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshC->SetSymbolsThickness(0.006);
    my_mesh->AddAsset(mvisualizemeshC);
 
 

    // ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
    // in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
    // If you need a finer control on which item really needs a visualization proxy in
    // Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

    application.AssetBindAll();

    // ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
    // that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

    application.AssetUpdateAll();

    //
    // THE SOFT-REAL-TIME CYCLE
    //
    my_system.ChangeLcpSolverSpeed(new ChLcpIterativeMINRES);
    //my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_MINRES);     // <- NEEDED because other solvers can't handle stiffness matrices
    my_system.SetIterLCPwarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
    my_system.SetIterLCPmaxItersSpeed(40);
    my_system.SetTolForce(1e-10);
    // chrono::ChLcpIterativeMINRES* msolver = (chrono::ChLcpIterativeMINRES*)my_system.GetLcpSolverSpeed();
    // msolver->SetVerbose(true);
    // msolver->SetDiagonalPreconditioning(true);
    my_system.SetIntegrationType(chrono::ChSystem::INT_EULER_IMPLICIT_LINEARIZED);  // fast, less precise

    //***TEST***
    /*
    ChMatlabEngine matlab_engine;
    ChLcpMatlabSolver* matlab_solver_stab  = new ChLcpMatlabSolver(matlab_engine);
    ChLcpMatlabSolver* matlab_solver_speed = new ChLcpMatlabSolver(matlab_engine);
    my_system.ChangeLcpSolverStab (matlab_solver_stab);
    my_system.ChangeLcpSolverSpeed(matlab_solver_speed);
    */
    application.SetTimestep(0.001);

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        application.DoStep();

        application.EndScene();
    }

    return 0;
}
