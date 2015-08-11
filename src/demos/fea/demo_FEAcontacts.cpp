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
#include "unit_FEA/ChElementBeamANCF.h"
#include "unit_FEA/ChBuilderBeam.h"


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
    // CREATE THE PHYSICAL SYSTEM
    //

    // Create the surface material, containing information
    // about friction etc.

    ChSharedPtr<ChMaterialSurfaceDEM> mysurfmaterial (new ChMaterialSurfaceDEM);
    mysurfmaterial->SetKn(2e5);
    mysurfmaterial->SetKt(2e5);
    mysurfmaterial->SetGn(2200);
    mysurfmaterial->SetGt(2200);

    // RIGID BODIES
    // Create some rigid bodies, for instance a floor and two bouncing items:

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

   
    // FINITE ELEMENT MESH
    // Create a mesh, that is a container for groups
    // of FEA elements and their referenced nodes.

    ChSharedPtr<ChMesh> my_mesh(new ChMesh);

    // 1) a FEA tetahedron:

    // Create a material, that must be assigned to each solid element in the mesh,
    // and set its parameters
    ChSharedPtr<ChContinuumElastic> mmaterial(new ChContinuumElastic);
    mmaterial->Set_E(0.01e9);  // rubber 0.01e9, steel 200e9
    mmaterial->Set_v(0.3);
    mmaterial->Set_RayleighDampingK(0.003);
    mmaterial->Set_density(1000);

    // Creates the nodes for the tetahedron
    ChSharedPtr<ChNodeFEAxyz> mnode1(new ChNodeFEAxyz(ChVector<>(0, 0.2, 0)));
    ChSharedPtr<ChNodeFEAxyz> mnode2(new ChNodeFEAxyz(ChVector<>(0, 0.2, 0.2)));
    ChSharedPtr<ChNodeFEAxyz> mnode3(new ChNodeFEAxyz(ChVector<>(0, 0.4, 0)));
    ChSharedPtr<ChNodeFEAxyz> mnode4(new ChNodeFEAxyz(ChVector<>(0.2, 0.2, 0)));

    my_mesh->AddNode(mnode1);
    my_mesh->AddNode(mnode2);
    my_mesh->AddNode(mnode3);
    my_mesh->AddNode(mnode4);

    ChSharedPtr<ChElementTetra_4> melement1(new ChElementTetra_4);
    melement1->SetNodes(mnode1,
                        mnode2, 
                        mnode3, 
                        mnode4);
    melement1->SetMaterial(mmaterial);

    my_mesh->AddElement(melement1);

  
    // 2) an ANCF cable:

	ChSharedPtr<ChBeamSectionCable> msection_cable2(new ChBeamSectionCable);
	msection_cable2->SetDiameter(0.05);
	msection_cable2->SetYoungModulus (0.01e9);
	msection_cable2->SetBeamRaleyghDamping(0.05);

	ChBuilderBeamANCF builder;

	builder.BuildBeam(	my_mesh,		// the mesh where to put the created nodes and elements 
						msection_cable2,// the ChBeamSectionCable to use for the ChElementBeamANCF elements
						10,				// the number of ChElementBeamANCF to create
						ChVector<>(0, 0.1, -0.1),		// the 'A' point in space (beginning of beam)
						ChVector<>(0.5, 0.13, -0.1));	// the 'B' point in space (end of beam)

    // Apply some gravity-like forces
     for (unsigned int i = 0; i< my_mesh->GetNnodes(); ++i)
        my_mesh->GetNode(i).DynamicCastTo<ChNodeFEAxyz>()->SetForce(ChVector<>(0,-10,0)); // to simulate gravity..


    // 3) the contact surface

    // Create the contact surface(s). 
    // In this case it is a ChContactSurfaceNodeCloud, so just pass 
    // all nodes to it.

    ChSharedPtr<ChContactSurfaceNodeCloud> mcontactsurf (new ChContactSurfaceNodeCloud);

    for (unsigned int i = 0; i< my_mesh->GetNnodes(); ++i)
        mcontactsurf->AddNode( my_mesh->GetNode(i).DynamicCastTo<ChNodeFEAxyz>() );

    mcontactsurf->SetMaterialSurface(mysurfmaterial);

    my_mesh->AddContactSurface(mcontactsurf);



    // This is necessary in order to precompute the
    // stiffness matrices for all inserted elements in mesh
    my_mesh->SetupInitial();

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);



    //
    // Optional...  visualuzation
    //

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

    my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_MINRES);     
    my_system.SetIterLCPwarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
    my_system.SetIterLCPmaxItersSpeed(40);
    my_system.SetTolForce(1e-10);
    my_system.SetIntegrationType(chrono::ChSystem::INT_EULER_IMPLICIT_LINEARIZED);  

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
