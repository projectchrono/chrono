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


#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChSystemDEM.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/lcp/ChLcpIterativeMINRES.h"
#include "chrono/geometry/ChCTriangleMeshConnected.h"

#include "chrono_fea/ChElementTetra_4.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"
#include "chrono_fea/ChElementBeamANCF.h"
#include "chrono_fea/ChBuilderBeam.h"

#include "chrono_irrlicht/ChIrrApp.h"


using namespace chrono;
using namespace geometry;
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


    //collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0); // not needed, already 0 when using ChSystemDEM
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.005); // max inside penetration - if not enough stiffness in material: troubles


    // Create the surface material, containing information
    // about friction etc.

    ChSharedPtr<ChMaterialSurfaceDEM> mysurfmaterial (new ChMaterialSurfaceDEM);
    mysurfmaterial->SetKn(2e5);
    mysurfmaterial->SetKt(2e5);
    mysurfmaterial->SetGn(2200);
    mysurfmaterial->SetGt(2200);


    // Create a floor:

    bool do_mesh_collision_floor = true;

    ChTriangleMeshConnected mmeshbox;
    mmeshbox.LoadWavefrontMesh(GetChronoDataFile("cube.obj"),true,true);

    if(do_mesh_collision_floor) {

        // floor as a triangle mesh surface:
        ChSharedPtr<ChBody> mfloor(new ChBody);
        mfloor->SetPos(ChVector<>(0, -1, 0));
        mfloor->SetBodyFixed(true);
        mfloor->SetMaterialSurface(mysurfmaterial);
        my_system.Add(mfloor);

        mfloor->GetCollisionModel()->ClearModel();
        mfloor->GetCollisionModel()->AddTriangleMesh(mmeshbox,false, false, VNULL, ChMatrix33<>(1), 0.004);
        mfloor->GetCollisionModel()->BuildModel();
        mfloor->SetCollide(true);

        ChSharedPtr<ChTriangleMeshShape> masset_meshbox(new ChTriangleMeshShape());
        masset_meshbox->SetMesh(mmeshbox);
        mfloor->AddAsset(masset_meshbox);

        ChSharedPtr<ChTexture> masset_texture(new ChTexture());
        masset_texture->SetTextureFilename(GetChronoDataFile("concrete.jpg"));
        mfloor->AddAsset(masset_texture);
        
    }
    else {
        // floor as a simple collision primitive:

        ChSharedPtr<ChBodyEasyBox> mfloor (new ChBodyEasyBox(2,0.1,2,2700, true));
        mfloor->SetBodyFixed(true);
        mfloor->SetMaterialSurface(mysurfmaterial);
        my_system.Add(mfloor);

        ChSharedPtr<ChTexture> masset_texture(new ChTexture());
        masset_texture->SetTextureFilename(GetChronoDataFile("concrete.jpg"));
        mfloor->AddAsset(masset_texture);
    }

    

    // two falling objects:

    ChSharedPtr<ChBodyEasyBox> mcube (new ChBodyEasyBox(0.1,0.1,0.1,2700, true));
    mcube->SetPos(ChVector<>(0.6,0.5,0.6));
    mcube->SetMaterialSurface(mysurfmaterial);
    my_system.Add(mcube);

    ChSharedPtr<ChBodyEasySphere> msphere (new ChBodyEasySphere(0.1,2700, true));
    msphere->SetPos(ChVector<>(0.8,0.5,0.6));
    msphere->SetMaterialSurface(mysurfmaterial);
    my_system.Add(msphere);


    //
    // Example 1: tetrahedrons, with collisions
    // 

    // Create a mesh. We will use it for tetahedrons.

    ChSharedPtr<ChMesh> my_mesh(new ChMesh);

    // 1) a FEA tetahedron(s):

    // Create a material, that must be assigned to each solid element in the mesh,
    // and set its parameters
    ChSharedPtr<ChContinuumElastic> mmaterial(new ChContinuumElastic);
    mmaterial->Set_E(0.01e9);  // rubber 0.01e9, steel 200e9
    mmaterial->Set_v(0.3);
    mmaterial->Set_RayleighDampingK(0.003);
    mmaterial->Set_density(1000);

    for (int i=0; i<1; ++i) {
        // Creates the nodes for the tetahedron
        ChVector<> offset(i*0.12,i*0.1, i*0.03);
        ChSharedPtr<ChNodeFEAxyz> mnode1(new ChNodeFEAxyz(ChVector<>(0,   0.1, 0  )+offset));
        ChSharedPtr<ChNodeFEAxyz> mnode2(new ChNodeFEAxyz(ChVector<>(0,   0.1, 0.2)+offset));
        ChSharedPtr<ChNodeFEAxyz> mnode3(new ChNodeFEAxyz(ChVector<>(0,   0.3, 0  )+offset));
        ChSharedPtr<ChNodeFEAxyz> mnode4(new ChNodeFEAxyz(ChVector<>(0.2, 0.1, 0  )+offset));

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
    }

    // Create the contact surface(s). 
    // In this case it is a ChContactSurfaceGeneric, that allows mesh-mesh collsions.

    ChSharedPtr<ChContactSurfaceGeneric> mcontactsurf (new ChContactSurfaceGeneric);
    my_mesh->AddContactSurface(mcontactsurf);

    mcontactsurf->AddFacesFromBoundary(0.004); // do this after my_mesh->AddContactSurface

    mcontactsurf->SetMaterialSurface(mysurfmaterial); // use the DEM penalty contacts


    // This is necessary in order to precompute the
    // stiffness matrices for all inserted elements in mesh
    my_mesh->SetupInitial();

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);


    //
    // Example 2: beams, with collisions
    // 

    // Create a mesh. We will use it for beams only. 

    ChSharedPtr<ChMesh> my_mesh_beams(new ChMesh);

    // 2) an ANCF cable:

	ChSharedPtr<ChBeamSectionCable> msection_cable2(new ChBeamSectionCable);
	msection_cable2->SetDiameter(0.05);
	msection_cable2->SetYoungModulus (0.01e9);
	msection_cable2->SetBeamRaleyghDamping(0.05);

	ChBuilderBeamANCF builder;

	builder.BuildBeam(	my_mesh_beams,	// the mesh where to put the created nodes and elements 
						msection_cable2,// the ChBeamSectionCable to use for the ChElementBeamANCF elements
						10,				// the number of ChElementBeamANCF to create
						ChVector<>(0, 0.1, -0.1),		// the 'A' point in space (beginning of beam)
						ChVector<>(0.5, 0.13, -0.1));	// the 'B' point in space (end of beam)

    // Create the contact surface(s). 
    // In this case it is a ChContactSurfaceNodeCloud, so just pass 
    // all nodes to it.

    ChSharedPtr<ChContactSurfaceNodeCloud> mcontactcloud (new ChContactSurfaceNodeCloud);
    my_mesh_beams->AddContactSurface(mcontactcloud);

    mcontactcloud->AddAllNodes(0.025); // use larger point size to match beam section radius

    mcontactcloud->SetMaterialSurface(mysurfmaterial);

    
    // This is necessary in order to precompute the
    // stiffness matrices for all inserted elements in mesh
    my_mesh_beams->SetupInitial();

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh_beams);


    



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

    ChSharedPtr<ChVisualizationFEAmesh> mvisualizemeshcoll(new ChVisualizationFEAmesh(*(my_mesh.get_ptr())));
    mvisualizemeshcoll->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_CONTACTSURFACES);
    mvisualizemeshcoll->SetWireframe(true);
    mvisualizemeshcoll->SetDefaultMeshColor(ChColor(1,0.5,0));
    my_mesh->AddAsset(mvisualizemeshcoll); 

    ChSharedPtr<ChVisualizationFEAmesh> mvisualizemeshbeam(new ChVisualizationFEAmesh(*(my_mesh_beams.get_ptr())));
    mvisualizemeshbeam->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    mvisualizemeshbeam->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemeshbeam->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizemeshbeam);

    ChSharedPtr<ChVisualizationFEAmesh> mvisualizemeshbeamnodes(new ChVisualizationFEAmesh(*(my_mesh_beams.get_ptr())));
    mvisualizemeshbeamnodes->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    mvisualizemeshbeamnodes->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshbeamnodes->SetSymbolsThickness(0.008);
    my_mesh->AddAsset(mvisualizemeshbeamnodes);
 
 

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
