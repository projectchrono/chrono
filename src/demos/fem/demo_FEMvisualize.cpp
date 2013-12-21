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
//     - FEM visualization using Irrlicht

  
     
// Include some headers used by this tutorial...

#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"
#include "lcp/ChLcpIterativePMINRES.h"
#include "unit_FEM/ChElementSpring.h"
#include "unit_FEM/ChElementBar.h"
#include "unit_FEM/ChElementTetra_4.h"
#include "unit_FEM/ChElementTetra_10.h"
#include "unit_FEM/ChElementHexa_8.h"
#include "unit_FEM/ChElementHexa_20.h"
#include "unit_FEM/ChMesh.h"
#include "unit_FEM/ChNodeBody.h"
#include "unit_FEM/ChVisualizationFEMmesh.h"
#include "irrlicht_interface/ChIrrApp.h"


// Remember to use the namespace 'chrono' because all classes 
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
using namespace fem;




// Do some tests in a single run, inside the main() function.
// Results will be simply text-formatted outputs in the console..

int main(int argc, char* argv[])
{
	// In CHRONO engine, The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed. 
	DLL_CreateGlobals();

	// Create a Chrono::Engine physical system
	ChSystem my_system;


	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrApp application(&my_system, L"Irrlicht FEM visualization",core::dimension2d<u32>(800,600),false, true);

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	application.AddTypicalLogo();
	application.AddTypicalSky();
	application.AddTypicalLights();
	application.AddTypicalCamera(core::vector3df(0,2,-3));



				// Create a mesh, that is a container for groups
				// of elements and their referenced nodes.
	ChSharedPtr<ChMesh> my_mesh(new ChMesh);
	
				// Create a material, that must be assigned to each element,
				// and set its parameters
	ChSharedPtr<ChContinuumElastic> mmaterial(new ChContinuumElastic);
	mmaterial->Set_E(0.01e9); // rubber 0.01e9, steel 200e9
	mmaterial->Set_v(0.3);
	mmaterial->Set_RayleighDampingK(0.0001);
	mmaterial->Set_density(1000);

/*
				// Create some nodes. These are the classical point-like
				// nodes with x,y,z degrees of freedom, that can be used 
				// for many types of FEM elements in space.
	ChSharedPtr<ChNodeFEMxyz> mnode1( new ChNodeFEMxyz(ChVector<>(0,0,0)) );
	ChSharedPtr<ChNodeFEMxyz> mnode2( new ChNodeFEMxyz(ChVector<>(0,0,1)) );
	ChSharedPtr<ChNodeFEMxyz> mnode3( new ChNodeFEMxyz(ChVector<>(0,1,0)) );
	ChSharedPtr<ChNodeFEMxyz> mnode4( new ChNodeFEMxyz(ChVector<>(1,0,0)) );

	ChSharedPtr<ChNodeFEMxyz> mnode6( new ChNodeFEMxyz(ChVector<>(0,1,1)) );
	ChSharedPtr<ChNodeFEMxyz> mnode7( new ChNodeFEMxyz(ChVector<>(0,2,0)) );
	ChSharedPtr<ChNodeFEMxyz> mnode8( new ChNodeFEMxyz(ChVector<>(1,1,0)) );

				// For example, set a point-like mass at a node:
	mnode3->SetMass(200);

				// For example, set an initial displacement to a node:
	mnode3->SetPos( mnode3->GetX0() + ChVector<>(0,0.26,0) );

	// Remember to add nodes and elements to the mesh!
	my_mesh->AddNode(mnode1);
	my_mesh->AddNode(mnode2);
	my_mesh->AddNode(mnode3);
	my_mesh->AddNode(mnode4);
	
	my_mesh->AddNode(mnode6);
	my_mesh->AddNode(mnode7);
	my_mesh->AddNode(mnode8);

				// Create the tetrahedron element, and assign 
				// nodes and material
	ChSharedPtr<ChElementTetra_4> melement1( new ChElementTetra_4);
	melement1->SetNodes(mnode1, mnode2, mnode3, mnode4);
	melement1->SetMaterial(mmaterial);
				// Remember to add elements to the mesh!
//	my_mesh->AddElement(melement1);

	ChSharedPtr<ChElementTetra_4> melement2( new ChElementTetra_4);
	melement2->SetNodes(mnode3, mnode6, mnode7, mnode8);
	melement2->SetMaterial(mmaterial);
//	my_mesh->AddElement(melement2);
*/
				// Load a .node file and a .ele  file from disk, defining a complicate tetahedron mesh.
				// This is much easier than creating all nodes and elements via C++ programming.
				// You can generate these files using the TetGen tool.
	try 
	{
		my_mesh->LoadFromTetGenFile("../data/unit_FEM/beam.node","../data/unit_FEM/beam.ele", mmaterial);
	}
	catch (ChException myerr) {
			GetLog() << myerr.what();
	}

				// Apply a force to a node
	ChSharedPtr<ChNodeFEMxyz> mnodelast (my_mesh->GetNode(my_mesh->GetNnodes()-1));
	mnodelast->SetForce( ChVector<>(400,0,0));


	double sx = 0.4;
	double sy = 0.4;
	double sz = 0.4;
	ChSharedPtr<ChNodeFEMxyz> hnode1(new ChNodeFEMxyz(ChVector<>(0, 0,  0)));
	ChSharedPtr<ChNodeFEMxyz> hnode2(new ChNodeFEMxyz(ChVector<>(0, 0,  sz)));
	ChSharedPtr<ChNodeFEMxyz> hnode3(new ChNodeFEMxyz(ChVector<>(sx,0,  sz)));
	ChSharedPtr<ChNodeFEMxyz> hnode4(new ChNodeFEMxyz(ChVector<>(sx,0,  0)));
	ChSharedPtr<ChNodeFEMxyz> hnode5(new ChNodeFEMxyz(ChVector<>(0, sy, 0)));
	ChSharedPtr<ChNodeFEMxyz> hnode6(new ChNodeFEMxyz(ChVector<>(0, sy, sz)));
	ChSharedPtr<ChNodeFEMxyz> hnode7(new ChNodeFEMxyz(ChVector<>(sx,sy, sz)));
	ChSharedPtr<ChNodeFEMxyz> hnode8(new ChNodeFEMxyz(ChVector<>(sx,sy, 0)));

				// For example, set an initial displacement to a node:
	hnode8->SetPos( hnode8->GetX0() + ChVector<>(0,0.1,0) );

				// Remember to add nodes and elements to the mesh!
	my_mesh->AddNode(hnode1);
	my_mesh->AddNode(hnode2);
	my_mesh->AddNode(hnode3);
	my_mesh->AddNode(hnode4);
	my_mesh->AddNode(hnode5);
	my_mesh->AddNode(hnode6);
	my_mesh->AddNode(hnode7);
	my_mesh->AddNode(hnode8);

				// Create the hexahedron element, and assign 
				// its nodes and material
	ChSharedPtr<ChElementHexa_8> helement1 (new ChElementHexa_8);
	helement1->SetNodes(hnode1, hnode2, hnode3, hnode4, hnode5, hnode6, hnode7, hnode8);
	helement1->SetMaterial(mmaterial);

				// Remember to add elements to the mesh!
	my_mesh->AddElement(helement1);



				// This is necessary in order to precompute the 
				// stiffness matrices for all inserted elements in mesh
	my_mesh->SetupInitial();

				// Remember to add the mesh to the system!
	my_system.Add(my_mesh);


				// Create also a truss
	ChSharedPtr<ChBody> truss(new ChBody);
	truss->SetBodyFixed(true);
	my_system.Add(truss);
	
				// Create constraints between nodes and truss

	for (int iconstr = 0; iconstr < 15; ++iconstr)
	{
		ChSharedPtr<ChNodeBody> constraint(new ChNodeBody);

		//constraint->Initialize(my_mesh,		// node container
		//					iconstr,	// index of node in node container 
		//					truss);		// body to be connected to
		constraint->Initialize(my_mesh->GetNode(iconstr),
								truss);
		my_system.Add(constraint);
	}

			// ==Asset== attach a visualization of the FEM mesh.
			// This will automatically update a triangle mesh (a ChTriangleMeshShape
			// asset that is internally managed) by setting  proper
			// coordinates and vertex colours as in the FEM elements.
			// Such triangle mesh can be rendered by Irrlicht or POVray or whatever
			// postprocessor that can handle a coloured ChTriangleMeshShape).
			// Do not forget AddAsset() at the end!

	ChSharedPtr<ChVisualizationFEMmesh> mvisualizemesh(new ChVisualizationFEMmesh(*(my_mesh.get_ptr())));
	mvisualizemesh->SetFEMdataType(ChVisualizationFEMmesh::E_PLOT_ELEM_STRAIN_VONMISES);
	mvisualizemesh->SetColorscaleMinMax(0.0,0.80);
	mvisualizemesh->SetShrinkElements(false,0.85);
	mvisualizemesh->SetSmoothFaces(true);
	my_mesh->AddAsset(mvisualizemesh);

	ChSharedPtr<ChVisualizationFEMmesh> mvisualizemeshwire(new ChVisualizationFEMmesh(*(my_mesh.get_ptr())));
	mvisualizemeshwire->SetFEMdataType(ChVisualizationFEMmesh::E_PLOT_NONE);
	mvisualizemeshwire->SetColorscaleMinMax(0.0,4.0);
	mvisualizemeshwire->SetWireframe(true);
	my_mesh->AddAsset(mvisualizemeshwire);

	ChSharedPtr<ChVisualizationFEMmesh> mvisualizemeshref(new ChVisualizationFEMmesh(*(my_mesh.get_ptr())));
	mvisualizemeshref->SetFEMdataType(ChVisualizationFEMmesh::E_PLOT_NONE);
	mvisualizemeshref->SetWireframe(true);
	mvisualizemeshref->SetDrawInUndeformedReference(true);
	my_mesh->AddAsset(mvisualizemeshref);

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

	my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_PMINRES); // <- NEEDED because other solvers can't handle stiffness matrices
	my_system.SetIterLCPmaxItersSpeed(40);
	my_system.SetTolSpeeds(1e-10);

	application.SetStepManage(true);
	application.SetTimestep(0.001);
	application.SetTryRealtime(false);

	while(application.GetDevice()->run()) 
	{
		application.BeginScene();

		application.DrawAll();
		
		application.DoStep();

		//	GetLog() << " t =" << my_system.GetChTime() << "  mnode3 pos.y=" << mnode3->GetPos().y << "  \n";

		application.EndScene();
		//break;
	}




	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}


