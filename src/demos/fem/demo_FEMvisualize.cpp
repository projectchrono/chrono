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
#include "physics/ChNodeBody.h"
#include "lcp/ChLcpIterativePMINRES.h"
#include "unit_FEM/ChElementSpring.h"
#include "unit_FEM/ChElementBar.h"
#include "unit_FEM/ChElementTetra_4.h"
#include "unit_FEM/ChElementTetra_10.h"
#include "unit_FEM/ChElementHexa_8.h"
#include "unit_FEM/ChElementHexa_20.h"
#include "unit_FEM/ChMesh.h"
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
	my_mesh->AddElement(melement1);

	ChSharedPtr<ChElementTetra_4> melement2( new ChElementTetra_4);
	melement2->SetNodes(mnode3, mnode6, mnode7, mnode8);
	melement2->SetMaterial(mmaterial);
	my_mesh->AddElement(melement2);

				// This is necessary in order to precompute the 
				// stiffness matrices for all inserted elements in mesh
	my_mesh->SetupInitial();

				// Remember to add the mesh to the system!
	my_system.Add(my_mesh);


				// Create also a truss
	ChSharedPtr<ChBody> truss(new ChBody);
	truss->SetBodyFixed(true);
	my_system.Add(truss);
	
				// Create a constraint between a node and the truss
	ChSharedPtr<ChNodeBody> constraint1(new ChNodeBody);
	ChSharedPtr<ChNodeBody> constraint2(new ChNodeBody);
	ChSharedPtr<ChNodeBody> constraint3(new ChNodeBody);

	constraint1->Initialize(my_mesh,		// node container
							0,				// index of node in node container 
							truss);			// body to be connected to

	constraint2->Initialize(my_mesh,		// node container
							1,				// index of node in node container 
							truss);			// body to be connected to
							
	constraint3->Initialize(my_mesh,		// node container
							3,				// index of node in node container 
							truss);			// body to be connected to
					
	my_system.Add(constraint1);
	my_system.Add(constraint2);
	my_system.Add(constraint3);

			// ==Asset== attach a visualization of the FEM mesh.
			// This will automatically update a triangle mesh (a ChTriangleMeshShape
			// asset that is internally managed) by setting  proper
			// coordinates and vertex colours as in the FEM elements.
			// Such triangle mesh can be rendered by Irrlicht or POVray or whatever
			// postprocessor that can handle a coloured ChTriangleMeshShape).
			// Do not forget AddAsset() at the end!

	ChSharedPtr<ChVisualizationFEMmesh> mvisualizemesh(new ChVisualizationFEMmesh(*(my_mesh.get_ptr())));
	mvisualizemesh->SetFEMdataType(ChVisualizationFEMmesh::E_PLOT_NODE_SPEED_NORM);
	mvisualizemesh->SetColorscaleMinMax(0.0,5.8);
	my_mesh->AddAsset(mvisualizemesh);


			// ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
			// in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
			// If you need a finer control on which item really needs a visualization proxy in 
			// Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

//	application.AssetBindAll();
application.AssetBind(my_mesh);

			// ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
			// that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

	//application.AssetUpdateAll();
application.AssetUpdate(my_mesh);

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


