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
	application.AddTypicalCamera(core::vector3df(0,0.6,-1));



				// Create a mesh, that is a container for groups
				// of elements and their referenced nodes.
	ChSharedPtr<ChMesh> my_mesh(new ChMesh);
	
				// Create a material, that must be assigned to each element,
				// and set its parameters
	ChSharedPtr<ChContinuumElastic> mmaterial(new ChContinuumElastic);
	mmaterial->Set_E(0.01e9); // rubber 0.01e9, steel 200e9
	mmaterial->Set_v(0.3);
	mmaterial->Set_RayleighDampingK(0.001);
	mmaterial->Set_density(1000);


	//
	// Add some TETAHEDRONS:
	//

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
	mnodelast->SetForce( ChVector<>(100,0,0));


	//
	// Add some HEXAHEDRONS (isoparametric bricks):
	//

	double sx = 0.1;
	double sy = 0.1;
	double sz = 0.1;

	ChSharedPtr<ChNodeFEMxyz> hnode1_lower;
	ChSharedPtr<ChNodeFEMxyz> hnode2_lower;
	ChSharedPtr<ChNodeFEMxyz> hnode3_lower;
	ChSharedPtr<ChNodeFEMxyz> hnode4_lower;

	for (int ilayer = 0; ilayer < 6; ++ilayer)
	{
		double hy = ilayer*sz;
		ChSharedPtr<ChNodeFEMxyz> hnode1(new ChNodeFEMxyz(ChVector<>(0, hy,  0)));
		ChSharedPtr<ChNodeFEMxyz> hnode2(new ChNodeFEMxyz(ChVector<>(0, hy,  sz)));
		ChSharedPtr<ChNodeFEMxyz> hnode3(new ChNodeFEMxyz(ChVector<>(sx,hy,  sz)));
		ChSharedPtr<ChNodeFEMxyz> hnode4(new ChNodeFEMxyz(ChVector<>(sx,hy,  0)));
		my_mesh->AddNode(hnode1);
		my_mesh->AddNode(hnode2);
		my_mesh->AddNode(hnode3);
		my_mesh->AddNode(hnode4);

		if (ilayer>0)
		{
			ChSharedPtr<ChElementHexa_8> helement1 (new ChElementHexa_8);
			helement1->SetNodes(hnode1_lower, hnode2_lower, hnode3_lower, hnode4_lower, hnode1, hnode2, hnode3, hnode4);
			helement1->SetMaterial(mmaterial);

			my_mesh->AddElement(helement1);
		}

		hnode1_lower = hnode1;
		hnode2_lower = hnode2;
		hnode3_lower = hnode3;
		hnode4_lower = hnode4;
	}

				// For example, set an initial displacement to a node:
	hnode4_lower->SetPos( hnode4_lower->GetX0() + ChVector<>(0.1,0.1,0) );

				// Apply a force to a node
	hnode4_lower->SetForce( ChVector<>(500,0,0));


	//
	// Final touches..
	// 

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
				// (for example, fix to ground all nodes which are near y=0
	for (int inode = 0; inode < my_mesh->GetNnodes(); ++inode)
	{
		if (my_mesh->GetNode(inode).IsType<ChNodeFEMxyz>())
		{
			ChSharedPtr<ChNodeFEMxyz> mnode ( my_mesh->GetNode(inode) ); // downcast
			if (mnode->GetPos().y <0.01)
			{

				ChSharedPtr<ChNodeBody> constraint(new ChNodeBody);
				constraint->Initialize(mnode,
									   truss);
				my_system.Add(constraint);

						// For example, attach small cube to show the constraint
				ChSharedPtr<ChBoxShape> mboxfloor(new ChBoxShape);
				mboxfloor->GetBoxGeometry().Size = ChVector<>(0.005);
				constraint->AddAsset(mboxfloor);
				
				
				// Otherwise there is an easier method: just set the node as fixed (but 
				// in this way you do not get infos about reaction forces as with a constraint):
				//
				//  mnode->SetFixed(true); 

			}
		}
	}


			// ==Asset== attach a visualization of the FEM mesh.
			// This will automatically update a triangle mesh (a ChTriangleMeshShape
			// asset that is internally managed) by setting  proper
			// coordinates and vertex colours as in the FEM elements.
			// Such triangle mesh can be rendered by Irrlicht or POVray or whatever
			// postprocessor that can handle a coloured ChTriangleMeshShape).
			// Do not forget AddAsset() at the end!

	ChSharedPtr<ChVisualizationFEMmesh> mvisualizemesh(new ChVisualizationFEMmesh(*(my_mesh.get_ptr())));
	mvisualizemesh->SetFEMdataType(ChVisualizationFEMmesh::E_PLOT_NODE_SPEED_NORM);
	mvisualizemesh->SetColorscaleMinMax(0.0,4.40);
	mvisualizemesh->SetShrinkElements(true,0.85);
	mvisualizemesh->SetSmoothFaces(true);
	my_mesh->AddAsset(mvisualizemesh);
/*
	ChSharedPtr<ChVisualizationFEMmesh> mvisualizemeshwire(new ChVisualizationFEMmesh(*(my_mesh.get_ptr())));
	mvisualizemeshwire->SetFEMdataType(ChVisualizationFEMmesh::E_PLOT_NONE);
	mvisualizemeshwire->SetColorscaleMinMax(0.0,4.0);
	mvisualizemeshwire->SetWireframe(true);
	my_mesh->AddAsset(mvisualizemeshwire);
*/
	ChSharedPtr<ChVisualizationFEMmesh> mvisualizemeshref(new ChVisualizationFEMmesh(*(my_mesh.get_ptr())));
	mvisualizemeshref->SetFEMdataType(ChVisualizationFEMmesh::E_PLOT_SURFACE);
	mvisualizemeshref->SetWireframe(true);
	mvisualizemeshref->SetDrawInUndeformedReference(true);
	my_mesh->AddAsset(mvisualizemeshref);

	ChSharedPtr<ChVisualizationFEMmesh> mvisualizemeshC(new ChVisualizationFEMmesh(*(my_mesh.get_ptr())));
	mvisualizemeshC->SetFEMglyphType(ChVisualizationFEMmesh::E_GLYPH_NODE_DOT_POS);
	mvisualizemeshC->SetFEMdataType(ChVisualizationFEMmesh::E_PLOT_NONE);
	mvisualizemeshC->SetSymbolsThickness(0.006);
	my_mesh->AddAsset(mvisualizemeshC);

	ChSharedPtr<ChVisualizationFEMmesh> mvisualizemeshD(new ChVisualizationFEMmesh(*(my_mesh.get_ptr())));
	mvisualizemeshD->SetFEMglyphType(ChVisualizationFEMmesh::E_GLYPH_NODE_VECT_SPEED);
	mvisualizemeshD->SetFEMdataType(ChVisualizationFEMmesh::E_PLOT_NONE);
	mvisualizemeshD->SetSymbolsScale(0.02);
	my_mesh->AddAsset(mvisualizemeshD);



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
	my_system.SetIterLCPwarmStarting(true); // this helps a lot to speedup convergence in this class of problems
	my_system.SetIterLCPmaxItersSpeed(40);
	my_system.SetTolSpeeds(1e-10);
	//chrono::ChLcpIterativePMINRES* msolver = (chrono::ChLcpIterativePMINRES*)my_system.GetLcpSolverSpeed();
	//msolver->SetVerbose(true);
	//msolver->SetDiagonalPreconditioning(true);

	application.SetTimestep(0.001);

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


