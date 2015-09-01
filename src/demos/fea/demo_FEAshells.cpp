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

  
     
// Include some headers used by this tutorial...

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/lcp/ChLcpIterativePMINRES.h"
#include "chrono/lcp/ChLcpIterativeMINRES.h"

#include "chrono_fea/ChElementShellANCF.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChLinkDirFrame.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_matlab/ChMatlabEngine.h"
#include "chrono_matlab/ChLcpMatlabSolver.h"

// Remember to use the namespace 'chrono' because all classes 
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
using namespace fea;
using namespace irr;



int main(int argc, char* argv[])
{
	// Create a Chrono::Engine physical system
	ChSystem my_system;


	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrApp application(&my_system, L"Shells FEA",core::dimension2d<u32>(800,600),false, true);

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	application.AddTypicalLogo();
	application.AddTypicalSky();
	application.AddTypicalLights();
	application.AddTypicalCamera(core::vector3df(0.f, 0.6f, -1.f));



				// Create a mesh, that is a container for groups
				// of elements and their referenced nodes.
	ChSharedPtr<ChMesh> my_mesh(new ChMesh);
	

	
	//
	// Add some ANCF SHELL BEAMS:
	//
    
	double shell_thickness = 0.01;
    double shell_L = 0.4;
    double shell_W = 0.2;
    
	// Create a section, i.e. thickness and material properties
	// for shell. This will be shared among some beams.
	/*
	ChSharedPtr<ChShellSectionBasic> msection_shell(new ChShellSectionBasic);
	msection_shell->SetThickness(shell_thickness);
	msection_shell->SetYoungModulus (0.01e9);
	msection_shell->SetBeamRaleyghDamping(0.000);
    */

	// Create the nodes (each with position & normal to shell)

	ChSharedPtr<ChNodeFEAxyzD> hnodeancf1(new ChNodeFEAxyzD( ChVector<>(0,      0.2,       0), ChVector<>(0,1,0) ) ); 
	ChSharedPtr<ChNodeFEAxyzD> hnodeancf2(new ChNodeFEAxyzD( ChVector<>(shell_L,0,       0), ChVector<>(0,1,0) ) );
    ChSharedPtr<ChNodeFEAxyzD> hnodeancf3(new ChNodeFEAxyzD( ChVector<>(shell_L,0, shell_W), ChVector<>(0,1,0) ) );
    ChSharedPtr<ChNodeFEAxyzD> hnodeancf4(new ChNodeFEAxyzD( ChVector<>(0,      0, shell_W), ChVector<>(0,1,0) ) );

	my_mesh->AddNode(hnodeancf1);
	my_mesh->AddNode(hnodeancf2);
    my_mesh->AddNode(hnodeancf3);
    my_mesh->AddNode(hnodeancf4);

	// Create the element 

	ChSharedPtr<ChElementShellANCF> elementancf1 (new ChElementShellANCF);

	elementancf1->SetNodes(hnodeancf1, hnodeancf2, hnodeancf3, hnodeancf4);
	//elementancf1->SetSection(msection_shell);

	my_mesh->AddElement(elementancf1);

				// Apply a lumped force to a node:
	hnodeancf2->SetForce( ChVector<>(0,3,0));

	hnodeancf1->SetFixed(true);
    hnodeancf4->SetFixed(true);
	


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

	ChSharedPtr<ChVisualizationFEAmesh> mvisualizeshellA(new ChVisualizationFEAmesh(*(my_mesh.get_ptr())));
	//mvisualizeshellA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_MZ);// not yet working
	//mvisualizeshellA->SetColorscaleMinMax(-0.4,0.4);
	mvisualizeshellA->SetSmoothFaces(true);
	mvisualizeshellA->SetWireframe(true);
	my_mesh->AddAsset(mvisualizeshellA);

	ChSharedPtr<ChVisualizationFEAmesh> mvisualizeshellB(new ChVisualizationFEAmesh(*(my_mesh.get_ptr())));
	//mvisualizeshellB->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);// not yet working
    mvisualizeshellB->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
	mvisualizeshellB->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
	mvisualizeshellB->SetSymbolsThickness(0.006);
	mvisualizeshellB->SetSymbolsScale(0.01);
	mvisualizeshellB->SetZbufferHide(false);
	my_mesh->AddAsset(mvisualizeshellB);


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
	my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_MINRES); // <- NEEDED THIS OR ::LCP_SIMPLEX because other solvers can't handle stiffness matrices
	my_system.SetIterLCPwarmStarting(true); // this helps a lot to speedup convergence in this class of problems
	my_system.SetIterLCPmaxItersSpeed(200);
	my_system.SetIterLCPmaxItersStab(200);
	my_system.SetTolForce(1e-13);
	chrono::ChLcpIterativeMINRES* msolver = (chrono::ChLcpIterativeMINRES*)my_system.GetLcpSolverSpeed();
	msolver->SetVerbose(false);
	msolver->SetDiagonalPreconditioning(true);
	

	// TEST: The Matlab external linear solver, for max precision in benchmarks
ChMatlabEngine matlab_engine;
ChLcpMatlabSolver* matlab_solver_stab  = new ChLcpMatlabSolver(matlab_engine);
ChLcpMatlabSolver* matlab_solver_speed = new ChLcpMatlabSolver(matlab_engine);
my_system.ChangeLcpSolverStab (matlab_solver_stab);
my_system.ChangeLcpSolverSpeed(matlab_solver_speed);
application.GetSystem()->Update();
application.SetPaused(true);


	
	// Change type of integrator: 
    //my_system.SetIntegrationType(chrono::ChSystem::INT_EULER_IMPLICIT_LINEARIZED);  // fast, less precise
	my_system.SetIntegrationType(chrono::ChSystem::INT_HHT);  // precise,slower, might iterate each step
	
	// if later you want to change integrator settings:
	if( ChSharedPtr<ChTimestepperHHT> mystepper = my_system.GetTimestepper().DynamicCastTo<ChTimestepperHHT>() )
	{
		mystepper->SetAlpha(-0.2);
		mystepper->SetMaxiters(2);
		mystepper->SetTolerance(1e-6);
	}
	 

	application.SetTimestep(0.01);



	while(application.GetDevice()->run()) 
	{
		application.BeginScene();

		application.DrawAll();
		
		application.DoStep();

		application.EndScene();
	}


	return 0;
}


