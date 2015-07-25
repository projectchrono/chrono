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

#include "physics/ChSystem.h"
#include "physics/ChBodyEasy.h"
#include "timestepper/ChTimestepper.h"
#include "lcp/ChLcpIterativePMINRES.h"
#include "lcp/ChLcpIterativeMINRES.h"
#include "unit_FEA/ChElementBeamANCF.h"
#include "unit_FEA/ChBuilderBeam.h"
#include "unit_FEA/ChMesh.h"
#include "unit_FEA/ChVisualizationFEAmesh.h"
#include "unit_FEA/ChLinkPointFrame.h"
#include "unit_FEA/ChLinkDirFrame.h"
#include "unit_IRRLICHT/ChIrrApp.h"
//#include "unit_MATLAB/ChMatlabEngine.h"
//#include "unit_MATLAB/ChLcpMatlabSolver.h"
#include "unit_MKL/ChLcpMklSolver.h"

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
	ChIrrApp application(&my_system, L"Cables FEM",core::dimension2d<u32>(800,600),false, true);

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	application.AddTypicalLogo();
	application.AddTypicalSky();
	application.AddTypicalLights();
	application.AddTypicalCamera(core::vector3df(0.f, 0.6f, -1.f));



				// Create a mesh, that is a container for groups
				// of elements and their referenced nodes.
	ChSharedPtr<ChMesh> my_mesh(new ChMesh);
	

    if (false){

	    // TEST 1
	    // Add some ANCF CABLE BEAMS:
	    //
    
	    double beam_L  = 0.1;
	    double beam_diameter = 0.015;
    
	    // Create a section, i.e. thickness and material properties
	    // for beams. This will be shared among some beams.
	
	    ChSharedPtr<ChBeamSectionCable> msection_cable(new ChBeamSectionCable);
	    msection_cable->SetDiameter(beam_diameter);
	    msection_cable->SetYoungModulus (0.01e9);
	    msection_cable->SetBeamRaleyghDamping(0.000);

	    // Create the nodes

	    ChSharedPtr<ChNodeFEAxyzD> hnodeancf1(new ChNodeFEAxyzD( ChVector<>(0,0,-0.2), ChVector<>(1,0,0) ) ); 
	    ChSharedPtr<ChNodeFEAxyzD> hnodeancf2(new ChNodeFEAxyzD( ChVector<>(beam_L,0,-0.2), ChVector<>(1,0,0) ) );

	    my_mesh->AddNode(hnodeancf1);
	    my_mesh->AddNode(hnodeancf2);

	    // Create the element 

	    ChSharedPtr<ChElementBeamANCF> belementancf1 (new ChElementBeamANCF);

	    belementancf1->SetNodes(hnodeancf1, hnodeancf2);
	    belementancf1->SetSection(msection_cable);

	    my_mesh->AddElement(belementancf1);

				    // Apply a force or a torque to a node:
	    hnodeancf2->SetForce( ChVector<>(0,3,0));

	    hnodeancf1->SetFixed(true);
	


	    // Add a rigid body connected to the end of the beam:

	    ChSharedPtr<ChBodyEasyBox> mbox (new ChBodyEasyBox(0.1,0.02,0.02,1000) );
	    mbox->SetPos(hnodeancf2->GetPos() + ChVector<>(0.05,0,0));
	    my_system.Add(mbox);

	    ChSharedPtr<ChLinkPointFrame> constraint_pos(new ChLinkPointFrame);
	    constraint_pos->Initialize(hnodeancf2, mbox);
	    my_system.Add(constraint_pos);

	    ChSharedPtr<ChLinkDirFrame> constraint_dir(new ChLinkDirFrame);
	    constraint_dir->Initialize(hnodeancf2, mbox);
	    constraint_dir->SetDirectionInAbsoluteCoords(ChVector<>(1,0,0));
	    my_system.Add(constraint_dir);
    }
    
    if (false)
    {

        // TEST 2
	    // Add some ANCF CABLE BEAMS (the fast way!)
	    //


	    // Create a section, i.e. thickness and material properties
	    // for beams. This will be shared among some beams.
	
	    ChSharedPtr<ChBeamSectionCable> msection_cable2(new ChBeamSectionCable);
	    msection_cable2->SetDiameter(0.015);
	    msection_cable2->SetYoungModulus (0.01e9);
	    msection_cable2->SetBeamRaleyghDamping(0.000);


    			    // Shortcut!
				    // This ChBuilderBeamANCF helper object is very useful because it will 
				    // subdivide 'beams' into sequences of finite elements of beam type, ex.
				    // one 'beam' could be made of 5 FEM elements of ChElementBeamANCF class.
				    // If new nodes are needed, it will create them for you.
	    ChBuilderBeamANCF builder;

				    // Now, simply use BuildBeam to create a beam from a point to another: 
	    builder.BuildBeam(	my_mesh,		// the mesh where to put the created nodes and elements 
						    msection_cable2,// the ChBeamSectionCable to use for the ChElementBeamANCF elements
						    10,				// the number of ChElementBeamANCF to create
						    ChVector<>(0, 0, -0.1),		// the 'A' point in space (beginning of beam)
						    ChVector<>(0.5, 0, -0.1));	// the 'B' point in space (end of beam)
	
				    // After having used BuildBeam(), you can retrieve the nodes used for the beam,
				    // For example say you want to fix both pos and dir of A end and apply a force to the B end:
	    //builder.GetLastBeamNodes().back()->SetFixed(true);
	    builder.GetLastBeamNodes().front()->SetForce( ChVector<>(0,-0.2,0));

                    // For instance, now retrieve the A end and add a constraint to
                    // block the position only of that node:
        ChSharedPtr<ChBody> mtruss (new ChBody);
        mtruss->SetBodyFixed(true);

        ChSharedPtr<ChLinkPointFrame> constraint_hinge(new ChLinkPointFrame);
	    constraint_hinge->Initialize(builder.GetLastBeamNodes().back(), mtruss);
	    my_system.Add(constraint_hinge);
    }


    if (true){

        // TEST 3
	    // Add ANCF CABLE BEAMS making chains with bodies
	    //

        ChSharedPtr<ChBeamSectionCable> msection_cable2(new ChBeamSectionCable);
	    msection_cable2->SetDiameter(0.015);
	    msection_cable2->SetYoungModulus (0.01e9);
	    msection_cable2->SetBeamRaleyghDamping(0.000);

        ChSharedPtr<ChBody> mtruss (new ChBody);
        mtruss->SetBodyFixed(true);

        for (int j= 0; j< 6; ++j)
        {
	        ChBuilderBeamANCF builder;

				        // Now, simply use BuildBeam to create a beam from a point to another: 
	        builder.BuildBeam(	my_mesh,		// the mesh where to put the created nodes and elements 
						        msection_cable2,// the ChBeamSectionCable to use for the ChElementBeamANCF elements
						        1+j,				// the number of ChElementBeamANCF to create
						        ChVector<>(0, 0, -0.1*j),		// the 'A' point in space (beginning of beam, front)
						        ChVector<>(0.1+0.1*j, 0, -0.1*j));	// the 'B' point in space (end of beam, back)
	
	        builder.GetLastBeamNodes().back()->SetForce( ChVector<>(0,-0.2,0));
           
            ChSharedPtr<ChLinkPointFrame> constraint_hinge(new ChLinkPointFrame);
	        constraint_hinge->Initialize(builder.GetLastBeamNodes().front(), mtruss);
	        my_system.Add(constraint_hinge);

            ChSharedPtr<ChSphereShape> msphere(new ChSphereShape);
            msphere->GetSphereGeometry().rad = 0.02;
            constraint_hinge->AddAsset(msphere);

                          // make a box and connect it 
            ChSharedPtr<ChBodyEasyBox> mbox (new ChBodyEasyBox(0.2,0.04,0.04,1000) );
	        mbox->SetPos(builder.GetLastBeamNodes().back()->GetPos() + ChVector<>(0.1,0,0));
	        my_system.Add(mbox);

	        ChSharedPtr<ChLinkPointFrame> constraint_pos(new ChLinkPointFrame);
	        constraint_pos->Initialize(builder.GetLastBeamNodes().back(), mbox);
	        my_system.Add(constraint_pos);

	        ChSharedPtr<ChLinkDirFrame> constraint_dir(new ChLinkDirFrame);
	        constraint_dir->Initialize(builder.GetLastBeamNodes().back(), mbox);
	        constraint_dir->SetDirectionInAbsoluteCoords(ChVector<>(1,0,0));
	        my_system.Add(constraint_dir);

                        // make another beam
	        builder.BuildBeam(	my_mesh,		// the mesh where to put the created nodes and elements 
						        msection_cable2,// the ChBeamSectionCable to use for the ChElementBeamANCF elements
						        1+(6-j),				// the number of ChElementBeamANCF to create
						        ChVector<>(mbox->GetPos().x+0.1, 0, -0.1*j),		// the 'A' point in space (beginning of beam, front)
						        ChVector<>(mbox->GetPos().x+0.1+0.1*(6-j), 0, -0.1*j));	// the 'B' point in space (end of beam, back)

             ChSharedPtr<ChLinkPointFrame> constraint_pos2(new ChLinkPointFrame);
	         constraint_pos2->Initialize(builder.GetLastBeamNodes().front(), mbox);
	         my_system.Add(constraint_pos2);

	         ChSharedPtr<ChLinkDirFrame> constraint_dir2(new ChLinkDirFrame);
	         constraint_dir2->Initialize(builder.GetLastBeamNodes().front(), mbox);
	         constraint_dir2->SetDirectionInAbsoluteCoords(ChVector<>(1,0,0));
	         my_system.Add(constraint_dir2);

                           // make a box and connect it 
            ChSharedPtr<ChBodyEasyBox> mbox2 (new ChBodyEasyBox(0.2,0.04,0.04,1000) );
	        mbox2->SetPos(builder.GetLastBeamNodes().back()->GetPos() + ChVector<>(0.1,0,0));
	        my_system.Add(mbox2);

	        ChSharedPtr<ChLinkPointFrame> constraint_pos3(new ChLinkPointFrame);
	        constraint_pos3->Initialize(builder.GetLastBeamNodes().back(), mbox2);
	        my_system.Add(constraint_pos3);

	        ChSharedPtr<ChLinkDirFrame> constraint_dir3(new ChLinkDirFrame);
	        constraint_dir3->Initialize(builder.GetLastBeamNodes().back(), mbox2);
	        constraint_dir3->SetDirectionInAbsoluteCoords(ChVector<>(1,0,0));
	        my_system.Add(constraint_dir3);

        }
    }

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

	ChSharedPtr<ChVisualizationFEAmesh> mvisualizebeamA(new ChVisualizationFEAmesh(*(my_mesh.get_ptr())));
	mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_MZ);
	mvisualizebeamA->SetColorscaleMinMax(-0.4,0.4);
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
    /*
    ChMatlabEngine matlab_engine;
    ChLcpMatlabSolver* matlab_solver_stab  = new ChLcpMatlabSolver(matlab_engine);
    ChLcpMatlabSolver* matlab_solver_speed = new ChLcpMatlabSolver(matlab_engine);
    my_system.ChangeLcpSolverStab (matlab_solver_stab);
    my_system.ChangeLcpSolverSpeed(matlab_solver_speed);
    application.GetSystem()->Update();
    */
    
    ChLcpMklSolver* mkl_solver_stab  = new ChLcpMklSolver;
    ChLcpMklSolver* mkl_solver_speed = new ChLcpMklSolver;
    my_system.ChangeLcpSolverStab (mkl_solver_stab);
	my_system.ChangeLcpSolverSpeed(mkl_solver_speed);
    application.GetSystem()->Update();
    
	
	// Change type of integrator: 
    my_system.SetIntegrationType(chrono::ChSystem::INT_EULER_IMPLICIT_LINEARIZED);  // fast, less precise
	//my_system.SetIntegrationType(chrono::ChSystem::INT_HHT);  // precise,slower, might iterate each step
	
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


