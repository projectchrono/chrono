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
//     - FEM for 3D beams and constrains

  
     
// Include some headers used by this tutorial...

#include "physics/ChSystem.h"
#include "physics/ChLinkMate.h"
#include "physics/ChLinkLock.h"
#include "physics/ChLinkMate.h"
#include "physics/ChLinkRackpinion.h"
#include "physics/ChBodyEasy.h"
#include "assets/ChVisualization.h"
#include "lcp/ChLcpIterativeMINRES.h"
#include "unit_FEM/ChElementBeamEuler.h"
#include "unit_FEM/ChBuilderBeam.h"
#include "unit_FEM/ChMesh.h"
#include "unit_FEM/ChVisualizationFEMmesh.h"
#include "unit_IRRLICHT/ChIrrApp.h"
#include "unit_MATLAB/ChMatlabEngine.h"

// Remember to use the namespace 'chrono' because all classes 
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
using namespace fem;




int main(int argc, char* argv[])
{
					// Create a Chrono::Engine physical system
	ChSystem my_system;

	double scales = 100;


	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrApp application(&my_system, L"Beams and constraints",core::dimension2d<u32>(800,600),false, true);

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	application.AddTypicalLogo();
	application.AddTypicalSky();
	application.AddTypicalLights();
	application.AddTypicalCamera(core::vector3df(0.f, scales*0.01f, scales*0.01f));
	application.GetSceneManager()->getActiveCamera()->setNearValue(0.001f);
	application.GetSceneManager()->getActiveCamera()->setFarValue(scales*0.03f);

	

	double thickZ = scales * 0.00015;
	double hbarW  = scales * 0.00070;
	double hbarL1 = scales * 0.00381;
	double hbarL2 = scales * 0.00387;
	double hbarL3 = scales * 0.00381;
	double vbarL  = scales * 0.01137;
	double vbarW  = scales * 0.00006;
	double Rpinion= scales * 0.00040;
	double OffPin = scales * 0.00050;
	double Rbalance=scales * 0.00500;
	double Wbalance=scales * 0.00015;
	bool   simple_rack = false;

	ChVector<> vAh(-hbarL1-hbarL2*0.5,	vbarL,	0);
	ChVector<> vBh(       -hbarL2*0.5,	vbarL,	0);
	ChVector<> vCh(        hbarL2*0.5,	vbarL,	0);
	ChVector<> vDh( hbarL1+hbarL2*0.5,	vbarL,	0);
	ChVector<> vAl(-hbarL1-hbarL2*0.5,	    0,	0);
	ChVector<> vBl(       -hbarL2*0.5,	    0,	0);
	ChVector<> vCl(        hbarL2*0.5,	    0,	0);
	ChVector<> vDl( hbarL1+hbarL2*0.5,	    0,	0);
	ChVector<> vP (                 0,-Rpinion-hbarW*0.5,	0);




				// Create a truss:
	ChSharedPtr<ChBody>  body_truss(new ChBody);

	body_truss->SetBodyFixed(true);

	my_system.AddBody(body_truss);

	/*
				// Attach a 'box' shape asset for visualization.
	ChSharedPtr<ChBoxShape> mboxtruss(new ChBoxShape);
	mboxtruss->GetBoxGeometry().Pos  = ChVector<>(-0.01, 0,0);
	mboxtruss->GetBoxGeometry().SetLengths( ChVector<>(0.02, 0.2, 0.1) );
	body_truss->AddAsset(mboxtruss);
	
	*/



		// Create a FEM mesh, that is a container for groups
		// of elements and their referenced nodes.
	ChSharedPtr<ChMesh> my_mesh(new ChMesh);


		// Create the horizontal beams
		
	ChSharedPtr<ChBeamSectionAdvanced> msectionH(new ChBeamSectionAdvanced);

	msectionH->SetDensity(7000); //***TEST*** must be 7k
	msectionH->SetYoungModulus (200.0e9);
	msectionH->SetGwithPoissonRatio(0.32);
	msectionH->SetAsRectangularSection(hbarW,thickZ);
	msectionH->SetBeamRaleyghDamping(0.00);



	ChBuilderBeam builder;

	builder.BuildBeam(	my_mesh,		// the mesh where to put the created nodes and elements 
						msectionH,		// the ChBeamSectionAdvanced to use for the ChElementBeamEuler elements
						2,				// the number of ChElementBeamEuler to create
						vAh,			// the 'Ah' point in space (beginning of beam)
						vBh,			// the 'Bh' point in space (end of beam)
						ChVector<>(0,1, 0));		// the 'Y' up direction of the section for the beam
	
				// After having used BuildBeam(), you can retrieve the nodes used for the beam
	ChSharedPtr<ChNodeFEMxyzrot> node_Ah = builder.GetLastBeamNodes().front();
	ChSharedPtr<ChNodeFEMxyzrot> node_Bh = builder.GetLastBeamNodes().back();


	builder.BuildBeam(	my_mesh,		// the mesh where to put the created nodes and elements 
						msectionH,		// the ChBeamSectionAdvanced to use for the ChElementBeamEuler elements
						2,				// the number of ChElementBeamEuler to create
						node_Bh,		// the 'Bh' point in space (beginning of beam)
						vCh,			// the 'Ch' point in space (end of beam)
						ChVector<>(0,1, 0));		// the 'Y' up direction of the section for the beam
	
				// After having used BuildBeam(), you can retrieve the nodes used for the beam
	ChSharedPtr<ChNodeFEMxyzrot> node_Ch = builder.GetLastBeamNodes().back();


	builder.BuildBeam(	my_mesh,		// the mesh where to put the created nodes and elements 
						msectionH,		// the ChBeamSectionAdvanced to use for the ChElementBeamEuler elements
						2,				// the number of ChElementBeamEuler to create
						node_Ch,		// the 'Ch' point in space (beginning of beam)
						vDh,			// the 'Dh' point in space (end of beam)
						ChVector<>(0,1, 0));		// the 'Y' up direction of the section for the beam
	
				// After having used BuildBeam(), you can retrieve the nodes used for the beam
	ChSharedPtr<ChNodeFEMxyzrot> node_Dh = builder.GetLastBeamNodes().back();


		// Create the vertical flexible beams
		
	ChSharedPtr<ChBeamSectionAdvanced> msectionV(new ChBeamSectionAdvanced);

	msectionV->SetDensity(7000); //***TEST*** must be 7k
	msectionV->SetYoungModulus (200.0e9);
	msectionV->SetGwithPoissonRatio(0.32);
	msectionV->SetAsRectangularSection(vbarW, thickZ);
	msectionV->SetBeamRaleyghDamping(0.00);


	builder.BuildBeam(	my_mesh,		// the mesh where to put the created nodes and elements 
						msectionV,		// the ChBeamSectionAdvanced to use for the ChElementBeamEuler elements
						6,				// the number of ChElementBeamEuler to create
						node_Ah,		// the 'Ah' point in space (beginning of beam)
						vAl,			// the 'Al' point in space (end of beam)
						ChVector<>(1,0, 0));		// the 'Y' up direction of the section for the beam
	
				// After having used BuildBeam(), you can retrieve the nodes used for the beam
	ChSharedPtr<ChNodeFEMxyzrot> node_Al = builder.GetLastBeamNodes().back();

	node_Al->SetFixed(true);



	builder.BuildBeam(	my_mesh,		// the mesh where to put the created nodes and elements 
						msectionV,		// the ChBeamSectionAdvanced to use for the ChElementBeamEuler elements
						6,				// the number of ChElementBeamEuler to create
						node_Dh,		// the 'Dh' point in space (beginning of beam)
						vDl,			// the 'Dl' point in space (end of beam)
						ChVector<>(1,0, 0));		// the 'Y' up direction of the section for the beam
	
				// After having used BuildBeam(), you can retrieve the nodes used for the beam
	ChSharedPtr<ChNodeFEMxyzrot> node_Dl = builder.GetLastBeamNodes().back();

	node_Dl->SetFixed(true);



				// Create the inner vertical flexible beams
			
		builder.BuildBeam(	my_mesh,		// the mesh where to put the created nodes and elements 
							msectionV,		// the ChBeamSectionAdvanced to use for the ChElementBeamEuler elements
							6,				// the number of ChElementBeamEuler to create
							node_Bh,		// the 'Bh' point in space (beginning of beam)
							vBl,			// the 'Bl' point in space (end of beam)
							ChVector<>(1,0, 0));		// the 'Y' up direction of the section for the beam
		
					// After having used BuildBeam(), you can retrieve the nodes used for the beam
		ChSharedPtr<ChNodeFEMxyzrot> node_Bl = builder.GetLastBeamNodes().back();

		builder.BuildBeam(	my_mesh,		// the mesh where to put the created nodes and elements 
							msectionV,		// the ChBeamSectionAdvanced to use for the ChElementBeamEuler elements
							6,				// the number of ChElementBeamEuler to create
							node_Ch,		// the 'Dh' point in space (beginning of beam)
							vCl,			// the 'Dl' point in space (end of beam)
							ChVector<>(1,0, 0));		// the 'Y' up direction of the section for the beam
		
					// After having used BuildBeam(), you can retrieve the nodes used for the beam
		ChSharedPtr<ChNodeFEMxyzrot> node_Cl = builder.GetLastBeamNodes().back();

				
			// Create the rack

		if (simple_rack)
		{
			builder.BuildBeam(	my_mesh,	// the mesh where to put the created nodes and elements 
						msectionH,			// the ChBeamSectionAdvanced to use for the ChElementBeamEuler elements
						1,					// the number of ChElementBeamEuler to create
						node_Bl,			// the 'Cl' point in space (beginning of beam)
						node_Cl,			// the 'Dl' point in space (end of beam)
						ChVector<>(0,1, 0));		// the 'Y' up direction of the section for the beam
		}

	//
	// Final touches to mesh..
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


	ChSharedPtr<ChVisualizationFEMmesh> mvisualizebeamA(new ChVisualizationFEMmesh(*(my_mesh.get_ptr())));
	mvisualizebeamA->SetFEMdataType(ChVisualizationFEMmesh::E_PLOT_NODE_SPEED_NORM) ;//E_PLOT_ELEM_BEAM_MZ);
	mvisualizebeamA->SetColorscaleMinMax(-30,30);
	mvisualizebeamA->SetSmoothFaces(true);
	mvisualizebeamA->SetWireframe(false);
	my_mesh->AddAsset(mvisualizebeamA);

	ChSharedPtr<ChVisualizationFEMmesh> mvisualizebeamC(new ChVisualizationFEMmesh(*(my_mesh.get_ptr())));
	mvisualizebeamC->SetFEMglyphType(ChVisualizationFEMmesh::E_GLYPH_NODE_CSYS);
	mvisualizebeamC->SetFEMdataType(ChVisualizationFEMmesh::E_PLOT_NONE);
	mvisualizebeamC->SetSymbolsThickness(0.001);
	mvisualizebeamC->SetSymbolsScale(0.01);
	mvisualizebeamC->SetZbufferHide(false);
	my_mesh->AddAsset(mvisualizebeamC);


	// 
	// The balance and the rigid rach
	//

	if (!simple_rack)
	{
		ChSharedPtr<ChBodyEasyBox> rack (new ChBodyEasyBox(hbarL2, hbarW, thickZ, 7000, false) );
		rack->SetPos ( 0.5*(vBl + vCl) );
		my_system.Add(rack);

		ChSharedPtr<ChLinkMateGeneric> constr_B(new ChLinkMateGeneric);
		constr_B->Initialize(  node_Bl,
								rack,
								false, 
								node_Bl->Frame(),
								node_Bl->Frame() 
							 );
		my_system.Add(constr_B);

		ChSharedPtr<ChLinkMateGeneric> constr_C(new ChLinkMateGeneric);
		constr_C->Initialize(  node_Cl,
								rack,
								false, 
								node_Cl->Frame(),
								node_Cl->Frame() 
							 );
		my_system.Add(constr_C);


		ChSharedPtr<ChBodyEasyCylinder> balance (new ChBodyEasyCylinder(Rbalance,Wbalance, 7000, false) );
		balance->SetPos( vP + ChVector<> (0,0,-OffPin) );
		balance->SetRot( Q_from_AngAxis( CH_C_PI_2, VECT_X) );
		for (int i = 0; i< 6; ++i)
		{
			double phi = CH_C_2PI *(i/6.0);
			ChSharedPtr<ChCylinderShape> vshape (new ChCylinderShape() );
			vshape->GetCylinderGeometry().p1 = ChVector<> (sin(phi)*Rbalance*0.8, Wbalance, cos(phi)*Rbalance*0.8);
			vshape->GetCylinderGeometry().p2 = vshape->GetCylinderGeometry().p1 + ChVector<> (0, 2*Wbalance, 0);
			vshape->GetCylinderGeometry().rad = Rbalance*0.1;
			balance->AddAsset( vshape );
		}
		ChSharedPtr<ChCylinderShape> vshaft (new ChCylinderShape() );
		vshaft->GetCylinderGeometry().p1 = vP + ChVector<> (0, -OffPin*10, 0);
		vshaft->GetCylinderGeometry().p2 = vP + ChVector<> (0,  OffPin*10, 0);
		vshaft->GetCylinderGeometry().rad = Rpinion;
		balance->AddAsset( vshaft );
		ChSharedPtr<ChVisualization> mcol (new ChVisualization());
		mcol->SetColor( ChColor(0.5,0.9,0.9));
		balance->AddAsset( mcol );

		my_system.Add(balance);

		ChSharedPtr<ChLinkLockRevolute> revolute ( new ChLinkLockRevolute() );
		ChSharedPtr<ChBody> mbalance = balance;
		revolute->Initialize(mbalance, 
							body_truss, 
							ChCoordsys<>( vP + ChVector<> (0,0,-0.01) ) );
		
	 	my_system.Add(revolute);

		ChSharedPtr<ChLinkRackpinion> constr_rack(new ChLinkRackpinion);
		constr_rack->Initialize(balance,
								rack,
								false, 
								ChFrame<>(),
								ChFrame<>() 
							 );

		ChFrameMoving<> f_pin_abs ( vP );
		ChFrameMoving<> f_rack_abs( vP + ChVector<>(0,0.1,0) );
		ChFrameMoving<> f_pin;
		ChFrameMoving<> f_rack;
		balance->TrasformParentToLocal( f_pin_abs, f_pin); 
		rack->TrasformParentToLocal( f_rack_abs, f_rack); 
		constr_rack->SetPinionRadius(Rpinion);
		constr_rack->SetPinionFrame ( f_pin);
		constr_rack->SetRackFrame ( f_rack);

		my_system.Add(constr_rack);

		balance->SetWvel_par( ChVector<> (0, 0, 1.5) );
	}



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

	my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_MINRES); // <- NEEDED because other solvers can't handle stiffness matrices
	my_system.SetIterLCPwarmStarting(true); // this helps a lot to speedup convergence in this class of problems
	my_system.SetIterLCPmaxItersSpeed(400);
	my_system.SetIterLCPmaxItersStab(400);
	my_system.SetTolSpeeds(1e-25);
	chrono::ChLcpIterativeMINRES* msolver = (chrono::ChLcpIterativeMINRES*)my_system.GetLcpSolverSpeed();
	msolver->SetVerbose(true);
	msolver->SetDiagonalPreconditioning(false);

//my_system.SetLcpSolverType(ChSystem::LCP_SIMPLEX);
//***TEST***
ChMatlabEngine matlab_engine;
ChLcpMatlabSolver* matlab_solver_stab  = new ChLcpMatlabSolver(matlab_engine);
ChLcpMatlabSolver* matlab_solver_speed = new ChLcpMatlabSolver(matlab_engine);
my_system.ChangeLcpSolverStab (matlab_solver_stab);
my_system.ChangeLcpSolverSpeed(matlab_solver_speed);

	my_system.Set_G_acc( ChVector<> (0,0,0) );

	// Do a static solution
	application.SetPaused(true);

GetLog() << "STATIC linear solve ----\n";
node_Cl->SetForce(ChVector<>(50,0,0));
//application.GetSystem()->DoStaticLinear();
node_Cl->SetForce(ChVector<>(0,0,0));

	if (simple_rack)
	{
		node_Cl->SetForce(ChVector<>(50,0,0));
		application.GetSystem()->DoStaticNonlinear(12);
		node_Cl->SetForce(ChVector<>(0,0,0));
	}

	application.SetTimestep(0.01);
	application.SetVideoframeSaveInterval(10);
	application.SetSymbolscale(0.01);


	while(application.GetDevice()->run()) 
	{
		application.BeginScene();

		application.DrawAll();
		
		ChIrrTools::drawGrid(application.GetVideoDriver(), 0.2, 0.2, 10, 10, ChCoordsys<>(VNULL, CH_C_PI_2, VECT_Z), video::SColor(50,90,100,100), true );

		application.DoStep();

		application.EndScene();
	}



	return 0;
}


