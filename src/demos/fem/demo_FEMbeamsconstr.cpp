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

#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"
#include "physics/ChLinkMate.h"
#include "physics/ChLinkLock.h"
#include "lcp/ChLcpIterativeMINRES.h"
#include "unit_FEM/ChElementBeamEuler.h"
#include "unit_FEM/ChBuilderBeam.h"
#include "unit_FEM/ChMesh.h"
#include "unit_FEM/ChVisualizationFEMmesh.h"
#include "irrlicht_interface/ChIrrApp.h"


// Remember to use the namespace 'chrono' because all classes 
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
using namespace fem;




int main(int argc, char* argv[])
{
	// In CHRONO engine, The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed. 
	DLL_CreateGlobals();


	//
	// TEST 1
	//

	{

					// Create a Chrono::Engine physical system
	ChSystem my_system;


	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrApp application(&my_system, L"Beams and constraints",core::dimension2d<u32>(800,600),false, true);

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	application.AddTypicalLogo();
	application.AddTypicalSky();
	application.AddTypicalLights();
	application.AddTypicalCamera(core::vector3df(0.f, 0.6f, -1.f));

	double L = 1;
	double H = 0.25;
	double K = 0.05;
	ChVector<> vA(0,0,0);
	ChVector<> vC(L,0,0);
	ChVector<> vB(L,-H,0);
	ChVector<> vG(L-K,-H,0);
	ChVector<> vd(0,0,0.001);


				// Create a truss:
	ChSharedPtr<ChBody>  body_truss(new ChBody);

	body_truss->SetBodyFixed(true);

	my_system.AddBody(body_truss);

				// Attach a 'box' shape asset for visualization.
	ChSharedPtr<ChBoxShape> mboxtruss(new ChBoxShape);
	mboxtruss->GetBoxGeometry().Pos  = ChVector<>(-0.01, 0,0);
	mboxtruss->GetBoxGeometry().SetLenghts( ChVector<>(0.02, 0.2, 0.1) );
	body_truss->AddAsset(mboxtruss);



				// Create a moving object as a crank
	ChSharedPtr<ChBody>  body_crank(new ChBody);

	body_crank->SetPos( (vB+vG)*0.5 );

	my_system.AddBody(body_crank);

				// Attach a 'box' shape asset for visualization.
	ChSharedPtr<ChBoxShape> mboxcrank(new ChBoxShape);
	mboxcrank->GetBoxGeometry().Pos  = ChVector<>(0,0,0);
	mboxcrank->GetBoxGeometry().SetLenghts( ChVector<>(K,0.02,0.02) );
	body_crank->AddAsset(mboxcrank);


				// Create a motor between the truss
				// and the slider body:
	ChSharedPtr<ChLinkEngine> constr_motor(new ChLinkEngine);
	constr_motor->Initialize(body_truss,
						     body_crank,
							 ChCoordsys<> (vG ) );
	my_system.Add(constr_motor);

	class ChFunction_myf : public ChFunction
	{
	public:
		ChFunction* new_Duplicate() {return new ChFunction_myf;} 

		double Get_y(double x) {if (x > 0.4) return CH_C_PI; else return - CH_C_PI * (1.0 - cos(CH_C_PI*x/0.4)) / 2.0;} 
	};

	ChFunction_myf* f_ramp = new ChFunction_myf;

	constr_motor->Set_eng_mode(ChLinkEngine::ENG_MODE_ROTATION);
	constr_motor->Set_rot_funct(f_ramp);


		// Create a FEM mesh, that is a container for groups
		// of elements and their referenced nodes.
	ChSharedPtr<ChMesh> my_mesh(new ChMesh);


		// Create the horizontal beam
		
	ChSharedPtr<ChBeamSectionAdvanced> msection1(new ChBeamSectionAdvanced);

	double beam_wy = 0.10;
	double beam_wz = 0.01;
	msection1->SetDensity(2700); 
	msection1->SetYoungModulus (73.0e9);
	msection1->SetGwithPoissonRatio(0.32);
	msection1->SetAsRectangularSection(beam_wy, beam_wz);
	msection1->SetBeamRaleyghDamping(0.000);

				// This helps creating sequences of nodes and ChElementBeamEuler elements:
	ChBuilderBeam builder;

	builder.BuildBeam(	my_mesh,		// the mesh where to put the created nodes and elements 
						msection1,		// the ChBeamSectionAdvanced to use for the ChElementBeamEuler elements
						13,				// the number of ChElementBeamEuler to create
						vA,	// the 'A' point in space (beginning of beam)
						vC,	// the 'B' point in space (end of beam)
						ChVector<>(0,1, 0));		// the 'Y' up direction of the section for the beam
	
				// After having used BuildBeam(), you can retrieve the nodes used for the beam,
				// For example say you want to fix the A end and apply a force to the B end:
	builder.GetLastBeamNodes().front()->SetFixed(true);

	ChSharedPtr<ChNodeFEMxyzrot> node_tip = builder.GetLastBeamNodes().back();


		// Create the vertical beam

	ChSharedPtr<ChBeamSectionAdvanced> msection2(new ChBeamSectionAdvanced);

	double hbeam_wy = 0.024;
	double hbeam_wz = 0.024;
	msection2->SetDensity(2700); 
	msection2->SetYoungModulus (73.0e9);
	msection1->SetGwithPoissonRatio(0.32);
	msection2->SetBeamRaleyghDamping(0.000);
	msection2->SetAsRectangularSection(hbeam_wy, hbeam_wz);

	builder.BuildBeam(	my_mesh,		// the mesh where to put the created nodes and elements 
						msection2,		// the ChBeamSectionAdvanced to use for the ChElementBeamEuler elements
						3,				// the number of ChElementBeamEuler to create
						vC+vd,	// the 'A' point in space (beginning of beam)
						vB+vd,	// the 'B' point in space (end of beam)
						ChVector<>(1,0, 0));	// the 'Y' up direction of the section for the beam
	
	ChSharedPtr<ChNodeFEMxyzrot> node_top  = builder.GetLastBeamNodes().front();
	ChSharedPtr<ChNodeFEMxyzrot> node_down = builder.GetLastBeamNodes().back();

				
		// Create a constraint between the vertical and horizontal beams:

	ChSharedPtr<ChLinkMateGeneric> constr_bb(new ChLinkMateGeneric);
	constr_bb->Initialize(  node_top,
							node_tip,
							false, 
							node_top->Frame(),
							node_top->Frame() 
							 );
	my_system.Add(constr_bb);

	constr_bb->SetConstrainedCoords( true, true, true,		// x, y, z
										true, true, false);   // Rx, Ry, Rz

				// For example, attach small shape to show the constraint
	ChSharedPtr<ChSphereShape> msphereconstr2(new ChSphereShape);
	msphereconstr2->GetSphereGeometry().rad = 0.01;
	constr_bb->AddAsset(msphereconstr2);


		// Create a constraint between the vertical beam and the crank:

	ChSharedPtr<ChLinkMateGeneric> constr_bc(new ChLinkMateGeneric);
	constr_bc->Initialize(  node_down,
							body_crank,
							false, 
							node_down->Frame(),
							node_down->Frame() 
							 );
	my_system.Add(constr_bc);

	constr_bc->SetConstrainedCoords( true, true, true,		// x, y, z
									 true, true, false);   // Rx, Ry, Rz

				// For example, attach small shape to show the constraint
	ChSharedPtr<ChSphereShape> msphereconstr3(new ChSphereShape);
	msphereconstr3->GetSphereGeometry().rad = 0.01;
	constr_bc->AddAsset(msphereconstr3);




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


	ChSharedPtr<ChVisualizationFEMmesh> mvisualizebeamA(new ChVisualizationFEMmesh(*(my_mesh.get_ptr())));
	mvisualizebeamA->SetFEMdataType(ChVisualizationFEMmesh::E_PLOT_ELEM_BEAM_MX);
	mvisualizebeamA->SetColorscaleMinMax(-5000,5000);
	mvisualizebeamA->SetSmoothFaces(true);
	mvisualizebeamA->SetWireframe(false);
	my_mesh->AddAsset(mvisualizebeamA);

	ChSharedPtr<ChVisualizationFEMmesh> mvisualizebeamC(new ChVisualizationFEMmesh(*(my_mesh.get_ptr())));
	mvisualizebeamC->SetFEMglyphType(ChVisualizationFEMmesh::E_GLYPH_NODE_CSYS);
	mvisualizebeamC->SetFEMdataType(ChVisualizationFEMmesh::E_PLOT_NONE);
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

	my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_MINRES); // <- NEEDED because other solvers can't handle stiffness matrices
	my_system.SetIterLCPwarmStarting(true); // this helps a lot to speedup convergence in this class of problems
	my_system.SetIterLCPmaxItersSpeed(600);
	my_system.SetIterLCPmaxItersStab(600);
	my_system.SetTolSpeeds(1e-12);
	chrono::ChLcpIterativeMINRES* msolver = (chrono::ChLcpIterativeMINRES*)my_system.GetLcpSolverSpeed();
	//msolver->SetVerbose(true);
	msolver->SetDiagonalPreconditioning(true);

	application.SetTimestep(0.001);
	application.SetVideoframeEach(10);

	while(application.GetDevice()->run()) 
	{
		//builder.GetLastBeamNodes().back()->SetTorque(ChVector<> (0,0,0.1*application.GetSystem()->GetChTime() ) );

		application.BeginScene();

		application.DrawAll();
		
		ChIrrTools::drawGrid(application.GetVideoDriver(), 0.05, 0.05, 20, 20, ChCoordsys<>(VNULL, CH_C_PI_2, VECT_Z), video::SColor(50,90,90,90), true );

		application.DoStep();

		application.EndScene();

	}


	}


	//
	// TEST 2
	//

	{

			// Create a Chrono::Engine physical system
	ChSystem my_system;


	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrApp application(&my_system, L"Beams and constraints",core::dimension2d<u32>(800,600),false, true);

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	application.AddTypicalLogo();
	application.AddTypicalSky();
	application.AddTypicalLights();
	application.AddTypicalCamera(core::vector3df(0.f, 0.6f, -1.f));

				// Create a truss:
	ChSharedPtr<ChBody>  my_body_A(new ChBody);

	my_body_A->SetBodyFixed(true);

	my_system.AddBody(my_body_A);

				// Attach a 'box' shape asset for visualization.
	ChSharedPtr<ChBoxShape> mboxtruss(new ChBoxShape);
	mboxtruss->GetBoxGeometry().Pos  = ChVector<>(-0.01, 0,0);
	mboxtruss->GetBoxGeometry().SetLenghts( ChVector<>(0.02, 0.2, 0.1) );
	my_body_A->AddAsset(mboxtruss);
				// Attach a 'box' shape asset for visualization.
	ChSharedPtr<ChBoxShape> mboxguide(new ChBoxShape);
	mboxguide->GetBoxGeometry().Pos  = ChVector<>(0.5+0.03, 0,0);
	mboxguide->GetBoxGeometry().SetLenghts( ChVector<>(0.03, 0.5, 0.04) );
	my_body_A->AddAsset(mboxguide);


				// Create a moving object as a slider
	ChSharedPtr<ChBody>  my_body_B(new ChBody);

	my_body_B->SetPos( ChVector<> (0.5,0,0) );

	my_system.AddBody(my_body_B);

				// Attach a 'box' shape asset for visualization.
	ChSharedPtr<ChBoxShape> mboxslider(new ChBoxShape);
	mboxslider->GetBoxGeometry().Pos  = ChVector<>(0.01,0,0);
	mboxslider->GetBoxGeometry().SetLenghts( ChVector<>(0.02, 0.1, 0.15) );
	my_body_B->AddAsset(mboxslider);


				// Create a slider between the truss
				// and the slider body:
	ChSharedPtr<ChLinkLockLock> constraint3(new ChLinkLockLock);
	constraint3->Initialize(my_body_A,
						    my_body_B,
							my_body_B->GetCoord() );
	my_system.Add(constraint3);

	ChFunction_Ramp* f_ramp = new ChFunction_Ramp;
	f_ramp->Set_ang(0.1);	// set angular coefficient;
	f_ramp->Set_y0(0.0);	// set y value for x=0;

	constraint3->SetMotion_Y(f_ramp);


		// Create a FEM mesh, that is a container for groups
		// of elements and their referenced nodes.
	ChSharedPtr<ChMesh> my_mesh(new ChMesh);


	for (int i = 0; i < 5; i++)
	{
		double z_spacing = -0.1;

		//
		// Add some EULER-BERNOULLI BEAMS:
		//

					// Create a section, i.e. thickness and material properties
					// for beams. This will be shared among some beams.
		
		ChSharedPtr<ChBeamSectionAdvanced> msection(new ChBeamSectionAdvanced);

		double beam_wy = 0.06 - 0.01*i;
		double beam_wz = 0.003;
		msection->SetAsRectangularSection(beam_wy, beam_wz);
		msection->SetYoungModulus (200.8e9);
		msection->SetGshearModulus(200.8e9 * 0.3);
		msection->SetBeamRaleyghDamping(0.002);
		//msection->SetCentroid(0.0,0.00002); 
		msection->SetShearCenter(0.0,0.00004); 
		//msection->SetSectionRotation(45*CH_C_RAD_TO_DEG);


					// This helps creating sequences of nodes and ChElementBeamEuler elements:
		ChBuilderBeam builder;

		builder.BuildBeam(	my_mesh,		// the mesh where to put the created nodes and elements 
							msection,		// the ChBeamSectionAdvanced to use for the ChElementBeamEuler elements
							10,				// the number of ChElementBeamEuler to create
							ChVector<>(0, 0, i*z_spacing),	// the 'A' point in space (beginning of beam)
							ChVector<>(0.5,0,i*z_spacing),	// the 'B' point in space (end of beam)
							ChVector<>(0,1, 0));		// the 'Y' up direction of the section for the beam
		
					// After having used BuildBeam(), you can retrieve the nodes used for the beam,
					// For example say you want to fix the A end and apply a force to the B end:
		builder.GetLastBeamNodes().front()->SetFixed(true);


					// Create a constraint between the beam end 
					// and the slider body:
		ChSharedPtr<ChLinkMateGeneric> constraint2(new ChLinkMateGeneric);
		constraint2->Initialize(builder.GetLastBeamNodes().back(),
								my_body_B,
								false, 
								builder.GetLastBeamNodes().back()->Frame(),
								my_body_B->GetFrame_REF_to_abs() 
								 );
		my_system.Add(constraint2);

		constraint2->SetConstrainedCoords( false, true, false,			// x, y, z
											false, false, false);   // Rx, Ry, Rz

					// For example, attach small shape to show the constraint
		ChSharedPtr<ChSphereShape> msphereconstr2(new ChSphereShape);
		msphereconstr2->GetSphereGeometry().rad = 0.01;
		constraint2->AddAsset(msphereconstr2);

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


	ChSharedPtr<ChVisualizationFEMmesh> mvisualizebeamA(new ChVisualizationFEMmesh(*(my_mesh.get_ptr())));
	mvisualizebeamA->SetFEMdataType(ChVisualizationFEMmesh::E_PLOT_ELEM_BEAM_MX);
	mvisualizebeamA->SetColorscaleMinMax(-5000,5000);
	mvisualizebeamA->SetSmoothFaces(true);
	mvisualizebeamA->SetWireframe(false);
	my_mesh->AddAsset(mvisualizebeamA);

	ChSharedPtr<ChVisualizationFEMmesh> mvisualizebeamC(new ChVisualizationFEMmesh(*(my_mesh.get_ptr())));
	mvisualizebeamC->SetFEMglyphType(ChVisualizationFEMmesh::E_GLYPH_NODE_CSYS);
	mvisualizebeamC->SetFEMdataType(ChVisualizationFEMmesh::E_PLOT_NONE);
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

	my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_MINRES); // <- NEEDED because other solvers can't handle stiffness matrices
	my_system.SetIterLCPwarmStarting(true); // this helps a lot to speedup convergence in this class of problems
	my_system.SetIterLCPmaxItersSpeed(600);
	my_system.SetIterLCPmaxItersStab(600);
	my_system.SetTolSpeeds(1e-12);
	chrono::ChLcpIterativeMINRES* msolver = (chrono::ChLcpIterativeMINRES*)my_system.GetLcpSolverSpeed();
	//msolver->SetVerbose(true);
	msolver->SetDiagonalPreconditioning(true);

	application.SetTimestep(0.001);
	application.SetVideoframeEach(10);

	while(application.GetDevice()->run()) 
	{
		//builder.GetLastBeamNodes().back()->SetTorque(ChVector<> (0,0,0.1*application.GetSystem()->GetChTime() ) );
	
		if (application.GetSystem()->GetChTime() > 0.5)
		{
			f_ramp->Set_ang(0);
			f_ramp->Set_y0(0.5 *0.1);
		}

		application.BeginScene();

		application.DrawAll();
		
		ChIrrTools::drawGrid(application.GetVideoDriver(), 0.05, 0.05, 20, 20, ChCoordsys<>(VNULL, CH_C_PI_2, VECT_Z), video::SColor(50,90,90,90), true );

		application.DoStep();

		application.EndScene();

	}


	}







	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}


