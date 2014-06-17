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
#include "unit_MATLAB/ChMatlabEngine.h"


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
	ChVector<> vd(0,0,0.0001);


				// Create a truss:
	ChSharedPtr<ChBody>  body_truss(new ChBody);

	body_truss->SetBodyFixed(true);

	my_system.AddBody(body_truss);

				// Attach a 'box' shape asset for visualization.
	ChSharedPtr<ChBoxShape> mboxtruss(new ChBoxShape);
	mboxtruss->GetBoxGeometry().Pos  = ChVector<>(-0.01, 0,0);
	mboxtruss->GetBoxGeometry().SetLenghts( ChVector<>(0.02, 0.2, 0.1) );
	body_truss->AddAsset(mboxtruss);



	// Create body for crank


	ChSharedPtr<ChBody>  body_crank(new ChBody);

	body_crank->SetPos( (vB+vG)*0.5 );

	my_system.AddBody(body_crank);

				// Attach a 'box' shape asset for visualization.
	ChSharedPtr<ChBoxShape> mboxcrank(new ChBoxShape);
	mboxcrank->GetBoxGeometry().Pos  = ChVector<>(0,0,0);
	mboxcrank->GetBoxGeometry().SetLenghts( ChVector<>(K,0.02,0.02) );
	body_crank->AddAsset(mboxcrank);


				// Create a motor between the truss
				// and the crank:
	ChSharedPtr<ChLinkEngine> constr_motor(new ChLinkEngine);
	constr_motor->Initialize(body_truss,
						     body_crank,
							 ChCoordsys<> (vG ) );
	my_system.Add(constr_motor);

	class ChFunction_myf : public ChFunction
	{
	public:
		ChFunction* new_Duplicate() {return new ChFunction_myf;} 

		double Get_y(double x) 
			{
					if (x > 0.4) 
						return CH_C_PI; 
					else 
						return - CH_C_PI * (1.0 - cos(CH_C_PI*x/0.4)) / 2.0;
			} 
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
	msection1->SetGwithPoissonRatio(0.3);
	msection1->SetAsRectangularSection(beam_wy, beam_wz);
	msection1->SetBeamRaleyghDamping(0.000);

				// This helps creating sequences of nodes and ChElementBeamEuler elements:
	ChBuilderBeam builder;

	builder.BuildBeam(	my_mesh,		// the mesh where to put the created nodes and elements 
						msection1,		// the ChBeamSectionAdvanced to use for the ChElementBeamEuler elements
						16,				// the number of ChElementBeamEuler to create
						vA,	// the 'A' point in space (beginning of beam)
						vC,	// the 'B' point in space (end of beam)
						ChVector<>(0,1, 0));		// the 'Y' up direction of the section for the beam
	
				// After having used BuildBeam(), you can retrieve the nodes used for the beam,
				// For example say you want to fix the A end and apply a force to the B end:
	builder.GetLastBeamNodes().front()->SetFixed(true);

	ChSharedPtr<ChNodeFEMxyzrot> node_tip = builder.GetLastBeamNodes().back();
	ChSharedPtr<ChNodeFEMxyzrot> node_mid = builder.GetLastBeamNodes()[7];

		// Create the vertical beam

	ChSharedPtr<ChBeamSectionAdvanced> msection2(new ChBeamSectionAdvanced);

	double hbeam_d = 0.024;
	msection2->SetDensity(2700); 
	msection2->SetYoungModulus (73.0e9);
	msection1->SetGwithPoissonRatio(0.3);
	msection2->SetBeamRaleyghDamping(0.000);
	msection2->SetAsCircularSection(hbeam_d);

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
										false, false, false);   // Rx, Ry, Rz

				// For example, attach small shape to show the constraint
	ChSharedPtr<ChSphereShape> msphereconstr2(new ChSphereShape);
	msphereconstr2->GetSphereGeometry().rad = 0.01;
	constr_bb->AddAsset(msphereconstr2);


				// Create a beam as a crank

	ChSharedPtr<ChBeamSectionAdvanced> msection3(new ChBeamSectionAdvanced);

	double crankbeam_d = 0.048;
	msection3->SetDensity(2700); 
	msection3->SetYoungModulus (73.0e9);
	msection3->SetGwithPoissonRatio(0.3);
	msection3->SetBeamRaleyghDamping(0.000);
	msection3->SetAsCircularSection(crankbeam_d);

	builder.BuildBeam(	my_mesh,		// the mesh where to put the created nodes and elements 
						msection3,		// the ChBeamSectionAdvanced to use for the ChElementBeamEuler elements
						3,				// the number of ChElementBeamEuler to create
						vG+vd,			// the 'A' point in space (beginning of beam)
						vB+vd,			// the 'B' point in space (end of beam)
						ChVector<>(0,1, 0));	// the 'Y' up direction of the section for the beam
	
	ChSharedPtr<ChNodeFEMxyzrot> node_crankG  = builder.GetLastBeamNodes().front();
	ChSharedPtr<ChNodeFEMxyzrot> node_crankB  = builder.GetLastBeamNodes().back();


		// Create a constraint between the crank beam and body crank:

	ChSharedPtr<ChLinkMateGeneric> constr_cbd(new ChLinkMateGeneric);
	constr_cbd->Initialize( node_crankG,
							body_crank,
							false, 
							node_crankG->Frame(),
							node_crankG->Frame() 
							 );
	my_system.Add(constr_cbd);

	constr_cbd->SetConstrainedCoords( true, true, true,		// x, y, z
									  true, true, true);   // Rx, Ry, Rz

		// Create a constraint between the vertical beam and the crank beam:

	ChSharedPtr<ChLinkMateGeneric> constr_bc(new ChLinkMateGeneric);
	constr_bc->Initialize(  node_down,
							node_crankB,
							false, 
							node_crankB->Frame(),
							node_crankB->Frame() 
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
	my_system.SetTolSpeeds(1e-20);
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


	application.SetTimestep(0.0005);
	application.SetVideoframeSaveInterval(10);

	// Output data
	chrono::ChStreamOutAsciiFile file_out1("benchmark_CE_buckling_mid.dat");

	while(application.GetDevice()->run()) 
	{
		//builder.GetLastBeamNodes().back()->SetTorque(ChVector<> (0,0,0.1*application.GetSystem()->GetChTime() ) );

		application.BeginScene();

		application.DrawAll();
		
		ChIrrTools::drawGrid(application.GetVideoDriver(), 0.05, 0.05, 20, 20, ChCoordsys<>(VNULL, CH_C_PI_2, VECT_Z), video::SColor(50,90,90,90), true );

		application.DoStep();

		file_out1 << application.GetSystem()->GetChTime() << " " << node_mid->GetPos().z << " " << node_mid->GetWvel_par().x << "\n";
		if (application.GetSystem()->GetChTime() > 0.4) 
			break;

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
	ChIrrApp application(&my_system, L"Statics of beam",core::dimension2d<u32>(800,600),false, true);

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
	mboxtruss->GetBoxGeometry().Pos  = ChVector<>(-0.01, 0, 0.4);
	mboxtruss->GetBoxGeometry().SetLenghts( ChVector<>(0.02, 0.2, 0.8) );
	my_body_A->AddAsset(mboxtruss);


		// Create a FEM mesh, that is a container for groups
		// of elements and their referenced nodes.
	ChSharedPtr<ChMesh> my_mesh(new ChMesh);


	double rotstep = 15;
	double rotmax  = 90;

	ChMatrixNM<double,3,1> loads;
	loads(0) = -4.448;
	loads(1) = -8.896;
	loads(2) = -13.345;

	double z_spacing = -0.1;
	double y_spacing = -0.2;

	std::vector< ChSharedPtr< ChNodeFEMxyzrot > > endnodes[3];



	for (int nload = 0; nload < 3; ++nload)
	{
		int i = 0;
		for (int rot = 0; rot <= rotmax; rot+= rotstep)
		{
			double rot_rad = rot * CH_C_DEG_TO_RAD;


			//
			// Add some EULER-BERNOULLI BEAMS:
			//

						// Create a section, i.e. thickness and material properties
						// for beams. This will be shared among some beams.
			
			ChSharedPtr<ChBeamSectionAdvanced> msection(new ChBeamSectionAdvanced);

			double beam_wz = 0.0032024; //3.175;
			double beam_wy = 0.01237; //12.7;
			double beam_L  = 0.508;
			msection->SetDensity(2700); 
			msection->SetYoungModulus (71.7e9);
			msection->SetGwithPoissonRatio(0.31);
			msection->SetBeamRaleyghDamping(0.0);
			msection->SetAsRectangularSection(beam_wy, beam_wz);


						// This helps creating sequences of nodes and ChElementBeamEuler elements:
			ChBuilderBeam builder;

			builder.BuildBeam(	my_mesh,		// the mesh where to put the created nodes and elements 
								msection,		// the ChBeamSectionAdvanced to use for the ChElementBeamEuler elements
								10,				// the number of ChElementBeamEuler to create
								ChVector<>(0,      nload*y_spacing, i*z_spacing),	// the 'A' point in space (beginning of beam)
								ChVector<>(beam_L, nload*y_spacing, i*z_spacing),	// the 'B' point in space (end of beam)
								ChVector<>(0, 1, 0)
								//ChVector<>(0, cos(rot_rad), sin(rot_rad))
								);		// the 'Y' up direction of the section for the beam
			
						// After having used BuildBeam(), you can retrieve the nodes used for the beam,
						// For example say you want to fix the A end and apply a force to the B end:
			builder.GetLastBeamNodes().front()->SetFixed(true);

			//builder.GetLastBeamNodes().back()->SetForce(ChVector<> (0, load,0));
			builder.GetLastBeamNodes().back()->SetForce(ChVector<> (0, loads(nload)*cos(rot_rad), loads(nload)*sin(rot_rad)));

			endnodes[nload].push_back( builder.GetLastBeamNodes().back() );

			++i;
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
	msolver->SetDiagonalPreconditioning(true);

//my_system.SetLcpSolverType(ChSystem::LCP_SIMPLEX);

//***TEST***
ChMatlabEngine matlab_engine;
ChLcpMatlabSolver* matlab_solver_stab  = new ChLcpMatlabSolver(matlab_engine);
ChLcpMatlabSolver* matlab_solver_speed = new ChLcpMatlabSolver(matlab_engine);
my_system.ChangeLcpSolverStab (matlab_solver_stab);
my_system.ChangeLcpSolverSpeed(matlab_solver_speed);

	application.SetTimestep(0.001);
	application.SetVideoframeSaveInterval(10);

	// Perform nonlinear statics
	my_system.DoStaticNonlinear(20);
	my_system.DoStaticNonlinear(10);  // just to be on the safe side :)
	application.SetPaused(true);


	// Output data
	chrono::ChStreamOutAsciiFile file_out1("benchmark_CE_princeton_L1.dat");
	for (int i = 0; i < endnodes[0].size() ; ++i)
	{

		double node_y = endnodes[0][i]->GetPos().y - 0 * y_spacing;
		double node_z = endnodes[0][i]->GetPos().z - i * z_spacing;
		double node_a = atan2( endnodes[0][i]->GetA()->Get_A_Yaxis().y, endnodes[0][i]->GetA()->Get_A_Yaxis().z ) - CH_C_PI_2;
		GetLog() << " Node " << i << " DY=" << node_y << " DZ=" << node_z << "  angle=" << node_a << " [rad]\n";
		file_out1 << node_y << " " << node_z << " " << node_a << "\n";
	}
	chrono::ChStreamOutAsciiFile file_out2("benchmark_CE_princeton_L2.dat");
	for (int i = 0; i < endnodes[1].size() ; ++i)
	{
		double node_y = endnodes[1][i]->GetPos().y - 1 * y_spacing;
		double node_z = endnodes[1][i]->GetPos().z - i * z_spacing;
		double node_a = atan2( endnodes[1][i]->GetA()->Get_A_Yaxis().y, endnodes[1][i]->GetA()->Get_A_Yaxis().z ) - CH_C_PI_2;
		GetLog() << " Node " << i << " DY=" << node_y << " DZ=" << node_z << "  angle=" << node_a << " [rad]\n";
		file_out2 << node_y << " " << node_z << " " << node_a << "\n";
	}
	chrono::ChStreamOutAsciiFile file_out3("benchmark_CE_princeton_L3.dat");
	for (int i = 0; i < endnodes[2].size() ; ++i)
	{
		double node_y = endnodes[2][i]->GetPos().y - 2 * y_spacing;
		double node_z = endnodes[2][i]->GetPos().z - i * z_spacing;
		double node_a = atan2( endnodes[2][i]->GetA()->Get_A_Yaxis().y, endnodes[2][i]->GetA()->Get_A_Yaxis().z ) - CH_C_PI_2;
		GetLog() << " Node " << i << " DY=" << node_y << " DZ=" << node_z << "  angle=" << node_a << " [rad]\n";
		file_out3 << node_y << " " << node_z << " " << node_a << "\n";
	}


	// 3D view

	while(application.GetDevice()->run()) 
	{

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


