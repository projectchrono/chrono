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
//     - FEM for 3D beams

  
     
// Include some headers used by this tutorial...

#include "physics/ChSystem.h"
#include "physics/ChLinkMate.h"
#include "lcp/ChLcpIterativePMINRES.h"
#include "lcp/ChLcpIterativeMINRES.h"
#include "unit_FEM/ChElementBeamEuler.h"
#include "unit_FEM/ChBuilderBeam.h"
#include "unit_FEM/ChMesh.h"
#include "unit_FEM/ChVisualizationFEMmesh.h"
#include "unit_IRRLICHT/ChIrrApp.h"
#include "unit_MATLAB/ChMatlabEngine.h"
#include "unit_MATLAB/ChLcpMatlabSolver.h"

// Remember to use the namespace 'chrono' because all classes 
// of Chrono::Engine belong to this namespace and its children...

using namespace chrono;
using namespace fem;
using namespace irr;



int main(int argc, char* argv[])
{
	// Create a Chrono::Engine physical system
	ChSystem my_system;


	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrApp application(&my_system, L"Beams FEM",core::dimension2d<u32>(800,600),false, true);

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	application.AddTypicalLogo();
	application.AddTypicalSky();
	application.AddTypicalLights();
	application.AddTypicalCamera(core::vector3df(0.f, 0.6f, -1.f));



				// Create a mesh, that is a container for groups
				// of elements and their referenced nodes.
	ChSharedPtr<ChMesh> my_mesh(new ChMesh);
	

				// Create a section, i.e. thickness and material properties
				// for beams. This will be shared among some beams.
	
	ChSharedPtr<ChBeamSectionAdvanced> msection(new ChBeamSectionAdvanced);

	double beam_wy = 0.012;
	double beam_wz = 0.025;
	msection->SetAsRectangularSection(beam_wy, beam_wz);
	msection->SetYoungModulus (0.01e9);
	msection->SetGshearModulus(0.01e9 * 0.3);
	msection->SetBeamRaleyghDamping(0.000);
	//msection->SetCentroid(0,0.02); 
	//msection->SetShearCenter(0,0.1); 
	//msection->SetSectionRotation(45*CH_C_RAD_TO_DEG);


	//
	// Add some EULER-BERNOULLI BEAMS:
	//

	double beam_L  = 0.1;
	

	ChSharedPtr<ChNodeFEMxyzrot> hnode1(new ChNodeFEMxyzrot( ChFrame<>(ChVector<>(0,0,0)) )); //, Q_from_AngAxis( -0.5, VECT_Y )) ));
	ChSharedPtr<ChNodeFEMxyzrot> hnode2(new ChNodeFEMxyzrot( ChFrame<>(ChVector<>(beam_L,0,0)) ));
	ChSharedPtr<ChNodeFEMxyzrot> hnode3(new ChNodeFEMxyzrot( ChFrame<>(ChVector<>(beam_L*2,0,0)) ));

	my_mesh->AddNode(hnode1);
	my_mesh->AddNode(hnode2);
	my_mesh->AddNode(hnode3);

	ChSharedPtr<ChElementBeamEuler> belement1 (new ChElementBeamEuler);

	belement1->SetNodes(hnode1, hnode2);
	belement1->SetSection(msection);

	my_mesh->AddElement(belement1);


	ChSharedPtr<ChElementBeamEuler> belement2 (new ChElementBeamEuler);

	belement2->SetNodes(hnode2, hnode3);
	belement2->SetSection(msection);

	my_mesh->AddElement(belement2);


				// Apply a force or a torque to a node:
	hnode2->SetForce( ChVector<>(4,2,0));
	//hnode3->SetTorque( ChVector<>(0, -0.04, 0));


				// Fix a node to ground:
	//hnode1->SetFixed(true);
	ChSharedPtr<ChBody> mtruss(new ChBody);
	mtruss->SetBodyFixed(true);
	my_system.Add(mtruss);

	ChSharedPtr<ChLinkMateGeneric> constr_bc(new ChLinkMateGeneric);
	constr_bc->Initialize(  hnode3,
							mtruss,
							false, 
							hnode3->Frame(),
							hnode3->Frame() 
							 );
	my_system.Add(constr_bc);
	constr_bc->SetConstrainedCoords( true, true, true,	  // x, y, z
									 true, true, true);   // Rx, Ry, Rz

	ChSharedPtr<ChLinkMateGeneric> constr_d(new ChLinkMateGeneric);
	constr_d->Initialize(  hnode1,
							mtruss,
							false, 
							hnode1->Frame(),
							hnode1->Frame() 
							 );
	my_system.Add(constr_d);
	constr_d->SetConstrainedCoords( false, true, true,	  // x, y, z
									 false, false, false);   // Rx, Ry, Rz

/*	
	//
	// Add some EULER-BERNOULLI BEAMS (the fast way!)
	//

				// Shortcut!
				// This ChBuilderBeam helper object is very useful because it will 
				// subdivide 'beams' into sequences of finite elements of beam type, ex.
				// one 'beam' could be made of 5 FEM elements of ChElementBeamEuler class.
				// If new nodes are needed, it will create them for you.
	ChBuilderBeam builder;

				// Now, simply use BuildBeam to create a beam from a point to another: 
	builder.BuildBeam(	my_mesh,		// the mesh where to put the created nodes and elements 
						msection,		// the ChBeamSectionAdvanced to use for the ChElementBeamEuler elements
						5,				// the number of ChElementBeamEuler to create
						ChVector<>(0, 0, -0.1),		// the 'A' point in space (beginning of beam)
						ChVector<>(0.2, 0, -0.1),	// the 'B' point in space (end of beam)
						ChVector<>(0,1,0));			// the 'Y' up direction of the section for the beam
	
				// After having used BuildBeam(), you can retrieve the nodes used for the beam,
				// For example say you want to fix the A end and apply a force to the B end:
	builder.GetLastBeamNodes().back()->SetFixed(true);
	builder.GetLastBeamNodes().front()->SetForce( ChVector<>(0,-1,0));

				// Again, use BuildBeam for creating another beam, this time
				// it uses one node (the last node created by the last beam) and one point:
 	builder.BuildBeam(  my_mesh, 
						msection, 
						5, 
						builder.GetLastBeamNodes().front(), // the 'A' node in space (beginning of beam)
						ChVector<>(0.2, 0.1, -0.1),	// the 'B' point in space (end of beam)
						ChVector<>(0,1,0));			// the 'Y' up direction of the section for the beam

*/



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


	/*
	ChSharedPtr<ChVisualizationFEMmesh> mvisualizebeamA(new ChVisualizationFEMmesh(*(my_mesh.get_ptr())));
	mvisualizebeamA->SetFEMdataType(ChVisualizationFEMmesh::E_PLOT_SURFACE);
	mvisualizebeamA->SetSmoothFaces(true);
	my_mesh->AddAsset(mvisualizebeamA);
	*/

	ChSharedPtr<ChVisualizationFEMmesh> mvisualizebeamA(new ChVisualizationFEMmesh(*(my_mesh.get_ptr())));
	mvisualizebeamA->SetFEMdataType(ChVisualizationFEMmesh::E_PLOT_ELEM_BEAM_MZ);
	mvisualizebeamA->SetColorscaleMinMax(-0.4,0.4);
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
	my_system.SetIterLCPmaxItersSpeed(460);
	my_system.SetIterLCPmaxItersStab(460);
	my_system.SetTolForce(1e-13);
	chrono::ChLcpIterativeMINRES* msolver = (chrono::ChLcpIterativeMINRES*)my_system.GetLcpSolverSpeed();
	msolver->SetVerbose(false);
	msolver->SetDiagonalPreconditioning(true);
	
	application.SetTimestep(0.001);

// TEST: The Matlab external linear solver, for max precision in benchmarks
ChMatlabEngine matlab_engine;
ChLcpMatlabSolver* matlab_solver_stab  = new ChLcpMatlabSolver(matlab_engine);
ChLcpMatlabSolver* matlab_solver_speed = new ChLcpMatlabSolver(matlab_engine);
my_system.ChangeLcpSolverStab (matlab_solver_stab);
my_system.ChangeLcpSolverSpeed(matlab_solver_speed);

//application.GetSystem()->Update();
application.SetPaused(true);






GetLog()<< "\n\n\n===========STATICS======== \n\n\n";

//application.GetSystem()->DoStaticNonlinear(25);
//application.GetSystem()->DoStaticLinear();

/*
belement1->SetDisableCorotate(true);
belement2->SetDisableCorotate(true);
application.GetSystem()->DoStaticLinear();


GetLog() << "BEAM RESULTS (STATIC ANALYSIS) \n\n";

ChVector<> F, M;
ChMatrixDynamic<> displ;

belement1->GetField(displ);
GetLog()<< displ;
for (double eta = -1; eta <= 1; eta += 0.4)
{	
	belement1->EvaluateSectionForceTorque(eta, displ, F, M);
	GetLog() << "  b1_at " << eta <<  " Mx=" << M.x << " My=" << M.y << " Mz=" << M.z << " Tx=" << F.x << " Ty=" << F.y << " Tz=" << F.z << "\n";
}
GetLog()<< "\n";
belement2->GetField(displ);
for (double eta = -1; eta <= 1; eta += 0.4)
{	
	belement2->EvaluateSectionForceTorque(eta, displ, F, M);
	GetLog() << "  b2_at " << eta <<  " Mx=" << M.x << " My=" << M.y << " Mz=" << M.z << " Tx=" << F.x << " Ty=" << F.y << " Tz=" << F.z << "\n";
}

GetLog() << "Node 3 coordinate x= " << hnode3->Frame().GetPos().x << "    y=" << hnode3->Frame().GetPos().y << "    z=" << hnode3->Frame().GetPos().z << "\n";

belement1->SetDisableCorotate(false);
belement2->SetDisableCorotate(false);
*/

	while(application.GetDevice()->run()) 
	{
		application.BeginScene();

		application.DrawAll();
		
		application.DoStep();

		application.EndScene();

		//GetLog() << " node pos =" << hnode3->Frame().GetPos() << "\n";
	}


	return 0;
}


