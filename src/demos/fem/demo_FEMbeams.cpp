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

#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"
#include "physics/ChLinkMate.h"
#include "lcp/ChLcpIterativePMINRES.h"
#include "unit_FEM/ChElementBeamEuler.h"
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
	ChIrrApp application(&my_system, L"Beams FEM",core::dimension2d<u32>(800,600),false, true);

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	application.AddTypicalLogo();
	application.AddTypicalSky();
	application.AddTypicalLights();
	application.AddTypicalCamera(core::vector3df(0.f, 0.6f, -1.f));



				// Create a mesh, that is a container for groups
				// of elements and their referenced nodes.
	ChSharedPtr<ChMesh> my_mesh(new ChMesh);
	


	//
	// Add some EULER-BERNOULLI BEAMS:
	//

	double beam_L  = 0.1;
	double beam_wy = 0.02;
	double beam_wz = 0.02;
/*
	ChSharedPtr<ChNodeFEMxyzrot> hnode1(new ChNodeFEMxyzrot( ChFrame<>(ChVector<>(0,0,0)) ));
	ChSharedPtr<ChNodeFEMxyzrot> hnode2(new ChNodeFEMxyzrot( ChFrame<>(ChVector<>(beam_L,0,0)) ));
	ChSharedPtr<ChNodeFEMxyzrot> hnode3(new ChNodeFEMxyzrot( ChFrame<>(ChVector<>(beam_L*2,0,0)) ));

	my_mesh->AddNode(hnode1);
	my_mesh->AddNode(hnode2);
	my_mesh->AddNode(hnode3);

	ChSharedPtr<ChElementBeamEuler> belement1 (new ChElementBeamEuler);

	belement1->SetNodes(hnode1, hnode2);

	belement1->SetAsRectangularSection(beam_wy, beam_wz);
	belement1->SetYoungModulus (0.01e9);
	belement1->SetGshearModulus(0.01e9 * 0.3);
	belement1->SetBeamRaleyghDamping(0.000);

	my_mesh->AddElement(belement1);



	ChSharedPtr<ChElementBeamEuler> belement2 (new ChElementBeamEuler);

	belement2->SetNodes(hnode2, hnode3);

	belement2->SetAsRectangularSection(beam_wy, beam_wz);
	belement2->SetYoungModulus (0.01e9);
	belement2->SetGshearModulus(0.01e9 * 0.3);
	belement2->SetBeamRaleyghDamping(0.000);

	my_mesh->AddElement(belement2);


				// Apply a force to a node
	//hnode3->SetForce( ChVector<>(0, -1, 0));
	//hnode2->SetForce( ChVector<>(0,-6,0));
	//hnode3->SetForce( ChVector<>(0,-3,-3));
	//hnode3->SetTorque( ChVector<>(0, 0, 0.4));



				// Fix a body to ground
	hnode1->SetFixed(true);
*/

/*
				// Create also a truss
				ChSharedPtr<ChBody> truss(new ChBody);
				truss->SetBodyFixed(true);
				my_system.Add(truss);

				ChSharedPtr<ChLinkMateGeneric> constraint(new ChLinkMateGeneric);
				ChFrame<> mfra;
				constraint->Initialize(hnode1,
										truss,false, mfra, mfra);
				my_system.Add(constraint);

						// For example, attach small cube to show the constraint
				ChSharedPtr<ChBoxShape> mboxfloor(new ChBoxShape);
				mboxfloor->GetBoxGeometry().Size = ChVector<>(0.005);
				constraint->AddAsset(mboxfloor);
*/


	ChSharedPtr<ChNodeFEMxyzrot> hnode4(new ChNodeFEMxyzrot( ChFrame<>(ChVector<>(2*beam_L,0,0), -CH_C_PI_2, VECT_Y )));
	ChSharedPtr<ChNodeFEMxyzrot> hnode5(new ChNodeFEMxyzrot( ChFrame<>(ChVector<>(2*beam_L,0,beam_L),  -CH_C_PI_2, VECT_Y ) ));
//ChSharedPtr<ChNodeFEMxyzrot> hnode4(new ChNodeFEMxyzrot( ChFrame<>(ChVector<>(2*beam_L,0,0) ) ));
//ChSharedPtr<ChNodeFEMxyzrot> hnode5(new ChNodeFEMxyzrot( ChFrame<>(ChVector<>(2*beam_L+beam_L,0,0) ) ));

	my_mesh->AddNode(hnode4);
	my_mesh->AddNode(hnode5);

	ChSharedPtr<ChElementBeamEuler> belement3 (new ChElementBeamEuler);

	belement3->SetNodes(hnode4, hnode5);

	belement3->SetAsRectangularSection(beam_wy, beam_wz);
	belement3->SetYoungModulus (0.01e9);
	belement3->SetGshearModulus(0.01e9 * 0.3);
	belement3->SetBeamRaleyghDamping(0.000);

	my_mesh->AddElement(belement3);

	belement3->UpdateRotation();
	
hnode4->SetFixed(true);
hnode5->SetForce( ChVector<>(0,-8,0));
//hnode5->SetTorque( ChVector<>(1,0,0));



/*
	ChSharedPtr<ChLinkMateGeneric> constraint2(new ChLinkMateGeneric);
	ChFrame<> mfra2;
	constraint2->Initialize(hnode3,
						    hnode4,false, hnode3->Frame(), hnode3->Frame());
	my_system.Add(constraint2);

	// For example, attach small cube to show the constraint
	ChSharedPtr<ChBoxShape> mboxconstr2(new ChBoxShape);
	mboxconstr2->GetBoxGeometry().SetLenghts( ChVector<>(beam_wy*1.05) );
	constraint2->AddAsset(mboxconstr2);
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
	mvisualizebeamA->SetFEMdataType(ChVisualizationFEMmesh::E_PLOT_SURFACE); //E_PLOT_ELEM_BEAM_TZ);
	mvisualizebeamA->SetColorscaleMinMax(-6,6);
	mvisualizebeamA->SetSmoothFaces(true);
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

	my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_PMINRES); // <- NEEDED because other solvers can't handle stiffness matrices
	my_system.SetIterLCPwarmStarting(true); // this helps a lot to speedup convergence in this class of problems
	my_system.SetIterLCPmaxItersSpeed(50);
	my_system.SetTolSpeeds(1e-5);
	//chrono::ChLcpIterativePMINRES* msolver = (chrono::ChLcpIterativePMINRES*)my_system.GetLcpSolverSpeed();
	//msolver->SetVerbose(true);
	//msolver->SetDiagonalPreconditioning(true);

	application.SetTimestep(0.001);
application.GetSystem()->Update();
application.SetPaused(true);

		// Impose displ.
//hnode5->Frame().SetPos( hnode5->Frame().GetPos() + ChVector<>(0,-0.1, 0) );
		// Impose rot.
/*
ChFrame<>* mf = &hnode5->Frame();
mf->ConcatenatePostTransformation( ChFrame<>(ChVector<>(0,0,0),  CH_C_DEG_TO_RAD*30.0, VECT_Z ) );
ChMatrixDynamic<> mfi(12,1);
belement3->GetField(mfi);
*/

/*
ChMatrixDynamic<> mfo(12,1);
belement3->ComputeInternalForces(mfo);
*/

GetLog()<< "\n===========STATICS======== \n\n";
//for (int is = 0; is<10; ++is)
application.GetSystem()->DoStaticLinear();

ChMatrixDynamic<> mfo(12,1);
belement3->ComputeInternalForces(mfo);

	while(application.GetDevice()->run()) 
	{
		application.BeginScene();

		application.DrawAll();
		
		application.DoStep();

		application.EndScene();

		//GetLog() << " node pos =" << hnode3->Frame().GetPos() << "\n";
	}

mfo(12,1);
belement3->ComputeInternalForces(mfo);
	/*
	//***TEST***

	ChVector<> delta_rot_dir;
	double     delta_rot_angle;
	hnode2->Frame().GetRot().Q_to_AngAxis(delta_rot_angle, delta_rot_dir);
	GetLog() << " el.inc.rot =" << (delta_rot_angle*delta_rot_dir);

	GetLog() << " el.inc.rot (deg)=" << (delta_rot_angle*delta_rot_dir) * ::CH_C_RAD_TO_DEG;

	ChMatrixDynamic<double> field(12,1);
	belement1->GetField(field);
	for (int r = 0; r<6; ++r)
		GetLog() << " el.fieldA =" << field(r) << "\n";
	for (int r = 6; r<12; ++r)
		GetLog() << " el.fieldB =" << field(r) << "\n";
	GetLog() << "\n";

	ChMatrixDynamic<double> iforces(12,1);
	belement1->ComputeInternalForces(iforces);
	for (int r = 0; r<6; ++r)
		GetLog() << " el.iforcesA =" << field(r) << "\n";
	for (int r = 6; r<12; ++r)
		GetLog() << " el.iforcesB =" << field(r) << "\n";
	*/


	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}


