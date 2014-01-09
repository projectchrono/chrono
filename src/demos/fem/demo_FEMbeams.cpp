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
#include "lcp/ChLcpIterativePMINRES.h"
#include "unit_FEM/ChElementBeamEuler.h"
#include "unit_FEM/ChMesh.h"
//#include "unit_FEM/ChNodeBody.h"
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
	double beam_wz = 0.01;

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
	belement1->SetBeamRaleyghDamping(0.001);

	my_mesh->AddElement(belement1);



	ChSharedPtr<ChElementBeamEuler> belement2 (new ChElementBeamEuler);

	belement2->SetNodes(hnode2, hnode3);

	belement2->SetAsRectangularSection(beam_wy, beam_wz);
	belement2->SetYoungModulus (0.01e9);
	belement2->SetGshearModulus(0.01e9 * 0.3);
	belement2->SetBeamRaleyghDamping(0.001);

	my_mesh->AddElement(belement2);


				// For example, set an initial displacement to a node:
//	hnode2->SetX0( hnode2->GetX0() >> ChFrame<>(ChVector<>(0,0.01,0)) );

				// Apply a force to a node
	hnode3->SetForce( ChVector<>(0,-3,0));


	//
	// Final touches..
	// 
				// This is necessary in order to precompute the 
				// stiffness matrices for all inserted elements in mesh
	my_mesh->SetupInitial();

				// Remember to add the mesh to the system!
	my_system.Add(my_mesh);

				// Fix a body to ground
	hnode1->SetFixed(true);


			// ==Asset== attach a visualization of the FEM mesh.
			// This will automatically update a triangle mesh (a ChTriangleMeshShape
			// asset that is internally managed) by setting  proper
			// coordinates and vertex colours as in the FEM elements.
			// Such triangle mesh can be rendered by Irrlicht or POVray or whatever
			// postprocessor that can handle a coloured ChTriangleMeshShape).
			// Do not forget AddAsset() at the end!


	ChSharedPtr<ChVisualizationFEMmesh> mvisualizemeshC(new ChVisualizationFEMmesh(*(my_mesh.get_ptr())));
	mvisualizemeshC->SetFEMglyphType(ChVisualizationFEMmesh::E_GLYPH_NODE_CSYS);
	mvisualizemeshC->SetFEMdataType(ChVisualizationFEMmesh::E_PLOT_NONE);
	mvisualizemeshC->SetSymbolsThickness(0.006);
	mvisualizemeshC->SetSymbolsScale(0.01);
	my_mesh->AddAsset(mvisualizemeshC);



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
	my_system.SetIterLCPmaxItersSpeed(12);
	my_system.SetTolSpeeds(1e-5);
	chrono::ChLcpIterativePMINRES* msolver = (chrono::ChLcpIterativePMINRES*)my_system.GetLcpSolverSpeed();
	//msolver->SetVerbose(true);
	msolver->SetDiagonalPreconditioning(true);

//my_system.DoStaticLinear();

	application.SetTimestep(0.001);

	while(application.GetDevice()->run()) 
	{
		application.BeginScene();

		application.DrawAll();
		
//my_system.DoStaticLinear();
		application.DoStep();

		//GetLog() << " t =" << my_system.GetChTime() << "  hnode2 pos =" << hnode2->Frame().GetPos() << "\n";

		application.EndScene();

	}



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

	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}


