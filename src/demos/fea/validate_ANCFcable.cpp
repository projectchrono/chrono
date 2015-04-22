//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

//  Code to run validation study of 3D ANCF gradient deficient beams as cable elements,
//   based on Alessandro's demo_FEAcables.
//  Validation based on comparison with (Sugiyama, Mikkola, Shabana 2003),
//  Section 4 results, including:
//  a) transverse deflection
//  b) deformed cable shapes
//  c) vertical tip displacement
//  d) Energy Balance (potential, kinetic, strain components)
//

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

using namespace chrono;
using namespace fea;
using namespace irr;

int main(int argc, char* argv[])
{
	// simulation and system settings
	const double gravity = 10.0;	// [m/s2]
	const double time_step = 0.01;
	const double hht_alpha = -0.2;
	const double hht_max_iters = 2;
	const double hht_tol = 1e-6;

	// ChSystem and Irrlicht application
	ChSystem my_system;
	ChIrrApp application(&my_system, L"Cables FEM",core::dimension2d<u32>(800,600),false, true);
	application.AddTypicalLogo();
	application.AddTypicalSky();
	application.AddTypicalLights();
	application.AddTypicalCamera(core::vector3df(0.f, 0.6f, -1.f));

	ChSharedPtr<ChMesh> my_mesh(new ChMesh);
	// ANCF Cable parameters, page 750 of reference.
	const double beam_L  = 1.0;	// [m]
	const double span_ratio = 7.3;	// length/diameter
	const double beam_diameter = beam_L / span_ratio;
  const double EI = 2.15E-4;	// [N/m2]
	const double EA = 1.26E4;	// [N]
  const double area = beam_diameter * beam_diameter / 4.0;  // cross section area [m2]
	const double youngs_mod = EA / area;	// [N/m2]
  const double area_I = EI / youngs_mod;
  const double rho = 2.7e3;  // material density [kg/m3]
  const double damping = 0.0; // rayleigh damping coef.
	
  ChSharedPtr<ChBeamSectionCable> msection_cable(new ChBeamSectionCable(youngs_mod, beam_diameter, rho, area_I, damping) );
	
	ChBuilderBeamANCF builder;
	builder.BuildBeam(my_mesh, msection_cable, 10, // the number of ChElementBeamANCF to create
						ChVector<>(0, 0, 0),		// beginning of beam
						ChVector<>(beam_L, 0, 0));	// end of beam
	
	// ground body
  ChSharedPtr<ChBody> ground (new ChBody);
  ground->SetBodyFixed(true);
  // attach the cable to the ground at the origin
  ChSharedPtr<ChLinkPointFrame> constraint_hinge(new ChLinkPointFrame);
	constraint_hinge->Initialize(builder.GetLastBeamNodes().front(), ground);
	my_system.Add(constraint_hinge);

	// precompute the stiffness matrices for all inserted elements in mesh
	my_mesh->SetupInitial();
	my_system.Add(my_mesh);

	// Create and attach visualization assets to the mesh.
	// Automatically update a triangle mesh by setting 
	// coordinates and vertex colours as in the elements.
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

	application.AssetBindAll();
	application.AssetUpdateAll();

	// solver settings
	my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_MINRES); // <- NEEDED THIS OR ::LCP_SIMPLEX because other solvers can't handle stiffness matrices
	my_system.SetIterLCPwarmStarting(true); // this helps a lot to speedup convergence in this class of problems
	my_system.SetIterLCPmaxItersSpeed(200);
	my_system.SetIterLCPmaxItersStab(200);
	// my_system.SetTolForce(1e-13);
	chrono::ChLcpIterativeMINRES* msolver = (chrono::ChLcpIterativeMINRES*)my_system.GetLcpSolverSpeed();
	msolver->SetVerbose(false);
	msolver->SetDiagonalPreconditioning(true);

	// Integrator type
    //my_system.SetIntegrationType(chrono::ChSystem::INT_EULER_IMPLICIT_LINEARIZED);  // fast, less precise
	my_system.SetIntegrationType(chrono::ChSystem::INT_HHT);  // precise,slower, might iterate each step
	// if later you want to change integrator settings:
	if( ChSharedPtr<ChTimestepperHHT> mystepper = my_system.GetTimestepper().DynamicCastTo<ChTimestepperHHT>() )
	{
		mystepper->SetAlpha(hht_alpha);
		mystepper->SetMaxiters(hht_max_iters);
		mystepper->SetTolerance(hht_tol);
	}
	 
	application.SetTimestep(time_step);

	// simulation time-stepping loop
	while(application.GetDevice()->run()) 
	{
		application.BeginScene();
		application.DrawAll();
		application.DoStep();
		application.EndScene();
	}

	return 0;
}

