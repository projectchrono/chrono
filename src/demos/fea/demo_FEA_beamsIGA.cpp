// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================
//
// FEA for 3D beams
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/solver/ChSolverPMINRES.h"
#include "chrono/solver/ChSolverMINRES.h"

#include "chrono_fea/ChElementBeamIGA.h"
#include "chrono_fea/ChBuilderBeam.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChLinkDirFrame.h"

#include "chrono_irrlicht/ChIrrApp.h"

#define USE_MKL

#ifdef USE_MKL
    #include "chrono_mkl/ChSolverMKL.h"
#endif

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

using namespace irr;

int main(int argc, char* argv[]) {
	GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

	// Create a Chrono::Engine physical system
	ChSystemNSC my_system;


	// Create a mesh, that is a container for groups
	// of elements and their referenced nodes.
	auto my_mesh = std::make_shared<ChMesh>();

	// Create a section, i.e. thickness and material properties
	// for beams. This will be shared among some beams.

	auto msection = std::make_shared<ChBeamSectionAdvanced>();

	double beam_wy = 0.012;
	double beam_wz = 0.025;
	msection->SetAsRectangularSection(beam_wy, beam_wz);
	msection->SetYoungModulus(0.02e10);
	msection->SetGshearModulus(0.02e10 * 0.3);
	msection->SetBeamRaleyghDamping(0.0000);
    msection->SetDensity(1000);
	// msection->SetCentroid(0,0.02);
	// msection->SetShearCenter(0,0.1);
	// msection->SetSectionRotation(45*CH_C_RAD_TO_DEG);

	//
	// Add some IGA BEAMS:
	//

    //
    // Example A: create nodes and elements (low level approach):
    //

	double beam_L = 0.1;

	auto hnode1 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(beam_L*0,           0, 0)));
    auto hnode2 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(beam_L*0.5,	  0.00,	0)));
    auto hnode3 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(beam_L*1.0,      0.00,	0)));
	auto hnode4 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(beam_L*1.5,      0.00, 0)));
	auto hnode5 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(beam_L*2.0,      0.00, 0)));
/*	
    my_mesh->AddNode(hnode1);
    my_mesh->AddNode(hnode2);
    my_mesh->AddNode(hnode3);
	my_mesh->AddNode(hnode4);
	my_mesh->AddNode(hnode5);
	
    // spline cubica con 2 span, 5 points e 9 knots= {0 0 0 0 1/2 1 1 1 1}

    auto belement1 = std::make_shared<ChElementBeamIGA>();
	
	belement1->SetNodesCubic(hnode1, hnode2, hnode3, hnode4, 0, 0, 0, 0, 1./2., 1, 1, 1);
	belement1->SetSection(msection);

	my_mesh->AddElement(belement1);
	
    auto belement2 = std::make_shared<ChElementBeamIGA>();

	belement2->SetNodesCubic(hnode2, hnode3, hnode4, hnode5, 0, 0, 0, 1./2., 1, 1, 1, 1);
    belement2->SetSection(msection);

    my_mesh->AddElement(belement2);
*/	

    //
    // Example B: Automatic creation of the nodes and knots 
    // using the ChBuilderBeamIGA tool for creating a straight rod divided in Nel elements: 
    //
 /*   
    ChBuilderBeamIGA builder;
    builder.BuildBeam(      my_mesh,            // the mesh to put the elements in
                            msection,           // section of the beam
                            15,                 // number of sections (spans)
                            ChVector<>(0,  0,0),// start point 
                            ChVector<>(0.4,0,0),// end point 
                            VECT_Y,             // suggested Y direction of section
                            3);                 // order (3 = cubic, etc)
    builder.GetLastBeamNodes().front()->SetFixed(true);
    builder.GetLastBeamNodes().back()->SetForce(ChVector<>(0,-2,0));
    //builder.GetLastBeamNodes().back()->SetTorque(ChVector<>(0,0, 1.2));
*/
    //
    // Example C: Automatic creation of the nodes and knots using the 
    // ChBuilderBeamIGA tool for creating a generic curved rod that matches a Bspline:
    //

	
	ChBuilderBeamIGA builderR;

    std::vector< ChVector<> > my_points = { {0,0,0.2}, {0,0,0.3}, { 0,-0.01,0.4 } , {0,-0.04,0.5}, {0,-0.1,0.6} };
    
    geometry::ChLineBspline my_spline(  3,          // order (3 = cubic, etc)
                                        my_points); // control points, will become the IGA nodes

    builderR.BuildBeam(      my_mesh,            // the mesh to put the elements in
                            msection,           // section of the beam
                            my_spline,          // Bspline to match (also order will be matched)
                            VECT_Y);            // suggested Y direction of section
    
	builderR.GetLastBeamNodes().front()->SetFixed(true);

	auto mbodywing = std::make_shared<ChBodyEasyBox>(0.01,0.2,0.05,2000);
	mbodywing->SetCoord(builderR.GetLastBeamNodes().back()->GetCoord());
	my_system.Add(mbodywing);

	auto myjoint = std::make_shared<ChLinkMateFix>();
	myjoint->Initialize(builderR.GetLastBeamNodes().back(), mbodywing);
	my_system.Add(myjoint);
	

    //
    // Final touches..
    //

    // We do not want gravity effect on FEA elements in this demo
    my_mesh->SetAutomaticGravity(false);

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);

    // ==Asset== attach a visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChTriangleMeshShape
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colors as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a colored ChTriangleMeshShape).
    // Do not forget AddAsset() at the end!

    
    auto mvisualizebeamA = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizebeamA->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizebeamA);

    auto mvisualizebeamC = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
    mvisualizebeamC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizebeamC->SetSymbolsThickness(0.006);
    mvisualizebeamC->SetSymbolsScale(0.01);
    mvisualizebeamC->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizebeamC);



    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"IGA beams DEMO (SPACE for dynamics, F10 / F11 statics)", core::dimension2d<u32>(800, 600),
                         false, true); 

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(-0.1f, 0.2f, -0.2f));

    // ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
    // in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
    // If you need a finer control on which item really needs a visualization proxy in
    // Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

    application.AssetBindAll();

    // ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
    // that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

    application.AssetUpdateAll();

    // Mark completion of system construction
    my_system.SetupInitial();

    //
    // THE SOFT-REAL-TIME CYCLE
    //
    my_system.SetSolverType(ChSolver::Type::MINRES);
    my_system.SetSolverWarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
    my_system.SetMaxItersSolverSpeed(500);
    my_system.SetMaxItersSolverStab(500);
    my_system.SetTolForce(1e-14);

    auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
    msolver->SetVerbose(false);
    msolver->SetDiagonalPreconditioning(true);

    #ifdef USE_MKL
        auto mkl_solver = std::make_shared<ChSolverMKL<>>();
        my_system.SetSolver(mkl_solver);
    #endif

    application.SetTimestep(0.01);

    //****TEST**** do a linear static analysis:

    application.GetSystem()->DoStaticLinear();
/*
    GetLog() << "\n\n\ TEST LINEAR STATIC: \n  tip displacement y = " 
             << builder.GetLastBeamNodes().back()->GetPos().y() - builder.GetLastBeamNodes().back()->GetX0().GetPos().y()
             << "\n"
             << "  exact should be: y = " << 
             (4*-2*pow(0.4,3)/(msection->GetYoungModulus()*beam_wz*pow(beam_wy,3)))
             +(-2*0.4)/((5./6.)*msection->GetGshearModulus()*beam_wz*beam_wy)
             << "\n";


    ChStreamOutAsciiFile my_file("output.txt");
    my_file << builder.GetLastBeamNodes().back()->GetPos().x() << "  " << builder.GetLastBeamNodes().back()->GetPos().y() << "\n";
*/
    GetLog() << "Press SPACE bar to start/stop dynamic simulation \n\n";
    GetLog() << "Press F10 for nonlinear static solution \n\n";
    GetLog() << "Press F11 for linear static solution \n\n";

    //application.SetPaused(true);

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        application.DoStep();

        application.EndScene();
    }

    return 0;
}
