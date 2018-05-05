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
#include "chrono/physics/ChLinkMotorLinearPosition.h"
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

int ID_current_example = 1;



//
// Example A: Low  level approach, creating single elements and nodes:
//

void MakeAndRunDemo0(ChIrrApp& myapp) {

	// Clear previous demo, if any:
	myapp.GetSystem()->Clear();
	myapp.GetSystem()->SetChTime(0);

	// Create a mesh, that is a container for groups
	// of elements and their referenced nodes.
	// Remember to add it to the system.
	auto my_mesh = std::make_shared<ChMesh>();
	my_mesh->SetAutomaticGravity(false);
	myapp.GetSystem()->Add(my_mesh);

	// Create a section, i.e. thickness and material properties
	// for beams. This will be shared among some beams.

	double beam_wy = 0.012;
	double beam_wz = 0.025;

	auto melasticity = std::make_shared<ChElasticityCosseratSimple>();
	melasticity->SetYoungModulus(0.02e10);
	melasticity->SetGshearModulus(0.02e10 * 0.3);
	melasticity->SetBeamRaleyghDamping(0.0000);
	auto msection = std::make_shared<ChBeamSectionCosserat>(melasticity);
	msection->SetDensity(1000);
	msection->SetAsRectangularSection(beam_wy, beam_wz);

	// Example A.  
	// Create an IGA beam using a low-level approach, i.e.
	// creating all elements and nodes one after the other:

	double beam_L = 0.1;

	auto hnode1 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(beam_L * 0, 0, 0)));
	auto hnode2 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(beam_L*0.5, 0.00, 0)));
	auto hnode3 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(beam_L*1.0, 0.00, 0)));
	auto hnode4 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(beam_L*1.5, 0.00, 0)));
	auto hnode5 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(beam_L*2.0, 0.00, 0)));
	
	my_mesh->AddNode(hnode1);
	my_mesh->AddNode(hnode2);
	my_mesh->AddNode(hnode3);
	my_mesh->AddNode(hnode4);
	my_mesh->AddNode(hnode5);

	// cubic spline with 2 spans, 5 control points and 9 knots= {0 0 0 0 1/2 1 1 1 1}

	auto belement1 = std::make_shared<ChElementBeamIGA>();

	belement1->SetNodesCubic(hnode1, hnode2, hnode3, hnode4, 0, 0, 0, 0, 1./2., 1, 1, 1);
	belement1->SetSection(msection);

	my_mesh->AddElement(belement1);

	auto belement2 = std::make_shared<ChElementBeamIGA>();

	belement2->SetNodesCubic(hnode2, hnode3, hnode4, hnode5, 0, 0, 0, 1./2., 1, 1, 1, 1);
	belement2->SetSection(msection);

	my_mesh->AddElement(belement2);


	// Attach a visualization of the FEM mesh.

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

	// This is needed if you want to see things in Irrlicht 3D view.
	myapp.AssetBindAll();
	myapp.AssetUpdateAll();

	// Mark completion of system construction
	myapp.GetSystem()->SetupInitial();

	while (ID_current_example == 1 && myapp.GetDevice()->run()) {
		myapp.BeginScene();
		myapp.DrawAll();
		myapp.DoStep();
		myapp.EndScene();
	}

}


//
// Example B: Automatic creation of the nodes and knots
// using the ChBuilderBeamIGA tool for creating a straight 
// rod automatically divided in Nel elements:
//

void MakeAndRunDemo1(ChIrrApp& myapp) {

	// Clear previous demo, if any:
	myapp.GetSystem()->Clear();
	myapp.GetSystem()->SetChTime(0);

	// Create a mesh, that is a container for groups
	// of elements and their referenced nodes.
	// Remember to add it to the system.
	auto my_mesh = std::make_shared<ChMesh>();
	my_mesh->SetAutomaticGravity(false);
	myapp.GetSystem()->Add(my_mesh);


	// Create a section, i.e. thickness and material properties
	// for beams. This will be shared among some beams.

	double beam_wy = 0.012;
	double beam_wz = 0.025;

	auto melasticity = std::make_shared<ChElasticityCosseratSimple>();
	melasticity->SetYoungModulus(0.02e10);
	melasticity->SetGshearModulus(0.02e10 * 0.3);
	melasticity->SetBeamRaleyghDamping(0.0000);
	auto msection = std::make_shared<ChBeamSectionCosserat>(melasticity);
	msection->SetDensity(1000);
	msection->SetAsRectangularSection(beam_wy, beam_wz);

	// Use the ChBuilderBeamIGA tool for creating a straight rod 
	// divided in Nel elements:

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

	
	// Attach a visualization of the FEM mesh.

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
	
	// This is needed if you want to see things in Irrlicht 3D view.
	myapp.AssetBindAll();
	myapp.AssetUpdateAll();

	// Mark completion of system construction
	myapp.GetSystem()->SetupInitial();

	// Do a linear static analysis.
	myapp.GetSystem()->DoStaticLinear();

	GetLog() << "\n\n TEST LINEAR STATIC: \n  for straght bar, tip displacement y = "
		<< builder.GetLastBeamNodes().back()->GetPos().y() - builder.GetLastBeamNodes().back()->GetX0().GetPos().y()
		<< "\n"
		<< "  exact should be: y = " <<
		(4 * -2 * pow(0.4, 3) / (melasticity->GetYoungModulus()*beam_wz*pow(beam_wy, 3)))
		+ (-2 * 0.4) / ((5. / 6.)*melasticity->GetGshearModulus()*beam_wz*beam_wy)
		<< "\n";

	while (ID_current_example == 1 && myapp.GetDevice()->run()) {
		myapp.BeginScene();
		myapp.DrawAll();
		myapp.DoStep();
		myapp.EndScene();
	}

}


//
// Example C: Automatic creation of the nodes and knots using the 
// ChBuilderBeamIGA tool for creating a generic curved rod that matches a Bspline.
//

void MakeAndRunDemo2(ChIrrApp& myapp) {

	// Clear previous demo, if any:
	myapp.GetSystem()->Clear();
	myapp.GetSystem()->SetChTime(0);

	// Create a mesh, that is a container for groups
	// of elements and their referenced nodes.
	// Remember to add it to the system.
	auto my_mesh = std::make_shared<ChMesh>();
	my_mesh->SetAutomaticGravity(false);
	myapp.GetSystem()->Add(my_mesh);


	// Create a section, i.e. thickness and material properties
	// for beams. This will be shared among some beams.

	double beam_wy = 0.012;
	double beam_wz = 0.025;

	auto melasticity = std::make_shared<ChElasticityCosseratSimple>();
	melasticity->SetYoungModulus(0.02e10);
	melasticity->SetGshearModulus(0.02e10 * 0.3);
	melasticity->SetBeamRaleyghDamping(0.0000);
	auto msection = std::make_shared<ChBeamSectionCosserat>(melasticity);
	msection->SetDensity(1000);
	msection->SetAsRectangularSection(beam_wy, beam_wz);


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
	myapp.GetSystem()->Add(mbodywing);

	auto myjoint = std::make_shared<ChLinkMateFix>();
	myjoint->Initialize(builderR.GetLastBeamNodes().back(), mbodywing);
	myapp.GetSystem()->Add(myjoint);


	// Attach a visualization of the FEM mesh.

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

	// This is needed if you want to see things in Irrlicht 3D view.
	myapp.AssetBindAll();
	myapp.AssetUpdateAll();

	// Mark completion of system construction
	myapp.GetSystem()->SetupInitial();

	while (ID_current_example == 2 && myapp.GetDevice()->run()) {
		myapp.BeginScene();
		myapp.DrawAll();
		myapp.DoStep();
		myapp.EndScene();
	}
}


//
// Example D: 
// Plasticity in IGA beams.
//

void MakeAndRunDemo3(ChIrrApp& myapp) {

	// Clear previous demo, if any:
	myapp.GetSystem()->Clear();
	myapp.GetSystem()->SetChTime(0);

	// Create a mesh, that is a container for groups
	// of elements and their referenced nodes.
	// Remember to add it to the system.
	auto my_mesh = std::make_shared<ChMesh>();
	my_mesh->SetAutomaticGravity(false);
	myapp.GetSystem()->Add(my_mesh);


	// Create a section, i.e. thickness and material properties
	// for beams. This will be shared among some beams.
	// Note that we will define some basic plasticity. One can 
	// set hardening curves, both isotropic hardening and/or kinematic hardening.

	double beam_wy = 0.012;
	double beam_wz = 0.025;

	auto melasticity = std::make_shared<ChElasticityCosseratSimple>();
	melasticity->SetYoungModulus(0.02e10);
	melasticity->SetGshearModulus(0.02e10 * 0.3);
	melasticity->SetBeamRaleyghDamping(0.0000);

	auto mplasticity = std::make_shared<ChPlasticityCosseratLumped>();
	// The isotropic hardening curve. The value at zero absyssa is the initial yeld.
	mplasticity->n_yeld_x = std::make_shared<ChFunction_Const>(3000);
	//mplasticity->n_yeld_x = std::make_shared<ChFunction_Ramp>(3000, 1e3);
	// The optional kinematic hardening curve:
	mplasticity->n_beta_x = std::make_shared<ChFunction_Ramp>(0, 1e3);

	// for bending (on y and z): some kinematic hardening
	mplasticity->n_yeld_My = std::make_shared<ChFunction_Const>(0.3);
	mplasticity->n_beta_My = std::make_shared<ChFunction_Ramp>(0, 0.001e2);
	mplasticity->n_yeld_Mz = std::make_shared<ChFunction_Const>(0.3);
	mplasticity->n_beta_Mz = std::make_shared<ChFunction_Ramp>(0, 0.001e2);


	auto msection = std::make_shared<ChBeamSectionCosserat>(melasticity, mplasticity);
	msection->SetDensity(1000);
	msection->SetAsRectangularSection(beam_wy, beam_wz);


	ChBuilderBeamIGA builder;
	builder.BuildBeam(my_mesh,            // the mesh to put the elements in
		msection,           // section of the beam
		5,                  // number of sections (spans)
		ChVector<>(0, 0, 0),// start point 
		ChVector<>(0.4, 0.0, 0),// end point 
		VECT_Y,             // suggested Y direction of section
		2);                 // order (3 = cubic, etc)
	builder.GetLastBeamNodes().front()->SetFixed(true);

	// Now create a linear motor that push-pulls the end of the beam
	// up to repeated plasticization.
	auto truss = std::make_shared<ChBody>();
	myapp.GetSystem()->Add(truss);
	truss->SetBodyFixed(true);

	auto motor = std::make_shared<ChLinkMotorLinearPosition>();
	myapp.GetSystem()->Add(motor);
	motor->Initialize(builder.GetLastBeamNodes().back(), truss, ChFrame<>(builder.GetLastBeamNodes().back()->GetPos(), chrono::Q_from_AngAxis(0*CH_C_PI_2, VECT_Z)));
	motor->SetGuideConstraint(ChLinkMotorLinear::GuideConstraint::SPHERICAL);
	auto rampup = std::make_shared<ChFunction_Ramp>(0, 0.1);
	auto rampdo = std::make_shared<ChFunction_Ramp>(0, -0.1);
	auto motfun = std::make_shared<ChFunction_Sequence>();
	motfun->InsertFunct(rampdo, 1, 0, true);
	motfun->InsertFunct(rampup, 1, 0, true);
	auto motrepeat = std::make_shared<ChFunction_Repeat>();
	motrepeat->Set_fa(motfun);
	motrepeat->Set_window_length(2);
	auto motfuntot = std::make_shared<ChFunction_Sequence>();
	motfuntot->InsertFunct(rampup, 0.5, 0, true);
	motfuntot->InsertFunct(motrepeat, 10, 0, true);
	motor->SetMotionFunction(motfuntot);


	// Attach a visualization of the FEM mesh.

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

	// This is needed if you want to see things in Irrlicht 3D view.
	myapp.AssetBindAll();
	myapp.AssetUpdateAll();

	// Mark completion of system construction
	myapp.GetSystem()->SetupInitial();

	ChStreamOutAsciiFile my_plasticfile("plasticity.txt");

	while (ID_current_example == 3 && myapp.GetDevice()->run()) {
		myapp.BeginScene();
		myapp.DrawAll();
		myapp.DoStep();

		// Save to file: plastic flow of the 1st element, and other data
		ChMatrixDynamic<> mK(builder.GetLastBeamElements()[0]->GetNdofs(), builder.GetLastBeamElements()[0]->GetNdofs());
		builder.GetLastBeamElements()[0]->ComputeKRMmatricesGlobal(mK, 1, 0, 0);
		auto plasticdat = builder.GetLastBeamElements()[0]->GetPlasticData()[0].get();
		auto plasticdata = dynamic_cast<ChInternalDataLumpedCosserat*>(plasticdat);
		
		my_plasticfile << myapp.GetSystem()->GetChTime() << " "
			<< builder.GetLastBeamElements()[0]->GetStrainE()[0].x() << " "
			<< builder.GetLastBeamElements()[0]->GetStressN()[0].x() << " "
			<< plasticdata->p_strain_acc << " "
			<< plasticdata->p_strain_e.x() << " "
			<< mK(0, 0) << " "
			<< motor->GetMotorForce() << " "
			<< motor->GetMotorPos() << "\n";
		/*
		my_plasticfile << myapp.GetSystem()->GetChTime() << " "
			<< builder.GetLastBeamElements()[0]->GetStrainK()[0].z() << " "
			<< builder.GetLastBeamElements()[0]->GetStressM()[0].z() << " "
			<< plasticdata->p_strain_acc << " "
			<< plasticdata->p_strain_k.z() << " "
			<< mK(5, 5) << " "
			<< motor->GetMotorForce() << " "
			<< motor->GetMotorPos() << "\n";
			*/
		myapp.EndScene();
	}
}




/// Following class will be used to manage events from the user interface

class MyEventReceiver : public IEventReceiver {
public:
	MyEventReceiver(ChIrrApp* myapp) {
		// store pointer to physical system & other stuff so we can tweak them by user keyboard
		app = myapp;
	}

	bool OnEvent(const SEvent& event) {
		// check if user presses keys
		if (event.EventType == irr::EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown) {
			switch (event.KeyInput.Key) {
			case irr::KEY_KEY_1:
				ID_current_example = 1;
				return true;
			case irr::KEY_KEY_2:
				ID_current_example = 2;
				return true;
			case irr::KEY_KEY_3:
				ID_current_example = 3;
				return true;
			default:
				break;
			}
		}

		return false;
	}

private:
	ChIrrApp* app;
};





int main(int argc, char* argv[]) {
	GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

	// Create a Chrono::Engine physical system
	ChSystemNSC my_system;



    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"IGA beams DEMO (SPACE for dynamics, F10 / F11 statics)", core::dimension2d<u32>(800, 600),
                         false, true); 

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(-0.1f, 0.2f, -0.2f));

	// This is for GUI tweaking of system parameters..
	MyEventReceiver receiver(&application);
	// note how to add a custom event receiver to the default interface:
	application.SetUserEventReceiver(&receiver);

	// Some help on the screen
	auto gad_textFPS = application.GetIGUIEnvironment()->addStaticText(L" Press 1: static analysis \n Press 2: curved beam connected to body \n Press 3: plasticity", irr::core::rect<irr::s32>(10, 60, 250, 110), false, true, 0);
	

	// Solver default settings for all the sub demos:
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

	// Run the sub-demos:

	while (true) {
		switch (ID_current_example) {
		case 1:
			MakeAndRunDemo1(application);
			break;
		case 2:
			MakeAndRunDemo2(application);
			break;
		case 3:
			MakeAndRunDemo3(application);
			break;
		default:
			break;
		}
		if (!application.GetDevice()->run())
			break;
	}

    return 0;
}
