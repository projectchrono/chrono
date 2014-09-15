//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   Demo code about  
//
//     - using IRRLICHT as a realtime 3D viewer
//       of a four-bar mechanism simulated with 
//       Chrono::Engine.
//     - Using the IRRLICHT graphical user interface (GUI)
//       to handle user-input via mouse.
// 
//	 CHRONO 
//   ------
//   Multibody dinamics engine
//  
// ------------------------------------------------ 
//             www.deltaknowledge.com
// ------------------------------------------------ 
///////////////////////////////////////////////////

 
 
#include "physics/ChSystem.h"
#include "unit_IRRLICHT/ChBodySceneNode.h"
#include "unit_IRRLICHT/ChBodySceneNodeTools.h" 
#include "unit_IRRLICHT/ChIrrTools.h" 
#include "unit_IRRLICHT/ChIrrWizard.h" 
 
#include <irrlicht.h>

 

// Use the namespace of Chrono

using namespace chrono;

// Use the main namespaces of Irrlicht
using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;


// Some static data (this is a simple application, we can
// do this ;) just to allow easy GUI manipulation
IGUIStaticText* text_enginespeed = 0;

// The MyEventReceiver class will be used to manage input
// from the GUI graphical user interface (the interface will
// be created with the basic -yet flexible- platform
// independent toolset of Irrlicht).

class MyEventReceiver : public IEventReceiver
{
public:

	MyEventReceiver(ChSystem* asystem,  
					IrrlichtDevice *adevice,
					ChSharedPtr<ChLinkEngine> aengine)
	{
		// store pointer to physical system & other stuff so we can tweak them by user keyboard
		msystem = asystem;
		mdevice = adevice;
		mengine = aengine;
	}

	bool OnEvent(const SEvent& event)
	{

		// check if user moved the sliders with mouse..
		if (event.EventType == EET_GUI_EVENT)
		{
			s32 id = event.GUIEvent.Caller->getID();
			IGUIEnvironment* env = mdevice->getGUIEnvironment();

			switch(event.GUIEvent.EventType)
			{
			case EGET_SCROLL_BAR_CHANGED:
					if (id == 101) // id of 'engine speed' slider..
					{
						s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
						double newspeed = 10*(double)pos/100.0;
						// set the speed into engine object
                        if (ChSharedPtr<ChFunction_Const> mfun = mengine->Get_spe_funct().DynamicCastTo<ChFunction_Const>())
							mfun->Set_yconst(newspeed);
						// show speed as formatted text in interface screen
						char message[50]; sprintf(message,"Engine speed: %g [rad/s]",newspeed);
						text_enginespeed->setText(core::stringw(message).c_str());
					}
			break;
			}
			
		} 

		return false;
	}

private:
	ChSystem*       msystem;
	IrrlichtDevice* mdevice;
	ChSharedPtr<ChLinkEngine> mengine;
};



 
int main(int argc, char* argv[])
{
	// Create the IRRLICHT context (device, etc.)
	IrrlichtDevice* device = createDevice(video::EDT_DIRECT3D9, core::dimension2d<u32>(800, 600));
	if (device == 0)
	{
		GetLog() << "Cannot use DirectX - switch to OpenGL \n"; 
		device = createDevice(video::EDT_OPENGL, core::dimension2d<u32>(800, 600));
		if (!device) return 1;
	}

	device->setWindowCaption(L"Example of integration of Chrono::Engine and Irrlicht, with GUI");

	// Easy shortcuts to add logo, camera, lights and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(device);
	ChIrrWizard::add_typical_Sky(device);
	ChIrrWizard::add_typical_Lights(device);
	ChIrrWizard::add_typical_Camera(device);
 
 
    // 
	// HERE YOU CREATE THE MECHANICAL SYSTEM OF CHRONO...
	//  

	// 1- Create a ChronoENGINE physical system: all bodies and constraints
	//    will be handled by this ChSystem object.
	ChSystem my_system;


	// 2- Create the rigid bodies of the four-bar mechanical system
	//   (a flywheel, a rod, a rocker, a truss), maybe setting 
	//   position/mass/inertias of their center of mass (COG) etc.
	
	// ..the truss
	ChSharedBodyPtr  my_body_A(new ChBody);   
	my_system.AddBody(my_body_A);
	my_body_A->SetBodyFixed(true);			// truss does not move!

	// ..the flywheel
	ChSharedBodyPtr  my_body_B(new ChBody);	  
	my_system.AddBody(my_body_B);
	my_body_B->SetPos(ChVector<>(0,0,0));	// position of COG of flywheel

	// ..the rod
	ChSharedBodyPtr  my_body_C(new ChBody);	 
	my_system.AddBody(my_body_C);
	my_body_C->SetPos(ChVector<>(4,0,0));	// position of COG of rod

	// ..the rocker
	ChSharedBodyPtr  my_body_D(new ChBody);	 
	my_system.AddBody(my_body_D);
	my_body_D->SetPos(ChVector<>(8,-4,0));	// position of COG of rod
 

	// 3- Create constraints: the mechanical joints between the 
	//    rigid bodies. Doesn't matter if some constraints are redundant.
    
	// .. an engine between flywheel and truss	
	ChSharedPtr<ChLinkEngine> my_link_AB(new ChLinkEngine);
	my_link_AB->Initialize(my_body_A, my_body_B, ChCoordsys<>(ChVector<>(0,0,0)));
	my_link_AB->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
    if (ChSharedPtr<ChFunction_Const> mfun = my_link_AB->Get_spe_funct().DynamicCastTo<ChFunction_Const>())
		mfun->Set_yconst(CH_C_PI); // speed w=3.145 rad/sec
	my_system.AddLink(my_link_AB);

	// .. a revolute joint between flywheel and rod
	ChSharedPtr<ChLinkLockRevolute>  my_link_BC(new ChLinkLockRevolute);
	my_link_BC->Initialize(my_body_B, my_body_C, ChCoordsys<>(ChVector<>(2,0,0)));
	my_system.AddLink(my_link_BC);

	// .. a revolute joint between rod and rocker
	ChSharedPtr<ChLinkLockRevolute> my_link_CD(new ChLinkLockRevolute);
	my_link_CD->Initialize(my_body_C, my_body_D, ChCoordsys<>(ChVector<>(8,0,0)));
	my_system.AddLink(my_link_CD);

	// .. a revolute joint between rocker and truss
	ChSharedPtr<ChLinkLockRevolute> my_link_DA(new ChLinkLockRevolute);
	my_link_DA->Initialize(my_body_D, my_body_A, ChCoordsys<>(ChVector<>(8,-8,0)));
	my_system.AddLink(my_link_DA);





	//
	// Prepare some graphical-user-interface (GUI) items to show
	// on the screen
	// 
 

	// ..add a GUI text and GUI slider to control motor of mechanism via mouse
	text_enginespeed         = device->getGUIEnvironment()->addStaticText(
					L"Engine speed:", rect<s32>(10,85,150,100), false);
	IGUIScrollBar* scrollbar = device->getGUIEnvironment()->addScrollBar(
					true, rect<s32>(10, 105, 150, 120), 0, 101);
	scrollbar->setMax(100);

	// ..Finally create the event receiver, for handling all the GUI (user will use
	//   buttons/sliders to modify parameters)
	MyEventReceiver receiver(&my_system, device, my_link_AB);
	device->setEventReceiver(&receiver);



	//
	// Configure the solver with non-default settings
	//

	// Note that default iterative solvers cannot guarantee 100% precision in 
	// satisfying the constraints, expecially in some cases (ex. LCP_ITERATIVE_SOR) 
	// In this case, we rather choose to use the _direct_ solver of simplex
	// type (LCP_SIMPLEX) that is very precise. NOTE!! The LCP_SIMPLEX cannot
	// be used for systems with unilateral constraints or collisions!!! So it
	// good for simple mechanisms like this one.
	my_system.SetLcpSolverType(ChSystem::LCP_SIMPLEX);


	// By default, the solver uses the INT_ANITESCU stepper, that is very
	// fast, but may allow some geometric error in constraints (because it is 
	// based on constraint stabilization). Alternatively, the INT_TASORA
	// stepper is less fast, but it is based on constraint projection, so
	// gaps in constraints are less noticeable (hence avoids the 'spongy' 
	// behaviour of the default INT_ANITESCU solver, which operates only 
	// on speed-impulse level and keeps constraints'closed' by a continuous 
	// stabilization)
	my_system.SetIntegrationType(ChSystem::INT_TASORA);
	 


	//
	// THE SOFT-REAL-TIME CYCLE, SHOWING THE SIMULATION
	//


	// use this array of points to store trajectory of a rod-point
	std::vector< chrono::ChVector<> > mtrajectory; 

	while(device->run())
	{ 
		// Irrlicht must prepare frame to draw
		device->getVideoDriver()->beginScene(true, true, SColor(255,140,161,192));
	
  
		// Irrlicht now draws simple lines in 3D world representing a 
		// skeleton of the mechanism, in this instant:
		//
		// .. draw items belonging to Irrlicht scene, if any
		device->getSceneManager()->drawAll();
		// .. draw GUI items belonging to Irrlicht screen, if any
		device->getGUIEnvironment()->drawAll();

		// .. draw a grid
		ChIrrTools::drawGrid(device->getVideoDriver(), 0.5, 0.5);
		// .. draw a circle representing flywheel
		ChIrrTools::drawCircle(device->getVideoDriver(), 2.1, ChCoordsys<>(ChVector<>(0,0,0), QUNIT));
		// .. draw a small circle representing joint BC
		ChIrrTools::drawCircle(device->getVideoDriver(), 0.06, ChCoordsys<>(my_link_BC->GetMarker1()->GetAbsCoord().pos, QUNIT));
		// .. draw a small circle representing joint CD
		ChIrrTools::drawCircle(device->getVideoDriver(), 0.06, ChCoordsys<>(my_link_CD->GetMarker1()->GetAbsCoord().pos, QUNIT));
		// .. draw a small circle representing joint DA
		ChIrrTools::drawCircle(device->getVideoDriver(), 0.06, ChCoordsys<>(my_link_DA->GetMarker1()->GetAbsCoord().pos, QUNIT));
		// .. draw the rod (from joint BC to joint CD)
		ChIrrTools::drawSegment(device->getVideoDriver(), 
			my_link_BC->GetMarker1()->GetAbsCoord().pos, 
			my_link_CD->GetMarker1()->GetAbsCoord().pos,
			video::SColor(255,   0,255,0));
		// .. draw the rocker (from joint CD to joint DA)
		ChIrrTools::drawSegment(device->getVideoDriver(), 
			my_link_CD->GetMarker1()->GetAbsCoord().pos, 
			my_link_DA->GetMarker1()->GetAbsCoord().pos,
			video::SColor(255, 255,0,0));
		// .. draw the trajectory of the rod-point
		ChIrrTools::drawPolyline(device->getVideoDriver(), 
			mtrajectory, video::SColor(255, 0,150,0));


		// HERE CHRONO INTEGRATION IS PERFORMED: THE 
		// TIME OF THE SIMULATION ADVANCES FOR A SINGLE
		// STEP:
		my_system.DoStepDynamics(0.01);

		// We need to add another point to the array of 3d
		// points describing the trajectory to be drawn..
		mtrajectory.push_back(my_body_C->Point_Body2World(ChVector<>(1,1,0)));
		// keep only last 150 points..
		if (mtrajectory.size()>150) mtrajectory.erase(mtrajectory.begin());

		// Irrlicht must finish drawing the frame
		device->getVideoDriver()->endScene(); 

	}

	// This safely delete every Irrlicht item..
	device->getVideoDriver()->drop();


	return 0;
}


