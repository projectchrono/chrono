///////////////////////////////////////////////////
//
//   Demo code about 
//
//     - constraints and 'engine' objects
//     - using IRRLICHT as a realtime 3D viewer
//       of a slider-crank mechanism simulated with
//       Chrono::Engine.
//     - using the real-time step.
//       This is just a possible method of integration
//       of Chrono::Engine + Irrlicht: many others
//       are possible.
//
//	 CHRONO   
//   ------
//   Multibody dinamics engine
//  
// ------------------------------------------------ 
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------ 
///////////////////////////////////////////////////
 
  
 
#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"
#include "irrlicht_interface/ChBodySceneNode.h"
#include "irrlicht_interface/ChBodySceneNodeTools.h" 
#include "irrlicht_interface/ChDisplayTools.h" 
#include "irrlicht_interface/ChIrrWizard.h"
#include "core/ChRealtimeStep.h"

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

 

int main(int argc, char* argv[])
{

	// In CHRONO engine, The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed.
	DLL_CreateGlobals();

	// Create the IRRLICHT context (device, etc.)
	IrrlichtDevice* device = createDevice(video::EDT_DIRECT3D9, core::dimension2d<u32>(640, 480), 24, false, false, true);
	if (device == 0)
	{
		GetLog() << "Cannot use DirectX - switch to OpenGL \n"; 
		device = createDevice(video::EDT_OPENGL, core::dimension2d<u32>(640, 480));
		if (!device) return 1;
	}

	device->setWindowCaption(L"SIMPLIEST example of integration of Chrono::Engine and Irrlicht");

	IVideoDriver* driver           = device->getVideoDriver();
	ISceneManager*	 msceneManager = device->getSceneManager();
	IGUIEnvironment* guienv        = device->getGUIEnvironment();

 
	// Easy shortcuts to add logo, camera, lights and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(device);
	ChIrrWizard::add_typical_Sky(device);
	ChIrrWizard::add_typical_Lights(device);
	ChIrrWizard::add_typical_Camera(device, core::vector3df(0,0,-6));



    // 
	// HERE YOU CREATE THE MECHANICAL SYSTEM OF CHRONO...
	// 

	// 1- Create a ChronoENGINE physical system: all bodies and constraints
	//    will be handled by this ChSystem object.

	ChSystem my_system;
 
	// 2- Create the rigid bodies of the slider-crank mechanical system
	//   (a crank, a rod, a truss), maybe setting position/mass/inertias of
	//   their center of mass (COG) etc.
	
	// ..the truss
	ChSharedBodyPtr  my_body_A(new ChBody);   
	my_system.AddBody(my_body_A);
	my_body_A->SetBodyFixed(true);			// truss does not move!

	// ..the crank
	ChSharedBodyPtr  my_body_B(new ChBody);	  
	my_system.AddBody(my_body_B);
	my_body_B->SetPos(ChVector<>(1,0,0));	// position of COG of crank

	// ..the rod
	ChSharedBodyPtr  my_body_C(new ChBody);	 
	my_system.AddBody(my_body_C);
	my_body_C->SetPos(ChVector<>(4,0,0));	// position of COG of rod
		

	// 3- Create constraints: the mechanical joints between the 
	//    rigid bodies.
    
	// .. a revolute joint between crank and rod
	ChSharedPtr<ChLinkLockRevolute>  my_link_BC(new ChLinkLockRevolute);
	my_link_BC->Initialize(my_body_B, my_body_C, ChCoordsys<>(ChVector<>(2,0,0)));
	my_system.AddLink(my_link_BC);

	// .. a slider joint between rod and truss
	ChSharedPtr<ChLinkLockPointLine> my_link_CA(new ChLinkLockPointLine);
	my_link_CA->Initialize(my_body_C, my_body_A, ChCoordsys<>(ChVector<>(6,0,0)));
	my_system.AddLink(my_link_CA);
 
	// .. an engine between crank and truss

	ChSharedPtr<ChLinkEngine> my_link_AB(new ChLinkEngine);
	my_link_AB->Initialize(my_body_A, my_body_B, ChCoordsys<>(ChVector<>(0,0,0)));
	my_link_AB->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
	my_link_AB->Get_spe_funct()->Set_yconst(CH_C_PI); // speed w=3.145 rad/sec
	my_system.AddLink(my_link_AB);


	//
	// THE SOFT-REAL-TIME CYCLE, SHOWING THE SIMULATION
	//
 

	// This will help choosing an integration step which matches the
	// real-time step of the simulation..
	ChRealtimeStepTimer m_realtime_timer;

	bool removed = false;

	while(device->run())
	{ 
		// Irrlicht must prepare frame to draw
		driver->beginScene(true, true, SColor(255,140,161,192));
	
		// Irrlicht now draws simple lines in 3D world representing a 
		// skeleton of the mechanism, in this instant:
		//
		// .. draw items belonging to Irrlicht scene, if any
		msceneManager->drawAll();
		// .. draw GUI items belonging to Irrlicht screen, if any
		guienv->drawAll();
		// .. draw a grid
		ChIrrTools::drawGrid(driver, 0.5, 0.5);
		// .. draw the rod (from joint BC to joint CA)
		ChIrrTools::drawSegment(driver, 
			my_link_BC->GetMarker1()->GetAbsCoord().pos, 
			my_link_CA->GetMarker1()->GetAbsCoord().pos,
			video::SColor(255,   0,255,0));
		// .. draw the crank (from joint AB to joint BC)
		ChIrrTools::drawSegment(driver, 
			my_link_AB->GetMarker1()->GetAbsCoord().pos, 
			my_link_BC->GetMarker1()->GetAbsCoord().pos,
			video::SColor(255, 255,0,0));
		// .. draw a small circle at crank origin
		ChIrrTools::drawCircle(driver, 0.1, ChCoordsys<>(ChVector<>(0,0,0), QUNIT));

		// HERE CHRONO INTEGRATION IS PERFORMED: THE 
		// TIME OF THE SIMULATION ADVANCES FOR A SINGLE
		// STEP:
	if (my_system.GetChTime() >10 && (!removed)) 
	{
			my_system.RemoveLink(my_link_AB); 
			removed = true;
	}

		my_system.DoStepDynamics( m_realtime_timer.SuggestSimulationStep(0.02) );

		// Irrlicht must finish drawing the frame
		driver->endScene(); 

	}

	// This safely delete every Irrlicht item..
	device->drop();
 


	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}


