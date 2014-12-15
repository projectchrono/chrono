//
// PROJECT CHRONO - http://projectchrono.org
//
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
///////////////////////////////////////////////////
//
//   A tracked vehicle, M113, built and simulated using the trackedVehicle library.
//   Build the vehicle using a hierarchy of subsystems.
//   Simulate by GUI input to an irrlicht EventReceiver.
//    - similar to demo_tracks:
//     - model track shoes with simple or complex collision geometry
//     - using clones of collision shapes
//     - use  SetFamilyMaskNoCollisionWithFamily, SetFamily etc., to avoid collisions between different families of bodies.
//
//	 Author: Justin Madsen, (c) 2014
///////////////////////////////////////////////////
  
 
#include "physics/ChSystem.h"
#include "particlefactory/ChParticleEmitter.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"
#include "unit_IRRLICHT/ChIrrApp.h"
 
#include "subsys/trackVehicle/trackVehicle.h"

// Use the main namespace of Chrono, and other chrono namespaces

using namespace chrono;
using namespace chrono::geometry;

// Use the main namespaces of Irrlicht
using namespace irr;    
using namespace core;
using namespace scene; 
using namespace video;
using namespace io; 
using namespace gui; 

#include <irrlicht.h>





// Define a MyEventReceiver class which will be used to manage input
// from the GUI graphical user interface (the interface will
// be created with the basic -yet flexible- platform
// independent toolset of Irrlicht).

class MyEventReceiver : public IEventReceiver
{
public:

	MyEventReceiver(ChIrrApp* app,
					TrackVehicle* vehicle)
          : m_app(app), m_vehicle(vehicle)
			{


				// ..add a GUI slider to control throttle left via mouse
				scrollbar_throttleL = m_app->GetIGUIEnvironment()->addScrollBar(
								true, rect<s32>(510, 20, 650, 35), 0, 101);
				scrollbar_throttleL->setMax(100); 
				scrollbar_throttleL->setPos(50);
				text_throttleL = m_app->GetIGUIEnvironment()->addStaticText(
							L"Left throttle ", rect<s32>(650,20,750,35), false);

				// ..add a GUI slider to control gas throttle right via mouse
				scrollbar_throttleR = m_app->GetIGUIEnvironment()->addScrollBar(
								true, rect<s32>(510, 45, 650, 60), 0, 102);
				scrollbar_throttleR->setMax(100); 
				scrollbar_throttleR->setPos(50);
				text_throttleR = m_app->GetIGUIEnvironment()->addStaticText(
							L"Right throttle", rect<s32>(650,45,750,60), false);
			}

	bool OnEvent(const SEvent& event)
			{

				// check if user moved the sliders with mouse..
				if (event.EventType == EET_GUI_EVENT)
				{
					s32 id = event.GUIEvent.Caller->getID();
					IGUIEnvironment* env = m_app->GetIGUIEnvironment();

					switch(event.GUIEvent.EventType)
					{
					case EGET_SCROLL_BAR_CHANGED:
							if (id == 101) // id of 'throttleL' slider..
							{
								s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
								double newthrottle =  ((double)(pos)-50)/50.0 ;
								this->m_vehicle->throttleL=newthrottle;
                                if (ChSharedPtr<ChFunction_Const> mfun = mtank->link_revoluteLB->Get_spe_funct().DynamicCastTo<ChFunction_Const>())
									mfun->Set_yconst(newthrottle*6);
								return true;
							}
							if (id == 102) // id of 'throttleR' slider..
							{
								s32 pos = ((IGUIScrollBar*)event.GUIEvent.Caller)->getPos();
								double newthrottle =  ((double)(pos)-50)/50.0 ;
								this->m_vehicle->throttleR=newthrottle;
                                if (ChSharedPtr<ChFunction_Const> mfun = mtank->link_revoluteRB->Get_spe_funct().DynamicCastTo<ChFunction_Const>())
									mfun->Set_yconst(newthrottle*6);
								return true;
							}
					break;
					}
					
				} 

				return false;
			}

private:
	ChIrrApp* m_app;
	TrackVehicle*    m_vehicle;

	IGUIStaticText* text_throttleL;
	IGUIScrollBar*  scrollbar_throttleL;
	IGUIStaticText* text_throttleR;
	IGUIScrollBar*  scrollbar_throttleR;
};



//
// This is the program which is executed
//

int main(int argc, char* argv[])
{
  // no system to create, it's in the trackVehicle

	// ..the tank (this class - see above - is a 'set' of bodies and links, automatically added at creation)
	TrackVehicle vehicle("name");

	// Create the Irrlicht visualization applicaiton
	ChIrrApp application(&vehicle, L"Modeling a simplified   tank",core::dimension2d<u32>(800,600),false, true); 


	// Easy shortcuts to add logo, camera, lights and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice());
	ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0,0,-6), core::vector3df(-2,2,0));

	
	// ground plate
  ChSharedPtr<ChBody> ground(new ChBodyEasyBox(60.0, 1.0, 100.0, 1000.0, true, true);
										
	ground->SetFriction(1.0);
  my_system.Add(ground);  // add this body to the system

	// ..some obstacles on the ground:
	for (int i=0; i<50; i++)
	{
		ChBodySceneNode* my_obstacle = (ChBodySceneNode*)addChBodySceneNode_easyBox(
											&my_system, application.GetSceneManager(),
											3.0,
											ChVector<>(-6+6*ChRandom(),2+1*ChRandom(), 6*ChRandom()),
											Q_from_AngAxis(ChRandom()*CH_C_PI, VECT_Y), 
											ChVector<>(0.6*(1-0.4*ChRandom()),
											           0.08,
													   0.3*(1-0.4*ChRandom()) ) );
		my_obstacle->addShadowVolumeSceneNode();
	}



	//
	// USER INTERFACE
	//
	 

	// Create some graphical-user-interface (GUI) items to show on the screen.
	// This requires an event receiver object.
	MyEventReceiver receiver(&application, vehicle);
	  // note how to add the custom event receiver to the default interface:
	application.SetUserEventReceiver(&receiver);


	//
	// SETTINGS 
	// 	

	my_system.SetIterLCPmaxItersSpeed(100); // the higher, the easier to keep the constraints 'mounted'.
	my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR); 



	//
	// THE SOFT-REAL-TIME CYCLE, SHOWING THE SIMULATION
	//


	application.SetStepManage(true);
	application.SetTimestep(0.03);
	application.SetTryRealtime(true);

	while(application.GetDevice()->run())
	{ 
		// Irrlicht must prepare frame to draw
		application.GetVideoDriver()->beginScene(true, true, SColor(255,140,161,192));
	
		// .. draw solid 3D items (boxes, cylinders, shapes) belonging to Irrlicht scene, if any
		application.DrawAll();

		// .. draw also a grid (rotated so that it's horizontal)
		ChIrrTools::drawGrid(application.GetVideoDriver(), 2, 2, 30,30, 
			ChCoordsys<>(ChVector<>(0,0.01,0), Q_from_AngX(CH_C_PI_2) ),
			video::SColor(255, 60,60,60), true);

		// HERE CHRONO INTEGRATION IS PERFORMED: 
		
		application.DoStep();


		application.GetVideoDriver()->endScene(); 
	}


	if (mytank) delete mytank;

	return 0;
}


