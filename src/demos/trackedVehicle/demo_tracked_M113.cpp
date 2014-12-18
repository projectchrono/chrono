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
#include "subsys/driver/ChIrrGuiTrack.h"
 
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


