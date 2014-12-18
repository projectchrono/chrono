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
// #include "particlefactory/ChParticleEmitter.h"


#include "core/ChFileutils.h"
#include "core/ChStream.h"
#include "core/ChRealtimeStep.h"
#include "physics/ChSystem.h"
#include "physics/ChLinkDistance.h"

#include "utils/ChUtilsInputOutput.h"
#include "utils/ChUtilsData.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"
/*
#if IRRLICHT_ENABLED
*/
#include "unit_IRRLICHT/ChIrrApp.h"
#include "subsys/driver/ChIrrGuiTrack.h"
 /*
 # define USE_IRRLICHT
#endif
 */
#include "subsys/trackVehicle/trackVehicle.h"

// Use the main namespace of Chrono
using namespace chrono;

// Use the main namespaces of Irrlicht
using namespace irr;    
using namespace core;


// // Initial vehicle position
ChVector<> initLoc(0, 1.0, 0);
// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);
//ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
//ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
//ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);
//ChQuaternion<> initRot(0, 0, 0, 1);

// Simulation step size
double step_size = 0.001;

// Time interval between two render frames
int FPS = 50;
double render_step_size = 1.0 / FPS;   // FPS = 50

// #ifdef USE_IRRLICHT
  // Point on chassis tracked by the camera
  ChVector<> trackPoint(0.0, 0.0, .75);
  /*
#else
  double tend = 20.0;

  const std::string out_dir = "../HMMWV9";
  const std::string pov_dir = out_dir + "/POVRAY";
#endif
  */




int main(int argc, char* argv[])
{

  // no system to create, it's in the TrackVehicle
	TrackVehicle vehicle("name");

  vehicle.Initialize(ChCoordsys<>(initLoc, initRot));

/*
#ifdef USE_IRRLICHT
*/
	// Create the Irrlicht visualization applicaiton
  ChIrrApp application(&vehicle,
                      L"HMMWV 9-body demo",
                      dimension2d<u32>(1000, 800),
                      false,
                      true);
	
  // make a skybox that has Z pointing up (default application.AddTypicalSky() makes Y up) 
  std::string mtexturedir = GetChronoDataFile("skybox/");
  std::string str_lf = mtexturedir + "sky_lf.jpg";
  std::string str_up = mtexturedir + "sky_up.jpg";
  std::string str_dn = mtexturedir + "sky_dn.jpg";
  irr::video::ITexture* map_skybox_side = 
      application.GetVideoDriver()->getTexture(str_lf.c_str());
  irr::scene::ISceneNode* mbox = application.GetSceneManager()->addSkyBoxSceneNode(
      application.GetVideoDriver()->getTexture(str_up.c_str()),
      application.GetVideoDriver()->getTexture(str_dn.c_str()),
      map_skybox_side,
      map_skybox_side,
      map_skybox_side,
      map_skybox_side);
  mbox->setRotation( irr::core::vector3df(0,0,0));
 
  bool do_shadows = true; // shadow map is experimental
  irr::scene::ILightSceneNode* mlight = 0;

  if (do_shadows)
  {
    mlight = application.AddLightWithShadow(
      irr::core::vector3df(10.f, 30.f, 60.f),
      irr::core::vector3df(0.f, 0.f, 0.f),
      150, 60, 80, 15, 512, irr::video::SColorf(1, 1, 1), false, false);
  }
  else
  {
    application.AddTypicalLights(
      irr::core::vector3df(30.f, -30.f, 100.f),
      irr::core::vector3df(30.f, 50.f, 100.f),
      250, 130);
  }

  application.SetTimestep(step_size);

  ChIrrGuiTrack driver();




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


