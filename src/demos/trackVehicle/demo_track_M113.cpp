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
//   Y-up, X-forward, Z-lateral global c-sys
//    - similar to demo_tracks:
//     - model track shoes with simple or complex collision geometry
//     - using clones of collision shapes
//     - use  SetFamilyMaskNoCollisionWithFamily, SetFamily etc., to avoid collisions between different families of bodies.
//
//	 Author: Justin Madsen, 2014
///////////////////////////////////////////////////
  
 
#include "physics/ChSystem.h"
// #include "particlefactory/ChParticleEmitter.h"


#include "core/ChFileutils.h"
#include "core/ChStream.h"
#include "core/ChRealtimeStep.h"
#include "physics/ChSystem.h"
#include "physics/ChLinkDistance.h"
#include "physics/ChBodyEasy.h"

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
#include "ModelDefs.h"
// Use the main namespace of Chrono
using namespace chrono;

// Use the main namespaces of Irrlicht
using namespace irr;    
using namespace core;

// =============================================================================
// User Settings
// =============================================================================
// display the 1) system heirarchy, 2) a set of subsystem hardpoints, 3) constraint violations
// #define DEBUG_LOG 

// // Initial vehicle position and heading. Defines the REF frame for the hull body
ChVector<> initLoc(0, 1.0, 0);
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
// Time interval between two output frames
double output_step_size = 1.0 / 1;    // once a second

// #ifdef USE_IRRLICHT
  // Point on chassis tracked by the camera
  ChVector<> trackPoint(0.0, 1.0, 0);
  /*
#else
  double tend = 20.0;

  const std::string out_dir = "../HMMWV9";
  const std::string pov_dir = out_dir + "/POVRAY";
#endif
  */




int main(int argc, char* argv[])
{
  // --------------------------
  // Create the tracked vehicle and the ground/environment

  // The vehicle inherits ChSystem. Input chassis visual and collision type
	TrackVehicle vehicle("name", 
    VisualizationType::MESH,
    CollisionType::PRIMITIVES);
  
  // set the chassis REF at the specified initial config.
  vehicle.Initialize(ChCoordsys<>(initLoc, initRot));

  // ground is a large flat plate, with superimposed obstacles. Top surface at y = 0
  ChSharedPtr<ChBody> ground(new ChBodyEasyBox(60.0, 1.0, 100.0, 1000.0, true, true));
  ground->SetPos(ChVector<>(0,-0.5,0));
	ground->SetIdentifier(-1);
  ground->SetName("ground");
  ground->SetFriction(1.0);
  // color asset for the ground
  ChSharedPtr<ChColorAsset> groundColor(new ChColorAsset);
  groundColor->SetColor(ChColor(0.4f, 0.4f, 0.6f));
  ground->AddAsset(groundColor);
  vehicle.Add(ground);  // add this body to the system, which is the vehicle

  // --------------------------
  // Setup the Irrlicht GUI

/*
#ifdef USE_IRRLICHT
*/
	// Create the Irrlicht visualization applicaiton
  ChIrrApp application(&vehicle,
                      L"M113 tracked vehicle demo",
                      dimension2d<u32>(1000, 800),
                      false,
                      true);
  // assumes Y-up
  application.AddTypicalSky();
  /*
  // a skybox that has Z pointing up (default application.AddTypicalSky() makes Y up) 
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
  mbox->setRotation( irr::core::vector3df(90,0,0));  // rotate skybox for z-up 
  */
 
  bool do_shadows = true; // shadow map is experimental
  irr::scene::ILightSceneNode* mlight = 0;

  if (do_shadows)
  {
    mlight = application.AddLightWithShadow(
      irr::core::vector3df(10.f, 60.f, 30.f),
      irr::core::vector3df(0.f, 0.f, 0.f),
      150, 60, 80, 15, 512, irr::video::SColorf(1, 1, 1), false, false);
  }
  else
  {
    application.AddTypicalLights(
      irr::core::vector3df(30.f, 100.f, 30.f),
      irr::core::vector3df(30.f, 100.f, 50.f),
      250, 130);
  }

  application.SetTimestep(step_size);

  // the GUI driver
  ChIrrGuiTrack driver(application, vehicle, trackPoint,3.0, 2.0);

  // Set the time response for steering and throttle keyboard inputs.
  // NOTE: this is not exact, since we do not render quite at the specified FPS.
  double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
  double throttle_time = 1.0;  // time to go from 0 to +1
  double braking_time = 0.3;  // time to go from 0 to +1
  driver.SetSteeringDelta(render_step_size / steering_time);
  driver.SetThrottleDelta(render_step_size / throttle_time);
  driver.SetBrakingDelta(render_step_size / braking_time);

  // Set up the assets for rendering
  application.AssetBindAll();
  application.AssetUpdateAll();
  if (do_shadows)
  {
    application.AddShadowAll();
  }


  // ---------------------
  // GUI and render settings

  // GUI driver inputs
  std::vector<double> throttle_input;
  std::vector<double> braking_input;
  double steering_input;

  // Number of simulation steps between two 3D view render frames
  int render_steps = (int)std::ceil(render_step_size / step_size);

  // Number of simulation steps between two output frames
  int output_steps = (int)std::ceil(output_step_size / step_size);

  // ---------------------
  // Simulation loop
#ifdef DEBUG_LOG
  GetLog() << "\n\n============ System Configuration ============\n";
  vehicle.ShowHierarchy(GetLog() );
#endif


  // Initialize simulation frame counter and simulation time
  int step_number = 0;
  double time = 0;
  ChRealtimeStepTimer realtime_timer;
  while (application.GetDevice()->run())
	{ 
		// keep track of the time spent calculating each sim step
    ChTimer<double> step_time;
    step_time.start();
		
    // Render scene
    if (step_number % render_steps == 0) {
      application.GetVideoDriver()->beginScene(true, true, irr::video::SColor(255, 140, 161, 192));
      driver.DrawAll();
      application.GetVideoDriver()->endScene();
    }

    // Collect output data from modules (for inter-module communication)
    throttle_input = driver.GetThrottle();
    steering_input = driver.GetSteering();
    braking_input = driver.GetBraking();

    // Update
    time = vehicle.GetChTime();

    driver.Update(time);

    vehicle.Update(time, throttle_input, braking_input);
    // Advance simulation for one timestep for all modules
    double step = realtime_timer.SuggestSimulationStep(step_size);

    driver.Advance(step);

    vehicle.Advance(step);

	}

	return 0;
}


