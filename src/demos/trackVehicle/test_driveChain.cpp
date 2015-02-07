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
//   test the drive chain system.
//
//	 Author: Justin Madsen, 2015
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
// Use the main namespaces of Irrlicht
using namespace irr;    
using namespace core;
 /*
 # define USE_IRRLICHT
#endif
 */
 
#include "subsys/trackVehicle/DriveChain.h"
#include "ModelDefs.h"
// Use the main namespace of Chrono
using namespace chrono;

// =============================================================================
// User Settings
// =============================================================================
// display the 1) system heirarchy, 2) a set of subsystem hardpoints, 3) constraint violations
//#define DEBUG_LOG 

// Initial vehicle position and heading. Defines the REF frame for the hull body
ChVector<> initLoc(0, 1.0, 0);
//ChQuaternion<> initRot = Q_from_AngAxis(CH_C_PI_4, VECT_Y);
ChQuaternion<> initRot(QUNIT);

// Simulation step size
double step_size = 0.001;

size_t num_idlers = 1;

// Time interval between two render frames
int FPS = 40;
double render_step_size = 1.0 / FPS;   // FPS = 50
// Time interval between two output frames
double output_step_size = 1.0 / 1;    // once a second

// #ifdef USE_IRRLICHT
  // Point on chassis tracked by the camera
ChVector<> trackPoint(0, 0.2, 0.3);
// if chase cam enabled:
double chaseDist = 1.5;
double chaseHeight = 0.0;
// set a static camera position
ChVector<> cameraPos(0.0, 1.2, 1.4);
bool use_fixed_camera = false;
bool do_shadows = false; // shadow map is experimental

  /*
#else
  double tend = 20.0;

  const std::string out_dir = "../test_driveChain";
  const std::string pov_dir = out_dir + "/POVRAY";
#endif
  */


int main(int argc, char* argv[])
{
  // NOTE: utils library built with this sets this statically
  // SetChronoDataPath(CHRONO_DATA_DIR);
  // --------------------------
  // Create the Chain system (drive gear, idler, chain)

  // The drive chain inherits ChSystem. Specify the 
  // collision type used by the gear here.
  DriveChain chainSystem("Justins driveChain system", 
    // VisualizationType::MESH,
    VisualizationType::COMPOUNDPRIMITIVES,
    CollisionType::CALLBACKFUNCTION,
    num_idlers);
  
  // set the chassis REF at the specified initial config.
  chainSystem.Initialize(ChCoordsys<>(initLoc, initRot));

/*
#ifdef USE_IRRLICHT
*/

  // --------------------------
  // Setup the Irrlicht GUI

  // Create the Irrlicht visualization applicaiton

  ChIrrApp application(chainSystem.GetSystem(),
                      L"test driveChain demo",
                      dimension2d<u32>(1200, 800),
                      false,
                      do_shadows);
  // assumes Y-up
  application.AddTypicalSky();
 
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
      irr::core::vector3df(50.f, -25.f, 30.f),
      irr::core::vector3df(-30.f, 80.f, -50.f),
      150, 125);
  }

  application.SetTimestep(step_size);

  // the GUI driver
  ChIrrGuiTrack driver(application, chainSystem, trackPoint, chaseDist, chaseHeight);
  // even though using a chase camera, set the initial camera position laterally
  if(use_fixed_camera)
    driver.SetCameraPos(cameraPos);

  // Set the time response for steering and throttle keyboard inputs.
  // NOTE: this is not exact, since we do not render quite at the specified FPS.
  double throttle_time = 1.0;  // time to go from 0 to +1
  double braking_time = 0.3;	// time to go from 0 to +1
  // driver.SetSteeringDelta(render_step_size / 1);
  driver.SetThrottleDelta(render_step_size / throttle_time);
  driver.SetBrakingDelta(render_step_size / braking_time);

  // Set up the assets for rendering
  application.AssetBindAll();
  application.AssetUpdateAll();
  if (do_shadows)
  {
    application.AddShadowAll();
  }
/*
#else
  Track_FuncDriver driver;
#endif
*/

  // ---------------------
  // GUI and render settings

  // GUI driver inputs
  std::vector<double> throttle_input;
  std::vector<double> braking_input;

  // Number of simulation steps between two 3D view render frames
  int render_steps = (int)std::ceil(render_step_size / step_size);

  // Number of simulation steps between two output frames
  int output_steps = (int)std::ceil(output_step_size / step_size);

  // ---------------------
  // Simulation loop
#ifdef DEBUG_LOG
  GetLog() << "\n\n============ System Configuration ============\n";
  chainSystem.GetSystem()->ShowHierarchy(GetLog() );
#endif

  // Initialize simulation frame counter and simulation time
  int step_number = 0;
  double time = 0;
/*
#ifdef USE_IRRLICHT
*/

  ChRealtimeStepTimer realtime_timer;
  while (application.GetDevice()->run())
	{ 
		// keep track of the time spent calculating each sim step
    ChTimer<double> step_time;
    step_time.start();
		
    // Render scene
    if (step_number % render_steps == 0) {
      application.GetVideoDriver()->beginScene(true, true,irr::video::SColor(255, 140, 161, 192));
      driver.DrawAll();
      application.GetVideoDriver()->endScene();
    }

    // Collect output data from modules (for inter-module communication)
    throttle_input = driver.GetThrottle();
    braking_input = driver.GetBraking();

    // Update
    time = chainSystem.GetSystem()->GetChTime();

    driver.Update(time);

    chainSystem.Update(time, throttle_input[0], braking_input[0]);

    // Advance simulation for one timestep for all modules
    // step_size = realtime_timer.SuggestSimulationStep(step_size);
    use_fixed_camera ? driver.Advance(step_size, cameraPos): driver.Advance(step_size); 

    // SETTLING FOLLOWED BY NORMAL OPERATION STEP SIZES HARDCODED
    // 1e-5 and 1e-4, respectively
    // application.SetPaused(true);
    if( !application.GetPaused() )
      chainSystem.Advance(step_size);

    step_number++;

  }

  application.GetDevice()->drop();

/*
#else

  int render_frame = 0;

  if(ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
    std::cout << "Error creating directory " << out_dir << std::endl;
    return 1;
  }
  if(ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
    std::cout << "Error creating directory " << pov_dir << std::endl;
    return 1;
  }

  chainSystem.ExportMeshPovray(out_dir);

  char filename[100];

  while (time < tend)
  {
    if (step_number % render_steps == 0) {
      // Output render data
      sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
      utils::WriteShapesPovray(chainSystem.GetSystem(), filename);
      std::cout << "Output frame:   " << render_frame << std::endl;
      std::cout << "Sim frame:      " << step_number << std::endl;
      std::cout << "Time:           " << time << std::endl;
      std::cout << " throttle: " << driver.GetThrottle() << " braking: " << driver.GetBraking() << std::endl;
      std::cout << std::endl;
      render_frame++;
    }

    // Collect output data from modules (for inter-module communication)
    throttle_input = driver.GetThrottle();
    braking_input = driver.GetBraking();

    // Update modules (process inputs from other modules)
    time = chainSystem.GetSystem()->GetChTime();

    driver.Update(time);

    chainSystem.Update(time, throttle_input, braking_input);

    // Advance simulation for one timestep for all modules
    driver.Advance(step_size);

    chainSystem.Advance(step_size);

    // Increment frame number
    step_number++;
  }

#endif

*/

	return 0;
}


