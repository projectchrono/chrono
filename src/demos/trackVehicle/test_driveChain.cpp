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
//  Summary: Test the DriveChain system.
//  Components: Gear, Powertrain, tensioned and passive Idler(s),
//              
//  Throttle, gear, pin damping coef., etc. driven by the user with an Irrlicht GUI.
//  Alternatively, scripted driven simulations supported thru Track_FuncDriver.
//
//  Output I care about:
//    The reaction forces in the pins adjoining 5 or so track shoes.
//    I'm going to analyze the time history of pin tension forces, and also the 
//    cumulative contact normal and friction forces between the pins and gear surface.
//    Changing Gear CollisionType from PRIMITIVES to CALLBACKFUNCTION with and without pin/gear
//    callback function enabled. So the gear collision shapes will be 
//      a) two concentric cylinders, contact w/ flat shoe surface
//      b) two concentric cylinders, further laterally outwards. 10 boxes for the top gear tooth surface.
//      c) same as b), but add the collision callback function, which looks for contact between shoe pins and 
//         concave gear tooth base surface.
//
//    Output to text files for Python plots, by uncommenting #define WRITE_OUTPUT
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
//  render and total runtimes
#define WRITE_OUTPUT            // write output data to file
// #define CONSOLE_SYSTEM_INFO  // display the system heirarchy in the console
#define CONSOLE_DEBUG_INFO      // log constraint violations to console,
// #define CONSOLE_TIMING       // time each render and simulation step, log info to console

int what_to_save = DBG_FIRSTSHOE | DBG_GEAR | DBG_IDLER | DBG_PTRAIN | DBG_CONSTRAINTS;
int what_to_console = DBG_FIRSTSHOE | DBG_GEAR | DBG_IDLER | DBG_PTRAIN | DBG_CONSTRAINTS;

// Initial vehicle position and heading. Defines the REF frame for the hull body
ChVector<> initLoc(0, 1.0, 0);
//ChQuaternion<> initRot = Q_from_AngAxis(CH_C_PI_4, VECT_Y);
ChQuaternion<> initRot(QUNIT);

size_t num_idlers = 2;
// Simulation step size
double step_size = 0.002;

// #ifdef USE_IRRLICHT
int FPS = 40; // render frame rate
double render_step_size = 1.0 / FPS;  // Time increment between two rendered frames
double output_step_size = 1.0 / 1;    // Time interval between two output frames

ChVector<> trackPoint(0, 0, 0.2);   // Point on chassis tracked by the camera, chassis c-sys

bool use_fixed_camera = true;
ChVector<> cameraPos(0.5, 1.50, 2.6); // static camera position, global c-sys
// if chase cam enabled:
double chaseDist = 2.5;
double chaseHeight = 0.5;

bool do_shadows = false; // shadow map is experimental

  /*
#else
  double tend = 20.0;

  const std::string out_dir = "../test_driveChain";
  const std::string pov_dir = out_dir + "/POVRAY";
#endif
  */


// =============================================================================
// =============================================================================
int main(int argc, char* argv[])
{
  // NOTE: utils library built with this sets this statically
  // SetChronoDataPath(CHRONO_DATA_DIR);

  // --------------------------
  // Create the Chain system (drive gear, idler, chain)

  // The drive chain inherits ChSystem. Specify the 
  // collision type used by the gear here.
  DriveChain chainSystem("Justins driveChain system", 
    VisualizationType::MESH,
    // VisualizationType::COMPOUNDPRIMITIVES,
    CollisionType::CALLBACKFUNCTION,
    num_idlers);
  
  // set the chassis REF at the specified initial config.
  chainSystem.Initialize(ChCoordsys<>(initLoc, initRot));

  // if writing an output file, setup what debugInformation we want added each step data is saved.
#ifdef WRITE_OUTPUT
  chainSystem.Setup_log_to_file(what_to_save,
    "test_driveChain_all.csv");
#endif

/*
#ifdef USE_IRRLICHT
*/
  size_t window_x_len = 1200;
  size_t window_y_len = 800;
  // --------------------------
  // Setup the Irrlicht GUI

  // Create the Irrlicht visualization applicaiton
  ChIrrApp application(chainSystem.GetSystem(),
                      L"test driveChain demo",
                      dimension2d<u32>(window_x_len, window_y_len),
                      false,
                      do_shadows);
  // assumes Y-up
  application.AddTypicalSky();
 
  irr::scene::ILightSceneNode* mlight = 0;

  if (do_shadows)
  {
    mlight = application.AddLightWithShadow(
      irr::core::vector3df(-50.f, -20.f, 80.f),
      irr::core::vector3df(-30.f, 80.f, -50.f),
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
  ChIrrGuiTrack driver(application, chainSystem, trackPoint, chaseDist, chaseHeight,window_x_len-150);
  // even though using a chase camera, set the initial camera position laterally
  if(use_fixed_camera)
    driver.SetCameraPos(cameraPos);

  // Set the time response for steering and throttle keyboard inputs.
  // NOTE: this is not exact, since we do not render quite at the specified FPS.
  double throttle_time = 2.0;  // time to go from 0 to +1
  double braking_time = 0.5;	// time to go from 0 to +1
  // driver.SetSteeringDelta(render_step_size / 1);
  driver.SetThrottleDelta(render_step_size / throttle_time);
  driver.SetBrakingDelta(render_step_size / braking_time);
  driver.SetDampingDelta(render_step_size / 3.0);

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
#ifdef CONSOLE_SYSTEM_INFO
  GetLog() << "\n\n============ System Configuration ============\n";
  chainSystem.GetSystem()->ShowHierarchy(GetLog() );
#endif

  // Initialize simulation frame counter and simulation time
  int step_number = 0;
  double time = 0;

  // create some timers, for the render and total time
  ChTimer<double> step_timer;
  double total_step_time = 0;
  double time_since_last_output = 0;
//  #ifdef USE_IRRLICHT
  // using Irrlicht? time that too.
  ChTimer<double> render_timer;
  double total_render_time = 0;
//  #endif
/*
#ifdef USE_IRRLICHT
*/
// write data to file?
#ifdef WRITE_OUTPUT
      chainSystem.Log_to_file();  // needs to already be setup before sim loop calls it
#endif
  ChRealtimeStepTimer realtime_timer;
  while (application.GetDevice()->run())
	{ 
		// keep track of the time spent calculating each sim step
    step_timer.start();
		
    // Render scene
    if (step_number % render_steps == 0) {
      render_timer.start(); // start the time it takes to render the scene
      application.GetVideoDriver()->beginScene(true, true,irr::video::SColor(255, 140, 161, 192));
      driver.DrawAll();
      application.GetVideoDriver()->endScene();

      render_timer.stop();  // stop the scene timer
      total_render_time += render_timer();  // increment the time it took to render this step
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

    // Settlings phase has hardcoded solver settings, for the first few timesteps
    // application.SetPaused(true);
    if( !application.GetPaused() )
      chainSystem.Advance(step_size);

    // stop and increment the step timer
    step_timer.stop();
    total_step_time += step_timer();
    time_since_last_output += step_timer();

    if (step_number % output_steps == 0) 
    {
     // log desired output to console?
#ifdef CONSOLE_DEBUG_INFO
      chainSystem.Log_to_console(what_to_console);
#endif

      // write data to file?
#ifdef WRITE_OUTPUT
      chainSystem.Log_to_file();  // needs to already be setup before sim loop calls it
#endif

      // write timer info to console?
#ifdef CONSOLE_TIMING
      GetLog() << "\n --------- TIMING -------------\n" << "time: " << chainSystem.GetSystem()->GetChTime();
      GetLog() << "\n total render time: " << total_render_time << ",  % of total: " << 100.*total_render_time / total_step_time;
      GetLog() << "\n total compute time: " << total_step_time << ", Avg. time per step " << time_since_last_output * chainSystem.GetSystem()->GetStep() / output_step_size;
      time_since_last_output = 0;
#endif
    }

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


