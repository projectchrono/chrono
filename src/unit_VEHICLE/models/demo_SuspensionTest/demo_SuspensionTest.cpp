// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Justin Madsen
// =============================================================================
//
// Suspension testing mechanism, using force or motion inputs to the locked wheels
//  to simulate the effect of a post-testing mechanism
//
// The Irrlicht interface used to observe the suspension test, and also to provide
// any steering inputs.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <vector>

#include "core/ChFileutils.h"
#include "core/ChStream.h"
#include "core/ChRealtimeStep.h"
#include "physics/ChSystem.h"
#include "physics/ChLinkDistance.h"

#include "utils/ChUtilsInputOutput.h"
#include "utils/ChUtilsData.h"

// subsystems, all read in fron JSON files
#include "models/ModelDefs.h"
#include "subsys/suspensionTest/SuspensionTest.h"
#include "subsys/tire/RigidTire.h"
#include "subsys/terrain/FlatTerrain.h"
#include "subsys/driver/ChDataDriver.h"

// Irrlicht includes
#if IRRLICHT_ENABLED
# include "unit_IRRLICHT/ChIrrApp.h"
# include "subsys/driver/ChIrrGuiST.h"
# define USE_IRRLICHT
#endif

// DEBUGGING:  Uncomment the following line to print shock data
//#define DEBUG_LOG
#define DEBUG_ACTUATOR    // debug actuator in SuspensionTest
using namespace chrono;


// =============================================================================
// USER SETTINGS
// =============================================================================
double steer_limit = 0.6;
double post_limit = 0.2;

// Simulation step size
double step_size = 0.001;
double render_step_size = 1.0 / 50;   // Time interval between two render frames
double output_step_size = 1.0 / 1;    // Time interval between two output frames

#ifdef USE_IRRLICHT
  // Point on chassis tracked by the camera
  ChVector<> trackPoint(1.65, 0.0, 0);
  double camera_chase = 0;
  double camera_height = 2.0;
#else
  double tend = 20.0;
  const std::string out_dir = "../HMMWV";
  const std::string pov_dir = out_dir + "/POVRAY";
#endif

// =============================================================================
// JSON file for suspension
  std::string suspensionTest_file = utils::GetModelDataFile("hmmwv/suspensionTest/HMMWV_ST_front.json");

// JSON files for tire models (rigid) and powertrain (simple)
std::string rigidtire_file = utils::GetModelDataFile("hmmwv/tire/HMMWV_RigidTire.json");

// Driver input file (if not using Irrlicht)
std::string driver_file = utils::GetModelDataFile("generic/driver/Sample_Maneuver.txt");

// radius of wheel + vertical distance between spindle and chassis center marker
ChVector<> initLoc(0, 0, 0.496); 
ChQuaternion<> initRot(1, 0, 0, 0);



// =============================================================================
int main(int argc, char* argv[])
{
  SetChronoDataPath(CHRONO_DATA_DIR);

  // Create the testing mechanism, initilize ity
  SuspensionTest tester(suspensionTest_file);
  tester.Initialize(ChCoordsys<>(initLoc, initRot));
  // tester.Save_DebugLog(DBG_SPRINGS | DBG_SHOCKS | DBG_CONSTRAINTS | DBG_SUSPENSIONTEST,"log_test_SuspensionTester.csv");
  tester.Save_DebugLog(DBG_SUSPENSIONTEST,"log_test_SuspensionTester.csv");

  // Create and initialize two rigid wheels
  ChSharedPtr<ChTire> tire_front_right;
  ChSharedPtr<ChTire> tire_front_left;

  // flat rigid terrain, height = 0
  FlatTerrain flat_terrain(0);

  // use rigid wheels to actuate suspension
  ChSharedPtr<RigidTire> tire_FL(new RigidTire(rigidtire_file, flat_terrain));
  ChSharedPtr<RigidTire> tire_FR(new RigidTire(rigidtire_file, flat_terrain));
   
  tire_FL->Initialize(tester.GetWheelBody(FRONT_LEFT));
  tire_FR->Initialize(tester.GetWheelBody(FRONT_RIGHT));
   
  tire_front_left = tire_FL;
  tire_front_right = tire_FR;

#ifdef USE_IRRLICHT
  irr::ChIrrApp application(&tester,
                            L"HMMWV Suspension test",
                            irr::core::dimension2d<irr::u32>(1000, 800),
                            false,
                            true);

  // make a skybox that has Z pointing up (default .AddTypicalSky()  Y up) 
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
  mbox->setRotation( irr::core::vector3df(90,0,0));
 
  bool do_shadows = true; // shadow map is experimental
  irr::scene::ILightSceneNode* mlight = 0;

  if (do_shadows)
  {
    mlight = application.AddLightWithShadow(
      irr::core::vector3df(-5.f, 1.f, 6.f),
      irr::core::vector3df(5.f, -1.f, -2.f),
      250, 60, 80, 15, 512, irr::video::SColorf(1, 1, 1), false, false);
  }
  else
  {
    application.AddTypicalLights(
      irr::core::vector3df(30.f, -30.f, 100.f),
      irr::core::vector3df(30.f, 50.f, 100.f),
      250, 130);
  }

  application.SetTimestep(step_size);

  ChIrrGuiST driver(application, tester, trackPoint, camera_chase, camera_height, steer_limit, post_limit);

  // Set the time response for steering keyboard inputs, when they are used
  // NOTE: this is not exact, since not rendered quite at the specified FPS.
  double steering_time = 2.0;  // time to go from 0 to max
  double post_time = 3.0; // time to go from 0 to max applied post motion
  driver.SetSteeringDelta(render_step_size / steering_time * steer_limit);
  driver.SetPostDelta(render_step_size / post_time * post_limit);

  // Set up the assets for rendering
  application.AssetBindAll();
  application.AssetUpdateAll();
  if (do_shadows)
  {
    application.AddShadowAll();
  }
#else
  ChDataDriver driver;
#endif

  // ---------------
  // Simulation loop
#ifdef DEBUG_LOG
  GetLog() << "\n\n============ System Configuration ============\n";
  vehicle.LogHardpointLocations();
#endif

  // Inter-module communication data
  ChTireForces tire_forces(2);
  ChWheelState wheel_states[2];
  double       steering_input;
  double       post_z_L, post_z_R;

  // Number of simulation steps between two 3D view render frames
  int render_steps = (int)std::ceil(render_step_size / step_size);

  // Number of simulation steps between two output frames
  int output_steps = (int)std::ceil(output_step_size / step_size);

  // Initialize simulation frame counter and simulation time
  int step_number = 0;
  double time = 0;

#ifdef USE_IRRLICHT

  ChRealtimeStepTimer realtime_timer;

  while (application.GetDevice()->run())
  {
    // update the position of the shadow mapping so that it follows the car
    if (do_shadows)
    {
      ChVector<> lightaim = tester.GetChassisPos();
      ChVector<> lightpos = tester.GetChassisPos() + ChVector<>(10, 30, 60);
      irr::core::vector3df mlightpos((irr::f32)lightpos.x, (irr::f32)lightpos.y, (irr::f32)lightpos.z);
      irr::core::vector3df mlightaim((irr::f32)lightaim.x, (irr::f32)lightaim.y, (irr::f32)lightaim.z);
      application.GetEffects()->getShadowLight(0).setPosition(mlightpos);
      application.GetEffects()->getShadowLight(0).setTarget(mlightaim);
      mlight->setPosition(mlightpos);
    }

    // Render scene
    if (step_number % render_steps == 0) {
      application.GetVideoDriver()->beginScene(true, true, irr::video::SColor(255, 140, 161, 192));
      driver.DrawAll();
      application.GetVideoDriver()->endScene();
    }

#ifdef DEBUG_LOG
    if (step_number % output_steps == 0) {
      GetLog() << "\n\n============ System Information ============\n";
      GetLog() << "Time = " << time << "\n\n";
      tester.DebugLog(DBG_SPRINGS | DBG_SHOCKS | DBG_CONSTRAINTS);
    }
#endif

#ifdef DEBUG_ACTUATOR
    if (step_number % output_steps == 0) {
      GetLog() << "\n\n============ System Information ============\n";
      GetLog() << "Time = " << time << "\n\n";
      tester.DebugLog(DBG_SPRINGS | DBG_SUSPENSIONTEST);
    }
    if (step_number % render_steps == 0) {
      // write output data
      tester.SaveLog();
    }
#endif

    // Collect output data from modules, here it's the steering and post displacements
    steering_input = driver.GetSteering();
    post_z_L = driver.Get_post_z_L();
    post_z_R = driver.Get_post_z_R();
  
    tire_forces[FRONT_LEFT.id()] = tire_front_left->GetTireForce();
    tire_forces[FRONT_RIGHT.id()] = tire_front_right->GetTireForce();
    wheel_states[FRONT_LEFT.id()] = tester.GetWheelState(FRONT_LEFT);
    wheel_states[FRONT_RIGHT.id()] = tester.GetWheelState(FRONT_RIGHT);

    // Update modules (process inputs from other modules)
    time = tester.GetChTime();

    driver.Update(time);

    flat_terrain.Update(time);

    tire_front_left->Update(time, wheel_states[FRONT_LEFT.id()]);
    tire_front_right->Update(time, wheel_states[FRONT_RIGHT.id()]);

    tester.Update(time, steering_input, post_z_L, post_z_R, tire_forces);

    // Advance simulation for one timestep for all modules
    double step = realtime_timer.SuggestSimulationStep(step_size);

    driver.Advance(step);

    flat_terrain.Advance(step);

    tire_front_left->Advance(step);
    tire_front_right->Advance(step);
   
    tester.Advance(step);

    // Increment frame number
    step_number++;
  }

  application.GetDevice()->drop();

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

  vehicle.ExportMeshPovray(out_dir);

  char filename[100];

  while (time < tend)
  {
    if (step_number % render_steps == 0) {
      // Output render data
      sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
      utils::WriteShapesPovray(&vehicle, filename);
      std::cout << "Output frame:   " << render_frame << std::endl;
      std::cout << "Sim frame:      " << step_number << std::endl;
      std::cout << "Time:           " << time << std::endl;
      std::cout << "             throttle: " << driver.GetThrottle() << " steering: " << driver.GetSteering() << std::endl;
      std::cout << std::endl;
      render_frame++;
    }

#ifdef DEBUG_LOG
    if (step_number % output_steps == 0) {
      GetLog() << "\n\n============ System Information ============\n";
      GetLog() << "Time = " << time << "\n\n";
      vehicle.DebugLog(DBG_SHOCKS);
    }
#endif

    // Collect output data from modules (for inter-module communication)
    steering_input = driver.GetSteering();

    tire_forces[FRONT_LEFT.id()] = tire_front_left->GetTireForce();
    tire_forces[FRONT_RIGHT.id()] = tire_front_right->GetTireForce();

    wheel_states[FRONT_LEFT.id()] = vehicle.GetWheelState(FRONT_LEFT);
    wheel_states[FRONT_RIGHT.id()] = vehicle.GetWheelState(FRONT_RIGHT);

    // Update modules (process inputs from other modules)
    time = vehicle.GetChTime();

    driver.Update(time);

    flat_terrain.Update(time);

    tire_front_left->Update(time, wheel_states[FRONT_LEFT.id()]);
    tire_front_right->Update(time, wheel_states[FRONT_RIGHT.id()]);
   

    tester.Update(time, steering_input);

    // Advance simulation for one timestep for all modules
    driver.Advance(step_size);

    flat_terrain.Advance(step_size);

    tire_front_right->Advance(step_size);
    tire_front_left->Advance(step_size);

    tester.Advance(step_size);

    // Increment frame number
    step_number++;
  }

#endif

  return 0;
}
