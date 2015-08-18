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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Main driver function for the HMMWV full model, using rigid tire-terrain
// contact.
//
// If using the Irrlicht interface, driver inputs are obtained from the keyboard.
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

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/tire/ChPacejkaTire.h"

#include "ModelDefs.h"
#include "hmmwv/vehicle/HMMWV_Vehicle.h"
#include "hmmwv/powertrain/HMMWV_Powertrain.h"
#include "hmmwv/powertrain/HMMWV_SimplePowertrain.h"
#include "hmmwv/tire/HMMWV_RigidTire.h"
#include "hmmwv/tire/HMMWV_LugreTire.h"
#include "hmmwv/HMMWV_FuncDriver.h"

// If Irrlicht support is available...
#ifdef CHRONO_IRRLICHT
  // ...include additional headers
# include "unit_IRRLICHT/ChIrrApp.h"
# include "chrono_vehicle/driver/ChIrrGuiDriver.h"

  // ...and specify whether the demo should actually use Irrlicht
# define USE_IRRLICHT
#endif

// DEBUGGING:  Uncomment the following line to print shock data
//#define DEBUG_LOG

using namespace chrono;
using namespace hmmwv;

// =============================================================================

// Initial vehicle position
ChVector<> initLoc(0, 0, 1.0);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);
//ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
//ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
//ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);
//ChQuaternion<> initRot(0, 0, 0, 1);

// Type of powertrain model (SHAFTS, SIMPLE)
PowertrainModelType powertrain_model = SHAFTS;

// Type of tire model (RIGID, PACEJKA, or LUGRE)
TireModelType tire_model = RIGID;

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 100.0;   // size in X direction
double terrainWidth  = 100.0;   // size in Y direction

// Simulation step size
double step_size = 0.001;

// Time interval between two render frames
double render_step_size = 1.0 / 50;   // FPS = 50

// Time interval between two output frames
double output_step_size = 1.0 / 1;    // once a second

#ifdef USE_IRRLICHT
  // Point on chassis tracked by the camera
  ChVector<> trackPoint(0.0, 0.0, 1.75);
#else
  double tend = 20.0;

  const std::string out_dir = "../HMMWV";
  const std::string pov_dir = out_dir + "/POVRAY";
#endif

// =============================================================================

int main(int argc, char* argv[])
{
  // --------------------------
  // Create the various modules
  // --------------------------

  // Create the HMMWV vehicle
  HMMWV_Vehicle vehicle(false,
                        RWD,
                        PRIMITIVES,
                        MESH);

  vehicle.Initialize(ChCoordsys<>(initLoc, initRot));

  // Create the ground
  RigidTerrain terrain(vehicle.GetSystem(), terrainHeight, terrainLength, terrainWidth, 0.8);
  //terrain.AddMovingObstacles(10);
  terrain.AddFixedObstacles();

  // Create and initialize the powertrain system
  ChSharedPtr<ChPowertrain> powertrain;

  switch (powertrain_model) {
  case SHAFTS:
  {
    ChSharedPtr<HMMWV_Powertrain> ptrain(new HMMWV_Powertrain);

    ptrain->Initialize(vehicle.GetChassis(), vehicle.GetDriveshaft());
    powertrain = ptrain;

    break;
  }
  case SIMPLE:
  {
    ChSharedPtr<HMMWV_SimplePowertrain> ptrain(new HMMWV_SimplePowertrain);

    ptrain->Initialize();
    powertrain = ptrain;

    break;
  }
  }

  // Create and initialize the tires
  ChSharedPtr<ChTire> tire_front_right;
  ChSharedPtr<ChTire> tire_front_left;
  ChSharedPtr<ChTire> tire_rear_right;
  ChSharedPtr<ChTire> tire_rear_left;

  switch (tire_model) {
  case RIGID:
  {
    ChSharedPtr<HMMWV_RigidTire> tire_FL(new HMMWV_RigidTire("FL", terrain, 0.7f));
    ChSharedPtr<HMMWV_RigidTire> tire_FR(new HMMWV_RigidTire("FR", terrain, 0.7f));
    ChSharedPtr<HMMWV_RigidTire> tire_RL(new HMMWV_RigidTire("RL", terrain, 0.7f));
    ChSharedPtr<HMMWV_RigidTire> tire_RR(new HMMWV_RigidTire("RR", terrain, 0.7f));

    tire_FL->Initialize(vehicle.GetWheelBody(FRONT_LEFT));
    tire_FR->Initialize(vehicle.GetWheelBody(FRONT_RIGHT));
    tire_RL->Initialize(vehicle.GetWheelBody(REAR_LEFT));
    tire_RR->Initialize(vehicle.GetWheelBody(REAR_RIGHT));

    tire_front_left = tire_FL;
    tire_front_right = tire_FR;
    tire_rear_left = tire_RL;
    tire_rear_right = tire_RR;

    break;
  }
  case LUGRE:
  {
    ChSharedPtr<HMMWV_LugreTire> tire_FL(new HMMWV_LugreTire("FL", terrain));
    ChSharedPtr<HMMWV_LugreTire> tire_FR(new HMMWV_LugreTire("FR", terrain));
    ChSharedPtr<HMMWV_LugreTire> tire_RL(new HMMWV_LugreTire("RL", terrain));
    ChSharedPtr<HMMWV_LugreTire> tire_RR(new HMMWV_LugreTire("RR", terrain));

    tire_FL->Initialize();
    tire_FR->Initialize();
    tire_RL->Initialize();
    tire_RR->Initialize();

    tire_front_left = tire_FL;
    tire_front_right = tire_FR;
    tire_rear_left = tire_RL;
    tire_rear_right = tire_RR;

    break;
  }
  case PACEJKA:
  {
    std::string param_file = vehicle::GetDataFile("hmmwv/tire/HMMWV_pacejka.tir");

    ChSharedPtr<ChPacejkaTire> tire_FL(new ChPacejkaTire("FL", param_file, terrain));
    ChSharedPtr<ChPacejkaTire> tire_FR(new ChPacejkaTire("FR", param_file, terrain));
    ChSharedPtr<ChPacejkaTire> tire_RL(new ChPacejkaTire("RL", param_file, terrain));
    ChSharedPtr<ChPacejkaTire> tire_RR(new ChPacejkaTire("RR", param_file, terrain));

    tire_FL->Initialize(LEFT,false);
    tire_FR->Initialize(RIGHT,false);
    tire_RL->Initialize(LEFT,true);
    tire_RR->Initialize(RIGHT,true);

    tire_front_left = tire_FL;
    tire_front_right = tire_FR;
    tire_rear_left = tire_RL;
    tire_rear_right = tire_RR;

    break;
  }
  }


#ifdef USE_IRRLICHT
  irr::ChIrrApp application(vehicle.GetSystem(),
                            L"HMMWV demo",
                            irr::core::dimension2d<irr::u32>(1000, 800),
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
  mbox->setRotation( irr::core::vector3df(90,0,0));
 
  bool do_shadows = false; // shadow map is experimental
  irr::scene::ILightSceneNode* mlight = 0;

  if (do_shadows) {
    mlight = application.AddLightWithShadow(
      irr::core::vector3df(10.f, 30.f, 60.f),
      irr::core::vector3df(0.f, 0.f, 0.f),
      150, 60, 80, 15, 512, irr::video::SColorf(1, 1, 1), false, false);
  } else {
    application.AddTypicalLights(
      irr::core::vector3df(30.f, -30.f, 100.f),
      irr::core::vector3df(30.f, 50.f, 100.f),
      250, 130);
  }

  application.SetTimestep(step_size);

  ChIrrGuiDriver driver(application, vehicle, *powertrain.get_ptr(), trackPoint, 6.0, 0.5, true);
  driver.LogInit("driver_inputs.out");

  // Set the time response for steering and throttle keyboard inputs.
  // NOTE: this is not exact, since we do not render quite at the specified FPS.
  double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
  double throttle_time = 1.0;  // time to go from 0 to +1
  double braking_time = 0.3;   // time to go from 0 to +1
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
#else
  HMMWV_FuncDriver driver;
#endif


  // ---------------
  // Simulation loop
  // ---------------

#ifdef DEBUG_LOG
  GetLog() << "\n\n============ System Configuration ============\n";
  vehicle.LogHardpointLocations();
#endif

  // Inter-module communication data
  ChTireForces  tire_forces(4);
  ChWheelState  wheel_states[4];
  double        driveshaft_speed;
  double        powertrain_torque;
  double        throttle_input;
  double        steering_input;
  double        braking_input;

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
      ChVector<> lightaim = vehicle.GetChassisPos();
      ChVector<> lightpos = vehicle.GetChassisPos() + ChVector<>(10, 30, 60);
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
      vehicle.DebugLog(DBG_SPRINGS | DBG_SHOCKS | DBG_CONSTRAINTS);
    }
#endif

    // Collect output data from modules (for inter-module communication)
    throttle_input = driver.GetThrottle();
    steering_input = driver.GetSteering();
    braking_input = driver.GetBraking();

    powertrain_torque = powertrain->GetOutputTorque();

    tire_forces[FRONT_LEFT.id()] = tire_front_left->GetTireForce();
    tire_forces[FRONT_RIGHT.id()] = tire_front_right->GetTireForce();
    tire_forces[REAR_LEFT.id()] = tire_rear_left->GetTireForce();
    tire_forces[REAR_RIGHT.id()] = tire_rear_right->GetTireForce();

    driveshaft_speed = vehicle.GetDriveshaftSpeed();

    wheel_states[FRONT_LEFT.id()] = vehicle.GetWheelState(FRONT_LEFT);
    wheel_states[FRONT_RIGHT.id()] = vehicle.GetWheelState(FRONT_RIGHT);
    wheel_states[REAR_LEFT.id()] = vehicle.GetWheelState(REAR_LEFT);
    wheel_states[REAR_RIGHT.id()] = vehicle.GetWheelState(REAR_RIGHT);

    // Update modules (process inputs from other modules)
    time = vehicle.GetSystem()->GetChTime();

    driver.Update(time);
    if (step_number % output_steps == 0)
      driver.Log(time);

    terrain.Update(time);

    tire_front_left->Update(time, wheel_states[FRONT_LEFT.id()]);
    tire_front_right->Update(time, wheel_states[FRONT_RIGHT.id()]);
    tire_rear_left->Update(time, wheel_states[REAR_LEFT.id()]);
    tire_rear_right->Update(time, wheel_states[REAR_RIGHT.id()]);

    powertrain->Update(time, throttle_input, driveshaft_speed);

    vehicle.Update(time, steering_input, braking_input, powertrain_torque, tire_forces);

    // Advance simulation for one timestep for all modules
    double step = realtime_timer.SuggestSimulationStep(step_size);

    driver.Advance(step);

    terrain.Advance(step);

    tire_front_left->Advance(step);
    tire_front_right->Advance(step);
    tire_rear_right->Advance(step);
    tire_rear_left->Advance(step);

    powertrain->Advance(step);

    vehicle.Advance(step);

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
      utils::WriteShapesPovray(vehicle.GetSystem(), filename);
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
    throttle_input = driver.GetThrottle();
    steering_input = driver.GetSteering();
    braking_input = driver.GetBraking();

    powertrain_torque = powertrain->GetOutputTorque();

    tire_forces[FRONT_LEFT.id()] = tire_front_left->GetTireForce();
    tire_forces[FRONT_RIGHT.id()] = tire_front_right->GetTireForce();
    tire_forces[REAR_LEFT.id()] = tire_rear_left->GetTireForce();
    tire_forces[REAR_RIGHT.id()] = tire_rear_right->GetTireForce();

    driveshaft_speed = vehicle.GetDriveshaftSpeed();

    wheel_states[FRONT_LEFT.id()] = vehicle.GetWheelState(FRONT_LEFT);
    wheel_states[FRONT_RIGHT.id()] = vehicle.GetWheelState(FRONT_RIGHT);
    wheel_states[REAR_LEFT.id()] = vehicle.GetWheelState(REAR_LEFT);
    wheel_states[REAR_RIGHT.id()] = vehicle.GetWheelState(REAR_RIGHT);

    // Update modules (process inputs from other modules)
    time = vehicle.GetSystem()->GetChTime();

    driver.Update(time);

    terrain.Update(time);

    tire_front_left->Update(time, wheel_states[FRONT_LEFT.id()]);
    tire_front_right->Update(time, wheel_states[FRONT_RIGHT.id()]);
    tire_rear_left->Update(time, wheel_states[REAR_LEFT.id()]);
    tire_rear_right->Update(time, wheel_states[REAR_RIGHT.id()]);

    powertrain->Update(time, throttle_input, driveshaft_speed);

    vehicle.Update(time, steering_input, braking_input, powertrain_torque, tire_forces);

    // Advance simulation for one timestep for all modules
    driver.Advance(step_size);

    terrain.Advance(step_size);

    tire_front_right->Advance(step_size);
    tire_front_left->Advance(step_size);
    tire_rear_right->Advance(step_size);
    tire_rear_left->Advance(step_size);

    powertrain->Advance(step_size);

    vehicle.Advance(step_size);

    // Increment frame number
    step_number++;
  }

#endif

  return 0;
}
