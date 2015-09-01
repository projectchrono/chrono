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
// Authors: Radu Serban
// =============================================================================
//
// Main driver function for a vehicle specified through JSON files.
//
// If using the Irrlicht interface, driver inputs are obtained from the keyboard.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <vector>

#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChStream.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_vehicle/vehicle/Vehicle.h"
#include "chrono_vehicle/powertrain/SimplePowertrain.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"

#include "chrono_vehicle/ChConfigVehicle.h"

// If Irrlicht support is available...
#ifdef CHRONO_IRRLICHT
  // ...include additional headers
# include "chrono_irrlicht/ChIrrApp.h"
# include "chrono_vehicle/driver/ChIrrGuiDriver.h"

  // ...and specify whether the demo should actually use Irrlicht
# define USE_IRRLICHT
#endif

using namespace chrono;

// =============================================================================

// JSON file for vehicle model
//std::string vehicle_file("hmmwv/vehicle/HMMWV_Vehicle.json");
//std::string vehicle_file("hmmwv/vehicle/HMMWV_Vehicle_simple_lugged.json");
//std::string vehicle_file("hmmwv/vehicle/HMMWV_Vehicle_4WD.json");
//std::string vehicle_file("generic/vehicle/Vehicle_DoubleWishbones.json");
std::string vehicle_file("generic/vehicle/Vehicle_DoubleWishbones_ARB.json");
//std::string vehicle_file("generic/vehicle/Vehicle_MultiLinks.json");
//std::string vehicle_file("generic/vehicle/Vehicle_SolidAxles.json");
//std::string vehicle_file("generic/vehicle/Vehicle_ThreeAxles.json");
//std::string vehicle_file("generic/vehicle_multisteer/Vehicle_DualFront_Independent.json");
//std::string vehicle_file("generic/vehicle_multisteer/Vehicle_DualFront_Shared.json");

// JSON files for tire models (rigid) and powertrain (simple)
std::string rigidtire_file("generic/tire/RigidTire.json");
std::string simplepowertrain_file("generic/powertrain/SimplePowertrain.json");

// Driver input file (if not using Irrlicht)
std::string driver_file("generic/driver/Sample_Maneuver.txt");

// Initial vehicle position
ChVector<> initLoc(0, 0, 1.0);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);
//ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
//ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
//ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);
//ChQuaternion<> initRot(0, 0, 0, 1);

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 300.0;   // size in X direction
double terrainWidth  = 200.0;   // size in Y direction

// Simulation step size
double step_size = 1e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 50;   // FPS = 50

// Time interval between two output frames
double output_step_size = 1.0 / 1;    // once a second

// Point on chassis tracked by the camera (Irrlicht only)
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Simulation length (Povray only)
double tend = 20.0;

// Output directories (Povray only)
const std::string out_dir = "../VEHICLE";
const std::string pov_dir = out_dir + "/POVRAY";

// =============================================================================

int main(int argc, char* argv[])
{
  // --------------------------
  // Create the various modules
  // --------------------------

  // Create the vehicle system
  Vehicle vehicle(vehicle::GetDataFile(vehicle_file));
  vehicle.Initialize(ChCoordsys<>(initLoc, initRot));
  ////vehicle.GetChassis()->SetBodyFixed(true);

  // Create the ground
  RigidTerrain terrain(vehicle.GetSystem(), terrainHeight, terrainLength, terrainWidth, 0.8);
  terrain.AddFixedObstacles();

  // Create and initialize the powertrain system
  SimplePowertrain powertrain(vehicle::GetDataFile(simplepowertrain_file));
  powertrain.Initialize();

  // Create and initialize the tires
  int num_axles = vehicle.GetNumberAxles();
  int num_wheels = 2 * num_axles;
  std::vector<ChSharedPtr<RigidTire> > tires(num_wheels);

  for (int i = 0; i < num_wheels; i++) {
    tires[i] = ChSharedPtr<RigidTire>(new RigidTire(vehicle::GetDataFile(rigidtire_file), terrain));
    tires[i]->Initialize(vehicle.GetWheelBody(i));
  }

#ifdef USE_IRRLICHT

  irr::ChIrrApp application(vehicle.GetSystem(),
                            L"Vehicle demo",
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

  ChIrrGuiDriver driver(application, vehicle, powertrain, trackPoint, 6.0, 0.5, true);

  // Set the time response for steering and throttle keyboard inputs.
  // NOTE: this is not exact, since we do not render quite at the specified FPS.
  double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
  double throttle_time = 1.0;  // time to go from 0 to +1
  double braking_time = 0.3;   // time to go from 0 to +1
  driver.SetSteeringDelta(render_step_size / steering_time);
  driver.SetThrottleDelta(render_step_size / throttle_time);
  driver.SetBrakingDelta(render_step_size / braking_time);

  // Set file with driver input time series
  driver.SetInputDataFile(vehicle::GetDataFile(driver_file));

  // Set up the assets for rendering
  application.AssetBindAll();
  application.AssetUpdateAll();
  if (do_shadows)
    application.AddShadowAll();

#else

  ChDataDriver driver(vehicle::GetDataFile(driver_file));

#endif


  // ---------------
  // Simulation loop
  // ---------------

  // Inter-module communication data
  ChTireForces   tire_forces(num_wheels);
  ChWheelStates  wheel_states(num_wheels);
  double         driveshaft_speed;
  double         powertrain_torque;
  double         throttle_input;
  double         steering_input;
  double         braking_input;

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
    // Render scene
    if (step_number % render_steps == 0) {
      // Update the position of the shadow mapping so that it follows the car
      if (do_shadows) {
        ChVector<> lightaim = vehicle.GetChassisPos();
        ChVector<> lightpos = vehicle.GetChassisPos() + ChVector<>(10, 30, 60);
        irr::core::vector3df mlightpos((irr::f32)lightpos.x, (irr::f32)lightpos.y, (irr::f32)lightpos.z);
        irr::core::vector3df mlightaim((irr::f32)lightaim.x, (irr::f32)lightaim.y, (irr::f32)lightaim.z);
        application.GetEffects()->getShadowLight(0).setPosition(mlightpos);
        application.GetEffects()->getShadowLight(0).setTarget(mlightaim);
        mlight->setPosition(mlightpos);
      }

      // Draw all scene elements
      application.GetVideoDriver()->beginScene(true, true, irr::video::SColor(255, 140, 161, 192));
      driver.DrawAll();
      application.GetVideoDriver()->endScene();
    }

    // Collect output data from modules (for inter-module communication)
    throttle_input = driver.GetThrottle();
    steering_input = driver.GetSteering();
    braking_input = driver.GetBraking();
    powertrain_torque = powertrain.GetOutputTorque();
    driveshaft_speed = vehicle.GetDriveshaftSpeed();
    for (int i = 0; i < num_wheels; i++) {
      tire_forces[i] = tires[i]->GetTireForce();
      wheel_states[i] = vehicle.GetWheelState(i);
    }

    // Update modules (process inputs from other modules)
    time = vehicle.GetSystem()->GetChTime();
    driver.Update(time);
    powertrain.Update(time, throttle_input, driveshaft_speed);
    vehicle.Update(time, steering_input, braking_input, powertrain_torque, tire_forces);
    terrain.Update(time);
    for (int i = 0; i < num_wheels; i++)
      tires[i]->Update(time, wheel_states[i]);

    // Advance simulation for one timestep for all modules
    double step = realtime_timer.SuggestSimulationStep(step_size);
    driver.Advance(step);
    powertrain.Advance(step);
    vehicle.Advance(step);
    terrain.Advance(step);
    for (int i = 0; i < num_wheels; i++)
      tires[i]->Advance(step);

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
      std::cout << "   throttle: " << driver.GetThrottle()
                << "   steering: " << driver.GetSteering()
                << "   braking:  " << driver.GetBraking() << std::endl;
      std::cout << std::endl;
      render_frame++;
    }

    // Collect output data from modules (for inter-module communication)
    throttle_input = driver.GetThrottle();
    steering_input = driver.GetSteering();
    braking_input = driver.GetBraking();
    powertrain_torque = powertrain.GetOutputTorque();
    driveshaft_speed = vehicle.GetDriveshaftSpeed();
    for (int i = 0; i < num_wheels; i++) {
      tire_forces[i] = tires[i]->GetTireForce();
      wheel_states[i] = vehicle.GetWheelState(i);
    }

    // Update modules (process inputs from other modules)
    time = vehicle.GetSystem()->GetChTime();
    driver.Update(time);
    powertrain.Update(time, throttle_input, driveshaft_speed);
    vehicle.Update(time, steering_input, braking_input, powertrain_torque, tire_forces);
    terrain.Update(time);
    for (int i = 0; i < num_wheels; i++)
      tires[i]->Update(time, wheel_states[i]);

    // Advance simulation for one timestep for all modules
    driver.Advance(step_size);
    powertrain.Advance(step_size);
    vehicle.Advance(step_size);
    terrain.Advance(step_size);
    for (int i = 0; i < num_wheels; i++)
      tires[i]->Advance(step_size);

    // Increment frame number
    step_number++;
  }

#endif

  return 0;
}
