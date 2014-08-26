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
// If using the Irrlicht interface, river inputs are obtained from the keyboard.
//
// The global reference frame has Z up, X towards the back of the vehicle, and
// Y pointing to the right.
//
// =============================================================================

#include <vector>

#include "core/ChFileutils.h"
#include "core/ChStream.h"
#include "core/ChRealtimeStep.h"
#include "physics/ChSystem.h"
#include "physics/ChLinkDistance.h"

#include "utils/ChUtilsInputOutput.h"

#include "HMMWV_Vehicle.h"

#include "../hmmwv_common/HMMWV.h"
#include "../hmmwv_common/HMMWV_FuncDriver.h"
#include "../hmmwv_common/HMMWV_RigidTire.h"
#include "../hmmwv_common/HMMWV_RigidTerrain.h"

// If Irrlicht support is available...
#if IRRLICHT_ENABLED
  // ...include additional headers
# include "unit_IRRLICHT/ChIrrApp.h"
# include "subsys/driver/ChIrrGuiDriver.h"

  // ...and specify whether the demo should actually use Irrlicht
# define USE_IRRLICHT
#endif


using namespace chrono;
using namespace hmmwv;


// =============================================================================

// Initial vehicle position
ChVector<>     initLoc(40, 0, 1.7);   // sprung mass height at design = 49.68 in
ChQuaternion<> initRot(1,0,0,0);      // forward is in the negative global x-direction

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 100.0;   // size in X direction
double terrainWidth  = 100.0;   // size in Y directoin

// Simulation step size
double step_size = 0.001;

// Rendering FPS
int FPS = 50;

#ifdef USE_IRRLICHT
  // Point on chassis tracked by the camera
  ChVector<> trackPoint(0.0, 0, 1.0);
#else
  double tend = 20.0;

  const std::string out_dir = "../HMMWV";
  const std::string pov_dir = out_dir + "/POVRAY";
#endif

// =============================================================================

int main(int argc, char* argv[])
{
  SetChronoDataPath(CHRONO_DATA_DIR);

  // --------------------------
  // Create the various modules
  // --------------------------

  // Create the HMMWV vehicle
  HMMWV_Vehicle vehicle(false,
                        hmmwv::NONE,
                        hmmwv::NONE);

  vehicle.Initialize(ChCoordsys<>(initLoc, initRot));

  // Create the ground
  HMMWV_RigidTerrain terrain(vehicle, terrainHeight, terrainLength, terrainWidth, 0.8);
  //terrain.AddMovingObstacles(10);
  terrain.AddFixedObstacles();

  // Create the tires
  HMMWV_RigidTire tire_front_right(terrain, 0.7f);
  HMMWV_RigidTire tire_front_left(terrain, 0.7f);
  HMMWV_RigidTire tire_rear_right(terrain, 0.7f);
  HMMWV_RigidTire tire_rear_left(terrain, 0.7f);

  tire_front_right.Initialize(vehicle.GetWheelBody(FRONT_RIGHT));
  tire_front_left.Initialize(vehicle.GetWheelBody(FRONT_LEFT));
  tire_rear_right.Initialize(vehicle.GetWheelBody(REAR_RIGHT));
  tire_rear_left.Initialize(vehicle.GetWheelBody(REAR_LEFT));


#ifdef USE_IRRLICHT
  irr::ChIrrApp application(&vehicle,
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

  if (!do_shadows)
  {
    application.AddTypicalLights(irr::core::vector3df(30.f, -30.f,  100.f),
                                irr::core::vector3df(30.f,  50.f,  100.f),
                                250, 130);
  }
  else
  {
    application.AddLightWithShadow(irr::core::vector3df(20.f,   20.f,  80.f), 
                                   irr::core::vector3df(20.f,   0.f,  0.f), 
                                   150, 60,100, 40, 512, irr::video::SColorf(1,1,1));
  }

  application.SetTimestep(step_size);

  ChIrrGuiDriver driver(application, vehicle, trackPoint, 6, 0.5);

  // Set the time response for steering and throttle keyboard inputs.
  // NOTE: this is not exact, since we do not render quite at the specified FPS.
  double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
  double throttle_time = 1.0;  // time to go from 0 to +1
  driver.SetSteeringDelta(1 / (steering_time * FPS));
  driver.SetThrottleDelta(1 / (throttle_time * FPS));

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

  ChTireForces tire_forces(4);

#ifdef USE_IRRLICHT

  ChRealtimeStepTimer realtime_timer;

  // Refresh 3D view only every N simulation steps
  int simul_substeps_num = (int) std::ceil((1 / step_size) / FPS);
  int simul_substep = 0;

  while (application.GetDevice()->run())
  {
    // Render scene
    if (simul_substep == 0) {
      application.GetVideoDriver()->beginScene(true, true, irr::video::SColor(255, 140, 161, 192));
      driver.DrawAll();
      application.GetVideoDriver()->endScene();
    }

    // Update modules (inter-module communication)
    double time = vehicle.GetChTime();

    driver.Update(time);

    terrain.Update(time);

    tire_front_right.Update(time, vehicle.GetWheelState(FRONT_RIGHT));
    tire_front_left.Update(time, vehicle.GetWheelState(FRONT_LEFT));
    tire_rear_right.Update(time, vehicle.GetWheelState(REAR_RIGHT));
    tire_rear_left.Update(time, vehicle.GetWheelState(REAR_LEFT));

    tire_forces[FRONT_LEFT] = tire_front_left.GetTireForce();
    tire_forces[FRONT_RIGHT] = tire_front_right.GetTireForce();
    tire_forces[REAR_LEFT] = tire_rear_left.GetTireForce();
    tire_forces[REAR_RIGHT] = tire_rear_right.GetTireForce();

    vehicle.Update(time, driver.getThrottle(), driver.getSteering(), tire_forces);

    // Advance simulation for one timestep for all modules
    double step = realtime_timer.SuggestSimulationStep(step_size);

    driver.Advance(step);

    terrain.Advance(step);

    tire_front_right.Advance(step);
    tire_front_left.Advance(step);
    tire_rear_right.Advance(step);
    tire_rear_left.Advance(step);

    vehicle.Advance(step);

    // Increment frame number
    ++simul_substep;
    if (simul_substep >= simul_substeps_num)
      simul_substep = 0;
  }

  application.GetDevice()->drop();

#else

  int out_steps = (int) std::ceil((1 / step_size) / FPS);

  double time = 0;
  int frame = 0;
  int out_frame = 0;

  if(ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
    std::cout << "Error creating directory " << out_dir << std::endl;
    return 1;
  }
  if(ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
    std::cout << "Error creating directory " << pov_dir << std::endl;
    return 1;
  }

  HMMWV_Vehicle::ExportMeshPovray(out_dir);
  HMMWV_WheelLeft::ExportMeshPovray(out_dir);
  HMMWV_WheelRight::ExportMeshPovray(out_dir);

  char filename[100];

  while (time < tend)
  {
    if (frame % out_steps == 0) {
      // Output render data
      sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), out_frame + 1);
      utils::WriteShapesPovray(&m_system, filename);
      std::cout << "Output frame:   " << out_frame << std::endl;
      std::cout << "Sim frame:      " << frame << std::endl;
      std::cout << "Time:           " << time << std::endl;
      std::cout << "             throttle: " << driver.getThrottle() << " steering: " << driver.getSteering() << std::endl;
      std::cout << std::endl;
      out_frame++;
    }

    // Update modules
    driver.Update(time);

    terrain.Update(time);

    tire_front_right.Update(time, vehicle.GetWheelState(FRONT_RIGHT));
    tire_front_left.Update(time, vehicle.GetWheelState(FRONT_LEFT));
    tire_rear_right.Update(time, vehicle.GetWheelState(REAR_RIGHT));
    tire_rear_left.Update(time, vehicle.GetWheelState(REAR_LEFT));

    tire_forces[FRONT_LEFT] = tire_front_left.GetTireForce();
    tire_forces[FRONT_RIGHT] = tire_front_right.GetTireForce();
    tire_forces[REAR_LEFT] = tire_rear_left.GetTireForce();
    tire_forces[REAR_RIGHT] = tire_rear_right.GetTireForce();

    vehicle.Update(time, driver.getThrottle(), driver.getSteering(), tire_forces);

    // Advance simulation for one timestep for all modules
    driver.Advance(step_size);

    terrain.Advance(step_size);

    tire_front_right.Advance(step);
    tire_front_left.Advance(step);
    tire_rear_right.Advance(step);
    tire_rear_left.Advance(step);

    vehicle.Advance(step_size);

    // Increment frame number
    time += step_size;
    frame++;
  }

#endif

  return 0;
}
