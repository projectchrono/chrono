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
// Main driver function for the HMMWV 9-body model, using rigid tire-terrain
// contact.
//
// If using the Irrlicht interface, river inputs are obtained from the keyboard.
//
// The global reference frame has Z up, X towards the back of the vehicle, and
// Y pointing to the right.
//
// =============================================================================


#include "core/ChFileutils.h"
#include "core/ChStream.h"
#include "core/ChRealtimeStep.h"
#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"
#include "physics/ChLinkDistance.h"

#include "utils/ChUtilsInputOutput.h"

#include "HMMWV9_Vehicle.h"
#include "HMMWV9_FuncDriver.h"
#include "HMMWV9_RigidTerrain.h"

// If Irrlicht support is available...
#if IRRLICHT_ENABLED
  // ...include additional headers
# include "unit_IRRLICHT/ChIrrApp.h"
# include "subsys/driver/ChIrrGuiDriver.h"

  // ...and specify whether the demo should actually use Irrlicht
# define USE_IRRLICHT
#endif


using namespace chrono;
using namespace hmmwv9;


// =============================================================================

// Initial vehicle position
ChVector<>     initLoc(0, 0, 1.7);    // sprung mass height at design = 49.68 in
ChQuaternion<> initRot(1,0,0,0);      // forward is the positive x-direction

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 100.0;   // size in X direction
double terrainWidth  = 100.0;   // size in Y directoin

// Simulation step size
double step_size = 0.001;

#ifdef USE_IRRLICHT
  // Point on chassis tracked by the camera
  ChVector<> trackPoint(0.5, 0, 1.0);
#else
  double tend = 20.0;
  int out_fps = 30;

  const std::string out_dir = "../HMMWV9";
  const std::string pov_dir = out_dir + "/POVRAY";
#endif

// =============================================================================

int main(int argc, char* argv[])
{
  DLL_CreateGlobals();

  // -----------------
  // Create the system
  // -----------------

  ChSystem m_system;

  m_system.Set_G_acc(ChVector<>(0, 0, -9.81));

  // Integration and Solver settings
  m_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
  m_system.SetIterLCPmaxItersSpeed(150);
  m_system.SetIterLCPmaxItersStab(150);
  m_system.SetMaxPenetrationRecoverySpeed(4.0);
  m_system.SetStep(step_size);

  // Create the HMMWV vehicle
  HMMWV9_Vehicle vehicle(m_system,
                         ChCoordsys<>(initLoc, initRot),
                         false,
                         hmmwv9::NONE,
                         hmmwv9::PRIMITIVES);

  // Create the ground
  HMMWV9_RigidTerrain terrain(m_system, terrainHeight, terrainLength, terrainWidth, 0.8);
  //terrain.AddMovingObstacles(10);
  terrain.AddFixedObstacles();

#ifdef USE_IRRLICHT
  irr::ChIrrApp application(&m_system,
                            L"HMMWV 9-body demo",
                            irr::core::dimension2d<irr::u32>(1000, 800),
                            false,
                            true);

  // make a skybox that has Z pointing up (default application.AddTypicalSky() makes Y up) 
  irr::core::stringc mtexturedir("../data/skybox/");
  irr::core::stringc str_lf = mtexturedir + "sky_lf.jpg";
  irr::core::stringc str_up = mtexturedir + "sky_up.jpg";
  irr::core::stringc str_dn = mtexturedir + "sky_dn.jpg";
  irr::video::ITexture* map_skybox_side = 
	 application.GetVideoDriver()->getTexture(str_lf.c_str() );
  irr::scene::ISceneNode* mbox = application.GetSceneManager()->addSkyBoxSceneNode(
     application.GetVideoDriver()->getTexture(str_up.c_str()),
     application.GetVideoDriver()->getTexture(str_dn.c_str()),
     map_skybox_side,
     map_skybox_side,
     map_skybox_side,
     map_skybox_side);
  mbox->setRotation( irr::core::vector3df(90,0,0));
 

  application.AddTypicalLights( irr::core::vector3df(30.f, -30.f,  100.f),
								irr::core::vector3df(30.f,  50.f,  100.f), 
								250,130 );

  application.SetTimestep(step_size);

  ChIrrGuiDriver driver(application, vehicle);

  driver.CreateCamera(trackPoint, 6, 0.5);

  // Set up the assets for rendering
  application.AssetBindAll();
  application.AssetUpdateAll();
#else
  HMMWV9_FuncDriver driver;
#endif

  // ---------------
  // Simulation loop
  // ---------------


#ifdef USE_IRRLICHT

  ChRealtimeStepTimer realtime_timer;

  while (application.GetDevice()->run())
  {
    // Render scene
    application.GetVideoDriver()->beginScene(true, true, irr::video::SColor(255, 140, 161, 192));

    driver.UpdateCamera(step_size);

    driver.DrawAll();

    // Update subsystems and advance simulation by one time step
    double time = m_system.GetChTime();
    driver.Update(time);
    vehicle.Update(time, driver.getThrottle(), driver.getSteering());

    m_system.DoStepDynamics(realtime_timer.SuggestSimulationStep(step_size));

    // Complete scene
    application.GetVideoDriver()->endScene();
  }

  application.GetDevice()->drop();

#else

  int out_steps = std::ceil((1 / step_size) / out_fps);

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

  utils::WriteMeshPovray(HMMWV9_Vehicle::ChassisMeshFile(), HMMWV9_Vehicle::ChassisMeshName(), out_dir);
  utils::WriteMeshPovray(HMMWV9_Wheel::MeshFile(), HMMWV9_Wheel::MeshName(), out_dir);

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

    // Update subsystems and advance simulation by one time step
    driver.Update(time);
    vehicle.Update(time, driver.getThrottle(), driver.getSteering());

    m_system.DoStepDynamics(step_size);

    time += step_size;
    frame++;
  }

#endif

  DLL_DeleteGlobals();

  return 0;
}
