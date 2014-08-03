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
#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"
#include "physics/ChLinkDistance.h"

#include "utils/ChUtilsInputOutput.h"

#include "HMMWV_9body_config.h"
#include "HMMWV9_Vehicle.h"
#include "HMMWV9_RigidTerrain.h"

// If Irrlicht support is available...
#if IRRLICHT_ENABLED
  // ...include additional headers
# include "unit_IRRLICHT/ChIrrApp.h"
# include <irrlicht.h>
# include "HMMWV9_IrrGuiDriver.h"
# include "HMMWV9_IrrRenderer.h"
  // ...and specify whether the demo should actually use Irrlicht
# define USE_IRRLICHT
#endif


using namespace chrono;

// =============================================================================

// Initial vehicle position
ChVector<>     initLoc(0, 0, 1.7);    // sprung mass height at design = 49.68 in
ChQuaternion<> initRot(1,0,0,0);      // forward is the positive x-direction

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 100.0;  // size in X direction
double terrainWidth  = 100.0;   // size in Y directoin

// Camera offset relative to vehicle
ChVector<> cameraOffset(2.5, 0, 2.0);

// Simulation parameters
double tend = 10.0;
double step_size = 0.0005;
int out_fps = 30;

// Output directories
const std::string out_dir = "../HMMWV9";
const std::string pov_dir = out_dir + "/POVRAY";

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
                         false);

  // Create the ground
  HMMWV9_RigidTerrain terrain(m_system, terrainHeight, terrainLength, terrainWidth, 0.8);


#ifdef USE_IRRLICHT
  irr::ChIrrApp application(&m_system,
                            L"HMMWV 9-body demo",
                            irr::core::dimension2d<irr::u32>(1000, 800),
                            false,
                            true);

  application.AddTypicalSky();
  application.AddTypicalLights();

  application.SetTimestep(step_size);

  HMMWV9_IrrGuiDriver driver(application);
  HMMWV9_IrrRenderer  renderer(application, vehicle, terrainHeight, initLoc + cameraOffset, initLoc, cameraOffset);

  // Set up the assets for rendering
  application.AssetBindAll();
  application.AssetUpdateAll();
#endif

  // ---------------
  // Simulation loop
  // ---------------

  int out_steps = std::ceil((1 / step_size) / out_fps);

  double time = 0;
  int frame = 0;
  int out_frame = 0;

#ifdef USE_IRRLICHT
  while (application.GetDevice()->run())
  {
    bool render = (frame % out_steps == 0);

    // Render scene
    if (render) {
      application.GetVideoDriver()->beginScene(true, true, irr::video::SColor(255, 140, 161, 192));

      application.DrawAll();
      renderer.OnFrame();

      out_frame++;
    }

    // Update subsystems and advance simulation by one time step
    driver.Update(time);
    vehicle.Update(time, driver.getThrottle(), driver.getSteering());

    m_system.DoStepDynamics(step_size);
    time += step_size;
    frame++;

    // Complete scene
    if (render)
      application.GetVideoDriver()->endScene();
  }
#else
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
      std::cout << "------------ Output frame:   " << out_frame << std::endl;
      std::cout << "             Sim frame:      " << frame << std::endl;
      std::cout << "             Time:           " << time << std::endl;
      out_frame++;

      break;

    }

    // Update subsystems and advance simulation by one time step
    ////driver.Update(time);
    ////vehicle.Update(time, driver.getThrottle(), driver.getSteering());

    m_system.DoStepDynamics(step_size);
    time += step_size;
    frame++;
  }
#endif

#ifdef USE_IRRLICHT
  application.GetDevice()->drop();
#endif

  DLL_DeleteGlobals();

  return 0;
}
