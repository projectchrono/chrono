//  - Demo of a  High Mobility Multi-Wheeled Vehicle (HMMWV).
//  - a nice user-based GUI to drive the car with your keyboard.
//  - Also works without the GUI (i.e., no Irrlicht), in which you must specify the 
//  driver throttle/torque each timestep yourself
//  - Using IrrAssets in an efficient way to create bodies, collision
//  objects and visualization assets quickly
//
//  Author: Justin Madsen, 2014
//  Part of the Chrono-T project
//
// The global reference frame has Z up, X towards the back of the vehicle, and
// Y pointing to the right.
// =============================================================================


// TESTING
// #define USE_PACTIRE

// CE includes
#include "core/ChFileutils.h"
#include "core/ChStream.h"
#include "physics/ChSystem.h"
#include "physics/ChLinkDistance.h"

#include "utils/ChUtilsInputOutput.h"

#include "HMMWV_9body_config.h"
#include "HMMWV_9body.h"
#include "HMMWVTerrain.h"

// If Irrlicht support is available...
#if IRRLICHT_ENABLED
  // ...include additional headers
# include "unit_IRRLICHT/ChIrrApp.h"
# include <irrlicht.h>
# include "HMMWVEventReceiver.h"

  // ...and specify whether the demo should actually use Irrlicht
# define USE_IRRLICHT
#endif

// =============================================================================

// Use the namespace of Chrono
using namespace chrono;

// how the tire will supply the reaction forces to the wheel rigid body
enum TireForceType {
	RIGIDCONTACT,
	PACJEKA,
	SOFTSOIL };

#ifdef USE_PACTIRE
	TireForceType tiretype = PACJEKA;
#else
	TireForceType tiretype = RIGIDCONTACT;
#endif


// GLOBAL VARIABLES
ChVector<> cameraOffset(2.5,0,2.0);   // camera should trail the car

ChVector<>     initLoc(0, 0, 1.7);    // sprung mass height at design = 49.68 in
ChQuaternion<> initRot(1,0,0,0);      // forward is the positive x-direction

// if using DVI rigid body contact, how large to make the ground width/length dims?
double terrainWidth = 100.0;
double terrainLength = 100.0;

// simulation parameters
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

  // create the HMMWV vehicle
  HMMWV_9body* mycar = new HMMWV_9body(m_system, initLoc, initRot, false);

  // create the ground
  HMMWVTerrain* terrain = new HMMWVTerrain(m_system, ChVector<>(0, 0, 0),
    terrainWidth, terrainLength, 0.8, 0.0, true);

  // show hierachy before simulation begins
#ifdef _DEBUG
  m_system.ShowHierarchy(GetLog());
#endif

#ifdef USE_IRRLICHT
  irr::ChIrrApp application(&m_system,
                            L"HMMWV 9-body demo",
                            irr::core::dimension2d<irr::u32>(1000, 800),
                            false,
                            true);

  application.AddTypicalSky();
  application.AddTypicalLights();

  application.SetTimestep(step_size);

  // This is for GUI for the on-road HMMWV vehicle
  irr::HMMWVEventReceiver receiver(&application, &m_system, mycar, terrain);

  // Create and manage the camera thru the receiver
  receiver.create_camera(initLoc + cameraOffset, initLoc);

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

      receiver.update_cameraPos(cameraOffset);
      receiver.drawLinks();
      receiver.drawSprings();

      if (receiver.gad_tab_controls->isVisible())
        receiver.drawCarDataOutput();
      if (receiver.gad_tab_wheelState->isVisible())
        receiver.drawWheelData_LF();

      application.DrawAll();

      out_frame++;
    }

    // Advance simulation by one time step
    mycar->ComputeWheelTorque();
    mycar->ComputeSteerDisplacement();
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

  utils::WriteMeshPovray(HMMWV_9body::ChassisMeshFile(), HMMWV_9body::ChassisMeshName(), out_dir);

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

    // Advance simulation by one time step
    mycar->ComputeWheelTorque();
    mycar->ComputeSteerDisplacement();
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
