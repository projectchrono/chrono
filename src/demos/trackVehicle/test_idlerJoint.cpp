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
// Authors: Justin Madsen, Radu Serban
// =============================================================================
//
// Original Chrono demo for slider-crank by Radu Serban:
// 	2-body slider-crank modeled with a crank body and a rod body.
//	The crank and rod are connected through a revolute joint and the rod tip is constrained to move along the global X axis.
//	The crank can be connected to ground either through a revolute joint or through a constant speed motor.
// Built along side slider crank is a slider with a ChLinkLockRevolutePrismatic, which is actuated to have the same kinematics as the slider-crank.
// 	a second weight is connected to the new slider body with a spring, in such a way as to induce reactions in all constrained DOFs.
//
// The global reference frame is Y up.
// =============================================================================

#include <stdio.h>
#include <cmath>

#include "core/ChFileutils.h"
#include "core/ChStream.h"

#include "physics/ChSystem.h"

#include "utils/ChUtilsCreators.h"
#include "utils/ChUtilsInputOutput.h"

#include <irrlicht.h>
#include "unit_IRRLICHT/ChIrrApp.h"

using namespace chrono;
using namespace irr;

// -----------------------------------------------------------------------------
// Simulation parameters
// Magnitude of the gravitational acceleration
const double gravity = 9.81;

// Simulation time, integration time-step, and output FPS
// const double time_end = 4.0;
const double time_step = 0.001;
const double out_fps = 50;
const double render_fps = 50;

// Name of the output folder
const std::string out_dir = "../SLIDER_CRANK";

// Name of the subfolder for PovRay output files (one file per output frame)
const std::string pov_dir = out_dir + "/POVRAY";

// Name of the output file with body positions (one line per output frame)
const std::string out_file = out_dir + "/output.txt";

// Uncomment the following line for a driven slider-crank (using a constant
// angular velocity engine). Otherwise, the mechanism moves under the action of
// gravity only.
#define DRIVEN


// -----------------------------------------------------------------------------
// Append a new line to the output file, containing body positions and
// orientations at the current time.
// -----------------------------------------------------------------------------
void OutputData(ChSystem*             system,
                ChStreamOutAsciiFile& ofile,
                double                time)
{
  ofile << time << ", ";

  for (int i = 0; i < system->Get_bodylist()->size(); i++) {
    ChBody* body = (ChBody*) system->Get_bodylist()->at(i);

    if (!body->IsActive())
      continue;

    const ChVector<>& pos = body->GetPos();
    const ChQuaternion<>& rot = body->GetRot();

    ofile << pos.x  << ", " << pos.y  << ", " << pos.z  << ", ";
    ofile << rot.e0 << ", " << rot.e1 << ", " << rot.e2 << ", " << rot.e3 << ", ";
  }

  ofile << "\n";
}

// -----------------------------------------------------------------------------
// Create the mechanical system, define bodies and joints, and perform
// simulation.
int main(int   argc,
         char* argv[])
{
  // 0. Create output directories.
  if(ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
    std::cout << "Error creating directory " << out_dir << std::endl;
    return 1;
  }
  if(ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
    std::cout << "Error creating directory " << pov_dir << std::endl;
    return 1;
  }

  ChStreamOutAsciiFile ofile(out_file.c_str());

  // 1. Create the physical system that will handle all bodies and constraints.
  //    Specify the gravitational acceleration vector, consistent with the
  //    global reference frame being Y up.
  ChSystem system;
  system.Set_G_acc(ChVector<>(0, -gravity, 0));

  // 2. Create the rigid bodies of the slider-crank mechanical system.
  ChQuaternion<> y2x;
  ChQuaternion<> z2y;
  y2x.Q_from_AngAxis(-CH_C_PI/2, ChVector<>(0, 0, 1));
  z2y.Q_from_AngAxis(-CH_C_PI/2, ChVector<>(1, 0, 0));

  // Ground
  ChSharedBodyPtr  ground(new ChBody);
  system.AddBody(ground);
  ground->SetIdentifier(-1);
  ground->SetBodyFixed(true);
  ground->SetCollide(false);
  ground->GetCollisionModel()->ClearModel();
  utils::AddCylinderGeometry(ground.get_ptr(), 0.15, 0.1);
  ground->GetCollisionModel()->BuildModel();

  // Crank
  ChSharedBodyPtr  crank(new ChBody);
  system.AddBody(crank);
  crank->SetIdentifier(1);
  crank->SetPos(ChVector<>(1,0,0));
  crank->SetCollide(false);
  crank->GetCollisionModel()->ClearModel();
  utils::AddCapsuleGeometry(crank.get_ptr(), 0.1, 1, ChVector<>(0, 0, 0), y2x);
  utils::AddCylinderGeometry(crank.get_ptr(), 0.15, 0.1, ChVector<>(1, 0, 0));
  crank->GetCollisionModel()->BuildModel();

  // Rod
  ChSharedBodyPtr  rod(new ChBody);
  system.AddBody(rod);
  rod->SetIdentifier(2);
  rod->SetPos(ChVector<>(4,0,0));
  rod->SetCollide(false);
  rod->GetCollisionModel()->ClearModel();
  utils::AddCapsuleGeometry(rod.get_ptr(), 0.1, 2, ChVector<>(0, 0, 0), y2x);
  utils::AddSphereGeometry(rod.get_ptr(), 0.15, ChVector<>(2, 0, 0));
  rod->GetCollisionModel()->BuildModel();

  // 3. Create joint constraints.
  //    Notes:
  //    - All joint locations are specified in the global frame.
  //    - The rotational axis of a revolute joint is along the Z axis of the
  //      specified coordinate frame.

  // Revolute joint between crank and rod.
  ChSharedPtr<ChLinkLockRevolute>  rev_crank_rod(new ChLinkLockRevolute);
  rev_crank_rod->Initialize(crank, rod, ChCoordsys<>(ChVector<>(2, 0, 0), QUNIT));
  system.AddLink(rev_crank_rod);

  // Slider (point on line) joint between rod and ground.
  ChSharedPtr<ChLinkLockPointLine> slider_rod_ground(new ChLinkLockPointLine);
  slider_rod_ground->Initialize(rod, ground, ChCoordsys<>(ChVector<>(6, 0, 0)));
  system.AddLink(slider_rod_ground);

#ifdef DRIVEN
  // Engine between ground and crank (also acts as a revolute joint).
  // Enforce constant angular speed of PI rad/s
  ChSharedPtr<ChLinkEngine> engine_ground_crank(new ChLinkEngine);
  engine_ground_crank->Initialize(ground, crank, ChCoordsys<>(ChVector<>(0, 0, 0), QUNIT));
  engine_ground_crank->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
  if (ChSharedPtr<ChFunction_Const> mfun = engine_ground_crank->Get_spe_funct().DynamicCastTo<ChFunction_Const>())
    mfun->Set_yconst(CH_C_PI);
  system.AddLink(engine_ground_crank);
#else
  // Revolute between ground and crank.
  ChSharedPtr<ChLinkLockRevolute>  rev_ground_crank(new ChLinkLockRevolute);
  rev_ground_crank->Initialize(ground, crank, ChCoordsys<>(ChVector<>(0,0,0), QUNIT));
  system.AddLink(rev_ground_crank);
#endif
  
  // 4. Create and setup the Irrlicht App
  ChIrrApp irrapp(&system, L"testing idler joint", core::dimension2d<u32>(1000,800),false,true);
  
  irrapp.AddTypicalLogo();
  irrapp.AddTypicalSky();
  irrapp.AddTypicalLights();
  irrapp.AddTypicalCamera(core::vector3df(0,4,-3));
  
  bool do_shadows = true; // shadow map is experimental
  irr::scene::ILightSceneNode* mlight = 0;

  if (do_shadows)
  {
    mlight = irrapp.AddLightWithShadow(
      irr::core::vector3df(10.f, 60.f, 30.f),
      irr::core::vector3df(0.f, 0.f, 0.f),
      150, 60, 80, 15, 512, irr::video::SColorf(1, 1, 1), false, false);
  }
  else
  {
    irrapp.AddTypicalLights(
      irr::core::vector3df(30.f, 100.f, -30.f),
      irr::core::vector3df(30.f, 100.f, 500.f),
      250, 130);
  }
  
  irrapp.SetTimestep(time_step);
  // Complete asset specification: convert all assets to Irrlicht
  irrapp.AssetBindAll();
  irrapp.AssetUpdateAll();
  if (do_shadows)
  {
    irrapp.AddShadowAll();
  }
  // 5. Perform the simulation.

  // Calculate the required number of simulation steps and the number of steps
  // between output frames (based on the requested FPS).
  // int num_steps = (int) std::ceil(time_end / time_step);
  int out_steps = (int) std::ceil((1 / time_step) / out_fps);
   // Number of simulation steps between two 3D view render frames
  // Time interval between two render frames
  double render_step_size = 1.0 / render_fps;
  int render_steps = (int)std::ceil(render_step_size / time_step);

  // Initialize simulation frame counter and simulation time
  int step_number = 0;
  double time = 0;
  ChRealtimeStepTimer realtime_timer;
  int out_frame = 0;
  char filename[100];

  //for (int i = 0; i < num_steps; i++) {
  while(irrapp.GetDevice()->run()) {
    // current simulation time
    time = system.GetChTime();
	
    // Render scene
    if (step_number % render_steps == 0) {
      irrapp.GetVideoDriver()->beginScene(true, true, irr::video::SColor(255, 140, 161, 192));
	  irrapp.DrawAll();
      irrapp.GetVideoDriver()->endScene();
    }
	// regardless of rendering the scene, take atime step
	irrapp.DoStep();
		
    // If this is an output frame, append body positions and orientations to the
    // output file and generate a rendering data file for this frame.
    // if (i % out_steps == 0) {
	if (step_number % out_steps == 0) {
      OutputData(&system, ofile, time);

      sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), out_frame+1);
      utils::WriteShapesPovray(&system, filename);

      out_frame++;
    }

    // when not using irrlicht:
    // system.DoStepDynamics(time_step);

	// increment the step counter
    step_number++;
  }

  return 0;
}


