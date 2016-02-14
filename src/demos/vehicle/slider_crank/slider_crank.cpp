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
// Chrono demo for slider-crank
//
// The model simulated here is a 2-body slider-crank modeled with a crank body
// and a rod body. The crank and rod are connected through a revolute joint and
// the rod tip is constrained to move along the global X axis. The crank can be
// connected to ground either through a revolute joint or through a constant
// speed motor.
//
// The global reference frame has Z up.
// =============================================================================

#include <stdio.h>
#include <cmath>

#include "core/ChFileutils.h"
#include "core/ChStream.h"

#include "physics/ChSystem.h"

#include "utils/ChUtilsCreators.h"
#include "utils/ChUtilsInputOutput.h"

using namespace chrono;

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Magnitude of the gravitational acceleration
const double gravity = 9.81;

// Simulation time, integration time-step, and output FPS
const double time_end = 4.0;
const double time_step = 0.01;
const double out_fps = 50;

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
      ChSharedPtr<ChBody> body = system->Get_bodylist()->at(i);

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
// -----------------------------------------------------------------------------
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
  //    global reference frame having Z up.
  ChSystem system;
  system.Set_G_acc(ChVector<>(0, 0, -gravity));

  // 2. Create the rigid bodies of the slider-crank mechanical system.
  //    Notes:
  //    - For each body, create the geometry relative to the body-fixed
  //      reference frame (which is centered at the body's COM).
  //    - We create both contact shapes and visualization assets, but contact
  //      is explicitly disabled for each body individually.
  //    - capsule and cylinder geometries (both for contact and visualization)
  //      are by default aligned with the Y axis.
  //    - We use negative identifiers for all fixed bodies and positive
  //      identifiers otherwise. This allows identifying a fixed body during
  //      rendering and optionally hiding it.

  // Define two quaternions representing:
  // - a rotation of -90 degrees around z (y2x)
  // - a rotation of -90 degrees around x (z2y)
  ChQuaternion<> y2x;
  ChQuaternion<> z2y;
  y2x.Q_from_AngAxis(-CH_C_PI/2, ChVector<>(0, 0, 1));
  z2y.Q_from_AngAxis(-CH_C_PI/2, ChVector<>(1, 0, 0));

  // Ground
  ChSharedPtr<ChBody>  ground(new ChBody);
  system.AddBody(ground);
  ground->SetIdentifier(-1);
  ground->SetBodyFixed(true);
  ground->SetCollide(false);
  ground->GetCollisionModel()->ClearModel();
  utils::AddCylinderGeometry(ground.get_ptr(), 0.15, 0.1);
  ground->GetCollisionModel()->BuildModel();

  // Crank
  ChSharedPtr<ChBody>  crank(new ChBody);
  system.AddBody(crank);
  crank->SetIdentifier(1);
  crank->SetPos(ChVector<>(1,0,0));
  crank->SetCollide(false);
  crank->GetCollisionModel()->ClearModel();
  utils::AddCapsuleGeometry(crank.get_ptr(), 0.1, 1, ChVector<>(0, 0, 0), y2x);
  utils::AddCylinderGeometry(crank.get_ptr(), 0.15, 0.1, ChVector<>(1, 0, 0));
  crank->GetCollisionModel()->BuildModel();

  // Rod
  ChSharedPtr<ChBody>  rod(new ChBody);
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
  //      specified coordinate frame.  Here, we apply the 'z2y' rotation to
  //      align it with the Y axis of the global reference frame.

  // Revolute joint between crank and rod.
  ChSharedPtr<ChLinkLockRevolute>  rev_crank_rod(new ChLinkLockRevolute);
  rev_crank_rod->Initialize(crank, rod, ChCoordsys<>(ChVector<>(2, 0, 0), z2y));
  system.AddLink(rev_crank_rod);

  // Slider (point on line) joint between rod and ground.
  ChSharedPtr<ChLinkLockPointLine> slider_rod_ground(new ChLinkLockPointLine);
  slider_rod_ground->Initialize(rod, ground, ChCoordsys<>(ChVector<>(6, 0, 0)));
  system.AddLink(slider_rod_ground);

#ifdef DRIVEN
  // Engine between ground and crank (also acts as a revolute joint).
  // Enforce constant angular speed of PI rad/s
  ChSharedPtr<ChLinkEngine> engine_ground_crank(new ChLinkEngine);
  engine_ground_crank->Initialize(ground, crank, ChCoordsys<>(ChVector<>(0, 0, 0), z2y));
  engine_ground_crank->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
  if (ChSharedPtr<ChFunction_Const> mfun = engine_ground_crank->Get_spe_funct().DynamicCastTo<ChFunction_Const>())
    mfun->Set_yconst(CH_C_PI);
  system.AddLink(engine_ground_crank);
#else
  // Revolute between ground and crank.
  ChSharedPtr<ChLinkLockRevolute>  rev_ground_crank(new ChLinkLockRevolute);
  rev_ground_crank->Initialize(ground, crank, ChCoordsys<>(ChVector<>(0,0,0), z2y));
  system.AddLink(rev_ground_crank);
#endif

  // 4. Perform the simulation.

  // Calculate the required number of simulation steps and the number of steps
  // between output frames (based on the requested FPS).
  int num_steps = (int) std::ceil(time_end / time_step);
  int out_steps = (int) std::ceil((1 / time_step) / out_fps);

  double time = 0;
  int out_frame = 0;
  char filename[100];

  for (int i = 0; i < num_steps; i++) {

    // If this is an output frame, append body positions and orientations to the
    // output file and generate a rendering data file for this frame.
    if (i % out_steps == 0) {
      OutputData(&system, ofile, time);

      sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), out_frame+1);
      utils::WriteShapesPovray(&system, filename);

      out_frame++;
    }

    // Advance system state for one step.
    system.DoStepDynamics(time_step);

    time += time_step;
  }

  return 0;
}


