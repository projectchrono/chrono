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
// Authors: Radu Serban, Hammad Mazhar
// =============================================================================
//
// ChronoParallel test program using penalty method for frictional contact.
//
// The model simulated here consists of a number of spherical objects falling
// onto a mixer blade attached through a revolute joint to the ground.
//
// The global reference frame has Z up.
//
// If available, OpenGL is used for run-time rendering. Otherwise, the 
// simulation is carried out for a pre-defined duration and output files are
// generated for post-processing with POV-Ray.
// =============================================================================

#include <stdio.h>
#include <vector>
#include <cmath>

#include "ChSystemParallel.h"
#include "ChLcpSystemDescriptorParallel.h"

#include "utils/ChUtilsCreators.h"
#include "utils/ChUtilsInputOutput.h"

#ifdef CHRONO_PARALLEL_HAS_OPENGL
#include "utils/opengl/ChOpenGL.h"
#endif

// Define this to save the data when using the OpenGL code
//#define SAVE_DATA

using namespace chrono;
using namespace geometry;

const char* out_folder = "../MIXER_DEM/POVRAY";
int out_steps;
int out_frame;

// -----------------------------------------------------------------------------
// Callback function invoked at every simulation frame. Generates postprocessing
// output with a frequency based on the specified FPS rate.
// -----------------------------------------------------------------------------
template<class T>
void SimFrameCallback(T* mSys, const int frame) {
  char filename[100];
  if (frame % out_steps == 0) {
    sprintf(filename, "%s/data_%03d.dat", out_folder, out_frame);
    utils::WriteShapesPovray(mSys, filename);
    out_frame++;
  }
}

// -----------------------------------------------------------------------------
// Create a bin consisting of five boxes attached to the ground and a mixer
// blade attached through a revolute joint to ground. The mixer is constrained
// to rotate at constant angular velocity.
// -----------------------------------------------------------------------------
void AddContainer(ChSystemParallelDEM* sys)
{
  // IDs for the two bodies
  int  binId = -200;
  int  mixerId = -201;

  // Create a common material
  ChSharedPtr<ChMaterialSurfaceDEM> mat(new ChMaterialSurfaceDEM);
  mat->SetYoungModulus(2e5f);
  mat->SetFriction(0.4f);
  mat->SetDissipationFactor(0.6f);

  // Create the containing bin (2 x 2 x 1)
  ChSharedBodyDEMPtr bin(new ChBodyDEM(new ChCollisionModelParallel));
  bin->SetMaterialSurfaceDEM(mat);
  bin->SetIdentifier(binId);
  bin->SetMass(1);
  bin->SetPos(ChVector<>(0, 0, 0));
  bin->SetRot(ChQuaternion<>(1, 0, 0, 0));
  bin->SetCollide(true);
  bin->SetBodyFixed(true);

  ChVector<> hdim(1, 1, 0.5);
  double     hthick = 0.1;

  bin->GetCollisionModel()->ClearModel();
  utils::AddBoxGeometry(bin.get_ptr(), ChVector<>(hdim.x, hdim.y, hthick), ChVector<>(0, 0, -hthick));
  utils::AddBoxGeometry(bin.get_ptr(), ChVector<>(hthick, hdim.y, hdim.z), ChVector<>(-hdim.x-hthick, 0, hdim.z));
  utils::AddBoxGeometry(bin.get_ptr(), ChVector<>(hthick, hdim.y, hdim.z), ChVector<>( hdim.x+hthick, 0, hdim.z));
  utils::AddBoxGeometry(bin.get_ptr(), ChVector<>(hdim.x, hthick, hdim.z), ChVector<>(0, -hdim.y-hthick, hdim.z));
  utils::AddBoxGeometry(bin.get_ptr(), ChVector<>(hdim.x, hthick, hdim.z), ChVector<>(0,  hdim.y+hthick, hdim.z));
  bin->GetCollisionModel()->BuildModel();

  sys->AddBody(bin);

  // The rotating mixer body (1.6 x 0.2 x 0.4)
  ChSharedBodyDEMPtr mixer(new ChBodyDEM(new ChCollisionModelParallel));
  mixer->SetMaterialSurfaceDEM(mat);
  mixer->SetIdentifier(mixerId);
  mixer->SetMass(10.0);
  mixer->SetInertiaXX(ChVector<>(50, 50, 50));
  mixer->SetPos(ChVector<>(0, 0, 0.205));
  mixer->SetBodyFixed(false);
  mixer->SetCollide(true);

  ChVector<> hsize(0.8, 0.1, 0.2);

  mixer->GetCollisionModel()->ClearModel();
  utils::AddBoxGeometry(mixer.get_ptr(), hsize);
  mixer->GetCollisionModel()->BuildModel();

  sys->AddBody(mixer);

  // Create an engine between the two bodies, constrained to rotate at 90 deg/s
  ChSharedPtr<ChLinkEngine> motor(new ChLinkEngine);

  ChSharedPtr<ChBody> mixer_(mixer);
  ChSharedPtr<ChBody> bin_(bin);

  motor->Initialize(mixer_, bin_, ChCoordsys<>(ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0)));
  motor->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
  if (ChFunction_Const* mfun = dynamic_cast<ChFunction_Const*>(motor->Get_spe_funct()))
    mfun->Set_yconst(CH_C_PI/2);

  sys->AddLink(motor);
}

// -----------------------------------------------------------------------------
// Create the falling spherical objects in a unfiorm rectangular grid.
// -----------------------------------------------------------------------------
void AddFallingBalls(ChSystemParallelDEM* sys)
{
  // Common material
  ChSharedPtr<ChMaterialSurfaceDEM> ballMat(new ChMaterialSurfaceDEM);
  ballMat->SetYoungModulus(2e5f);
  ballMat->SetFriction(0.4f);
  ballMat->SetDissipationFactor(0.6f);

  // Create the falling balls
  int        ballId = 0;
  double     mass = 1;
  double     radius = 0.15;
  ChVector<> inertia = (2.0/5.0)*mass*radius*radius*ChVector<>(1, 1, 1);

  for (int ix = -2; ix < 3; ix++) {
    for (int iy = -2; iy < 3; iy++) {
      ChVector<> pos(0.4 * ix, 0.4 * iy, 1);

      ChSharedBodyDEMPtr ball(new ChBodyDEM(new ChCollisionModelParallel));
      ball->SetMaterialSurfaceDEM(ballMat);

      ball->SetIdentifier(ballId++);
      ball->SetMass(mass);
      ball->SetInertiaXX(inertia);
      ball->SetPos(pos);
      ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
      ball->SetBodyFixed(false);
      ball->SetCollide(true);

      ball->GetCollisionModel()->ClearModel();
      utils::AddSphereGeometry(ball.get_ptr(), radius);
      ball->GetCollisionModel()->BuildModel();

      sys->AddBody(ball);
    }
  }
}

// -----------------------------------------------------------------------------
// Create the system, specify simulation parameters, and run simulation loop.
// -----------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  int threads = 8;

  // Simulation parameters
  // ---------------------

  double gravity = 9.81;
  double time_step = 1e-5;
  double time_end = 1;

  double out_fps = 50;

  uint max_iteration = 50;
  real tolerance = 1e-8;

  // Create system
  // -------------

  ChSystemParallelDEM msystem;

  // Set number of threads.
  int max_threads = msystem.GetParallelThreadNumber();
  if (threads > max_threads)
    threads = max_threads;
  msystem.SetParallelThreadNumber(threads);
  omp_set_num_threads(threads);

  // Set gravitational acceleration
  msystem.Set_G_acc(ChVector<>(0, 0, -gravity));

  // Set solver parameters
  msystem.SetMaxiter(max_iteration);
  msystem.SetIterLCPmaxItersSpeed(max_iteration);
  msystem.SetTol(1e-3);
  msystem.SetTolSpeeds(1e-3);
  msystem.SetStep(time_step);

  ((ChCollisionSystemParallel*) msystem.GetCollisionSystem())->setBinsPerAxis(I3(10, 10, 10));
  ((ChCollisionSystemParallel*) msystem.GetCollisionSystem())->setBodyPerBin(100, 50);

  ((ChCollisionSystemParallel*) msystem.GetCollisionSystem())->ChangeNarrowphase(new ChCNarrowphaseR);

  // Set tolerance and maximum number of iterations for bilateral constraint solver
  ((ChLcpSolverParallelDEM*) msystem.GetLcpSolverSpeed())->SetMaxIteration(max_iteration);
  ((ChLcpSolverParallelDEM*) msystem.GetLcpSolverSpeed())->SetTolerance(tolerance);

  // Create the fixed and moving bodies
  // ----------------------------------

  AddContainer(&msystem);
  AddFallingBalls(&msystem);

  // Perform the simulation
  // ----------------------

  out_steps = std::ceil((1 / time_step) / out_fps);
  out_frame = 0;

#ifdef CHRONO_PARALLEL_HAS_OPENGL
  // The OpenGL manager will automatically run the simulation
  utils::ChOpenGLManager * window_manager = new  utils::ChOpenGLManager();
  utils::ChOpenGL openGLView(window_manager, &msystem, 800, 600, 0, 0, "mixerDEM");
  openGLView.render_camera->camera_position = glm::vec3(0, 5, 10);
  openGLView.render_camera->camera_look_at = glm::vec3(0, 0, 0);
  openGLView.render_camera->camera_scale = 1;
#ifdef SAVE_DATA
  openGLView.SetCustomCallback(SimFrameCallback);
#endif
  openGLView.StartSpinning(window_manager);
  window_manager->CallGlutMainLoop();
#else
  // Run simulation for specified time
  int    num_steps = std::ceil(time_end / time_step);
  double time = 0;

  for (int i = 0; i < num_steps; i++) {
    SimFrameCallback(&msystem, i);
    msystem.DoStepDynamics(time_step);
    time += time_step;
  }
#endif

  return 0;
}

