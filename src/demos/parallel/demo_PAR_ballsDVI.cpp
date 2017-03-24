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
// ChronoParallel test program using DVI method for frictional contact.
//
// The model simulated here consists of a number of spherical objects falling
// in a fixed container.
//
// The global reference frame has Z up.
//
// If available, OpenGL is used for run-time rendering. Otherwise, the
// simulation is carried out for a pre-defined duration and output files are
// generated for post-processing with POV-Ray.
// =============================================================================

#include <cstdio>
#include <vector>
#include <cmath>

#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace chrono::collision;

const char* out_folder = "../BALLS_DVI/POVRAY";

// Tilt angle (about global Y axis) of the container.
double tilt_angle = 1 * CH_C_PI / 20;

// Number of balls: (2 * count_X + 1) * (2 * count_Y + 1)
int count_X = 2;
int count_Y = 2;

// -----------------------------------------------------------------------------
// Generate postprocessing output with current system state.
// -----------------------------------------------------------------------------
void OutputData(ChSystemParallel* sys, int out_frame, double time) {
  char filename[100];
  sprintf(filename, "%s/data_%03d.dat", out_folder, out_frame);
  utils::WriteShapesPovray(sys, filename);
  std::cout << "time = " << time << std::flush << std::endl;
}

// -----------------------------------------------------------------------------
// Create a bin consisting of five boxes attached to the ground.
// -----------------------------------------------------------------------------
void AddContainer(ChSystemParallelDVI* sys) {
  // IDs for the two bodies
  int binId = -200;

  // Create a common material
  auto mat = std::make_shared<ChMaterialSurface>();
  mat->SetFriction(0.4f);

  // Create the containing bin (4 x 4 x 1)
  auto bin = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>());
  bin->SetMaterialSurface(mat);
  bin->SetIdentifier(binId);
  bin->SetMass(1);
  bin->SetPos(ChVector<>(0, 0, 0));
  bin->SetRot(Q_from_AngY(tilt_angle));
  bin->SetCollide(true);
  bin->SetBodyFixed(true);

  ChVector<> hdim(2, 2, 0.5);
  double hthick = 0.1;

  bin->GetCollisionModel()->ClearModel();
  utils::AddBoxGeometry(bin.get(), ChVector<>(hdim.x(), hdim.y(), hthick), ChVector<>(0, 0, -hthick));
  utils::AddBoxGeometry(bin.get(), ChVector<>(hthick, hdim.y(), hdim.z()), ChVector<>(-hdim.x() - hthick, 0, hdim.z()));
  utils::AddBoxGeometry(bin.get(), ChVector<>(hthick, hdim.y(), hdim.z()), ChVector<>(hdim.x() + hthick, 0, hdim.z()));
  utils::AddBoxGeometry(bin.get(), ChVector<>(hdim.x(), hthick, hdim.z()), ChVector<>(0, -hdim.y() - hthick, hdim.z()));
  utils::AddBoxGeometry(bin.get(), ChVector<>(hdim.x(), hthick, hdim.z()), ChVector<>(0, hdim.y() + hthick, hdim.z()));
  bin->GetCollisionModel()->BuildModel();

  sys->AddBody(bin);
}

// -----------------------------------------------------------------------------
// Create the falling spherical objects in a uniform rectangular grid.
// -----------------------------------------------------------------------------
void AddFallingBalls(ChSystemParallel* sys) {
  // Common material
  auto ballMat = std::make_shared<ChMaterialSurface>();
  ballMat->SetFriction(0.4f);

  // Create the falling balls
  int ballId = 0;
  double mass = 1;
  double radius = 0.15;
  ChVector<> inertia = (2.0 / 5.0) * mass * radius * radius * ChVector<>(1, 1, 1);

  for (int ix = -count_X; ix <= count_X; ix++) {
    for (int iy = -count_Y; iy <= count_Y; iy++) {
      ChVector<> pos(0.4 * ix, 0.4 * iy, 1);

      auto ball = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>());
      ball->SetMaterialSurface(ballMat);

      ball->SetIdentifier(ballId++);
      ball->SetMass(mass);
      ball->SetInertiaXX(inertia);
      ball->SetPos(pos);
      ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
      ball->SetBodyFixed(false);
      ball->SetCollide(true);

      ball->GetCollisionModel()->ClearModel();
      utils::AddSphereGeometry(ball.get(), radius);
      ball->GetCollisionModel()->BuildModel();

      sys->AddBody(ball);
    }
  }
}

// -----------------------------------------------------------------------------
// Create the system, specify simulation parameters, and run simulation loop.
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
  int threads = 8;

  // Simulation parameters
  // ---------------------

  double gravity = 9.81;
  double time_step = 1e-3;
  double time_end = 2;

  double out_fps = 50;

  uint max_iteration = 300;
  real tolerance = 1e-3;

  // Create system
  // -------------

  ChSystemParallelDVI msystem;

  // Set number of threads.
  int max_threads = CHOMPfunctions::GetNumProcs();
  if (threads > max_threads)
    threads = max_threads;
  msystem.SetParallelThreadNumber(threads);
  CHOMPfunctions::SetNumThreads(threads);

  // Set gravitational acceleration
  msystem.Set_G_acc(ChVector<>(0, 0, -gravity));

  // Set solver parameters
  msystem.GetSettings()->solver.solver_mode = SolverMode::SLIDING;
  msystem.GetSettings()->solver.max_iteration_normal = max_iteration / 3;
  msystem.GetSettings()->solver.max_iteration_sliding = max_iteration / 3;
  msystem.GetSettings()->solver.max_iteration_spinning = 0;
  msystem.GetSettings()->solver.max_iteration_bilateral = max_iteration / 3;
  msystem.GetSettings()->solver.tolerance = tolerance;
  msystem.GetSettings()->solver.alpha = 0;
  msystem.GetSettings()->solver.contact_recovery_speed = 10000;
  msystem.ChangeSolverType(SolverType::APGDREF);
  msystem.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;

  msystem.GetSettings()->collision.collision_envelope = 0.01;
  msystem.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

  // Create the fixed and moving bodies
  // ----------------------------------

  AddContainer(&msystem);
  AddFallingBalls(&msystem);

// Perform the simulation
// ----------------------

#ifdef CHRONO_OPENGL
  opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
  gl_window.Initialize(1280, 720, "ballsDVI", &msystem);
  gl_window.SetCamera(ChVector<>(0, -10, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));

  // Uncomment the following two lines for the OpenGL manager to automatically
  // run the simulation in an infinite loop.
  // gl_window.StartDrawLoop(time_step);
  // return 0;

  while (true) {
    if (gl_window.Active()) {
      gl_window.DoStepDynamics(time_step);
      gl_window.Render();
      ////if (gl_window.Running()) {
      ////  msystem.CalculateContactForces();
      ////  real3 frc = msystem.GetBodyContactForce(0);
      ////  std::cout << frc.x << "  " << frc.y << "  " << frc.z << std::endl;
      ////}
    } else {
      break;
    }
  }
#else
  // Run simulation for specified time
  int num_steps = (int)std::ceil(time_end / time_step);
  int out_steps = (int)std::ceil((1 / time_step) / out_fps);
  int out_frame = 0;
  double time = 0;

  for (int i = 0; i < num_steps; i++) {
    if (i % out_steps == 0) {
      OutputData(&msystem, out_frame, time);
      out_frame++;
    }
    msystem.DoStepDynamics(time_step);
    time += time_step;
  }
#endif

  return 0;
}
