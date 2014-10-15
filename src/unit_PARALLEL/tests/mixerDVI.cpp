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

#include "chrono_parallel/ChSystemParallel.h"
#include "chrono_parallel/ChLcpSystemDescriptorParallel.h"

#include "chrono_utils/ChUtilsCreators.h"
#include "chrono_utils/ChUtilsInputOutput.h"

#ifdef CHRONO_PARALLEL_HAS_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace chrono::collision;

const char* out_folder = "../MIXER_DVI/POVRAY";

// -----------------------------------------------------------------------------
// Generate postprocessing output with current system state.
// -----------------------------------------------------------------------------
void OutputData(ChSystemParallel* sys,
                int out_frame,
                double time) {
   char filename[100];
   sprintf(filename, "%s/data_%03d.dat", out_folder, out_frame);
   utils::WriteShapesPovray(sys, filename);
   std::cout << "time = " << time << std::flush << std::endl;
}

// -----------------------------------------------------------------------------
// Create a bin consisting of five boxes attached to the ground and a mixer
// blade attached through a revolute joint to ground. The mixer is constrained
// to rotate at constant angular velocity.
// -----------------------------------------------------------------------------
void AddContainer(ChSystemParallelDVI* sys) {
   // IDs for the two bodies
   int binId = -200;
   int mixerId = -201;

   // Create a common material
   ChSharedPtr<ChMaterialSurface> mat(new ChMaterialSurface);
   mat->SetFriction(0.4f);

   // Create the containing bin (2 x 2 x 1)
   ChSharedBodyPtr bin(new ChBody(new ChCollisionModelParallel));
   bin->SetMaterialSurface(mat);
   bin->SetIdentifier(binId);
   bin->SetMass(1);
   bin->SetPos(ChVector<>(0, 0, 0));
   bin->SetRot(ChQuaternion<>(1, 0, 0, 0));
   bin->SetCollide(true);
   bin->SetBodyFixed(true);

   ChVector<> hdim(1, 1, 0.5);
   double hthick = 0.1;

   bin->GetCollisionModel()->ClearModel();
   utils::AddBoxGeometry(bin.get_ptr(), ChVector<>(hdim.x, hdim.y, hthick), ChVector<>(0, 0, -hthick));
   utils::AddBoxGeometry(bin.get_ptr(), ChVector<>(hthick, hdim.y, hdim.z), ChVector<>(-hdim.x - hthick, 0, hdim.z));
   utils::AddBoxGeometry(bin.get_ptr(), ChVector<>(hthick, hdim.y, hdim.z), ChVector<>(hdim.x + hthick, 0, hdim.z));
   utils::AddBoxGeometry(bin.get_ptr(), ChVector<>(hdim.x, hthick, hdim.z), ChVector<>(0, -hdim.y - hthick, hdim.z));
   utils::AddBoxGeometry(bin.get_ptr(), ChVector<>(hdim.x, hthick, hdim.z), ChVector<>(0, hdim.y + hthick, hdim.z));
   bin->GetCollisionModel()->BuildModel();

   sys->AddBody(bin);

   // The rotating mixer body (1.6 x 0.2 x 0.4)
   ChSharedBodyPtr mixer(new ChBody(new ChCollisionModelParallel));
   mixer->SetMaterialSurface(mat);
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

   motor->Initialize(mixer, bin, ChCoordsys<>(ChVector<>(0, 0, 0), ChQuaternion<>(1, 0, 0, 0)));

   motor->Set_eng_mode(ChLinkEngine::ENG_MODE_ROTATION);
   motor->Set_rot_funct(ChSharedPtr<ChFunction>(new ChFunction_Ramp(0, CH_C_PI / 2)));

   sys->AddLink(motor);
}

// -----------------------------------------------------------------------------
// Create the falling spherical objects in a unfiorm rectangular grid.
// -----------------------------------------------------------------------------
void AddFallingBalls(ChSystemParallel* sys) {
   // Common material
   ChSharedPtr<ChMaterialSurface> ballMat(new ChMaterialSurface);
   ballMat->SetFriction(0.4f);

   // Create the falling balls
   int ballId = 0;
   double mass = 1;
   double radius = 0.15;
   ChVector<> inertia = (2.0 / 5.0) * mass * radius * radius * ChVector<>(1, 1, 1);

   for (int ix = -2; ix < 3; ix++) {
      for (int iy = -2; iy < 3; iy++) {
         ChVector<> pos(0.4 * ix, 0.4 * iy, 1);

         ChSharedBodyPtr ball(new ChBody(new ChCollisionModelParallel));
         ball->SetMaterialSurface(ballMat);

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
int main(int argc,
         char* argv[]) {
   int threads = 8;

   // Simulation parameters
   // ---------------------

   double gravity = 9.81;
   double time_step = 1e-3;
   double time_end = 1;

   double out_fps = 50;

   uint max_iteration = 30;
   real tolerance = 1e-8;

   // Create system
   // -------------

   ChSystemParallelDVI msystem;

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
   msystem.SetMaxPenetrationRecoverySpeed(1e9);

   msystem.GetSettings()->solver.solver_mode = SLIDING;
   msystem.GetSettings()->solver.max_iteration_normal = max_iteration / 3;
   msystem.GetSettings()->solver.max_iteration_sliding = max_iteration / 3;
   msystem.GetSettings()->solver.max_iteration_spinning = 0;
   msystem.GetSettings()->solver.max_iteration_bilateral = max_iteration / 3;
   msystem.GetSettings()->solver.tolerance = 1e-3;
   msystem.GetSettings()->solver.alpha = 0;
   msystem.GetSettings()->solver.contact_recovery_speed = 10000;
   msystem.ChangeSolverType(APGDRS);
   msystem.GetSettings()->collision.narrowphase_algorithm = NARROWPHASE_HYBRID_MPR;

   msystem.GetSettings()->collision.collision_envelope = 0.01;
   msystem.GetSettings()->collision.bins_per_axis = I3(10, 10, 10);
   msystem.GetSettings()->collision.min_body_per_bin = 50;
   msystem.GetSettings()->collision.max_body_per_bin = 100;

   // Create the fixed and moving bodies
   // ----------------------------------

   AddContainer(&msystem);
   AddFallingBalls(&msystem);

   // Perform the simulation
   // ----------------------

#ifdef CHRONO_PARALLEL_HAS_OPENGL
   opengl::ChOpenGLWindow &gl_window = opengl::ChOpenGLWindow::getInstance();
   gl_window.Initialize(1280, 720, "mixerDVI", &msystem);
   gl_window.SetCamera(ChVector<>(0, -10, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));

   // Uncomment the following two lines for the OpenGL manager to automatically
   // run the simulation in an infinite loop.
   //gl_window.StartDrawLoop();
   //return 0;
#endif

   // Run simulation for specified time
   int num_steps = std::ceil(time_end / time_step);
   int out_steps = std::ceil((1 / time_step) / out_fps);
   double time = 0;
   int out_frame = 0;

   for (int i = 0; i < num_steps; i++) {
      if (i % out_steps == 0) {
         OutputData(&msystem, out_frame, time);
         out_frame++;
      }

#ifdef CHRONO_PARALLEL_HAS_OPENGL
      if (gl_window.Active()) {
         gl_window.DoStepDynamics(time_step);
         gl_window.Render();
      }
#else
      msystem.DoStepDynamics(time_step);
#endif

      time += time_step;
   }

   return 0;
}

