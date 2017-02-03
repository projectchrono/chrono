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
// Test for bilateral joint constraints in a DVI system.
//
// The mechanism consists of three bodies (ground, sled, and pendulum) with a
// prismatic joint between ground and sled and a revolute joint between sled and
// pendulum.
// The system is simulated with different combinations of solver settings
// (type of solver, solver mode, maximum number of iterations).  Constraint
// violations are monitored and verified.
//
// =============================================================================


#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace chrono::collision;

struct Options {
  SolverMode mode;
  SolverType type;

  uint max_iter_bilateral;
  uint max_iter_normal;
  uint max_iter_sliding;
};

// -----------------------------------------------------------------------------
// Create and simulate the mechanism
// -----------------------------------------------------------------------------
bool TestMechanism(Options opts, bool animate) {
  std::cout << "Solver type:  " << as_integer(opts.type) << "  mode:  " << as_integer(opts.mode) << std::endl
            << "     max_iter_bilateral: " << opts.max_iter_bilateral
            << "     max_iter_normal: " << opts.max_iter_normal << "     max_iter_sliding: " << opts.max_iter_sliding
            << std::endl;

  // Additional solver settings
  //---------------------------

  int threads = 1;
  bool thread_tuning = false;

  double time_end = 2;
  double time_step = 1e-3;

  double tolerance = 1e-5;

  bool clamp_bilaterals = false;
  double bilateral_clamp_speed = 1000;

  // Maximum allowable constraint violation
  double max_cnstr_violation = 1e-4;

  // Problem parameters
  // ------------------

  double init_vel = 2;

  // Create the mechanical system
  // ----------------------------

  ChSystemParallelDVI* system = new ChSystemParallelDVI();

  system->Set_G_acc(ChVector<>(0, 0, -9.81));

  // Set number of threads.
  CHOMPfunctions::SetNumThreads(threads);
  system->GetSettings()->max_threads = threads;
  system->GetSettings()->perform_thread_tuning = thread_tuning;

  // Edit system settings
  system->GetSettings()->solver.tolerance = tolerance;
  system->GetSettings()->solver.max_iteration_bilateral = opts.max_iter_bilateral;
  system->GetSettings()->solver.clamp_bilaterals = clamp_bilaterals;
  system->GetSettings()->solver.bilateral_clamp_speed = bilateral_clamp_speed;

  system->GetSettings()->solver.solver_mode = opts.mode;
  system->GetSettings()->solver.max_iteration_normal = opts.max_iter_normal;
  system->GetSettings()->solver.max_iteration_sliding = opts.max_iter_sliding;
  system->GetSettings()->solver.max_iteration_spinning = 0;
  system->GetSettings()->solver.alpha = 0;
  system->GetSettings()->solver.contact_recovery_speed = 1e4;
  system->ChangeSolverType(opts.type);

  // Create the bodies
  // -----------------

  // Create the ground body
  auto ground = std::make_shared<ChBody>(std::make_shared<collision::ChCollisionModelParallel>());
  ground->SetIdentifier(-1);
  ground->SetBodyFixed(true);
  ground->SetCollide(false);
  system->AddBody(ground);

  // Create the sled body
  auto sled = std::make_shared<ChBody>(std::make_shared<collision::ChCollisionModelParallel>());
  sled->SetIdentifier(1);
  sled->SetMass(550);
  sled->SetInertiaXX(ChVector<>(100, 100, 100));
  sled->SetPos(ChVector<>(0, 0, 0));
  sled->SetPos_dt(ChVector<>(init_vel, 0, 0));
  sled->SetBodyFixed(false);
  sled->SetCollide(false);

  auto box_sled = std::make_shared<ChBoxShape>();
  box_sled->GetBoxGeometry().Size = ChVector<>(1, 0.25, 0.25);
  box_sled->Pos = ChVector<>(0, 0, 0);
  box_sled->Rot = ChQuaternion<>(1, 0, 0, 0);
  sled->AddAsset(box_sled);

  system->AddBody(sled);

  // Create the wheel body
  auto wheel = std::make_shared<ChBody>(std::make_shared<collision::ChCollisionModelParallel>());
  wheel->SetIdentifier(2);
  wheel->SetMass(350);
  wheel->SetInertiaXX(ChVector<>(50, 138, 138));
  wheel->SetPos(ChVector<>(2, 0, 0));
  wheel->SetRot(ChQuaternion<>(1, 0, 0, 0));
  wheel->SetPos_dt(ChVector<>(init_vel, 0, 0));
  wheel->SetBodyFixed(false);
  wheel->SetCollide(true);

  wheel->GetCollisionModel()->ClearModel();
  utils::AddCylinderGeometry(wheel.get(), 0.3, 0.1, ChVector<>(0, 0, 0), Q_from_AngZ(CH_C_PI_2));
  wheel->GetCollisionModel()->BuildModel();

  system->AddBody(wheel);

  // Create joints
  // -------------

  // Create and initialize translational joint ground - sled
  auto prismatic = std::make_shared<ChLinkLockPrismatic>();
  prismatic->Initialize(ground, sled, ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngY(CH_C_PI_2)));
  system->AddLink(prismatic);

  // Create and initialize revolute joint sled - wheel
  auto revolute = std::make_shared<ChLinkLockRevolute>();
  revolute->Initialize(wheel, sled, ChCoordsys<>(ChVector<>(1, 0, 0), Q_from_AngX(CH_C_PI_2)));
  system->AddLink(revolute);

  // Perform the simulation
  // ----------------------
  double time = 0;
  bool passed = true;

  if (animate) {
#ifdef CHRONO_OPENGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "Pressure Sinkage Test", system);
    gl_window.SetCamera(ChVector<>(0, -8, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));
    gl_window.SetRenderMode(opengl::WIREFRAME);

    while (time < time_end) {
      if (gl_window.Active()) {
        gl_window.DoStepDynamics(time_step);
        gl_window.Render();
      }
      time += time_step;
    }
#endif
  } else {
#ifndef CHRONO_OPENGL
    ChTimerParallel timer;
    timer.AddTimer("simulation_time");
    timer.Reset();
    timer.start("simulation_time");

    while (time < time_end) {
      // Advance simulation.
      system->DoStepDynamics(time_step);
      time += time_step;

      // Check constraints for prismatic joint
      ChMatrix<>* pC = prismatic->GetC();
      for (int i = 0; i < 5; i++) {
        if (std::abs(pC->GetElement(i, 0)) > max_cnstr_violation) {
          std::cout << "VIOLATION prismatic " << i << "  = " << pC->GetElement(i, 0) << std::endl;
          passed = false;
        }
      }

      // check constraints for revolute joint
      ChMatrix<>* rC = revolute->GetC();
      for (int i = 0; i < 5; i++) {
        if (std::abs(rC->GetElement(i, 0)) > max_cnstr_violation) {
          std::cout << "VIOLATION revolute " << i << "  = " << rC->GetElement(i, 0) << std::endl;
          passed = false;
        }
      }
    }

    timer.stop("simulation_time");
    std::cout << (passed ? "PASSED" : "FAILED") << "  sim. time: " << timer.GetTime("simulation_time") << std::endl
              << std::endl;
#endif
  }

  return passed;
}

// -----------------------------------------------------------------------------
// Main driver function.
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
  // No animation by default (i.e. when no program arguments)
  bool animate = (argc > 1);

  bool test_passed = true;

  // Run the problem with different combinations of solver options.
  Options opts;

  opts.type = SolverType::APGDREF;
  opts.mode = SolverMode::NORMAL;
  opts.max_iter_bilateral = 100;
  opts.max_iter_normal = 1000;
  opts.max_iter_sliding = 0;
  test_passed &= TestMechanism(opts, animate);

  opts.type = SolverType::APGDREF;
  opts.mode = SolverMode::NORMAL;
  opts.max_iter_bilateral = 0;
  opts.max_iter_normal = 1000;
  opts.max_iter_sliding = 0;
  test_passed &= TestMechanism(opts, animate);

  opts.type = SolverType::APGDREF;
  opts.mode = SolverMode::SLIDING;
  opts.max_iter_bilateral = 100;
  opts.max_iter_normal = 0;
  opts.max_iter_sliding = 1000;
  test_passed &= TestMechanism(opts, animate);

  opts.type = SolverType::APGDREF;
  opts.mode = SolverMode::SLIDING;
  opts.max_iter_bilateral = 0;
  opts.max_iter_normal = 0;
  opts.max_iter_sliding = 1000;
  test_passed &= TestMechanism(opts, animate);

  /*
  opts.type = SolverType::APGD;
  opts.mode = SolverMode::NORMAL;
  opts.max_iter_bilateral = 100;
  opts.max_iter_normal = 1000;
  opts.max_iter_sliding = 0;
  test_passed &= TestMechanism(opts, animate);

  opts.type = SolverType::APGD;
  opts.mode = SolverMode::NORMAL;
  opts.max_iter_bilateral = 0;
  opts.max_iter_normal = 1000;
  opts.max_iter_sliding = 0;
  test_passed &= TestMechanism(opts, animate);

  opts.type = SolverType::APGD;
  opts.mode = SolverMode::SLIDING;
  opts.max_iter_bilateral = 100;
  opts.max_iter_normal = 0;
  opts.max_iter_sliding = 1000;
  test_passed &= TestMechanism(opts, animate);

  opts.type = SolverType::APGD;
  opts.mode = SolverMode::SLIDING;
  opts.max_iter_bilateral = 0;
  opts.max_iter_normal = 0;
  opts.max_iter_sliding = 1000;
  test_passed &= TestMechanism(opts, animate);
  */

  // Return 0 if all tests passed and 1 otherwise
  return !test_passed;
}
