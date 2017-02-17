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
// Test for ChBodyAuxRef (body defined with respect to a non-centroidal frame),
// using two identical pendulums, modeled as a ChBody and as a ChBodyAuxRef,
// respectively.
//
// The two pendulums have length 2 and are pinned to ground through revolute
// joints with the rotation axis along the global Y axis. The absolute locations
// of the revolute joints are at (0, 1, 0) and (0, -1, 0), respectively.
//
// The ChBody pendulum is defined with respect to a centroidal frame (as assumed
// by ChBody) located at the geometric center of the pendulum, with the X axis
// along the length of the pendulum.
// The ChBodyAuxRef pendulum is defined with respect to a local frame, parallel
// to its centroidal frame but located at the pin location.  In other words, the
// center of mass of the pendulum is at (1, 0, 0), relative to the body frame.
//
// The two pendulums move under the action of gravity, acting along the negative
// global Z direction.
//
// =============================================================================

#include "chrono_parallel/physics/ChSystemParallel.h"

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace chrono::collision;

// Function declarations.
bool VerifySolution(double time, std::shared_ptr<ChBody> pend_1, std::shared_ptr<ChBodyAuxRef> pend_2);

// -----------------------------------------------------------------------------
// Main driver function for running the simulation and validating the results.
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
  // No animation by default (i.e. when no program arguments)
  bool animate = (argc > 1);

  // Settings
  //---------

  int threads = 1;
  bool thread_tuning = false;

  double time_end = 5;
  double time_step = 1e-3;

  double tolerance = 1e-5;

  int max_iteration_bilateral = 100;
  int max_iteration_normal = 0;
  int max_iteration_sliding = 100;
  int max_iteration_spinning = 0;

  bool clamp_bilaterals = false;
  double bilateral_clamp_speed = 1000;

  double contact_recovery_speed = 1;

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
  system->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
  system->GetSettings()->solver.clamp_bilaterals = clamp_bilaterals;
  system->GetSettings()->solver.bilateral_clamp_speed = bilateral_clamp_speed;

  system->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
  system->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
  system->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
  system->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
  system->ChangeSolverType(SolverType::APGD);

  // Define a couple of rotations for later use
  ChQuaternion<> y2x;
  ChQuaternion<> z2y;
  y2x.Q_from_AngZ(-CH_C_PI / 2);
  z2y.Q_from_AngX(-CH_C_PI / 2);

  // Create the ground body
  // ----------------------

  auto ground = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>());
  ground->SetBodyFixed(true);
  system->AddBody(ground);

  // Attach a visualization asset representing the Y axis.
  auto box = std::make_shared<ChBoxShape>();
  box->GetBoxGeometry().Size = ChVector<>(0.02, 3, 0.02);
  ground->AddAsset(box);

  // Create a pendulum modeled using ChBody
  // --------------------------------------

  auto pend_1 = std::make_shared<ChBody>(std::make_shared<ChCollisionModelParallel>());
  system->AddBody(pend_1);
  pend_1->SetIdentifier(1);
  pend_1->SetBodyFixed(false);
  pend_1->SetCollide(false);
  pend_1->SetMass(1);
  pend_1->SetInertiaXX(ChVector<>(0.2, 1, 1));

  // Attach a visualization asset. Note that the cylinder is defined with
  // respect to the centroidal reference frame (which is the body reference
  // frame for a ChBody).
  auto cyl_1 = std::make_shared<ChCylinderShape>();
  cyl_1->GetCylinderGeometry().p1 = ChVector<>(0, -1, 0);
  cyl_1->GetCylinderGeometry().p2 = ChVector<>(0, 1, 0);
  cyl_1->GetCylinderGeometry().rad = 0.2;
  cyl_1->Pos = ChVector<>(0, 0, 0);
  cyl_1->Rot = y2x;
  pend_1->AddAsset(cyl_1);
  auto col_1 = std::make_shared<ChColorAsset>();
  col_1->SetColor(ChColor(0.6f, 0, 0));
  pend_1->AddAsset(col_1);

  // Specify the initial position of the pendulum (horizontal, pointing towards
  // positive X). In this case, we set the absolute position of its center of
  // mass.
  pend_1->SetPos(ChVector<>(1, 1, 0));

  // Create a revolute joint to connect pendulum to ground. We specify the link
  // coordinate frame in the absolute frame.
  auto rev_1 = std::make_shared<ChLinkLockRevolute>();
  rev_1->Initialize(ground, pend_1, ChCoordsys<>(ChVector<>(0, 1, 0), z2y));
  system->AddLink(rev_1);

  // Create a pendulum modeled using ChBodyAuxRef
  // --------------------------------------------

  auto pend_2 = std::make_shared<ChBodyAuxRef>(std::make_shared<ChCollisionModelParallel>());
  system->Add(pend_2);
  pend_2->SetIdentifier(2);
  pend_2->SetBodyFixed(false);
  pend_2->SetCollide(false);
  pend_2->SetMass(1);
  pend_2->SetInertiaXX(ChVector<>(0.2, 1, 1));
  // NOTE: the inertia tensor must still be expressed in the centroidal frame!

  // Attach a visualization asset. Note that now the cylinder is defined with
  // respect to the body reference frame.
  auto cyl_2 = std::make_shared<ChCylinderShape>();
  cyl_2->GetCylinderGeometry().p1 = ChVector<>(0, -1, 0);
  cyl_2->GetCylinderGeometry().p2 = ChVector<>(0, 1, 0);
  cyl_2->GetCylinderGeometry().rad = 0.2;
  cyl_2->Pos = ChVector<>(1, 0, 0);
  cyl_2->Rot = y2x;
  pend_2->AddAsset(cyl_2);
  auto col_2 = std::make_shared<ChColorAsset>();
  col_2->SetColor(ChColor(0, 0, 0.6f));
  pend_2->AddAsset(col_2);

  // In this case, we must specify the centroidal frame, relative to the body
  // reference frame.
  pend_2->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(1, 0, 0), ChQuaternion<>(1, 0, 0, 0)));

  // Specify the initial position of the pendulum (horizontal, pointing towards
  // positive X).  Here, we want to specify the position of the body reference
  // frame (relative to the absolute frame). Recall that the body reference
  // frame is located at the pin.
  pend_2->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(0, -1, 0)));

  // Create a revolute joint to connect pendulum to ground. We specify the link
  // coordinate frame in the absolute frame.
  auto rev_2 = std::make_shared<ChLinkLockRevolute>();
  rev_2->Initialize(ground, pend_2, ChCoordsys<>(ChVector<>(0, -1, 0), z2y));
  system->AddLink(rev_2);

  // Perform the simulation
  // ----------------------

  bool passed = true;
  double time = 0;

  if (animate) {
#ifdef CHRONO_OPENGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, "BodyAuxRef", system);
    gl_window.SetCamera(ChVector<>(6, -6, 1), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));
    gl_window.SetRenderMode(opengl::WIREFRAME);
    gl_window.StartDrawLoop(time_step);
#else
    std::cout << "OpenGL support not available.  Cannot animate mechanism." << std::endl;
    return false;
#endif
  } else {
    while (time < time_end) {
      system->DoStepDynamics(time_step);
      time += time_step;
      if (!VerifySolution(time, pend_1, pend_2))
        passed = false;
    }
  }

  std::cout << (passed ? "PASSED" : "FAILED") << std::endl << std::endl;

  // Return 0 if all tests passed and 1 otherwise
  return !passed;
}

// -----------------------------------------------------------------------------
// Verify simulation results against analytical solution at the specified time.
// -----------------------------------------------------------------------------
bool VerifySolution(double time,                           // current time
                    std::shared_ptr<ChBody> pend_1,        // handle to ChBody pendulum
                    std::shared_ptr<ChBodyAuxRef> pend_2)  // handle to ChBodyAuxRef pendulum
{
  // Tolerances
  double pos_tol = 1e-6;
  double vel_tol = 1e-6;
  double acc_tol = 1e-5;

  double quat_tol = 1e-6;
  double avel_tol = 1e-6;
  double aacc_tol = 1e-5;

  // Position and orientation of the COGs
  ChVector<> pos_1 = pend_1->GetPos();
  ChVector<> pos_2 = pend_2->GetPos();
  ChQuaternion<> quat_1 = pend_1->GetRot();
  ChQuaternion<> quat_2 = pend_2->GetRot();

  if (std::abs(pos_1.x() - pos_2.x()) > pos_tol) {
    std::cout << "   at t = " << time << "   pos_1.x - pos_2.x = " << pos_1.x() - pos_2.x() << std::endl;
    return false;
  }
  if (std::abs(pos_1.z() - pos_2.z()) > pos_tol) {
    std::cout << "   at t = " << time << "   pos_1.z - pos_2.z = " << pos_1.z() - pos_2.z() << std::endl;
    return false;
  }

  ChQuaternion<> quat_delta = quat_1 - quat_2;
  if (quat_delta.Length() > quat_tol) {
    std::cout << "   at t = " << time << "   quat_1 - quat_2 = " << quat_delta.e0() << "  " << quat_delta.e1() << "  "
              << quat_delta.e2() << "  " << quat_delta.e3() << std::endl;
    return false;
  }

  // Linear and angular velocities of the COGs (expressed in global frame)
  ChVector<> vel_1 = pend_1->GetPos_dt();
  ChVector<> vel_2 = pend_2->GetPos_dt();
  ChVector<> avel_1 = pend_1->GetWvel_par();
  ChVector<> avel_2 = pend_2->GetWvel_par();

  ChVector<> vel_delta = vel_1 - vel_2;
  if (vel_delta.Length() > vel_tol) {
    std::cout << "   at t = " << time << "   vel_1 - vel_2 = " << vel_delta.x() << "  " << vel_delta.y() << "  "
              << vel_delta.z() << std::endl;
    return false;
  }

  ChVector<> avel_delta = avel_1 - avel_2;
  if (avel_delta.Length() > avel_tol) {
    std::cout << "   at t = " << time << "   avel_1 - avel_2 = " << avel_delta.x() << "  " << avel_delta.y() << "  "
              << avel_delta.z() << std::endl;
    return false;
  }

  // Linear and angular accelerations of the COGs (expressed in global frame)
  ChVector<> acc_1 = pend_1->GetPos_dtdt();
  ChVector<> acc_2 = pend_2->GetPos_dtdt();
  ChVector<> aacc_1 = pend_1->GetWacc_par();
  ChVector<> aacc_2 = pend_2->GetWacc_par();

  ChVector<> acc_delta = acc_1 - acc_2;
  if (acc_delta.Length() > acc_tol) {
    std::cout << "   at t = " << time << "   acc_1 - acc_2 = " << acc_delta.x() << "  " << acc_delta.y() << "  "
              << acc_delta.z() << std::endl;
    return false;
  }

  ChVector<> aacc_delta = aacc_1 - aacc_2;
  if (aacc_delta.Length() > aacc_tol) {
    std::cout << "   at t = " << time << "   aacc_1 - aacc_2 = " << aacc_delta.x() << "  " << aacc_delta.y() << "  "
              << aacc_delta.z() << std::endl;
    return false;
  }

  return true;
}
