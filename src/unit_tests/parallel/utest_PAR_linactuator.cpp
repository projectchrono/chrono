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
// Test for linear actuator
//
// =============================================================================

#include <ostream>
#include <fstream>


#include "chrono/ChConfig.h"
#include "chrono/core/ChFileutils.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_parallel/physics/ChSystemParallel.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace chrono::collision;

// -----------------------------------------------------------------------------
// Problem definition
// -----------------------------------------------------------------------------
double mass = 1.0;
ChVector<> inertiaXX(1, 1, 1);
ChVector<> gravity(0, 0, -9.80665);

// -----------------------------------------------------------------------------
// Verify simulation results against analytical solution at the specified time.
// -----------------------------------------------------------------------------
bool VerifySolution(double time,                                     // current time
                    const ChQuaternion<>& rot,                       // translation along Z axis
                    double speed,                                    // imposed translation speed
                    std::shared_ptr<ChBody> plate,                   // handle to plate body
                    std::shared_ptr<ChLinkLockPrismatic> prismatic,  // handle to prismatic joint
                    std::shared_ptr<ChLinkLinActuator> actuator)     // handle to linear actuator
{
    // Tolerances
    double pos_tol = 1e-6;
    double vel_tol = 1e-6;
    double acc_tol = 1e-5;

    double quat_tol = 1e-6;
    double avel_tol = 1e-6;
    double aacc_tol = 1e-5;

    double rforce_tol = 1e-5;
    double rtorque_tol = 5e-3;

    double cnstr_tol = 1e-10;

    // Unit vector along translation axis, expressed in global frame
    ChVector<> axis = rot.GetZaxis();

    // Position, velocity, and acceleration (expressed in global frame)
    // ----------------------------------------------------------------

    ChVector<> pos = plate->GetPos();
    ChVector<> vel = plate->GetPos_dt();
    ChVector<> acc = plate->GetPos_dtdt();

    // The motion must be constant speed along the translation axis.
    ChVector<> pos_an = time * speed * axis;
    ChVector<> vel_an = speed * axis;
    ChVector<> acc_an = ChVector<>(0, 0, 0);

    ChVector<> pos_delta = pos - pos_an;
    if (pos_delta.Length() > pos_tol) {
        std::cout << "   at t = " << time << "   pos - pos_an = " << pos_delta.x() << "  " << pos_delta.y() << "  "
                  << pos_delta.z() << std::endl;
        return false;
    }

    ChVector<> vel_delta = vel - vel_an;
    if (vel_delta.Length() > vel_tol) {
        std::cout << "   at t = " << time << "   vel - vel_an = " << vel_delta.x() << "  " << vel_delta.y() << "  "
                  << vel_delta.z() << std::endl;
        return false;
    }

    ChVector<> acc_delta = acc - acc_an;
    if (acc_delta.Length() > acc_tol) {
        std::cout << "   at t = " << time << "   acc - acc_an = " << acc_delta.x() << "  " << acc_delta.y() << "  "
                  << acc_delta.z() << std::endl;
        return false;
    }

    // Orientation and angular velocity / acceleration (expressed in global frame)
    // ---------------------------------------------------------------------------

    ChQuaternion<> quat = plate->GetRot();
    ChVector<> avel = plate->GetWvel_par();
    ChVector<> aacc = plate->GetWacc_par();

    // The motion must maintain constant orientation of the plate body.
    ChQuaternion<> quat_an = rot;
    ChVector<> avel_an = ChVector<>(0, 0, 0);
    ChVector<> aacc_an = ChVector<>(0, 0, 0);

    ChQuaternion<> quat_delta = quat - quat_an;
    if (quat_delta.Length() > quat_tol) {
        std::cout << "   at t = " << time << "   quat - quat_an = " << quat_delta.e0() << "  " << quat_delta.e1()
                  << "  " << quat_delta.e2() << "  " << quat_delta.e3() << std::endl;
        return false;
    }

    ChVector<> avel_delta = avel - avel_an;
    if (avel_delta.Length() > avel_tol) {
        std::cout << "   at t = " << time << "   avel - avel_an = " << avel_delta.x() << "  " << avel_delta.y() << "  "
                  << avel_delta.z() << std::endl;
        return false;
    }

    ChVector<> aacc_delta = aacc - aacc_an;
    if (aacc_delta.Length() > aacc_tol) {
        std::cout << "   at t = " << time << "   aacc - aacc_an = " << aacc_delta.x() << "  " << aacc_delta.y() << "  "
                  << aacc_delta.z() << std::endl;
        return false;
    }

    // Reaction force and torque in prismatic joint
    // --------------------------------------------

    // These are expressed in the link coordinate system. We convert them to
    // the coordinate system of Body2 (in our case this is the ground).
    ChCoordsys<> linkCoordsysP = prismatic->GetLinkRelativeCoords();
    ChVector<> rforceP = prismatic->Get_react_force();
    ChVector<> rforceP_ground = linkCoordsysP.TransformDirectionLocalToParent(rforceP);

    ChVector<> rtorqueP = prismatic->Get_react_torque();
    ChVector<> rtorqueP_ground = linkCoordsysP.TransformDirectionLocalToParent(rtorqueP);

    // The reaction force in the prismatic joint is perpendicular to the
    // translation direction. This can be obtained from a force diagram.
    ChVector<> rforceP_an = gravity - Vdot(gravity, axis) * axis;
    ChVector<> rforceP_delta = rforceP_ground - rforceP_an;
    if (rforceP_delta.Length() > rforce_tol) {
        std::cout << "   at t = " << time << "   rforceP - rforceP_an = " << rforceP_delta.x() << "  " << rforceP_delta.y()
                  << "  " << rforceP_delta.z() << std::endl;
        return false;
    }

    // The reaction torque at the joint location on ground has a non-zero
    // component in the y direction only.
    ChVector<> rtorqueP_an = Vcross(pos_an, rforceP_an);
    ChVector<> rtorqueP_delta = rtorqueP_ground - rtorqueP_an;
    if (rtorqueP_delta.Length() > rtorque_tol) {
        std::cout << "   at t = " << time << "   rtorqueP - rtorqueP_an = " << rtorqueP_delta.x() << "  "
                  << rtorqueP_delta.y() << "  " << rtorqueP_delta.z() << std::endl;
        return false;
    }

    // Reaction force and torque in linear actuator
    // --------------------------------------------

    // These are expressed in the link coordinate system. The reaction force
    // represents the force that needs to be applied to the plate in order to
    // maintain the prescribed constant velocity.
    ChCoordsys<> linkCoordsysA = actuator->GetLinkRelativeCoords();
    ChVector<> rforceA = actuator->Get_react_force();
    ChVector<> rtorqueA = actuator->Get_react_torque();

    // Analytically, the driving force can be obtained from a force diagram along
    // the translation axis.
    double rforceA_an = mass * Vdot(acc_an - gravity, axis);
    double rforceA_delta = (-rforceA.x()) - rforceA_an;
    if (std::abs(rforceA_delta) > rforce_tol) {
        std::cout << "   at t = " << time << "   rforceA = " << -rforceA.x() << "  "
                  << "   rforceA_an = " << rforceA_an << "  "
                  << "   rforceA - rforceA_an = " << rforceA_delta << std::endl;
        return false;
    }

    ChVector<> rtorqueA_an = ChVector<>(0, 0, 0);
    ChVector<> rtorqueA_delta = rtorqueA - rtorqueA_an;
    if (rtorqueA_delta.Length() > rtorque_tol) {
        std::cout << "   at t = " << time << "   rtorqueA - rtorqueA_an = " << rtorqueA_delta.x() << "  "
                  << rtorqueA_delta.y() << "  " << rtorqueA_delta.z() << std::endl;
        return false;
    }

    // Constraint violations in prismatic joint
    // ----------------------------------------

    ChMatrix<>* CP = prismatic->GetC();
    for (int i = 0; i < 5; i++) {
        if (std::abs(CP->GetElement(i, 0)) > cnstr_tol) {
            std::cout << "   at t = " << time << "  constraint violation (prismatic " << i
                      << ") = " << CP->GetElement(i, 0) << std::endl;
            return false;
        }
    }

    // Constraint violations in linear actuator
    // ----------------------------------------

    ChMatrix<>* CA = actuator->GetC();
    if (std::abs(CA->GetElement(0, 0)) > cnstr_tol) {
        std::cout << "   at t = " << time << "  constraint violation (actuator) = " << CA->GetElement(0, 0)
                  << std::endl;
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------
// Worker function for performing the simulation with specified parameters.
// -----------------------------------------------------------------------------
bool TestLinActuator(ChMaterialSurfaceBase::ContactMethod cm,  // type of system (DEM or DVI)
                     const char* test_name,                    // name of this test
                     const ChQuaternion<>& rot,                // translation along Z axis
                     double speed,                             // imposed translation speed
                     bool animate)                             // if true, animate with OpenGL
{
    std::cout << test_name << std::endl;

  // Unit vector along translation axis, expressed in global frame
  ChVector<> axis = rot.GetZaxis();

  // Settings
  //---------

  int threads = 1;
  bool thread_tuning = false;

  double time_end = 2;
  double time_step = 1e-3;

  double tolerance = 1e-5;

  int max_iteration_bilateral = 100;
  int max_iteration_normal = 0;
  int max_iteration_sliding = 0;
  int max_iteration_spinning = 0;

  bool clamp_bilaterals = false;
  double bilateral_clamp_speed = 1000;

  double contact_recovery_speed = 1;

  // Create the mechanical system
  // ----------------------------

  ChSystemParallel* msystem;

  switch (cm) {
      case ChMaterialSurfaceBase::DEM:
          msystem = new ChSystemParallelDEM();
          break;
      case ChMaterialSurfaceBase::DVI:
          msystem = new ChSystemParallelDVI();
          break;
  }
  msystem->Set_G_acc(gravity);

  // Set number of threads.
  CHOMPfunctions::SetNumThreads(threads);
  msystem->GetSettings()->max_threads = threads;
  msystem->GetSettings()->perform_thread_tuning = thread_tuning;

  // Edit system settings
  msystem->GetSettings()->solver.tolerance = tolerance;
  msystem->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
  msystem->GetSettings()->solver.clamp_bilaterals = clamp_bilaterals;
  msystem->GetSettings()->solver.bilateral_clamp_speed = bilateral_clamp_speed;

  if (cm == ChMaterialSurfaceBase::DVI) {
    ChSystemParallelDVI* msystemDVI = static_cast<ChSystemParallelDVI*>(msystem);
    msystemDVI->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    msystemDVI->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    msystemDVI->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    msystemDVI->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    msystemDVI->ChangeSolverType(SolverType::APGD);
  }

  // Create the ground body.
  std::shared_ptr<ChBody> ground(msystem->NewBody());

  msystem->AddBody(ground);
  ground->SetBodyFixed(true);

  auto box_g = std::make_shared<ChBoxShape>();
  box_g->GetBoxGeometry().SetLengths(ChVector<>(0.1, 0.1, 5));
  box_g->Pos = 2.5 * axis;
  box_g->Rot = rot;
  ground->AddAsset(box_g);

  // Create the plate body.

  std::shared_ptr<ChBody> plate(msystem->NewBody());

  msystem->AddBody(plate);
  plate->SetPos(ChVector<>(0, 0, 0));
  plate->SetRot(rot);
  plate->SetPos_dt(speed * axis);
  plate->SetMass(mass);
  plate->SetInertiaXX(inertiaXX);

  auto box_p = std::make_shared<ChBoxShape>();
  box_p->GetBoxGeometry().SetLengths(ChVector<>(1, 1, 0.2));
  plate->AddAsset(box_p);

  // Create prismatic (translational) joint between plate and ground.
  // We set the ground as the "master" body (second one in the initialization
  // call) so that the link coordinate system is expressed in the ground frame.

  auto prismatic = std::make_shared<ChLinkLockPrismatic>();
  prismatic->Initialize(plate, ground, ChCoordsys<>(ChVector<>(0, 0, 0), rot));
  msystem->AddLink(prismatic);

  // Create a ramp function to impose constant speed.  This function returns
  //   y(t) = 0 + t * speed
  //   y'(t) = speed

  auto actuator_fun = std::make_shared<ChFunction_Ramp>(0.0, speed);

  // Create the linear actuator, connecting the plate to the ground.
  // Here, we set the plate as the master body (second one in the initialization
  // call) so that the link coordinate system is expressed in the plate body
  // frame.

  auto actuator = std::make_shared<ChLinkLinActuator>();
  ChVector<> pt1 = ChVector<>(0, 0, 0);
  ChVector<> pt2 = axis;
  actuator->Initialize(ground, plate, false, ChCoordsys<>(pt1, rot), ChCoordsys<>(pt2, rot));
  actuator->Set_lin_offset(1);
  actuator->Set_dist_funct(actuator_fun);
  msystem->AddLink(actuator);

  // Perform the simulation
  // ----------------------

  bool passed = true;
  double time = 0;

  if (animate) {
#ifdef CHRONO_OPENGL
    opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
    gl_window.Initialize(1280, 720, test_name, msystem);
    gl_window.SetCamera(ChVector<>(0, -10, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));
    gl_window.SetRenderMode(opengl::WIREFRAME);

    while (time < time_end) {
      // Advance simulation by one step.
      gl_window.DoStepDynamics(time_step);
      gl_window.Render();
      time += time_step;
      if (!VerifySolution(time, rot, speed, plate, prismatic, actuator))
        passed = false;
    }
#else
    std::cout << "OpenGL support not available.  Cannot animate mechanism." << std::endl;
    return false;
#endif
  } else {
    while (time < time_end) {
      msystem->DoStepDynamics(time_step);
      time += time_step;
      if (!VerifySolution(time, rot, speed, plate, prismatic, actuator))
        passed = false;
    }
  }

  std::cout << (passed ? "PASSED" : "FAILED") << std::endl << std::endl;

  return passed;
}

// -----------------------------------------------------------------------------
// Main driver function for running the simulation and validating the results.
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
  // No animation by default (i.e. when no program arguments)
  bool animate = (argc > 1);

  bool test_passed = true;

  // Case 1 - Translation axis vertical, imposed speed 1 m/s
  test_passed &= TestLinActuator(ChMaterialSurfaceBase::DEM, "Case 1 (DEM)", QUNIT, 1, animate);
  test_passed &= TestLinActuator(ChMaterialSurfaceBase::DVI, "Case 1 (DVI)", QUNIT, 1, animate);

  // Case 2 - Translation axis along X = Z, imposed speed 0.5 m/s
  test_passed &= TestLinActuator(ChMaterialSurfaceBase::DEM, "Case 2 (DEM)", Q_from_AngY(CH_C_PI / 4), 0.5, animate);
  test_passed &= TestLinActuator(ChMaterialSurfaceBase::DVI, "Case 2 (DVI)", Q_from_AngY(CH_C_PI / 4), 0.5, animate);

  // Return 0 if all tests passed and 1 otherwise
  return !test_passed;
}
