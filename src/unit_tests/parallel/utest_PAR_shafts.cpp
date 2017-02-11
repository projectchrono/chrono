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
// Tests for ChShaft and related components.
//
// =============================================================================

#include "chrono/physics/ChShaftsGear.h"
#include "chrono/physics/ChShaftsTorsionSpring.h"
#include "chrono/physics/ChShaftsBody.h"
#include "chrono/physics/ChShaftsPlanetary.h"
#include "chrono/physics/ChShaftsClutch.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_parallel/physics/ChSystemParallel.h"

using namespace chrono;
using namespace chrono::collision;

// -----------------------------------------------------------------------------
// Create a parallel system of specified type and set solver options
// -----------------------------------------------------------------------------
ChSystemParallel* CreateSystem(ChMaterialSurfaceBase::ContactMethod cm) {
  // Settings
  int threads = 1;
  bool thread_tuning = false;

  double tolerance = 1e-5;

  int max_iteration_bilateral = 100;
  int max_iteration_normal = 0;
  int max_iteration_sliding = 100;
  int max_iteration_spinning = 0;

  bool clamp_bilaterals = false;
  double bilateral_clamp_speed = 1000;

  double contact_recovery_speed = 1;

  // Create the mechanical system
  ChSystemParallel* system;

  switch (cm) {
      case ChMaterialSurfaceBase::DEM:
          system = new ChSystemParallelDEM();
          break;
      case ChMaterialSurfaceBase::DVI:
          system = new ChSystemParallelDVI();
          break;
  }

  // Set number of threads.
  CHOMPfunctions::SetNumThreads(threads);
  system->GetSettings()->max_threads = threads;
  system->GetSettings()->perform_thread_tuning = thread_tuning;

  // Edit system settings
  system->GetSettings()->solver.tolerance = tolerance;
  system->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
  system->GetSettings()->solver.clamp_bilaterals = clamp_bilaterals;
  system->GetSettings()->solver.bilateral_clamp_speed = bilateral_clamp_speed;

  if (cm == ChMaterialSurfaceBase::DVI) {
    ChSystemParallelDVI* systemDVI = static_cast<ChSystemParallelDVI*>(system);
    systemDVI->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    systemDVI->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
    systemDVI->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
    systemDVI->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
    systemDVI->ChangeSolverType(SolverType::APGD);
  }

  return system;
}

// -----------------------------------------------------------------------------
// Two shafts (inertias J1 and J2, respectively) connected through a gear with
// transmission ratio r.  A constant torque T is applied to the first shaft.
//
//             A           B
//         T  ||---[ r ]---||
//
// The analytical solution is obtained from:
//    J1 * acc1 = T + Tr1
//    J2 * acc2 = Tr2
//    acc2 = r * acc1
//    Tr1 = -r * Tr2
// as
//    acc1 = T / (J1 + J2 * r^2)
//    acc2 = r * acc1
//    Tr2 = J2 * acc2
//    Tr1 = -r * Tr2
// -----------------------------------------------------------------------------
bool TestShaftShaft(const char* test_name, ChMaterialSurfaceBase::ContactMethod cm) {
  std::cout << test_name << std::endl;

  // Parameters
  double J1 = 10;   // inertia of first shaft
  double J2 = 100;  // inertia of second shaft
  double r = -0.1;  // gear transmission ratio
  double T = 6;     // torque applied to first shaft

  // Create the system
  ChSystemParallel* system = CreateSystem(cm);

  // Create two 1-D shaft objects, with a constant torque applied to the first
  // shaft. By default, a ChShaft is free to rotate.
  auto shaftA = std::make_shared<ChShaft>();
  shaftA->SetInertia(J1);
  shaftA->SetAppliedTorque(T);
  system->Add(shaftA);

  auto shaftB = std::make_shared<ChShaft>();
  shaftB->SetInertia(J2);
  system->Add(shaftB);

  // Create a connection between the two shafts with a given trnsmission ratio.
  auto gearAB = std::make_shared<ChShaftsGear>();
  gearAB->Initialize(shaftA, shaftB);
  gearAB->SetTransmissionRatio(r);
  system->Add(gearAB);

  // Perform the simulation and verify results.
  double tol_pos = 1e-3;
  double tol_vel = 1e-4;
  double tol_acc = 1e-4;
  double tol_trq = 1e-4;

  bool passed = true;
  double time_end = 0.5;
  double time_step = 1e-3;
  double time = 0;

  while (time < time_end) {
    system->DoStepDynamics(time_step);
    time += time_step;

    // Verify solution
    double acc1_an = T / (J1 + J2 * r * r);
    double acc2_an = r * acc1_an;

    double Tr2_an = J2 * acc2_an;
    double Tr1_an = -r * Tr2_an;

    double vel1_an = acc1_an * time;
    double vel2_an = acc2_an * time;

    double pos1_an = acc1_an * time * time / 2;
    double pos2_an = acc2_an * time * time / 2;

    double pos1 = shaftA->GetPos();
    double vel1 = shaftA->GetPos_dt();
    double acc1 = shaftA->GetPos_dtdt();

    double pos2 = shaftB->GetPos();
    double vel2 = shaftB->GetPos_dt();
    double acc2 = shaftB->GetPos_dtdt();

    double Tr1 = gearAB->GetTorqueReactionOn1();
    double Tr2 = gearAB->GetTorqueReactionOn2();

    if (std::abs(pos1 - pos1_an) > tol_pos || std::abs(pos2 - pos2_an) > tol_pos) {
      passed = false;
    }

    if (std::abs(vel1 - vel1_an) > tol_vel || std::abs(vel2 - vel2_an) > tol_vel) {
      passed = false;
    }

    if (std::abs(acc1 - acc1_an) > tol_acc || std::abs(acc2 - acc2_an) > tol_acc) {
      passed = false;
    }

    if (std::abs(Tr1 - Tr1_an) > tol_trq || std::abs(Tr2 - Tr2_an) > tol_trq) {
      passed = false;
    }

    if (!passed)
        break;
  }

  std::cout << (passed ? "PASSED" : "FAILED") << std::endl;

  std::cout << "Time: " << time << "\n"
            << "  shaft A rot: " << shaftA->GetPos() << "  speed: " << shaftA->GetPos_dt()
            << "  accel: " << shaftA->GetPos_dtdt() << "\n"
            << "  shaft B rot: " << shaftB->GetPos() << "  speed: " << shaftB->GetPos_dt()
            << "  accel: " << shaftB->GetPos_dtdt() << "\n"
            << "  torque on A side: " << gearAB->GetTorqueReactionOn1()
            << "  torque on B side: " << gearAB->GetTorqueReactionOn2() << "\n\n\n";

  // Delete the system
  delete system;

  return passed;
}

// -----------------------------------------------------------------------------
// Test for shaft-body constraints: shaftA connected to bodyB.
// In this example we also add a 'torsional spring damper', shown as [ t ]
// that connects shafts A and C (C is shown as * because fixed).
// An external torque is applied to bodyB
//
//               B             A           C
//         Ta   <>---[ bs ]---||---[ t ]---*
//
// -----------------------------------------------------------------------------
bool TestShaftBody(const char* test_name, ChMaterialSurfaceBase::ContactMethod cm) {
  std::cout << test_name << std::endl;

  // Create the system
  ChSystemParallel* system = CreateSystem(cm);

  // Create 'A', a 1D shaft
  auto shaftA = std::make_shared<ChShaft>();
  shaftA->SetInertia(9);
  system->Add(shaftA);

  // Create 'C', a 1D shaft, fixed
  auto shaftC = std::make_shared<ChShaft>();
  shaftC->SetShaftFixed(true);
  system->Add(shaftC);

  // Create 'B', a 3D rigid body
  std::shared_ptr<ChBody> bodyB(system->NewBody());

  bodyB->Accumulate_torque(ChVector<>(0, 0, 3), true);  // set some constant torque to body
  system->Add(bodyB);

  // Make the torsional spring-damper between shafts A and C.
  auto shaft_torsionAC = std::make_shared<ChShaftsTorsionSpring>();
  shaft_torsionAC->Initialize(shaftA, shaftC);
  shaft_torsionAC->SetTorsionalStiffness(40);
  shaft_torsionAC->SetTorsionalDamping(0);
  system->Add(shaft_torsionAC);

  // Make the shaft 'A' connected to the rotation of the 3D body 'B'.
  // We must specify the direction (in body coordinates) along which the
  // shaft will affect the body.
  auto shaftbody_connection = std::make_shared<ChShaftsBody>();
  ChVector<> shaftdir(VECT_Z);
  shaftbody_connection->Initialize(shaftA, bodyB, shaftdir);
  system->Add(shaftbody_connection);

  // Perform the simulation and verify results.
  double tol_pos = 1e-3;
  double tol_vel = 1e-4;
  double tol_acc = 1e-4;
  double tol_trq = 1e-4;

  double time_end = 0.5;
  double time_step = 1e-3;
  double time = 0;

  while (time < time_end) {
    system->DoStepDynamics(time_step);
    time += time_step;
  }

  // Validate solution at t = 0.5, using the Chrono reference solution (h = 1e-3)
  bool passed = true;

  double posA = 0.0345404;  // shaftA angle
  double velA = 0.126221;   // shaftA angular velocity
  double accA = 0.162343;   // shaftA angular acceleration

  double avelB = 0.126221;  // z component of bodyB angular velocity
  double aaccB = 0.162343;  // z component of bodyB angular acceleration

  double spring_trqA = -1.38162;  // spring torque on shaftA
  double spring_trqC = 1.38162;   // spring torque on shaftC

  double trqA = 2.83766;   // reaction on shaftA
  double trqB = -2.83766;  // reaction on bodyB (z component)

  if (std::abs(shaftA->GetPos() - posA) > tol_pos || std::abs(shaftA->GetPos_dt() - velA) > tol_vel ||
      std::abs(shaftA->GetPos_dtdt() - accA) > tol_acc)
    passed = false;

  if (std::abs(bodyB->GetWvel_loc().z() - avelB) > tol_acc || std::abs(bodyB->GetWacc_loc().z() - aaccB) > tol_acc)
    passed = false;

  if (std::abs(shaft_torsionAC->GetTorqueReactionOn1() - spring_trqA) > tol_trq ||
      std::abs(shaft_torsionAC->GetTorqueReactionOn2() - spring_trqC) > tol_trq)
    passed = false;

  if (std::abs(shaftbody_connection->GetTorqueReactionOnShaft() - trqA) > tol_trq ||
      std::abs(shaftbody_connection->GetTorqueReactionOnBody().z() - trqB) > tol_trq)
    passed = false;

  std::cout << (passed ? "PASSED" : "FAILED") << std::endl;

  std::cout << "Time: " << time << "\n"
            << "  shaft A rot: " << shaftA->GetPos() << "  speed: " << shaftA->GetPos_dt()
            << "  accel: " << shaftA->GetPos_dtdt() << "\n"
            << "  body B angular speed on z: " << bodyB->GetWvel_loc().z() << "  accel on z: " << bodyB->GetWacc_loc().z()
            << "\n"
            << "  AC spring, torque on A side: " << shaft_torsionAC->GetTorqueReactionOn1()
            << "  torque on C side: " << shaft_torsionAC->GetTorqueReactionOn2() << "\n"
            << "  torque on shaft A: " << shaftbody_connection->GetTorqueReactionOnShaft() << "\n"
            << "  torque on body B: " << shaftbody_connection->GetTorqueReactionOnBody().x() << " "
            << shaftbody_connection->GetTorqueReactionOnBody().y() << " "
            << shaftbody_connection->GetTorqueReactionOnBody().z() << " "
            << "\n\n\n";

  // Delete the system
  delete system;

  return passed;
}

// -----------------------------------------------------------------------------
// Two shafts A and B, connected by a clutch [ c ]. Shafts starts with nonzero
// speed, and are free to rotate independently until the clutch is activated.
// After activation, the shafts decelerateuntil they have the same speed.
//
//       A           B
//  Ta  ||---[ c ]---||
//
// -----------------------------------------------------------------------------
bool TestClutch(const char* test_name, ChMaterialSurfaceBase::ContactMethod cm) {
  std::cout << test_name << std::endl;

  // Create the system
  ChSystemParallel* system = CreateSystem(cm);

  // Create a ChShaft that starts with nonzero angular velocity
  auto shaftA = std::make_shared<ChShaft>();
  shaftA->SetInertia(0.5);
  shaftA->SetPos_dt(30);
  system->Add(shaftA);

  // Create another ChShaft, with opposite initial angular velocity
  auto shaftB = std::make_shared<ChShaft>();
  shaftB->SetInertia(0.6);
  shaftB->SetPos_dt(-10);
  system->Add(shaftB);

  // Create a ChShaftsClutch, that represents a simplified model
  // of a clutch between two ChShaft objects (something that limits
  // the max transmitted torque, up to slippage).
  auto clutchAB = std::make_shared<ChShaftsClutch>();
  clutchAB->Initialize(shaftA, shaftB);
  clutchAB->SetTorqueLimit(60);
  clutchAB->SetModulation(0);
  system->Add(clutchAB);

  // Perform the simulation and verify results.
  bool passed = true;
  double tol_pos = 1e-3;
  double tol_vel = 1e-4;
  double tol_acc = 1e-4;
  double tol_trq = 1e-4;

  double time_end = 1.5;
  double time_step = 1e-3;
  double time = 0;

  while (time < time_end) {
    system->DoStepDynamics(time_step);
    time += time_step;

    if (time > 0.8)
      clutchAB->SetModulation(1);

    std::cout << "Time: " << time << "\n"
              << "  shaft A rot: " << shaftA->GetPos() << "  speed: " << shaftA->GetPos_dt()
              << "  accel: " << shaftA->GetPos_dtdt() << "\n"
              << "  shaft B rot: " << shaftB->GetPos() << "  speed: " << shaftB->GetPos_dt()
              << "  accel: " << shaftB->GetPos_dtdt() << "\n"
              << "  torque on A side: " << clutchAB->GetTorqueReactionOn1()
              << "  torque on B side: " << clutchAB->GetTorqueReactionOn2() << "\n";
  }

  std::cout << "Time: " << time << "\n"
            << "  shaft A rot: " << shaftA->GetPos() << "  speed: " << shaftA->GetPos_dt()
            << "  accel: " << shaftA->GetPos_dtdt() << "\n"
            << "  shaft B rot: " << shaftB->GetPos() << "  speed: " << shaftB->GetPos_dt()
            << "  accel: " << shaftB->GetPos_dtdt() << "\n"
            << "  torque on A side: " << clutchAB->GetTorqueReactionOn1()
            << "  torque on B side: " << clutchAB->GetTorqueReactionOn2() << "\n\n\n";

  // Delete the system
  delete system;

  return passed;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
bool TestShaftShaftShaft(const char* test_name, ChMaterialSurfaceBase::ContactMethod cm) {
  std::cout << test_name << std::endl;

  // Create the system
  ChSystemParallel* system = CreateSystem(cm);

  // Create shaft A, with applied torque
  auto shaftA = std::make_shared<ChShaft>();
  shaftA->SetInertia(0.5);
  shaftA->SetAppliedTorque(10);
  system->Add(shaftA);

  // Create shaft B
  auto shaftB = std::make_shared<ChShaft>();
  shaftB->SetInertia(0.5);
  system->Add(shaftB);

  // Create shaft C, that will be fixed (to be used as truss of epicycloidal reducer)
  auto shaftC = std::make_shared<ChShaft>();
  shaftC->SetShaftFixed(true);
  system->Add(shaftC);

  // Create a ChShaftsPlanetary, that represents a simplified model
  // of a planetary gear between THREE ChShaft objects (ex.: a car differential)
  // An epicycloidal reducer is a special type of planetary gear.
  auto planetaryBAC = std::make_shared<ChShaftsPlanetary>();
  planetaryBAC->Initialize(shaftB, shaftA, shaftC);  // output, carrier, fixed

  // We can set the ratios of the planetary using a simplified formula, for the
  // so called 'Willis' case. Imagine we hold fixed the carrier (shaft B in epic. reducers),
  // and leave free the truss C (the outer gear with inner teeth in our reducer); which is
  // the transmission ratio t0 that we get? It is simply t0=-Za/Zc, with Z = num of teeth of gears.
  // So just use the following to set all the three ratios automatically:
  double t0 = -50.0 / 100.0;  // suppose, in the reducer, that pinion A has 50 teeth and truss has 100 inner teeth.
  planetaryBAC->SetTransmissionRatioOrdinary(t0);
  system->Add(planetaryBAC);

  // Now, let's make a shaft D, that is fixed, and used for the right side
  // of a clutch (so the clutch will act as a brake).
  auto shaftD = std::make_shared<ChShaft>();
  shaftD->SetShaftFixed(true);
  system->Add(shaftD);

  // Make the brake. It is, in fact a clutch between shafts B and D, where
  // D is fixed as a truss, so the clutch will operate as a brake.
  auto clutchBD = std::make_shared<ChShaftsClutch>();
  clutchBD->Initialize(shaftB, shaftD);
  clutchBD->SetTorqueLimit(60);
  system->Add(clutchBD);

  // Perform the simulation and verify results.
  bool passed = true;
  double tol_pos = 1e-3;
  double tol_vel = 1e-4;
  double tol_acc = 1e-4;
  double tol_trq = 1e-4;

  double time_end = 0.5;
  double time_step = 1e-3;
  double time = 0;

  while (time < time_end) {
    system->DoStepDynamics(time_step);
    time += time_step;

    std::cout << "Time: " << time << "\n"
              << "  shaft A rot: " << shaftA->GetPos() << "  speed: " << shaftA->GetPos_dt()
              << "  accel: " << shaftA->GetPos_dtdt() << "\n"
              << "  shaft B rot: " << shaftB->GetPos() << "  speed: " << shaftB->GetPos_dt()
              << "  accel: " << shaftB->GetPos_dtdt() << "\n"
              << "  planetary react torques on shafts:\n"
              << "     on A: " << planetaryBAC->GetTorqueReactionOn2()
              << "     on B: " << planetaryBAC->GetTorqueReactionOn1()
              << "     on C: " << planetaryBAC->GetTorqueReactionOn3() << "\n";
  }

  std::cout << "Time: " << time << "\n"
            << "  shaft A rot: " << shaftA->GetPos() << "  speed: " << shaftA->GetPos_dt()
            << "  accel: " << shaftA->GetPos_dtdt() << "\n"
            << "  shaft B rot: " << shaftB->GetPos() << "  speed: " << shaftB->GetPos_dt()
            << "  accel: " << shaftB->GetPos_dtdt() << "\n"
            << "  planetary react torques on shafts:\n"
            << "     on A: " << planetaryBAC->GetTorqueReactionOn2()
            << "     on B: " << planetaryBAC->GetTorqueReactionOn1()
            << "     on C: " << planetaryBAC->GetTorqueReactionOn3() << "\n\n\n";

  return passed;
}

// -----------------------------------------------------------------------------
// Main driver function for running the simulation and validating the results.
// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
  bool test_passed = true;

  // Test shaft - shaft connection
  test_passed &= TestShaftShaft("shaft-shaft (DEM)", ChMaterialSurfaceBase::DEM);
  test_passed &= TestShaftShaft("shaft-shaft (DVI)", ChMaterialSurfaceBase::DVI);

  // Test shaft - body connection
  test_passed &= TestShaftBody("shaft-body (DEM)", ChMaterialSurfaceBase::DEM);
  test_passed &= TestShaftBody("shaft-body (DVI)", ChMaterialSurfaceBase::DVI);

  // Test clutch between shafts
  ////test_passed &= TestClutch("clutch (DEM)", ChMaterialSurfaceBase::DEM);
  ////test_passed &= TestClutch("clutch (DVI)", ChMaterialSurfaceBase::DVI);

  // Test shaft - shaft - shaft connection
  ////test_passed &= TestShaftShaftShaft("shaft-shaft-shaft (DEM)", ChMaterialSurfaceBase::DEM);
  ////test_passed &= TestShaftShaftShaft("shaft-shaft-shaft (DVI)", ChMaterialSurfaceBase::DVI);

  // Return 0 if all tests passed and 1 otherwise
  return !test_passed;
}
