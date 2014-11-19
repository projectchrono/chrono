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
// Authors: Mike Taylor, Radu Serban
// =============================================================================
//
// Test for the spherical joint
//
// Recall that Irrlicht uses a left-hand frame, so everything is rendered with
// left and right flipped.
//
// =============================================================================

#include <ostream>
#include <fstream>

#include "core/ChFileutils.h"

#include "physics/ChSystem.h"
#include "physics/ChBody.h"

#include "unit_IRRLICHT/ChIrrApp.h"

#include "ChronoT_config.h"
#include "utils/ChUtilsData.h"
#include "utils/ChUtilsInputOutput.h"
#include "utils/ChUtilsValidation.h"

using namespace chrono;
using namespace irr;


// =============================================================================
// Local variables
//
static const std::string val_dir = "../VALIDATION/";
static const std::string out_dir = val_dir + "SPHERICAL_JOINT/";
static const std::string ref_dir = "validation/spherical_joint/";

// =============================================================================
// Prototypes of local functions
//
bool TestSpherical(const ChVector<>& jointLoc, const ChQuaternion<>& jointRot,
                  double simTimeStep, double outTimeStep,
                  const std::string& testName, bool animate);
bool ValidateReference(const std::string& testName, const std::string& what, double tolerance);
bool ValidateConstraints(const std::string& testName, double tolerance);
bool ValidateEnergy(const std::string& testName, double tolerance);
utils::CSV_writer OutStream();

// =============================================================================
//
// Main driver function for running the simulation and validating the results.
//
int main(int argc, char* argv[])
{
  bool animate = (argc > 1);

  // Set the path to the Chrono data folder
  SetChronoDataPath(CHRONO_DATA_DIR);

  // Create output directory (if it does not already exist)
  if (ChFileutils::MakeDirectory(val_dir.c_str()) < 0) {
    std::cout << "Error creating directory " << val_dir << std::endl;
    return 1;
  }
  if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
    std::cout << "Error creating directory " << out_dir << std::endl;
    return 1;
  }

  // Set the simulation and output step sizes
  double sim_step = 5e-4;
  double out_step = 1e-2;

  std::string test_name;
  bool test_passed = true;

  // Case 1 - Joint at the origin and aligned with the global frame.

  test_name = "Spherical_Case01";
  TestSpherical(ChVector<>(0, 0, 0), QUNIT, sim_step, out_step, test_name, animate);
  if (!animate) {
    test_passed &= ValidateReference(test_name, "Pos", 2e-3);
    test_passed &= ValidateReference(test_name, "Vel", 1e-4);
    test_passed &= ValidateReference(test_name, "Acc", 2e-2);
    test_passed &= ValidateReference(test_name, "Quat", 1e-3);
    test_passed &= ValidateReference(test_name, "Avel", 2e-2);
    test_passed &= ValidateReference(test_name, "Aacc", 2e-2);
    test_passed &= ValidateReference(test_name, "Rforce", 2e-2);
    test_passed &= ValidateReference(test_name, "Rtorque", 1e-6);
    test_passed &= ValidateEnergy(test_name, 1e-2);
    test_passed &= ValidateConstraints(test_name, 1e-5);
  }

  // Case 2 - Joint at (1,2,3) and joint z aligned with the global axis along Y = Z.
  // In this case, the joint must be rotated -pi/4 about the global X-axis.

  test_name = "Spherical_Case02";
  TestSpherical(ChVector<>(1, 2, 3), Q_from_AngX(-CH_C_PI_4), sim_step, out_step, test_name, animate);
  if (!animate) {
    test_passed &= ValidateReference(test_name, "Pos", 2e-3);
    test_passed &= ValidateReference(test_name, "Vel", 1e-4);
    test_passed &= ValidateReference(test_name, "Acc", 2e-2);
    test_passed &= ValidateReference(test_name, "Quat", 1e-3);
    test_passed &= ValidateReference(test_name, "Avel", 2e-2);
    test_passed &= ValidateReference(test_name, "Aacc", 2e-2);
    test_passed &= ValidateReference(test_name, "Rforce", 2e-2);
    test_passed &= ValidateReference(test_name, "Rtorque", 1e-6);
    test_passed &= ValidateEnergy(test_name, 1e-2);
    test_passed &= ValidateConstraints(test_name, 1e-5);
  }

  // Return 0 if all tests passed and 1 otherwise
  return !test_passed;
}

// =============================================================================
//
// Worker function for performing the simulation with specified parameters.
//
bool TestSpherical(const ChVector<>&     jointLoc,         // absolute location of joint
                   const ChQuaternion<>& jointRot,         // orientation of joint
                   double                simTimeStep,      // simulation time step
                   double                outTimeStep,      // output time step
                   const std::string&    testName,         // name of this test
                   bool                  animate)          // if true, animate with Irrlich
{
  std::cout << "TEST: " << testName << std::endl;

  // Settings
  //---------

  // There are no units in Chrono, so values must be consistent
  // (MKS is used in this example)

  double mass = 1.0;                     // mass of pendulum
  double length = 4.0;                   // length of pendulum
  ChVector<> inertiaXX(0.04, 0.1, 0.1);  // mass moments of inertia of pendulum (centroidal frame)
  double g = 9.80665;

  double timeRecord = 5;                 // simulation length

  // Create the mechanical system
  // ----------------------------

  // Create a ChronoENGINE physical system: all bodies and constraints will be
  // handled by this ChSystem object.

  ChSystem my_system;
  my_system.Set_G_acc(ChVector<>(0.0, 0.0, -g));

  my_system.SetIntegrationType(ChSystem::INT_ANITESCU);
  my_system.SetIterLCPmaxItersSpeed(100);
  my_system.SetIterLCPmaxItersStab(100); //Tasora stepper uses this, Anitescu does not
  my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
  my_system.SetTol(1e-6);
  my_system.SetTolForce(1e-4);

  // Create the ground body

  ChSharedBodyPtr  ground(new ChBody);
  my_system.AddBody(ground);
  ground->SetBodyFixed(true);
  // Add some geometry to the ground body for visualizing the spherical joint
  ChSharedPtr<ChSphereShape> sph_g(new ChSphereShape);
  sph_g->GetSphereGeometry().center = jointLoc;
  sph_g->GetSphereGeometry().rad = 0.2;
  ground->AddAsset(sph_g);



  // Create the pendulum body in an initial configuration at rest, with an 
  // orientatoin that matches the specified joint orientation and a position
  // consistent with the specified joint location.
  // The pendulum CG is assumed to be at half its length.

  ChSharedBodyPtr  pendulum(new ChBody);
  my_system.AddBody(pendulum);
  pendulum->SetPos(jointLoc + jointRot.Rotate(ChVector<>(length / 2, 0, 0)));
  pendulum->SetRot(jointRot);
  pendulum->SetMass(mass);
  pendulum->SetInertiaXX(inertiaXX);
  // Add some geometry to the pendulum for visualization
  ChSharedPtr<ChCylinderShape> cyl_p1(new ChCylinderShape);
  cyl_p1->GetCylinderGeometry().p1 = ChVector<>(-length / 2, 0, 0);
  cyl_p1->GetCylinderGeometry().p2 = ChVector<>(length / 2, 0, 0);
  cyl_p1->GetCylinderGeometry().rad = 0.1;
  pendulum->AddAsset(cyl_p1);
  ChSharedPtr<ChCylinderShape> cyl_p2(new ChCylinderShape);
  cyl_p2->GetCylinderGeometry().p1 = ChVector<>(-length / 2, 0, -0.2);
  cyl_p2->GetCylinderGeometry().p2 = ChVector<>(-length / 2, 0, 0.2);
  cyl_p2->GetCylinderGeometry().rad = 0.1;
  pendulum->AddAsset(cyl_p2);

  // Create revolute joint between pendulum and ground at "loc" in the global
  // reference frame. The revolute joint's axis of rotation will be the Z axis
  // of the specified rotation matrix.

  ChSharedPtr<ChLinkLockSpherical>  sphericalJoint(new ChLinkLockSpherical);
  sphericalJoint->Initialize(pendulum, ground, ChCoordsys<>(jointLoc, jointRot));
  my_system.AddLink(sphericalJoint);

  // Perform the simulation (animation with Irrlicht option)
  // -------------------------------------------------------

  if (animate)
  {
    // Create the Irrlicht application for visualization
    ChIrrApp * application = new ChIrrApp(&my_system, L"ChLinkSpherical demo", core::dimension2d<u32>(800, 600), false, true);
    application->AddTypicalLogo();
    application->AddTypicalSky();
    application->AddTypicalLights();
    core::vector3df lookat((f32)jointLoc.x, (f32)jointLoc.y, (f32)jointLoc.z);
    application->AddTypicalCamera(lookat + core::vector3df(0, 3, -6), lookat);

    // Now have the visulization tool (Irrlicht) create its geometry from the
    // assets defined above
    application->AssetBindAll();
    application->AssetUpdateAll();

    application->SetTimestep(simTimeStep);

    // Simulation loop
    while (application->GetDevice()->run())
    {
      application->BeginScene();
      application->DrawAll();

      // Draw an XZ grid at the global origin to add in visualization
      ChIrrTools::drawGrid(
        application->GetVideoDriver(), 1, 1, 20, 20,
        ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)),
        video::SColor(255, 80, 100, 100), true);

      application->DoStep();  //Take one step in time
      application->EndScene();
    }

    return true;
  }

  // Perform the simulation (record results option)
  // ------------------------------------------------

  // Create the CSV_Writer output objects (TAB delimited)
  utils::CSV_writer out_pos = OutStream();
  utils::CSV_writer out_vel = OutStream();
  utils::CSV_writer out_acc = OutStream();

  utils::CSV_writer out_quat = OutStream();
  utils::CSV_writer out_avel = OutStream();
  utils::CSV_writer out_aacc = OutStream();

  utils::CSV_writer out_rfrc = OutStream();
  utils::CSV_writer out_rtrq = OutStream();

  utils::CSV_writer out_energy = OutStream();

  utils::CSV_writer out_cnstr = OutStream();

  // Write headers
  out_pos << "Time" << "X_Pos" << "Y_Pos" << "Z_Pos" << std::endl;
  out_vel << "Time" << "X_Vel" << "Y_Vel" << "Z_Vel" << std::endl;
  out_acc << "Time" << "X_Acc" << "Y_Acc" << "Z_Acc" << std::endl;

  out_quat << "Time" << "e0" << "e1" << "e2" << "e3" << std::endl;
  out_avel << "Time" << "X_AngVel" << "Y_AngVel" << "Z_AngVel" << std::endl;
  out_aacc << "Time" << "X_AngAcc" << "Y_AngAcc" << "Z_AngAcc" << std::endl;

  out_rfrc << "Time" << "X_Force" << "Y_Force" << "Z_Force" << std::endl;
  out_rtrq << "Time" << "X_Torque" << "Y_Torque" << "Z_Torque" << std::endl;

  out_energy << "Time" << "Transl_KE" << "Rot_KE" << "Delta_PE" << "KE+PE" << std::endl;

  out_cnstr << "Time" << "Cnstr_1" << "Cnstr_2" << "Cnstr_3" << "Constraint_4" << "Cnstr_5" << std::endl;

  // Perform a system assembly to ensure we have the correct accelerations at
  // the initial time.
  my_system.DoFullAssembly();

  // Total energy at initial time.
  ChMatrix33<> inertia = pendulum->GetInertia();
  ChVector<> angVelLoc = pendulum->GetWvel_loc();
  double transKE = 0.5 * mass * pendulum->GetPos_dt().Length2();
  double rotKE = 0.5 * Vdot(angVelLoc, inertia * angVelLoc);
  double deltaPE = mass * g * (pendulum->GetPos().z - jointLoc.z);
  double totalE0 = transKE + rotKE + deltaPE;

  // Simulation loop
  double simTime = 0;
  double outTime = 0;

  while (simTime <= timeRecord + simTimeStep / 2)
  {
    // Ensure that the final data point is recorded.
    if (simTime >= outTime - simTimeStep / 2)
    {

      // CM position, velocity, and acceleration (expressed in global frame).
      const ChVector<>& position = pendulum->GetPos();
      const ChVector<>& velocity = pendulum->GetPos_dt();
      out_pos << simTime << position << std::endl;
      out_vel << simTime << velocity << std::endl;
      out_acc << simTime << pendulum->GetPos_dtdt() << std::endl;

      // Orientation, angular velocity, and angular acceleration (expressed in
      // global frame).
      out_quat << simTime << pendulum->GetRot() << std::endl;
      out_avel << simTime << pendulum->GetWvel_par() << std::endl;
      out_aacc << simTime << pendulum->GetWacc_par() << std::endl;

      // Reaction Force and Torque
      // These are expressed in the link coordinate system. We convert them to
      // the coordinate system of Body2 (in our case this is the ground).
      ChCoordsys<> linkCoordsys = sphericalJoint->GetLinkRelativeCoords();
      ChVector<> reactForce = sphericalJoint->Get_react_force();
      ChVector<> reactForceGlobal = linkCoordsys.TransformDirectionLocalToParent(reactForce);
      out_rfrc << simTime << reactForceGlobal << std::endl;

      ChVector<> reactTorque = sphericalJoint->Get_react_torque();
      ChVector<> reactTorqueGlobal = linkCoordsys.TransformDirectionLocalToParent(reactTorque);
      out_rtrq << simTime << reactTorqueGlobal << std::endl;

      // Conservation of Energy
      // Translational Kinetic Energy (1/2*m*||v||^2)
      // Rotational Kinetic Energy (1/2 w'*I*w)
      // Delta Potential Energy (m*g*dz)
      ChMatrix33<> inertia = pendulum->GetInertia();
      ChVector<> angVelLoc = pendulum->GetWvel_loc();
      double transKE = 0.5 * mass * velocity.Length2();
      double rotKE = 0.5 * Vdot(angVelLoc, inertia * angVelLoc);
      double deltaPE = mass * g * (position.z - jointLoc.z);
      double totalE = transKE + rotKE + deltaPE;
      out_energy << simTime << transKE << rotKE << deltaPE << totalE - totalE0 << std::endl;;

      // Constraint violations
      ChMatrix<>* C = sphericalJoint->GetC();
      out_cnstr << simTime
                << C->GetElement(0, 0)
                << C->GetElement(1, 0)
                << C->GetElement(2, 0) << std::endl;

      // Increment output time
      outTime += outTimeStep;
    }

    // Advance simulation by one step
    my_system.DoStepDynamics(simTimeStep);

    // Increment simulation time
    simTime += simTimeStep;
  }

  // Write output files
  out_pos.write_to_file(out_dir + testName + "_CHRONO_Pos.txt", testName + "\n\n");
  out_vel.write_to_file(out_dir + testName + "_CHRONO_Vel.txt", testName + "\n\n");
  out_acc.write_to_file(out_dir + testName + "_CHRONO_Acc.txt", testName + "\n\n");

  out_quat.write_to_file(out_dir + testName + "_CHRONO_Quat.txt", testName + "\n\n");
  out_avel.write_to_file(out_dir + testName + "_CHRONO_Avel.txt", testName + "\n\n");
  out_aacc.write_to_file(out_dir + testName + "_CHRONO_Aacc.txt", testName + "\n\n");

  out_rfrc.write_to_file(out_dir + testName + "_CHRONO_Rforce.txt", testName + "\n\n");
  out_rtrq.write_to_file(out_dir + testName + "_CHRONO_Rtorque.txt", testName + "\n\n");

  out_energy.write_to_file(out_dir + testName + "_CHRONO_Energy.txt", testName + "\n\n");

  out_cnstr.write_to_file(out_dir + testName + "_CHRONO_Constraints.txt", testName + "\n\n");

  return true;
}

// =============================================================================
//
// Wrapper function for comparing the specified simulation quantities against a
// reference file.
//
bool ValidateReference(const std::string& testName,    // name of this test
                       const std::string& what,        // identifier for test quantity
                       double             tolerance)   // validation tolerance
{
  std::string sim_file = out_dir + testName + "_CHRONO_" + what + ".txt";
  std::string ref_file = ref_dir + testName + "_ADAMS_" + what + ".txt";
  utils::DataVector norms;

  bool check = utils::Validate(sim_file, utils::GetModelDataFile(ref_file), utils::RMS_NORM, tolerance, norms);
  std::cout << "   validate " << what << (check ? ": Passed" : ": Failed") << "  [  ";
  for (size_t col = 0; col < norms.size(); col++)
    std::cout << norms[col] << "  ";
  std::cout << "  ]" << std::endl;

  return check;
}

// Wrapper function for checking constraint violations.
//
bool ValidateConstraints(const std::string& testName,  // name of this test
                         double             tolerance) // validation tolerance
{
  std::string sim_file = out_dir + testName + "_CHRONO_Constraints.txt";
  utils::DataVector norms;

  bool check = utils::Validate(sim_file, utils::RMS_NORM, tolerance, norms);
  std::cout << "   validate Constraints" << (check ? ": Passed" : ": Failed") << "  [  ";
  for (size_t col = 0; col < norms.size(); col++)
    std::cout << norms[col] << "  ";
  std::cout << "  ]" << std::endl;

  return check;
}

// wrapper function for checking energy conservation.
//
bool ValidateEnergy(const std::string& testName,  // name of this test
                    double             tolerance) // validation tolerance
{
  std::string sim_file = out_dir + testName + "_CHRONO_Energy.txt";
  utils::DataVector norms;

  utils::Validate(sim_file, utils::RMS_NORM, tolerance, norms);

  bool check = norms[norms.size() - 1] <= tolerance;
  std::cout << "   validate Energy" << (check ? ": Passed" : ": Failed") 
            << "  [  " << norms[norms.size() - 1] << "  ]" << std::endl;

  return check;
}

// =============================================================================
//
// Utility function to create a CSV output stream and set output format options.
//
utils::CSV_writer OutStream()
{
  utils::CSV_writer out("\t");

  out.stream().setf(std::ios::scientific | std::ios::showpos);
  out.stream().precision(6);

  return out;
}















//// =============================================================================
//// PROJECT CHRONO - http://projectchrono.org
////
//// Copyright (c) 2014 projectchrono.org
//// All right reserved.
////
//// Use of this source code is governed by a BSD-style license that can be found
//// in the LICENSE file at the top level of the distribution and at
//// http://projectchrono.org/license-chrono.txt.
////
//// =============================================================================
//// Authors: Mike Taylor, Radu Serban
//// =============================================================================
////
//// Test for spherical joint
////
//// Recall that Irrlicht uses a left-hand frame, so everything is rendered with
//// left and right flipped.
////
//// =============================================================================
//// TO DO:
////    Report test run time & test pass/fail (determine what the criteria is)
//// =============================================================================
//
//#include <ostream>
//#include <fstream>
//
//#include "core/ChFileutils.h"
//
//#include "physics/ChSystem.h"
//#include "physics/ChBody.h"
//
//#include "unit_IRRLICHT/ChIrrApp.h"
//
//#include "ChronoT_config.h"
//#include "utils/ChUtilsData.h"
//#include "utils/ChUtilsInputOutput.h"
//#include "utils/ChUtilsValidation.h"
//
//using namespace chrono;
//using namespace irr;
//
//
//// =============================================================================
//
//void TestSpherical(const ChVector<>&     loc,
//                   const ChQuaternion<>& revAxisRot,
//                   double                simTimeStep,
//                   const std::string&    outputFilename,
//                   bool                  animate)
//{
//
//  //Settings
//  //----------------------------------------------------------------------------
//  // There are no units in Chrono, so values must be consistant (MKS is used in this example)
//
//  double mass = 1.0;                // mass of pendulum
//  double length = 4.0;              // length of pendulum
//  ChVector<> inertiaXX(1, 1, 1);    // mass moments of inertia of pendulum
//  double g = 9.80665;
//
//  double timeRecord = 5;            // Stop recording to the file after this much simulated time
//  double printTimeStep = 0.001;     // Write the output file at this simulation time step
//
//  SetChronoDataPath(CHRONO_DATA_DIR);
//
//
//  // Create the mechanical system
//  // ----------------------------
//
//  // 1- Create a ChronoENGINE physical system: all bodies and constraints
//  //    will be handled by this ChSystem object.
//
//  ChSystem my_system;
//  my_system.Set_G_acc(ChVector<>(0.0, 0.0, -g));
//
//  my_system.SetIntegrationType(ChSystem::INT_ANITESCU);
//  my_system.SetIterLCPmaxItersSpeed(100);
//  my_system.SetIterLCPmaxItersStab(100); //Tasora stepper uses this, Anitescu does not
//  my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
//
//  // 2- Create the rigid bodies of the system
//
//  // ..the ground
//  ChSharedBodyPtr  ground(new ChBody);
//  my_system.AddBody(ground);
//  ground->SetBodyFixed(true);
//  // Add some geometry to the ground body for visualizing the spherical joint
//  ChSharedPtr<ChSphereShape> sph_g(new ChSphereShape);
//  sph_g->GetSphereGeometry().center = loc;
//  sph_g->GetSphereGeometry().rad = 0.2;
//  ground->AddAsset(sph_g);
//
//  // ..the pendulum (Assumes the pendulum's CG is at half its length)
//  ChSharedBodyPtr  pendulum(new ChBody);
//  my_system.AddBody(pendulum);
//  pendulum->SetPos(loc + ChVector<>(length / 2, 0, 0));   // position of COG of pendulum in the Global Reference Frame
//  pendulum->SetMass(mass);
//  pendulum->SetInertiaXX(inertiaXX);   // Set the body's inertia about the CG in the Global Reference Frame 
//  // Add some geometry to the pendulum for visualization
//  ChSharedPtr<ChCylinderShape> cyl_p(new ChCylinderShape);
//  cyl_p->GetCylinderGeometry().p1 = ChVector<>(-length / 2, 0, 0);
//  cyl_p->GetCylinderGeometry().p2 = ChVector<>(length / 2, 0, 0);
//  cyl_p->GetCylinderGeometry().rad = 0.1;
//  pendulum->AddAsset(cyl_p);
//
//  // 3- Create constraints: the mechanical joints between the rigid bodies.
//
//  // .. a spherical joint between pendulum and ground at "loc" in the global reference frame with the applied rotation
//  ChSharedPtr<ChLinkLockSpherical>  sphericalJoint(new ChLinkLockSpherical);
//  sphericalJoint->Initialize(pendulum, ground, ChCoordsys<>(loc, revAxisRot));
//  my_system.AddLink(sphericalJoint);
//
//
//  // Create the Irrlicht application for visualization
//  // -------------------------------------------------
//  ChIrrApp * application;
//  if(animate){
//    application = new ChIrrApp(&my_system, L"ChLinkLockSpherical demo", core::dimension2d<u32>(800, 600), false, true);
//    application->AddTypicalLogo();
//    application->AddTypicalSky();
//    application->AddTypicalLights();
//    core::vector3df lookat((f32)loc.x, (f32)loc.y, (f32)loc.z);
//    application->AddTypicalCamera(lookat + core::vector3df(0, 3, -6), lookat);
//
//    application->AssetBindAll();     //Now have the visulization tool (Irrlicht) create its geometry from the assets defined above
//    application->AssetUpdateAll();
//
//    application->SetTimestep(simTimeStep);
//  }
//
//  // Create output file for results & add in column headers (tab deliminated)
//  // ------------------------------------------------------------------------
//  std::ofstream outf(outputFilename.c_str());
//  if (outf) {
//    outf << "timeElapsed(s)\t";
//    outf << "X_Pos(m)\tY_Pos(m)\tZ_Pos\tLength_Pos(m)\t";
//    outf << "X_Vel(m/s)\tY_Vel(m/s)\tZ_Vel(m/s)\tLength_Vel(m/s)\t";
//    outf << "X_Accel(m/s^2)\tY_Accel(m/s^2)\tZ_Accell(m/s^2)\tLength_Accel(m/s^2)\t";
//    outf << "e0_quaternion\te1_quaternion\te2_quaternion\te3_quaternion\t";
//    outf << "X_AngVel(rad/s)\tY_AngVel(rad/s)\tZ_AngVel(rad/s)\tLength_AngVel(rad/s)\t";
//    outf << "X_AngAccel(rad/s^2)\tY_AngAccel(rad/s^2)\tZ_AngAccell(rad/s^2)\tLength_AngAccel(rad/s^2)\t";
//    outf << "X_Glb_ReactionFrc(N)\tY_Glb_ReactionFrc(N)\tZ_Glb_ReactionFrc(N)\tLength_Glb_ReactionFrc(N)\t";
//    outf << "X_Glb_ReactionTrq(Nm)\tY_Glb_ReactionTrq(Nm)\tZ_Glb_ReactionTrq(Nm)\tLength_Glb_ReactionTrq(Nm)\t";
//    outf << "Total_Kinetic_Energy(J)\tTranslational_Kinetic_Energy(J)\tAngular_Kinetic_Energy(J)\tDelta_Potential_Energy(J)\t";
//    outf << std::endl;
//  }
//  else {
//    std::cout << "Output file is invalid" << std::endl;
//  }
//
//
//  // Simulation loop
//  // ---------------
//
//  double timeElapsed = 0;
//  double lastPrint = -printTimeStep;
//  bool continueSimulation = true;
//
//  while (continueSimulation)
//  {
//    // Write current translational and rotational position, velocity, acceleration, 
//    // reaction force, and reaction torque of pendulum to output file
//
//    //Add a little error tolerance on the end time to ensure that the final data point is recorded
//    if (outf && (timeElapsed <= timeRecord+simTimeStep/2) && (timeElapsed+simTimeStep/2>=lastPrint+printTimeStep)) {
//      lastPrint = lastPrint + printTimeStep;
//
//      // Time elapsed
//      outf << timeElapsed << "\t";
//
//      // Position of the Pendulum's CG in the Global Reference Frame
//      ChVector<double> position = pendulum->GetPos();
//      outf << position.x << "\t" << position.y << "\t" << position.z << "\t" << (position - loc).Length() << "\t";
//
//      // Velocity of the Pendulum's CG in the Global Reference Frame
//      ChVector<double> velocity = pendulum->GetPos_dt();
//      outf << velocity.x << "\t" << velocity.y << "\t" << velocity.z << "\t" << velocity.Length() << "\t";
//
//      // Acceleration of the Pendulum's CG in the Global Reference Frame
//      ChVector<double> acceleration = pendulum->GetPos_dtdt();
//      outf << acceleration.x << "\t" << acceleration.y << "\t" << acceleration.z << "\t" << acceleration.Length() << "\t";
//
//      // Angular Position quaternion of the Pendulum with respect to the Global Reference Frame
//      ChQuaternion<double> rot = pendulum->GetRot();
//      outf << rot.e0 << "\t" << rot.e1 << "\t" << rot.e2 << "\t" << rot.e3 << "\t";
//
//      // Angular Velocity of the Pendulum with respect to the Global Reference Frame
//      ChVector<double> angVel = pendulum->GetWvel_par();
//      outf << angVel.x << "\t" << angVel.y << "\t" << angVel.z << "\t" << angVel.Length() << "\t";
//
//      // Angular Acceleration of the Pendulum with respect to the Global Reference Frame
//      ChVector<double> angAccel = pendulum->GetWacc_par();
//      outf << angAccel.x << "\t" << angAccel.y << "\t" << angAccel.z << "\t" << angAccel.Length() << "\t";
//
//      // Reaction Force and Torque
//      // These are expressed in the link coordinate system. We convert them to
//      // the coordinate system of Body2 (in our case this is the ground).
//      ChCoordsys<> linkCoordsys = sphericalJoint->GetLinkRelativeCoords();
//      ChVector<double> reactForce = sphericalJoint->Get_react_force();
//      ChVector<double> reactForceGlobal = linkCoordsys.TransformDirectionLocalToParent(reactForce);
//      outf << reactForceGlobal.x << "\t" << reactForceGlobal.y << "\t" << reactForceGlobal.z << "\t" << reactForceGlobal.Length() << "\t";
//
//      ChVector<double> reactTorque = sphericalJoint->Get_react_torque();
//      ChVector<double> reactTorqueGlobal = linkCoordsys.TransformDirectionLocalToParent(reactTorque);
//      outf << reactTorqueGlobal.x << "\t" << reactTorqueGlobal.y << "\t" << reactTorqueGlobal.z << "\t" << reactTorqueGlobal.Length() << "\t";
//
//      // Conservation of Energy
//      //Translational Kinetic Energy (1/2*m*||v||^2)
//      //Rotational Kinetic Energy (1/2 w'*I*w)  ChMatrix33*vector is valid since [3x3]*[3x1] = [3x1]
//      //Delta Potential Energy (m*g*dz)
//      ChMatrix33<> inertia = pendulum->GetInertia(); //3x3 Inertia Tensor in the local coordinate frame
//      ChVector<> angVelLoc = pendulum->GetWvel_loc();
//      double transKE = 0.5*mass*velocity.Length2();
//      double rotKE = 0.5*Vdot(angVelLoc, inertia * angVelLoc);
//      double deltaPE = mass*g*(position.z-loc.z);
//      double totalKE = transKE + rotKE;
//      outf << totalKE  << "\t" << transKE  << "\t" << rotKE  << "\t" << deltaPE  << "\t"  << std::endl;;
//    }
//
//
//    // Output a message to the command window once timeRecord has been reached
//    //   Add a little error tolerance to make sure this event is captured
//    if ((timeElapsed >= timeRecord-simTimeStep/2) && (timeElapsed <= timeRecord+simTimeStep/2)) {
//        std::cout << "All Simulation Results have been recorded to file." << std::endl;
//    }
//
//    // Advance simulation by one step
//    timeElapsed += simTimeStep;
//    if(animate) {
//      application->BeginScene();
//      application->DrawAll();
//
//      // Draw an XZ grid at the global origin to add in visualization
//      ChIrrTools::drawGrid(
//        application->GetVideoDriver(), 1, 1, 20, 20,
//        ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)),
//        video::SColor(255, 80, 100, 100), true);
//
//      application->DoStep();  //Take one step in time
//      application->EndScene();
//
//      continueSimulation = application->GetDevice()->run();
//    }
//    else{
//      my_system.DoStepDynamics(simTimeStep);  //Take one step in time
//      continueSimulation = (timeElapsed <= timeRecord+simTimeStep/2);
//    }
//  }
//
//  // Close output file
//  outf.close();
//
//
//}
//
//// =============================================================================
//
//int main(int argc, char* argv[])
//{
//  bool animate = (argc > 1);
//
//  // Set the path to the Chrono data folder
//  // --------------------------------------
//
//  SetChronoDataPath(CHRONO_DATA_DIR);
//
//  // Create output directory (if it does not already exist)
//  if (ChFileutils::MakeDirectory("../VALIDATION") < 0) {
//    std::cout << "Error creating directory '../VALIDATION'" << std::endl;
//    return 1;
//  }
//  if (ChFileutils::MakeDirectory("../VALIDATION/SPHERICAL_JOINT") < 0) {
//    std::cout << "Error creating directory '../VALIDATION/SPHERICAL_JOINT'" << std::endl;
//    return 1;
//  }
//
//  std::string out_dir = "../VALIDATION/SPHERICAL_JOINT/";
//  std::string ref_dir = "validation/spherical_joint/";
//
//  bool test_passed = true;
//
//
//  std::cout << "\nStarting Spherical Test Case 01\n\n";
//  //Case 1 - Spherical Joint at the origin, and aligned with the global coordinate system
//  //  Note the spherical joint only allows 3 DOF(all 3 rotations)
//  TestSpherical(ChVector<> (0, 0, 0), ChQuaternion<> (Q_from_AngX(0)), .001, "SphericalJointData_Case01.txt",animate);
//
//  std::cout << "\nStarting Spherical Test Case 02\n\n";
//  //Case 2 - Spherical Joint at (1,2,3), and rotated to align the z axis with line Y=Z
//  //  Note the spherical joint only allows 3 DOF(all 3 rotations)
//  //    A joint rotation here does not change the kinematics, its just for test purposes
//  TestSpherical(ChVector<> (1, 2, 3), ChQuaternion<> (Q_from_AngX(-CH_C_PI_4)), .001, "SphericalJointData_Case02.txt",animate);
//
//
//  // Return 0 if all test passed and 1 otherwise
//  // -------------------------------------------
//
//  return !test_passed;
//}
