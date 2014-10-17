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
static const std::string out_dir = val_dir + "LIN_ACTUATOR/";
static const std::string ref_dir = "validation/lin_actuator/";

// =============================================================================
// Prototypes of local functions
//
bool TestLinActuator(const ChQuaternion<>& rot, double desiredSpeed,
                     double simTimeStep, double outTimeStep,
                     const std::string& testName, bool animate);
bool ValidateReference(const std::string& testName, const std::string& what, double tolerance);
bool ValidateConstraints(const std::string& testName, const std::string& what, double tolerance);
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
  double sim_step = 1e-3;
  double out_step = 1e-2;

  std::string test_name;
  bool test_passed = true;

  // Case 1 - Translation axis vertical, imposed speed 0.1 m/s

  test_name = "LinActuator_Case01";
  TestLinActuator(QUNIT, 0.1, sim_step, out_step, test_name, animate);
  if (!animate) {
    ////test_passed &= ValidateReference(test_name, "Pos", 2e-3);
    ////test_passed &= ValidateReference(test_name, "Vel", 1e-3);
    ////test_passed &= ValidateReference(test_name, "Acc", 2e-2);
    ////test_passed &= ValidateReference(test_name, "Quat", 1e-3);
    ////test_passed &= ValidateReference(test_name, "Avel", 2e-2);
    ////test_passed &= ValidateReference(test_name, "Aacc", 2e-2);
    ////test_passed &= ValidateReference(test_name, "Rforce", 2e-2);
    ////test_passed &= ValidateReference(test_name, "Rtorque", 1e-10);
    ////test_passed &= ValidateReference(test_name, "Energy", 2e-2);
    test_passed &= ValidateConstraints(test_name, "ConstraintsP", 1e-5);
    test_passed &= ValidateConstraints(test_name, "ConstraintsA", 1e-5);
  }

  // Case 2 - Translation axis along X = Z, imposed speed 0.2 m/s

  test_name = "LinActuator_Case02";
  TestLinActuator(Q_from_AngY(CH_C_PI / 4), 0.2, sim_step, out_step, test_name, animate);
  if (!animate) {
    ////test_passed &= ValidateReference(test_name, "Pos", 2e-3);
    ////test_passed &= ValidateReference(test_name, "Vel", 1e-3);
    ////test_passed &= ValidateReference(test_name, "Acc", 2e-2);
    ////test_passed &= ValidateReference(test_name, "Quat", 1e-3);
    ////test_passed &= ValidateReference(test_name, "Avel", 2e-2);
    ////test_passed &= ValidateReference(test_name, "Aacc", 2e-2);
    ////test_passed &= ValidateReference(test_name, "Rforce", 2e-2);
    ////test_passed &= ValidateReference(test_name, "Rtorque", 1e-10);
    ////test_passed &= ValidateReference(test_name, "Energy", 2e-2);
    test_passed &= ValidateConstraints(test_name, "ConstraintsP", 1e-5);
    test_passed &= ValidateConstraints(test_name, "ConstraintsA", 1e-5);
  }


  // Return 0 if all tests passed and 1 otherwise
  return !test_passed;
}

// =============================================================================
//
// Worker function for performing the simulation with specified parameters.
//
bool TestLinActuator(const ChQuaternion<>& rot,              // translation along Z axis
                     double                desiredSpeed,     // imposed translation speed
                     double                simTimeStep,      // simulation time step
                     double                outTimeStep,      // output time step
                     const std::string&    testName,         // name of this test
                     bool                  animate)          // if true, animate with Irrlich
{
  std::cout << "TEST: " << testName << std::endl;

  // Unit vector along translation axis, expressed in global frame
  ChVector<> axis = rot.GetZaxis();

  // Settings
  //---------

  double mass = 1.0;              // mass of plate
  ChVector<> inertiaXX(1, 1, 1);  // mass moments of inertia of plate (centroidal frame)
  double g = 9.80665;

  double timeRecord = 5;          // Stop recording to the file after this much simulated time

  // Create the mechanical system
  // ----------------------------

  ChSystem my_system;
  my_system.Set_G_acc(ChVector<>(0.0, 0.0, -g));

  my_system.SetIntegrationType(ChSystem::INT_ANITESCU);
  my_system.SetIterLCPmaxItersSpeed(100);
  my_system.SetIterLCPmaxItersStab(100); //Tasora stepper uses this, Anitescu does not
  my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);

  // Create the ground body.

  ChSharedBodyPtr  ground(new ChBody);
  my_system.AddBody(ground);
  ground->SetBodyFixed(true);

  // Add geometry to the ground body for visualizing the translational joint
  ChSharedPtr<ChBoxShape> box_g(new ChBoxShape);
  box_g->GetBoxGeometry().SetLengths(ChVector<>(0.1, 0.1, 5));
  box_g->GetBoxGeometry().Pos = 2.5 * axis;
  box_g->GetBoxGeometry().Rot = rot;
  ground->AddAsset(box_g);

  ChSharedPtr<ChColorAsset> col_g(new ChColorAsset);
  col_g->SetColor(ChColor(0.6f, 0.2f, 0.2f));
  ground->AddAsset(col_g);

  // Create the plate body.

  ChSharedBodyPtr  plate(new ChBody);
  my_system.AddBody(plate);
  plate->SetPos(ChVector<>(0, 0, 0));
  plate->SetRot(rot);
  plate->SetPos_dt(desiredSpeed * axis);
  plate->SetMass(mass);
  plate->SetInertiaXX(inertiaXX);

  // Add geometry to the plate for visualization
  ChSharedPtr<ChBoxShape> box_p(new ChBoxShape);
  box_p->GetBoxGeometry().SetLengths(ChVector<>(1, 1, 0.2));
  plate->AddAsset(box_p);

  ChSharedPtr<ChColorAsset> col_p(new ChColorAsset);
  col_p->SetColor(ChColor(0.2f, 0.2f, 0.6f));
  plate->AddAsset(col_p);

  // Create prismatic (translational) joint between plate and ground.
  // We set the ground as the "master" body (second one in the initialization
  // call) so that the link coordinate system is expressed in the ground frame.

  ChSharedPtr<ChLinkLockPrismatic> prismatic(new ChLinkLockPrismatic);
  prismatic->Initialize(plate, ground, ChCoordsys<>(ChVector<>(0, 0, 0), rot));
  my_system.AddLink(prismatic);

  // Create a ramp function to impose constant speed.  This function returns
  //   y(t) = 0 + t * desiredSpeed
  //   y'(t) = desiredSpeed

  ChSharedPtr<ChFunction_Ramp> actuator_fun(new ChFunction_Ramp(0.0, desiredSpeed));

  // Create the linear actuator, connecting the plate to the ground.
  // Here, we set the plate as the master body (second one in the initialization
  // call) so that the link coordinate system is expressed in the plate body
  // frame.

  ChSharedPtr<ChLinkLinActuator> actuator(new ChLinkLinActuator);
  ChVector<> pt1 = ChVector<>(0, 0, 0);
  ChVector<> pt2 = axis;
  actuator->Initialize(ground, plate, false, ChCoordsys<>(pt1, rot), ChCoordsys<>(pt2, rot));
  actuator->Set_lin_offset(1);
  actuator->Set_dist_funct(actuator_fun);
  my_system.AddLink(actuator);

  // Perform the simulation (animation with Irrlicht)
  // ------------------------------------------------

  if (animate)
  {
    // Create the Irrlicht application for visualization
    ChIrrApp application(&my_system, L"ChLinkRevolute demo", core::dimension2d<u32>(800, 600), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(1, 2, -2), core::vector3df(0, 0, 2));

    application.AssetBindAll();
    application.AssetUpdateAll();

    application.SetStepManage(true);
    application.SetTimestep(simTimeStep);

    // Simulation loop
    while (application.GetDevice()->run())
    {
      application.BeginScene();
      application.DrawAll();

      // Draw an XZ grid at the global origin to add in visualization
      ChIrrTools::drawGrid(
        application.GetVideoDriver(), 1, 1, 20, 20,
        ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)),
        video::SColor(255, 80, 100, 100), true);

      application.DoStep();  //Take one step in time
      application.EndScene();
    }

    return true;
  }

  // Perform the simulation (record results)
  // ------------------------------------------------

  // Create the CSV_Writer output objects (TAB delimited)
  utils::CSV_writer out_pos = OutStream();
  utils::CSV_writer out_vel = OutStream();
  utils::CSV_writer out_acc = OutStream();

  utils::CSV_writer out_quat = OutStream();
  utils::CSV_writer out_avel = OutStream();
  utils::CSV_writer out_aacc = OutStream();

  utils::CSV_writer out_rfrcP = OutStream();
  utils::CSV_writer out_rtrqP = OutStream();

  utils::CSV_writer out_rfrcA = OutStream();
  utils::CSV_writer out_rtrqA = OutStream();

  utils::CSV_writer out_cnstrP = OutStream();

  utils::CSV_writer out_cnstrA = OutStream();

  // Write headers
  out_pos << "Time" << "X_Pos" << "Y_Pos" << "Z_Pos" << std::endl;
  out_vel << "Time" << "X_Vel" << "Y_Vel" << "Z_Vel" << std::endl;
  out_acc << "Time" << "X_Acc" << "Y_Acc" << "Z_Acc" << std::endl;

  out_quat << "Time" << "e0" << "e1" << "e2" << "e3" << std::endl;
  out_avel << "Time" << "X_AngVel" << "Y_AngVel" << "Z_AngVel" << std::endl;
  out_aacc << "Time" << "X_AngAcc" << "Y_AngAcc" << "Z_AngAcc" << std::endl;

  out_rfrcP << "Time" << "X_Force" << "Y_Force" << "Z_Force" << std::endl;
  out_rtrqP << "Time" << "X_Torque" << "Y_Torque" << "Z_Torque" << std::endl;

  out_rfrcA << "Time" << "X_Force" << "Y_Force" << "Z_Force" << std::endl;
  out_rtrqA << "Time" << "X_Torque" << "Y_Torque" << "Z_Torque" << std::endl;

  out_cnstrP << "Time" << "Cnstr_1" << "Cnstr_2" << "Cnstr_3" << "Constraint_4" << "Cnstr_5" << std::endl;

  out_cnstrA << "Time" << "Cnstr_1" << std::endl;

  // Simulation loop
  double simTime = 0;
  double outTime = 0;

  while (simTime <= timeRecord + simTimeStep / 2)
  {
    // Ensure that the final data point is recorded.
    if (simTime >= outTime - simTimeStep / 2)
    {

      // CM position, velocity, and acceleration (expressed in global frame).
      const ChVector<>& position = plate->GetPos();
      const ChVector<>& velocity = plate->GetPos_dt();
      out_pos << simTime << position << std::endl;
      out_vel << simTime << velocity << std::endl;
      out_acc << simTime << plate->GetPos_dtdt() << std::endl;

      // Orientation, angular velocity, and angular acceleration (expressed in
      // global frame).
      out_quat << simTime << plate->GetRot() << std::endl;
      out_avel << simTime << plate->GetWvel_par() << std::endl;
      out_aacc << simTime << plate->GetWacc_par() << std::endl;

      // Reaction Force and Torque in prismatic joint.
      // These are expressed in the link coordinate system. We convert them to
      // the coordinate system of Body2 (in our case this is the ground).
      ChCoordsys<> linkCoordsysP = prismatic->GetLinkRelativeCoords();
      ChVector<> reactForceP = prismatic->Get_react_force();
      ChVector<> reactForceGlobalP = linkCoordsysP.TransformDirectionLocalToParent(reactForceP);
      out_rfrcP << simTime << reactForceGlobalP << std::endl;

      ChVector<> reactTorqueP = prismatic->Get_react_torque();
      ChVector<> reactTorqueGlobalP = linkCoordsysP.TransformDirectionLocalToParent(reactTorqueP);
      out_rtrqP << simTime << reactTorqueGlobalP << std::endl;

      // Reaction force and Torque in linear actuator.
      // These are expressed  in the link coordinate system. We convert them to
      // the coordinate system of Body2 (in our case this is the plate). As such,
      // the reaction force represents the force that needs to be applied to the
      // plate in order to maintain the prescribed constant velocity.
      ChCoordsys<> linkCoordsysA = actuator->GetLinkRelativeCoords();
      ChVector<> reactForceA = actuator->Get_react_force();
      ChVector<> reactForceGlobalA = linkCoordsysA.TransformDirectionLocalToParent(reactForceA);
      out_rfrcA << simTime << reactForceGlobalA << std::endl;

      ChVector<> reactTorqueA = actuator->Get_react_torque();
      ChVector<> reactTorqueGlobalA = linkCoordsysA.TransformDirectionLocalToParent(reactTorqueA);
      out_rtrqA << simTime << reactTorqueGlobalA << std::endl;

      // Constraint violations in prismatic joint
      ChMatrix<>* CP = prismatic->GetC();
      out_cnstrP << simTime
                 << CP->GetElement(0, 0)
                 << CP->GetElement(1, 0)
                 << CP->GetElement(2, 0)
                 << CP->GetElement(3, 0)
                 << CP->GetElement(4, 0) << std::endl;

      // Constraint violations in linear actuator
      ChMatrix<>* CA = actuator->GetC();
      out_cnstrA << simTime << CA->GetElement(0, 0) << std::endl;

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

  out_rfrcP.write_to_file(out_dir + testName + "_CHRONO_RforceP.txt", testName + "\n\n");
  out_rtrqP.write_to_file(out_dir + testName + "_CHRONO_RtorqueP.txt", testName + "\n\n");

  out_rfrcA.write_to_file(out_dir + testName + "_CHRONO_RforceA.txt", testName + "\n\n");
  out_rtrqA.write_to_file(out_dir + testName + "_CHRONO_RtorqueA.txt", testName + "\n\n");

  out_cnstrP.write_to_file(out_dir + testName + "_CHRONO_ConstraintsP.txt", testName + "\n\n");

  out_cnstrA.write_to_file(out_dir + testName + "_CHRONO_ConstraintsA.txt", testName + "\n\n");

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
  std::string& sim_file = out_dir + testName + "_CHRONO_" + what + ".txt";
  std::string& ref_file = ref_dir + testName + "_ADAMS_" + what + ".txt";
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
                         const std::string& what,      // identifier for test quantity
                         double             tolerance) // validation tolerance
{
  std::string& sim_file = out_dir + testName + "_CHRONO_" + what + ".txt";
  utils::DataVector norms;

  bool check = utils::Validate(sim_file, utils::RMS_NORM, tolerance, norms);
  std::cout << "   validate Constraints" << (check ? ": Passed" : ": Failed") << "  [  ";
  for (size_t col = 0; col < norms.size(); col++)
    std::cout << norms[col] << "  ";
  std::cout << "  ]" << std::endl;

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
