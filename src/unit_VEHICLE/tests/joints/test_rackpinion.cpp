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
// Test for the rack and pinion joint
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
#include "physics/ChLinkRackpinion.h"

#include "unit_IRRLICHT/ChIrrApp.h"

#include "ChronoT_config.h"
#include "utils/ChUtilsInputOutput.h"
#include "utils/ChUtilsValidation.h"

using namespace chrono;
using namespace irr;


// =============================================================================
// Local variables
//
static const std::string val_dir = "../VALIDATION/";
static const std::string out_dir = val_dir + "rackpinion_joint/";
static const std::string ref_dir = "rackpinion_joint/";

// =============================================================================
// Prototypes of local functions
//
bool TestRackPinion(const ChVector<>& jointLoc, const ChQuaternion<>& jointRot,
                  double simTimeStep, double outTimeStep,
                  const std::string& testName, bool animate, bool save);
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
  bool save = (argc > 2);

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

  // Case 1 - Joint at the origin and aligned with the global Frame.
  // Pendulum Falls due to gravity.

  test_name = "RackPinion_Case01";
  TestRackPinion(ChVector<>(0, 0, 0), QUNIT, sim_step, out_step, test_name, animate, save);
  if (!animate) {
    //test_passed &= ValidateReference(test_name, "Pinion_Pos", 2e-3);
    //test_passed &= ValidateReference(test_name, "Pinion_Vel", 1e-4);
    //test_passed &= ValidateReference(test_name, "Pinion_Acc", 2e-2);
    //test_passed &= ValidateReference(test_name, "Pinion_Quat", 1e-3);
    //test_passed &= ValidateReference(test_name, "Pinion_Avel", 2e-2);
    //test_passed &= ValidateReference(test_name, "Pinion_Aacc", 2e-2);
    //test_passed &= ValidateReference(test_name, "Rack_Pos", 2e-3);
    //test_passed &= ValidateReference(test_name, "Rack_Vel", 1e-4);
    //test_passed &= ValidateReference(test_name, "Rack_Acc", 2e-2);
    //test_passed &= ValidateReference(test_name, "Rack_Quat", 1e-3);
    //test_passed &= ValidateReference(test_name, "Rack_Avel", 2e-2);
    //test_passed &= ValidateReference(test_name, "Rack_Aacc", 2e-2);
    test_passed &= ValidateEnergy(test_name, 1e-2);
  }




  // Return 0 if all tests passed and 1 otherwise
  return !test_passed;
}

// =============================================================================
//
// Worker function for performing the simulation with specified parameters.
//
bool TestRackPinion(const ChVector<>&     jointLoc,         // absolute location of joint
                  const ChQuaternion<>& jointRot,         // orientation of joint
                  double                simTimeStep,      // simulation time step
                  double                outTimeStep,      // output time step
                  const std::string&    testName,         // name of this test
                  bool                  animate,          // if true, animate with Irrlich
                  bool                  save)             // if true, also save animation data
{
  std::cout << "TEST: " << testName << std::endl;

  // Settings
  //---------

  // There are no units in Chrono, so values must be consistent
  // (MKS is used in this example)

  double massPinion = 1.0;                     // mass of pinion
  double radiusPinion = 0.1;                   // radius of pinion
  ChVector<> inertiaXX_Pinion(0.1, 0.1, 0.04);  // mass moments of inertia of pinion (centroidal frame)

  double massRack = 1.0;                     // mass of pendulum
  double lengthRack = 8.0;                   // length of pendulum
  ChVector<> inertiaXX_Rack(0.1, 0.1, 0.04);  // mass moments of inertia of pendulum (centroidal frame)
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
  // Add some geometry to the ground body for visualizing the pinion revolute joint
  ChSharedPtr<ChCylinderShape> cyl_g(new ChCylinderShape);
  cyl_g->GetCylinderGeometry().p1 = jointLoc + Q_from_AngY(CH_C_PI_2).Rotate(ChVector<>(0, 0, -0.4));
  cyl_g->GetCylinderGeometry().p2 = jointLoc + Q_from_AngY(CH_C_PI_2).Rotate(ChVector<>(0, 0, 0.4));
  cyl_g->GetCylinderGeometry().rad = 0.05;
  ground->AddAsset(cyl_g);


  // Create the pinion body in an initial configuration at rest
  ChSharedBodyPtr  pinion(new ChBody);
  my_system.AddBody(pinion);
  pinion->SetPos(jointLoc);
  pinion->SetRot(Q_from_AngY(CH_C_PI_2));
  pinion->SetMass(massPinion);
  pinion->SetInertiaXX(inertiaXX_Pinion);
  ChSharedPtr<ChCylinderShape> cyl_p(new ChCylinderShape);
  cyl_p->GetCylinderGeometry().p1 = ChVector<>(0, 0, -0.2);
  cyl_p->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0.2);
  cyl_p->GetCylinderGeometry().rad = 0.1;
  pinion->AddAsset(cyl_p);
  ChSharedPtr<ChBoxShape> box_p(new ChBoxShape);
  box_p->GetBoxGeometry().Size = ChVector<>(0.08,0.8,0.2);
  pinion->AddAsset(box_p);
  ChSharedPtr<ChColorAsset> col_p(new ChColorAsset);
  col_p->SetColor(ChColor(0.6f, 0.2f, 0.2f));
  pinion->AddAsset(col_p);

  // Create the rack body in an initial configuration at rest
  ChSharedBodyPtr  rack(new ChBody);
  my_system.AddBody(rack);
  rack->SetPos(jointLoc+ChVector<>(0, radiusPinion, 0));
  rack->SetRot(QUNIT);
  rack->SetMass(massRack);
  rack->SetInertiaXX(inertiaXX_Rack);
  ChSharedPtr<ChBoxShape> box_r(new ChBoxShape);
  box_r->GetBoxGeometry().Size = ChVector<>(0.05 * lengthRack, 0.01 * lengthRack, 0.5 * lengthRack);
  rack->AddAsset(box_r);

    // Create revolute joint between pinion and ground at "loc" in the global
  // reference frame. The revolute joint's axis of rotation (Z) will be the 
  // global x axis.
  ChSharedPtr<ChLinkLockRevolute>  revoluteJoint(new ChLinkLockRevolute);
  revoluteJoint->Initialize(pinion, ground, ChCoordsys<>(jointLoc, Q_from_AngY(CH_C_PI_2)));
  my_system.AddLink(revoluteJoint);

  // Create prismatic joint between rack and ground at "loc" - Pinion Radius in the global
  // reference frame. The prismatic joint's axis of translation will be the Z axis
  // of the specified rotation matrix.
  ChSharedPtr<ChLinkLockPrismatic>  prismaticJoint(new ChLinkLockPrismatic);
  prismaticJoint->Initialize(rack, ground, ChCoordsys<>(jointLoc+ChVector<>(0, -radiusPinion, 0), QUNIT));
  my_system.AddLink(prismaticJoint);

  //Create the Rack and Pinion joint
  ChSharedPtr<ChLinkRackpinion>  rackpinionJoint(new ChLinkRackpinion);
  rackpinionJoint->Initialize(pinion,rack,false,ChFrame<>(jointLoc, Q_from_AngY(-CH_C_PI_2)),ChFrame<>(jointLoc+ChVector<>(0, 0, 0), Q_from_AngY(-CH_C_PI_2)));
  rackpinionJoint->SetPinionRadius(-radiusPinion);
  rackpinionJoint->SetAlpha(CH_C_PI_4);
  rackpinionJoint->SetBeta(0);
  rackpinionJoint->SetCheckphase(1);
  my_system.AddLink(rackpinionJoint);


  // Perform the simulation (animation with Irrlicht option)
  // -------------------------------------------------------

  if (animate)
  {
    // Create the Irrlicht application for visualization
    ChIrrApp * application = new ChIrrApp(&my_system, L"ChLinkRackpinion demo", core::dimension2d<u32>(800, 600), false, true);
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
    double outTime = 0;
    int    outFrame = 1;

    std::string pov_dir = out_dir + "POVRAY_" + testName;
    if (ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
      std::cout << "Error creating directory " << pov_dir << std::endl;
      return false;
    }

    while (application->GetDevice()->run())
    {
      if (save && my_system.GetChTime() >= outTime - simTimeStep / 2) {
        char filename[100];
        sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), outFrame);
        utils::WriteShapesPovray(&my_system, filename);
        outTime += outTimeStep;
        outFrame++;
      }

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
  utils::CSV_writer out_posPinion = OutStream();
  utils::CSV_writer out_velPinion = OutStream();
  utils::CSV_writer out_accPinion = OutStream();

  utils::CSV_writer out_quatPinion = OutStream();
  utils::CSV_writer out_avelPinion = OutStream();
  utils::CSV_writer out_aaccPinion = OutStream();

  utils::CSV_writer out_posRack = OutStream();
  utils::CSV_writer out_velRack = OutStream();
  utils::CSV_writer out_accRack = OutStream();

  utils::CSV_writer out_quatRack = OutStream();
  utils::CSV_writer out_avelRack = OutStream();
  utils::CSV_writer out_aaccRack = OutStream();

  utils::CSV_writer out_energy = OutStream();

  // Write headers
  out_posPinion << "Time" << "X_Pos" << "Y_Pos" << "Z_Pos" << std::endl;
  out_velPinion << "Time" << "X_Vel" << "Y_Vel" << "Z_Vel" << std::endl;
  out_accPinion << "Time" << "X_Acc" << "Y_Acc" << "Z_Acc" << std::endl;

  out_quatPinion << "Time" << "e0" << "e1" << "e2" << "e3" << std::endl;
  out_avelPinion << "Time" << "X_AngVel" << "Y_AngVel" << "Z_AngVel" << std::endl;
  out_aaccPinion << "Time" << "X_AngAcc" << "Y_AngAcc" << "Z_AngAcc" << std::endl;

  out_posRack << "Time" << "X_Pos" << "Y_Pos" << "Z_Pos" << std::endl;
  out_velRack << "Time" << "X_Vel" << "Y_Vel" << "Z_Vel" << std::endl;
  out_accRack << "Time" << "X_Acc" << "Y_Acc" << "Z_Acc" << std::endl;

  out_quatRack << "Time" << "e0" << "e1" << "e2" << "e3" << std::endl;
  out_avelRack << "Time" << "X_AngVel" << "Y_AngVel" << "Z_AngVel" << std::endl;
  out_aaccRack << "Time" << "X_AngAcc" << "Y_AngAcc" << "Z_AngAcc" << std::endl;

  out_energy << "Time" << "Transl_KE" << "Rot_KE" << "Delta_PE" << "KE+PE" << std::endl;

  // Perform a system assembly to ensure we have the correct accelerations at
  // the initial time.
  my_system.DoFullAssembly();

  // Total energy at initial time.
  ChMatrix33<> inertiaPinion = pinion->GetInertia();
  ChVector<> angVelLocPinion = pinion->GetWvel_loc();
  ChMatrix33<> inertiaRack = rack->GetInertia();
  ChVector<> angVelLocRack = rack->GetWvel_loc();
  double transKE = 0.5 * massPinion * pinion->GetPos_dt().Length2() + 0.5 * massRack * rack->GetPos_dt().Length2();
  double rotKE = 0.5 * Vdot(angVelLocPinion, inertiaPinion * angVelLocPinion) + 0.5 * Vdot(angVelLocRack, inertiaRack * angVelLocRack);
  double deltaPE = massPinion * g * (pinion->GetPos().z - jointLoc.z) + massRack * g * (rack->GetPos().z - jointLoc.z);
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
      const ChVector<>& positionPinion = pinion->GetPos();
      const ChVector<>& velocityPinion = pinion->GetPos_dt();
      const ChVector<>& positionRack = rack->GetPos();
      const ChVector<>& velocityRack = rack->GetPos_dt();
      out_posPinion << simTime << positionPinion << std::endl;
      out_velPinion << simTime << velocityPinion << std::endl;
      out_accPinion << simTime << pinion->GetPos_dtdt() << std::endl;

      out_posRack << simTime << positionRack << std::endl;
      out_velRack << simTime << velocityRack << std::endl;
      out_accRack << simTime << rack->GetPos_dtdt() << std::endl;

      // Orientation, angular velocity, and angular acceleration (expressed in
      // global frame).
      out_quatPinion << simTime << pinion->GetRot() << std::endl;
      out_avelPinion << simTime << pinion->GetWvel_par() << std::endl;
      out_aaccPinion << simTime << pinion->GetWacc_par() << std::endl;

      out_quatRack << simTime << rack->GetRot() << std::endl;
      out_avelRack << simTime << rack->GetWvel_par() << std::endl;
      out_aaccRack << simTime << rack->GetWacc_par() << std::endl;


      // Conservation of Energy
      // Translational Kinetic Energy (1/2*m*||v||^2)
      // Rotational Kinetic Energy (1/2 w'*I*w)
      // Delta Potential Energy (m*g*dz)
      ChMatrix33<> inertiaPinion = pinion->GetInertia();
      ChVector<> angVelLocPinion = pinion->GetWvel_loc();
      ChMatrix33<> inertiaRack = rack->GetInertia();
      ChVector<> angVelLocRack = rack->GetWvel_loc();
      double transKE = 0.5 * massPinion * pinion->GetPos_dt().Length2() + 0.5 * massRack * rack->GetPos_dt().Length2();
      double rotKE = 0.5 * Vdot(angVelLocPinion, inertiaPinion * angVelLocPinion) + 0.5 * Vdot(angVelLocRack, inertiaRack * angVelLocRack);
      double deltaPE = massPinion * g * (pinion->GetPos().z - jointLoc.z) + massRack * g * (rack->GetPos().z - jointLoc.z);
      double totalE = transKE + rotKE + deltaPE;
      out_energy << simTime << transKE << rotKE << deltaPE << totalE - totalE0 << std::endl;;

      // Increment output time
      outTime += outTimeStep;
    }

    // Advance simulation by one step
    my_system.DoStepDynamics(simTimeStep);

    // Increment simulation time
    simTime += simTimeStep;
  }

  // Write output files
  out_posPinion.write_to_file(out_dir + testName + "_CHRONO_Pinion_Pos.txt", testName + "\n\n");
  out_velPinion.write_to_file(out_dir + testName + "_CHRONO_Pinion_Vel.txt", testName + "\n\n");
  out_accPinion.write_to_file(out_dir + testName + "_CHRONO_Pinion_Acc.txt", testName + "\n\n");

  out_posRack.write_to_file(out_dir + testName + "_CHRONO_Rack_Pos.txt", testName + "\n\n");
  out_velRack.write_to_file(out_dir + testName + "_CHRONO_Rack_Vel.txt", testName + "\n\n");
  out_accRack.write_to_file(out_dir + testName + "_CHRONO_Rack_Acc.txt", testName + "\n\n");

  out_quatPinion.write_to_file(out_dir + testName + "_CHRONO_Pinion_Quat.txt", testName + "\n\n");
  out_avelPinion.write_to_file(out_dir + testName + "_CHRONO_Pinion_Avel.txt", testName + "\n\n");
  out_aaccPinion.write_to_file(out_dir + testName + "_CHRONO_Pinion_Aacc.txt", testName + "\n\n");

  out_quatRack.write_to_file(out_dir + testName + "_CHRONO_Rack_Quat.txt", testName + "\n\n");
  out_avelRack.write_to_file(out_dir + testName + "_CHRONO_Rack_Avel.txt", testName + "\n\n");
  out_aaccRack.write_to_file(out_dir + testName + "_CHRONO_Rack_Aacc.txt", testName + "\n\n");

  out_energy.write_to_file(out_dir + testName + "_CHRONO_Energy.txt", testName + "\n\n");

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

  bool check = utils::Validate(sim_file, utils::GetValidationDataFile(ref_file), utils::RMS_NORM, tolerance, norms);
  std::cout << "   validate " << what << (check ? ": Passed" : ": Failed") << "  [  ";
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
