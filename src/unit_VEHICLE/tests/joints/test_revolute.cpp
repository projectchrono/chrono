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
// Test for revolute joint
//
// Recall that Irrlicht uses a left-hand frame, so everything is rendered with
// left and right flipped.
//
// =============================================================================
// TO DO:
//    Report test run time & test pass/fail (determine what the criteria is)
// =============================================================================

#include <ostream>
#include <fstream>

#include "core/ChFileutils.h"

#include "physics/ChSystem.h"
#include "physics/ChBody.h"

#include "unit_IRRLICHT/ChIrrApp.h"

#include "ChronoT_config.h"

using namespace chrono;
using namespace irr;


// =============================================================================

void WriteOutput(std::ofstream&                  outf,
                 double                          time,
                 ChSharedPtr<ChBody>             pendulum,
                 ChSharedPtr<ChLinkLockRevolute> revoluteJoint)
{
  // Time elapsed
  outf << time << "\t";

  // Position of the Pendulum's CG in the Global Reference Frame
  ChVector<> jointLoc = revoluteJoint->GetMarker1()->GetAbsCoord().pos;
  ChVector<> position = pendulum->GetPos();
  outf << position.x << "\t" << position.y << "\t" << position.z << "\t" << (position - jointLoc).Length() << "\t";

  // Velocity of the Pendulum's CG in the Global Reference Frame
  ChVector<> velocity = pendulum->GetPos_dt();
  outf << velocity.x << "\t" << velocity.y << "\t" << velocity.z << "\t" << velocity.Length() << "\t";

  // Acceleration of the Pendulum's CG in the Global Reference Frame
  ChVector<> acceleration = pendulum->GetPos_dtdt();
  outf << acceleration.x << "\t" << acceleration.y << "\t" << acceleration.z << "\t" << acceleration.Length() << "\t";

  // Angular Position quaternion of the Pendulum with respect to the Global Reference Frame
  ChQuaternion<> rot = pendulum->GetRot();
  outf << rot.e0 << "\t" << rot.e1 << "\t" << rot.e2 << "\t" << rot.e3 << "\t";

  // Angular Velocity of the Pendulum with respect to the Global Reference Frame
  ChVector<> angVel = pendulum->GetWvel_par();
  outf << angVel.x << "\t" << angVel.y << "\t" << angVel.z << "\t" << angVel.Length() << "\t";

  // Angular Acceleration of the Pendulum with respect to the Global Reference Frame
  ChVector<> angAccel = pendulum->GetWacc_par();
  outf << angAccel.x << "\t" << angAccel.y << "\t" << angAccel.z << "\t" << angAccel.Length() << "\t";

  // Reaction Force and Torque
  // These are expressed in the link coordinate system. We convert them to
  // the coordinate system of Body2 (in our case this is the ground).
  ChCoordsys<> linkCoordsys = revoluteJoint->GetLinkRelativeCoords();
  ChVector<> reactForce = revoluteJoint->Get_react_force();
  ChVector<> reactForceGlobal = linkCoordsys.TransformDirectionLocalToParent(reactForce);
  outf << reactForceGlobal.x << "\t" << reactForceGlobal.y << "\t" << reactForceGlobal.z << "\t" << reactForceGlobal.Length() << "\t";

  ChVector<> reactTorque = revoluteJoint->Get_react_torque();
  ChVector<> reactTorqueGlobal = linkCoordsys.TransformDirectionLocalToParent(reactTorque);
  outf << reactTorqueGlobal.x << "\t" << reactTorqueGlobal.y << "\t" << reactTorqueGlobal.z << "\t" << reactTorqueGlobal.Length() << "\t";

  // Conservation of Energy
  // Translational Kinetic Energy (1/2*m*||v||^2)
  // Rotational Kinetic Energy (1/2 w'*I*w)  ChMatrix33*vector is valid since [3x3]*[3x1] = [3x1]
  // Delta Potential Energy (m*g*dz)
  double g = pendulum->GetSystem()->Get_G_acc().z;
  double mass = pendulum->GetMass();
  ChMatrix33<> inertia = pendulum->GetInertia(); //3x3 Inertia Tensor in the local coordinate frame
  ChVector<> angVelLoc = pendulum->GetWvel_loc();
  double transKE = 0.5 * mass * velocity.Length2();
  double rotKE = 0.5 * Vdot(angVelLoc, inertia * angVelLoc);
  double deltaPE = mass * g * (position.z - jointLoc.z);
  double totalKE = transKE + rotKE;
  outf << totalKE << "\t" << transKE << "\t" << rotKE << "\t" << deltaPE << "\t" << std::endl;;
}

// =============================================================================

void TestRevolute(const ChVector<>&     jointLoc,         // absolute location of joint
                  const ChQuaternion<>& jointRot,         // orientation of joint
                  double                simTimeStep,      // simulation time step
                  double                outTimeStep,      // output time step
                  const std::string&    outputFilename,   // output file name
                  bool                  animate)          // if true, animate with Irrlich
{

  //Settings
  //----------------------------------------------------------------------------
  // There are no units in Chrono, so values must be consistent
  // (MKS is used in this example)

  double mass = 1.0;              // mass of pendulum
  double length = 4.0;            // length of pendulum
  ChVector<> inertiaXX(1, 1, 1);  // mass moments of inertia of pendulum (centroidal frame)
  double g = 9.80665;

  double timeRecord = 5;          // Stop recording to the file after this much simulated time

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

  // Create the ground body

  ChSharedBodyPtr  ground(new ChBody);
  my_system.AddBody(ground);
  ground->SetBodyFixed(true);
  // Add some geometry to the ground body for visualizing the revolute joint
  ChSharedPtr<ChCylinderShape> cyl_g(new ChCylinderShape);
  cyl_g->GetCylinderGeometry().p1 = jointLoc + jointRot.Rotate(ChVector<>(0, 0, -0.2));
  cyl_g->GetCylinderGeometry().p2 = jointLoc + jointRot.Rotate(ChVector<>(0, 0, 0.2));
  cyl_g->GetCylinderGeometry().rad = 0.1;
  ground->AddAsset(cyl_g);

  // Create the pendulum body, in an initial configuration at rest, aligned with
  // the global X axis. The pendulum CG is assumed to be at half its length.

  ChSharedBodyPtr  pendulum(new ChBody);
  my_system.AddBody(pendulum);
  pendulum->SetPos(jointLoc + ChVector<>(length / 2, 0, 0));
  pendulum->SetMass(mass);
  pendulum->SetInertiaXX(inertiaXX);
  // Add some geometry to the pendulum for visualization
  ChSharedPtr<ChCylinderShape> cyl_p(new ChCylinderShape);
  cyl_p->GetCylinderGeometry().p1 = ChVector<>(-length / 2, 0, 0);
  cyl_p->GetCylinderGeometry().p2 = ChVector<>(length / 2, 0, 0);
  cyl_p->GetCylinderGeometry().rad = 0.1;
  pendulum->AddAsset(cyl_p);

  // Create revolute joint between pendulum and ground at "loc" in the global
  // reference frame. The revolute joint's axis of rotation will be the Z axis
  // of the specified rotation matrix.

  ChSharedPtr<ChLinkLockRevolute>  revoluteJoint(new ChLinkLockRevolute);
  revoluteJoint->Initialize(pendulum, ground, ChCoordsys<>(jointLoc, jointRot));
  my_system.AddLink(revoluteJoint);

  // Perform the simulation
  // ----------------------

  if (animate)     //  ----  IRRLICHT ANIMATION
  {
    // Create the Irrlicht application for visualization
    ChIrrApp * application = new ChIrrApp(&my_system, L"ChLinkRevolute demo", core::dimension2d<u32>(800, 600), false, true);
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
  }
  else             //  ----  RECORD SIMULATION RESULTS
  {
    // Create output file for results
    std::ofstream outf(outputFilename.c_str());

    if (!outf) {
      std::cout << "Output file is invalid" << std::endl;
      return;
    }

    // Write column headers (tab delimitated)
    outf << "timeElapsed(s)\t";
    outf << "X_Pos(m)\tY_Pos(m)\tZ_Pos\tLength_Pos(m)\t";
    outf << "X_Vel(m/s)\tY_Vel(m/s)\tZ_Vel(m/s)\tLength_Vel(m/s)\t";
    outf << "X_Accel(m/s^2)\tY_Accel(m/s^2)\tZ_Accell(m/s^2)\tLength_Accel(m/s^2)\t";
    outf << "e0_quaternion\te1_quaternion\te2_quaternion\te3_quaternion\t";
    outf << "X_AngVel(rad/s)\tY_AngVel(rad/s)\tZ_AngVel(rad/s)\tLength_AngVel(rad/s)\t";
    outf << "X_AngAccel(rad/s^2)\tY_AngAccel(rad/s^2)\tZ_AngAccell(rad/s^2)\tLength_AngAccel(rad/s^2)\t";
    outf << "X_Glb_ReactionFrc(N)\tY_Glb_ReactionFrc(N)\tZ_Glb_ReactionFrc(N)\tLength_Glb_ReactionFrc(N)\t";
    outf << "X_Glb_ReactionTrq(Nm)\tY_Glb_ReactionTrq(Nm)\tZ_Glb_ReactionTrq(Nm)\tLength_Glb_ReactionTrq(Nm)\t";
    outf << "Total_Kinetic_Energy(J)\tTranslational_Kinetic_Energy(J)\tAngular_Kinetic_Energy(J)\tDelta_Potential_Energy(J)\t";
    outf << std::endl;

    // Simulation loop
    // ---------------

    double timeElapsed = 0;
    double lastPrint = -outTimeStep;
    bool continueSimulation = true;

    while (timeElapsed <= timeRecord + simTimeStep / 2)
    {
      // Write current translational and rotational position, velocity, and
      // acceleration, as well as the reaction force and reaction torque in the
      // revolute joint.

      //Add a little error tolerance on the end time to ensure that the final data point is recorded
      if ((timeElapsed <= timeRecord + simTimeStep / 2) && (timeElapsed + simTimeStep / 2 >= lastPrint + outTimeStep))
      {
        lastPrint = lastPrint + outTimeStep;
        WriteOutput(outf, timeElapsed, pendulum, revoluteJoint);
      }

      // Advance simulation by one step
      my_system.DoStepDynamics(simTimeStep);

      // Increment time
      timeElapsed += simTimeStep;
    }

    // Close output file
    outf.close();
  }

}

// =============================================================================

int main(int argc, char* argv[])
{
  bool animate = (argc > 1);

  // Set the path to the Chrono data folder
  SetChronoDataPath(CHRONO_DATA_DIR);

  // Create output directory (if it does not already exist)
  std::string out_dir = "../VALIDATION";

  if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
    std::cout << "Error creating directory " << out_dir << std::endl;
    return 1;
  }

  std::string filename;

  //Case 1 - Revolute Joint at the origin, and aligned with the global Y axis
  //  Note the revolute joint only allows 1 DOF(rotation about joint z axis)
  //    Therefore, the joint must be rotated -pi/2 about the global x-axis
  std::cout << "\nStarting Revolute Test Case 01\n\n";
  filename = out_dir + "/RevoluteJointData_Case01.txt";
  TestRevolute(ChVector<>(0, 0, 0), Q_from_AngX(-CH_C_PI_2), 1e-3, 1e-2, filename, animate);

  //Case 2 - Revolute Joint at (1,2,3), and aligned with the global axis along Y = Z
  //  Note the revolute joint only allows 1 DOF(rotation about joint z axis)
  //    Therefore, the joint must be rotated -pi/4 about the global x-axis
  std::cout << "\nStarting Revolute Test Case 02\n\n";
  filename = out_dir + "/RevoluteJointData_Case02.txt";
  TestRevolute(ChVector<>(1, 2, 3), Q_from_AngX(-CH_C_PI_4), 1e-3, 1e-2, filename, animate);

  return 0;
}
