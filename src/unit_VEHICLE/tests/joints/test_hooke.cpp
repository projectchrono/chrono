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
// Test for hooke joint
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

#include "physics/ChSystem.h"
#include "physics/ChBody.h"

#include "unit_IRRLICHT/ChIrrApp.h"

#include "ChronoT_config.h"

using namespace chrono;
using namespace irr;


// =============================================================================

void TestHooke(ChVector<> loc, ChQuaternion<> revAxisRot, double simTimeStep, std::string outputFilename, bool animate)
{

  //Settings
  //----------------------------------------------------------------------------
  // There are no units in Chrono, so values must be consistant (MKS is used in this example)

  double mass = 1.0;                // mass of pendulum
  double length = 4.0;              // length of pendulum
  ChVector<> inertiaXX(1, 1, 1);    // mass moments of inertia of pendulum
  double g = 9.80665;

  double timeRecord = 5;            // Stop recording to the file after this much simulated time
  double printTimeStep = 0.001;     // Write the output file at this simulation time step

  SetChronoDataPath(CHRONO_DATA_DIR);


  // Create the mechanical system
  // ----------------------------

  // 1- Create a ChronoENGINE physical system: all bodies and constraints
  //    will be handled by this ChSystem object.

  ChSystem my_system;
  my_system.Set_G_acc(ChVector<>(0.0, 0.0, -g));

  my_system.SetIntegrationType(ChSystem::INT_ANITESCU);
  my_system.SetIterLCPmaxItersSpeed(100);
  my_system.SetIterLCPmaxItersStab(100); //Tasora stepper uses this, Anitescu does not
  my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);

  // 2- Create the rigid bodies of the system

  // ..the ground
  ChSharedBodyPtr  ground(new ChBody);
  my_system.AddBody(ground);
  ground->SetBodyFixed(true);
  // Add some geometry to the ground body for visualizing the revolute joint
  ChSharedPtr<ChCylinderShape> cyl1_g(new ChCylinderShape);
  cyl1_g->GetCylinderGeometry().p1 = loc + revAxisRot.Rotate(ChVector<>(-0.2, 0, 0));
  cyl1_g->GetCylinderGeometry().p2 = loc + revAxisRot.Rotate(ChVector<>(0.2, 0, 0));
  cyl1_g->GetCylinderGeometry().rad = 0.1;
  ground->AddAsset(cyl1_g);
  ChSharedPtr<ChCylinderShape> cyl2_g(new ChCylinderShape);
  cyl2_g->GetCylinderGeometry().p1 = loc + revAxisRot.Rotate(ChVector<>(0, 0, -0.2));
  cyl2_g->GetCylinderGeometry().p2 = loc + revAxisRot.Rotate(ChVector<>(0, 0, 0.2));
  cyl2_g->GetCylinderGeometry().rad = 0.1;
  ground->AddAsset(cyl2_g);

  // ..the pendulum (Assumes the pendulum's CG is at half its length)
  ChSharedBodyPtr  pendulum(new ChBody);
  my_system.AddBody(pendulum);
  pendulum->SetPos(loc + ChVector<>(length / 2, 0, 0));   // position of COG of pendulum in the Global Reference Frame
  pendulum->SetMass(mass);
  pendulum->SetInertiaXX(inertiaXX);   // Set the body's inertia about the CG in the Global Reference Frame 
  // Add some geometry to the pendulum for visualization
  ChSharedPtr<ChCylinderShape> cyl_p(new ChCylinderShape);
  cyl_p->GetCylinderGeometry().p1 = ChVector<>(-length / 2, 0, 0);
  cyl_p->GetCylinderGeometry().p2 = ChVector<>(length / 2, 0, 0);
  cyl_p->GetCylinderGeometry().rad = 0.1;
  pendulum->AddAsset(cyl_p);

  // 3- Create constraints: the mechanical joints between the rigid bodies.

  // .. a revolute joint between pendulum and ground at "loc" in the global reference frame with the applied rotation
  ChSharedPtr<ChLinkLockHook>  hookeJoint(new ChLinkLockHook);
  hookeJoint->Initialize(pendulum, ground, ChCoordsys<>(loc, revAxisRot));
  my_system.AddLink(hookeJoint);


  // Create the Irrlicht application for visualization
  // -------------------------------------------------
  ChIrrApp * application;
  if(animate){
    application = new ChIrrApp(&my_system, L"ChLinkLockUniversal demo", core::dimension2d<u32>(800, 600), false, true);
    application->AddTypicalLogo();
    application->AddTypicalSky();
    application->AddTypicalLights();
    core::vector3df lookat((f32)loc.x, (f32)loc.y, (f32)loc.z);
    application->AddTypicalCamera(lookat + core::vector3df(0, 3, -6), lookat);

    application->AssetBindAll();     //Now have the visulization tool (Irrlicht) create its geometry from the assets defined above
    application->AssetUpdateAll();

    application->SetTimestep(simTimeStep);
  }

  // Create output file for results & add in column headers (tab deliminated)
  // ------------------------------------------------------------------------
  std::ofstream outf(outputFilename.c_str());
  if (outf) {
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
  }
  else {
    std::cout << "Output file is invalid" << std::endl;
  }


  // Simulation loop
  // ---------------

  double timeElapsed = 0;
  double lastPrint = -printTimeStep;
  bool continueSimulation = true;

  while (continueSimulation)
  {
    // Write current translational and rotational position, velocity, acceleration, 
    // reaction force, and reaction torque of pendulum to output file

    //Add a little error tolerance on the end time to ensure that the final data point is recorded
    if (outf && (timeElapsed <= timeRecord+simTimeStep/2) && (timeElapsed+simTimeStep/2>=lastPrint+printTimeStep)) {
      lastPrint = lastPrint + printTimeStep;

      // Time elapsed
      outf << timeElapsed << "\t";

      // Position of the Pendulum's CG in the Global Reference Frame
      ChVector<double> position = pendulum->GetPos();
      outf << position.x << "\t" << position.y << "\t" << position.z << "\t" << (position - loc).Length() << "\t";

      // Velocity of the Pendulum's CG in the Global Reference Frame
      ChVector<double> velocity = pendulum->GetPos_dt();
      outf << velocity.x << "\t" << velocity.y << "\t" << velocity.z << "\t" << velocity.Length() << "\t";

      // Acceleration of the Pendulum's CG in the Global Reference Frame
      ChVector<double> acceleration = pendulum->GetPos_dtdt();
      outf << acceleration.x << "\t" << acceleration.y << "\t" << acceleration.z << "\t" << acceleration.Length() << "\t";

      // Angular Position quaternion of the Pendulum with respect to the Global Reference Frame
      ChQuaternion<double> rot = pendulum->GetRot();
      outf << rot.e0 << "\t" << rot.e1 << "\t" << rot.e2 << "\t" << rot.e3 << "\t";

      // Angular Velocity of the Pendulum with respect to the Global Reference Frame
      ChVector<double> angVel = pendulum->GetWvel_par();
      outf << angVel.x << "\t" << angVel.y << "\t" << angVel.z << "\t" << angVel.Length() << "\t";

      // Angular Acceleration of the Pendulum with respect to the Global Reference Frame
      ChVector<double> angAccel = pendulum->GetWacc_par();
      outf << angAccel.x << "\t" << angAccel.y << "\t" << angAccel.z << "\t" << angAccel.Length() << "\t";

      // Reaction Force and Torque
      // These are expressed in the link coordinate system. We convert them to
      // the coordinate system of Body2 (in our case this is the ground).
      ChCoordsys<> linkCoordsys = hookeJoint->GetLinkRelativeCoords();
      ChVector<double> reactForce = hookeJoint->Get_react_force();
      ChVector<double> reactForceGlobal = linkCoordsys.TransformDirectionLocalToParent(reactForce);
      outf << reactForceGlobal.x << "\t" << reactForceGlobal.y << "\t" << reactForceGlobal.z << "\t" << reactForceGlobal.Length() << "\t";

      ChVector<double> reactTorque = hookeJoint->Get_react_torque();
      ChVector<double> reactTorqueGlobal = linkCoordsys.TransformDirectionLocalToParent(reactTorque);
      outf << reactTorqueGlobal.x << "\t" << reactTorqueGlobal.y << "\t" << reactTorqueGlobal.z << "\t" << reactTorqueGlobal.Length() << "\t";

      // Conservation of Energy
      //Translational Kinetic Energy (1/2*m*||v||^2)
      //Rotational Kinetic Energy (1/2 w'*I*w)  ChMatrix33*vector is valid since [3x3]*[3x1] = [3x1]
      //Delta Potential Energy (m*g*dz)
      ChMatrix33<> inertia = pendulum->GetInertia(); //3x3 Inertia Tensor in the local coordinate frame
      ChVector<> angVelLoc = pendulum->GetWvel_loc();
      double transKE = 0.5*mass*velocity.Length2();
      double rotKE = 0.5*Vdot(angVelLoc, inertia * angVelLoc);
      double deltaPE = mass*g*(position.z-loc.z);
      double totalKE = transKE + rotKE;
      outf << totalKE  << "\t" << transKE  << "\t" << rotKE  << "\t" << deltaPE  << "\t"  << std::endl;;
    }


    // Output a message to the command window once timeRecord has been reached
    //   Add a little error tolerance to make sure this event is captured
    if ((timeElapsed >= timeRecord-simTimeStep/2) && (timeElapsed <= timeRecord+simTimeStep/2)) {
        std::cout << "All Simulation Results have been recorded to file." << std::endl;
    }

    // Advance simulation by one step
    timeElapsed += simTimeStep;
    if(animate) {
      application->BeginScene();
      application->DrawAll();

      // Draw an XZ grid at the global origin to add in visualization
      ChIrrTools::drawGrid(
        application->GetVideoDriver(), 1, 1, 20, 20,
        ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)),
        video::SColor(255, 80, 100, 100), true);

      application->DoStep();  //Take one step in time
      application->EndScene();

      continueSimulation = application->GetDevice()->run();
    }
    else{
      my_system.DoStepDynamics(simTimeStep);  //Take one step in time
      continueSimulation = (timeElapsed <= timeRecord+simTimeStep/2);
    }
  }

  // Close output file
  outf.close();


}

// =============================================================================

int main(int argc, char* argv[])
{

  std::cout << "\nStarting Hooke Test Case 01\n\n";
  //Case 1 - Hooke Joint at the origin, and aligned with the global Y axis
  //  Note the hooke joint only allows 2 DOF(rotation about joint x & z axes)
  //    Therefore, the joint must be rotated -pi/2 about the global x-axis
  TestHooke(ChVector<> (0, 0, 0), ChQuaternion<> (Q_from_AngX(-CH_C_PI_2)), .001, "HookeJointData_Case01.txt",true);

  std::cout << "\nStarting Hooke Test Case 02\n\n";
  //Case 2 - Hooke Joint at (1,2,3), and joint z aligned with the global axis along Y = Z
  //  Note the revolute joint only allows 1 DOF(rotation about joint z axis)
  //    Therefore, the joint must be rotated -pi/4 about the global x-axis
  TestHooke(ChVector<> (1, 2, 3), ChQuaternion<> (Q_from_AngX(-CH_C_PI_4)), .001, "HookeJointData_Case02.txt",true);


  return 0;
}
