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

#include <ostream>
#include <fstream>

#include "physics/ChSystem.h"
#include "physics/ChBody.h"

#include "unit_IRRLICHT/ChIrrApp.h"

#include "ChronoT_config.h"

using namespace chrono;
using namespace irr;


// =============================================================================

double mass = 1.0;
double length = 4.0;
ChVector<> inertiaXX(1, 1, 1);

double timeRecord = 5;
double timeElapsed = 0;
double timeStep = 0.001;

// =============================================================================

int main(int argc, char* argv[])
{
  SetChronoDataPath(CHRONO_DATA_DIR);

  // Create the mechanical system
  // ----------------------------

  // 1- Create a ChronoENGINE physical system: all bodies and constraints
  //    will be handled by this ChSystem object.

  ChSystem my_system;
  my_system.Set_G_acc(ChVector<double>(0.0, 0.0, -9.80665));

  my_system.SetIntegrationType(ChSystem::INT_ANITESCU);
  my_system.SetIterLCPmaxItersSpeed(100);
  my_system.SetIterLCPmaxItersStab(100); //Tasora stepper uses this, Anitescu does not
  my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);

  // 2- Create the rigid bodies of the system

  // ..the ground
  ChSharedBodyPtr  ground(new ChBody);
  my_system.AddBody(ground);
  ground->SetBodyFixed(true);

  // ..the pendulum
  ChSharedBodyPtr  pendulum(new ChBody);
  my_system.AddBody(pendulum);
  pendulum->SetPos(ChVector<>(length / 2, 0, 0));   // position of COG of pendulum
  pendulum->SetMass(mass);
  pendulum->SetInertiaXX(inertiaXX);

  ChSharedMarkerPtr pendEndMrkr(new ChMarker);
  pendEndMrkr->SetPos(ChVector<>(length, 0, 0));  // position of end of crank (beginning is at origin)
  pendulum->AddMarker(pendEndMrkr);

  // 3- Create constraints: the mechanical joints between the rigid bodies.

  // .. a revolute joint between pendulum and ground
  ChSharedPtr<ChLinkLockRevolute>  revoluteJoint(new ChLinkLockRevolute);
  revoluteJoint->Initialize(pendulum, ground, ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)));
  my_system.AddLink(revoluteJoint);

  // Create the Irrlicht application
  // -------------------------------

  ChIrrApp application(&my_system, L"ChLinkRevolute demo", core::dimension2d<u32>(800, 600), false, true);
  application.AddTypicalLogo();
  application.AddTypicalSky();
  application.AddTypicalLights();
  application.AddTypicalCamera(core::vector3df(0, 3, 6));

  application.AssetBindAll();
  application.AssetUpdateAll();

  // Simulation loop
  // ---------------

  // Create output file for results
  std::ofstream outf("RevoluteJointData.txt");

  if (outf) {
    outf << "timeElapsed(s)\t";
    outf << "X_Pos\tY_Pos\tZ_Pos\tLength_Pos\t";
    outf << "X_Vel\tY_Vel\tZ_Vel\tLength_Vel\t";
    outf << "X_Accel\tY_Accel\tZ_Accell\tLength_Accel\t";
    outf << "X_Glb_ReactionFrc\tY_Glb_ReactionFrc\tZ_Glb_ReactionFrc\tLength_Glb_ReactionFrc\t";
    outf << "X_Glb_ReactionTrq\tY_Glb_ReactionTrq\tZ_Glb_ReactionTrq\tLength_Glb_ReactionTrq\t" << std::endl;
  }
  else {
    std::cout << "Output file is invalid" << std::endl;
  }


  application.SetTimestep(timeStep);

  while (application.GetDevice()->run())
  {
    application.BeginScene();

    application.DrawAll();

    // Draw a grid
    ChIrrTools::drawGrid(
      application.GetVideoDriver(), 0.5, 0.5, 20, 20,
      ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)),
      video::SColor(255, 80, 100, 100), true);

    // Draw the pendulum (from revolute joint to the end marker)
    ChIrrTools::drawSegment(
      application.GetVideoDriver(),
      revoluteJoint->GetMarker1()->GetAbsCoord().pos,
      pendEndMrkr->GetAbsCoord().pos,
      video::SColor(255, 255, 0, 0));

    // Draw a small circle at pendulum origin
    ChIrrTools::drawCircle(
      application.GetVideoDriver(), 0.1,
      ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)));

    // Write current position, velocity, acceleration, reaction force, and 
    // reaction torque of pendulum to output file
    if (outf && timeElapsed <= timeRecord) {
      // Time elapsed
      outf << timeElapsed << "\t";

      // Position
      ChVector<double> position = pendulum->GetPos();
      outf << position.x << "\t" << position.y << "\t" << position.z << "\t" << position.Length() << "\t";

      // Velocity
      ChVector<double> velocity = pendulum->GetPos_dt();
      outf << velocity.x << "\t" << velocity.y << "\t" << velocity.z << "\t" << velocity.Length() << "\t";

      // Acceleration
      ChVector<double> acceleration = pendulum->GetPos_dtdt();
      outf << acceleration.x << "\t" << acceleration.y << "\t" << acceleration.z << "\t" << acceleration.Length() << "\t";

      // Reaction Force
      ChVector<double> reactForce = revoluteJoint->Get_react_force();
      ChVector<double> reactForceGlobal = revoluteJoint->GetLinkRelativeCoords().TransformLocalToParent(reactForce);
      outf << reactForceGlobal.x << "\t" << reactForceGlobal.y << "\t" << reactForceGlobal.z << "\t" << reactForceGlobal.Length() << "\t";

      // Reaction Torque
      ChVector<double> reactTorque = pendulum->Get_Xtorque();
      outf << reactTorque.x << "\t" << reactTorque.y << "\t" << reactTorque.z << "\t" << reactTorque.Length() << std::endl;
    }

    // Advance simulation by one step
    application.DoStep();
    timeElapsed += timeStep;

    application.EndScene();
  }

  // Close output file
  outf.close();

  return 0;
}

