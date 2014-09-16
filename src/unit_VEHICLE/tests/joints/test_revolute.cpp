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

ChVector<> loc(0, 0, 0);          // location of revolute joint (in global frame)

double mass = 1.0;                // mass of pendulum
double length = 4.0;              // length of pendulum
ChVector<> inertiaXX(1, 1, 1);    // moments of inertia of pendulum

double timeRecord = 5;
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

  ChSharedPtr<ChCylinderShape> cyl_g(new ChCylinderShape);
  cyl_g->GetCylinderGeometry().p1 = loc + ChVector<>(0, -0.2, 0);
  cyl_g->GetCylinderGeometry().p2 = loc + ChVector<>(0, 0.2, 0);
  cyl_g->GetCylinderGeometry().rad = 0.1;
  ground->AddAsset(cyl_g);

  // ..the pendulum
  ChSharedBodyPtr  pendulum(new ChBody);
  my_system.AddBody(pendulum);
  pendulum->SetPos(loc + ChVector<>(length / 2, 0, 0));   // position of COG of pendulum
  pendulum->SetMass(mass);
  pendulum->SetInertiaXX(inertiaXX);

  ChSharedPtr<ChCylinderShape> cyl_p(new ChCylinderShape);
  cyl_p->GetCylinderGeometry().p1 = ChVector<>(-length / 2, 0, 0);
  cyl_p->GetCylinderGeometry().p2 = ChVector<>(length / 2, 0, 0);
  cyl_p->GetCylinderGeometry().rad = 0.1;
  pendulum->AddAsset(cyl_p);

  // 3- Create constraints: the mechanical joints between the rigid bodies.

  // .. a revolute joint between pendulum and ground
  ChSharedPtr<ChLinkLockRevolute>  revoluteJoint(new ChLinkLockRevolute);
  revoluteJoint->Initialize(pendulum, ground, ChCoordsys<>(loc, Q_from_AngX(CH_C_PI_2)));
  my_system.AddLink(revoluteJoint);

  // Create the Irrlicht application
  // -------------------------------

  ChIrrApp application(&my_system, L"ChLinkRevolute demo", core::dimension2d<u32>(800, 600), false, true);
  application.AddTypicalLogo();
  application.AddTypicalSky();
  application.AddTypicalLights();
  core::vector3df lookat((f32)loc.x, (f32)loc.y, (f32)loc.z);
  application.AddTypicalCamera(lookat + core::vector3df(0, 3, -6), lookat);

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

  double timeElapsed = 0;

  while (application.GetDevice()->run())
  {
    application.BeginScene();

    application.DrawAll();

    // Draw a grid
    ChIrrTools::drawGrid(
      application.GetVideoDriver(), 0.5, 0.5, 20, 20,
      ChCoordsys<>(loc, Q_from_AngX(CH_C_PI_2)),
      video::SColor(255, 80, 100, 100), true);

    // Write current position, velocity, acceleration, reaction force, and 
    // reaction torque of pendulum to output file
    if (outf && timeElapsed <= timeRecord) {
      // Time elapsed
      outf << timeElapsed << "\t";

      // Position
      ChVector<double> position = pendulum->GetPos();
      outf << position.x << "\t" << position.y << "\t" << position.z << "\t" << (position - loc).Length() << "\t";

      // Velocity
      ChVector<double> velocity = pendulum->GetPos_dt();
      outf << velocity.x << "\t" << velocity.y << "\t" << velocity.z << "\t" << velocity.Length() << "\t";

      // Acceleration
      ChVector<double> acceleration = pendulum->GetPos_dtdt();
      outf << acceleration.x << "\t" << acceleration.y << "\t" << acceleration.z << "\t" << acceleration.Length() << "\t";

      // Reaction Force and Torque
      // These are expressed in the link coordinate system. We convert them to
      // the coordinate system of Body2 (in our case this is the ground).
      ChCoordsys<> linkCoordsys = revoluteJoint->GetLinkRelativeCoords();
      ChVector<double> reactForce = revoluteJoint->Get_react_force();
      ChVector<double> reactForceGlobal = linkCoordsys.TransformDirectionLocalToParent(reactForce);
      outf << reactForceGlobal.x << "\t" << reactForceGlobal.y << "\t" << reactForceGlobal.z << "\t" << reactForceGlobal.Length() << "\t";

      ChVector<double> reactTorque = revoluteJoint->Get_react_torque();
      ChVector<double> reactTorqueGlobal = linkCoordsys.TransformDirectionLocalToParent(reactTorque);
      outf << reactTorqueGlobal.x << "\t" << reactTorqueGlobal.y << "\t" << reactTorqueGlobal.z << "\t" << reactTorqueGlobal.Length() << std::endl;
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

