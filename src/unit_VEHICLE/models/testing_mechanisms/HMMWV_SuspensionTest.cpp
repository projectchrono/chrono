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
// Authors: Justin Madsen, Daniel Melanz, Radu Serban
// =============================================================================
//
// Suspension testing mechanism with a double wishbone/pitman arm steering combo
//
// =============================================================================

#include "assets/ChSphereShape.h"
#include "assets/ChTriangleMeshShape.h"

#include "utils/ChUtilsData.h"
#include "utils/ChUtilsInputOutput.h"

#include "models/testing_mechanisms/HMMWV_SuspensionTest.h"

using namespace chrono;

// -----------------------------------------------------------------------------
// Static variables
static const double in2m = 0.0254;
static const double lb2kg = 0.453592;
static const double lbf2N = 4.44822162;


// const std::string HMMWV_SuspensionTest::m_chassisMeshName = "hmmwv_chassis";
// const std::string HMMWV_SuspensionTest::m_chassisMeshFile = utils::GetModelDataFile("hmmwv/hmmwv_chassis.obj");


// -----------------------------------------------------------------------------
HMMWV_SuspensionTest::HMMWV_SuspensionTest(VisualizationType    wheelVis)
{
  // these parameters don't matter much
  double chassisMass = lb2kg * 7740.7;
  ChVector<> chassisCOM = in2m * ChVector<>(-18.8, -0.585, 33.329);  // COM location
  ChVector<> chassisInertia(125.8, 497.4, 531.4);   // chassis inertia (roll,pitch,yaw)
  // this might matter
  ChCoordsys<> driverCsys(ChVector<>(0.8735, -0.27475, 1.052), ChQuaternion<>(1, 0, 0, 0));
  m_driverCsys = driverCsys;

  // -------------------------------------------
  // Create the chassis body, fixed in place
  m_chassis = ChSharedPtr<ChBodyAuxRef>(new ChBodyAuxRef);
  m_chassis->SetIdentifier(0);
  m_chassis->SetName("chassis");
  m_chassis->SetMass(chassisMass);
  m_chassis->SetFrame_COG_to_REF(ChFrame<>(chassisCOM, ChQuaternion<>(1, 0, 0, 0)));
  m_chassis->SetInertiaXX(chassisInertia);
  m_chassis->SetBodyFixed(true);

  // place a sphere at the COM loc
  ChSharedPtr<ChSphereShape> sphere(new ChSphereShape);
  sphere->GetSphereGeometry().rad = 0.1;
  sphere->Pos = chassisCOM;
  m_chassis->AddAsset(sphere);

  Add(m_chassis);

  // -------------------------------------------
  // Create the suspension subsystems, only a front suspension for the tester
  m_suspensions.resize(1);
  m_suspensions[0] = ChSharedPtr<ChSuspension>(new hmmwv::HMMWV_DoubleWishboneFront("FrontSusp"));

  // -----------------------------
  // Create the steering subsystem
  m_steering = ChSharedPtr<ChSteering>(new hmmwv::HMMWV_PitmanArm("Steering"));

  // -----------------
  // Create the wheels
  m_front_right_wheel = ChSharedPtr<hmmwv::HMMWV_Wheel>(new hmmwv::HMMWV_WheelRight(wheelVis));
  m_front_left_wheel = ChSharedPtr<hmmwv::HMMWV_Wheel>(new hmmwv::HMMWV_WheelLeft(wheelVis));

  // create the functions for actuation
  m_actuator_L = ChSharedPtr<ChFunction_Const>(new ChFunction_Const(0));
  m_actuator_R = ChSharedPtr<ChFunction_Const>(new ChFunction_Const(0));
}


HMMWV_SuspensionTest::~HMMWV_SuspensionTest()
{
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_SuspensionTest::Initialize(const ChCoordsys<>& chassisPos)
{
   m_chassis->SetFrame_REF_to_abs(ChFrame<>(chassisPos));

  // Initialize the steering subsystem (specify the steering subsystem's frame
  // relative to the chassis reference frame).
  ChVector<> offset = in2m * ChVector<>(49.015, 0, 4.304);
  ChQuaternion<> rotation = Q_from_AngAxis(18.5 * CH_C_PI / 180, ChVector<>(0, 1, 0));
  m_steering->Initialize(m_chassis, offset, rotation);

  // Initialize the suspension subsystems (specify the suspension subsystems'
  // frames relative to the chassis reference frame).
  m_suspensions[0]->Initialize(m_chassis, in2m * ChVector<>(66.59, 0, 1.039), m_steering->GetSteeringLink());

  // Initialize wheels
  m_front_left_wheel->Initialize(m_suspensions[0]->GetSpindle(LEFT));
  m_front_right_wheel->Initialize(m_suspensions[0]->GetSpindle(RIGHT));

}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double HMMWV_SuspensionTest::GetSpringForce(const ChWheelID& wheel_id) const
{
  return m_suspensions[0].StaticCastTo<ChDoubleWishbone>()->GetSpringForce(wheel_id.side());
}

double HMMWV_SuspensionTest::GetSpringLength(const ChWheelID& wheel_id) const
{
  return m_suspensions[0].StaticCastTo<ChDoubleWishbone>()->GetSpringLength(wheel_id.side());
}

double HMMWV_SuspensionTest::GetSpringDeformation(const ChWheelID& wheel_id) const
{
  return m_suspensions[0].StaticCastTo<ChDoubleWishbone>()->GetSpringDeformation(wheel_id.side());
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double HMMWV_SuspensionTest::GetShockForce(const ChWheelID& wheel_id) const
{
  return m_suspensions[0].StaticCastTo<ChDoubleWishbone>()->GetShockForce(wheel_id.side());
}

double HMMWV_SuspensionTest::GetShockLength(const ChWheelID& wheel_id) const
{
  return m_suspensions[0].StaticCastTo<ChDoubleWishbone>()->GetShockLength(wheel_id.side());
}

double HMMWV_SuspensionTest::GetShockVelocity(const ChWheelID& wheel_id) const
{
  return m_suspensions[0].StaticCastTo<ChDoubleWishbone>()->GetShockVelocity(wheel_id.side());
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_SuspensionTest::Update(double       time,
                           double              steering,
                           const ChTireForces& tire_forces)
{
  // Let the steering subsystem process the steering input.
  m_steering->Update(time, steering);

  // Apply tire forces to spindle bodies.
  m_suspensions[0]->ApplyTireForce(LEFT, tire_forces[FRONT_LEFT.id()]);
  m_suspensions[0]->ApplyTireForce(RIGHT, tire_forces[FRONT_RIGHT.id()]);

}


// -----------------------------------------------------------------------------
// Log the hardpoint locations for the left/right suspension, [in]
void HMMWV_SuspensionTest::LogHardpointLocations()
{
  GetLog().SetNumFormat("%7.3f");

  GetLog() << "\n---- FRONT suspension hardpoint locations (LEFT side)\n";
  m_suspensions[0].StaticCastTo<ChDoubleWishbone>()->LogHardpointLocations(ChVector<>(-37.78, 0, 30.77), true);

  GetLog() << "\n\n";

  GetLog().SetNumFormat("%g");
}


// -----------------------------------------------------------------------------
// Log the spring length, deformation, and force.
// Log the shock length, velocity, and force.
// Log constraint violations of suspension joints.
//
// Lengths are reported in inches, velocities in inches/s, and forces in lbf
// -----------------------------------------------------------------------------
void HMMWV_SuspensionTest::DebugLog(int what)
{
  GetLog().SetNumFormat("%10.2f");

  if (what & DBG_SPRINGS)
  {
    GetLog() << "\n---- Spring (front-left, front-right, rear-left, rear-right)\n";
    GetLog() << "Length [inch]       "
      << GetSpringLength(FRONT_LEFT) / in2m << "  "
      << GetSpringLength(FRONT_RIGHT) / in2m << "  "
      << GetSpringLength(REAR_LEFT) / in2m << "  "
      << GetSpringLength(REAR_RIGHT) / in2m << "\n";
    GetLog() << "Deformation [inch]  "
      << GetSpringDeformation(FRONT_LEFT) / in2m << "  "
      << GetSpringDeformation(FRONT_RIGHT) / in2m << "  "
      << GetSpringDeformation(REAR_LEFT) / in2m << "  "
      << GetSpringDeformation(REAR_RIGHT) / in2m << "\n";
    GetLog() << "Force [lbf]         "
      << GetSpringForce(FRONT_LEFT) / lbf2N << "  "
      << GetSpringForce(FRONT_RIGHT) / lbf2N << "  "
      << GetSpringForce(REAR_LEFT) / lbf2N << "  "
      << GetSpringForce(REAR_RIGHT) / lbf2N << "\n";
  }

  if (what & DBG_SHOCKS)
  {
    GetLog() << "\n---- Shock (front-left, front-right, rear-left, rear-right)\n";
    GetLog() << "Length [inch]       "
      << GetShockLength(FRONT_LEFT) / in2m << "  "
      << GetShockLength(FRONT_RIGHT) / in2m << "  "
      << GetShockLength(REAR_LEFT) / in2m << "  "
      << GetShockLength(REAR_RIGHT) / in2m << "\n";
    GetLog() << "Velocity [inch/s]   "
      << GetShockVelocity(FRONT_LEFT) / in2m << "  "
      << GetShockVelocity(FRONT_RIGHT) / in2m << "  "
      << GetShockVelocity(REAR_LEFT) / in2m << "  "
      << GetShockVelocity(REAR_RIGHT) / in2m << "\n";
    GetLog() << "Force [lbf]         "
      << GetShockForce(FRONT_LEFT) / lbf2N << "  "
      << GetShockForce(FRONT_RIGHT) / lbf2N << "  "
      << GetShockForce(REAR_LEFT) / lbf2N << "  "
      << GetShockForce(REAR_RIGHT) / lbf2N << "\n";
  }

  if (what & DBG_CONSTRAINTS)
  {
    // Report constraint violations for all joints
    LogConstraintViolations();
  }

  GetLog().SetNumFormat("%g");
}

