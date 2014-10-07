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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz
// =============================================================================
//
// HMMWV full vehicle model...
//
// =============================================================================

#include "assets/ChSphereShape.h"
#include "assets/ChTriangleMeshShape.h"

#include "utils/ChUtilsData.h"
#include "utils/ChUtilsInputOutput.h"

#include "models/hmmwv/vehicle/HMMWV_Vehicle.h"

using namespace chrono;

namespace hmmwv {


// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;
static const double lb2kg = 0.453592;
static const double lbf2N = 4.44822162;

const double     HMMWV_Vehicle::m_chassisMass = lb2kg * 7740.7;                           // chassis sprung mass
const ChVector<> HMMWV_Vehicle::m_chassisCOM = in2m * ChVector<>(-18.8, -0.585, 33.329);  // COM location
const ChVector<> HMMWV_Vehicle::m_chassisInertia(125.8, 497.4, 531.4);                    // chassis inertia (roll,pitch,yaw)

const std::string HMMWV_Vehicle::m_chassisMeshName = "hmmwv_chassis";
const std::string HMMWV_Vehicle::m_chassisMeshFile = utils::GetModelDataFile("hmmwv/hmmwv_chassis.obj");

const ChCoordsys<> HMMWV_Vehicle::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_Vehicle::HMMWV_Vehicle(const bool           fixed,
                             VisualizationType    chassisVis,
                             VisualizationType    wheelVis)
{
  // -------------------------------------------
  // Create the chassis body
  // -------------------------------------------

  m_chassis = ChSharedPtr<ChBodyAuxRef>(new ChBodyAuxRef);

  m_chassis->SetIdentifier(0);
  m_chassis->SetName("chassis");
  m_chassis->SetMass(m_chassisMass);
  m_chassis->SetFrame_COG_to_REF(ChFrame<>(m_chassisCOM, ChQuaternion<>(1, 0, 0, 0)));
  m_chassis->SetInertiaXX(m_chassisInertia);
  m_chassis->SetBodyFixed(fixed);

  switch (chassisVis) {
  case PRIMITIVES:
  {
    ChSharedPtr<ChSphereShape> sphere(new ChSphereShape);
    sphere->GetSphereGeometry().rad = 0.1;
    sphere->Pos = m_chassisCOM;
    m_chassis->AddAsset(sphere);

    break;
  }
  case MESH:
  {
    geometry::ChTriangleMeshConnected trimesh;
    trimesh.LoadWavefrontMesh(m_chassisMeshFile, false, false);

    ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(m_chassisMeshName);
    m_chassis->AddAsset(trimesh_shape);

    break;
  }
  }

  Add(m_chassis);

  // -------------------------------------------
  // Create the suspension subsystems
  // -------------------------------------------

  m_front_susp = ChSharedPtr<HMMWV_DoubleWishboneFront>(new HMMWV_DoubleWishboneFront("FrontSusp", false));
  m_rear_susp = ChSharedPtr<HMMWV_DoubleWishboneRear>(new HMMWV_DoubleWishboneRear("RearSusp", true));

  // -----------------------------
  // Create the steering subsystem
  // -----------------------------

  m_steering = ChSharedPtr<HMMWV_PitmanArm>(new HMMWV_PitmanArm("Steering"));

  // -----------------
  // Create the wheels
  // -----------------

  m_front_right_wheel = ChSharedPtr<HMMWV_Wheel>(new HMMWV_WheelRight(wheelVis));
  m_front_left_wheel = ChSharedPtr<HMMWV_Wheel>(new HMMWV_WheelLeft(wheelVis));
  m_rear_right_wheel = ChSharedPtr<HMMWV_Wheel>(new HMMWV_WheelRight(wheelVis));
  m_rear_left_wheel = ChSharedPtr<HMMWV_Wheel>(new HMMWV_WheelLeft(wheelVis));

  // --------------------
  // Create the driveline
  // --------------------

  m_driveline = ChSharedPtr<HMMWV_Driveline2WD>(new HMMWV_Driveline2WD(this));

  // -----------------
  // Create the brakes
  // -----------------
  m_front_right_brake = ChSharedPtr<HMMWV_BrakeSimple>(new HMMWV_BrakeSimple);
  m_front_left_brake = ChSharedPtr<HMMWV_BrakeSimple>(new HMMWV_BrakeSimple);
  m_rear_right_brake = ChSharedPtr<HMMWV_BrakeSimple>(new HMMWV_BrakeSimple);
  m_rear_left_brake = ChSharedPtr<HMMWV_BrakeSimple>(new HMMWV_BrakeSimple);
}


HMMWV_Vehicle::~HMMWV_Vehicle()
{
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_Vehicle::Initialize(const ChCoordsys<>& chassisPos)
{
  m_chassis->SetFrame_REF_to_abs(ChFrame<>(chassisPos));

  // Initialize the steering subsystem (specify the steering subsystem's frame
  // relative to the chassis reference frame).
  ChVector<> offset = in2m * ChVector<>(49.015, 0, 4.304);
  ChQuaternion<> rotation = Q_from_AngAxis(18.5 * CH_C_PI / 180, ChVector<>(0, 1, 0));
  m_steering->Initialize(m_chassis, offset, rotation);

  // Initialize the suspension subsystems (specify the suspension subsystems'
  // frames relative to the chassis reference frame).
  m_front_susp->Initialize(m_chassis, in2m * ChVector<>(66.59, 0, 1.039), m_steering->GetSteeringLink());
  m_rear_susp->Initialize(m_chassis, in2m * ChVector<>(-66.4, 0, 1.039), m_chassis);

  // Initialize wheels
  m_front_left_wheel->Initialize(m_front_susp->GetSpindle(LEFT));
  m_front_right_wheel->Initialize(m_front_susp->GetSpindle(RIGHT));
  m_rear_left_wheel->Initialize(m_rear_susp->GetSpindle(LEFT));
  m_rear_right_wheel->Initialize(m_rear_susp->GetSpindle(RIGHT));

  // Initialize the driveline subsystem (RWD)
  ChSuspensionList susp(1, m_rear_susp);
  m_driveline->Initialize(m_chassis, susp);

  // Initialize the four brakes
  m_front_left_brake->Initialize(m_front_susp->GetRevolute(LEFT));
  m_front_right_brake->Initialize(m_front_susp->GetRevolute(RIGHT));
  m_rear_left_brake->Initialize(m_rear_susp->GetRevolute(LEFT));
  m_rear_right_brake->Initialize(m_rear_susp->GetRevolute(RIGHT));
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSharedPtr<ChBody> HMMWV_Vehicle::GetWheelBody(const ChWheelID& wheel_id) const
{
  switch (wheel_id.axle()) {
  case 0:  return m_front_susp->GetSpindle(wheel_id.side());
  case 1:  return m_rear_susp->GetSpindle(wheel_id.side());
  default: return ChSharedPtr<ChBody>(NULL);
  }
}

const ChVector<>& HMMWV_Vehicle::GetWheelPos(const ChWheelID& wheel_id) const
{
  switch (wheel_id.axle()) {
  case 0:  return m_front_susp->GetSpindlePos(wheel_id.side());
  case 1:  return m_rear_susp->GetSpindlePos(wheel_id.side());
  default: return m_front_susp->GetSpindlePos(wheel_id.side());
  }
}

const ChQuaternion<>& HMMWV_Vehicle::GetWheelRot(const ChWheelID& wheel_id) const
{
  switch (wheel_id.axle()) {
  case 0:  return m_front_susp->GetSpindleRot(wheel_id.side());
  case 1:  return m_rear_susp->GetSpindleRot(wheel_id.side());
  default: return m_front_susp->GetSpindleRot(wheel_id.side());
  }
}

const ChVector<>& HMMWV_Vehicle::GetWheelLinVel(const ChWheelID& wheel_id) const
{
  switch (wheel_id.axle()) {
  case 0:  return m_front_susp->GetSpindleLinVel(wheel_id.side());
  case 1:  return m_rear_susp->GetSpindleLinVel(wheel_id.side());
  default: return m_front_susp->GetSpindleLinVel(wheel_id.side());
  }
}

ChVector<> HMMWV_Vehicle::GetWheelAngVel(const ChWheelID& wheel_id) const
{
  switch (wheel_id.axle()) {
  case 0:  return m_front_susp->GetSpindleAngVel(wheel_id.side());
  case 1:  return m_rear_susp->GetSpindleAngVel(wheel_id.side());
  default: return ChVector<>(0, 0, 0);
  }
}

double HMMWV_Vehicle::GetWheelOmega(const ChWheelID& wheel_id) const
{
  switch (wheel_id.axle()) {
  case 0:  return m_front_susp->GetAxleSpeed(wheel_id.side());
  case 1:  return m_rear_susp->GetAxleSpeed(wheel_id.side());
  default: return -1;
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double HMMWV_Vehicle::GetSpringForce(const ChWheelID& wheel_id) const
{
  switch (wheel_id.axle()) {
  case 0:  return m_front_susp->GetSpringForce(wheel_id.side());
  case 1:  return m_rear_susp->GetSpringForce(wheel_id.side());
  default: return -1;
  }
}

double HMMWV_Vehicle::GetSpringLength(const ChWheelID& wheel_id) const
{
  switch (wheel_id.axle()) {
  case 0:  return m_front_susp->GetSpringLength(wheel_id.side());
  case 1:  return m_rear_susp->GetSpringLength(wheel_id.side());
  default: return -1;
  }
}

double HMMWV_Vehicle::GetSpringDeformation(const ChWheelID& wheel_id) const
{
  switch (wheel_id.axle()) {
  case 0:  return m_front_susp->GetSpringDeformation(wheel_id.side());
  case 1:  return m_rear_susp->GetSpringDeformation(wheel_id.side());
  default: return -1;
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double HMMWV_Vehicle::GetShockForce(const ChWheelID& wheel_id) const
{
  switch (wheel_id.axle()) {
  case 0:  return m_front_susp->GetShockForce(wheel_id.side());
  case 1:  return m_rear_susp->GetShockForce(wheel_id.side());
  default: return -1;
  }
}

double HMMWV_Vehicle::GetShockLength(const ChWheelID& wheel_id) const
{
  switch (wheel_id.axle()) {
  case 0:  return m_front_susp->GetShockLength(wheel_id.side());
  case 1:  return m_rear_susp->GetShockLength(wheel_id.side());
  default: return -1;
  }
}

double HMMWV_Vehicle::GetShockVelocity(const ChWheelID& wheel_id) const
{
  switch (wheel_id.axle()) {
  case 0:  return m_front_susp->GetShockVelocity(wheel_id.side());
  case 1:  return m_rear_susp->GetShockVelocity(wheel_id.side());
  default: return -1;
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_Vehicle::Update(double              time,
                           double              steering,
                           double              braking,
                           double              powertrain_torque,
                           const ChTireForces& tire_forces)
{
  // Apply powertrain torque to the driveline's input shaft.
  m_driveline->ApplyDriveshaftTorque(powertrain_torque);

  // Let the steering subsystem process the steering input.
  m_steering->Update(time, steering);

  // Apply tire forces to spindle bodies.
  m_front_susp->ApplyTireForce(LEFT, tire_forces[FRONT_LEFT.id()]);
  m_front_susp->ApplyTireForce(RIGHT, tire_forces[FRONT_RIGHT.id()]);
  m_rear_susp->ApplyTireForce(LEFT, tire_forces[REAR_LEFT.id()]);
  m_rear_susp->ApplyTireForce(RIGHT, tire_forces[REAR_RIGHT.id()]);

  // Apply braking
  m_front_left_brake->ApplyBrakeModulation(braking);
  m_front_right_brake->ApplyBrakeModulation(braking);
  m_rear_left_brake->ApplyBrakeModulation(braking);
  m_rear_right_brake->ApplyBrakeModulation(braking);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_Vehicle::ExportMeshPovray(const std::string& out_dir)
{
  utils::WriteMeshPovray(m_chassisMeshFile, m_chassisMeshName, out_dir, ChColor(0.82f, 0.7f, 0.5f));
}


// -----------------------------------------------------------------------------
// Log the hardpoint locations for the front-right and rear-right suspension
// subsystems (display in inches)
// -----------------------------------------------------------------------------
void HMMWV_Vehicle::LogHardpointLocations()
{
  GetLog().SetNumFormat("%7.3f");

  GetLog() << "\n---- FRONT suspension hardpoint locations (LEFT side)\n";
  m_front_susp->LogHardpointLocations(ChVector<>(-37.78, 0, 30.77), true);

  GetLog() << "\n---- REAR suspension hardpoint locations (LEFT side)\n";
  m_rear_susp->LogHardpointLocations(ChVector<>(-170.77, 0, 30.77), true);

  GetLog() << "\n\n";

  GetLog().SetNumFormat("%g");
}


// -----------------------------------------------------------------------------
// Log constraint violations
// -----------------------------------------------------------------------------
void HMMWV_Vehicle::LogConstraintViolations()
{
  GetLog().SetNumFormat("%16.4e");

  // Report constraint violations for the suspension joints
  GetLog() << "\n---- FRONT-LEFT suspension constraint violation\n\n";
  m_front_susp->LogConstraintViolations(LEFT);
  GetLog() << "\n---- FRONT-RIGHT suspension constraint violation\n\n";
  m_front_susp->LogConstraintViolations(RIGHT);
  GetLog() << "\n---- REAR-LEFT suspension constraint violation\n\n";
  m_rear_susp->LogConstraintViolations(LEFT);
  GetLog() << "\n---- REAR-RIGHT suspension constraint violation\n\n";
  m_rear_susp->LogConstraintViolations(RIGHT);

  // Report constraint violations for the steering joints
  GetLog() << "\n---- STEERING constrain violation\n\n";
  m_steering->LogConstraintViolations();

  GetLog().SetNumFormat("%g");
}


// -----------------------------------------------------------------------------
// Log the spring length, deformation, and force.
// Log the shock length, velocity, and force.
// Log constraint violations of suspension joints.
//
// Lengths are reported in inches, velocities in inches/s, and forces in lbf
// -----------------------------------------------------------------------------
void HMMWV_Vehicle::DebugLog(int what)
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


} // end namespace hmmwv
