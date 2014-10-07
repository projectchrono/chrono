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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// HMMWV 9-body vehicle model...
//
// =============================================================================

#include "assets/ChSphereShape.h"
#include "assets/ChTriangleMeshShape.h"

#include "utils/ChUtilsData.h"
#include "utils/ChUtilsInputOutput.h"

#include "models/hmmwv/vehicle/HMMWV_VehicleReduced.h"

using namespace chrono;

namespace hmmwv {


// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;
static const double lb2kg = 0.453592;
static const double lbf2N = 4.44822162;

const double     HMMWV_VehicleReduced::m_chassisMass = lb2kg * 7740.7;                           // chassis sprung mass
const ChVector<> HMMWV_VehicleReduced::m_chassisCOM = in2m * ChVector<>(-18.8, -0.585, 33.329);  // COM location
const ChVector<> HMMWV_VehicleReduced::m_chassisInertia(125.8, 497.4, 531.4);                    // chassis inertia (roll,pitch,yaw)

const std::string HMMWV_VehicleReduced::m_chassisMeshName = "hmmwv_chassis";
const std::string HMMWV_VehicleReduced::m_chassisMeshFile = utils::GetModelDataFile("hmmwv/hmmwv_chassis.obj");

const ChCoordsys<> HMMWV_VehicleReduced::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_VehicleReduced::HMMWV_VehicleReduced(const bool           fixed,
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

  m_front_susp = ChSharedPtr<HMMWV_DoubleWishboneReducedFront>(new HMMWV_DoubleWishboneReducedFront("FrontSusp", false));
  m_rear_susp = ChSharedPtr<HMMWV_DoubleWishboneReducedRear>(new HMMWV_DoubleWishboneReducedRear("RearSusp", true));

  // -----------------------------
  // Create the steering subsystem
  // -----------------------------

  m_steering = ChSharedPtr<ChSteering>(new HMMWV_RackPinion("Steering"));

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

  m_driveline = ChSharedPtr<ChDriveline>(new HMMWV_Driveline2WD);

  // -----------------
  // Create the brakes
  // -----------------
  m_front_right_brake = ChSharedPtr<HMMWV_BrakeSimple>(new HMMWV_BrakeSimple);
  m_front_left_brake = ChSharedPtr<HMMWV_BrakeSimple>(new HMMWV_BrakeSimple);
  m_rear_right_brake = ChSharedPtr<HMMWV_BrakeSimple>(new HMMWV_BrakeSimple);
  m_rear_left_brake = ChSharedPtr<HMMWV_BrakeSimple>(new HMMWV_BrakeSimple);
}


HMMWV_VehicleReduced::~HMMWV_VehicleReduced()
{
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_VehicleReduced::Initialize(const ChCoordsys<>& chassisPos)
{
  m_chassis->SetFrame_REF_to_abs(ChFrame<>(chassisPos));

  // Initialize the steering subsystem (specify the steering subsystem's frame
  // relative to the chassis reference frame).
  ChVector<> offset = in2m * ChVector<>(56.735, 0, 3.174);
  m_steering->Initialize(m_chassis, offset, ChQuaternion<>(1, 0, 0, 0));

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
ChSharedPtr<ChBody> HMMWV_VehicleReduced::GetWheelBody(const ChWheelID& wheel_id) const
{
  switch (wheel_id.axle()) {
  case 0:  return m_front_susp->GetSpindle(wheel_id.side());
  case 1:  return m_rear_susp->GetSpindle(wheel_id.side());
  default: return ChSharedPtr<ChBody>(NULL);
  }
}

const ChVector<>& HMMWV_VehicleReduced::GetWheelPos(const ChWheelID& wheel_id) const
{
  switch (wheel_id.axle()) {
  case 0:  return m_front_susp->GetSpindlePos(wheel_id.side());
  case 1:  return m_rear_susp->GetSpindlePos(wheel_id.side());
  default: return m_front_susp->GetSpindlePos(wheel_id.side());
  }
}

const ChQuaternion<>& HMMWV_VehicleReduced::GetWheelRot(const ChWheelID& wheel_id) const
{
  switch (wheel_id.axle()) {
  case 0:  return m_front_susp->GetSpindleRot(wheel_id.side());
  case 1:  return m_rear_susp->GetSpindleRot(wheel_id.side());
  default: return m_front_susp->GetSpindleRot(wheel_id.side());
  }
}

const ChVector<>& HMMWV_VehicleReduced::GetWheelLinVel(const ChWheelID& wheel_id) const
{
  switch (wheel_id.axle()) {
  case 0:  return m_front_susp->GetSpindleLinVel(wheel_id.side());
  case 1:  return m_rear_susp->GetSpindleLinVel(wheel_id.side());
  default: return m_front_susp->GetSpindleLinVel(wheel_id.side());
  }
}

ChVector<> HMMWV_VehicleReduced::GetWheelAngVel(const ChWheelID& wheel_id) const
{
  switch (wheel_id.axle()) {
  case 0:  return m_front_susp->GetSpindleAngVel(wheel_id.side());
  case 1:  return m_rear_susp->GetSpindleAngVel(wheel_id.side());
  default: return ChVector<>(0, 0, 0);
  }
}

double HMMWV_VehicleReduced::GetWheelOmega(const ChWheelID& wheel_id) const
{
  switch (wheel_id.axle()) {
  case 0:  return m_front_susp->GetAxleSpeed(wheel_id.side());
  case 1:  return m_rear_susp->GetAxleSpeed(wheel_id.side());
  default: return -1;
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_VehicleReduced::Update(double              time,
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
void HMMWV_VehicleReduced::ExportMeshPovray(const std::string& out_dir)
{
  utils::WriteMeshPovray(m_chassisMeshFile, m_chassisMeshName, out_dir);
}


// -----------------------------------------------------------------------------
// Log constraint violations
// -----------------------------------------------------------------------------
void HMMWV_VehicleReduced::LogConstraintViolations()
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


} // end namespace hmmwv
