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
// HMMWV full vehicle model with solid axle suspension...
//
// =============================================================================

#include "assets/ChSphereShape.h"
#include "assets/ChTriangleMeshShape.h"

#include "utils/ChUtilsData.h"
#include "utils/ChUtilsInputOutput.h"

#include "subsys/suspension/SolidAxle.h"
#include "models/hmmwv/suspension/HMMWV_SolidAxle.h"

#include "models/hmmwv/vehicle/HMMWV_VehicleSolidAxle.h"

using namespace chrono;

namespace hmmwv {


// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;
static const double lb2kg = 0.453592;
static const double lbf2N = 4.44822162;

const double     HMMWV_VehicleSolidAxle::m_chassisMass = lb2kg * 7747.0;                           // chassis sprung mass
const ChVector<> HMMWV_VehicleSolidAxle::m_chassisCOM = in2m * ChVector<>(-18.8, -0.585, 33.329);  // COM location
const ChVector<> HMMWV_VehicleSolidAxle::m_chassisInertia(125.8, 497.4, 531.4);                    // chassis inertia (roll,pitch,yaw)

const std::string HMMWV_VehicleSolidAxle::m_chassisMeshName = "hmmwv_chassis";
const std::string HMMWV_VehicleSolidAxle::m_chassisMeshFile = utils::GetModelDataFile("hmmwv/hmmwv_chassis.obj");

const ChCoordsys<> HMMWV_VehicleSolidAxle::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
bool use_JSON = false;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_VehicleSolidAxle::HMMWV_VehicleSolidAxle(const bool           fixed,
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
  m_suspensions.resize(2);
  if (use_JSON)
  {
    m_suspensions[0] = ChSharedPtr<ChSuspension>(new SolidAxle(utils::GetModelDataFile("generic/suspension/SolidAxleFront.json"), false));
    m_suspensions[1] = ChSharedPtr<ChSuspension>(new SolidAxle(utils::GetModelDataFile("generic/suspension/SolidAxleRear.json"), true));
  }
  else
  {
    m_suspensions[0] = ChSharedPtr<ChSuspension>(new HMMWV_SolidAxleFront("FrontSusp", false));
    m_suspensions[1] = ChSharedPtr<ChSuspension>(new HMMWV_SolidAxleRear("RearSusp", true));
  }

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


HMMWV_VehicleSolidAxle::~HMMWV_VehicleSolidAxle()
{
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_VehicleSolidAxle::Initialize(const ChCoordsys<>& chassisPos)
{
  m_chassis->SetFrame_REF_to_abs(ChFrame<>(chassisPos));

  // Initialize the steering subsystem (specify the steering subsystem's frame
  // relative to the chassis reference frame).
  ChVector<> offset = in2m * ChVector<>(63, 0, -3.1);
  m_steering->Initialize(m_chassis, offset, ChQuaternion<>(1, 0, 0, 0));

  // Initialize the suspension subsystems (specify the suspension subsystems'
  // frames relative to the chassis reference frame).
  m_suspensions[0]->Initialize(m_chassis, in2m * ChVector<>(66.59, 0, 0), m_steering->GetSteeringLink());
  m_suspensions[1]->Initialize(m_chassis, in2m * ChVector<>(-66.4, 0, 0), m_chassis);

  // Initialize wheels
  m_front_left_wheel->Initialize(m_suspensions[0]->GetSpindle(LEFT));
  m_front_right_wheel->Initialize(m_suspensions[0]->GetSpindle(RIGHT));
  m_rear_left_wheel->Initialize(m_suspensions[1]->GetSpindle(LEFT));
  m_rear_right_wheel->Initialize(m_suspensions[1]->GetSpindle(RIGHT));

  // Initialize the driveline subsystem (RWD)
  ChSuspensionList susp(1, m_suspensions[1]);
  m_driveline->Initialize(m_chassis, susp);

  // Initialize the four brakes
  m_front_left_brake->Initialize(m_suspensions[0]->GetRevolute(LEFT));
  m_front_right_brake->Initialize(m_suspensions[0]->GetRevolute(RIGHT));
  m_rear_left_brake->Initialize(m_suspensions[1]->GetRevolute(LEFT));
  m_rear_right_brake->Initialize(m_suspensions[1]->GetRevolute(RIGHT));
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double HMMWV_VehicleSolidAxle::GetSpringForce(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()].StaticCastTo<ChSolidAxle>()->GetSpringForce(wheel_id.side());
}

double HMMWV_VehicleSolidAxle::GetSpringLength(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()].StaticCastTo<ChSolidAxle>()->GetSpringLength(wheel_id.side());
}

double HMMWV_VehicleSolidAxle::GetSpringDeformation(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()].StaticCastTo<ChSolidAxle>()->GetSpringDeformation(wheel_id.side());
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double HMMWV_VehicleSolidAxle::GetShockForce(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()].StaticCastTo<ChSolidAxle>()->GetShockForce(wheel_id.side());
}

double HMMWV_VehicleSolidAxle::GetShockLength(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()].StaticCastTo<ChSolidAxle>()->GetShockLength(wheel_id.side());
}

double HMMWV_VehicleSolidAxle::GetShockVelocity(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()].StaticCastTo<ChSolidAxle>()->GetShockVelocity(wheel_id.side());
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_VehicleSolidAxle::Update(double              time,
                                    double              steering,
                                    double              braking,
                                    double              powertrain_torque,
                                    const ChTireForces& tire_forces)
{
  // Apply powertrain torque to the driveline's input shaft.
  m_driveline->ApplyDriveshaftTorque(powertrain_torque);

  // Let the steering subsystem process the steering input.
  m_steering->Update(time, 0.5 * steering);

  // Apply tire forces to spindle bodies.
  m_suspensions[0]->ApplyTireForce(LEFT, tire_forces[FRONT_LEFT.id()]);
  m_suspensions[0]->ApplyTireForce(RIGHT, tire_forces[FRONT_RIGHT.id()]);
  m_suspensions[1]->ApplyTireForce(LEFT, tire_forces[REAR_LEFT.id()]);
  m_suspensions[1]->ApplyTireForce(RIGHT, tire_forces[REAR_RIGHT.id()]);

  // Apply braking
  m_front_left_brake->ApplyBrakeModulation(braking);
  m_front_right_brake->ApplyBrakeModulation(braking);
  m_rear_left_brake->ApplyBrakeModulation(braking);
  m_rear_right_brake->ApplyBrakeModulation(braking);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_VehicleSolidAxle::ExportMeshPovray(const std::string& out_dir)
{
  utils::WriteMeshPovray(m_chassisMeshFile, m_chassisMeshName, out_dir, ChColor(0.82f, 0.7f, 0.5f));
}


// -----------------------------------------------------------------------------
// Log the hardpoint locations for the front-right and rear-right suspension
// subsystems (display in inches)
// -----------------------------------------------------------------------------
void HMMWV_VehicleSolidAxle::LogHardpointLocations()
{
  GetLog().SetNumFormat("%7.3f");

  GetLog() << "\n---- FRONT suspension hardpoint locations (RIGHT side)\n";
  m_suspensions[0].StaticCastTo<ChSolidAxle>()->LogHardpointLocations(ChVector<>(0, 0, 0), true);

  GetLog() << "\n---- REAR suspension hardpoint locations (RIGHT side)\n";
  m_suspensions[1].StaticCastTo<ChSolidAxle>()->LogHardpointLocations(ChVector<>(0, 0, 0), true);

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
void HMMWV_VehicleSolidAxle::DebugLog(int what)
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
