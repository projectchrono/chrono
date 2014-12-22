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

#include "subsys/ChVehicleModelData.h"

#include "utils/ChUtilsInputOutput.h"

#include "models/hmmwv/vehicle/HMMWV_Vehicle.h"

using namespace chrono;

namespace hmmwv {


// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double     HMMWV_Vehicle::m_chassisMass = 2086.524902;                           // chassis sprung mass
const ChVector<> HMMWV_Vehicle::m_chassisCOM = ChVector<>(0.055765, 0, 0.52349);       // COM location
const ChVector<> HMMWV_Vehicle::m_chassisInertia(1078.52344, 2955.66050, 3570.20377);  // chassis inertia (roll,pitch,yaw)

const std::string HMMWV_Vehicle::m_chassisMeshName = "hmmwv_chassis_POV_geom";
const std::string HMMWV_Vehicle::m_chassisMeshFile = vehicle::GetDataFile("hmmwv/hmmwv_chassis.obj");

const ChCoordsys<> HMMWV_Vehicle::m_driverCsys(ChVector<>(0.8735, -0.27475, 1.052), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_Vehicle::HMMWV_Vehicle(const bool           fixed,
                             DrivelineType        driveType,
                             VisualizationType    chassisVis,
                             VisualizationType    wheelVis)
: m_driveType(driveType)
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
  m_suspensions[0] = ChSharedPtr<ChSuspension>(new HMMWV_DoubleWishboneFront("FrontSusp"));
  m_suspensions[1] = ChSharedPtr<ChSuspension>(new HMMWV_DoubleWishboneRear("RearSusp"));

  // -----------------------------
  // Create the steering subsystem
  // -----------------------------
  m_steering = ChSharedPtr<ChSteering>(new HMMWV_PitmanArm("Steering"));

  // -----------------
  // Create the wheels
  // -----------------
  m_wheels.resize(4);
  m_wheels[0] = ChSharedPtr<ChWheel>(new HMMWV_WheelLeft(wheelVis));
  m_wheels[1] = ChSharedPtr<ChWheel>(new HMMWV_WheelRight(wheelVis));
  m_wheels[2] = ChSharedPtr<ChWheel>(new HMMWV_WheelLeft(wheelVis));
  m_wheels[3] = ChSharedPtr<ChWheel>(new HMMWV_WheelRight(wheelVis));

  // --------------------
  // Create the driveline
  // --------------------
  switch (m_driveType) {
  case FWD:
  case RWD:
    m_driveline = ChSharedPtr<ChDriveline>(new HMMWV_Driveline2WD);
    break;
  case AWD:
    m_driveline = ChSharedPtr<ChDriveline>(new HMMWV_Driveline4WD);
    break;
  }

  // -----------------
  // Create the brakes
  // -----------------
  m_brakes.resize(4);
  m_brakes[0] = ChSharedPtr<ChBrake>(new HMMWV_BrakeSimple);
  m_brakes[1] = ChSharedPtr<ChBrake>(new HMMWV_BrakeSimple);
  m_brakes[2] = ChSharedPtr<ChBrake>(new HMMWV_BrakeSimple);
  m_brakes[3] = ChSharedPtr<ChBrake>(new HMMWV_BrakeSimple);
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
  ChVector<> offset = ChVector<>(1.24498, 0, 0.101322);
  ChQuaternion<> rotation = Q_from_AngAxis(18.5 * CH_C_PI / 180, ChVector<>(0, 1, 0));
  m_steering->Initialize(m_chassis, offset, rotation);

  // Initialize the suspension subsystems (specify the suspension subsystems'
  // frames relative to the chassis reference frame).
  m_suspensions[0]->Initialize(m_chassis, ChVector<>(1.688965, 0, 0), m_steering->GetSteeringLink());
  m_suspensions[1]->Initialize(m_chassis, ChVector<>(-1.688965, 0, 0), m_chassis);

  // Initialize wheels
  m_wheels[0]->Initialize(m_suspensions[0]->GetSpindle(LEFT));
  m_wheels[1]->Initialize(m_suspensions[0]->GetSpindle(RIGHT));
  m_wheels[2]->Initialize(m_suspensions[1]->GetSpindle(LEFT));
  m_wheels[3]->Initialize(m_suspensions[1]->GetSpindle(RIGHT));

  // Initialize the driveline subsystem
  std::vector<int> driven_susp_indexes(m_driveline->GetNumDrivenAxles());

  switch (m_driveType) {
  case FWD:
    driven_susp_indexes[0] = 0;
    break;
  case RWD:
    driven_susp_indexes[0] = 1;
    break;
  case AWD:
    driven_susp_indexes[0] = 0;
    driven_susp_indexes[1] = 1;
    break;
  }

  m_driveline->Initialize(m_chassis, m_suspensions, driven_susp_indexes);

  // Initialize the four brakes
  m_brakes[0]->Initialize(m_suspensions[0]->GetRevolute(LEFT));
  m_brakes[1]->Initialize(m_suspensions[0]->GetRevolute(RIGHT));
  m_brakes[2]->Initialize(m_suspensions[1]->GetRevolute(LEFT));
  m_brakes[3]->Initialize(m_suspensions[1]->GetRevolute(RIGHT));
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double HMMWV_Vehicle::GetSpringForce(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()].StaticCastTo<ChDoubleWishbone>()->GetSpringForce(wheel_id.side());
}

double HMMWV_Vehicle::GetSpringLength(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()].StaticCastTo<ChDoubleWishbone>()->GetSpringLength(wheel_id.side());
}

double HMMWV_Vehicle::GetSpringDeformation(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()].StaticCastTo<ChDoubleWishbone>()->GetSpringDeformation(wheel_id.side());
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double HMMWV_Vehicle::GetShockForce(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()].StaticCastTo<ChDoubleWishbone>()->GetShockForce(wheel_id.side());
}

double HMMWV_Vehicle::GetShockLength(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()].StaticCastTo<ChDoubleWishbone>()->GetShockLength(wheel_id.side());
}

double HMMWV_Vehicle::GetShockVelocity(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()].StaticCastTo<ChDoubleWishbone>()->GetShockVelocity(wheel_id.side());
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
  m_suspensions[0]->ApplyTireForce(LEFT, tire_forces[FRONT_LEFT.id()]);
  m_suspensions[0]->ApplyTireForce(RIGHT, tire_forces[FRONT_RIGHT.id()]);
  m_suspensions[1]->ApplyTireForce(LEFT, tire_forces[REAR_LEFT.id()]);
  m_suspensions[1]->ApplyTireForce(RIGHT, tire_forces[REAR_RIGHT.id()]);

  // Apply braking
  m_brakes[0]->ApplyBrakeModulation(braking);
  m_brakes[1]->ApplyBrakeModulation(braking);
  m_brakes[2]->ApplyBrakeModulation(braking);
  m_brakes[3]->ApplyBrakeModulation(braking);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_Vehicle::ExportMeshPovray(const std::string& out_dir)
{
  utils::WriteMeshPovray(m_chassisMeshFile,
                         m_chassisMeshName,
                         out_dir,
                         ChColor(0.82f, 0.7f, 0.5f));

  HMMWV_Wheel* wheelFL = static_cast<HMMWV_Wheel*>(m_wheels[0].get_ptr());
  HMMWV_Wheel* wheelFR = static_cast<HMMWV_Wheel*>(m_wheels[1].get_ptr());
  wheelFL->ExportMeshPovray(out_dir);
  wheelFR->ExportMeshPovray(out_dir);
}


// -----------------------------------------------------------------------------
// Log the hardpoint locations for the front-right and rear-right suspension
// subsystems (display in inches)
// -----------------------------------------------------------------------------
void HMMWV_Vehicle::LogHardpointLocations()
{
  GetLog().SetNumFormat("%7.3f");

  GetLog() << "\n---- FRONT suspension hardpoint locations (LEFT side)\n";
  m_suspensions[0].StaticCastTo<ChDoubleWishbone>()->LogHardpointLocations(ChVector<>(-37.78, 0, 30.77), true);

  GetLog() << "\n---- REAR suspension hardpoint locations (LEFT side)\n";
  m_suspensions[1].StaticCastTo<ChDoubleWishbone>()->LogHardpointLocations(ChVector<>(-170.77, 0, 30.77), true);

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
void HMMWV_Vehicle::DebugLog(int what)
{
  GetLog().SetNumFormat("%10.2f");

  if (what & DBG_SPRINGS)
  {
    GetLog() << "\n---- Spring (front-left, front-right, rear-left, rear-right)\n";
    GetLog() << "Length [m]       "
      << GetSpringLength(FRONT_LEFT) << "  "
      << GetSpringLength(FRONT_RIGHT) << "  "
      << GetSpringLength(REAR_LEFT) << "  "
      << GetSpringLength(REAR_RIGHT) << "\n";
    GetLog() << "Deformation [m]  "
      << GetSpringDeformation(FRONT_LEFT) << "  "
      << GetSpringDeformation(FRONT_RIGHT) << "  "
      << GetSpringDeformation(REAR_LEFT) << "  "
      << GetSpringDeformation(REAR_RIGHT) << "\n";
    GetLog() << "Force [N]         "
      << GetSpringForce(FRONT_LEFT) << "  "
      << GetSpringForce(FRONT_RIGHT) << "  "
      << GetSpringForce(REAR_LEFT) << "  "
      << GetSpringForce(REAR_RIGHT) << "\n";
  }

  if (what & DBG_SHOCKS)
  {
    GetLog() << "\n---- Shock (front-left, front-right, rear-left, rear-right)\n";
    GetLog() << "Length [m]       "
      << GetShockLength(FRONT_LEFT) << "  "
      << GetShockLength(FRONT_RIGHT) << "  "
      << GetShockLength(REAR_LEFT) << "  "
      << GetShockLength(REAR_RIGHT) << "\n";
    GetLog() << "Velocity [m/s]   "
      << GetShockVelocity(FRONT_LEFT) << "  "
      << GetShockVelocity(FRONT_RIGHT) << "  "
      << GetShockVelocity(REAR_LEFT) << "  "
      << GetShockVelocity(REAR_RIGHT) << "\n";
    GetLog() << "Force [N]         "
      << GetShockForce(FRONT_LEFT) << "  "
      << GetShockForce(FRONT_RIGHT) << "  "
      << GetShockForce(REAR_LEFT) << "  "
      << GetShockForce(REAR_RIGHT) << "\n";
  }

  if (what & DBG_CONSTRAINTS)
  {
    // Report constraint violations for all joints
    LogConstraintViolations();
  }

  GetLog().SetNumFormat("%g");
}


} // end namespace hmmwv
