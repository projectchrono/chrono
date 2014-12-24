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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz, Alessandro Tasora
// =============================================================================
//
// Articulated vehicle model. 
// Can be constructed either with solid-axle or with multi-link suspensions.
// Always uses a rack-pinion steering and a 2WD driveline model.
//
// =============================================================================

#include "assets/ChSphereShape.h"
#include "assets/ChTriangleMeshShape.h"

#include "subsys/ChVehicleModelData.h"

#include "utils/ChUtilsInputOutput.h"

#include "models/articulated/Articulated_SolidAxle.h"
#include "models/articulated/Articulated_MultiLink.h"

#include "models/articulated/Articulated_Vehicle.h"

using namespace chrono;


// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double     Articulated_Vehicle::m_chassisMass = 3500;                   // chassis sprung mass
const ChVector<> Articulated_Vehicle::m_chassisCOM (-0.2, 0, 0.8);            // COM location
const ChVector<> Articulated_Vehicle::m_chassisInertia(125.8, 497.4, 531.4);  // chassis inertia (roll,pitch,yaw)

const ChCoordsys<> Articulated_Vehicle::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Articulated_Vehicle::Articulated_Vehicle(const bool        fixed,
                                         SuspensionType    suspType,
                                         VisualizationType wheelVis)
: m_suspType(suspType)
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

  ChSharedPtr<ChSphereShape> sphere(new ChSphereShape);
  sphere->GetSphereGeometry().rad = 0.1;
  sphere->Pos = m_chassisCOM;
  m_chassis->AddAsset(sphere);

  m_system->Add(m_chassis);

  // -------------------------------------------
  // Create the suspension subsystems
  // -------------------------------------------
  m_suspensions.resize(2);

  assert(m_suspType == SOLID_AXLE || m_suspType == MULTI_LINK);

  switch (m_suspType) {
  case SOLID_AXLE:
    m_suspensions[0] = ChSharedPtr<ChSuspension>(new Articulated_SolidAxleFront("FrontSusp"));
    m_suspensions[1] = ChSharedPtr<ChSuspension>(new Articulated_SolidAxleRear("RearSusp"));
    break;
  case MULTI_LINK:
    m_suspensions[0] = ChSharedPtr<ChSuspension>(new Articulated_MultiLinkFront("FrontSusp"));
    m_suspensions[1] = ChSharedPtr<ChSuspension>(new Articulated_MultiLinkRear("RearSusp"));
    break;
  }

  // -----------------------------
  // Create the steering subsystem
  // -----------------------------
  m_steering = ChSharedPtr<ChSteering>(new Articulated_RackPinion("Steering"));

  // -----------------
  // Create the wheels
  // -----------------
  m_wheels.resize(4);
  m_wheels[0] = ChSharedPtr<ChWheel>(new Articulated_Wheel(wheelVis));
  m_wheels[1] = ChSharedPtr<ChWheel>(new Articulated_Wheel(wheelVis));
  m_wheels[2] = ChSharedPtr<ChWheel>(new Articulated_Wheel(wheelVis));
  m_wheels[3] = ChSharedPtr<ChWheel>(new Articulated_Wheel(wheelVis));

  // --------------------
  // Create the driveline
  // --------------------
  m_driveline = ChSharedPtr<ChDriveline>(new Articulated_Driveline2WD);

  // -----------------
  // Create the brakes
  // -----------------
  m_brakes.resize(4);
  m_brakes[0] = ChSharedPtr<ChBrake>(new Articulated_BrakeSimple);
  m_brakes[1] = ChSharedPtr<ChBrake>(new Articulated_BrakeSimple);
  m_brakes[2] = ChSharedPtr<ChBrake>(new Articulated_BrakeSimple);
  m_brakes[3] = ChSharedPtr<ChBrake>(new Articulated_BrakeSimple);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Articulated_Vehicle::Initialize(const ChCoordsys<>& chassisPos)
{
  m_chassis->SetFrame_REF_to_abs(ChFrame<>(chassisPos));

  // Initialize the steering subsystem (specify the steering subsystem's frame
  // relative to the chassis reference frame).
  ChVector<> offset;
  switch (m_suspType) {
  case SOLID_AXLE: offset = ChVector<>(1.60, 0, -0.07); break;
  case MULTI_LINK: offset = ChVector<>(1.65, 0, -0.12); break;
  }
  m_steering->Initialize(m_chassis, offset, ChQuaternion<>(1, 0, 0, 0));

  // Initialize the suspension subsystems (specify the suspension subsystems'
  // frames relative to the chassis reference frame).
  m_suspensions[0]->Initialize(m_chassis, ChVector<>(1.6914, 0, 0), m_steering->GetSteeringLink());
  m_suspensions[1]->Initialize(m_chassis, ChVector<>(-1.6865, 0, 0), m_chassis);

  // Initialize wheels
  m_wheels[0]->Initialize(m_suspensions[0]->GetSpindle(LEFT));
  m_wheels[1]->Initialize(m_suspensions[0]->GetSpindle(RIGHT));
  m_wheels[2]->Initialize(m_suspensions[1]->GetSpindle(LEFT));
  m_wheels[3]->Initialize(m_suspensions[1]->GetSpindle(RIGHT));

  // Initialize the driveline subsystem (RWD)
  std::vector<int> driven_susp(1, 1);
  m_driveline->Initialize(m_chassis, m_suspensions, driven_susp);

  // Initialize the four brakes
  m_brakes[0]->Initialize(m_suspensions[0]->GetRevolute(LEFT));
  m_brakes[1]->Initialize(m_suspensions[0]->GetRevolute(RIGHT));
  m_brakes[2]->Initialize(m_suspensions[1]->GetRevolute(LEFT));
  m_brakes[3]->Initialize(m_suspensions[1]->GetRevolute(RIGHT));
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double Articulated_Vehicle::GetSpringForce(const ChWheelID& wheel_id) const
{
  switch (m_suspType) {
  case SOLID_AXLE:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChSolidAxle>()->GetSpringForce(wheel_id.side());
  case MULTI_LINK:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChMultiLink>()->GetSpringForce(wheel_id.side());
  default:
    return -1;
  }
}

double Articulated_Vehicle::GetSpringLength(const ChWheelID& wheel_id) const
{
  switch (m_suspType) {
  case SOLID_AXLE:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChSolidAxle>()->GetSpringLength(wheel_id.side());
  case MULTI_LINK:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChMultiLink>()->GetSpringLength(wheel_id.side());
  default:
    return -1;
  }
}

double Articulated_Vehicle::GetSpringDeformation(const ChWheelID& wheel_id) const
{
  switch (m_suspType) {
  case SOLID_AXLE:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChSolidAxle>()->GetSpringDeformation(wheel_id.side());
  case MULTI_LINK:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChMultiLink>()->GetSpringDeformation(wheel_id.side());
  default:
    return -1;
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double Articulated_Vehicle::GetShockForce(const ChWheelID& wheel_id) const
{
  switch (m_suspType) {
  case SOLID_AXLE:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChSolidAxle>()->GetShockForce(wheel_id.side());
  case MULTI_LINK:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChMultiLink>()->GetShockForce(wheel_id.side());
  default:
    return -1;
  }
}

double Articulated_Vehicle::GetShockLength(const ChWheelID& wheel_id) const
{
  switch (m_suspType) {
  case SOLID_AXLE:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChSolidAxle>()->GetShockLength(wheel_id.side());
  case MULTI_LINK:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChMultiLink>()->GetShockLength(wheel_id.side());
  default:
    return -1;
  }
}

double Articulated_Vehicle::GetShockVelocity(const ChWheelID& wheel_id) const
{
  switch (m_suspType) {
  case SOLID_AXLE:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChSolidAxle>()->GetShockVelocity(wheel_id.side());
  case MULTI_LINK:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChMultiLink>()->GetShockVelocity(wheel_id.side());
  default:
    return -1;
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Articulated_Vehicle::Update(double              time,
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
  m_brakes[0]->ApplyBrakeModulation(braking);
  m_brakes[1]->ApplyBrakeModulation(braking);
  m_brakes[2]->ApplyBrakeModulation(braking);
  m_brakes[3]->ApplyBrakeModulation(braking);
}


// -----------------------------------------------------------------------------
// Log the hardpoint locations for the front-right and rear-right suspension
// subsystems (display in inches)
// -----------------------------------------------------------------------------
void Articulated_Vehicle::LogHardpointLocations()
{
  GetLog().SetNumFormat("%7.3f");

  switch (m_suspType) {
  case SOLID_AXLE:
    GetLog() << "\n---- FRONT suspension hardpoint locations (RIGHT side)\n";
    m_suspensions[0].StaticCastTo<ChSolidAxle>()->LogHardpointLocations(ChVector<>(0, 0, 0), true);
    GetLog() << "\n---- REAR suspension hardpoint locations (RIGHT side)\n";
    m_suspensions[1].StaticCastTo<ChSolidAxle>()->LogHardpointLocations(ChVector<>(0, 0, 0), true);
    break;
  case MULTI_LINK:
    GetLog() << "\n---- FRONT suspension hardpoint locations (RIGHT side)\n";
    m_suspensions[0].StaticCastTo<ChMultiLink>()->LogHardpointLocations(ChVector<>(0, 0, 0), true);
    GetLog() << "\n---- REAR suspension hardpoint locations (RIGHT side)\n";
    m_suspensions[1].StaticCastTo<ChMultiLink>()->LogHardpointLocations(ChVector<>(0, 0, 0), true);
    break;
  }

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
void Articulated_Vehicle::DebugLog(int what)
{
  GetLog().SetNumFormat("%10.2f");

  if (what & DBG_SPRINGS)
  {
    GetLog() << "\n---- Spring (front-left, front-right, rear-left, rear-right)\n";
    GetLog() << "Length [inch]       "
      << GetSpringLength(FRONT_LEFT) << "  "
      << GetSpringLength(FRONT_RIGHT) << "  "
      << GetSpringLength(REAR_LEFT) << "  "
      << GetSpringLength(REAR_RIGHT) << "\n";
    GetLog() << "Deformation [inch]  "
      << GetSpringDeformation(FRONT_LEFT) << "  "
      << GetSpringDeformation(FRONT_RIGHT) << "  "
      << GetSpringDeformation(REAR_LEFT) << "  "
      << GetSpringDeformation(REAR_RIGHT) << "\n";
    GetLog() << "Force [lbf]         "
      << GetSpringForce(FRONT_LEFT) << "  "
      << GetSpringForce(FRONT_RIGHT) << "  "
      << GetSpringForce(REAR_LEFT) << "  "
      << GetSpringForce(REAR_RIGHT) << "\n";
  }

  if (what & DBG_SHOCKS)
  {
    GetLog() << "\n---- Shock (front-left, front-right, rear-left, rear-right)\n";
    GetLog() << "Length [inch]       "
      << GetShockLength(FRONT_LEFT) << "  "
      << GetShockLength(FRONT_RIGHT) << "  "
      << GetShockLength(REAR_LEFT) << "  "
      << GetShockLength(REAR_RIGHT) << "\n";
    GetLog() << "Velocity [inch/s]   "
      << GetShockVelocity(FRONT_LEFT) << "  "
      << GetShockVelocity(FRONT_RIGHT) << "  "
      << GetShockVelocity(REAR_LEFT) << "  "
      << GetShockVelocity(REAR_RIGHT) << "\n";
    GetLog() << "Force [lbf]         "
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
