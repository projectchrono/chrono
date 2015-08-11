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
// Generic 2-axle vehicle model.
// Can be constructed either with solid-axle or with multi-link suspensions.
// Always uses a generic rack-pinion steering and a 2WD driveline model.
//
// =============================================================================

#include "assets/ChSphereShape.h"
#include "assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_vehicle_utils/ChUtilsInputOutput.h"

#include "generic/Generic_Vehicle.h"

#include "generic/Generic_SolidAxle.h"
#include "generic/Generic_MultiLink.h"
#include "generic/Generic_DoubleWishbone.h"
#include "generic/Generic_HendricksonPRIMAXX.h"

#include "generic/Generic_AntirollBarRSD.h"

#include "generic/Generic_Wheel.h"
#include "generic/Generic_RackPinion.h"
#include "generic/Generic_Driveline2WD.h"
#include "generic/Generic_BrakeSimple.h"

using namespace chrono;


// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double     Generic_Vehicle::m_chassisMass = 3500;                   // chassis sprung mass
const ChVector<> Generic_Vehicle::m_chassisCOM (-0.2, 0, 0.8);            // COM location
const ChVector<> Generic_Vehicle::m_chassisInertia(125.8, 497.4, 531.4);  // chassis inertia (roll,pitch,yaw)

const ChCoordsys<> Generic_Vehicle::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Generic_Vehicle::Generic_Vehicle(const bool        fixed,
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

  switch (m_suspType) {
  case SOLID_AXLE:
    m_suspensions[0] = ChSharedPtr<ChSuspension>(new Generic_SolidAxle("FrontSusp"));
    m_suspensions[1] = ChSharedPtr<ChSuspension>(new Generic_SolidAxle("RearSusp"));
    break;
  case MULTI_LINK:
    m_suspensions[0] = ChSharedPtr<ChSuspension>(new Generic_MultiLink("FrontSusp"));
    m_suspensions[1] = ChSharedPtr<ChSuspension>(new Generic_MultiLink("RearSusp"));
    break;
  case DOUBLE_WISHBONE:
    m_suspensions[0] = ChSharedPtr<ChSuspension>(new Generic_DoubleWishbone("Front suspension"));
    m_suspensions[1] = ChSharedPtr<ChSuspension>(new Generic_DoubleWishbone("Rear suspension"));
  case HENDRICKSON_PRIMAXX:
    m_suspensions[0] = ChSharedPtr<ChSuspension>(new Generic_HendricksonPRIMAXX("Front suspension"));
    m_suspensions[1] = ChSharedPtr<ChSuspension>(new Generic_HendricksonPRIMAXX("Rear suspension"));
  }

  // --------------------------------
  // Create the antirollbar subsystem
  // --------------------------------
  ////if (m_suspensions[0]->IsIndependent()) {
  ////  m_antirollbars.resize(1);
  ////  m_antirollbars[0] = ChSharedPtr<Generic_AntirollBarRSD>(new Generic_AntirollBarRSD("Antiroll Bar"));
  ////}

  // -----------------------------
  // Create the steering subsystem
  // -----------------------------
  m_steerings.resize(1);
  m_steerings[0] = ChSharedPtr<ChSteering>(new Generic_RackPinion("Steering"));

  // -----------------
  // Create the wheels
  // -----------------
  m_wheels.resize(4);
  m_wheels[0] = ChSharedPtr<ChWheel>(new Generic_Wheel(wheelVis));
  m_wheels[1] = ChSharedPtr<ChWheel>(new Generic_Wheel(wheelVis));
  m_wheels[2] = ChSharedPtr<ChWheel>(new Generic_Wheel(wheelVis));
  m_wheels[3] = ChSharedPtr<ChWheel>(new Generic_Wheel(wheelVis));

  // --------------------
  // Create the driveline
  // --------------------
  m_driveline = ChSharedPtr<ChDriveline>(new Generic_Driveline2WD);

  // -----------------
  // Create the brakes
  // -----------------
  m_brakes.resize(4);
  m_brakes[0] = ChSharedPtr<ChBrake>(new Generic_BrakeSimple);
  m_brakes[1] = ChSharedPtr<ChBrake>(new Generic_BrakeSimple);
  m_brakes[2] = ChSharedPtr<ChBrake>(new Generic_BrakeSimple);
  m_brakes[3] = ChSharedPtr<ChBrake>(new Generic_BrakeSimple);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Generic_Vehicle::Initialize(const ChCoordsys<>& chassisPos)
{
  m_chassis->SetFrame_REF_to_abs(ChFrame<>(chassisPos));

  // Initialize the steering subsystem (specify the steering subsystem's frame
  // relative to the chassis reference frame).
  ChVector<> offset;
  switch (m_suspType) {
  case SOLID_AXLE:      offset = ChVector<>(1.60, 0, -0.07); break;
  case MULTI_LINK:      offset = ChVector<>(1.65, 0, -0.12); break;
  case DOUBLE_WISHBONE: offset = ChVector<>(1.4, 0, -0.03); break;
  }
  m_steerings[0]->Initialize(m_chassis, offset, ChQuaternion<>(1, 0, 0, 0));

  // Initialize the suspension subsystems (specify the suspension subsystems'
  // frames relative to the chassis reference frame).
  m_suspensions[0]->Initialize(m_chassis, ChVector<>(1.6914, 0, 0), m_steerings[0]->GetSteeringLink());
  m_suspensions[1]->Initialize(m_chassis, ChVector<>(-1.6865, 0, 0), m_chassis);

  // Initialize the antiroll bar subsystem.
  ////if (m_antirollbars.size() == 1) {
  ////  m_antirollbars[0]->Initialize(m_chassis,
  ////                               ChVector<>(1.3, 0, 0.0),
  ////                               m_suspensions[0]->GetLeftBody(),
  ////                               m_suspensions[0]->GetRightBody());
  ////}

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
double Generic_Vehicle::GetSpringForce(const ChWheelID& wheel_id) const
{
  switch (m_suspType) {
  case SOLID_AXLE:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChSolidAxle>()->GetSpringForce(wheel_id.side());
  case MULTI_LINK:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChMultiLink>()->GetSpringForce(wheel_id.side());
  case DOUBLE_WISHBONE:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChDoubleWishbone>()->GetSpringForce(wheel_id.side());
  default:
    return -1;
  }
}

double Generic_Vehicle::GetSpringLength(const ChWheelID& wheel_id) const
{
  switch (m_suspType) {
  case SOLID_AXLE:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChSolidAxle>()->GetSpringLength(wheel_id.side());
  case MULTI_LINK:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChMultiLink>()->GetSpringLength(wheel_id.side());
  case DOUBLE_WISHBONE:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChDoubleWishbone>()->GetSpringLength(wheel_id.side());
  default:
    return -1;
  }
}

double Generic_Vehicle::GetSpringDeformation(const ChWheelID& wheel_id) const
{
  switch (m_suspType) {
  case SOLID_AXLE:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChSolidAxle>()->GetSpringDeformation(wheel_id.side());
  case MULTI_LINK:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChMultiLink>()->GetSpringDeformation(wheel_id.side());
  case DOUBLE_WISHBONE:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChDoubleWishbone>()->GetSpringDeformation(wheel_id.side());
  default:
    return -1;
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double Generic_Vehicle::GetShockForce(const ChWheelID& wheel_id) const
{
  switch (m_suspType) {
  case SOLID_AXLE:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChSolidAxle>()->GetShockForce(wheel_id.side());
  case MULTI_LINK:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChMultiLink>()->GetShockForce(wheel_id.side());
  case DOUBLE_WISHBONE:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChDoubleWishbone>()->GetShockForce(wheel_id.side());
  default:
    return -1;
  }
}

double Generic_Vehicle::GetShockLength(const ChWheelID& wheel_id) const
{
  switch (m_suspType) {
  case SOLID_AXLE:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChSolidAxle>()->GetShockLength(wheel_id.side());
  case MULTI_LINK:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChMultiLink>()->GetShockLength(wheel_id.side());
  case DOUBLE_WISHBONE:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChDoubleWishbone>()->GetShockLength(wheel_id.side());
  default:
    return -1;
  }
}

double Generic_Vehicle::GetShockVelocity(const ChWheelID& wheel_id) const
{
  switch (m_suspType) {
  case SOLID_AXLE:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChSolidAxle>()->GetShockVelocity(wheel_id.side());
  case MULTI_LINK:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChMultiLink>()->GetShockVelocity(wheel_id.side());
  case DOUBLE_WISHBONE:
    return m_suspensions[wheel_id.axle()].StaticCastTo<ChDoubleWishbone>()->GetShockVelocity(wheel_id.side());
  default:
    return -1;
  }
}


// -----------------------------------------------------------------------------
// Log the hardpoint locations for the front-right and rear-right suspension
// subsystems (display in inches)
// -----------------------------------------------------------------------------
void Generic_Vehicle::LogHardpointLocations()
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
  case DOUBLE_WISHBONE:
    GetLog() << "\n---- FRONT suspension hardpoint locations (RIGHT side)\n";
    m_suspensions[0].StaticCastTo<ChDoubleWishbone>()->LogHardpointLocations(ChVector<>(0, 0, 0), true);
    GetLog() << "\n---- REAR suspension hardpoint locations (RIGHT side)\n";
    m_suspensions[1].StaticCastTo<ChDoubleWishbone>()->LogHardpointLocations(ChVector<>(0, 0, 0), true);
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
void Generic_Vehicle::DebugLog(int what)
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
