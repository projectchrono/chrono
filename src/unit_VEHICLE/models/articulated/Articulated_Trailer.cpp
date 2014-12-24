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
// Authors: Alessandro Tasora
// =============================================================================
//
// Trailer for articulated vehicle model.
//
// =============================================================================

#include "assets/ChSphereShape.h"
#include "assets/ChTriangleMeshShape.h"

#include "subsys/ChVehicleModelData.h"

#include "utils/ChUtilsInputOutput.h"

#include "models/articulated/Articulated_SolidAxle.h"
#include "models/articulated/Articulated_MultiLink.h"

#include "models/articulated/Articulated_Vehicle.h"
#include "models/articulated/Articulated_Trailer.h"

using namespace chrono;


// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double     Articulated_Trailer::m_chassisMass = 3500;                   // chassis sprung mass
const ChVector<> Articulated_Trailer::m_chassisCOM (-0.2, 0, 0.8);            // COM location
const ChVector<> Articulated_Trailer::m_chassisInertia(125.8, 497.4, 531.4);  // chassis inertia (roll,pitch,yaw)
const double     Articulated_Trailer::m_frontaxleMass = 500;                    // frontaxle sprung mass
const ChVector<> Articulated_Trailer::m_frontaxleREF( 2, 0, 0);             // frontaxle ref-chassis relative location
const ChVector<> Articulated_Trailer::m_frontaxleCOM( 0.2, 0, 0);             // frontaxle COM cog-ref relative location
const ChVector<> Articulated_Trailer::m_frontaxleInertia(50.4, 10.4, 50.4);  // frontaxle inertia (roll,pitch,yaw)

const ChVector<> Articulated_Trailer::m_frontaxleSphericalJoint(2.0, 0, 0.5);  // joint between frontaxle and trailer chassis
const ChVector<> Articulated_Trailer::m_frontaxlePullerJoint(3.0, 0, 0.6);  // joint between frontaxle and trailer chassis

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Articulated_Trailer::Articulated_Trailer(ChSystem*         mysystem,
                                         const bool        fixed,
                                         SuspensionType    suspType,
                                         VisualizationType wheelVis)
: m_suspType(suspType)
{
  // -------------------------------------------
  // Create the chassis body
  // -------------------------------------------
  m_chassis = ChSharedPtr<ChBodyAuxRef>(new ChBodyAuxRef);

  m_chassis->SetIdentifier(100);
  m_chassis->SetName("trailer");
  m_chassis->SetMass(m_chassisMass);
  m_chassis->SetFrame_COG_to_REF(ChFrame<>(m_chassisCOM, ChQuaternion<>(1, 0, 0, 0)));
  m_chassis->SetInertiaXX(m_chassisInertia);
  m_chassis->SetBodyFixed(fixed);

  ChSharedPtr<ChSphereShape> sphere(new ChSphereShape);
  sphere->GetSphereGeometry().rad = 0.1;
  sphere->Pos = m_chassisCOM;
  m_chassis->AddAsset(sphere);

  mysystem->Add(m_chassis);

  // -------------------------------------------
  // Create the front steering axle body
  // -------------------------------------------
  m_frontaxle = ChSharedPtr<ChBodyAuxRef>(new ChBodyAuxRef);

  m_frontaxle->SetIdentifier(101);
  m_frontaxle->SetMass(m_frontaxleMass);
  m_frontaxle->SetFrame_REF_to_abs(ChFrame<>(m_frontaxleREF, ChQuaternion<>(1, 0, 0, 0)));
  m_frontaxle->SetFrame_COG_to_REF(ChFrame<>(m_frontaxleCOM, ChQuaternion<>(1, 0, 0, 0)));
  m_frontaxle->SetInertiaXX(m_frontaxleInertia);
  m_frontaxle->SetBodyFixed(fixed);

  ChSharedPtr<ChSphereShape> sphereB(new ChSphereShape);
  sphereB->GetSphereGeometry().rad = 0.1;
  sphereB->Pos = m_frontaxleCOM;
  m_frontaxle->AddAsset(sphereB);
  ChSharedPtr<ChBoxShape> boxB(new ChBoxShape);
  boxB->GetBoxGeometry().SetLengths(ChVector<>(0.1, 1.5, 0.1));
  m_frontaxle->AddAsset(boxB);

  mysystem->Add(m_frontaxle);

  // -------------------------------------------
  // Create the frontaxle - trailer chassis joint
  // -------------------------------------------
  
  m_joint = ChSharedPtr<ChLinkLockSpherical>(new ChLinkLockSpherical);
  m_joint->Initialize(this->m_chassis, this->m_frontaxle, ChCoordsys<>(this->m_frontaxleSphericalJoint));
  mysystem->Add(m_joint);





  // -------------------------------------------
  // Create the suspension subsystems
  // -------------------------------------------
  m_suspensions.resize(2);

  assert(m_suspType == SOLID_AXLE || m_suspType == MULTI_LINK);

  switch (m_suspType) {
  case SOLID_AXLE:
    m_suspensions[0] = ChSharedPtr<ChSuspension>(new Articulated_SolidAxleRear("FrontSusp"));
    m_suspensions[1] = ChSharedPtr<ChSuspension>(new Articulated_SolidAxleRear("RearSusp"));
    break;
  case MULTI_LINK:
    m_suspensions[0] = ChSharedPtr<ChSuspension>(new Articulated_MultiLinkRear("FrontSusp"));
    m_suspensions[1] = ChSharedPtr<ChSuspension>(new Articulated_MultiLinkRear("RearSusp"));
    break;
  }

  // -----------------
  // Create the wheels
  // -----------------
  m_front_right_wheel = ChSharedPtr<Articulated_Wheel>(new Articulated_Wheel(wheelVis));
  m_front_left_wheel = ChSharedPtr<Articulated_Wheel>(new Articulated_Wheel(wheelVis));
  m_rear_right_wheel = ChSharedPtr<Articulated_Wheel>(new Articulated_Wheel(wheelVis));
  m_rear_left_wheel = ChSharedPtr<Articulated_Wheel>(new Articulated_Wheel(wheelVis));


  // -----------------
  // Create the brakes
  // -----------------
  m_front_right_brake = ChSharedPtr<Articulated_BrakeSimple>(new Articulated_BrakeSimple);
  m_front_left_brake = ChSharedPtr<Articulated_BrakeSimple>(new Articulated_BrakeSimple);
  m_rear_right_brake = ChSharedPtr<Articulated_BrakeSimple>(new Articulated_BrakeSimple);
  m_rear_left_brake = ChSharedPtr<Articulated_BrakeSimple>(new Articulated_BrakeSimple);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Articulated_Trailer::Initialize(const ChCoordsys<>& chassisPos, 
                                     const bool          connect_to_puller,
                                     chrono::ChSharedPtr<chrono::ChBodyAuxRef> pulling_vehicle)
{
  //*m_chassis.get_ptr() >>= chassisPos;
  //*m_frontaxle.get_ptr() >>= chassisPos;
  //m_chassis->ConcatenatePreTransformation(ChFrameMoving<>(chassisPos));
  //m_frontaxle->ConcatenatePreTransformation(ChFrameMoving<>(chassisPos));
  m_chassis->SetFrame_REF_to_abs(ChFrame<>(chassisPos));
  m_frontaxle->SetFrame_REF_to_abs(ChFrame<>(m_frontaxleREF >> chassisPos));


  // -------------------------------------------
  // Create the frontaxle - puller chassis joint
  // -------------------------------------------

  if (connect_to_puller)
  {
    m_puller = ChSharedPtr<ChLinkLockSpherical>(new ChLinkLockSpherical);
    m_puller->Initialize(this->m_frontaxle, pulling_vehicle,
      ChCoordsys<>(this->m_frontaxlePullerJoint) >> chassisPos);
    pulling_vehicle->GetSystem()->Add(m_puller);
  }


  // Initialize the suspension subsystems (specify the suspension subsystems'
  // frames relative to the chassis reference frame).
  m_suspensions[0]->Initialize(m_frontaxle, ChVector<>(0, 0, 0), m_frontaxle);
  m_suspensions[1]->Initialize(m_chassis, ChVector<>(-2, 0, 0),  m_chassis);
  

  // Initialize wheels
  m_front_left_wheel->Initialize(m_suspensions[0]->GetSpindle(LEFT));
  m_front_right_wheel->Initialize(m_suspensions[0]->GetSpindle(RIGHT));
  m_rear_left_wheel->Initialize(m_suspensions[1]->GetSpindle(LEFT));
  m_rear_right_wheel->Initialize(m_suspensions[1]->GetSpindle(RIGHT));

  // Initialize the four brakes
  m_front_left_brake->Initialize(m_suspensions[0]->GetRevolute(LEFT));
  m_front_right_brake->Initialize(m_suspensions[0]->GetRevolute(RIGHT));
  m_rear_left_brake->Initialize(m_suspensions[1]->GetRevolute(LEFT));
  m_rear_right_brake->Initialize(m_suspensions[1]->GetRevolute(RIGHT));
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double Articulated_Trailer::GetSpringForce(const ChWheelID& wheel_id) const
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

double Articulated_Trailer::GetSpringLength(const ChWheelID& wheel_id) const
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

double Articulated_Trailer::GetSpringDeformation(const ChWheelID& wheel_id) const
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
double Articulated_Trailer::GetShockForce(const ChWheelID& wheel_id) const
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

double Articulated_Trailer::GetShockLength(const ChWheelID& wheel_id) const
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

double Articulated_Trailer::GetShockVelocity(const ChWheelID& wheel_id) const
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
void Articulated_Trailer::Update(double              time,
                             double              braking,
                             const ChTireForces& tire_forces)
{
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
// Log the hardpoint locations for the front-right and rear-right suspension
// subsystems (display in inches)
// -----------------------------------------------------------------------------
void Articulated_Trailer::LogHardpointLocations()
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
void Articulated_Trailer::DebugLog(int what)
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

  GetLog().SetNumFormat("%g");
}


ChSharedPtr<ChBody> Articulated_Trailer::GetWheelBody(const ChWheelID& wheel_id) const
{
  return m_suspensions[wheel_id.axle()]->GetSpindle(wheel_id.side());
}
