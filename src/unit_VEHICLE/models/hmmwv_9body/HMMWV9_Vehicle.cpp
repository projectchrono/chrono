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

#include "assets/ChBoxShape.h"
#include "assets/ChTriangleMeshShape.h"

#include "HMMWV9_Vehicle.h"


// TEMPORARY HACK
// 0:  no chassis visualization
// 1:  two boxes
// 2:  mesh
#define CHASSIS_VISUALIZATION 0

using namespace chrono;


// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double HMMWV9_Vehicle::m_chassisMass = 7500.0 / 2.2;

// Inertias, from my HMMWV model
// (roll, pitch, yaw) = (13320, 52680, 56280) lb-in-sec^2
const ChVector<> HMMWV9_Vehicle::m_chassisInertia(125.8, 497.4, 531.4);	// kg-m2;

const std::string HMMWV9_Vehicle::m_chassisMeshName = "hmmwv_chassis";
const std::string HMMWV9_Vehicle::m_chassisMeshFile = "../data/humvee4_scaled_rotated_decimated_centered.obj";


// helpful for unit conversions
static double in_to_m = 1.0/39.3701;	// inches to meters
static double inlb_to_Nm = 1.0/8.851;	// in-lb to N-m


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV9_Vehicle::HMMWV9_Vehicle(ChSystem&            my_system,
                               const ChCoordsys<>&  chassisPos,
                               const bool           fixed)
{
  // -------------------------------------------
  // Create the chassis body
  // -------------------------------------------

  m_chassis = ChSharedBodyPtr(new ChBody);

  m_chassis->SetIdentifier(0);
  m_chassis->SetName("chassis");
  m_chassis->SetMass(m_chassisMass);
  m_chassis->SetInertiaXX(m_chassisInertia);
  m_chassis->SetPos(chassisPos.pos);
  m_chassis->SetRot(chassisPos.rot);
  m_chassis->SetBodyFixed(fixed);

#if CHASSIS_VISUALIZATION == 1
  ChSharedPtr<ChBoxShape> box1(new ChBoxShape);
  box1->GetBoxGeometry().SetLenghts(ChVector<>(5, 1.7, 0.4));
  box1->Pos = ChVector<>(0, 0, -0.4);
  m_chassis->AddAsset(box1);
  ChSharedPtr<ChBoxShape> box2(new ChBoxShape);
  box2->GetBoxGeometry().SetLenghts(ChVector<>(4, 1.7, 0.4));
  box2->Pos = ChVector<>(0.5, 0, 0);
  m_chassis->AddAsset(box2);
#elif CHASSIS_VISUALIZATION == 2
  geometry::ChTriangleMeshConnected trimesh;
  trimesh.LoadWavefrontMesh(m_chassisMeshFile, false, false);
  ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
  trimesh_shape->SetMesh(trimesh);
  trimesh_shape->SetName(m_chassisMeshName);
  m_chassis->AddAsset(trimesh_shape);
#endif

  my_system.Add(m_chassis);

  // -------------------------------------------
  // Create the suspension subsystems
  // -------------------------------------------
  
  m_front_right_susp = ChSharedPtr<HMMWV9_DoubleWishboneFront>(new HMMWV9_DoubleWishboneFront("RFsusp"));
  m_front_right_susp->Initialize(m_chassis, ChVector<>(-83.8, 35.82, -33.325)*in_to_m, false);

  m_front_left_susp = ChSharedPtr<HMMWV9_DoubleWishboneFront>(new HMMWV9_DoubleWishboneFront("LFsusp"));
  m_front_left_susp->Initialize(m_chassis, ChVector<>(-83.8, -35.82, -33.325)*in_to_m, true);

  m_rear_right_susp = ChSharedPtr<HMMWV9_DoubleWishboneRear>(new HMMWV9_DoubleWishboneRear("RBsusp", true));
  m_rear_right_susp->Initialize(m_chassis, ChVector<>(46.2, 35.82, -33.325)*in_to_m, false);

  m_rear_left_susp = ChSharedPtr<HMMWV9_DoubleWishboneRear>(new HMMWV9_DoubleWishboneRear("RBsusp", true));
  m_rear_left_susp->Initialize(m_chassis, ChVector<>(46.2, -35.82, -33.325)*in_to_m, true);

  // -------------------------------------------
  // Create the wheels and attach to suspension
  // -------------------------------------------

  ChSharedPtr<HMMWV9_Wheel> front_right_wheel(new HMMWV9_Wheel(true));
  m_front_right_susp->AttachWheel(front_right_wheel);

  ChSharedPtr<HMMWV9_Wheel> front_left_wheel(new HMMWV9_Wheel(true));
  m_front_left_susp->AttachWheel(front_left_wheel);

  ChSharedPtr<HMMWV9_Wheel> rear_right_wheel(new HMMWV9_Wheel(true));
  m_rear_right_susp->AttachWheel(rear_right_wheel);

  ChSharedPtr<HMMWV9_Wheel> rear_left_wheel(new HMMWV9_Wheel(true));
  m_rear_left_susp->AttachWheel(rear_left_wheel);

  // -------------------------------
  // Create the powertrain subsystem
  //--------------------------------

  m_powertrain = ChSharedPtr<HMMWV9_Powertrain>(new HMMWV9_Powertrain(this));

}


HMMWV9_Vehicle::~HMMWV9_Vehicle()
{
}


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
double HMMWV9_Vehicle::GetWheelAngSpeed(ChWheelId which)
{
  switch (which) {
  case FRONT_LEFT:
    return m_front_left_susp->GetSpindleAngSpeed();
  case FRONT_RIGHT:
    return m_front_right_susp->GetSpindleAngSpeed();
  case REAR_LEFT:
    return m_rear_left_susp->GetSpindleAngSpeed();
  case REAR_RIGHT:
    return 0;
    return m_rear_right_susp->GetSpindleAngSpeed();
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV9_Vehicle::Update(double time,
                            double throttle,
                            double steering)
{
  // Apply steering input.
  double displ = 0.08 * steering;

  m_front_left_susp->ApplySteering(displ);
  m_front_right_susp->ApplySteering(displ);

  // Let the powertrain subsystem process the throttle input
  m_powertrain->Update(time, throttle);

  // Apply motor torques on driven wheels
  double wheelTorqueL = m_powertrain->GetWheelTorque(REAR_LEFT);
  double wheelTorqueR = m_powertrain->GetWheelTorque(REAR_RIGHT);

  m_rear_left_susp->ApplyTorque(wheelTorqueL);
  m_rear_right_susp->ApplyTorque(wheelTorqueR);

  // Debug...
  GetLog() << "motor torque=" << m_powertrain->GetMotorTorque()
           << "  motor speed=" << m_powertrain->GetMotorSpeed()
           << "  wheel torque=" << m_powertrain->GetWheelTorque(REAR_RIGHT)
           << "\n";

}

