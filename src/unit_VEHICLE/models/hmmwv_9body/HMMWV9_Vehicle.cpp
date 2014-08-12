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

#include "utils/ChUtilsData.h"

#include "HMMWV9_Vehicle.h"

using namespace chrono;

namespace hmmwv9 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;

const double HMMWV9_Vehicle::m_chassisMass = 7500.0 / 2.2;
const ChVector<> HMMWV9_Vehicle::m_chassisInertia(125.8, 497.4, 531.4); // chassis inertia (roll,pitch,yaw)

const std::string HMMWV9_Vehicle::m_chassisMeshName = "hmmwv_chassis";
const std::string HMMWV9_Vehicle::m_chassisMeshFile = utils::GetModelDataFile("hmmwv/humvee4_scaled_rotated_decimated_centered.obj");


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV9_Vehicle::HMMWV9_Vehicle(ChSystem&            my_system,
                               const ChCoordsys<>&  chassisPos,
                               const bool           fixed,
                               VisualizationType    chassisVis,
                               VisualizationType    wheelVis)
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

  switch (chassisVis) {
  case PRIMITIVES:
  {
    ChSharedPtr<ChBoxShape> box1(new ChBoxShape);
    box1->GetBoxGeometry().SetLenghts(ChVector<>(5, 1.7, 0.4));
    box1->Pos = ChVector<>(0, 0, -0.4);
    m_chassis->AddAsset(box1);

    ChSharedPtr<ChBoxShape> box2(new ChBoxShape);
    box2->GetBoxGeometry().SetLenghts(ChVector<>(4, 1.7, 0.4));
    box2->Pos = ChVector<>(0.5, 0, 0);
    m_chassis->AddAsset(box2);

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

  my_system.Add(m_chassis);

  // -------------------------------------------
  // Create the suspension subsystems
  // -------------------------------------------

  m_front_right_susp = ChSharedPtr<HMMWV9_DoubleWishboneFront>(new HMMWV9_DoubleWishboneFront("FRsusp", ChSuspension::RIGHT));
  m_front_right_susp->Initialize(m_chassis, in2m * ChVector<>(-83.8, 35.82, -19.949));

  m_front_left_susp = ChSharedPtr<HMMWV9_DoubleWishboneFront>(new HMMWV9_DoubleWishboneFront("FLsusp", ChSuspension::LEFT));
  m_front_left_susp->Initialize(m_chassis, in2m * ChVector<>(-83.8, -35.82, -19.949));

  m_rear_right_susp = ChSharedPtr<HMMWV9_DoubleWishboneRear>(new HMMWV9_DoubleWishboneRear("RRsusp", ChSuspension::RIGHT, true));
  m_rear_right_susp->Initialize(m_chassis, in2m * ChVector<>(46.2, 35.82, -19.949));

  m_rear_left_susp = ChSharedPtr<HMMWV9_DoubleWishboneRear>(new HMMWV9_DoubleWishboneRear("RLsusp", ChSuspension::LEFT, true));
  m_rear_left_susp->Initialize(m_chassis, in2m * ChVector<>(46.2, -35.82, -19.949));

  // -------------------------------------------
  // Create the wheels and attach to suspension
  // -------------------------------------------

  ChSharedPtr<HMMWV9_Wheel> front_right_wheel(new HMMWV9_Wheel(true, 0.7, wheelVis));
  m_front_right_susp->AttachWheel(front_right_wheel);

  ChSharedPtr<HMMWV9_Wheel> front_left_wheel(new HMMWV9_Wheel(true, 0.7, wheelVis));
  m_front_left_susp->AttachWheel(front_left_wheel);

  ChSharedPtr<HMMWV9_Wheel> rear_right_wheel(new HMMWV9_Wheel(true, 0.7, wheelVis));
  m_rear_right_susp->AttachWheel(rear_right_wheel);

  ChSharedPtr<HMMWV9_Wheel> rear_left_wheel(new HMMWV9_Wheel(true, 0.7, wheelVis));
  m_rear_left_susp->AttachWheel(rear_left_wheel);

  // -------------------------------
  // Create the powertrain subsystem
  //--------------------------------

  ////m_powertrain = ChSharedPtr<HMMWV9_SimplePowertrain>(new HMMWV9_SimplePowertrain(this));
  ////m_powertrain->Initialize(m_chassis, m_rear_left_susp, m_rear_right_susp);

  m_powertrain = ChSharedPtr<HMMWV9_Powertrain>(new HMMWV9_Powertrain(this));
  m_powertrain->Initialize(m_chassis, m_rear_left_susp->GetSpindle(), m_rear_right_susp->GetSpindle());

}


HMMWV9_Vehicle::~HMMWV9_Vehicle()
{
}


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const ChVector<>& HMMWV9_Vehicle::GetWheelPos(ChWheelId which) const
{
  switch (which) {
  case FRONT_LEFT:
    return m_front_left_susp->GetSpindlePos();
  case FRONT_RIGHT:
    return m_front_right_susp->GetSpindlePos();
  case REAR_LEFT:
    return m_rear_left_susp->GetSpindlePos();
  case REAR_RIGHT:
    return m_rear_right_susp->GetSpindlePos();
  }
}

const ChQuaternion<>& HMMWV9_Vehicle::GetWheelRot(ChWheelId which) const
{
  switch (which) {
  case FRONT_LEFT:
    return m_front_left_susp->GetSpindleRot();
  case FRONT_RIGHT:
    return m_front_right_susp->GetSpindleRot();
  case REAR_LEFT:
    return m_rear_left_susp->GetSpindleRot();
  case REAR_RIGHT:
    return m_rear_right_susp->GetSpindleRot();
  }
}

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
}


} // end namespace hmmwv9
