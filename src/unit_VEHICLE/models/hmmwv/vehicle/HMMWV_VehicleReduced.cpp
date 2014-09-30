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

  // -----------------
  // Create the wheels
  // -----------------

  m_front_right_wheel = ChSharedPtr<HMMWV_Wheel>(new HMMWV_WheelRight(wheelVis));
  m_front_left_wheel = ChSharedPtr<HMMWV_Wheel>(new HMMWV_WheelLeft(wheelVis));
  m_rear_right_wheel = ChSharedPtr<HMMWV_Wheel>(new HMMWV_WheelRight(wheelVis));
  m_rear_left_wheel = ChSharedPtr<HMMWV_Wheel>(new HMMWV_WheelLeft(wheelVis));

  // ----------------------------------------------
  // Create the driveline and powertrain subsystems
  // ----------------------------------------------

  m_driveline = ChSharedPtr<HMMWV_Driveline2WD>(new HMMWV_Driveline2WD(this));
  m_powertrain = ChSharedPtr<HMMWV_Powertrain>(new HMMWV_Powertrain(this));

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

  // Initialize the suspension subsystems (specify the suspension subsystems'
  // frames relative to the chassis reference frame).
  m_front_susp->Initialize(m_chassis, in2m * ChVector<>(66.59, 0, 1.039));
  m_rear_susp->Initialize(m_chassis, in2m * ChVector<>(-66.4, 0, 1.039));

  // Initialize wheels
  m_front_right_wheel->Initialize(m_front_susp->GetSpindle(ChSuspension::RIGHT));
  m_front_left_wheel->Initialize(m_front_susp->GetSpindle(ChSuspension::LEFT));
  m_rear_right_wheel->Initialize(m_rear_susp->GetSpindle(ChSuspension::RIGHT));
  m_rear_left_wheel->Initialize(m_rear_susp->GetSpindle(ChSuspension::LEFT));

  // Initialize the driveline subsystem (RWD)
  m_driveline->Initialize(m_chassis, m_rear_susp->GetAxle(ChSuspension::LEFT), m_rear_susp->GetAxle(ChSuspension::RIGHT));

  // Initialize the powertrain subsystem
  m_powertrain->Initialize(m_chassis, m_driveline->GetDriveshaft());

  // Initialize the four brakes
  m_front_right_brake->Initialize(m_front_susp->GetRevolute(ChSuspension::RIGHT));
  m_front_left_brake->Initialize(m_front_susp->GetRevolute(ChSuspension::LEFT));
  m_rear_right_brake->Initialize(m_rear_susp->GetRevolute(ChSuspension::RIGHT));
  m_rear_left_brake->Initialize(m_rear_susp->GetRevolute(ChSuspension::LEFT));
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSharedBodyPtr HMMWV_VehicleReduced::GetWheelBody(ChWheelId which) const
{
  switch (which) {
  case FRONT_LEFT:
    return m_front_susp->GetSpindle(ChSuspension::LEFT);
  case FRONT_RIGHT:
    return m_front_susp->GetSpindle(ChSuspension::RIGHT);
  case REAR_LEFT:
    return m_rear_susp->GetSpindle(ChSuspension::LEFT);
  case REAR_RIGHT:
    return m_rear_susp->GetSpindle(ChSuspension::RIGHT);
  default:
    return m_front_susp->GetSpindle(ChSuspension::RIGHT);  // should not happen
  }
}

const ChVector<>& HMMWV_VehicleReduced::GetWheelPos(ChWheelId which) const
{
  switch (which) {
  case FRONT_LEFT:
    return m_front_susp->GetSpindlePos(ChSuspension::LEFT);
  case FRONT_RIGHT:
    return m_front_susp->GetSpindlePos(ChSuspension::RIGHT);
  case REAR_LEFT:
    return m_rear_susp->GetSpindlePos(ChSuspension::LEFT);
  case REAR_RIGHT:
    return m_rear_susp->GetSpindlePos(ChSuspension::RIGHT);
  default:
    return m_front_susp->GetSpindlePos(ChSuspension::RIGHT);  // should not happen
  }
}

const ChQuaternion<>& HMMWV_VehicleReduced::GetWheelRot(ChWheelId which) const
{
  switch (which) {
  case FRONT_LEFT:
    return m_front_susp->GetSpindleRot(ChSuspension::LEFT);
  case FRONT_RIGHT:
    return m_front_susp->GetSpindleRot(ChSuspension::RIGHT);
  case REAR_LEFT:
    return m_rear_susp->GetSpindleRot(ChSuspension::LEFT);
  case REAR_RIGHT:
    return m_rear_susp->GetSpindleRot(ChSuspension::RIGHT);
  default:
    return m_front_susp->GetSpindleRot(ChSuspension::RIGHT);  // should not happen
  }
}

const ChVector<>& HMMWV_VehicleReduced::GetWheelLinVel(ChWheelId which) const
{
  switch (which) {
  case FRONT_LEFT:
    return m_front_susp->GetSpindleLinVel(ChSuspension::LEFT);
  case FRONT_RIGHT:
    return m_front_susp->GetSpindleLinVel(ChSuspension::RIGHT);
  case REAR_LEFT:
    return m_rear_susp->GetSpindleLinVel(ChSuspension::LEFT);
  case REAR_RIGHT:
    return m_rear_susp->GetSpindleLinVel(ChSuspension::RIGHT);
  default:
    return m_front_susp->GetSpindleLinVel(ChSuspension::RIGHT);  // should not happen
  }
}

ChVector<> HMMWV_VehicleReduced::GetWheelAngVel(ChWheelId which) const
{
  switch (which) {
  case FRONT_LEFT:
    return m_front_susp->GetSpindleAngVel(ChSuspension::LEFT);
  case FRONT_RIGHT:
    return m_front_susp->GetSpindleAngVel(ChSuspension::RIGHT);
  case REAR_LEFT:
    return m_rear_susp->GetSpindleAngVel(ChSuspension::LEFT);
  case REAR_RIGHT:
    return m_rear_susp->GetSpindleAngVel(ChSuspension::RIGHT);
  default:
    return m_front_susp->GetSpindleAngVel(ChSuspension::RIGHT);  // should not happen
  }
}

double HMMWV_VehicleReduced::GetWheelOmega(ChWheelId which) const
{
  switch (which) {
  case FRONT_LEFT:
    return m_front_susp->GetAxleSpeed(ChSuspension::LEFT);
  case FRONT_RIGHT:
    return m_front_susp->GetAxleSpeed(ChSuspension::RIGHT);
  case REAR_LEFT:
    return m_rear_susp->GetAxleSpeed(ChSuspension::LEFT);
  case REAR_RIGHT:
    return m_rear_susp->GetAxleSpeed(ChSuspension::RIGHT);
  default:
    return -1;  // should not happen
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_VehicleReduced::Update(double              time,
                                  double              throttle,
                                  double              steering,
                                  double              braking,
                                  const ChTireForces& tire_forces)
{
  // Apply steering input.
  double displ = 0.08 * steering;

  m_front_susp->ApplySteering(displ);

  // Let the powertrain subsystem process the throttle input.
  m_powertrain->Update(time, throttle);

  // Apply tire forces to spindle bodies.
  m_front_susp->ApplyTireForce(ChSuspension::RIGHT, tire_forces[FRONT_RIGHT]);
  m_front_susp->ApplyTireForce(ChSuspension::LEFT, tire_forces[FRONT_LEFT]);
  m_rear_susp->ApplyTireForce(ChSuspension::RIGHT, tire_forces[REAR_RIGHT]);
  m_rear_susp->ApplyTireForce(ChSuspension::LEFT, tire_forces[REAR_LEFT]);

  // Apply braking
  m_front_right_brake->ApplyBrakeModulation(braking);
  m_front_left_brake->ApplyBrakeModulation(braking);
  m_rear_right_brake->ApplyBrakeModulation(braking);
  m_rear_left_brake->ApplyBrakeModulation(braking);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_VehicleReduced::ExportMeshPovray(const std::string& out_dir)
{
  utils::WriteMeshPovray(m_chassisMeshFile, m_chassisMeshName, out_dir);
}


} // end namespace hmmwv
