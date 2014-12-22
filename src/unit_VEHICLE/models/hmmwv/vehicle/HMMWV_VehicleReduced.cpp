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

#include "subsys/ChVehicleModelData.h"

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

const std::string HMMWV_VehicleReduced::m_chassisMeshName = "hmmwv_chassis_POV_geom";
const std::string HMMWV_VehicleReduced::m_chassisMeshFile = vehicle::GetDataFile("hmmwv/hmmwv_chassis.obj");

const ChCoordsys<> HMMWV_VehicleReduced::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_VehicleReduced::HMMWV_VehicleReduced(const bool           fixed,
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
  m_suspensions[0] = ChSharedPtr<ChSuspension>(new HMMWV_DoubleWishboneReducedFront("FrontSusp"));
  m_suspensions[1] = ChSharedPtr<ChSuspension>(new HMMWV_DoubleWishboneReducedRear("RearSusp"));

  // -----------------------------
  // Create the steering subsystem
  // -----------------------------
  m_steering = ChSharedPtr<ChSteering>(new HMMWV_RackPinion("Steering"));

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
  m_suspensions[0]->Initialize(m_chassis, in2m * ChVector<>(66.59, 0, 1.039), m_steering->GetSteeringLink());
  m_suspensions[1]->Initialize(m_chassis, in2m * ChVector<>(-66.4, 0, 1.039), m_chassis);

  // Initialize wheels
  m_wheels[0]->Initialize(m_suspensions[0]->GetSpindle(LEFT));
  m_wheels[1]->Initialize(m_suspensions[0]->GetSpindle(RIGHT));
  m_wheels[2]->Initialize(m_suspensions[1]->GetSpindle(LEFT));
  m_wheels[3]->Initialize(m_suspensions[1]->GetSpindle(RIGHT));

  // Initialize the driveline subsystem.
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
void HMMWV_VehicleReduced::ExportMeshPovray(const std::string& out_dir)
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


} // end namespace hmmwv
