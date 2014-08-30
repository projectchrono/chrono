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

#include "assets/ChBoxShape.h"
#include "assets/ChTriangleMeshShape.h"

#include "utils/ChUtilsData.h"
#include "utils/ChUtilsInputOutput.h"

#include "models/hmmwv/vehicle/HMMWV_Vehicle.h"

using namespace chrono;

namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;

const double     HMMWV_Vehicle::m_chassisMass = 7747.0 / 2.2;	// chassis sprung mass
const ChVector<> HMMWV_Vehicle::m_chassisInertia(125.8, 497.4, 531.4); // chassis inertia (roll,pitch,yaw)

const std::string HMMWV_Vehicle::m_chassisMeshName = "hmmwv_chassis";
const std::string HMMWV_Vehicle::m_chassisMeshFile = utils::GetModelDataFile("hmmwv/humvee4_scaled_rotated_decimated_centered.obj");

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_Vehicle::HMMWV_Vehicle(const bool           fixed,
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
  m_chassis->SetBodyFixed(fixed);

  switch (chassisVis) {
  case PRIMITIVES:
  {
    ChSharedPtr<ChBoxShape> box1(new ChBoxShape);
    box1->GetBoxGeometry().SetLengths(ChVector<>(5, 1.7, 0.4));
    box1->Pos = ChVector<>(0, 0, -0.4);
    m_chassis->AddAsset(box1);

    ChSharedPtr<ChBoxShape> box2(new ChBoxShape);
    box2->GetBoxGeometry().SetLengths(ChVector<>(4, 1.7, 0.4));
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

  Add(m_chassis);

  // -------------------------------------------
  // Create the suspension subsystems
  // -------------------------------------------

  m_front_right_susp = ChSharedPtr<HMMWV_DoubleWishboneFront>(new HMMWV_DoubleWishboneFront("FRsusp", ChSuspension::RIGHT));
  m_front_left_susp = ChSharedPtr<HMMWV_DoubleWishboneFront>(new HMMWV_DoubleWishboneFront("FLsusp", ChSuspension::LEFT));
  m_rear_right_susp = ChSharedPtr<HMMWV_DoubleWishboneRear>(new HMMWV_DoubleWishboneRear("RRsusp", ChSuspension::RIGHT, true));
  m_rear_left_susp = ChSharedPtr<HMMWV_DoubleWishboneRear>(new HMMWV_DoubleWishboneRear("RLsusp", ChSuspension::LEFT, true));

  // -----------------
  // Create the wheels
  // -----------------

  m_front_right_wheel = ChSharedPtr<HMMWV_Wheel>(new HMMWV_WheelRight(wheelVis));
  m_front_left_wheel = ChSharedPtr<HMMWV_Wheel>(new HMMWV_WheelLeft(wheelVis));
  m_rear_right_wheel = ChSharedPtr<HMMWV_Wheel>(new HMMWV_WheelRight(wheelVis));
  m_rear_left_wheel = ChSharedPtr<HMMWV_Wheel>(new HMMWV_WheelLeft(wheelVis));

  // -------------------------------
  // Create the powertrain subsystem
  //--------------------------------

  ////m_powertrain = ChSharedPtr<HMMWV_SimplePowertrain>(new HMMWV_SimplePowertrain(this));
  m_powertrain = ChSharedPtr<HMMWV_Powertrain>(new HMMWV_Powertrain(this));
}


HMMWV_Vehicle::~HMMWV_Vehicle()
{
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_Vehicle::Initialize(const ChCoordsys<>& chassisPos)
{
  m_chassis->SetPos(chassisPos.pos);
  m_chassis->SetRot(chassisPos.rot);

  // Initialize the suspension subsystems
  m_front_right_susp->Initialize(m_chassis, in2m * ChVector<>(-85.39, 12.10, -18.914));
  m_front_left_susp->Initialize(m_chassis, in2m * ChVector<>(-85.39, -12.10, -18.914));
  m_rear_right_susp->Initialize(m_chassis, in2m * ChVector<>(47.60, 12.10, -18.914));
  m_rear_left_susp->Initialize(m_chassis, in2m * ChVector<>(47.60, -12.10, -18.914));

  // Initialize wheels
  m_front_right_wheel->Initialize(m_front_right_susp->GetSpindle());
  m_front_left_wheel->Initialize(m_front_left_susp->GetSpindle());
  m_rear_right_wheel->Initialize(m_rear_right_susp->GetSpindle());
  m_rear_left_wheel->Initialize(m_rear_left_susp->GetSpindle());

  // Initialize the powertrain subsystem
  m_powertrain->Initialize(m_chassis, m_rear_left_susp->GetAxle(), m_rear_right_susp->GetAxle());

}


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
ChSharedBodyPtr HMMWV_Vehicle::GetWheelBody(ChWheelId which) const
{
  switch (which) {
  case FRONT_LEFT:
    return m_front_left_susp->GetSpindle();
  case FRONT_RIGHT:
    return m_front_right_susp->GetSpindle();
  case REAR_LEFT:
    return m_rear_left_susp->GetSpindle();
  case REAR_RIGHT:
    return m_rear_right_susp->GetSpindle();
  default:
    return m_front_left_susp->GetSpindle();  // should not happen
  }
}

const ChVector<>& HMMWV_Vehicle::GetWheelPos(ChWheelId which) const
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
  default:
    return m_front_left_susp->GetSpindlePos();  // should not happen
  }
}

const ChQuaternion<>& HMMWV_Vehicle::GetWheelRot(ChWheelId which) const
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
  default:
    return m_front_left_susp->GetSpindleRot();  // should not happen
  }
}

const ChVector<>& HMMWV_Vehicle::GetWheelLinVel(ChWheelId which) const
{
  switch (which) {
  case FRONT_LEFT:
    return m_front_left_susp->GetSpindleLinVel();
  case FRONT_RIGHT:
    return m_front_right_susp->GetSpindleLinVel();
  case REAR_LEFT:
    return m_rear_left_susp->GetSpindleLinVel();
  case REAR_RIGHT:
    return m_rear_right_susp->GetSpindleLinVel();
  default:
    return m_front_left_susp->GetSpindleLinVel();  // should not happen
  }
}

ChVector<> HMMWV_Vehicle::GetWheelAngVel(ChWheelId which) const
{
  switch (which) {
  case FRONT_LEFT:
    return m_front_left_susp->GetSpindleAngVel();
  case FRONT_RIGHT:
    return m_front_right_susp->GetSpindleAngVel();
  case REAR_LEFT:
    return m_rear_left_susp->GetSpindleAngVel();
  case REAR_RIGHT:
    return m_rear_right_susp->GetSpindleAngVel();
  default:
    return m_front_left_susp->GetSpindleAngVel();  // should not happen
  }
}

double HMMWV_Vehicle::GetWheelOmega(ChWheelId which) const
{
  switch (which) {
  case FRONT_LEFT:
    return m_front_left_susp->GetAxleSpeed();
  case FRONT_RIGHT:
    return m_front_right_susp->GetAxleSpeed();
  case REAR_LEFT:
    return m_rear_left_susp->GetAxleSpeed();
  case REAR_RIGHT:
    return m_rear_right_susp->GetAxleSpeed();
  default:
    return -1;  // should not happen
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_Vehicle::Update(double              time,
                           double              throttle,
                           double              steering,
                           const ChTireForces& tire_forces)
{
  // Apply steering input.
  double displ = 0.08 * steering;

  m_front_left_susp->ApplySteering(displ);
  m_front_right_susp->ApplySteering(displ);

  // Let the powertrain subsystem process the throttle input.
  m_powertrain->Update(time, throttle);

  // Apply tire forces to spindle bodies.
  m_front_right_susp->ApplyTireForce(tire_forces[FRONT_RIGHT]);
  m_front_left_susp->ApplyTireForce(tire_forces[FRONT_LEFT]);
  m_rear_right_susp->ApplyTireForce(tire_forces[REAR_RIGHT]);
  m_rear_left_susp->ApplyTireForce(tire_forces[REAR_LEFT]);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_Vehicle::ExportMeshPovray(const std::string& out_dir)
{
  utils::WriteMeshPovray(m_chassisMeshFile, m_chassisMeshName, out_dir);
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_Vehicle::CheckShocks(const size_t step_num, const double simTime)
{
  // 1) report the spring error every n_th time step, force and length @ design
  size_t report_time_interval = 1000;  // 1 second between logging this

  if( !(step_num % report_time_interval) )
  {
    // spring forces, front left & right (lbs)
    double springF_FL = GetSpringForce(FRONT_LEFT)/4.45;       // design = 3491 lb.
    double springF_FR = GetSpringForce(FRONT_RIGHT)/4.45;      // design = 3491 lb.
    // spring forces, back left & right (lbs)
    double springF_RL = GetSpringForce(REAR_LEFT)/4.45;        // design = 6388 lb.
    double springF_RR = GetSpringForce(REAR_RIGHT)/4.45;       // design = 6388 lb.

    // spring lengths, front left & right (inches)
    double springLen_FL = GetSpringLength(FRONT_LEFT)*39.37;   // design = 9.7" + 4.65"
    double springLen_FR = GetSpringLength(FRONT_RIGHT)*39.37;  // design = 9.7" + 4.65
    // spring lengths, rear left & right (inches)
    double springLen_RL = GetSpringLength(REAR_LEFT)*39.37;    // design = 12.0" + 2.35"
    double springLen_RR = GetSpringLength(REAR_RIGHT)*39.37;   // design = 12.0" + 2.35"

    // springs are mounted at shock points, add the distance between top shock and spring hardpoints
    double dl_F = 9.7 + 4.65;
    double dl_R = 12.0 + 2.35;

    GetLog() << " \n ---- Spring, Shock info, time = " << simTime <<
      "    ---- \n Forces [lbs.]: \nFL= " <<
      springF_FL << "\nFR= " << springF_FR << "\nRL= " << springF_RL << "\nRR= " <<
      springF_RR <<  "\n\n Lengths [inches]: \nFL= " << springLen_FL << "\nFR= " <<
      springLen_FR << "\nRL= " << springLen_RL <<	"\nRR= " << springLen_RR << "\n\n";

    GetLog() << " ***** Spring Force, length error relative to design \n Force ERROR[lbs.]: \nFL= " <<
      springF_FL-3491.0 << "\nFR= " << springF_FR-3491.0 << "\nRL= " <<
      springF_RL-6388.0 << "\nRR= " << springF_RR-6388 <<
      "\n\n Length ERROR [inches]: \nFL= " << springLen_FL-dl_F <<	"\nFR= " <<
      springLen_FR-dl_F << "\nRL= " << springLen_RL-dl_R << "\nRR= " <<
      springLen_RR-dl_R << "\n\n";
  }

}

double HMMWV_Vehicle::GetSpringForce(chrono::ChWheelId which){
  switch (which) {
  case FRONT_LEFT:
    return m_front_left_susp->GetSpringForce();
  case FRONT_RIGHT:
    return m_front_right_susp->GetSpringForce();
  case REAR_LEFT:
    return m_rear_left_susp->GetSpringForce();
  case REAR_RIGHT:
    return m_rear_right_susp->GetSpringForce();
  default:
    return m_front_left_susp->GetSpringForce();  // should not happen
  }
}

double HMMWV_Vehicle::GetSpringLength(chrono::ChWheelId which){
  switch (which) {
  case FRONT_LEFT:
    return m_front_left_susp->GetSpringLen();
  case FRONT_RIGHT:
    return m_front_right_susp->GetSpringLen();
  case REAR_LEFT:
    return m_rear_left_susp->GetSpringLen();
  case REAR_RIGHT:
    return m_rear_right_susp->GetSpringLen();
  default:
    return m_front_left_susp->GetSpringLen();  // should not happen
  }
}


} // end namespace hmmwv
