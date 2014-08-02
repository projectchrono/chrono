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

#include "assets/ChObjShapeFile.h"
#include "assets/ChAssetLevel.h"

#include "HMMWV9_Vehicle.h"


// DEBUG HACK
// 0:  no chassis visualization
// 1:  two boxes
// 2:  mesh
#define CHASSIS_VISUALIZATION 0

using namespace chrono;

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double HMMWV9_Vehicle::chassisMass = 7500.0 / 2.2;
const double HMMWV9_Vehicle::spindleMass = 100.0 / 2.2;

// Inertias, from my HMMWV model
// (roll, pitch, yaw) = (13320, 52680, 56280) lb-in-sec^2
const ChVector<> HMMWV9_Vehicle::chassisInertia(125.8, 497.4, 531.4);	// kg-m2;
const ChVector<> HMMWV9_Vehicle::spindleInertia = chassisInertia / 60;

const std::string HMMWV9_Vehicle::chassisMeshName = "hmmwv_chassis";
const std::string HMMWV9_Vehicle::chassisMeshFile = "../data/humvee4_scaled_rotated_decimated_centered.obj";


// helpful for unit conversions
static double in_to_m = 1.0/39.3701;	// inches to meters
static double inlb_to_Nm = 1.0/8.851;	// in-lb to N-m


// -----------------------------------------------------------------------------
HMMWV9_Vehicle::HMMWV9_Vehicle(ChSystem&            my_system,
                               const ChCoordsys<>&  chassisPos,
                               const bool           fixed)
{
  // -------------------------------------------
  // Create the chassis body
  // -------------------------------------------

  chassis = ChSharedBodyPtr(new ChBody);

  chassis->SetIdentifier(0);
  chassis->SetName("chassis");
  chassis->SetMass(chassisMass);
  chassis->SetInertiaXX(chassisInertia);
  chassis->SetPos(chassisPos.pos);
  chassis->SetRot(chassisPos.rot);
  chassis->SetBodyFixed(fixed);

#if CHASSIS_VISUALIZATION == 1
  ChSharedPtr<ChBoxShape> box1(new ChBoxShape);
  box1->GetBoxGeometry().SetLenghts(ChVector<>(5, 1.7, 0.4));
  box1->Pos = ChVector<>(0, 0, -0.4);
  chassis->AddAsset(box1);
  ChSharedPtr<ChBoxShape> box2(new ChBoxShape);
  box2->GetBoxGeometry().SetLenghts(ChVector<>(4, 1.7, 0.4));
  box2->Pos = ChVector<>(0.5, 0, 0);
  chassis->AddAsset(box2);
#elif CHASSIS_VISUALIZATION == 2
  geometry::ChTriangleMeshConnected trimesh;
  trimesh.LoadWavefrontMesh(chassisMeshFile, false, false);
  ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
  trimesh_shape->SetMesh(trimesh);
  trimesh_shape->SetName(chassisMeshName);
  chassis->AddAsset(trimesh_shape);
#endif

  my_system.Add(chassis);


  // -------------------------------------------
  // Create the suspension subsystems
  // -------------------------------------------
  
  m_front_right_susp = ChSharedPtr<HMMWV9_DoubleWishboneFront>(new HMMWV9_DoubleWishboneFront("RFsusp"));
  m_front_right_susp->Initialize(chassis, ChVector<>(-83.8, 35.82, -33.325)*in_to_m, false);

  m_front_left_susp = ChSharedPtr<HMMWV9_DoubleWishboneFront>(new HMMWV9_DoubleWishboneFront("LFsusp"));
  m_front_left_susp->Initialize(chassis, ChVector<>(-83.8, -35.82, -33.325)*in_to_m, true);

  m_rear_right_susp = ChSharedPtr<HMMWV9_DoubleWishboneRear>(new HMMWV9_DoubleWishboneRear("RBsusp"));
  m_rear_right_susp->Initialize(chassis, ChVector<>(46.2, 35.82, -33.325)*in_to_m, false);

  m_rear_left_susp = ChSharedPtr<HMMWV9_DoubleWishboneRear>(new HMMWV9_DoubleWishboneRear("RBsusp"));
  m_rear_left_susp->Initialize(chassis, ChVector<>(46.2, -35.82, -33.325)*in_to_m, true);


}


HMMWV9_Vehicle::~HMMWV9_Vehicle()
{
}
