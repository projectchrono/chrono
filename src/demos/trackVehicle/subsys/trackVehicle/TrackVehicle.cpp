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
// Authors: Justin Madsen
// =============================================================================
//
// Tracked vehicle model built from subsystems.
//  Location of subsystems hard-coded for M113 vehicle
//  TODO: specify this w/ JSON input data file
//
// =============================================================================

#include <cstdio>

#include "assets/ChCylinderShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"
#include "physics/ChGlobal.h"

#include "TrackVehicle.h"

#include "subsys/trackSystem/TrackSystem.h"
#include "subsys/driveline/TrackDriveline.h"

#include "utils/ChUtilsInputOutput.h"
#include "utils/ChUtilsData.h"

// collision mesh
#include "geometry/ChCTriangleMeshSoup.h"

namespace chrono {

// -----------------------------------------------------------------------------
// Static variables
const size_t TrackVehicle::m_num_tracks = 2;    // number of trackSystems to create
const size_t TrackVehicle::m_num_engines = 1;   // number of powertrains (and drivelines) to create

const double     TrackVehicle::m_mass = 5489.2;   // chassis sprung mass
const ChVector<> TrackVehicle::m_COM = ChVector<>(0.1, 0.0, 0.3);  // COM location, relative to body Csys REF frame
const ChVector<> TrackVehicle::m_inertia(1786.9, 10449.7, 10721.2);  // chassis inertia (roll,yaw,pitch)

const std::string TrackVehicle::m_meshName("M113_mesh");
const std::string TrackVehicle::m_meshFile = utils::GetModelDataFile("data/M113_hull.obj");
const ChCoordsys<> TrackVehicle::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));
const ChVector<> TrackVehicle::m_chassisBoxSize(4.0, 2.0, 3.0);

/// constructor sets the basic integrator settings for this ChSystem, as well as the usual stuff
TrackVehicle::TrackVehicle(bool fixed, VisualizationType chassisVis, CollisionType chassisCollide)
  : m_vis(chassisVis), m_collide(chassisCollide)
{
  // Integration settings
  // Integration and Solver settings
  SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
  SetIterLCPmaxItersSpeed(150);
  SetIterLCPmaxItersStab(150);
  SetMaxPenetrationRecoverySpeed(4.0);

  // create the chassis body    
  m_chassis = ChSharedPtr<ChBodyAuxRef>(new ChBodyAuxRef);
  m_chassis->SetIdentifier(0);
  m_chassis->SetName("chassis");
  m_chassis->SetFrame_COG_to_REF(ChFrame<>(m_COM, ChQuaternion<>(1, 0, 0, 0)));
  // basic body info
  m_chassis->SetMass(m_mass);
  m_chassis->SetInertiaXX(m_inertia);
  m_chassis->SetBodyFixed(fixed);
  // add visualization assets to the chassis
  AddVisualization();
  // add the chassis body to the system
  Add(m_chassis);

  // resize all vectors for the number of track systems
  m_TrackSystems.resize(m_num_tracks);
  m_TrackSystem_locs.resize(m_num_tracks);

  // Each trackSystem has its own driveline and powertrain, so the left and right
  // sides can have power applied independently
  m_drivelines.resize(m_num_tracks);
  m_ptrains.resize(m_num_tracks);

  // create track systems
  for (int i = 0; i < m_num_tracks; i++) {
    m_TrackSystems[i] = ChSharedPtr<TrackSystem>(new TrackSystem("track chain "+std::to_string(i), i));
  }
  
  // create the powertrain and drivelines
  for (int j = 0; j < m_num_engines; j++)
  {
    m_drivelines[j] = ChSharedPtr<TrackDriveline>(new TrackDriveline("driveline "+std::to_string(j))  );
    m_ptrains[j] = ChSharedPtr<TrackPowertrain>(new TrackPowertrain("powertrain "+std::to_string(j)) );
  
  }

  // TODO: add brakes. Perhaps they are a part of the suspension subsystem?

}


// Set any collision geometry on the hull, then Initialize() all subsystems
void TrackVehicle::Initialize(const ChCoordsys<>& chassis_Csys)
{
  // move the chassis REF frame to the specified initial position/orientation
  m_chassis->SetFrame_REF_to_abs(ChFrame<>(chassis_Csys));

  // add collision geometry to the chassis
  AddCollisionGeometry();

  // initialize the subsystems with the initial c-sys and specified offsets
  for (int i = 0; i < m_num_tracks; i++)
  {
    m_TrackSystems[i]->Initialize(m_chassis, m_TrackSystem_locs[i]);
  }

  // initialize the powertrain, drivelines
  for (int j = 0; j < m_num_engines; j++)
  {
    size_t driveGear_L_idx = 2*j;
    size_t driveGear_R_idx = 2*j + 1;
    m_drivelines[j]->Initialize(m_chassis,
      m_TrackSystems[driveGear_L_idx]->GetDriveGear(),
      m_TrackSystems[driveGear_R_idx]->GetDriveGear());
    m_ptrains[j]->Initialize(m_chassis, m_drivelines[j]->GetDriveshaft());
  }

}


void TrackVehicle::Update(double	time,
                          const std::vector<double>&  throttle,
                          const std::vector<double>&  braking)
{
 
  // update left and right powertrains, with the new left and right throttle/shaftspeed
  m_ptrains[0]->Update(time, throttle[0], m_drivelines[0]->GetDriveshaftSpeed() );
  m_ptrains[1]->Update(time, throttle[1], m_drivelines[1]->GetDriveshaftSpeed() );
}

void TrackVehicle::AddVisualization()
{
  // add visual geometry asset to the chassis, if enabled
  switch (m_vis) {
  case VisualizationType::PRIMITIVES:
  {
    // put a sphere at the chassis COM and at the REF point
    ChSharedPtr<ChSphereShape> COMsphere(new ChSphereShape);
    COMsphere->GetSphereGeometry().rad = 0.1;
    COMsphere->Pos = m_COM;
    // make the COM sphere blue
    COMsphere->SetColor(ChColor(0.1f, 0.2f, 0.8f));
    m_chassis->AddAsset(COMsphere);
    
   
    ChSharedPtr<ChSphereShape> REFsphere(new ChSphereShape);
    REFsphere->GetSphereGeometry().rad = 0.1;
    REFsphere->Pos = ChVector<>(0,0,0); // REF should be at the body c-sys origin
    // make the REF sphere red
    REFsphere->SetColor(ChColor(0.8f, 0.2f, 0.1f));
    m_chassis->AddAsset(REFsphere);

    break;
  }
  case VisualizationType::MESH:
  {
    geometry::ChTriangleMeshConnected trimesh;
    trimesh.LoadWavefrontMesh(utils::GetModelDataFile(m_meshFile), false, false);

    ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName("chassis triMesh");
    m_chassis->AddAsset(trimesh_shape);

    break;
  }
  } // end switch


}

void TrackVehicle::AddCollisionGeometry()
{
  // add collision geometrey to the chassis, if enabled
  m_chassis->SetCollide(true);
  m_chassis->GetCollisionModel()->ClearModel();

  switch (m_collide) {
  case CollisionType::PRIMITIVES:
  {
    // use a simple box
    m_chassis->GetCollisionModel()->AddBox(m_chassisBoxSize.x, m_chassisBoxSize.y, m_chassisBoxSize.z);

    break;
  }
  case CollisionType::MESH:
  {
    // use a triangle mesh
   
		geometry::ChTriangleMeshSoup temp_trianglemesh; 
		
    // TODO: fill the triangleMesh here with some track shoe geometry

		m_chassis->GetCollisionModel()->SetSafeMargin(0.004);	// inward safe margin
		m_chassis->GetCollisionModel()->SetEnvelope(0.010);		// distance of the outward "collision envelope"
		m_chassis->GetCollisionModel()->ClearModel();

    // is there an offset??
    double shoelength = 0.2;
    ChVector<> mesh_displacement(shoelength*0.5,0,0);  // since mesh origin is not in body center of mass
    m_chassis->GetCollisionModel()->AddTriangleMesh(temp_trianglemesh, false, false, mesh_displacement);

    break;
  }
  case CollisionType::CONVEXHULL:
  {
    // use convex hulls, loaded from file
    ChStreamInAsciiFile chull_file(GetChronoDataFile("track_shoe.chulls").c_str());
    // transform the collision geometry as needed
    double mangle = 45.0; // guess
    ChQuaternion<>rot;
    rot.Q_from_AngAxis(mangle*(CH_C_PI/180.),VECT_X);
    ChMatrix33<> rot_offset(rot);
    ChVector<> disp_offset(0,0,0);  // no displacement offset
    m_chassis->GetCollisionModel()->AddConvexHullsFromFile(chull_file, disp_offset, rot_offset);
    break;
  }
  } // end switch

  // set the collision family
  m_chassis->GetCollisionModel()->SetFamily( (int)CollisionFam::HULL );
  // don't collide with rolling elements or tracks
  m_chassis->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily( (int)(CollisionFam::GEARS) );
  m_chassis->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily( (int)(CollisionFam::WHEELS) );
  m_chassis->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily( (int)(CollisionFam::SHOES) );

  m_chassis->GetCollisionModel()->BuildModel();
}

void TrackVehicle::Advance(double step)
{
  double t = 0;
  while (t < step) {
    double h = std::min<>(m_stepsize, step - t);
    DoStepDynamics(h);
    t += h;
  }
}



} // end namespace chrono
