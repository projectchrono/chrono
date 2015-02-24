// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
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
// Base class for a Track vehicle model.
//
// =============================================================================

#include <algorithm>

#include "ChTrackVehicle.h"

#include "assets/ChAssetLevel.h"
#include "assets/ChBoxShape.h"
#include "assets/ChColorAsset.h"
#include "assets/ChCylinderShape.h"
#include "assets/ChSphereShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "assets/ChTexture.h"

#include "utils/ChUtilsInputOutput.h"
#include "utils/ChUtilsData.h"

// collision mesh
#include "geometry/ChCTriangleMeshSoup.h"

namespace chrono {


ChTrackVehicle::ChTrackVehicle(const std::string& name,
                               VisualizationType vis,
                               CollisionType collide,
                               double mass,
                               const ChVector<>& Ixx,
                               size_t num_engines,
                               double step_size)
: m_ownsSystem(true),
  m_vis(vis),
  m_collide(collide),
  // m_mass(mass),
  // m_inertia(Ixx),
  m_num_engines(num_engines),
  m_stepsize(step_size),
  m_save_log_to_file(false), // save the DebugLog() info to file? default false
  m_log_what_to_file(0),     // set this in Setup_log_to_file(), if writing to file
  m_log_file_exists(false), // written the headers for log file yet?
  m_log_what_to_console(0)  // pre-set what to write to console when calling 
{
  // create a new system, set gravity, default solver settings
  m_system = new ChSystem;
  m_system->Set_G_acc(ChVector<>(0, -9.81, 0));
  m_system->SetStep(m_stepsize);
  m_system->SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);
  m_system->SetIterLCPmaxItersSpeed(150);
  m_system->SetIterLCPmaxItersStab(150);
  m_system->SetTol(0);
  m_system->SetMaxPenetrationRecoverySpeed(1.5);
  m_system->SetMinBounceSpeed(1);
  // m_system->SetIterLCPomega(0.8);
  // m_system->SetIterLCPsharpnessLambda(0.9);

  // create the chassis to attach mass, inertia to.
  m_chassis = ChSharedPtr<ChBodyAuxRef>(new ChBodyAuxRef);
  m_chassis->SetMass(mass);
  m_chassis->SetInertiaXX(Ixx);
  m_chassis->SetNameString(name);
  // add the chassis body to the system
  m_system->Add(m_chassis);

  // init. any other variables with a default value
  m_meshName = "meshName";
  m_meshFile = utils::GetModelDataFile("M113/Chassis_XforwardYup.obj");
  m_chassisBoxSize = ChVector<>(4.0, 1.2, 1.5); // full length, height, width of chassis box

  // set any vector known sizes here
  m_ptrains.resize(num_engines);

}

// system already exists, create vehicle with specified input
ChTrackVehicle::ChTrackVehicle(ChSystem* system,
                               const std::string& name,
                               VisualizationType vis,
                               CollisionType collide,
                               double mass,
                               const ChVector<>& Ixx,
                               size_t num_engines
): m_ownsSystem(false),
  m_system(system),
  m_vis(vis),
  m_collide(collide),
  // m_mass(mass),
  // m_inertia(Ixx),
  m_num_engines(num_engines),
  m_stepsize(system->GetStep()),
  m_save_log_to_file(false), // save the DebugLog() info to file? default false
  m_log_what_to_file(0),     // set this in Setup_log_to_file(), if writing to file
  m_log_file_exists(false), // written the headers for log file yet?
  m_log_what_to_console(0)  // pre-set what to write to console when calling 
{
  // don't worry about solver settings, other system will already handle that.

  // create the chassis to attach mass, inertia to.
  m_chassis = ChSharedPtr<ChBodyAuxRef>(new ChBodyAuxRef);
  m_chassis->SetMass(mass);
  m_chassis->SetInertiaXX(Ixx);
  m_chassis->SetNameString(name);
  // add the chassis body to the system
  m_system->Add(m_chassis);

  // init. any other variables with a default value
  m_meshName = "meshName";
  m_meshFile = utils::GetModelDataFile("M113/Chassis_XforwardYup.obj");
  m_chassisBoxSize = ChVector<>(4.0, 1.2, 1.5); // full length, height, width of chassis box
}


ChTrackVehicle::~ChTrackVehicle()
{
  if (m_ownsSystem)
    delete m_system;
}


// -----------------------------------------------------------------------------
// Advance the state of the system, taking as many steps as needed to exactly
// reach the specified value 'step'.
void ChTrackVehicle::Advance(double step)
{
  double t = 0;
  while (t < step) {
    double h = std::min<>(m_stepsize, step - t);
    m_system->DoStepDynamics(h);
    t += h;
  }
}

// -----------------------------------------------------------------------------
// Return the global driver position
// -----------------------------------------------------------------------------
const ChVector<>& ChTrackVehicle::GetDriverPos() const
{
  return m_chassis->GetCoord().TransformPointLocalToParent(GetLocalDriverCoordsys().pos);
}



void ChTrackVehicle::AddVisualization()
{
  // add visual geometry asset to the chassis, if enabled
  switch (m_vis) {
  case VisualizationType::NONE:
  {
    // put a sphere at the chassis COM and at the REF point
    ChSharedPtr<ChSphereShape> COMsphere(new ChSphereShape);
    COMsphere->GetSphereGeometry().rad = 0.1;
    COMsphere->Pos = m_chassis->GetPos();
    m_chassis->AddAsset(COMsphere);
    // make the COM sphere blue
    ChSharedPtr<ChColorAsset> blue(new ChColorAsset(0.1f, 0.2f, 0.8f));
    m_chassis->AddAsset(blue);
   
    // to give the REF sphere a different color, add it to the level first.
    ChSharedPtr<ChAssetLevel> ref_level(new ChAssetLevel);
    ChSharedPtr<ChSphereShape> REFsphere(new ChSphereShape);
    REFsphere->GetSphereGeometry().rad = 0.1;
    REFsphere->Pos = ChVector<>(0,0,0); // REF should be at the body c-sys origin
    ref_level->AddAsset(REFsphere);
    // make the REF sphere red
    ChSharedPtr<ChColorAsset> red(new ChColorAsset(0.8f, 0.2f, 0.1f) );
    ref_level->AddAsset(red);
    // add the level to the body
    m_chassis->AddAsset(ref_level);

    break;
  }
  case VisualizationType::PRIMITIVES:
  {
    ChSharedPtr<ChBoxShape> box(new ChBoxShape);
    // uses full lengths as input
    box->GetBoxGeometry().SetLengths(m_chassisBoxSize );
    m_chassis->AddAsset(box);
    break;
  }
  case VisualizationType::MESH:
  {
    geometry::ChTriangleMeshConnected trimesh;
    trimesh.LoadWavefrontMesh(m_meshFile, true, true);

    ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName("chassis triMesh");
    m_chassis->AddAsset(trimesh_shape);

    break;
  }
  } // end switch


}

void ChTrackVehicle::AddCollisionGeometry(double mu,
                                          double mu_sliding,
                                          double mu_roll,
                                          double mu_spin)
{
  // add collision geometrey to the chassis, if enabled
  if( m_collide == CollisionType::NONE)
  {
    m_chassis->SetCollide(false);
    return;
  }
  m_chassis->SetCollide(true);
  m_chassis->GetCollisionModel()->ClearModel();

  m_chassis->GetCollisionModel()->SetSafeMargin(0.001);	// inward safe margin
	m_chassis->GetCollisionModel()->SetEnvelope(0.002);		// distance of the outward "collision envelope"

  // set the collision material
  m_chassis->GetMaterialSurface()->SetSfriction(mu);
  m_chassis->GetMaterialSurface()->SetKfriction(mu_sliding);
  m_chassis->GetMaterialSurface()->SetRollingFriction(mu_roll);
  m_chassis->GetMaterialSurface()->SetSpinningFriction(mu_spin);

  switch (m_collide) {
  case CollisionType::PRIMITIVES:
  {
    // use a simple box, half dimensions as input
    m_chassis->GetCollisionModel()->AddBox(0.5*m_chassisBoxSize.x, 
      0.5*m_chassisBoxSize.y,
      0.5*m_chassisBoxSize.z);

    break;
  }
  case CollisionType::MESH:
  {
    // use a triangle mesh
   
		geometry::ChTriangleMeshSoup temp_trianglemesh; 
		
    // TODO: fill the triangleMesh here with some track shoe geometry

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
  default:
    // no collision geometry
    GetLog() << "not recognized CollisionType: " << (int)m_collide <<" for chassis \n";
    m_chassis->SetCollide(false);
    return;
  } // end switch

  // set the collision family
  m_chassis->GetCollisionModel()->SetFamily( (int)CollisionFam::HULL );
  // don't collide with rolling elements or tracks
  m_chassis->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily( (int)(CollisionFam::WHEELS) );
  m_chassis->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily( (int)(CollisionFam::SHOES) );
  m_chassis->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::GEAR);

  m_chassis->GetCollisionModel()->BuildModel();
}



}  // end namespace chrono
