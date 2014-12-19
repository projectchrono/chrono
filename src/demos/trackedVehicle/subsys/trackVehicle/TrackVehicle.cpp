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
// Tracked vehicle model built from subsystems specified w/ JSON input data file
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


namespace chrono {

// -----------------------------------------------------------------------------
// Static variables
const double     TrackVehicle::m_Mass = 10000;   // chassis sprung mass
const ChVector<> TrackVehicle::m_COM = ChVector<>(0.3, 0.0, 0.5);  // COM location
const ChVector<> TrackVehicle::m_Inertia(425.8, 697.4, 731.4);  // chassis inertia (roll,pitch,yaw)

const std::string TrackVehicle::m_MeshFile = utils::GetModelDataFile("hmmwv/hmmwv_chassis.obj");

const ChCoordsys<> TrackVehicle::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));



TrackVehicle::TrackVehicle(bool fixed, bool chassisVis)
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
  m_chassis->SetInertiaXX(m_Inertia);
  m_chassis->SetBodyFixed(fixed);

  if (chassisVis)
  {

    geometry::ChTriangleMeshConnected trimesh;
    trimesh.LoadWavefrontMesh(utils::GetModelDataFile(m_MeshFile), false, false);

    ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName("chassis triMesh");
    m_chassis->AddAsset(trimesh_shape);
  }
  else
  {
    ChSharedPtr<ChSphereShape> sphere(new ChSphereShape);
    sphere->GetSphereGeometry().rad = 0.1;
    sphere->Pos = m_COM;
    m_chassis->AddAsset(sphere);
  }
  
  Add(m_chassis);


  m_num_tracks = 2;
  // resize the vectors for the number of track systems to use
  m_TrackSystems.resize(m_num_tracks);
  m_TrackSystem_locs.resize(m_num_tracks);
  m_drivelines.resize(m_num_tracks);
  m_ptrains.resize(m_num_tracks);

  // create track systems
  for (int i = 0; i < m_num_tracks; i++) {
    m_TrackSystems[i] = ChSharedPtr<TrackSystem>(new TrackSystem("track chain"+std::to_string(i), i));
    // create the powertrain and drivelines
    m_drivelines[i] = ChSharedPtr<TrackDriveline>(new TrackDriveline);
    m_ptrains[i] = ChSharedPtr<TrackPowertrain>(new TrackPowertrain);
  
  }


}



void TrackVehicle::Initialize(const ChCoordsys<>& chassisPos)
{
  m_chassis->SetFrame_REF_to_abs(ChFrame<>(chassisPos));

  // Initialize the track systems

  // Initialize the suspension, wheel, and brake subsystems.
  for (int i = 0; i < m_num_tracks; i++)
  {
    m_TrackSystems[i]->Initialize(m_chassis, m_TrackSystem_locs[i], QUNIT);
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
