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

#include "physics/ChGlobal.h"

#include "TrackVehicle.h"

#include "subsys/trackSystem/TrackSystem.h"
#include "subsys/driveline/TrackDriveline.h"

#include "utils/ChUtilsInputOutput.h"
#include "utils/ChUtilsData.h"


namespace chrono {

// -----------------------------------------------------------------------------
// Static variables
const ChVector<> TrackVehicle::m_trackPos_Right(0.23644, -0.4780, 0.83475); // relative to chassis COG
const ChVector<> TrackVehicle::m_trackPos_Left(0.23644, -0.4780, -0.83475); // relative to chassis COG

const double     TrackVehicle::m_mass = 5489.2;   // chassis sprung mass
const ChVector<> TrackVehicle::m_COM = ChVector<>(0., 0.0, 0.);  // COM location, relative to body Csys REF frame
const ChVector<> TrackVehicle::m_inertia(1786.9, 10449.7, 10721.2);  // chassis inertia (roll,yaw,pitch)

const ChCoordsys<> TrackVehicle::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));

/// constructor sets the basic integrator settings for this ChSystem, as well as the usual stuff
TrackVehicle::TrackVehicle(const std::string& name,
                           VisualizationType chassisVis,
                           CollisionType chassisCollide)
  :ChTrackVehicle(),
  m_num_tracks(2)
{
  // ---------------------------------------------------------------------------
  // Set the base class variables
  m_vis = chassisVis;
  m_collide = chassisCollide;
  m_meshName = "M113_chassis";
  m_meshFile = utils::GetModelDataFile("M113/Chassis_XforwardYup.obj");
  m_chassisBoxSize = ChVector<>(2.0, 0.6, 0.75);

  // create the chassis body    
  m_chassis = ChSharedPtr<ChBodyAuxRef>(new ChBodyAuxRef);
  m_chassis->SetIdentifier(0);
  m_chassis->SetNameString(name);
  m_chassis->SetFrame_COG_to_REF(ChFrame<>(m_COM, ChQuaternion<>(1, 0, 0, 0)));
  // basic body info
  m_chassis->SetMass(m_mass);
  m_chassis->SetInertiaXX(m_inertia);

  // add visualization assets to the chassis
  AddVisualization();

  // m_chassis->SetBodyFixed(true);
  // add the chassis body to the system
  m_system->Add(m_chassis);

  // resize all vectors for the number of track systems
  m_TrackSystems.resize(m_num_tracks);
  m_TrackSystem_locs.resize(m_num_tracks);
  // Right and Left track System relative locations, respectively
  m_TrackSystem_locs[0] = m_trackPos_Right;
  m_TrackSystem_locs[1] = m_trackPos_Left;

  // two drive Gears, like a 2WD driven vehicle.
  m_num_engines = 1;
  m_drivelines.resize(m_num_engines);
  m_ptrains.resize(m_num_engines);

  // create track systems
  for (int i = 0; i < m_num_tracks; i++) {
    m_TrackSystems[i] = ChSharedPtr<TrackSystem>(new TrackSystem("track chain "+std::to_string(i), i) );
  }
  
  // create the powertrain and drivelines
  for (int j = 0; j < m_num_engines; j++)
  {
    m_drivelines[j] = ChSharedPtr<TrackDriveline>(new TrackDriveline("driveline "+std::to_string(j)) );
    m_ptrains[j] = ChSharedPtr<TrackPowertrain>(new TrackPowertrain("powertrain "+std::to_string(j)) );
  }

  // TODO: add brakes. Perhaps they are a part of the suspension subsystem?

}


TrackVehicle::~TrackVehicle()
{
  if(m_ownsSystem)
    delete m_system;
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
    size_t driveGear_R_idx = 2*j;
    size_t driveGear_L_idx = 2*j + 1;
    m_drivelines[j]->Initialize(m_chassis,
      m_TrackSystems[driveGear_R_idx]->GetDriveGear(),
      m_TrackSystems[driveGear_L_idx]->GetDriveGear());
    m_ptrains[j]->Initialize(m_chassis, m_drivelines[j]->GetDriveshaft());
  }

}


void TrackVehicle::Update(double	time,
                          const std::vector<double>&  throttle,
                          const std::vector<double>&  braking)
{
  assert( throttle.size() >= m_num_tracks);
  assert( braking.size() >= m_num_tracks );
  // update left and right powertrains, with the new left and right throttle/shaftspeed
  for(int i = 0; i < m_num_engines; i++)
  {
    m_ptrains[i]->Update(time, throttle[i], m_drivelines[0]->GetDriveshaftSpeed() );
  }

}

void TrackVehicle::Advance(double step)
{
  double t = 0;
  double settlePhaseA = 0.001;
  double settlePhaseB = 0.01;
  while (t < step) {
    double h = std::min<>(m_stepsize, step - t);
    if( m_system->GetChTime() < settlePhaseA )
    {
      h = 1e-5;
    } else if ( m_system->GetChTime() < settlePhaseB )
    {
      h = 1e-4;
    }
    m_system->DoStepDynamics(h);
    t += h;
  }
}


double TrackVehicle::GetIdlerForce(size_t side)
{
  assert(side < m_num_tracks);
  ChVector<> out_force = m_TrackSystems[side]->Get_idler_spring_react();

  return out_force.Length();
}


double TrackVehicle::GetDriveshaftSpeed(size_t idx) const
{
  assert(idx < m_drivelines.size() );
  return m_drivelines[idx]->GetDriveshaftSpeed();
}


TrackPowertrain* TrackVehicle::GetPowertrain(size_t idx)
{ 
  assert( idx < m_num_engines );
  TrackPowertrain* out_ptr = m_ptrains[idx].get_ptr();
  return out_ptr;
}


} // end namespace chrono
