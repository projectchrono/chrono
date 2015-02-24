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
#include <algorithm>

#include "physics/ChGlobal.h"

#include "TrackVehicle.h"

#include "subsys/trackSystem/TrackSystem.h"
#include "subsys/driveline/TrackDriveline.h"

#include "utils/ChUtilsInputOutput.h"
#include "utils/ChUtilsData.h"


namespace chrono {

// -----------------------------------------------------------------------------
// Static variables
const double     TrackVehicle::mass_override = 5489.2 / 5.0; // chassis sprung mass override
const ChVector<> TrackVehicle::COM_override = ChVector<>(0., 0.0, 0.);  // COM location, relative to body Csys REF frame
const ChVector<> TrackVehicle::inertia_override(1786.9/5.0, 10449.7/5.0, 10721.2/5.0);  // chassis inertia (roll,yaw,pitch)

const ChCoordsys<> TrackVehicle::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));

/// constructor sets the basic integrator settings for this ChSystem, as well as the usual stuff
TrackVehicle::TrackVehicle(const std::string& name,
                           VisualizationType chassisVis,
                           CollisionType chassisCollide,
                           double mass,
                           const ChVector<>& Ixx,
                           const ChVector<>& left_pos_rel,
                           const ChVector<>& right_pos_rel
):ChTrackVehicle(name, chassisVis, chassisCollide, mass, Ixx, 1),
  m_num_tracks(2),
  m_trackSys_L(left_pos_rel),
  m_trackSys_R(right_pos_rel)
{
  // ---------------------------------------------------------------------------
  // Set the base class variables not created by constructor, if we plan to use them.
  m_meshName = "M113_chassis";
  m_meshFile = utils::GetModelDataFile("M113/Chassis_XforwardYup.obj");
  m_chassisBoxSize = ChVector<>(4.0, 1.2, 1.5); // full length, height, width of chassis box

  // setup the chassis body
  m_chassis->SetIdentifier(0);
  m_chassis->SetFrame_COG_to_REF(ChFrame<>(COM_override, ChQuaternion<>(1, 0, 0, 0)));
  // add visualization assets to the chassis
  AddVisualization();

  // resize all vectors for the number of track systems
  m_TrackSystems.resize(m_num_tracks);
  m_TrackSystem_locs.resize(m_num_tracks);
  // Right and Left track System relative locations, respectively
  m_TrackSystem_locs[0] = m_trackSys_L;
  m_TrackSystem_locs[1] = m_trackSys_R;

  // two drive Gears, like a 2WD driven vehicle.
  m_drivelines.resize(m_num_engines);
  // m_ptrains.resize(m_num_engines); // done by base vehicle class

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
  double settlePhaseA = 0.3;
  double settlePhaseB = 0.8;
  m_system->SetIterLCPmaxItersStab(80);
  m_system->SetIterLCPmaxItersSpeed(100);
  while (t < step) {
    double h = std::min<>(m_stepsize, step - t);
    if( m_system->GetChTime() < settlePhaseA )
    {
      h = 4e-4;
      m_system->SetIterLCPmaxItersStab(120);
      m_system->SetIterLCPmaxItersSpeed(150);
    } else if ( m_system->GetChTime() < settlePhaseB )
    {
      m_system->SetIterLCPmaxItersStab(100);
      m_system->SetIterLCPmaxItersSpeed(150);
      h = 6e-4;
    }
    m_system->DoStepDynamics(h);
    t += h;
  }
}


double TrackVehicle::GetDriveshaftSpeed(size_t idx) const
{
  assert(idx < m_drivelines.size() );
  return m_drivelines[idx]->GetDriveshaftSpeed();
}


const ChSharedPtr<TrackPowertrain> TrackVehicle::GetPowertrain(size_t idx) const
{ 
  assert( idx < m_num_engines );
  return  m_ptrains[idx];
}

} // end namespace chrono
