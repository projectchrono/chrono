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
// Tracked vehicle model built from subsystems.
//  Location of subsystems hard-coded for M113 vehicle
//  Sprocket body driven with a specified motion.
//
// =============================================================================

#include <cstdio>
#include <algorithm>

#include "physics/ChGlobal.h"

#include "TrackVehicleM113.h"

#include "subsys/trackSystem/TrackSystemM113.h"

#include "utils/ChUtilsInputOutput.h"
#include "utils/ChUtilsData.h"


namespace chrono {

// -----------------------------------------------------------------------------
// Static variables
const double     TrackVehicleM113::mass_override = 5489.2 / 5.0; // chassis sprung mass override
const ChVector<> TrackVehicleM113::COM_override = ChVector<>(0., 0.0, 0.);  // COM location, relative to body Csys REF frame
const ChVector<> TrackVehicleM113::inertia_override(1786.9/5.0, 10449.7/5.0, 10721.2/5.0);  // chassis inertia (roll,yaw,pitch)

const ChCoordsys<> TrackVehicleM113::m_driverCsys(ChVector<>(0.0, 0.5, 1.2), ChQuaternion<>(1, 0, 0, 0));

/// constructor sets the basic integrator settings for this ChSystem, as well as the usual stuff
TrackVehicleM113::TrackVehicleM113(const std::string& name,
                           VisualizationType::Enum chassisVis,
                           CollisionType::Enum chassisCollide,
                           double mass,
                           const ChVector<>& Ixx,
                           double pin_damping_coef,
                           double tensioner_preload,
                           const ChVector<>& left_pos_rel,
                           const ChVector<>& right_pos_rel
): ChTrackVehicle(name, chassisVis, chassisCollide, mass, Ixx, 1),
  m_num_tracks(2),
  m_trackSys_L(left_pos_rel),
  m_trackSys_R(right_pos_rel),
  m_damping(pin_damping_coef),
  m_tensioner_preload(tensioner_preload)
{
  // Solver variables
  m_system->SetIterLCPomega(0.9);
  m_system->SetIterLCPsharpnessLambda(0.9);
  m_system->SetMaxPenetrationRecoverySpeed(1.5);

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
  // m_drivelines.resize(m_num_engines);
  // m_ptrains.resize(m_num_engines); // done by base vehicle class

  // create track systems
  for (int i = 0; i < m_num_tracks; i++) {
    std::stringstream t_ss;
    t_ss << "track chain " << i;
    m_TrackSystems[i] = ChSharedPtr<TrackSystemM113>(new TrackSystemM113(t_ss.str(), i, m_tensioner_preload) );
  }
  
  // create the powertrain and drivelines
  for (int j = 0; j < m_num_engines; j++)
  {
    std::stringstream dl_ss;
    dl_ss << "driveline " << j;
    std::stringstream pt_ss;
    pt_ss << "powertrain " << j;
    // m_drivelines[j] = ChSharedPtr<TrackDriveline>(new TrackDriveline(dl_ss.str() ) );
    m_ptrains[j] = ChSharedPtr<TrackPowertrain>(new TrackPowertrain(pt_ss.str() ) );
  }

  // add a dummy shaft
  m_axle = ChSharedPtr<ChShaft>(new ChShaft);
  m_axle->SetName("dummy shaft");
  m_system->Add(m_axle);
}


TrackVehicleM113::~TrackVehicleM113()
{
  if(m_ownsSystem)
    delete m_system;
}

// Set any collision geometry on the hull, then Initialize() all subsystems
void TrackVehicleM113::Initialize(const ChCoordsys<>& chassis_Csys)
{
  // move the chassis REF frame to the specified initial position/orientation
  m_chassis->SetFrame_REF_to_abs(ChFrame<>(chassis_Csys));

  // add collision geometry to the chassis
  AddCollisionGeometry();

  // initialize the subsystems with the initial c-sys and specified offsets
  for (int i = 0; i < m_num_tracks; i++)
  {
    m_TrackSystems[i]->Initialize(m_chassis, m_TrackSystem_locs[i],
      dynamic_cast<ChTrackVehicle*>(this),
      m_damping);
  }

  // initialize the powertrain, drivelines
  for (int j = 0; j < m_num_engines; j++)
  {
    size_t driveGear_R_idx = 2*j;
    size_t driveGear_L_idx = 2*j + 1;
    m_ptrains[j]->Initialize(m_chassis, m_axle );
  }

}

// 
void TrackVehicleM113::Update(double	time,
                          const std::vector<double>&  throttle,
                          const std::vector<double>&  braking)
{
  assert( throttle.size() == m_num_tracks);
  
  // throttle is applied as the rotational speed on the gears
  for (int i = 0; i < m_num_tracks; i++)
  {
    m_TrackSystems[i]->Update(time, throttle[i]);
  }
}

void TrackVehicleM113::Advance(double step)
{
  double t = 0;
  double settlePhaseA = 0.25;
  double settlePhaseB = 0.5;
  m_system->SetIterLCPmaxItersStab(100);
  m_system->SetIterLCPmaxItersSpeed(150);
  while (t < step) {
    double h = std::min<>(m_stepsize, step - t);
    if( m_system->GetChTime() < settlePhaseA )
    {
      m_system->SetIterLCPmaxItersStab(100);
      m_system->SetIterLCPmaxItersSpeed(100);
      // h = step/2.0;
    } else if ( m_system->GetChTime() < settlePhaseB )
    {
      m_system->SetIterLCPmaxItersStab(150);
      m_system->SetIterLCPmaxItersSpeed(200);
      // h = step/2.0;
    }
    m_system->DoStepDynamics(h);
    t += h;
  }
}


// call the chain function to update the constant damping coef.
void TrackVehicleM113::SetShoePinDamping(double damping)
{
  m_damping = damping;
  for( int i = 0; i < m_num_tracks; i++)
  {
    (m_TrackSystems[i]->GetTrackChain())->Set_pin_friction(damping);
  }
}

double TrackVehicleM113::GetDriveshaftSpeed(size_t idx) const
{
  assert(idx < m_num_tracks );
  return m_axle->GetPos_dt();
}


const ChSharedPtr<TrackPowertrain> TrackVehicleM113::GetPowertrain(size_t idx) const
{ 
  assert( idx < m_num_engines );
  return  m_ptrains[idx];
}

} // end namespace chrono
