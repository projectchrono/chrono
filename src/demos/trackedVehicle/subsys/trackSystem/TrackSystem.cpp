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
// model a single track chain system, as part of a tracked vehicle. Uses JSON input files
//
// =============================================================================

#include <cstdio>

#include "subsys/trackSystem/TrackSystem.h"

namespace chrono {


TrackSystem::TrackSystem(const std::string& name, int track_idx)
  : m_track_idx(track_idx), m_name( name)
{
  // FILE* fp = fopen(filename.c_str(), "r");
  // char readBuffer[65536];
  // fclose(fp);

  Create(track_idx);

}


void TrackSystem::Create(int track_idx)
{
  /*
  // read idler info
  assert(d.HasMember("Idler"));
  m_idlerMass = d["Idler"]["Mass"].GetDouble();
  m_idlerPos = loadVector(d["Idler"]["Location"]);
  m_idlerInertia = loadVector(d["Idler"]["Inertia"]);
  m_idlerRadius = d["Spindle"]["Radius"].GetDouble();
  m_idlerWidth = d["Spindle"]["Width"].GetDouble();
  m_idler_K = d["Idler"]["SpringK"].GetDouble();
  m_idler_C = d["Idler"]["SpringC"].GetDouble();

  // Read Drive Gear data
  assert(d.HasMember("Drive Gear"));
  assert(d["Drive Gear"].IsObject());

  m_gearMass = d["Drive Gear"]["Mass"].GetDouble();
  m_gearPos = loadVector(d["Drive Gear"]["Location"]);
  m_gearInertia = loadVector(d["Drive Gear"]["Inertia"]);
  m_gearRadius = d["Drive Gear"]["Radius"].GetDouble();
  m_gearWidth = d["Drive Gear"]["Width"].GetDouble();

  // Read Support Roller info
  assert(d.HasMember("Support Roller"));
  assert(d["Support Roller"].IsObject());

  m_rollerMass = d["Support Roller"]["Mass"].GetDouble();
  m_rollerInertia = loadVector(d["Support Roller"]["Inertia"]);
  m_rollerRadius = d["Support Roller"]["Radius"].GetDouble();
  m_rollerWidth = d["Support Roller"]["Width"].GetDouble();
  
  assert(d["Support Roller"]["Location"].IsArray());
  m_NumRollers = d["Support Roller"]["Location"].Size();
  
  m_rollerLocs.resize(m_NumRollers);
  for(int i = 0; i < m_NumRollers; i++ )
  {
	m_rollerLocs[i] = loadVector(d["Support Roller"]["Location"][i]);
  }

  // Read Suspension data
  assert(d.HasMember("Suspension"));
  assert(d["Suspension"].IsObject());
  assert(d["Suspension"]["Location"].IsArray() );
  
  m_suspensionFilename = d["Suspension"]["Input File"].GetString();
  m_NumSuspensions = d["Suspension"]["Location"].Size();
  
  m_suspensionLocs.resize(m_NumSuspensions);
  for(int j = 0; j < m_NumSuspensions; j++)
  {
    m_suspensionLocs[j] = loadVector(d["Suspension"]["Locaiton"][j]);
  }

  // Read Track Chain data
  assert(d.HasMember("Track Chain"));
  assert(d["Track Chain"].IsObject()); 
  m_trackChainFilename = d["Track Chain"]["Input File"].GetString()
  
  // create the various subsystems
  BuildSubsystems();

  */
}

void TrackSystem::BuildSubsystems()
{
  // build one of each of the following subsystems
  m_driveGear = ChSharedPtr<DriveGear>(new DriveGear(m_gearMass, m_gearInertia, m_gearRadius, m_gearWidth));
  m_idler = ChSharedPtr<IdlerSimple>(new IdlerSimple(m_idlerMass, m_idlerInertia, m_idlerRadius, m_idlerWidth, m_idler_K, m_idler_C ));
  m_chain = ChSharedPtr<TrackChain>(new TrackChain(m_trackChainFilename));
  
  // build suspension/road wheel subsystems
  m_suspensions.resize(m_NumSuspensions);
  for(int i = 0; i < m_NumSuspensions; i++)
  {
    m_suspensions[i] = ChSharedPtr<TorsionArmSuspension>(new TorsionArmSuspension(m_suspensionFilename));
  }
  
  // build support wheel subsystems
  m_supportRollers.resize(m_NumRollers);
  for(int j = 0; j < m_NumRollers; j++)
  {
    m_supportRollers[i] = ChSharedPtr<ChBody>(new ChBody);
  }

}

void TrackSystem::Initialize(ChSharedPtr<ChBodyAuxRef> chassis,
			 const ChVector<>&         location,
			 const ChQuaternion<>&     rotation)
{
  m_driveGear->Initialize(chassis, m_gearPos, m_gearMass, m_gearInertia, m_gearRadius); 
  m_idler->Initialize(chassis, m_idlerPos, m_idlerMass, m_idlerInertia, m_idler_K, m_idler_C, m_idlerRadius, m_idlerWidth);
  
  for(int i = 0; i < m_suspensionLocs.size(); i++)
  {
    m_suspensions[i]->Initialize(chassis, m_suspensionFileName);
  }
  
}


} // end namespace chrono
