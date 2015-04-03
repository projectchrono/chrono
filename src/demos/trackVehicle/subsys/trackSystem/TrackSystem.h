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
// Model a single track chain system, as part of a tracked vehicle. This subsystem
//  contains a number of other subsystems.
//
// =============================================================================

#ifndef TRACKSYSTEM_H
#define TRACKSYSTEM_H

#include "physics/ChSystem.h"
#include "subsys/ChApiSubsys.h"
#include "physics/ChBodyAuxRef.h"

#include "subsys/idler/IdlerSimple.h"
#include "subsys/driveGear/DriveGear.h"
#include "subsys/trackChain/TrackChain.h"
#include "subsys/suspension/TorsionArmSuspension.h"

namespace chrono {


class CH_SUBSYS_API TrackSystem : public ChShared
{
friend class TrackVehicle;
public:

  /// specify name and a unique track identifier
  TrackSystem(const std::string& filename, int track_idx);

  ~TrackSystem() {}

  /// Initialize by attaching subsystem to the specified chassis body at the
  /// specified location (with respect to and expressed in the reference frame
  /// of the chassis). It is assumed that the suspension reference frame is
  /// always aligned with the chassis reference frame.
  void Initialize(ChSharedPtr<ChBodyAuxRef> chassis,
    const ChVector<>&  location,
    ChTrackVehicle* vehicle,
    double pin_damping = 0);

  void Create(int track_idx);

  // Accessors

  /// handle to the drive gear subsystem, to initialize the driveline
  const ChSharedPtr<DriveGear> GetDriveGear() const { return m_driveGear; }

  /// handle to track chain
  ChSharedPtr<TrackChain> GetTrackChain() const { return m_chain; }
  
  // subsystem relative to trackSystem coords
  const ChVector<>& Get_gearPosRel() const { return m_gearPosRel; }
  
  // subsystem relative to trackSystem coords
  const ChVector<>& Get_idlerPosRel() const { return m_idlerPosRel; }

  /// get the reaction force vector from the spring in the idler subsystem
  const ChVector<> Get_idler_spring_react();

private:

  // private functions
  void BuildSubsystems();
  
  // private variables
  // subsystems, and other bodies attached to this tracksystem
  ChSharedPtr<DriveGear>    m_driveGear;
  ChSharedPtr<IdlerSimple>	m_idler;
  ChSharedPtr<TrackChain>   m_chain;
  std::vector<ChSharedPtr<TorsionArmSuspension> > m_suspensions;
  
  std::string m_name; ///< name of the track chain system
  ChVector<> m_local_pos; ///< location of ref-frame, w.r.t. chassis c-sys

  ChVector<> m_gearPosRel;
  ChVector<> m_idlerPosRel;
  double  m_idler_preload;

  int m_track_idx;  // give unique ID to each TrackSystem, to use as a collision family ID for all associated sub-systems 
  
  // hard-coded in TrackSystem.cpp, for now
  // idler
  static const ChVector<>     m_idlerPos; // relative to TrackSystem _REF c-sys
  static const ChQuaternion<> m_idlerRot; 
  
  // drive gear
  static const ChVector<> m_gearPos;  // relative to Tracksystem _REF c-sys
  static const ChQuaternion<> m_gearRot;
  
  // suspension
  std::vector<ChVector<> > m_suspensionLocs;  // relative to local c-sys
  static const int        m_numSuspensions;
  static const ChVector<> m_armWheel;   // relative arm distance to wheel
};


} // end namespace chrono


#endif
