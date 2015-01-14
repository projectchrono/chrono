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
public:

  /// specify name and a unique track identifier
  TrackSystem(const std::string& filename, int track_idx);

  ~TrackSystem() {}

  /// Initialize by attaching subsystem to the specified chassis body at the
  /// specified location (with respect to and expressed in the reference frame
  /// of the chassis). It is assumed that the suspension reference frame is
  /// always aligned with the chassis reference frame.
  void Initialize(ChSharedPtr<ChBodyAuxRef> chassis,
				 const ChVector<>&         location);

  void Create(int track_idx);

  /// handle to the drive gear subsystem, to initialize the driveline
  ChSharedPtr<DriveGear> GetDriveGear() { return m_driveGear; }
  
private:

  // private functions
  void BuildSubsystems();
  
  // initialize a roller at the specified location and orientation, attach to chassis
  void initialize_roller(ChSharedPtr<ChBody> body, ChSharedPtr<ChBodyAuxRef> chassis,
    const ChVector<>& loc, const ChQuaternion<>& rot, int idx);

  // private variables
  // subsystems, and other bodies attached to this tracksystem
  ChSharedPtr<DriveGear> m_driveGear;
  ChSharedPtr<IdlerSimple>	m_idler;
  ChSharedPtr<TrackChain> m_chain;
  std::vector<ChSharedPtr<TorsionArmSuspension>> m_suspensions;
  std::vector<ChSharedPtr<ChBody>> m_supportRollers;
  std::vector<ChSharedPtr<ChLinkLockRevolute>> m_supportRollers_rev;
  
  std::string m_name; ///< name of the track chain system
  ChVector<> m_Pos_local; ///< location of ref-frame, w.r.t. chassis c-sys

  // hard-coded in TrackSystem.cpp, for now
  // idler
  static const ChVector<> m_idlerPos; // relative to TrackSystem _REF c-sys
  static const ChQuaternion<> m_idlerRot; 
  
  // drive gear
  static const ChVector<> m_gearPos;  // relative to Tracksystem _REF c-sys
  static const ChQuaternion<> m_gearRot;
  
  // Support rollers
  static const int m_numRollers;
  static const double m_roller_mass;
  static const ChVector<> m_roller_inertia;
  static const double m_roller_radius;
  static const double m_roller_width;
  std::vector<ChVector<> > m_rollerLocs;  ///< relative to the Tracksys _REF c-sys
  std::vector<ChQuaternion<> > m_rollerRots;
  
  // suspension
  // static const std::string m_suspensionFilename;
  std::vector<ChVector<> > m_suspensionLocs;  // relative to local c-sys
  static const int m_numSuspensions;
  static const ChVector<> m_armWheel;   // relative arm distance to wheel
  
  // Track Chain
  // std::string m_trackChainFilename;
  int m_track_idx;  // give unique ID to each TrackSystem, to use as a collision family ID for all associated sub-systems
};


} // end namespace chrono


#endif
