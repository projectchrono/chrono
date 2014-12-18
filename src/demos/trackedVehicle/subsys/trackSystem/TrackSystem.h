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


  void Initialize(ChSharedPtr<ChBodyAuxRef> chassis,
				 const ChVector<>&         location,
				 const ChQuaternion<>&     rotation);

  void Create(int track_idx);
  
private:

  // private functions
  void BuildSubsystems();
  
  // initialize a roller at the specified location and orientation
  void initialize_roller(ChSharedPtr<ChBody> body, const ChVector<>& loc, const ChQuaternion<>& rot, int idx);

  // private variables
  ChSharedPtr<DriveGear> m_driveGear;
  ChSharedPtr<IdlerSimple>	m_idler;
  ChSharedPtr<TrackChain> m_chain;
  std::vector<ChSharedPtr<TorsionArmSuspension>> m_suspensions;
  std::vector<ChSharedPtr<ChBody>> m_supportRollers;
  std::vector<ChVector<> > m_rollerLocs;  ///< relative to the 
  std::vector<ChSharedPtr<ChLinkLockRevolute>> m_supportRollers_rev;
  
  std::string m_name; ///< name of the track chain system

  // hard-coded in TrackSystem.cpp, for now
  // idler
  static const double m_idlerMass;
  static const ChVector<> m_idlerInertia;
  static const ChVector<> m_idlerPos;
  static const double m_idlerRadius;
  static const double m_idlerWidth;
  static const double m_idler_K;
  static const double m_idler_C;
  
  // drive gear
  static const double m_gearMass;
  static const ChVector<> m_gearPos;
  static const double m_gearRadius;
  static const double m_gearWidth;
  static const ChVector<> m_gearInertia;
  
  // Support rollers
  static const double m_rollerMass;
  static const double m_rollerRadius;
  static const double m_rollerWidth;
  static const ChVector<> m_rollerInertia;

  static const int m_NumRollers;
  
  // suspension
  // static const std::string m_suspensionFilename;
  std::vector<ChVector<> > m_suspensionLocs;
  static const int m_NumSuspensions;
  
  // Track Chain
  std::string m_trackChainFilename;
  int m_track_idx;
};


} // end namespace chrono


#endif
