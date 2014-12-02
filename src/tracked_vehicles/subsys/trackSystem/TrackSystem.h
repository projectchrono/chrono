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

#include "subsys/ChApiSubsys.h"
#include "rapidjson/document.h"

#include "subsys/idler/Idler.h"
#include "subsys/driveGear/DriveGear.h"
#include "subsys/trackChain/TrackChain.h"
#include "subsys/suspension/TorsionArmSuspension.h"

namespace chrono {


class CH_SUBSYS_API TrackSystem
{
public:

  TrackSystem(const std::string& filename);
  TrackSystem(const rapidjson::Document& d);
  ~TrackSystem() {}


  void Initialize(ChSharedPtr<ChBodyAuxRef> chassis,
				 const ChVector<>&         location,
				 const ChQuaternion<>&     rotation));
  
private:

  // private functions
  void Create(const rapidjson::Document& d);

  // private variables
  ChSharedPtr<DriveGear> m_driveGear;
  ChSharedPtr<Idler>	m_idler;
  ChSharedPtr<TrackChain> m_chain;
  std::vector<ChSharedPtr<TorsionArmSuspension>> m_suspensions;
  std::vector<ChSharedPtr<ChBody>> m_supportRollers;
  
  // idler
  double m_idlerMass;
  ChVector<> m_idlerInertia;
  ChVector<> m_idlerPos
  double m_idlerRadius;
  double m_idlerWidth;
  double m_idler_K;
  double m_idler_C;
  
  // drive gear
  double m_gearMass;
  ChVector<> m_gearPos;
  double m_gearRadius;
  ChVector<> m_gearInertia;
  
  // Support rollers
  double m_rollerMass;
  double m_rollerRadius;
  double m_rollerWidth;
  ChVector<> m_rollerInertia;
  std::vector<ChVector<> > m_rollerLocs;
  
  // suspension
  std::string m_suspensionFilename;
  std::vector<ChVector<> > m_suspensionLocs;
  
  // Track Chain
  std::string m_trackChainFilename;
  
};


} // end namespace chrono


#endif
