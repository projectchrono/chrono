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
  virtual ~TrackSystem() {}


private:

  void Create(const rapidjson::Document& d);


  ChSharedPtr<DriveGear> m_driveGear;
  ChSharedPtr<Idler>	m_idler;
  ChSharedPtr<TrackChain> m_chain;
  std::vector<ChSharedPtr<TorsionArmSuspension>> m_suspensions;
  
  std::vector<ChSharedPtr<ChBody>> m_supportRollers;
};


} // end namespace chrono


#endif
