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
// The drive gear propels the tracked vehicle
//
// =============================================================================

#ifndef DRIVEGEAR_H
#define DRIVEGEAR_H

#include "subsys/ChApiSubsys.h"
#include "physics/ChSystem.h"
#include "physics/ChBodyAuxRef.h"

namespace chrono {


/// Drive gear class, a single rigid body. Attached to the chassis
class CH_SUBSYS_API DriveGear : public ChShared
{
public:

  DriveGear(double mass, const ChVector<>& inertia, double rad, double width);
  ~DriveGear() {}

  void Initialize(ChSharedPtr<ChBodyAuxRef> chassis, const ChVector<>& pos, const ChQuaternion<>& rot);

  ChSharedPtr<ChBody> GetBody() { return m_gear; }
  
  
private:
  // private functions
  const std::string& getMeshName() const { return m_meshName; }
  const std::string& getMeshFile() const { return m_meshFile; }

  
  // private variables
  ChSharedPtr<ChBody> m_gear;
  double m_radius;
  double m_width;

  int m_visType;    // 0 = none, 1 = primitive, 2 = mesh
  static const std::string m_meshName;
  static const std::string m_meshFile;
  
};


} // end namespace chrono


#endif
