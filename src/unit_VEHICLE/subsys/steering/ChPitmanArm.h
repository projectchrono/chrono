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
// Authors: Radu Serban
// =============================================================================
//
// Base class for a Pitman Arm steering subsystem.
// Derived from ChSteering, but still an abstract base class.
//
// =============================================================================

#ifndef CH_PITMANARM_H
#define CH_PITMANARM_H

#include "subsys/ChApiSubsys.h"
#include "subsys/ChSteering.h"

namespace chrono {


class CH_SUBSYS_API ChPitmanArm : public ChSteering {
public:
  ChPitmanArm(const std::string& name);
  virtual ~ChPitmanArm() {}

  virtual void Initialize(ChSharedBodyPtr   chassis,
                          const ChVector<>& location);
protected:
  ChSharedBodyPtr   m_arm;
  ChSharedBodyPtr   m_link;

  ChSharedPtr<ChLinkLockRevolute>      m_revolute;
  ChSharedPtr<ChLinkRevoluteSpherical> m_revsph;
  ChSharedPtr<ChLinkLockUniversal>     m_universal;
};


} // end namespace chrono


#endif
