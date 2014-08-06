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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Base class for all suspension subsystems
//
// =============================================================================

#ifndef CH_SUSPENSION_H
#define CH_SUSPENSION_H

#include <string>

#include "core/ChShared.h"
#include "physics/ChSystem.h"

#include "subsys/ChApiSubsys.h"
#include "subsys/ChWheel.h"

namespace chrono {


class CH_SUBSYS_API ChSuspension : public ChShared
{
public:

  enum Side {
    LEFT,
    RIGHT
  };

  ChSuspension(const std::string& name,
               Side               side,
               bool               driven = false);

  virtual ~ChSuspension() {}

  virtual void Initialize(ChSharedBodyPtr   chassis,
                          const ChVector<>& location) = 0;

  virtual void AttachWheel(ChSharedPtr<ChWheel> wheel) = 0;

  virtual void ApplySteering(double displ) = 0;
  virtual void ApplyTorque(double torque) = 0;

  virtual const ChVector<>& GetSpindlePos() const = 0;
  virtual const ChQuaternion<>& GetSpindleRot() const = 0;
  virtual double GetSpindleAngSpeed() const = 0;

protected:

  std::string       m_name;
  Side              m_side;
  bool              m_driven;

};


} // end namespace chrono


#endif