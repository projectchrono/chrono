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
#include "physics/ChShaft.h"
#include "physics/ChShaftsBody.h"

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

  const ChSharedPtr<ChBody>  GetSpindle() const { return m_spindle; }
  const ChSharedPtr<ChShaft> GetAxle() const    { return m_axle; }

  const ChVector<>& GetSpindlePos() const       { return m_spindle->GetPos(); }
  const ChQuaternion<>& GetSpindleRot() const   { return m_spindle->GetRot(); }
  const ChVector<>& GetSpindleLinVel() const    { return m_spindle->GetPos_dt(); }
  ChVector<> GetSpindleAngVel() const           { return m_spindle->GetWvel_par(); }

  void ApplyAxleTorque(double torque);
  double GetAxleSpeed() const;

  virtual void Initialize(ChSharedPtr<ChBody>  chassis,
                          const ChVector<>&    location) = 0;

  virtual void AttachWheel(ChSharedPtr<ChWheel> wheel) = 0;

  virtual void ApplySteering(double displ) = 0;

protected:
  ChSharedPtr<ChBody>       m_spindle;
  ChSharedPtr<ChShaft>      m_axle;
  ChSharedPtr<ChShaftsBody> m_axle_to_spindle;

  std::string       m_name;
  Side              m_side;
  bool              m_driven;
};


} // end namespace chrono


#endif
