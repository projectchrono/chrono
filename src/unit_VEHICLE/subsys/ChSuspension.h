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
#include "physics/ChBodyAuxRef.h"
#include "physics/ChShaft.h"
#include "physics/ChShaftsBody.h"

#include "subsys/ChApiSubsys.h"
#include "subsys/ChVehicle.h"

namespace chrono {


class CH_SUBSYS_API ChSuspension : public ChShared
{
public:

  enum Side {
    LEFT  = 0,
    RIGHT = 1
  };

  ChSuspension(const std::string& name,
               bool               steerable = false,
               bool               driven = false);

  virtual ~ChSuspension() {}

  const std::string& GetName() const { return m_name; }
  void SetName(const std::string& name) { m_name = name; }

  bool IsSteerable() const { return m_steerable; }
  void SetSteerable(bool val) { m_steerable = val; }

  bool IsDriven() const { return m_driven; }
  void SetDriven(bool val) { m_driven = val; }

  ChSharedPtr<ChBody>  GetSpindle(Side side) const { return m_spindle[side]; }
  ChSharedPtr<ChShaft> GetAxle(Side side) const    { return m_axle[side]; }

  ChSharedPtr<ChLinkLockRevolute>  GetRevolute(Side side) const { return m_revolute[side]; }

  const ChVector<>& GetSpindlePos(Side side) const       { return m_spindle[side]->GetPos(); }
  const ChQuaternion<>& GetSpindleRot(Side side) const   { return m_spindle[side]->GetRot(); }
  const ChVector<>& GetSpindleLinVel(Side side) const    { return m_spindle[side]->GetPos_dt(); }
  ChVector<> GetSpindleAngVel(Side side) const           { return m_spindle[side]->GetWvel_par(); }

  double GetAxleSpeed(Side side) const;

  void ApplyTireForce(Side side, const ChTireForce& tire_force);
  void ApplyAxleTorque(Side side, double torque);

  virtual void Initialize(ChSharedPtr<ChBodyAuxRef>  chassis,
                          const ChVector<>&          location) = 0;

  virtual void ApplySteering(double displ) = 0;

protected:
  std::string               m_name;
  bool                      m_driven;
  bool                      m_steerable;

  ChSharedPtr<ChBody>               m_spindle[2];
  ChSharedPtr<ChShaft>              m_axle[2];
  ChSharedPtr<ChShaftsBody>         m_axle_to_spindle[2];
  ChSharedPtr<ChLinkLockRevolute>   m_revolute[2];
};


} // end namespace chrono


#endif
