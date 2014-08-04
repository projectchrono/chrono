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
// Base class for a vehicle model.
//
// =============================================================================

#ifndef CH_VEHICLE_H
#define CH_VEHICLE_H


#include "core/ChShared.h"
#include "core/ChVector.h"
#include "physics/ChBody.h"


namespace chrono {

enum ChWheelId {
  FRONT_LEFT,
  FRONT_RIGHT,
  REAR_LEFT,
  REAR_RIGHT
};

class ChVehicle : public ChShared {
public:
  ChVehicle() {}
  virtual ~ChVehicle() {}

  virtual double GetWheelAngSpeed(ChWheelId which) = 0;

  virtual void Update(double time, double throttle, double steering) = 0;

  const ChSharedBodyPtr GetChassis() const    { return m_chassis; }
  const ChVector<>&     GetChassisPos() const { return m_chassis->GetPos(); }
  const ChQuaternion<>& GetChassisRot() const { return m_chassis->GetRot(); }

protected:
  ChSharedBodyPtr  m_chassis;

};


} // end namespace chrono


#endif
