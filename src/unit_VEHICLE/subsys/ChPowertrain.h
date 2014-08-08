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
// Base class for a vehicle powertrain.
//
// =============================================================================

#ifndef CH_POWERTRAIN_H
#define CH_POWERTRAIN_H

#include "core/ChShared.h"
#include "core/ChVector.h"
#include "physics/ChBody.h"

#include "subsys/ChApiSubsys.h"
#include "subsys/ChVehicle.h"

namespace chrono {


enum ChDriveType {
  FWD,
  RWD,
  AWD
};

class CH_SUBSYS_API ChPowertrain : public ChShared {
public:

  ChPowertrain(ChVehicle* car, ChDriveType type);

  virtual ~ChPowertrain() {}

  virtual double GetMotorSpeed() const = 0;
  virtual double GetMotorTorque() const = 0;
  virtual double GetWheelTorque(ChWheelId which) const = 0;

  virtual void Update(double time, double throttle) = 0;

protected:
  ChVehicle*    m_car;
  ChDriveType   m_type;
};


} // end namespace chrono


#endif
