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
// Base class for a vehicle driveline.
//
// =============================================================================

#ifndef CH_DRIVELINE_H
#define CH_DRIVELINE_H

#include "core/ChShared.h"
#include "core/ChVector.h"
#include "physics/ChShaft.h"

#include "subsys/ChApiSubsys.h"
#include "subsys/ChVehicle.h"

namespace chrono {


class CH_SUBSYS_API ChDriveline : public ChShared {
public:

  enum DriveType {
    FWD,
    RWD,
    AWD
  };

  ChDriveline(ChVehicle* car,
              DriveType  type);

  virtual ~ChDriveline() {}

  ChSharedPtr<ChShaft> GetDriveshaft() const { return m_driveshaft; }

  double GetDriveshaftSpeed() const          { return m_driveshaft->GetPos_dt(); }
  void ApplyDriveshaftTorque(double torque)  { m_driveshaft->SetAppliedTorque(torque); }

  virtual double GetWheelTorque(ChWheelId which) const = 0;

protected:
  ChVehicle*            m_car;
  DriveType             m_type;

  ChSharedPtr<ChShaft>  m_driveshaft;
};


} // end namespace chrono


#endif
