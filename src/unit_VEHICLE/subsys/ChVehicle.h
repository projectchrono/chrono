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

#include <vector>

#include "core/ChVector.h"
#include "physics/ChSystem.h"
#include "physics/ChBodyAuxRef.h"

#include "subsys/ChApiSubsys.h"

namespace chrono {


enum ChWheelId {
  FRONT_LEFT,
  FRONT_RIGHT,
  REAR_LEFT,
  REAR_RIGHT
};

struct ChBodyState {
  ChVector<>     pos;
  ChQuaternion<> rot;
  ChVector<>     lin_vel;
  ChVector<>     ang_vel;
};

struct ChTireForce {
  ChVector<> force;
  ChVector<> point;
  ChVector<> moment;
};

typedef std::vector<ChTireForce> ChTireForces;

class ChPowertrain;
class ChDriveline;

class CH_SUBSYS_API ChVehicle : public ChSystem {
public:
  ChVehicle();
  virtual ~ChVehicle() {}

  virtual ChSharedPtr<ChBody> GetWheelBody(ChWheelId which) const = 0;

  virtual const ChVector<>& GetWheelPos(ChWheelId which) const = 0;
  virtual const ChQuaternion<>& GetWheelRot(ChWheelId which) const = 0;
  virtual const ChVector<>& GetWheelLinVel(ChWheelId which) const = 0;
  virtual ChVector<> GetWheelAngVel(ChWheelId which) const = 0;
  virtual double GetWheelOmega(ChWheelId which) const = 0;

  ChBodyState GetWheelState(ChWheelId which);

  virtual void Initialize(const ChCoordsys<>& chassisPos) {}
  virtual void Update(double              time,
                      double              throttle,
                      double              steering,
                      double              braking,
                      const ChTireForces& tire_forces) {}
  virtual void Advance(double step);

  const ChSharedPtr<ChBodyAuxRef> GetChassis() const  { return m_chassis; }

  /// Return the position and orientation of the chassis reference frame
  const ChVector<>&     GetChassisPos() const         { return m_chassis->GetFrame_REF_to_abs().GetPos(); }
  const ChQuaternion<>& GetChassisRot() const         { return m_chassis->GetFrame_REF_to_abs().GetRot(); }

  /// Return the position and orientation of the chassis COM
  const ChVector<>&     GetChassisPosCOM() const      { return m_chassis->GetPos(); }
  const ChQuaternion<>& GetChassisRotCOM() const      { return m_chassis->GetRot(); }

  /// Return the speed measured at the origin of the chassis reference frame
  double GetVehicleSpeed() const      { return m_chassis->GetFrame_REF_to_abs().GetPos_dt().Length(); }

  /// Return the speed measured at the chassis COM
  double GetVehicleSpeedCOM() const   { return m_chassis->GetPos_dt().Length(); }

  /// Set/get the integration step size for the vehicle subsystem
  void SetStepsize(double val) { m_stepsize = val; }
  double GetStepsize() const { return m_stepsize; }

protected:
  ChSharedPtr<ChBodyAuxRef>  m_chassis;

  ChDriveline*     m_driveline;
  ChPowertrain*    m_powertrain;

  double  m_stepsize;

  friend class ChPowertrain;
  friend class ChDriveline;
  friend class ChIrrGuiDriver;
};


} // end namespace chrono


#endif
