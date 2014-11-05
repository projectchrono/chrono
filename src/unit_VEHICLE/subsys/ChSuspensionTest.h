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
// Authors: Justin Madsen, Radu Serban
// =============================================================================
//
// Base class for a suspension testing subsystem, replaces ChVehicle, since no
//  need for braking and throttle. All front subsystems are still included, and
//  there are multiple ways to actuate the wheels.
//
// The reference frame follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_SUSPENSIONTEST_H
#define CH_SUSPENSIONTEST_H

#include <vector>

#include "core/ChVector.h"
#include "physics/ChSystem.h"
#include "physics/ChBodyAuxRef.h"

#include "subsys/ChApiSubsys.h"
#include "subsys/ChSubsysDefs.h"
#include "subsys/ChSuspension.h"
#include "subsys/ChSteering.h"
#include "subsys/ChWheel.h"
#include "models/ModelDefs.h"

namespace chrono {

///
/// Base class for suspension and steering tester
/// This class provides the interface between the systems
///
class CH_SUBSYS_API ChSuspensionTest : public ChSystem
{
public:

  ChSuspensionTest();
  virtual ~ChSuspensionTest() {}

  /// Get a handle to the vehicle's chassis body.
  const ChSharedPtr<ChBody> GetChassis() const { return m_chassis; }

  /// Get the global location of the chassis reference frame origin.
  const ChVector<>& GetChassisPos() const { return m_chassis->GetFrame_REF_to_abs().GetPos(); }

  /// Get the orientation of the chassis reference frame.
  /// The chassis orientation is returned as a quaternion representing a
  /// rotation with respect to the global reference frame.
  const ChQuaternion<>& GetChassisRot() const { return m_chassis->GetFrame_REF_to_abs().GetRot(); }

  /// Get the global location of the chassis center of mass.
  const ChVector<>& GetChassisPosCOM() const { return m_chassis->GetPos(); }

  /// Get the orientation of the chassis centroidal frame.
  /// The chassis orientation is returned as a quaternion representing a
  /// rotation with respect to the global reference frame.
  const ChQuaternion<>& GetChassisRotCOM() const { return m_chassis->GetRot(); }

  /// Get a handle to the specified wheel body.
  ChSharedPtr<ChBody> GetWheelBody(const ChWheelID& wheelID) const;

  /// Get the global location of the specified wheel.
  const ChVector<>& GetWheelPos(const ChWheelID& wheel_id) const;

  /// Get the global rotation of the specified wheel.
  const ChQuaternion<>& GetWheelRot(const ChWheelID& wheel_id) const;

  /// global linear velocity of wheel
  const ChVector<>& GetWheelLinVel(const ChWheelID& wheel_id) const;

  /// global angular velocity of wheel
  const ChVector<>& GetWheelAngVel(const ChWheelID& wheel_id) const;


  /// Get the complete state for the specified wheel.
  /// In this case, no wheel spin (omega) is given, since the wheels are locked.
  ChWheelState GetWheelState(const ChWheelID& wheel_id) const;

  /// Get the local driver position and orientation.
  /// This is a coordinate system relative to the chassis reference frame.
  virtual ChCoordsys<> GetLocalDriverCoordsys() const = 0;

  /// Get the global location of the driver.
  ChVector<> GetDriverPos() const;

  /// Initialize this chassis at the specified global location and orientation.
  virtual void Initialize(
    const ChCoordsys<>& chassisPos  ///< [in] initial global position and orientation
    ) {}

  /// set the actuator function on the left wheel
  virtual void SetActuator_func_L(const ChSharedPtr<ChFunction>& funcL) {
    m_actuator_L = funcL;
  }

  virtual void SetActuator_func_R(const ChSharedPtr<ChFunction>& funcR) {
    m_actuator_R = funcR;
  }

  /// Update the state at the current time.
  /// steering between -1 and +1, and no force need be applied if using external actuation
  virtual void Update(
    double              time,               ///< [in] current time
    double              steering,           ///< [in] current steering input [-1,+1]
    const ChTireForces& tire_forces         ///< [in] tires force to apply to wheel
    ) {}

  /// Advance the state of this vehicle by the specified time step.
  virtual void Advance(double step);

  /// Set the integration step size for the vehicle subsystem.
  void SetStepsize(double val) { m_stepsize = val; }

  /// Get the current value of the integration step size for the vehicle
  /// subsystem.
  double GetStepsize() const { return m_stepsize; }

  /// Log current constraint violations.
  void LogConstraintViolations();

protected:

  ChSharedPtr<ChBodyAuxRef>  m_chassis;     ///< handle to the chassis body, fixed to ground
  ChSuspensionList           m_suspensions; ///< list of handles to suspension subsystems, only 1 in this case.
  ChSharedPtr<ChBody>        m_post_L;      ///< left shaker post  
  ChSharedPtr<ChBody>        m_post_R;      ///< right shaker post

  ChSharedPtr<ChSteering>    m_steering;     ///< handle to the steering subsystem.
  ChWheelList                m_wheels;       ///< list of handles to wheel subsystems, 2 in this case.

  double                     m_stepsize;   ///< integration step-size for the vehicle system
  ChSharedPtr<ChFunction>    m_actuator_L;  ///< actuator function applied to left wheel
  ChSharedPtr<ChFunction>    m_actuator_R;  ///< actuator function applied to right wheel

  friend class ChIrrGuiST;
};


} // end namespace chrono


#endif	// CH_SUSPENSIONTEST_H
