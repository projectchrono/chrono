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
#include <vector>

#include "core/ChShared.h"
#include "physics/ChSystem.h"
#include "physics/ChBodyAuxRef.h"
#include "physics/ChShaft.h"
#include "physics/ChShaftsBody.h"

#include "subsys/ChApiSubsys.h"
#include "subsys/ChVehicle.h"

namespace chrono {

///
/// Base class for a suspension subsystem.
///
class CH_SUBSYS_API ChSuspension : public ChShared
{
public:

  enum Side {
    LEFT  = 0,
    RIGHT = 1
  };

  ChSuspension(
    const std::string& name,               ///< [in] name of the subsystem
    bool               driven = false      ///< [in] true if attached to driveline subsystem
    );

  virtual ~ChSuspension() {}

  /// Specify whether or not this suspension can be steered.
  virtual bool IsSteerable() const = 0;

  /// Get the name identifier for this suspension subsystem.
  const std::string& GetName() const { return m_name; }

  /// Set the name identifier for this suspension subsystem.
  void SetName(const std::string& name) { m_name = name; }

  /// Return true if attached to driveline subsystem and false otherwise.
  bool IsDriven() const { return m_driven; }

  /// Get a handle to the spindle body on the specified side.
  ChSharedPtr<ChBody>  GetSpindle(Side side) const { return m_spindle[side]; }

  /// Get a handle to the axle shaft on the specified side.
  ChSharedPtr<ChShaft> GetAxle(Side side) const { return m_axle[side]; }

  /// Get a handle to the revolute joint on the specified side.
  ChSharedPtr<ChLinkLockRevolute>  GetRevolute(Side side) const { return m_revolute[side]; }

  /// Get the global location of the spindle on the specified side.
  const ChVector<>& GetSpindlePos(Side side) const { return m_spindle[side]->GetPos(); }

  /// Get the orientation of the spindle body on the specified side.
  /// The spindle body orientation is returned as a quaternion representing a
  /// rotation with respect to the global reference frame.
  const ChQuaternion<>& GetSpindleRot(Side side) const { return m_spindle[side]->GetRot(); }

  /// Get the linear velocity of the spindle body on the specified side.
  /// Return the linear velocity of the spindle center, expressed in the global
  /// reference frame.
  const ChVector<>& GetSpindleLinVel(Side side) const { return m_spindle[side]->GetPos_dt(); }

  /// Get the angular velocity of the spindle body on the specified side.
  /// Return the angular velocity of the spindle frame, expressed in the global
  /// reference frame.
  ChVector<> GetSpindleAngVel(Side side) const { return m_spindle[side]->GetWvel_par(); }

  /// Get the angular speed of the axle on the specified side.
  double GetAxleSpeed(Side side) const;

  /// Apply the provided tire forces.
  /// The given tire force and moment is applied to the specified (left or
  /// right) spindle body.  This function provides the interface to the tire
  /// system (intermediated by the vehicle system).
  void ApplyTireForce(Side side, const ChTireForce& tire_force);

  /// Apply the provided motor torque.
  /// The given torque is applied to the specified (left or right) axle. This
  /// function provides the interface to the drivetrain subsystem (intermediated
  /// by the vehicle system).
  void ApplyAxleTorque(Side side, double torque);

  /// Initialize this suspension subsystem.
  /// The suspension subsystem is initialized by attaching it to the specified
  /// chassis body at the specified location (with respect to and expressed in
  /// the reference frame of the chassis). It is assumed that the suspension
  /// reference frame is always aligned with the chassis reference frame.
  /// Finally, tierod_body is a handle to the body to which the suspension
  /// tierods are to be attached. For a steerable suspension, this will be the
  /// steering link of a suspension subsystem.  Otherwise, this is the chassis.
  virtual void Initialize(
    ChSharedPtr<ChBodyAuxRef>  chassis,     ///< [in] handle to the chassis body
    const ChVector<>&          location,    ///< [in] location relative to the chassis frame
    ChSharedPtr<ChBody>        tierod_body  ///< [in] body to which tireods are connected
    ) = 0;

  /// Log current constraint violations.
  virtual void LogConstraintViolations(Side side) {}

protected:

  std::string               m_name;       ///< name of the subsystem
  bool                      m_driven;     ///< true if attached to steering subsystem

  ChSharedPtr<ChBody>               m_spindle[2];         ///< handles to spindle bodies
  ChSharedPtr<ChShaft>              m_axle[2];            ///< handles to axle shafts
  ChSharedPtr<ChShaftsBody>         m_axle_to_spindle[2]; ///< handles to spindle-shaft connectors
  ChSharedPtr<ChLinkLockRevolute>   m_revolute[2];        ///< handles to spindle revolute joints
};

/// Vector of handles to suspension subsystems.
typedef std::vector<ChSharedPtr<ChSuspension> >  ChSuspensionList;


} // end namespace chrono


#endif
