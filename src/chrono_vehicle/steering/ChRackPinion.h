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
// Base class for a Rack-Pinion steering subsystem.
// Derived from ChSteering, but still an abstract base class.
//
// The steering subsystem is modeled with respect to a right-handed frame with
// with X pointing towards the front, Y to the left, and Z up (ISO standard).
// The steering link translates along the Y axis. We do not explicitly model the
// pinion but instead use the implied rack-pinion constraint to calculate the
// rack displacement from a given pinion rotation angle.
//
// =============================================================================

#ifndef CH_RACKPINION_H
#define CH_RACKPINION_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSteering.h"

namespace chrono {

///
/// Base class for a Rack-Pinion steering subsystem.
/// Derived from ChSteering, but still an abstract base class.
///
/// The steering subsystem is modeled with respect to a right-handed frame with
/// with X pointing towards the front, Y to the left, and Z up (ISO standard).
/// The steering link translates along the Y axis. We do not explicitly model the
/// pinion but instead use the implied rack-pinion constraint to calculate the
/// rack displacement from a given pinion rotation angle.
///
class CH_VEHICLE_API ChRackPinion : public ChSteering
{
public:

  /// Construct a rack-pinion steering mechanism with given base name.
  ChRackPinion(
    const std::string& name    ///< [in] name of the subsystem
    );

  virtual ~ChRackPinion() {}

  /// Initialize the steering subsystem.
  /// This attached the steering mechanism to the specified chassis body at the
  /// given offset and orientation, relative to the frame of the chassis.
  virtual void Initialize(
    ChSharedPtr<ChBodyAuxRef> chassis,   ///< pin] handle to the chassis body
    const ChVector<>&         location,  ///< [in] location relative to the chassis frame
    const ChQuaternion<>&     rotation   ///< [in] orientation relative to the chassis frame
    );

  /// Update the state of this steering subsystem at the current time.
  /// The steering subsystem is provided the current steering driver input (a
  /// value between -1 and +1).  Positive steering input indicates steering
  /// to the left. This function is called during the vehicle update.
  virtual void Update(
    double time,       ///< [in] current time
    double steering    ///< [in] current steering input [-1,+1]
    );

  /// Log current constraint violations.
  virtual void LogConstraintViolations();

protected:

  /// Return the mass of the steering link.
  virtual double GetSteeringLinkMass() const = 0;

  /// Return the moments of inertia of the steering link.
  virtual ChVector<> GetSteeringLinkInertia() const = 0;

  /// Return the steering link COM offset in Y direction (positive to the left).
  virtual double GetSteeringLinkCOM() const = 0;

  /// Return the radius of the steering link (visualization only).
  virtual double GetSteeringLinkRadius() const = 0;

  /// Return the length of the steering link (visualization only).
  virtual double GetSteeringLinkLength() const = 0;

  /// Return the radius of the pinion.
  virtual double GetPinionRadius() const = 0;

  /// Return the maximum rotation angle of the pinion (in either direction).
  virtual double GetMaxAngle() const = 0;

  ChSharedPtr<ChLinkLockPrismatic> m_prismatic;  ///< handle to the prismatic joint chassis-link
  ChSharedPtr<ChLinkLinActuator> m_actuator;     ///< handle to the linear actuator on steering link

private:

  void AddVisualizationSteeringLink();
};


} // end namespace chrono


#endif
