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
// Authors: Radu Serban, Holger Haut
// =============================================================================
//
// Base class for a Hendrickson PRIMAXX EX suspension.
// Derived from ChSuspension, but still an abstract base class.
//
// The suspension subsystem is modeled with respect to a right-handed frame,
// with X pointing towards the front, Y to the left, and Z up (ISO standard).
// The suspension reference frame is assumed to be always aligned with that of
// the vehicle.  When attached to a chassis, only an offset is provided.
//
// All point locations are assumed to be given for the left half of the
// supspension and will be mirrored (reflecting the y coordinates) to construct
// the right side.
//
// =============================================================================

#ifndef CH_HENDRICKSON_PRIMAXX_H
#define CH_HENDRICKSON_PRIMAXX_H

#include <vector>

#include "subsys/ChApiSubsys.h"
#include "subsys/ChSuspension.h"

namespace chrono {

///
/// Base class for a Hendrickson PRIMAXX EX suspension.
/// Derived from ChSuspension, but still an abstract base class.
///
/// The suspension subsystem is modeled with respect to a right-handed frame,
/// with X pointing towards the front, Y to the left, and Z up (ISO standard).
/// The suspension reference frame is assumed to be always aligned with that of
/// the vehicle.  When attached to a chassis, only an offset is provided.
///
/// All point locations are assumed to be given for the left half of the
/// supspension and will be mirrored (reflecting the y coordinates) to construct
/// the right side.
///
class CH_SUBSYS_API ChHendricksonPRIMAXX : public ChSuspension
{
public:

  ChHendricksonPRIMAXX(
    const std::string& name   ///< [in] name of the subsystem
    );

  virtual ~ChHendricksonPRIMAXX() {}

  /// Specify whether or not this suspension can be steered.
  virtual bool IsSteerable() const { return true; }

  /// Specify whether or not this is an independent suspension.
  virtual bool IsIndependent() const { return false; }

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
    );


  /// Log current constraint violations.
  virtual void LogConstraintViolations(ChVehicleSide side);

  /// Log the locations of all hardpoints.
  /// The reported locations are expressed in the suspension reference frame.
  /// By default, these values are reported in SI units (meters), but can be
  /// optionally reported in inches.
  void LogHardpointLocations(const ChVector<>& ref,
                             bool              inches = false);

protected:

  /// Identifiers for the various hardpoints.
  enum PointId {
    SPINDLE,    ///< spindle location
    NUM_POINTS
  };

  /// Return the location of the specified hardpoint.
  /// The returned location must be expressed in the suspension reference frame.
  virtual const ChVector<> getLocation(PointId which) = 0;

  /// Return the mass of the spindle body.
  virtual double getSpindleMass() const = 0;

  /// Return the moments of inertia of the spindle body.
  virtual const ChVector<>& getSpindleInertia() const = 0;

  /// Return the inertia of the axle shaft.
  virtual double getAxleInertia() const = 0;

  /// Return the radius of the spindle body (visualization only).
  virtual double getSpindleRadius() const = 0;
  /// Return the width of the spindle body (visualization only).
  virtual double getSpindleWidth() const = 0;


private:

  void InitializeSide(ChVehicleSide                   side,
                      ChSharedPtr<ChBodyAuxRef>       chassis,
                      ChSharedPtr<ChBody>             tierod_body,
                      const std::vector<ChVector<> >& points);

  static void AddVisualizationSpindle(ChSharedBodyPtr spindle,
                                      double          radius,
                                      double          width);

  static const std::string  m_pointNames[NUM_POINTS];
};


} // end namespace chrono


#endif
