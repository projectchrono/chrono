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
// Base class for a double-A arm suspension modeled with distance constraints.
// Derived from ChSuspension, but still an abstract bas class.
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

#ifndef CH_DOUBLEWISHBONEREDUCED_H
#define CH_DOUBLEWISHBONEREDUCED_H

#include <vector>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSuspension.h"

namespace chrono {

///
/// Base class for a double-A arm suspension modeled with distance constraints.
/// Derived from ChSuspension, but still an abstract bas class.
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
class CH_VEHICLE_API ChDoubleWishboneReduced : public ChSuspension
{
public:

  ChDoubleWishboneReduced(
    const std::string& name               ///< [in] name of the subsystem
    );

  virtual ~ChDoubleWishboneReduced() {}

  /// Specify whether or not this suspension can be steered.
  virtual bool IsSteerable() const { return true; }

  /// Specify whether or not this is an independent suspension.
  virtual bool IsIndependent() const { return true; }

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

  /// Specify the left body for a possible antirollbar subsystem.
  /// Return a handle to the left upright.
  virtual ChSharedPtr<ChBody> GetLeftBody() const { return m_upright[0]; }

  /// Specify the right body for a possible antirollbar subsystem.
  /// Return a handle to the right upright.
  virtual ChSharedPtr<ChBody> GetRightBody() const { return m_upright[1]; }

  /// Log current constraint violations.
  virtual void LogConstraintViolations(ChVehicleSide side);

protected:

  /// Identifiers for the various hardpoints.
  enum PointId {
    SPINDLE,    ///< spindle location
    UPRIGHT,    ///< upright location
    UCA_F,      ///< upper control arm, chassis front
    UCA_B,      ///< upper control arm, chassis back
    UCA_U,      ///< upper control arm, upright
    LCA_F,      ///< lower control arm, chassis front
    LCA_B,      ///< lower control arm, chassis back
    LCA_U,      ///< lower control arm, upright
    SHOCK_C,    ///< shock, chassis
    SHOCK_U,    ///< shock, upright
    TIEROD_C,   ///< tierod, chassis
    TIEROD_U,   ///< tierod, upright
    NUM_POINTS
  };

  /// Return the location of the specified hardpoint.
  /// The returned location must be expressed in the suspension reference frame.
  virtual const ChVector<> getLocation(PointId which) = 0;

  /// Return the mass of the spindle body.
  virtual double getSpindleMass() const = 0;
  /// Return the mass of the upright body.
  virtual double getUprightMass() const = 0;

  /// Return the moments of inertia of the spindle body.
  virtual const ChVector<>& getSpindleInertia() const = 0;
  /// Return the moments of inertia of the upright body.
  virtual const ChVector<>& getUprightInertia() const = 0;

  /// Return the inertia of the axle shaft.
  virtual double getAxleInertia() const = 0;

  /// Return the radius of the spindle body (visualization only).
  virtual double getSpindleRadius() const = 0;
  /// Return the width of the spindle body (visualization only).
  virtual double getSpindleWidth() const = 0;
  /// Return the radius of the upright body (visualization only).
  virtual double getUprightRadius() const = 0;

  /// Return the free (rest) length of the spring-damper element.
  virtual double getSpringRestLength() const = 0;
  /// Return the callback function for shock force (spring-damper).
  virtual ChSpringForceCallback* getShockForceCallback()  const = 0;

  ChSharedBodyPtr                   m_upright[2];      ///< handles to the upright bodies (left/right)

  ChSharedPtr<ChLinkDistance>       m_distUCA_F[2];    ///< handles to the front UCA distance constraints (left/right)
  ChSharedPtr<ChLinkDistance>       m_distUCA_B[2];    ///< handles to the back UCA distance constraints (left/right)
  ChSharedPtr<ChLinkDistance>       m_distLCA_F[2];    ///< handles to the front LCA distance constraints (left/right)
  ChSharedPtr<ChLinkDistance>       m_distLCA_B[2];    ///< handles to the back LCA distance constraints (left/right)
  ChSharedPtr<ChLinkDistance>       m_distTierod[2];   ///< handles to the tierod distance constraints (left/right)

  ChSharedPtr<ChLinkSpringCB>       m_shock[2];        ///< handles to the spring-damper force elements (left/right)

private:

  void InitializeSide(ChVehicleSide                   side,
                      ChSharedPtr<ChBodyAuxRef>       chassis,
                      ChSharedPtr<ChBody>             tierod_body,
                      const std::vector<ChVector<> >& points);

  static void AddVisualizationUpright(ChSharedBodyPtr   upright,
                                      const ChVector<>  pt_C,
                                      const ChVector<>  pt_U,
                                      const ChVector<>  pt_L,
                                      const ChVector<>  pt_T,
                                      double            radius);
  static void AddVisualizationSpindle(ChSharedBodyPtr spindle,
                                      double          radius,
                                      double          width);
};


} // end namespace chrono


#endif
