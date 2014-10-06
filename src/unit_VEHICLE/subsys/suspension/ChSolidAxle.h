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
// Authors: Daniel Melanz, Radu Serban
// =============================================================================
//
// Base class for a solid axle suspension modeled with bodies and constraints.
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
// If marked as 'driven', the suspension subsystem also creates the ChShaft axle
// element and its connection to the spindle body (which provides the interface
// to the driveline subsystem).
//
// =============================================================================

#ifndef CH_SOLIDAXLE_H
#define CH_SOLIDAXLE_H

#include <vector>

#include "assets/ChColorAsset.h"

#include "subsys/ChApiSubsys.h"
#include "subsys/ChSuspension.h"

namespace chrono {

///
/// Base class for a solid axle suspension modeled with bodies and constraints.
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
/// If marked as 'driven', the suspension subsystem also creates the ChShaft axle
/// element and its connection to the spindle body (which provides the interface
///
class CH_SUBSYS_API ChSolidAxle : public ChSuspension
{
public:

  ChSolidAxle(const std::string& name,               ///< [in] name of the subsystem
              bool               driven = false      ///< [in] true if attached to driveline subsystem
              );

  virtual ~ChSolidAxle() {}

  /// Specify whether or not this suspension can be steered.
  virtual bool IsSteerable() const { return true; }

  /// Initialize this suspension subsystem.
  /// The suspension subsystem is initialized by attaching it to the specified
  /// chassis body at the specified location (with respect to and expressed in
  /// the reference frame of the chassis). It is assumed that the suspension
  /// reference frame is always aligned with the chassis reference frame.
  /// Finally, tierod_body is a handle to the body to which the suspension
  /// tierods are to be attached. For a steerable suspension, this will be the
  /// steering link of a suspension subsystem.  Otherwise, this is the chassis.
  virtual void Initialize(ChSharedPtr<ChBodyAuxRef>  chassis,     ///< [in] handle to the chassis body
                          const ChVector<>&          location,    ///< [in] location relative to the chassis frame
                          ChSharedPtr<ChBody>        tierod_body  ///< [in] body to which tireods are connected
                          );

  /// Get the force in the spring element.
  double GetSpringForce(ChSuspension::Side side)       const { return m_spring[side]->Get_SpringReact(); }

  /// Get the current length of the spring element
  double GetSpringLength(ChSuspension::Side side)      const { return m_spring[side]->Get_SpringLength(); }

  /// Get the current deformation of the spring element.
  double GetSpringDeformation(ChSuspension::Side side) const { return m_spring[side]->Get_SpringDeform(); }

  /// Get the force in the shock (damper) element.
  double GetShockForce(ChSuspension::Side side)        const { return m_shock[side]->Get_SpringReact(); }

  /// Get the current length of the shock (damper) element.
  double GetShockLength(ChSuspension::Side side)       const { return m_shock[side]->Get_SpringLength(); }

  /// Get the current deformation velocity of the shock (damper) element.
  double GetShockVelocity(ChSuspension::Side side)     const { return m_shock[side]->Get_SpringVelocity(); }

  /// Log current constraint violations.
  virtual void LogConstraintViolations(ChSuspension::Side side);

  void LogHardpointLocations(const ChVector<>& ref,
                             bool              inches = false);

protected:

  /// Identifiers for the various hardpoints.
  enum PointId {
    SHOCK_A,    ///< shock, axle
    SHOCK_C,    ///< shock, chassis
    KNUCKLE_L,  ///< lower knuckle point
    KNUCKLE_U,  ///< upper knuckle point
    LL_A,       ///< lower link, axle
    LL_C,       ///< lower link, chassis
    UL_A,       ///< upper link, axle
    UL_C,       ///< upper link, chassis
    SPRING_A,   ///< spring, axle
    SPRING_C,   ///< spring, chassis
    TIEROD_C,   ///< tierod, chassis
    TIEROD_K,   ///< tierod, knuckle
    SPINDLE,    ///< spindle location
    KNUCKLE_CM, ///< knuckle, center of mass
    LL_CM,      ///< lower link, center of mass
    UL_CM,      ///< upper link, center of mass
    NUM_POINTS
  };

  /// Identifiers for the various vectors.
  enum DirectionId {
    UNIV_AXIS_LINK_L,     ///< universal joint (lower link, link side)
    UNIV_AXIS_CHASSIS_L,  ///< universal joint (lower link, chassis side)
    UNIV_AXIS_LINK_U,     ///< universal joint (upper link, link side)
    UNIV_AXIS_CHASSIS_U,  ///< universal joint (upper link, chassis side)
    NUM_DIRS
  };

  /// Return the location of the specified hardpoint.
  /// The returned location must be expressed in the suspension reference frame.
  virtual const ChVector<> getLocation(PointId which) = 0;

  /// Return the vector of the specified direction.
  virtual const ChVector<> getDirection(DirectionId which) = 0;

  /// Return the center of mass of the axle tube.
  virtual const ChVector<> getAxleTubeCOM() const = 0;

  /// Return the mass of the axle tube body.
  virtual double getAxleTubeMass() const = 0;
  /// Return the mass of the spindle body.
  virtual double getSpindleMass() const = 0;
  /// Return the mass of the upper link body.
  virtual double getULMass() const = 0;
  /// Return the mass of the lower link body.
  virtual double getLLMass() const = 0;
  /// Return the mass of the knuckle body.
  virtual double getKnuckleMass() const = 0;

  /// Return the radius of the axle tube body (visualization only).
  virtual double getAxleTubeRadius() const = 0;
  /// Return the radius of the spindle body (visualization only).
  virtual double getSpindleRadius() const = 0;
  /// Return the width of the spindle body (visualization only).
  virtual double getSpindleWidth() const = 0;
  /// Return the radius of the upper link body (visualization only).
  virtual double getULRadius() const = 0;
  /// Return the radius of the lower link body (visualization only).
  virtual double getLLRadius() const = 0;
  /// Return the radius of the knuckle body (visualization only).
  virtual double getKnuckleRadius() const = 0;

  /// Return the moments of inertia of the axle tube body.
  virtual const ChVector<>& getAxleTubeInertia() const = 0;
  /// Return the moments of inertia of the spindle body.
  virtual const ChVector<>& getSpindleInertia() const = 0;
  /// Return the moments of inertia of the upper link body.
  virtual const ChVector<>& getULInertia() const = 0;
  /// Return the moments of inertia of the lower link body.
  virtual const ChVector<>& getLLInertia() const = 0;
  /// Return the moments of inertia of the knuckle body.
  virtual const ChVector<>& getKnuckleInertia() const = 0;

  /// Return the inertia of the axle shaft.
  virtual double getAxleInertia() const = 0;

  /// Return the spring coefficient (for linear spring elements).
  virtual double getSpringCoefficient() const = 0;
  /// Return the damping coefficient (for linear shock elements).
  virtual double getDampingCoefficient() const = 0;

  /// Return the free (rest) length of the spring element.
  virtual double getSpringRestLength() const = 0;


  ChSharedBodyPtr                   m_axleTube;                 ///< handles to the axle tube body
  ChSharedBodyPtr                   m_knuckle[2];               ///< handles to the knuckle bodies (left/right)
  ChSharedBodyPtr                   m_upperLink[2];             ///< handles to the upper link bodies (left/right)
  ChSharedBodyPtr                   m_lowerLink[2];             ///< handles to the lower link bodies (left/right)

  ChSharedPtr<ChLinkLockRevolute>   m_revoluteKingpin[2];       ///< handles to the knuckle-axle tube revolute joints (left/right)
  ChSharedPtr<ChLinkLockSpherical>  m_sphericalUpperLink[2];    ///< handles to the upper link-axle tube spherical joints (left/right)
  ChSharedPtr<ChLinkLockSpherical>  m_sphericalLowerLink[2];    ///< handles to the lower link-axle tube spherical joints (left/right)
  ChSharedPtr<ChLinkLockUniversal>  m_universalUpperLink[2];    ///< handles to the upper link-chassis universal joints (left/right)
  ChSharedPtr<ChLinkLockUniversal>  m_universalLowerLink[2];    ///< handles to the lower link-chassis universal joints (left/right)
  ChSharedPtr<ChLinkDistance>       m_distTierod[2];            ///< handles to the tierod distance constraints (left/right)

  ChSharedPtr<ChLinkSpring>         m_shock[2];                 ///< handles to the spring links (left/right)
  ChSharedPtr<ChLinkSpring>         m_spring[2];                ///< handles to the shock links (left/right)

private:

  void CreateSide(ChSuspension::Side side,
                  const std::string& suffix);
  void InitializeSide(ChSuspension::Side              side,
                      ChSharedPtr<ChBodyAuxRef>       chassis,
                      ChSharedPtr<ChBody>             tierod_body,
                      const std::vector<ChVector<> >& points,
                      const std::vector<ChVector<> >& dirs);

  static void AddVisualizationLink(ChSharedBodyPtr    body,
                                   const ChVector<>&  pt_1,
                                   const ChVector<>&  pt_2,
                                   double             radius,
                                   const ChColor&     color);
  static void AddVisualizationKnuckle(ChSharedBodyPtr    knuckle,
                                      const ChVector<>&  pt_U,
                                      const ChVector<>&  pt_L,
                                      const ChVector<>&  pt_T,
                                      double             radius);
  static void AddVisualizationSpindle(ChSharedBodyPtr spindle,
                                      double          radius,
                                      double          width);

  static const std::string  m_pointNames[NUM_POINTS];
};


} // end namespace chrono


#endif
