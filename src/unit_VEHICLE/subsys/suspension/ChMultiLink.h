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
// Base class for a multi-link suspension modeled with bodies and constraints.
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

#ifndef CH_MULTILINK_H
#define CH_MULTILINK_H

#include <vector>

#include "subsys/ChApiSubsys.h"
#include "subsys/ChSuspension.h"

namespace chrono {

///
/// Base class for a multi-link suspension modeled with bodies and constraints.
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
class CH_SUBSYS_API ChMultiLink : public ChSuspension
{
public:

  ChMultiLink(
    const std::string& name               ///< [in] name of the subsystem
    );

  virtual ~ChMultiLink();

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
  virtual void Initialize(
    ChSharedPtr<ChBodyAuxRef>  chassis,     ///< [in] handle to the chassis body
    const ChVector<>&          location,    ///< [in] location relative to the chassis frame
    ChSharedPtr<ChBody>        tierod_body  ///< [in] body to which tireods are connected
    );

  /// Get the force in the spring element.
  double GetSpringForce(ChVehicleSide side) const { return m_spring[side]->Get_SpringReact(); }

  /// Get the current length of the spring element
  double GetSpringLength(ChVehicleSide side) const { return m_spring[side]->Get_SpringLength(); }

  /// Get the current deformation of the spring element.
  double GetSpringDeformation(ChVehicleSide side) const { return m_spring[side]->Get_SpringDeform(); }

  /// Get the force in the shock (damper) element.
  double GetShockForce(ChVehicleSide side) const { return m_shock[side]->Get_SpringReact(); }

  /// Get the current length of the shock (damper) element.
  double GetShockLength(ChVehicleSide side) const { return m_shock[side]->Get_SpringLength(); }

  /// Get the current deformation velocity of the shock (damper) element.
  double GetShockVelocity(ChVehicleSide side) const { return m_shock[side]->Get_SpringVelocity(); }

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
    UPRIGHT,    ///< upright location
    UA_F,       ///< upper arm, chassis front
    UA_B,       ///< upper arm, chassis back
    UA_U,       ///< upper arm, upright
    UA_CM,      ///< upper arm, center of mass
    LAT_C,      ///< lateral, chassis
    LAT_U,      ///< lateral, upright
    LAT_CM,     ///< lateral, center of mass
    TL_C,       ///< trailing link, chassis
    TL_U,       ///< trailing link, upright
    TL_CM,      ///< trailing link, center of mass
    SHOCK_C,    ///< shock, chassis
    SHOCK_L,    ///< shock, trailing link
    SPRING_C,   ///< spring, chassis
    SPRING_L,   ///< spring, trailing link
    TIEROD_C,   ///< tierod, chassis
    TIEROD_U,   ///< tierod, upright
    NUM_POINTS
  };

  /// Identifiers for the various vectors.
  enum DirectionId {
    UNIV_AXIS_LINK_TL,     ///< universal joint (trailing link, link side)
    UNIV_AXIS_CHASSIS_TL,  ///< universal joint (trailing link, chassis side)
    UNIV_AXIS_LINK_LAT,    ///< universal joint (lateral, link side)
    UNIV_AXIS_CHASSIS_LAT, ///< universal joint (lateral, chassis side)
    NUM_DIRS
  };

  /// Return the location of the specified hardpoint.
  /// The returned location must be expressed in the suspension reference frame.
  virtual const ChVector<> getLocation(PointId which) = 0;

  /// Return the vector of the specified direction.
  virtual const ChVector<> getDirection(DirectionId which) = 0;

  /// Return the mass of the spindle body.
  virtual double getSpindleMass() const = 0;
  /// Return the mass of the upper arm body.
  virtual double getUpperArmMass() const = 0;
  /// Return the mass of the lateral body.
  virtual double getLateralMass() const = 0;
  /// Return the mass of the trailing link body.
  virtual double getTrailingLinkMass() const = 0;
  /// Return the mass of the upright body.
  virtual double getUprightMass() const = 0;

  /// Return the moments of inertia of the spindle body.
  virtual const ChVector<>& getSpindleInertia() const = 0;
  /// Return the moments of inertia of the upper arm body.
  virtual const ChVector<>& getUpperArmInertia() const = 0;
  /// Return the moments of inertia of the lateral body.
  virtual const ChVector<>& getLateralInertia() const = 0;
  /// Return the moments of inertia of the trailing link body.
  virtual const ChVector<>& getTrailingLinkInertia() const = 0;
  /// Return the moments of inertia of the upright body.
  virtual const ChVector<>& getUprightInertia() const = 0;

  /// Return the inertia of the axle shaft.
  virtual double getAxleInertia() const = 0;

  /// Return the radius of the spindle body (visualization only).
  virtual double getSpindleRadius() const = 0;
  /// Return the width of the spindle body (visualization only).
  virtual double getSpindleWidth() const = 0;
  /// Return the radius of the upper arm body (visualization only).
  virtual double getUpperArmRadius() const = 0;
  /// Return the radius of the lateral body (visualization only).
  virtual double getLateralRadius() const = 0;
  /// Return the radius of the trailing link body (visualization only).
  virtual double getTrailingLinkRadius() const = 0;
  /// Return the radius of the upright body (visualization only).
  virtual double getUprightRadius() const = 0;

  /// Indicate whether the spring is modeled as a nonlinear element.
  /// If true, the concrete class must provide a callback function to calculate
  /// the force in the spring element (see getSpringForceCallback).
  virtual bool useNonlinearSpring() const { return false; }
  /// Indicate whether the shock is modeled as a nonlinear element.
  /// If true, the concrete class must provide a callback function to calculate
  /// the force in the shock element (see getShockForceCallback).
  virtual bool useNonlinearShock() const  { return false; }

  /// Return the spring coefficient (for linear spring elements).
  virtual double getSpringCoefficient() const  { return 1.0; }
  /// Return the damping coefficient (for linear shock elements).
  virtual double getDampingCoefficient() const { return 1.0; }

  /// Return the free (rest) length of the spring element.
  virtual double getSpringRestLength() const = 0;

  /// Return the callback function for spring force (for nonlinear spring).
  virtual ChSpringForceCallback* getSpringForceCallback() const { return NULL; }
  /// Return the callback function for shock force (for nonlinear shock).
  virtual ChSpringForceCallback* getShockForceCallback()  const { return NULL; }

  ChSharedBodyPtr                   m_upright[2];      ///< handles to the upright bodies (left/right)
  ChSharedBodyPtr                   m_upperArm[2];     ///< handles to the upper arm bodies (left/right)
  ChSharedBodyPtr                   m_lateral[2];      ///< handles to the lateral bodies (left/right)
  ChSharedBodyPtr                   m_trailingLink[2]; ///< handles to the trailing link bodies (left/right)

  ChSharedPtr<ChLinkLockRevolute>   m_revoluteUA[2];              ///< handles to the chassis-UA revolute joints (left/right)
  ChSharedPtr<ChLinkLockSpherical>  m_sphericalUA[2];             ///< handles to the upright-UA spherical joints (left/right)
  ChSharedPtr<ChLinkLockUniversal>  m_universalLateralChassis[2]; ///< handles to the chassis-lateral universal joints (left/right)
  ChSharedPtr<ChLinkLockSpherical>  m_sphericalLateralUpright[2]; ///< handles to the upright-lateral spherical joints (left/right)
  ChSharedPtr<ChLinkLockUniversal>  m_universalTLChassis[2];      ///< handles to the chassis-trailing link universal joints (left/right)
  ChSharedPtr<ChLinkLockSpherical>  m_sphericalTLUpright[2];      ///< handles to the upright-trailing link spherical joints (left/right)
  ChSharedPtr<ChLinkDistance>       m_distTierod[2];              ///< handles to the tierod distance constraints (left/right)

  bool                              m_nonlinearShock;  ///< true if using a callback function to calculate shock forces.
  bool                              m_nonlinearSpring; ///< true if using a callback function to calculate spring forces.

  ChSpringForceCallback*            m_shockCB;         ///< callback function for calculating shock forces
  ChSpringForceCallback*            m_springCB;        ///< callback function for calculating spring forces

  ChSharedPtr<ChLinkSpringCB>       m_shock[2];        ///< handles to the spring links (left/right)
  ChSharedPtr<ChLinkSpringCB>       m_spring[2];       ///< handles to the shock links (left/right)

private:

  void CreateSide(ChVehicleSide      side,
                  const std::string& suffix);
  void InitializeSide(ChVehicleSide                   side,
                      ChSharedPtr<ChBodyAuxRef>       chassis,
                      ChSharedPtr<ChBody>             tierod_body,
                      const std::vector<ChVector<> >& points,
                      const std::vector<ChVector<> >& dirs);

  static void AddVisualizationUpperArm(ChSharedBodyPtr    arm,
                                         const ChVector<>&  pt_F,
                                         const ChVector<>&  pt_B,
                                         const ChVector<>&  pt_U,
                                         double             radius);
  static void AddVisualizationLateral(ChSharedBodyPtr    rod,
                                         const ChVector<>&  pt_C,
                                         const ChVector<>&  pt_U,
                                         double             radius);
  static void AddVisualizationTrailingLink(ChSharedBodyPtr    link,
                                         const ChVector<>&  pt_C,
                                         const ChVector<>&  pt_S,
                                         const ChVector<>&  pt_U,
                                         double             radius);
  static void AddVisualizationUpright(ChSharedBodyPtr    upright,
                                         const ChVector<>&  pt_UA,
                                         const ChVector<>&  pt_TR,
                                         const ChVector<>&  pt_TL,
                                         const ChVector<>&  pt_T,
                                         const ChVector<>&  pt_U,
                                         double             radius);
  static void AddVisualizationSpindle(ChSharedBodyPtr spindle,
                                         double          radius,
                                         double          width);

  static const std::string  m_pointNames[NUM_POINTS];
};


} // end namespace chrono


#endif
