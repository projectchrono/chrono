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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz
// =============================================================================
//
// Base class for a double-A arm suspension modeled with bodies and constraints.
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

#ifndef CH_DOUBLEWISHBONE_H
#define CH_DOUBLEWISHBONE_H

#include <vector>

#include "subsys/ChApiSubsys.h"
#include "subsys/ChSuspension.h"

namespace chrono {

///
/// Base class for a double-A arm suspension modeled with bodies and constraints.
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
class CH_SUBSYS_API ChDoubleWishbone : public ChSuspension
{
public:

  ChDoubleWishbone(
    const std::string& name               ///< [in] name of the subsystem
    );

  virtual ~ChDoubleWishbone();

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
    UCA_F,      ///< upper control arm, chassis front
    UCA_B,      ///< upper control arm, chassis back
    UCA_U,      ///< upper control arm, upright
    UCA_CM,     ///< upper control arm, center of mass
    LCA_F,      ///< lower control arm, chassis front
    LCA_B,      ///< lower control arm, chassis back
    LCA_U,      ///< lower control arm, upright
    LCA_CM,     ///< lower control arm, center of mass
    SHOCK_C,    ///< shock, chassis
    SHOCK_A,    ///< shock, lower control arm
    SPRING_C,   ///< spring, chassis
    SPRING_A,   ///< spring, lower control arm
    TIEROD_C,   ///< tierod, chassis
    TIEROD_U,   ///< tierod, upright
    NUM_POINTS
  };

  /// Return the location of the specified hardpoint.
  /// The returned location must be expressed in the suspension reference frame.
  virtual const ChVector<> getLocation(PointId which) = 0;

  /// Return the mass of the spindle body.
  virtual double getSpindleMass() const = 0;
  /// Return the mass of the upper control arm body.
  virtual double getUCAMass() const = 0;
  /// Return the mass of the lower control body.
  virtual double getLCAMass() const = 0;
  /// Return the mass of the upright body.
  virtual double getUprightMass() const = 0;

  /// Return the moments of inertia of the spindle body.
  virtual const ChVector<>& getSpindleInertia() const = 0;
  /// Return the moments of inertia of the upper control arm body.
  virtual const ChVector<>& getUCAInertia() const = 0;
  /// Return the moments of inertia of the lower control arm body.
  virtual const ChVector<>& getLCAInertia() const = 0;
  /// Return the moments of inertia of the upright body.
  virtual const ChVector<>& getUprightInertia() const = 0;

  /// Return the inertia of the axle shaft.
  virtual double getAxleInertia() const = 0;

  /// Return the radius of the spindle body (visualization only).
  virtual double getSpindleRadius() const = 0;
  /// Return the width of the spindle body (visualization only).
  virtual double getSpindleWidth() const = 0;
  /// Return the radius of the upper control arm body (visualization only).
  virtual double getUCARadius() const = 0;
  /// Return the radius of the lower control arm body (visualization only).
  virtual double getLCARadius() const = 0;
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
  ChSharedBodyPtr                   m_UCA[2];          ///< handles to the upper control arm bodies (left/right)
  ChSharedBodyPtr                   m_LCA[2];          ///< handles to the lower control arm bodies (left/right)

  ChSharedPtr<ChLinkLockRevolute>   m_revoluteUCA[2];  ///< handles to the chassis-UCA revolute joints (left/right)
  ChSharedPtr<ChLinkLockSpherical>  m_sphericalUCA[2]; ///< handles to the upright-UCA spherical joints (left/right)
  ChSharedPtr<ChLinkLockRevolute>   m_revoluteLCA[2];  ///< handles to the chassis-LCA revolute joints (left/right)
  ChSharedPtr<ChLinkLockSpherical>  m_sphericalLCA[2]; ///< handles to the upright-LCA spherical joints (left/right)
  ChSharedPtr<ChLinkDistance>       m_distTierod[2];   ///< handles to the tierod distance constraints (left/right)

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
                      const std::vector<ChVector<> >& points);

  static void AddVisualizationControlArm(ChSharedBodyPtr   arm,
                                         const ChVector<>  pt_F,
                                         const ChVector<>  pt_B,
                                         const ChVector<>  pt_U,
                                         double            radius);
  static void AddVisualizationUpright(ChSharedBodyPtr   upright,
                                      const ChVector<>  pt_C,
                                      const ChVector<>  pt_U,
                                      const ChVector<>  pt_L,
                                      const ChVector<>  pt_T,
                                      double            radius);
  static void AddVisualizationSpindle(ChSharedBodyPtr spindle,
                                      double          radius,
                                      double          width);

  static const std::string  m_pointNames[NUM_POINTS];
};


} // end namespace chrono


#endif
