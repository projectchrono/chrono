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

#include "assets/ChColorAsset.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSuspension.h"

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
class CH_VEHICLE_API ChHendricksonPRIMAXX : public ChSuspension
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

  /// TODO Holger
  /// There could be a spring (coil or air) and damper element between chassis and lower beam
  /// and a second spring and damper element between chassis and housing

  /// Spring (coil or air) and damper element between chassis and lower beam (LB)
  /// Get the force in the air spring (coil or spring) and a damper element

  /// Get the force in the spring element.
  double GetSpringLBForce(ChVehicleSide side) const { return m_springLB[side]->Get_SpringReact(); }

  /// Get the current length of the spring element
  double GetSpringLBLength(ChVehicleSide side) const { return m_springLB[side]->Get_SpringLength(); }

  /// Get the current deformation of the spring element.
  double GetSpringLBDeformation(ChVehicleSide side) const { return m_springLB[side]->Get_SpringDeform(); }

  /// Get the force in the shock (damper) element.
  double GetShockLBForce(ChVehicleSide side) const { return m_shockLB[side]->Get_SpringReact(); }

  /// Get the current length of the shock (damper) element.
  double GetShockLBLength(ChVehicleSide side) const { return m_shockLB[side]->Get_SpringLength(); }

  /// Get the current deformation velocity of the shock (damper) element.
  double GetShockLBVelocity(ChVehicleSide side) const { return m_shockLB[side]->Get_SpringVelocity(); }


  /// Spring (coil or air) and damper element between chassis and axle housing (AH)
  /// Get the force in the air spring (coil or spring) and a damper element

  /// Get the force in the spring element.
  double GetSpringAHForce(ChVehicleSide side) const { return m_springAH[side]->Get_SpringReact(); }

  /// Get the current length of the spring element
  double GetSpringAHLength(ChVehicleSide side) const { return m_springAH[side]->Get_SpringLength(); }

  /// Get the current deformation of the spring element.
  double GetSpringAHDeformation(ChVehicleSide side) const { return m_springAH[side]->Get_SpringDeform(); }

  /// Get the force in the shock (damper) element.
  double GetShockAHForce(ChVehicleSide side) const { return m_shockAH[side]->Get_SpringReact(); }

  /// Get the current length of the shock (damper) element.
  double GetShockAHLength(ChVehicleSide side) const { return m_shockAH[side]->Get_SpringLength(); }

  /// Get the current deformation velocity of the shock (damper) element.
  double GetShockAHVelocity(ChVehicleSide side) const { return m_shockAH[side]->Get_SpringVelocity(); }

  /// End of TODO Holger

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
    SPINDLE,           ///< spindle location
    KNUCKLE_L,         ///< lower knuckle point
    KNUCKLE_U,         ///< upper knuckle point
    TIEROD_C,          ///< tierod, chassis
    TIEROD_K,          ///< tierod, knuckle
    TORQUEROD_C,       ///< torquerod, chassis
    TORQUEROD_AH,      ///< torquerod, axle housing (AH)
    LOWERBEAM_C,       ///< lowerbeam, chassis
    LOWERBEAM_AH,      ///< lowerbeam, axle housing (AH)
    LOWERBEAM_TB,      ///< lowerbeam, transverse beam 
    SHOCKAH_C,         ///< shock at axle housing (AH), chasis
    SHOCKAH_AH,        ///< shock at axle housing (AH), axle housing
    SPRINGAH_C,        ///< spring at axle housing (AH), chasis
    SPRINGAH_AH,       ///< spring at axle housing (AH), axle housing
    SHOCKLB_C,         ///< shock at lower beam (LB), chasis
    SHOCKLB_LB,        ///< shock at lower beam (LB), lower beam
    SPRINGLB_C,        ///< spring at lower beam (LB), chasis
    SPRINGLB_LB,       ///< spring at lower beam (LB), lower beam
    KNUCKLE_CM,        ///< knuckle, center of mass
    TORQUEROD_CM,      ///< torquerod, center of mass
    LOWERBEAM_CM,      ///< lowerbeam, center of mass
    TRANSVERSEBEAM_CM, ///< transverse beam, center of mass
    NUM_POINTS
  };

  /// Identifiers for the various vectors.
  enum DirectionId {
    UNIV_AXIS_LOWERBEAM_BEAM,     ///< universal joint (lowerbeam, beam side)
    UNIV_AXIS_LOWERBEAM_CHASSIS,  ///< universal joint (lowerbeam, chassis side)
    UNIV_AXIS_TORQUEROD_ROD,      ///< universal joint (torquerod, rod side)
    UNIV_AXIS_TORQUEROD_CHASSIS,  ///< universal joint (torquerod, chasis side)
    NUM_DIRS
  };

  /// Return the location of the specified hardpoint.
  /// The returned location must be expressed in the suspension reference frame.
  virtual const ChVector<> getLocation(PointId which) = 0;

  /// HH Question in CHsolidAxle.h there is the follwing vector for direction
  ///// Return the vector of the specified direction.
  virtual const ChVector<> getDirection(DirectionId which) = 0;
  /// HH Question ends

  /// Return the center of mass of the axle tube.
  virtual const ChVector<> getAxlehousingCOM() const = 0;

  /// Return the mass of the spindle body.
  /// HH QUESTION WHERE IS THE HARDPOINT DEF OF THE SPINDELS MASS???
  virtual double getSpindleMass() const = 0;
  ///HH END QUESTION
  /// Return the mass of the knuckle body.
  virtual double getKnuckleMass() const = 0;
  /// Return the mass of the torque rod body.
  virtual double getTorquerodMass() const = 0;
  /// Return the mass of the lower beam body.
  virtual double getLowerbeamMass() const = 0;
  /// Return the mass of the transverse beam body.
  virtual double getTransversebeamMass() const = 0;
  /// Return the mass of the axlehousing body.
  virtual double getAxlehousingMass() const = 0;

  /// Return the moments of inertia of the spindle body.
  virtual const ChVector<>& getSpindleInertia() const = 0;
  /// Return the moments of inertia of the knuckle body.
  virtual const ChVector<>& getKnuckleInertia() const = 0;
  /// Return the moments of inertia of the torque rod body.
  virtual const ChVector<>& getTorquerodInertia() const = 0;
  /// Return the moments of inertia of the lower beam body.
  virtual const ChVector<>& getLowerbeamInertia() const = 0;
  /// Return the moments of inertia of the transverse beam body.
  virtual const ChVector<>& getTransversebeamInertia() const = 0;
  /// Return the moments of inertia of the axlehousing body.
  virtual const ChVector<>& getAxlehousingInertia() const = 0;

  /// Return the inertia of the axle shaft.
  virtual double getAxleInertia() const = 0;

  /// Return the radius of the spindle body (visualization only).
  virtual double getSpindleRadius() const = 0;
  /// Return the width of the spindle body (visualization only).
  virtual double getSpindleWidth() const = 0;
  /// Return the radius of the knuckle body (visualization only).
  virtual double getKnuckleRadius() const = 0;

  /// Return the radius of the torque rod body (visualization only).
  virtual double getTorquerodRadius() const = 0;
  /// Return the radius of the lower beam body (visualization only).
  virtual double getLowerbeamRadius() const = 0;
  /// Return the radius of the transverse beam body (visualization only).
  virtual double getTransversebeamRadius() const = 0;
  /// Return the radius of the axlehousing body (visualization only).
  virtual double getAxlehousingRadius() const = 0;

  /// Lower beam spring and damper
  /// Return the free (rest) length of the spring element.
  virtual double getSpringLBRestLength() const = 0;
  /// Return the callback function for spring force.
  virtual ChSpringForceCallback* getSpringLBForceCallback() const = 0;
  /// Return the callback function for shock force.
  virtual ChSpringForceCallback* getShockLBForceCallback()  const = 0;

  /// Axlehousing spring and damper
  /// Return the free (rest) length of the spring element.
  virtual double getSpringAHRestLength() const = 0;
  /// Return the callback function for spring force.
  virtual ChSpringForceCallback* getSpringAHForceCallback() const = 0;
  /// Return the callback function for shock force.
  virtual ChSpringForceCallback* getShockAHForceCallback()  const = 0;

  ChSharedBodyPtr                   m_knuckle[2];               ///< handles to the knuckle bodies (left/right)
  ChSharedBodyPtr                   m_torquerod[2];             ///< handles to torquerod bodies (left/right)
  ChSharedBodyPtr                   m_lowerbeam[2];             ///< handles to lowerbeam bodies (left/right)
  ChSharedBodyPtr                   m_transversebeam;           ///< handles to transversebeam body
  ChSharedBodyPtr                   m_axlehousing;              ///< handles to axlehousing body

  ChSharedPtr<ChLinkLockRevolute>   m_revoluteKingpin[2];       ///< handles to the knuckle-axle housing revolute joints (left/right)
  /// HH OK?
  ChSharedPtr<ChLinkLockSpherical>  m_sphericalTorquerod[2];    ///< handles to the torque rod-axle housing spherical joints (left/right)
  ChSharedPtr<ChLinkUniversal>      m_universalTorquerod[2];    ///< handles to the torque rod-chassis universal joints (left/right)
  ChSharedPtr<ChLinkLockSpherical>  m_sphericalLowerbeam[2];    ///< handles to the lower beam-axle housing spherical joints (left/right)
  ChSharedPtr<ChLinkUniversal>      m_universalLowerbeam[2];    ///< handles to the lower beam-chassis universal joints (left/right)
  /// think about constraints
  ChSharedPtr<ChLinkLockSpherical>  m_sphericalTB[2];           ///< handles to the transversebeam-lower beam spherical joints (left/right)
  /// end of thinking about transverse beam constraints
  ChSharedPtr<ChLinkDistance>       m_distTierod[2];            ///< handles to the tierod distance constraints (left/right)

  ChSharedPtr<ChLinkSpringCB>       m_shockLB[2];                 ///< handles to the spring links (left/right)
  ChSharedPtr<ChLinkSpringCB>       m_springLB[2];                ///< handles to the shock links (left/right)
  ChSharedPtr<ChLinkSpringCB>       m_shockAH[2];                 ///< handles to the spring links (left/right)
  ChSharedPtr<ChLinkSpringCB>       m_springAH[2];                ///< handles to the shock links (left/right)
    /// HH OK? END

private:

  void InitializeSide(ChVehicleSide                   side,
                                     ChSharedPtr<ChBodyAuxRef>       chassis,
                                     ChSharedPtr<ChBody>             tierod_body,
                                     const std::vector<ChVector<> >& points,
                                     const std::vector<ChVector<> >& dirs);

  static void AddVisualizationLink(ChSharedBodyPtr   body,
                                      const ChVector<>  pt_1,
                                      const ChVector<>  pt_2,
                                      double            radius,
                                      const ChColor&    color);
    static void AddVisualizationKnuckle(ChSharedBodyPtr   knuckle,
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
