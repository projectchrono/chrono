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
// with X pointing towards the rear, Y to the right, and Z up. All point
// locations are assumed to be given for the right half of the supspension and
// will be mirrored (reflecting the y coordinates) to construct the left side.
//
// If marked as 'driven', the suspension subsystem also creates the ChShaft axle
// element and its connection to the spindle body (which provides the interface
// to the driveline subsystem).
//
// =============================================================================

#ifndef CH_DOUBLEWISHBONE_H
#define CH_DOUBLEWISHBONE_H

#include <vector>

#include "subsys/ChApiSubsys.h"
#include "subsys/ChSuspension.h"

namespace chrono {


class CH_SUBSYS_API ChDoubleWishbone : public ChSuspension
{
public:

  ChDoubleWishbone(const std::string& name,
                   bool               steerable = false,
                   bool               driven = false);
  virtual ~ChDoubleWishbone() {}

  virtual void Initialize(ChSharedPtr<ChBodyAuxRef>  chassis,
                          const ChVector<>&          location);

  virtual void ApplySteering(double displ);

  double GetSpringForce(ChSuspension::Side side)       const { return m_spring[side]->Get_SpringReact(); }
  double GetSpringLength(ChSuspension::Side side)      const { return m_spring[side]->Get_SpringLength(); }
  double GetSpringDeformation(ChSuspension::Side side) const { return m_spring[side]->Get_SpringDeform(); }

  double GetShockForce(ChSuspension::Side side)        const { return m_shock[side]->Get_SpringReact(); }
  double GetShockLength(ChSuspension::Side side)       const { return m_shock[side]->Get_SpringLength(); }
  double GetShockVelocity(ChSuspension::Side side)     const { return m_shock[side]->Get_SpringVelocity(); }

  void LogHardpointLocations(const ChVector<>& ref,
                             bool              inches = false);
  void LogConstraintViolations(ChSuspension::Side side);

protected:

  enum PointId {
    SPINDLE,    // spindle location
    UPRIGHT,    // upright location
    UCA_F,      // upper control arm, chassis front
    UCA_B,      // upper control arm, chassis back
    UCA_U,      // upper control arm, upright
    UCA_CM,     // upper control arm, center of mass
    LCA_F,      // lower control arm, chassis front
    LCA_B,      // lower control arm, chassis back
    LCA_U,      // lower control arm, upright
    LCA_CM,     // lower control arm, center of mass
    SHOCK_C,    // shock, chassis
    SHOCK_A,    // shock, lower control arm
    SPRING_C,   // spring, chassis
    SPRING_A,   // spring, lower control arm
    TIEROD_C,   // tierod, chassis
    TIEROD_U,   // tierod, upright
    NUM_POINTS
  };

  virtual const ChVector<> getLocation(PointId which) = 0;

  virtual double getSpindleMass() const = 0;
  virtual double getUCAMass() const = 0;
  virtual double getLCAMass() const = 0;
  virtual double getUprightMass() const = 0;

  virtual double getSpindleRadius() const = 0;
  virtual double getSpindleWidth() const = 0;
  virtual double getUCARadius() const = 0;
  virtual double getLCARadius() const = 0;
  virtual double getUprightRadius() const = 0;

  virtual const ChVector<>& getSpindleInertia() const = 0;
  virtual const ChVector<>& getUCAInertia() const = 0;
  virtual const ChVector<>& getLCAInertia() const = 0;
  virtual const ChVector<>& getUprightInertia() const = 0;

  virtual double getAxleInertia() const = 0;

  virtual double getSpringCoefficient() const = 0;
  virtual double getDampingCoefficient() const = 0;
  virtual double getSpringRestLength() const = 0;


  ChSharedBodyPtr                   m_upright[2];
  ChSharedBodyPtr                   m_UCA[2];
  ChSharedBodyPtr                   m_LCA[2];

  ChSharedPtr<ChLinkLockRevolute>   m_revoluteUCA[2];
  ChSharedPtr<ChLinkLockSpherical>  m_sphericalUCA[2];
  ChSharedPtr<ChLinkLockRevolute>   m_revoluteLCA[2];
  ChSharedPtr<ChLinkLockSpherical>  m_sphericalLCA[2];
  ChSharedPtr<ChLinkDistance>       m_distTierod[2];

  ChSharedPtr<ChLinkSpring>         m_shock[2];
  ChSharedPtr<ChLinkSpring>         m_spring[2];

  ChVector<>                        m_tierod_marker[2];

private:

  void CreateSide(ChSuspension::Side side,
                  const std::string& suffix);
  void InitializeSide(ChSuspension::Side              side,
                      ChSharedPtr<ChBodyAuxRef>       chassis,
                      const std::vector<ChVector<> >& points);

  static void AddVisualizationControlArm(ChSharedBodyPtr    arm,
                                         const ChVector<>&  pt_F,
                                         const ChVector<>&  pt_B,
                                         const ChVector<>&  pt_U,
                                         double             radius);
  static void AddVisualizationUpright(ChSharedBodyPtr    upright,
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
