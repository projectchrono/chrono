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


class CH_SUBSYS_API ChSolidAxle : public ChSuspension
{
public:

  ChSolidAxle(const std::string& name,
              bool               driven = false);
  virtual ~ChSolidAxle() {}

  virtual bool IsSteerable() const { return true; }

  virtual void Initialize(ChSharedPtr<ChBodyAuxRef>  chassis,
                          const ChVector<>&          location,
                          ChSharedPtr<ChBody>        tierod_body);

  double GetSpringForce(ChSuspension::Side side)       const { return m_spring[side]->Get_SpringReact(); }
  double GetSpringLength(ChSuspension::Side side)      const { return m_spring[side]->Get_SpringLength(); }
  double GetSpringDeformation(ChSuspension::Side side) const { return m_spring[side]->Get_SpringDeform(); }

  double GetShockForce(ChSuspension::Side side)        const { return m_shock[side]->Get_SpringReact(); }
  double GetShockLength(ChSuspension::Side side)       const { return m_shock[side]->Get_SpringLength(); }
  double GetShockVelocity(ChSuspension::Side side)     const { return m_shock[side]->Get_SpringVelocity(); }

  virtual void LogConstraintViolations(ChSuspension::Side side);

  void LogHardpointLocations(const ChVector<>& ref,
                             bool              inches = false);

protected:

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

  enum DirectionId {
    UNIV_AXIS_LINK_L,     ///< universal joint (lower link, link side)
    UNIV_AXIS_CHASSIS_L,  ///< universal joint (lower link, chassis side)
    UNIV_AXIS_LINK_U,     ///< universal joint (upper link, link side)
    UNIV_AXIS_CHASSIS_U,  ///< universal joint (upper link, chassis side)
    NUM_DIRS
  };

  virtual const ChVector<> getLocation(PointId which) = 0;
  virtual const ChVector<> getDirection(DirectionId which) = 0;

  virtual const ChVector<> getAxleTubeCOM() const = 0;

  virtual double getAxleTubeMass() const = 0;
  virtual double getSpindleMass() const = 0;
  virtual double getULMass() const = 0;
  virtual double getLLMass() const = 0;
  virtual double getKnuckleMass() const = 0;

  virtual double getAxleTubeRadius() const = 0;
  virtual double getSpindleRadius() const = 0;
  virtual double getSpindleWidth() const = 0;
  virtual double getULRadius() const = 0;
  virtual double getLLRadius() const = 0;
  virtual double getKnuckleRadius() const = 0;

  virtual const ChVector<>& getAxleTubeInertia() const = 0;
  virtual const ChVector<>& getSpindleInertia() const = 0;
  virtual const ChVector<>& getULInertia() const = 0;
  virtual const ChVector<>& getLLInertia() const = 0;
  virtual const ChVector<>& getKnuckleInertia() const = 0;

  virtual double getAxleInertia() const = 0;

  virtual double getSpringCoefficient() const = 0;
  virtual double getDampingCoefficient() const = 0;
  virtual double getSpringRestLength() const = 0;


  ChSharedBodyPtr                   m_axleTube;
  ChSharedBodyPtr                   m_knuckle[2];
  ChSharedBodyPtr                   m_upperLink[2];
  ChSharedBodyPtr                   m_lowerLink[2];

  ChSharedPtr<ChLinkLockRevolute>   m_revoluteKingpin[2];
  ChSharedPtr<ChLinkLockSpherical>  m_sphericalUpperLink[2];
  ChSharedPtr<ChLinkLockSpherical>  m_sphericalLowerLink[2];
  ChSharedPtr<ChLinkLockUniversal>  m_universalUpperLink[2];
  ChSharedPtr<ChLinkLockUniversal>  m_universalLowerLink[2];
  ChSharedPtr<ChLinkDistance>       m_distTierod[2];

  ChSharedPtr<ChLinkSpring>         m_shock[2];
  ChSharedPtr<ChLinkSpring>         m_spring[2];

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
