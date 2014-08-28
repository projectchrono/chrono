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
// with X pointing towards the rear, Y to the right, and Z up. By default, a
// right suspension is constructed.  This can be mirrored to obtain a left
// suspension. Note that this is done by reflecting the y coordinates of the
// hardpoints. As such, the orientation of the suspension reference frame must
// be as specified above. However, its location relative to the chassis is
// arbitrary (and left up to a derived class).
//
// If marked as 'driven', the suspension subsystem also creates the ChShaft axle
// element and its connection to the spindle body (which provides the interface
// to the powertrain subsystem).
//
// =============================================================================

#ifndef CH_DOUBLEWISHBONE_H
#define CH_DOUBLEWISHBONE_H

#include "subsys/ChApiSubsys.h"
#include "subsys/ChSuspension.h"

namespace chrono {


class CH_SUBSYS_API ChDoubleWishbone : public ChSuspension
{
public:

  ChDoubleWishbone(const std::string& name,
                          ChSuspension::Side side,
                          bool               driven = false);
  virtual ~ChDoubleWishbone() {}

  virtual void Initialize(ChSharedBodyPtr   chassis,
                          const ChVector<>& location);

  virtual void ApplySteering(double displ);

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
    SHOCK_U,    // shock, upright
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

  virtual void OnInitializeSpindle()  {}
  virtual void OnInitializeUCA()  {}
  virtual void OnInitializeLCA()  {}
  virtual void OnInitializeUpright()  {}

  ChVector<>        m_points[NUM_POINTS];

  ChSharedBodyPtr   m_upright;

  ChSharedPtr<ChLinkLockRevolute>   m_revolute;
  ChSharedBodyPtr                   m_UCA;
  ChSharedBodyPtr                   m_LCA;
  ChSharedPtr<ChLinkLockRevolute>   m_revoluteUCA;
  ChSharedPtr<ChLinkLockSpherical>  m_sphericalUCA;
  ChSharedPtr<ChLinkLockRevolute>   m_revoluteLCA;
  ChSharedPtr<ChLinkLockSpherical>  m_sphericalLCA;
  ChSharedPtr<ChLinkDistance>       m_distTierod;

  ChSharedPtr<ChLinkSpring>         m_shock;

  ChVector<>                        m_tierod_marker;

private:

  void   AddVisualizationUCA();
  void   AddVisualizationLCA();
  void   AddVisualizationUpright();
  void   AddVisualizationSpindle();
};


} // end namespace chrono


#endif
