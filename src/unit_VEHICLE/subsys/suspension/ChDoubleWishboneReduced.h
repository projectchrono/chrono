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
// with X pointing towards the rear, Y to the right, and Z up. The origin of
// the reference frame is assumed to be the center of th spindle body (i.e. the
// center of the wheel).
// By default, a right suspension is constructed.  This can be mirrored to
// obtain a left suspension.
// If marked as 'driven', the suspension subsystem also includes an engine link.
//
// =============================================================================

#ifndef CH_DOUBLEWISHBONEREDUCED_H
#define CH_DOUBLEWISHBONEREDUCED_H

#include "subsys/ChApiSubsys.h"
#include "subsys/ChSuspension.h"

namespace chrono {


class CH_SUBSYS_API ChDoubleWishboneReduced : public ChSuspension
{
public:

  ChDoubleWishboneReduced(const std::string& name,
                          ChSuspension::Side side,
                          bool               driven = false);
  virtual ~ChDoubleWishboneReduced() {}

  virtual void Initialize(ChSharedBodyPtr   chassis,
                          const ChVector<>& location);

  virtual void AttachWheel(ChSharedPtr<ChWheel> wheel);

  virtual void ApplySteering(double displ);
  virtual void ApplyTorque(double torque);

  virtual const ChVector<>& GetSpindlePos() const { return m_spindle->GetPos(); }
  virtual const ChQuaternion<>& GetSpindleRot() const { return m_spindle->GetRot(); }
  virtual double GetSpindleAngSpeed() const;

protected:

  enum PointId {
    SPINDLE,    // spindle location
    UPRIGHT,    // upright location
    UCA_F,      // upper control arm, chassis front
    UCA_B,      // upper control arm, chassis back
    UCA_U,      // upper control arm, upright
    LCA_F,      // lower control arm, chassis front
    LCA_B,      // lower control arm, chassis back
    LCA_U,      // lower control arm, upright
    SHOCK_C,    // shock, chassis
    SHOCK_U,    // shock, upright
    TIEROD_C,   // tierod, chassis
    TIEROD_U,   // tierod, upright
    NUM_POINTS
  };

  virtual const ChVector<> getLocation(PointId which) = 0;

  virtual double getSpindleMass() const = 0;
  virtual double getUprightMass() const = 0;

  virtual const ChVector<>& getSpindleInertia() const = 0;
  virtual const ChVector<>& getUprightInertia() const = 0;

  virtual double getSpringCoefficient() const = 0;
  virtual double getDampingCoefficient() const = 0;
  virtual double getSpringRestLength() const = 0;

  virtual void OnInitializeSpindle() {}
  virtual void OnInitializeUpright() {}

  ChVector<>        m_points[NUM_POINTS];

  ChSharedBodyPtr   m_spindle;
  ChSharedBodyPtr   m_upright;

  ChSharedPtr<ChLinkLockRevolute>   m_revolute;
  ChSharedPtr<ChLinkDistance>       m_distUCA_F;
  ChSharedPtr<ChLinkDistance>       m_distUCA_B;
  ChSharedPtr<ChLinkDistance>       m_distLCA_F;
  ChSharedPtr<ChLinkDistance>       m_distLCA_B;
  ChSharedPtr<ChLinkDistance>       m_distTierod;

  ChSharedPtr<ChLinkSpring>         m_shock;

  ChSharedPtr<ChLinkEngine>         m_engine;

  ChVector<>                        m_tierod_marker;

private:

};


} // end namespace chrono


#endif
