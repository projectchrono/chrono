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
// Authors: Justin Madsen
// =============================================================================
//
// A simple Idler system that keeps track chain tension by pre-loading a 
//	spring/damper element
//
// =============================================================================

#ifndef IDLERSIMPLE_H
#define IDLERSIMPLE_H

#include "subsys/ChApiSubsys.h"
#include "physics/ChSystem.h"
#include "physics/ChBodyAuxRef.h"


namespace chrono {


/// An idler system that includes the chain tensioning system.
/// The simplest case of this is a prismatic (translational) constraint
/// between the idler and chassis, and mount a pre-loaded spring-damper
/// along the DOF axis
class CH_SUBSYS_API IdlerSimple : public ChShared
{
public:

  IdlerSimple();

  ~IdlerSimple() {}

  void Initialize(ChSharedPtr<ChBodyAuxRef> chassis,
                  const ChVector<>&         location,
                  const ChQuaternion<>&     rotation);
  
  
  double getSpringCoefficient() const { return m_springK; }
  double getDampingCoefficient() const { return m_springC; }
  double getSpringRestLength() const { return m_springRestLength; }
  ChSharedPtr<ChBody> GetBody() { return m_idler; }

private:
  // private functions
  void AddVisualizationIdler();
  
  // private variables
  ChSharedPtr<ChBody> m_idler;

  // static variables
  static const double m_mass;
  static const ChVector<> m_inertia;
  static const double m_springK;
  static const double m_springC;
  static const double m_springRestLength;
  static const double m_width;
  static const double m_radius;

};


} // end namespace chrono


#endif
