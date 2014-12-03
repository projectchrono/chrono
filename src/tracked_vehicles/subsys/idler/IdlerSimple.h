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

namespace chrono {


class CH_SUBSYS_API IdlerSimple
{
public:

  IdlerSimple(double mass, const ChVector<>& inertia, double radius, double width, double K, double C);

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
  double m_springK;
  double m_springC;
  double m_springRestLength;
  double m_width;
  double m_radius;

};


} // end namespace chrono


#endif
