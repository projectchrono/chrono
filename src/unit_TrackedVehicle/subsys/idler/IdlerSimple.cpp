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
// A Simple Idler system that keeps track chain tension by pre-loading a 
//  spring/damper elemnt
//
// =============================================================================

#include <cstdio>

#include "IdlerSimple.h"

namespace chrono {

IdlerSimple::IdlerSimple(double mass, const ChVector<>& inertia, double radius, double width, double K, double C)
: m_radius(radius), m_width(width), m_springK(K), m_springC(C)
{
  m_idler = ChSharedPtr<ChBody>(new ChBody);
  m_idler->SetMass(mass);
  m_idler->SetInertiaXX(inertia);
}

void IdlerSimple::Initialize(ChSharedPtr<ChBodyAuxRef> chassis,
                             const ChVector<>&         location,
                             const ChQuaternion<>&     rotation)
{
  // Express the steering reference frame in the absolute coordinate system.
  ChFrame<> idler_to_abs(location, rotation);
  idler_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

  
  
}

void IdlerSimple::AddVisualizationIdler()
{

}

} // end namespace chrono
