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
// The drive gear propels the tracked vehicle
//
// =============================================================================

#include "DriveGear.h"

namespace chrono {


DriveGear::DriveGear(double mass, const ChVector<>& inertia, double rad, double width)
: m_radius(rad), m_width(width)
{
  m_gear = ChSharedPtr<ChBody>(new ChBody);
  m_gear->SetMass(mass);
  m_gear->SetInertiaXX(inertia);
}


} // end namespace chrono
