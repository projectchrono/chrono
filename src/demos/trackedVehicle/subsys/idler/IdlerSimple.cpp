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

// Static variables
const double IdlerSimple::m_mass = 200.0;
const ChVector<> IdlerSimple::m_inertia = ChVector<>(10,10,15);

const double IdlerSimple::m_radius = 0.5;
const double IdlerSimple::m_width = 0.35;
const double IdlerSimple::m_springK = 100000;
const double IdlerSimple::m_springC = 1000;
const double IdlerSimple::m_springRestLength = 1.0;



IdlerSimple::IdlerSimple(const std::string& name, VisualizationType vis, CollisionType collide)
{
  m_idler = ChSharedPtr<ChBody>(new ChBody);

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
