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
// Base class for a vehicle wheel.
// A wheel subsystem does not own a body. Instead, when attached to a suspension
// subsystem, the wheel's mass properties are used to update those of the
// spindle body owned by the suspension.
// A concrete wheel subsystem can optionally carry its own visualization assets
// (which are associated with the suspension's spindle body).
//
// =============================================================================

#include "subsys/ChWheel.h"


namespace chrono {


// The base class initialization function attaches this wheel to the specified
// suspension spindle body (by incrementing the spindle's mass and inertia with
// that of the wheel.  A derived class should always invoke this base method.
void ChWheel::Initialize(ChSharedBodyPtr spindle)
{
  spindle->SetMass(spindle->GetMass() + getMass());
  spindle->SetInertiaXX(spindle->GetInertiaXX() + getInertia());
}


}  // end namespace chrono
