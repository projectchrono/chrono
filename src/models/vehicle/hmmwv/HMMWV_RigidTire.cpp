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
// Authors: Radu Serban
// =============================================================================
//
// HMMWV wheel subsystem
//
// =============================================================================

#include "models/vehicle/hmmwv/HMMWV_RigidTire.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;

const double HMMWV_RigidTire::m_radius = 18.5 * in2m;
const double HMMWV_RigidTire::m_width = 10 * in2m;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_RigidTire::HMMWV_RigidTire(const std::string& name) : ChRigidTire(name) {
    SetContactMaterial(0.9f, 0.1f, 2e7f, 0.3f);
}

}  // end namespace hmmwv
