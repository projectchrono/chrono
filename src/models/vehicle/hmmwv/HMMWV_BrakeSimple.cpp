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
// Authors: Alessandro Tasora
// =============================================================================
//
// HMMWV simple brake model.
//
// =============================================================================

#include "models/vehicle/hmmwv/HMMWV_BrakeSimple.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double HMMWV_BrakeSimple::m_maxtorque = 4000;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_BrakeSimple::HMMWV_BrakeSimple() {
}

}  // end namespace hmmwv
