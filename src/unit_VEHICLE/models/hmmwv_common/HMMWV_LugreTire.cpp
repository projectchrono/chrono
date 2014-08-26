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
// HMMWV LuGre subsystem
//
// =============================================================================


#include "HMMWV_LugreTire.h"

using namespace chrono;

namespace hmmwv9 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;

const double HMMWV9_LugreTire::m_radius = 18.5 * in2m;
const double HMMWV9_LugreTire::m_discLocs[] = { -5 * in2m, 0 * in2m, 5 * in2m };


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV9_LugreTire::HMMWV9_LugreTire(const ChTerrain& terrain)
: ChLugreTire(terrain)
{
}


} // end namespace hmmwv9
