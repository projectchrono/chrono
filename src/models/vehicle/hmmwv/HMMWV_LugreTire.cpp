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

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"

#include "models/vehicle/hmmwv/HMMWV_LugreTire.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;

const double HMMWV_LugreTire::m_radius = 18.15 * in2m;
const double HMMWV_LugreTire::m_discLocs[] = {-5 * in2m, 0 * in2m, 5 * in2m};

const double HMMWV_LugreTire::m_normalStiffness = 2e6;
const double HMMWV_LugreTire::m_normalDamping = 1e3;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_LugreTire::HMMWV_LugreTire(const std::string& name) : ChLugreTire(name) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_LugreTire::SetLugreParams() {
    // Longitudinal direction
    m_sigma0[0] = 181;
    m_sigma1[0] = 1;
    m_sigma2[0] = 0.02;

    m_Fc[0] = 0.6;
    m_Fs[0] = 1.0;

    m_vs[0] = 3.5;

    // Lateral direction
    m_sigma0[1] = 60;
    m_sigma1[1] = 0.1;
    m_sigma2[1] = 0.002;

    m_Fc[1] = 0.6;
    m_Fs[1] = 1.0;

    m_vs[1] = 3.5;
}

}  // end namespace hmmwv
