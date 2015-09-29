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
// HMMWV Fiala subsystem
//
// =============================================================================

#include "hmmwv/tire/HMMWV_FialaTire.h"

using namespace chrono;

namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double HMMWV_FialaTire::m_normalStiffness = 2e6;
const double HMMWV_FialaTire::m_normalDamping = 1e3;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_FialaTire::HMMWV_FialaTire(const std::string& name) : ChFialaTire(name) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_FialaTire::SetFialaParams() {
    m_unloaded_radius = 0.3099;
    m_width = 0.235;
    m_rolling_resistance = 0.000;
    m_c_slip = 1000000.0;
    m_c_alpha = 45836.6236;
    m_u_min = 0.9;
    m_u_max = 1.0;
    m_relax_length_x = 0.00005;
    m_relax_length_y = 0.00015;
}

}  // end namespace hmmwv
