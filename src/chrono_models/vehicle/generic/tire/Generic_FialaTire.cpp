// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Michael Taylor
// =============================================================================
//
// Generic Fiala tire subsystem
// Parameters are based on the MSC ADAMS/tire 2015.1 help document
//
// =============================================================================

#include <cmath>
#include "chrono_models/vehicle/generic/tire/Generic_FialaTire.h"

namespace chrono {
namespace vehicle {
namespace generic {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double Generic_FialaTire::m_normalStiffness = 310000;
const double Generic_FialaTire::m_normalDamping = 3100;

const double Generic_FialaTire::m_mass = 35.0;
const ChVector<> Generic_FialaTire::m_inertia(3.0, 6.0, 3.0);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Generic_FialaTire::Generic_FialaTire(const std::string& name) : ChFialaTire(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Generic_FialaTire::SetFialaParams() {
    m_unloaded_radius = 0.3099;
    m_width = 0.235;
    m_rolling_resistance = 0.001;
    m_c_slip = 1000000;
    m_c_alpha = 45836.6236;
    m_u_min = 0.9;
    m_u_max = 1.0;
    m_relax_length_x = 0.05;
    m_relax_length_y = 0.15;
}

double Generic_FialaTire::GetNormalStiffnessForce(double depth) const {
    depth = depth * (depth > 0);  // Ensure that depth is positive;

    return (m_normalStiffness * depth);
}

double Generic_FialaTire::GetNormalDampingForce(double depth, double velocity) const {
    return m_normalDamping * velocity;
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
