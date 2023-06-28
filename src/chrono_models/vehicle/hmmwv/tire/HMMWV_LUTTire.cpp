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
// Authors: Radu Serban
// =============================================================================
//
// HMMWV LUT tire subsystem
//
// =============================================================================

#include "chrono/core/ChCubicSpline.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_LUTTire.h"

using namespace chrono::fea;

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const int HMMWV_LUTTire::m_num_divs = 40;
const std::vector<double> HMMWV_LUTTire::m_radius = {0.2683, 0.4183, 0.4673, 0.4183, 0.2683};
const std::vector<double> HMMWV_LUTTire::m_offset = {-0.115, -0.0575, 0, +0.0575, +0.115};

const double HMMWV_LUTTire::m_tire_mass = 37.6;
const double HMMWV_LUTTire::m_default_pressure = 200e3;

const float HMMWV_LUTTire::m_friction = 0.9f;
const float HMMWV_LUTTire::m_restitution = 0.1f;
const float HMMWV_LUTTire::m_Young = 2.0e6f;
const float HMMWV_LUTTire::m_Poisson = 0.3f;
const float HMMWV_LUTTire::m_kn = 2.0e6f;
const float HMMWV_LUTTire::m_gn = 1.3e1f;
const float HMMWV_LUTTire::m_kt = 1.0e6f;
const float HMMWV_LUTTire::m_gt = 0;

// -----------------------------------------------------------------------------

HMMWV_LUTTire::HMMWV_LUTTire(const std::string& name)
    : ChLUTTire(name) {
    SetTireGeometry(m_radius, m_offset, m_num_divs);
    SetTireMass(m_tire_mass);
    ChContactMaterialData mat;
    mat.mu = m_friction;
    mat.cr = m_restitution;
    mat.Y = m_Young;
    mat.nu = m_Poisson;
    mat.kn = m_kn;
    mat.gn = m_gn;
    mat.kt = m_kt;
    mat.gt = m_gt;
    SetTireContactMaterial(mat);
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
