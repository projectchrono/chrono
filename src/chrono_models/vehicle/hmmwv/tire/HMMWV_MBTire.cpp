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
// HMMWV multibody tire subsystem
//
// =============================================================================

#include "chrono/core/ChCubicSpline.h"
#include "chrono_models/vehicle/hmmwv/tire/HMMWV_MBTire.h"

using namespace chrono::fea;

namespace chrono {
namespace vehicle {
namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const int HMMWV_MBTire::m_num_divs = 40;
const std::vector<double> HMMWV_MBTire::m_radius = {0.388, 0.465, 0.467, 0.465, 0.388};
const std::vector<double> HMMWV_MBTire::m_offset = {-0.15, -0.1, 0, +0.1, +0.15};

const double HMMWV_MBTire::m_rim_radius = 0.268;

const double HMMWV_MBTire::m_tire_mass = 37.6;
const double HMMWV_MBTire::m_default_pressure = 200e3;

const float HMMWV_MBTire::m_friction = 0.9f;
const float HMMWV_MBTire::m_restitution = 0.1f;
const float HMMWV_MBTire::m_Young = 2.0e8f;
const float HMMWV_MBTire::m_Poisson = 0.3f;
const float HMMWV_MBTire::m_kn = 2.0e6f;
const float HMMWV_MBTire::m_gn = 1.3e2f;
const float HMMWV_MBTire::m_kt = 1.0e6f;
const float HMMWV_MBTire::m_gt = 0;

// -----------------------------------------------------------------------------

HMMWV_MBTire::HMMWV_MBTire(const std::string& name) : ChMBTire(name) {
    SetTireMass(m_tire_mass);

    SetTireGeometry(m_radius, m_offset, m_num_divs, m_rim_radius);

    // Distributed spring coefficients (independent of the mesh resolution)
    double kR = 3.8e5;   // radial spring elastic coefficient
    double cR = 4.7e3;   // radial spring damping coefficient
    double kC = 1.2e5;   // circumferential spring elastic coefficient
    double cC = 7.3e2;   // circumferential spring damping coefficient
    double kT = 2.2e5;   // transversal spring elastic coefficient
    double cT = 1.4e3;   // transversal spring damping coefficient
    double kBC = 1.2e3;  // circumferential bending spring elastic coefficient
    double cBC = 0;      // circumferential bending spring damping coefficient
    double kBT = 2.2e3;  // transversal bending spring elastic coefficient
    double cBT = 0;      // transversal bending spring damping coefficient
    SetSpringCoefficientType(SpringCoefficientType::DISTRIBUTED);
    SetRadialSpringCoefficients(kR, cR);
    SetMeshSpringCoefficients(kC, cC, kT, cT);
    SetBendingSpringCoefficients(kBC, cBC, kBT, cBT);

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
    SetContactReferenceDivisions(m_num_divs);
}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
