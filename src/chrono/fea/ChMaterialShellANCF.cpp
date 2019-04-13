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
// Authors: Bryan Peterson, Milad Rakhsha, Antonio Recuero, Radu Serban
// =============================================================================

#include "chrono/fea/ChMaterialShellANCF.h"

namespace chrono {
namespace fea {

// Construct an isotropic material.
ChMaterialShellANCF::ChMaterialShellANCF(double rho,  // material density
                                         double E,    // Young's modulus
                                         double nu    // Poisson ratio
                                         )
    : m_rho(rho) {
    double G = 0.5 * E / (1 + nu);
    Calc_E_eps(ChVector<>(E), ChVector<>(nu), ChVector<>(G));
}

// Construct a (possibly) orthotropic material.
ChMaterialShellANCF::ChMaterialShellANCF(double rho,            // material density
                                         const ChVector<>& E,   // elasticity moduli (E_x, E_y, E_z)
                                         const ChVector<>& nu,  // Poisson ratios (nu_xy, nu_xz, nu_yz)
                                         const ChVector<>& G    // shear moduli (G_xy, G_xz, G_yz)
                                         )
    : m_rho(rho) {
    Calc_E_eps(E, nu, G);
}

// Calculate the matrix of elastic coefficients.
// Always assume that the material could be orthotropic
void ChMaterialShellANCF::Calc_E_eps(const ChVector<>& E, const ChVector<>& nu, const ChVector<>& G) {
    double delta = 1.0 - (nu.x() * nu.x()) * E.y() / E.x() - (nu.y() * nu.y()) * E.z() / E.x() -
                   (nu.z() * nu.z()) * E.z() / E.y() - 2.0 * nu.x() * nu.y() * nu.z() * E.z() / E.x();
    double nu_yx = nu.x() * E.y() / E.x();
    double nu_zx = nu.y() * E.z() / E.x();
    double nu_zy = nu.z() * E.z() / E.y();
    m_E_eps.Reset();
    m_E_eps(0, 0) = E.x() * (1.0 - (nu.z() * nu.z()) * E.z() / E.y()) / delta;
    m_E_eps(1, 1) = E.y() * (1.0 - (nu.y() * nu.y()) * E.z() / E.x()) / delta;
    m_E_eps(3, 3) = E.z() * (1.0 - (nu.x() * nu.x()) * E.y() / E.x()) / delta;
    m_E_eps(0, 1) = E.y() * (nu.x() + nu.y() * nu.z() * E.z() / E.y()) / delta;
    m_E_eps(0, 3) = E.z() * (nu.y() + nu.z() * nu.x()) / delta;
    m_E_eps(1, 0) = E.y() * (nu.x() + nu.y() * nu.z() * E.z() / E.y()) / delta;
    m_E_eps(1, 3) = E.z() * (nu.z() + nu.y() * nu.x() * E.y() / E.x()) / delta;
    m_E_eps(3, 0) = E.z() * (nu.y() + nu.z() * nu.x()) / delta;
    m_E_eps(3, 1) = E.z() * (nu.z() + nu.y() * nu.x() * E.y() / E.x()) / delta;
    m_E_eps(2, 2) = G.x();
    m_E_eps(4, 4) = G.y();
    m_E_eps(5, 5) = G.z();
}

}  // end of namespace fea
}  // end of namespace chrono
