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
// Authors: Michael Taylor, Antonio Recuero, Radu Serban
// =============================================================================

#include "chrono/fea/ChMaterialHexaANCF.h"

namespace chrono {
namespace fea {

// Construct an isotropic material.
ChMaterialHexaANCF::ChMaterialHexaANCF(double rho,  // material density
                                       double E,    // Young's modulus
                                       double nu    // Poisson ratio
                                       )
    : m_rho(rho) {
    double G = 0.5 * E / (1 + nu);
    Calc_D(ChVector<>(E), ChVector<>(nu), ChVector<>(G));
}

// Construct a (possibly) orthotropic material.
ChMaterialHexaANCF::ChMaterialHexaANCF(double rho,            // material density
                                       const ChVector<>& E,   // elasticity moduli (E_x, E_y, E_z)
                                       const ChVector<>& nu,  // Poisson ratios (nu_xy, nu_xz, nu_yz)
                                       const ChVector<>& G    // shear moduli (G_xy, G_xz, G_yz)
                                       )
    : m_rho(rho) {
    Calc_D(E, nu, G);
}

// Calculate the 6x6 matrix form of stiffness tensors used by the ANCF element
void ChMaterialHexaANCF::Calc_D(const ChVector<>& E, const ChVector<>& nu, const ChVector<>& G) {
    // orthotropic material ref: http://homes.civil.aau.dk/lda/Continuum/material.pdf
    // except position of the shear terms is different to match the original ANCF reference paper

    double nu_12 = nu.x();
    double nu_13 = nu.y();
    double nu_23 = nu.z();
    double nu_21 = nu_12 * E.y() / E.x();
    double nu_31 = nu_13 * E.z() / E.x();
    double nu_32 = nu_23 * E.z() / E.y();
    double k = 1.0 - nu_23 * nu_32 - nu_12 * nu_21 - nu_13 * nu_31 - nu_12 * nu_23 * nu_31 - nu_21 * nu_32 * nu_13;

    ChMatrixNM<double, 6, 6> D;
    D.setZero();
    D(0, 0) = E.x() * (1 - nu_23 * nu_32) / k;
    D(1, 0) = E.y() * (nu_13 * nu_32 + nu_12) / k;
    D(2, 0) = E.z() * (nu_12 * nu_23 + nu_13) / k;

    D(0, 1) = E.x() * (nu_23 * nu_31 + nu_21) / k;
    D(1, 1) = E.y() * (1 - nu_13 * nu_31) / k;
    D(2, 1) = E.z() * (nu_13 * nu_21 + nu_23) / k;

    D(0, 2) = E.x() * (nu_21 * nu_32 + nu_31) / k;
    D(1, 2) = E.y() * (nu_12 * nu_31 + nu_32) / k;
    D(2, 2) = E.z() * (1 - nu_12 * nu_21) / k;

    D(3, 3) = G.z();
    D(4, 4) = G.y();
    D(5, 5) = G.x();

    m_D = D;
}

}  // end of namespace fea
}  // end of namespace chrono
