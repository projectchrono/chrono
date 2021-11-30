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
// Material class for ANCF beam elements using the Enhanced Continuum Mechanics based method
//
// A description of the Enhanced Continuum Mechanics based method can be found in: K. Nachbagauer, P. Gruber, and J.
// Gerstmayr. Structural and Continuum Mechanics Approaches for a 3D Shear Deformable ANCF Beam Finite Element :
// Application to Static and Linearized Dynamic Examples.J.Comput.Nonlinear Dynam, 8 (2) : 021004, 2012.
// =============================================================================

#ifndef CHMATERIALBEAMANCF_H
#define CHMATERIALBEAMANCF_H

#include "chrono/fea/ChElementBeam.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Definition of materials to be used for ANCF beams utilizing the Enhanced Continuum Mechanics based method
class ChApi ChMaterialBeamANCF {
  public:
    /// Construct an isotropic material.
    ChMaterialBeamANCF(double rho,  ///< material density
                       double E,    ///< Young's modulus
                       double nu,   ///< Poisson ratio
                       double k1,   ///< Shear correction factor along beam local y axis
                       double k2    ///< Shear correction factor along beam local z axis
    );

    /// Construct a (possibly) orthotropic material.
    ChMaterialBeamANCF(double rho,            ///< material density
                       const ChVector<>& E,   ///< elasticity moduli (E_x, E_y, E_z)
                       const ChVector<>& nu,  ///< Poisson ratios (nu_xy, nu_xz, nu_yz)
                       const ChVector<>& G,   ///< shear moduli (G_xy, G_xz, G_yz)
                       double k1,             ///< Shear correction factor along beam local y axis
                       double k2              ///< Shear correction factor along beam local z axis
    );

    /// Return the material density.
    double Get_rho() const { return m_rho; }

    /// Complete Elasticity Tensor in 6x6 matrix form
    void Get_D(ChMatrixNM<double, 6, 6>& D);

    /// Diagonal components of the 6x6 elasticity matrix form without the terms contributing to the Poisson effect
    const ChVectorN<double, 6>& Get_D0() const { return m_D0; }

    /// Upper 3x3 block of the elasticity matrix with the terms contributing to the Poisson effect
    const ChMatrixNM<double, 3, 3>& Get_Dv() const { return m_Dv; }

    /// Return the matrix of elastic coefficients: Diagonal terms. (For compatibility with ChElementBeam only)
    const ChMatrixNM<double, 6, 6>& Get_E_eps() const { return m_E_eps; }

    /// Return the matrix of elastic coefficients: Coupling terms. (For compatibility with ChElementBeam only)
    const ChMatrixNM<double, 6, 6>& Get_E_eps_Nu() const { return m_E_eps_Nu; }

  private:
    /// Calculate the matrix form of two stiffness tensors used by the ANCF beam for selective reduced integration of
    /// the Poisson effect k1 and k2 are Timoshenko shear correction factors.
    void Calc_D0_Dv(const ChVector<>& E, const ChVector<>& nu, const ChVector<>& G, double k1, double k2);

    /// Calculate the matrix of elastic coefficients: k1 and k2 are Timoshenko shear correction factors. (For
    /// compatibility with ChElementBeam only)
    void Calc_E_eps(const ChVector<>& E, const ChVector<>& nu, const ChVector<>& G, double k1, double k2);

    /// Calculate the matrix of elastic coefficients. (For compatibility with ChElementBeam only)
    void Calc_E_eps_Nu(const ChVector<>& E, const ChVector<>& nu, const ChVector<>& G);

    double m_rho;               ///< density
    ChVectorN<double, 6> m_D0;  ///< Diagonal components of the 6x6 elasticity matrix form without the terms
                                ///< contributing to the Poisson effect
    ChMatrixNM<double, 3, 3>
        m_Dv;  ///< Upper 3x3 block of the elasticity matrix with the terms contributing to the Poisson effect
    ChMatrixNM<double, 6, 6> m_E_eps;  ///< matrix of elastic coefficients (For compatibility with ChElementBeam only)
    ChMatrixNM<double, 6, 6>
        m_E_eps_Nu;  ///< matrix of elastic coefficients (For compatibility with ChElementBeam only)

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @} fea_elements

}  // end of namespace fea
}  // end of namespace chrono

#endif
