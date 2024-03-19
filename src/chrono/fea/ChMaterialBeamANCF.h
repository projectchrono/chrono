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
                       const ChVector3d& E,   ///< elasticity moduli (E_x, E_y, E_z)
                       const ChVector3d& nu,  ///< Poisson ratios (nu_xy, nu_xz, nu_yz)
                       const ChVector3d& G,   ///< shear moduli (G_xy, G_xz, G_yz)
                       double k1,             ///< Shear correction factor along beam local y axis
                       double k2              ///< Shear correction factor along beam local z axis
    );

    /// Return the material density.
    double GetDensity() const { return m_rho; }

  private:
    /// Upper 3x3 block of the elasticity matrix with the terms contributing to the Poisson effect
    const ChMatrixNM<double, 3, 3>& Get_Dv() const { return m_Dv; }

    /// Complete Elasticity Tensor in 6x6 matrix form
    void Get_D(ChMatrix66d& D);

    /// Diagonal components of the 6x6 elasticity matrix form without the terms contributing to the Poisson effect
    const ChVectorN<double, 6>& Get_D0() const { return m_D0; }

    /// Return the matrix of elastic coefficients: Coupling terms. (For compatibility with ChElementBeam only)
    const ChMatrix66d& Get_E_eps_Nu() const { return m_E_eps_Nu; }

    /// Return the matrix of elastic coefficients: Diagonal terms. (For compatibility with ChElementBeam only)
    const ChMatrix66d& Get_E_eps() const { return m_E_eps; }

    /// Calculate the matrix form of two stiffness tensors used by the ANCF beam for selective reduced integration of
    /// the Poisson effect k1 and k2 are Timoshenko shear correction factors.
    void Calc_D0_Dv(const ChVector3d& E, const ChVector3d& nu, const ChVector3d& G, double k1, double k2);

    /// Calculate the matrix of elastic coefficients: k1 and k2 are Timoshenko shear correction factors. (For
    /// compatibility with ChElementBeam only)
    void Calc_E_eps(const ChVector3d& E, const ChVector3d& nu, const ChVector3d& G, double k1, double k2);

    /// Calculate the matrix of elastic coefficients. (For compatibility with ChElementBeam only)
    void Calc_E_eps_Nu(const ChVector3d& E, const ChVector3d& nu, const ChVector3d& G);

    double m_rho;                   ///< density
    ChVectorN<double, 6> m_D0;      ///< Diagonal components of 6x6 elasticity matrix form without Poisson effect terms
    ChMatrixNM<double, 3, 3> m_Dv;  ///< Upper 3x3 block of the elasticity matrix with Poisson effect terms
    ChMatrix66d m_E_eps;            ///< matrix of elastic coefficients (For compatibility with ChElementBeam only)
    ChMatrix66d m_E_eps_Nu;         ///< matrix of elastic coefficients (For compatibility with ChElementBeam only)

    friend class ChElementBeamANCF_3243;
    friend class ChElementBeamANCF_3333;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @} fea_elements

}  // end of namespace fea
}  // end of namespace chrono

#endif
