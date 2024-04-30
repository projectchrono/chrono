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
// Material for ANCF shells
// =============================================================================

#ifndef CHMATERIALSHELLANCF_H
#define CHMATERIALSHELLANCF_H

#include "chrono/fea/ChElementShell.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Definition of materials to be used for ANCF shells.
/// This class implements material properties for a layer.
class ChApi ChMaterialShellANCF {
  public:
    /// Construct an isotropic material.
    ChMaterialShellANCF(double rho,  ///< material density
                        double E,    ///< Young's modulus
                        double nu    ///< Poisson ratio
    );

    /// Construct a (possibly) orthotropic material.
    ChMaterialShellANCF(double rho,            ///< material density
                        const ChVector3d& E,   ///< elasticity moduli (E_x, E_y, E_z)
                        const ChVector3d& nu,  ///< Poisson ratios (nu_xy, nu_xz, nu_yz)
                        const ChVector3d& G    ///< shear moduli (G_xy, G_xz, G_yz)
    );

    /// Return the material density.
    double GetDensity() const { return m_rho; }

  private:
    /// Return the matrix of elastic coefficients.
    const ChMatrix66d& Get_E_eps() const { return m_E_eps; }

    /// Calculate the matrix of elastic coefficients.
    void Calc_E_eps(const ChVector3d& E, const ChVector3d& nu, const ChVector3d& G);

    double m_rho;         ///< density
    ChMatrix66d m_E_eps;  ///< matrix of elastic coefficients

    friend class ChElementShellANCF_3833;
    friend class ChElementShellANCF_3443;
    friend class ChElementShellANCF_3423;
    friend class ShellANCF_Force;
    friend class ShellANCF_Jacobian;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @} fea_elements

}  // end of namespace fea
}  // end of namespace chrono

#endif
