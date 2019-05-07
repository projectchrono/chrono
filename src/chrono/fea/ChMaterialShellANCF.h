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
                        const ChVector<>& E,   ///< elasticity moduli (E_x, E_y, E_z)
                        const ChVector<>& nu,  ///< Poisson ratios (nu_xy, nu_xz, nu_yz)
                        const ChVector<>& G    ///< shear moduli (G_xy, G_xz, G_yz)
    );

    /// Return the material density.
    double Get_rho() const { return m_rho; }

    /// Return the matrix of elastic coefficients.
    const ChMatrixNM<double, 6, 6>& Get_E_eps() const { return m_E_eps; }

  private:
    /// Calculate the matrix of elastic coefficients.
    void Calc_E_eps(const ChVector<>& E, const ChVector<>& nu, const ChVector<>& G);

    double m_rho;                      ///< density
    ChMatrixNM<double, 6, 6> m_E_eps;  ///< matrix of elastic coefficients
};

/// @} fea_elements

}  // end of namespace fea
}  // end of namespace chrono

#endif
