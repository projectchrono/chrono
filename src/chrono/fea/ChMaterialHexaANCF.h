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
// Material for ANCF brick Element
// =============================================================================

#ifndef CHMATERIALBRICKANCF_H
#define CHMATERIALBRICKANCF_H

#include "chrono/fea/ChElementGeneric.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Definition of materials to be used for ANCF brick elements.
class ChApi ChMaterialHexaANCF {
  public:
    /// Construct an isotropic material.
    ChMaterialHexaANCF(double rho,  ///< material density
                       double E,    ///< Young's modulus
                       double nu    ///< Poisson ratio
    );

    /// Construct a (possibly) orthotropic material.
    ChMaterialHexaANCF(double rho,            ///< material density
                       const ChVector<>& E,   ///< elasticity moduli (E_x, E_y, E_z)
                       const ChVector<>& nu,  ///< Poisson ratios (nu_xy, nu_xz, nu_yz)
                       const ChVector<>& G    ///< shear moduli (G_xy, G_xz, G_yz)
    );

    /// Return the material density.
    double Get_rho() const { return m_rho; }

    const ChMatrixNM<double, 6, 6>& Get_D() const { return m_D; }

  private:
    /// Calculate the matrix form of 6x6 stiffness tensor
    void Calc_D(const ChVector<>& E, const ChVector<>& nu, const ChVector<>& G);

    double m_rho;                  ///< density
    ChMatrixNM<double, 6, 6> m_D;  ///< matrix of elastic coefficients

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// @} fea_elements

}  // end of namespace fea
}  // end of namespace chrono

#endif
