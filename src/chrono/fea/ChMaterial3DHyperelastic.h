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
// Authors: Alessandro Tasora 
// =============================================================================

#ifndef CHMATERIAL3DHYPERELASTIC_H
#define CHMATERIAL3DHYPERELASTIC_H

#include "chrono/fea/ChMaterial3DStress.h"


namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{



/// Class for hyperelastic materials, that is materials with a constitutive equation 
/// of the type  P = f(C) , where P is the 2nd Piola-Kirchhoff strain and 
/// C is the right Cauchy-Green deformation tensor C=F^T*F  (in material space)
/// as in E = 1/2(C -I) where E the Green Lagrange strain tensor. 
/// Or in general, materials for which the stress–strain 
/// relationship derives from a strain energy density function.

class ChMaterial3DHyperelastic : public ChMaterial3DStress {
public:
    ChMaterial3DHyperelastic() {}

    virtual ~ChMaterial3DHyperelastic() {}

    /// Implement interface to lower level stress material
    virtual void ComputeStress(ChStressTensor<>& S_stress,          ///< output stress, PK2
                                const ChMatrix33d& F,               ///< current deformation gradient tensor
                                const ChMatrix33d* l,               ///< current spatial velocity gradient (might be nullptr if IsSpatialVelocityGradientNeeded() is false)
                                ChFieldData* data_per_point,///< pointer to auxiliary data (ex states), if any, per quadrature point
                                ChElementData* data_per_element     ///< pointer to auxiliary data (ex states), if any, per element 
    ) override {
  
        // right Cauchy-Green deformation tensor C = F^T*F
        ChMatrix33d C_deformation = F.transpose() * F;

        // Use the hyperelastic stress computation
        ComputeElasticStress(S_stress, C_deformation);
    };

    /// Implement interface to lower level stress material
    virtual void ComputeTangentModulus(ChMatrixNM<double, 6, 6>& StressStrainMatrix, ///< output C tangent modulus, dP=C*dE
                                        const ChMatrix33d& F,               ///< current deformation gradient tensor
                                        const ChMatrix33d* l,               ///< current spatial velocity gradient (might be nullptr if IsSpatialVelocityGradientNeeded() is false)
                                        ChFieldData* data_per_point,        ///< pointer to auxiliary data (ex states), if any, per quadrature point
                                        ChElementData* data_per_element     ///< pointer to auxiliary data (ex states), if any, per element 
    ) override {

        // right Cauchy-Green deformation tensor C = F^T*F
        ChMatrix33d C_deformation = F.transpose() * F;

        // Use the hyperelastic tangent modulus computation
        ComputeElasticTangentModulus(StressStrainMatrix, C_deformation);
    };


    /// Compute elastic stress from the right Cauchy-Green deformation tensor C = F^T*F. 
    /// Return stress as Piola-Kirchhoff S tensor, in Voigt notation. 
    /// Children materials MUST implement this.
    virtual void ComputeElasticStress(ChStressTensor<>& S_stress, const ChMatrix33d& C_deformation) = 0;

    /// Computes the tangent modulus for a given deformation.
    /// Children materials MUST implement this.
    virtual void ComputeElasticTangentModulus(ChMatrixNM<double, 6, 6>& StressStrainMatrix, const ChMatrix33d& C_deformation) = 0;
};




/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
