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

#include "chrono/fea/multiphysics/ChMaterial3DStress.h"


namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{



/// Class for hyperelastic materials, that is materials with a constitutive equation 
/// of the type  S = f(C) , where S is the 2nd Piola-Kirchhoff stress and 
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
    virtual void ComputeTangentModulus(ChMatrixNM<double, 6, 6>& C,         ///< output C tangent modulus, dP=C*dE
                                        const ChMatrix33d& F,               ///< current deformation gradient tensor
                                        const ChMatrix33d* l,               ///< current spatial velocity gradient (might be nullptr if IsSpatialVelocityGradientNeeded() is false)
                                        ChFieldData* data_per_point,        ///< pointer to auxiliary data (ex states), if any, per quadrature point
                                        ChElementData* data_per_element     ///< pointer to auxiliary data (ex states), if any, per element 
    ) override {

        // right Cauchy-Green deformation tensor C = F^T*F
        ChMatrix33d C_deformation = F.transpose() * F;

        // Use the hyperelastic tangent modulus computation
        ComputeElasticTangentModulus(C, C_deformation);
    };

    /// Hyperelastic materials do not need info on the spatial velocity gradient  l=\nabla_x v ,
    /// Returning false from this means that the ChDomainXXYY can know that the
    /// computation of the "l" parameter could be skipped left to null when calling ComputeStress(...)
    virtual bool IsSpatialVelocityGradientNeeded() const override { 
        return false;
    }

    /// Compute elastic stress from the right Cauchy-Green deformation tensor C_def = F^T*F. 
    /// Return stress as 2nd Piola-Kirchhoff S tensor, in Voigt notation. 
    /// Children materials MUST implement this.
    
    virtual void ComputeElasticStress(ChStressTensor<>& S_stress, const ChMatrix33d& C_def) = 0;

    /// Computes the tangent modulus for a given deformation.
    /// It takes the right Cauchy-Green deformation tensor C_def, and returns the tangent 
    /// modulus in 6x6 matrix C_tangent, for  dS = C_tangent * dE  where S is 2nd Piola-Kirchhoff stress, and E is Green Lagrange strain.
    /// A fallback implementation is provided here, that computes C via numerical differentiation by calling 6 times
    /// the ComputeElasticStress() function. However, if you are able to compute a custom analytical expression of C_tangent, 
    /// and the analytical expression is fast to compute, then you SHOULD override this.
    
    virtual void ComputeElasticTangentModulus(ChMatrixNM<double, 6, 6>& C_tangent, const ChMatrix33d& C_def) {
        // Step size for perturbation (relative)
        const double eps = 1e-6;

        ChStressTensor S0_voigt;
        ComputeElasticStress(S0_voigt, C_def);

        C_tangent.setZero();

        ChStrainEngTensor C_voigt;
        C_voigt.ConvertFromMatrix(C_def);

        // Perturb each component of C
        for (int i = 0; i < 6; ++i) {
            // Compute step size
            double h = eps;
            if (std::abs(C_voigt(i)) > 1e-8) {
                h = eps * std::abs(C_voigt(i));  // relative perturbation
            }

            ChStrainEngTensor C_plus_voigt = C_voigt;
            C_plus_voigt(i) += h;

            ChMatrix33d C_perturbed;
            C_plus_voigt.ConvertToMatrix(C_perturbed);

            // Compute stress at perturbed C
            ChStressTensor S_plus_voigt;
            ComputeElasticStress(S_plus_voigt, C_perturbed);

            // Forward difference for dS/dC = (S_plus_voigt - S0_voigt) / h
            // But we need dS/dE. Since E = 0.5*(C - I), we have dE = 0.5*dC. Hence the 2.0 factor here.
            ChVectorN<double, 6> dS_dE = (S_plus_voigt - S0_voigt) * (2.0 / h);

            C_tangent.col(i) = dS_dE;
        }
    }
};




/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
