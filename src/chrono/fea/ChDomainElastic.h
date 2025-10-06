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

#ifndef CHDOMAINELASTIC_H
#define CHDOMAINELASTIC_H

#include "chrono/fea/ChDomain.h"
#include "chrono/fea/ChMaterial3DStressStVenant.h"

namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{


/// Domain for FEA nonlinear finite strain analysis. It is based on a vector field,
/// ChFieldDisplacement3D, that is used to store x, the absolute spatial position of nodes (NOT
/// the displacement from the material reference position, d=x-X, as in some software).
/// Material properties are defined via a material from the ChMaterial3DStress subclasses
/// (the simplest of these materials is the ChMaterial3DStressStVenant, that corresponds to 
/// conventional linear elasticity for small strains). 

class ChDomainElastic : public ChDomainImpl<
    std::tuple<ChFieldDisplacement3D>, // per each node
    ChFieldDataNONE,   // per each GP
    ChFeaPerElementDataKRM> { // per each element
public:
    ChDomainElastic(std::shared_ptr<ChFieldDisplacement3D> melasticfield)
        : ChDomainImpl( melasticfield )
    {
        // attach  default materials to simplify user side
        material = chrono_types::make_shared<ChMaterial3DStressStVenant>();
    }

    /// Elastic properties of this domain 
    std::shared_ptr<ChMaterial3DStress>  material;

    // INTERFACES

    /// Computes the internal loads Fi for one quadrature point, except quadrature weighting, 
    /// and *ADD* the s-scaled result to Fi vector.
    virtual void PointComputeInternalLoads(std::shared_ptr<ChFieldElement> melement,
                                        DataPerElement& data,
                                        const int i_point,
                                        ChVector3d& eta,
                                        const double s,
                                        ChVectorDynamic<>& Fi
    ) override {
        ChVectorDynamic<> T;
        this->GetStateBlock(melement, T);
        ChMatrixDynamic<> dNdX;
        ChRowVectorDynamic<> N;
        melement->ComputedNdX(eta, dNdX);
        melement->ComputeN(eta, N);

        // F deformation tensor = J_x * J_X^{-1}
        //   J_X: already available via element->ComputeJ()
        //   J_x: compute via   [x1|x2|x3|x4..]*dNde'
        ChMatrixDynamic<> Xhat(3, melement->GetNumNodes());
        for (unsigned int i = 0; i < melement->GetNumNodes(); ++i) {
            Xhat.block(0, i, 3, 1) = ((ChFieldDataPos3D*)(data.nodes_data[i][0]))->GetPos().eigen();
        }
        ChMatrixDynamic<> dNde;
        melement->ComputedNde(eta, dNde);

        ChMatrix33d J_X_inv;
        melement->ComputeJinv(eta, J_X_inv);

        ChMatrix33d F = Xhat * dNde.transpose() * J_X_inv;  

        // Compute  2nd Piola-Kirchhoff tensor in Voigt notation using the constitutive relation of material
        ChStressTensor<> S_stress; 
        material->ComputeStress(S_stress, F);

        ChMatrixDynamic<> B(6, 3 * melement->GetNumNodes());
        this->ComputeB(B, dNdX, F);
        
        // We have:             Fi = - sum (B' * S * w * |J|)
        // so here we compute  Fi += - B' * S * s
        Fi += -(B.transpose() * S_stress) * s;
    }

    /// Sets matrix H = Mfactor*M + Rfactor*dFi/dv + Kfactor*dFi/dx, as scaled sum of the tangent matrices M,R,K,:
    /// H = Mfactor*M + Rfactor*R + Kfactor*K. 
    /// Setting Mfactor=1 and Rfactor=Kfactor=0, it can be used to get just mass matrix, etc.
    virtual void PointComputeKRMmatrices(std::shared_ptr<ChFieldElement> melement,
                                        DataPerElement& data,
                                        const int i_point,
                                        ChVector3d& eta,
                                        ChMatrixRef H,
                                        double Kpfactor,
                                        double Rpfactor = 0,
                                        double Mpfactor = 0
    ) override {
        ChMatrixDynamic<> dNdX;
        ChRowVectorDynamic<> N;
        melement->ComputedNdX(eta, dNdX);
        melement->ComputeN(eta, N);

        // F deformation tensor = J_x * J_X^{-1}
        //   J_X: already available via element->ComputeJ()
        //   J_x: compute via   [x1|x2|x3|x4..]*dNde'
        ChMatrixDynamic<> Xhat(3, melement->GetNumNodes());
        for (unsigned int i = 0; i < melement->GetNumNodes(); ++i) {
            Xhat.block(0, i, 3, 1) = ((ChFieldDataPos3D*)(data.nodes_data[i][0]))->GetPos().eigen();
        }

        ChMatrixDynamic<> dNde;
        melement->ComputedNde(eta, dNde);

        ChMatrix33d J_X_inv;
        melement->ComputeJinv(eta, J_X_inv);

        ChMatrix33d F = Xhat * dNde.transpose() * J_X_inv;

        ChMatrixDynamic<> B(6, 3 * melement->GetNumNodes());
        this->ComputeB(B, dNdX, F);

        // Compute tangent modulus (assumed: dP=[C]dE with P  2nd Piola-Kirchhoff, E Green-Lagrange)
        ChMatrix66<double> C;
        this->material->ComputeTangentModulus(C, F);

        // K  matrix 
        // K = sum (B' * k * B  * w * |J|)  
        if (Kpfactor) {
            H += Kpfactor * (B.transpose() * C * B);
            // ***TODO*** add the geometric tangent stiffness
            // ***TODO*** rayleigh damping
        }

        // M  matrix : consistent mass matrix:   
        // M = sum (N' * rho * N * w * |J|)
        if (Mpfactor) {
            // If we had the "3 rows" form of the shape function matrix, say N_ where N_=[N(1)*I, N(2)*I, ], it would be
            //   M = sum (N_' * rho * N_ * w * |J|)     that is simply:
            //   H += (Mpfactor * this->material->GetDensity()) * (N_.transpose() * N_);
            // But the N_ matrix would be very sparse, so to speedup computation we unroll it and do:
            double scalar_factor = Mpfactor * this->material->GetDensity();
            for (int i = 0; i < N.cols(); i++) {
                for (int j = 0; j < N.cols(); j++) {
                    // Compute the scalar entry for the 8x8 scalar mass matrix
                    double scalar_entry = scalar_factor * N(i) * N(j);
                    int row_start = i * 3;
                    int col_start = j * 3;
                    H(row_start, col_start)         += scalar_entry; // xx
                    H(row_start + 1, col_start + 1) += scalar_entry; // yy  
                    H(row_start + 2, col_start + 2) += scalar_entry; // zz
                }
            }

            // ***TODO*** rayleigh damping
        }
    }

private:
    /// Utility: Compute  B as in  dE = B dx  where dE is variation in Green Lagrange strain (Voigt notation)
    /// and dx is the variation in spatial node coordinates (also works as  dE = B du  with du variation in displacements)
    void ComputeB(ChMatrixRef B, ChMatrixConstRef dNdX, ChMatrixConstRef F) {
        B.resize(6, 3 * dNdX.cols());
        B.setZero();
        for (int i = 0; i < dNdX.cols(); ++i) {
            // g = ∇₀ N_i = J_X⁻¹ ∇_ξ N_i = dNdX(:, i)
            //                          g₁* [F₁₁, F₂₁, F₃₁]
            B.block<1, 3>(0, i * 3) = dNdX(0, i) * F.block<3, 1>(0, 0).transpose();
            //                          g₂* [F₁₂, F₂₂, F₃₂]
            B.block<1, 3>(1, i * 3) = dNdX(1, i) * F.block<3, 1>(0, 1).transpose();
            //                          g₃* [F₁₃, F₂₃, F₃₃]
            B.block<1, 3>(2, i * 3) = dNdX(2, i) * F.block<3, 1>(0, 2).transpose();
            //                          g₂* [F₁₃, F₂₃, F₃₃]                             + g₃ * [F₁₂, F₂₂, F₃₂]
            B.block<1, 3>(3, i * 3) = dNdX(1, i) * F.block<3, 1>(0, 2).transpose() + dNdX(2, i) * F.block<3, 1>(0, 1).transpose();
            //                          g₁* [F₁₃, F₂₃, F₃₃]                             + g₃ * [F₁₁, F₂₁, F₃₁]
            B.block<1, 3>(4, i * 3) = dNdX(0, i) * F.block<3, 1>(0, 2).transpose() + dNdX(2, i) * F.block<3, 1>(0, 0).transpose();
            //                          g₁* [F₁₂, F₂₂, F₃₂]                             + g₂ * [F₁₁, F₂₁, F₃₁]
            B.block<1, 3>(5, i * 3) = dNdX(0, i) * F.block<3, 1>(0, 1).transpose() + dNdX(1, i) * F.block<3, 1>(0, 0).transpose();
        }
    }

};




/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
