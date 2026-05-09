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

#ifndef CHMATERIAL3DSTRESSVISCONEWTON_H
#define CHMATERIAL3DSTRESSVISCONEWTON_H

#include "chrono/fea/multiphysics/ChMaterial3DStress.h"
#include "chrono/core/ChTensors.h"


namespace chrono {
namespace fea {

// Forward:
class ChFieldData;
class ChElementData;


/// @addtogroup chrono_fea
/// @{


/// Class for Newtonian damping. This provides damping forces proportional to the spatial velocity
/// gradient. It is assumed to be used in parallel with some elastic stress model like ChMaterial3DStressStVenant.
/// If used alone, it provides a fluid-like viscous damping.

class ChMaterial3DStressViscoNewton : public ChMaterial3DStress {
public:

    ChMaterial3DStressViscoNewton() : volumetric_damping(0), deviatoric_damping(0) {}

    virtual ~ChMaterial3DStressViscoNewton() {}

    /// Set the damping for the deviatoric effect. This corresponds to the viscous
    /// damping μ in Newtonian fluids.
    virtual void SetDeviatoricDamping(double mdamp) { deviatoric_damping = mdamp; }
    virtual double GetDeviatoricDamping() const { return deviatoric_damping; }
    
    /// Set the damping for the volumetric effect. If the material is incompressible or
    /// nearly incompressible, this term has no effect.
    virtual void SetVolumetricDamping(double mdamp) { volumetric_damping = mdamp; }
    virtual double GetVolumetricDamping() const { return volumetric_damping; }

    /// Compute elastic stress from spatial velocity gradient "l"
    
    virtual void ComputeStress(ChStressTensor<>& S_stress,          ///< output stress, PK2
                                const ChMatrix33d& F,               ///< current deformation gradient tensor
                                const ChMatrix33d* l,               ///< current spatial velocity gradient (might be nullptr if IsSpatialVelocityGradientNeeded() is false)
                                ChFieldData* data_per_point,        ///< pointer to auxiliary data (ex states), if any, per quadrature point
                                ChElementData* data_per_element     ///< pointer to auxiliary data (ex states), if any, per element 
    ) override {
        // compute d, rate of deformation tensor:
        ChMatrix33d d = 0.5 * (*l + l->transpose());  
        
        // Cauchy σ = ζtr(d)I + 2μd
        ChMatrix33d sigma = this->volumetric_damping * d.trace() * Eigen::Matrix3d::Identity()  
                            + 2.0 * this->deviatoric_damping * d;

        // Cauchy σ -> PK2 stress S
        // S = J F⁻¹ (ζ tr(d) I + 2μ d) F⁻ᵀ
        double J = F.determinant();
        ChMatrix33d F_inv = F.inverse();
        ChMatrix33d F_inv_T = F_inv.transpose(); // transpose of F_inv (F^{-T})
        ChMatrix33d S = J * F_inv * sigma * F_inv_T;
        S_stress.ConvertFromMatrix(S);
    }

    /// Computes the tangent modulus 

    virtual void ComputeTangentModulus(ChMatrixNM<double, 6, 6>& C, ///< output C tangent modulus, as dS=C*dE
                                       ChMatrixNM<double, 6, 6>* D, ///< output D tangent modulus, as dS=C*d(E_dot) (maybe nullptr if IsSpatialVelocityGradientNeeded() is false)
                                const ChMatrix33d& F_def,       ///< current deformation gradient tensor
                                const ChMatrix33d* l,           ///< current spatial velocity gradient (might be nullptr if IsSpatialVelocityGradientNeeded() is false)
                                ChFieldData* data_per_point,    ///< pointer to auxiliary data (ex states), if any, per quadrature point
                                ChElementData* data_per_element ///< pointer to auxiliary data (ex states), if any, per element 
    ) override {
        C.setZero();

        assert(D);

        D->setZero();

        const double lambda_v = this->volumetric_damping;
        const double mu_v = this->deviatoric_damping;

        const double J = F_def.determinant();

        ChMatrix33d Finv = F_def.inverse();
        ChMatrix33d FinvT = Finv.transpose();

        auto VoigtToTensor = [](int k, int& i, int& j) {
            switch (k) {
                case 0:
                    i = 0;
                    j = 0;
                    break;
                case 1:
                    i = 1;
                    j = 1;
                    break;
                case 2:
                    i = 2;
                    j = 2;
                    break;
                case 3:
                    i = 1;
                    j = 2;
                    break;
                case 4:
                    i = 0;
                    j = 2;
                    break;
                case 5:
                    i = 0;
                    j = 1;
                    break;
            }
        };

        // loop over Voigt columns
        // each column corresponds to one basis perturbation dE_dot

        for (int col = 0; col < 6; ++col) {
            // build perturbation dE_dot

            ChMatrix33d dEdot = ChMatrix33d::Zero();

            int a, b;
            VoigtToTensor(col, a, b);

            dEdot(a, b) = 1.0;

            if (a != b)
                dEdot(b, a) = 1.0;

            // d = F^{-T} dE_dot F^{-1}

            ChMatrix33d d = FinvT * dEdot * Finv;

            // dsigma

            ChMatrix33d dsigma = lambda_v * d.trace() * ChMatrix33d::Identity() + 2.0 * mu_v * d;

            // dS = J F^{-1} dsigma F^{-T}

            ChMatrix33d dS = J * Finv * dsigma * FinvT;

            // store into Voigt matrix

            for (int row = 0; row < 6; ++row) {
                int i, j;
                VoigtToTensor(row, i, j);

                double val = dS(i, j);

                // engineering shear scaling:
                // strain-like quantities use factor 2

                if (col >= 3)
                    val *= 0.5;

                (*D)(row, col) = val;
            }
        }
    }

    /// This material need info on the spatial velocity gradient  l=\nabla_x v 
    virtual bool IsSpatialVelocityGradientNeeded() const {
        return true;
    }

    //virtual void ArchiveOut(ChArchiveOut& archive_out) override;  TODO
    //virtual void ArchiveIn(ChArchiveIn& archive_in) override; 

private:
    double volumetric_damping;
    double deviatoric_damping;
};




/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
