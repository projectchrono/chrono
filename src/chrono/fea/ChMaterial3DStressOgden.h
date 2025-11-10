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

#ifndef CHMATERIAL3DSTRESSOGDEN_H
#define CHMATERIAL3DSTRESSOGDEN_H

#include "chrono/fea/ChMaterial3DHyperelastic.h"


namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{



/// Class for Ogden materials. A subtype of hyperelastic materials.
/// It is mostly used for plastics, rubbers, organic tissue, etc.
/// It supports very large strains.

class ChMaterial3DStressOgden : public ChMaterial3DHyperelastic {
private:
    std::vector<double> mu;
    std::vector<double> alpha;
    double kappa;

public:
    ChMaterial3DStressOgden() {
 
    }

    virtual ~ChMaterial3DStressOgden() {}

    /// access coefficients "mu" in Ogden material
    std::vector<double>& CoefficientsMu()  {
        return mu;
    }
    
    /// access coefficients "alpha" in Ogden material
    std::vector<double>& CoefficientsAlpha() {
        return alpha;
    }

    /// Set bulk modulus kappa
    void SetKappa(double mk) {
        kappa = mk;
    }

    /// Get bulk modulus kappa
    double GetKappa() {
        return kappa;
    }


    //  CONSTITUTIVE LAW
    /// Compute elastic stress from elastic strain. 
    /// Starts computing Green-Lagrange strain E from C_deformation, the right Cauchy-Green deformation.
    /// For small strains the Green Lagrange strain in Voigt notation coincides with espilon tensor.
    /// Return stress as Piola-Kirchhoff S tensor, in Voigt notation. 
    /// This is a very simple material, ie. a linear funciton  S=C:E with C as 4th order constant tensor,
    /// also S=[C]*E with 6x6 C in Voigt notation. 
    virtual void ComputeElasticStress(ChStressTensor<>& stress, const ChMatrix33d& C_deformation) override {
        
        const double J = std::sqrt(C_deformation.determinant());

        const ChMatrix33d Cinv = C_deformation.inverse();

        // Eigen-decomposition of C
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(C_deformation);
        const ChVector3d lambda_sq = solver.eigenvalues();
        const ChMatrix33d N = solver.eigenvectors(); // columns = N_i

        // Principal stretches
        auto alambda = lambda_sq.eigen().array().sqrt();
        ChVector3d lambda(alambda[0], alambda[1], alambda[2]);
        const double Jm13 = std::pow(J, -1.0 / 3.0);
        auto alambda_bar = lambda.eigen().array()* Jm13;
        const ChVector3d lambda_bar(alambda_bar[0], alambda_bar[1], alambda_bar[2]);

        // Isochoric stress
        ChMatrix33d S_iso = ChMatrix33d::Zero();
        for (size_t p = 0; p < mu.size(); ++p) {
            for (int i = 0; i < 3; ++i) {
                const double coeff = mu[p] * std::pow(lambda_bar.eigen()(i), alpha[p] - 2.0);
                S_iso += coeff * (N.col(i) * N.col(i).transpose());
            }
        }
        S_iso *= std::pow(J, -2.0 / 3.0);

        // Volumetric stress
        const ChMatrix33d S_vol = kappa * (J - 1.0) * J * Cinv;
        const ChMatrix33d S = S_iso + S_vol;
        stress.ConvertFromMatrix(S);

    }

    /// Computes the tangent modulus C. 
    /// (The Cauchy-Green deformation "C_deformation" is not used here, as C  in S=C:E is independent on C_deformation)
    virtual void ComputeElasticTangentModulus(ChMatrixNM<double, 6, 6>& C, const ChMatrix33d& C_deformation) override {

        const double J = std::sqrt(C_deformation.determinant());

        const ChMatrix33d Cinv = C_deformation.inverse();

        // Eigen-decomposition
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(C_deformation);
        const ChVector3d lambda_sq = solver.eigenvalues();
        const ChMatrix33d N = solver.eigenvectors();
        auto alambda = lambda_sq.eigen().array().sqrt();
        const ChVector3d lambda(alambda[0], alambda[1], alambda[2]);
        const double Jm13 = std::pow(J, -1.0 / 3.0);
        auto alambda_bar = lambda.eigen().array() * Jm13;
        const ChVector3d lambda_bar(alambda_bar[0], alambda_bar[1], alambda_bar[2]); 

        // Compute s_i and t_i
        ChVector3d s_bar = VNULL, t_bar = VNULL;
        for (size_t p = 0; p < mu.size(); ++p) {
            for (int i = 0; i < 3; ++i) {
                const double l_alpha = std::pow(lambda_bar.eigen()(i), alpha[p]);
                s_bar.eigen()(i) += mu[p] * l_alpha / (lambda_bar.eigen()(i) * lambda_bar.eigen()(i));
                t_bar.eigen()(i) += mu[p] * (alpha[p] - 2.0) * l_alpha /
                    (lambda_bar.eigen()(i) * lambda_bar.eigen()(i) * lambda_bar.eigen()(i) * lambda_bar.eigen()(i));
            }
        }

        // Scale and account for J-dependence
        const double Jm23 = std::pow(J, -2.0 / 3.0);
        const ChVector3d s = Jm23 * s_bar;
        const ChVector3d t = Jm23 * (t_bar - (2.0 / 3.0) * s_bar);

        // Build tangent in principal basis
        ChMatrixNM<double, 6, 6> D_hat; D_hat.setZero();
        const double eps = 1e-12;

        // Normal components (0,1,2)
        for (int i = 0; i < 3; ++i) {
            D_hat(i, i) = 0.5 * t.eigen()(i);
        }

        // Off-diagonal normal-normal
        for (int i = 0; i < 3; ++i) {
            for (int j = i + 1; j < 3; ++j) {
                const double denom = lambda_sq.eigen()(i) - lambda_sq.eigen()(j);
                const double value = (std::abs(denom) > eps) ?
                    (s.eigen()(i) - s.eigen()(j)) / denom : 0.5 * t.eigen()(i);
                D_hat(i, j) = D_hat(j, i) = value;
            }
        }

        // Shear components (3,4,5)
        D_hat(3, 3) = (std::abs(lambda_sq.eigen()(1) + lambda_sq.eigen()(2)) > eps) ?
            (s.eigen()(1) + s.eigen()(2)) / (lambda_sq.eigen()(1) + lambda_sq.eigen()(2)) : 0.0;
        D_hat(4, 4) = (std::abs(lambda_sq.eigen()(0) + lambda_sq.eigen()(2)) > eps) ?
            (s.eigen()(0) + s.eigen()(2)) / (lambda_sq.eigen()(0) + lambda_sq.eigen()(2)) : 0.0;
        D_hat(5, 5) = (std::abs(lambda_sq.eigen()(0) + lambda_sq.eigen()(1)) > eps) ?
            (s.eigen()(0) + s.eigen()(1)) / (lambda_sq.eigen()(0) + lambda_sq.eigen()(1)) : 0.0;

        // Convert dS/dC to dS/dE and rotate to global basis
        ChMatrixNM<double, 6, 6> D_iso = 2.0 * D_hat;
        const ChMatrixNM<double, 6, 6> Q = buildVoigtRotationMatrix(N);
        D_iso = Q * D_iso * Q.transpose();

        // Volumetric tangent (dominant term)
        ChStressTensor Cinv_voigt;
        Cinv_voigt.ConvertFromMatrix(Cinv); // no *2 factor in shear
        const ChMatrixNM<double, 6, 6> D_vol = kappa * (2 * J - 1.0) * Cinv_voigt * Cinv_voigt.transpose();

        C = D_iso + D_vol;
    }


    

    //virtual void ArchiveOut(ChArchiveOut& archive_out) override;  TODO
    //virtual void ArchiveIn(ChArchiveIn& archive_in) override; 

private:

    // Build 6x6 Voigt rotation matrix from eigenvectors (columns of R)
    ChMatrixNM<double, 6, 6> buildVoigtRotationMatrix(const ChMatrix33d& R) {
        double r11 = R(0, 0), r12 = R(0, 1), r13 = R(0, 2);
        double r21 = R(1, 0), r22 = R(1, 1), r23 = R(1, 2);
        double r31 = R(2, 0), r32 = R(2, 1), r33 = R(2, 2);

        ChMatrixNM<double, 6, 6> Q;
        Q <<
            r11 * r11, r12* r12, r13* r13, 2 * r12 * r13, 2 * r11 * r13, 2 * r11 * r12,
            r21* r21, r22* r22, r23* r23, 2 * r22 * r23, 2 * r21 * r23, 2 * r21 * r22,
            r31* r31, r32* r32, r33* r33, 2 * r32 * r33, 2 * r31 * r33, 2 * r31 * r32,
            r21* r31, r22* r32, r23* r33, r22* r33 + r23 * r32, r21* r33 + r23 * r31, r21* r32 + r22 * r31,
            r11* r31, r12* r32, r13* r33, r12* r33 + r13 * r32, r11* r33 + r13 * r31, r11* r32 + r12 * r31,
            r11* r21, r12* r22, r13* r23, r12* r23 + r13 * r22, r11* r23 + r13 * r21, r11* r22 + r12 * r21;
        return Q;
    }
};






/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
