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

#include "chrono/fea/multiphysics/ChMaterial3DHyperelastic.h"

namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{



/// Class for Ogden materials. A subtype of hyperelastic materials.
/// It is mostly used for plastics, rubbers, organic tissue, etc.
/// It supports very large strains.
/// Note, it depends on pairs of {mu_i, alpha_i} parameters, plus one 
/// parameter for bulk modulus. For stability, it must be (mu_i * alpha_i)>0,
/// for all pairs.

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

    /// For testing reasons or other reasons, initialize {alpha, mu} parameters of
    /// this material such that, for small deformations, it behaves like the linear elastic 
    /// material with a given Young modulus and Poisson coeff. It takes just two couples of {alpha, mu}.
    void SetAsEquivalentNeoHookean(double E,  ///< Young modulus
                                   double nu  ///< Poisson 
    ) {
        double G0 = E / (2.0 * (1.0 + nu));
        this->CoefficientsMu() = {G0 / 2.0, -G0 / 2.0};
        this->CoefficientsAlpha() = {2, -2};
        this->SetKappa(E / (3 * (1.0 - 2.0 * nu)));
    }

    //  CONSTITUTIVE LAW
    /// Compute elastic stress from elastic strain. 
    /// Starts computing Green-Lagrange strain E from C_deformation, the right Cauchy-Green deformation.
    /// For small strains the Green Lagrange strain in Voigt notation coincides with espilon tensor.
    /// Return stress as Piola-Kirchhoff S tensor, in Voigt notation. 
    /// This is a very simple material, ie. a linear funciton  S=C:E with C as 4th order constant tensor,
    /// also S=[C]*E with 6x6 C in Voigt notation. 
    
    virtual void ComputeElasticStress(ChStressTensor<>& stress, const ChMatrix33d& C_deformation) override {
        // Ensure symmetry
        const ChMatrix33d C_sym = (C_deformation + C_deformation.transpose()) * 0.5;

        const double J = std::sqrt(C_sym.determinant());
        const ChMatrix33d Cinv = C_sym.inverse();

        // Eigen-decomposition
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(C_sym);
        const ChVector3d lambda_sq = solver.eigenvalues();
        const ChMatrix33d N = solver.eigenvectors();

        // Principal stretches
        ChVector3d lambda(std::sqrt(lambda_sq[0]), std::sqrt(lambda_sq[1]), std::sqrt(lambda_sq[2]));
        const double Jm13 = std::pow(J, -1.0 / 3.0);
        const ChVector3d lambda_bar(lambda[0] * Jm13, lambda[1] * Jm13, lambda[2] * Jm13);

        // Isochoric stress
        ChMatrix33d S_iso = ChMatrix33d::Zero();
        for (size_t p = 0; p < mu.size(); ++p) {
            // Compute average term
            double avg_term = 0.0;
            for (int b = 0; b < 3; ++b) {
                avg_term += std::pow(lambda_bar[b], alpha[p] - 2.0);
            }
            avg_term /= 3.0;

            // Add contributions
            for (int i = 0; i < 3; ++i) {
                const double term = mu[p] * (std::pow(lambda_bar[i], alpha[p] - 2.0) - avg_term);
                S_iso += term * (N.col(i) * N.col(i).transpose());
            }
        }
        S_iso *= std::pow(J, -2.0 / 3.0);

        // Volumetric stress
        const ChMatrix33d S_vol = kappa * (J - 1.0) * J * Cinv;
        const ChMatrix33d S = S_iso + S_vol;

        stress.ConvertFromMatrix(S);
    }

    
    //virtual void ArchiveOut(ChArchiveOut& archive_out) override;  TODO
    //virtual void ArchiveIn(ChArchiveIn& archive_in) override; 

private:

   

};






/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
