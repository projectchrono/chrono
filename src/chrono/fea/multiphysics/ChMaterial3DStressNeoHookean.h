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

#ifndef CHMATERIAL3DSTRESSNEOHOOKEAN_H
#define CHMATERIAL3DSTRESSNEOHOOKEAN_H

#include "chrono/fea/multiphysics/ChMaterial3DHyperelastic.h"


namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{



/// Class for properties of the 3D elasticity from Neo-Hookean model.
/// It is a simple type of hyperelastic material that, for small strains, converges
/// to linear elasticity. Yet can be used for large rotations etc. Better than the ChMaterial3DStressStVenant
/// because it can withstand larger compression. It can be used for metals and rubber (up to moderate deformations)

class ChMaterial3DStressNeoHookean : public ChMaterial3DHyperelastic {
private:
    double m_E;                            ///< Young Modulus
    double m_poisson;                      ///< Poisson ratio
    double m_lamefirst;                    ///< Lame's first parameter

    double m_rayl_damping_alpha;  ///< Rayleigh damping coeff, M proportional
    double m_rayl_damping_beta;   ///< Rayleigh damping coeff, K proportional

public:
    ChMaterial3DStressNeoHookean(double young = 10000000, double poisson = 0.4, double density = 1000) {
        m_E = young;
        SetPoissonRatio(poisson);     // sets also Lamé, precomputes E matrix
        this->m_rayl_damping_alpha = 0;
        this->m_rayl_damping_beta = 0;
    }

    virtual ~ChMaterial3DStressNeoHookean() {}

    /// Set the Young elastic modulus, in Pa (N/m^2), as the ratio of the uniaxial
    /// stress over the uniaxial strain, for hookean materials.
    void SetYoungModulus(double E) {
        m_E = E;
        m_lamefirst = (m_poisson * m_E) / ((1 + m_poisson) * (1 - 2 * m_poisson));  // Lame's constant l                                                // updates Elasticity matrix
    }

    /// Get the Young elastic modulus, in Pa (N/m^2).
    double GetYoungModulus() const { return m_E; }

    /// Set the Poisson ratio, as v=-transverse_strain/axial_strain, so
    /// takes into account the 'squeezing' effect of materials that are pulled.
    /// Note: v=0.5 means perfectly incompressible material, that could give problems with some type of solvers.
    /// Setting v also changes G.
    void SetPoissonRatio(double v) {
        m_poisson = v;
        m_lamefirst = (m_poisson * m_E) / ((1 + m_poisson) * (1 - 2 * m_poisson));  // Lame's constant l                                               // updates Elasticity matrix
    }

    /// Get the Poisson ratio, as v=-transverse_strain/axial_strain.
    double GetPoissonRatio() const { return m_poisson; }

    /// Set the shear modulus G, in Pa (N/m^2).
    /// Setting G also changes Poisson ratio v.
    void SetShearModulus(double G) {
        m_poisson = (m_E / (2 * G)) - 1;                                            // fixed G, E, get v
        m_lamefirst = (m_poisson * m_E) / ((1 + m_poisson) * (1 - 2 * m_poisson));  // Lame's constant l                                                // updates Elasticity matrix
    }

    /// Get the shear modulus G, in Pa (N/m^2)
    double GetShearModulus() const { return m_E / (2 * (1 + m_poisson)); }

    /// Get Lamé first parameter (the second is shear modulus, so GetShearModulus() )
    double GetLameFirstParam() const { return m_lamefirst; }

    /// Get bulk modulus (increase of pressure for decrease of volume), in Pa.
    double GetBulkModulus() const { return m_E / (3 * (1 - 2 * m_poisson)); }

    /// Get P-wave modulus (if V=speed of propagation of a P-wave, then (M/density)=V^2 )
    double GetPWaveModulus() const { return m_E * ((1 - m_poisson) / (1 + m_poisson) * (1 - 2 * m_poisson)); }

    //  CONSTITUTIVE LAW
    /// Compute elastic stress from elastic strain. 
    /// Starts computing Green-Lagrange strain E from C_deformation, the right Cauchy-Green deformation.
    /// For small strains the Green Lagrange strain in Voigt notation coincides with espilon tensor.
    /// Return stress as Piola-Kirchhoff S tensor, in Voigt notation. 
 
    virtual void ComputeElasticStress(ChStressTensor<>& stress, const ChMatrix33d& C_deformation) override {
        
        // Compute invariants
        double J = std::sqrt(C_deformation.determinant());  // J = det(F)
        double lnJ = std::log(J);

        // Compute C^{-1} (inverse of right Cauchy-Green)
        ChMatrix33d C_inv = C_deformation.inverse();

        // Extract components of C^{-1} in Voigt order: xx, yy, zz, yz, xz, xy
        double c11 = C_inv(0, 0);
        double c22 = C_inv(1, 1);
        double c33 = C_inv(2, 2);
        double c23 = C_inv(1, 2);
        double c13 = C_inv(0, 2);
        double c12 = C_inv(0, 1);

        // Material constants (already stored as member variables)
        double mu = GetShearModulus();  // shear modulus = E/(2*(1+nu))
        double lambda = GetLameFirstParam();  // Lame parameter = E*nu/((1+nu)*(1-2*nu))

        // Compute 2nd PK stress: S = mu*(I - C^{-1}) + lambda * lnJ * C^{-1}
        // In Voigt notation with engineering strains (no extra factors for shear)

        // Normal components (xx, yy, zz)
        stress(0) = mu * (1.0 - c11) + lambda * lnJ * c11;
        stress(1) = mu * (1.0 - c22) + lambda * lnJ * c22;
        stress(2) = mu * (1.0 - c33) + lambda * lnJ * c33;

        // Shear components (yz, xz, xy) - note: NO factor of 2 because S is tensorial
        stress(3) = mu * (0.0 - c23) + lambda * lnJ * c23;  // S_23
        stress(4) = mu * (0.0 - c13) + lambda * lnJ * c13;  // S_13
        stress(5) = mu * (0.0 - c12) + lambda * lnJ * c12;  // S_12

    }

    /// Computes the tangent modulus C. 
    /// It takes the right Cauchy-Green deformation tensor C_def, and returns the tangent 
    /// modulus in 6x6 matrix C, for  dS = C * dE  where S is 2nd PK stress, and E is Green Lagrange strain.
    
    virtual void ComputeElasticTangentModulus(ChMatrixNM<double, 6, 6>& C, const ChMatrix33d& C_def) override {
        
        // Invariants
        double J = std::sqrt(C_def.determinant());
        double lnJ = std::log(J);

        // Inverse
        ChMatrix33d Ci = C_def.inverse();

        double c11 = Ci(0, 0);
        double c22 = Ci(1, 1);
        double c33 = Ci(2, 2);
        double c23 = Ci(1, 2);
        double c13 = Ci(0, 2);
        double c12 = Ci(0, 1);

        double mu = GetShearModulus();
        double lambda = GetLameFirstParam();

        double fact = 2.0 * (mu - lambda * lnJ);

        // Voigt vector
        double v[6] = {c11, c22, c33, c23, c13, c12};

        // Initialize
        C.setZero();

        // ---- First term: lambda * v * v^T ----
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                C(i, j) += lambda * v[i] * v[j];

        // ---- Second term: corrected A matrix ----

        // Row 0
        C(0, 0) += fact * (c11 * c11);
        C(0, 1) += fact * (c12 * c12);
        C(0, 2) += fact * (c13 * c13);
        C(0, 3) += fact * (c12 * c13);
        C(0, 4) += fact * (c11 * c13);
        C(0, 5) += fact * (c11 * c12);

        // Row 1
        C(1, 0) += fact * (c12 * c12);
        C(1, 1) += fact * (c22 * c22);
        C(1, 2) += fact * (c23 * c23);
        C(1, 3) += fact * (c22 * c23);
        C(1, 4) += fact * (c12 * c23);
        C(1, 5) += fact * (c12 * c22);

        // Row 2
        C(2, 0) += fact * (c13 * c13);
        C(2, 1) += fact * (c23 * c23);
        C(2, 2) += fact * (c33 * c33);
        C(2, 3) += fact * (c23 * c33);
        C(2, 4) += fact * (c13 * c33);
        C(2, 5) += fact * (c13 * c23);

        // Row 3
        C(3, 0) += fact * (c12 * c13);
        C(3, 1) += fact * (c22 * c23);
        C(3, 2) += fact * (c23 * c33);
        C(3, 3) += fact * 0.5 * (c22 * c33 + c23 * c23);
        C(3, 4) += fact * 0.5 * (c12 * c33 + c13 * c23);
        C(3, 5) += fact * 0.5 * (c12 * c23 + c13 * c22);

        // Row 4
        C(4, 0) += fact * (c11 * c13);
        C(4, 1) += fact * (c12 * c23);
        C(4, 2) += fact * (c13 * c33);
        C(4, 3) += fact * 0.5 * (c12 * c33 + c13 * c23);
        C(4, 4) += fact * 0.5 * (c11 * c33 + c13 * c13);
        C(4, 5) += fact * 0.5 * (c11 * c23 + c12 * c13);

        // Row 5
        C(5, 0) += fact * (c11 * c12);
        C(5, 1) += fact * (c12 * c22);
        C(5, 2) += fact * (c13 * c23);
        C(5, 3) += fact * 0.5 * (c12 * c23 + c13 * c22);
        C(5, 4) += fact * 0.5 * (c11 * c23 + c12 * c13);
        C(5, 5) += fact * 0.5 * (c11 * c22 + c12 * c12);
    
        //ChMatrixNM<double, 6, 6> C_test;
        //this->ChMaterial3DHyperelastic::ComputeElasticTangentModulus(C_test, C_def);
        //std::cout << "C \n" << C << "\n";
        //std::cout << "C_tst\n" << C_test << "\n\n";
    }


    /// Set the Rayleigh mass-proportional damping factor alpha, to
    /// build damping R as R=alpha*M + beta*K
    void SetRayleighDampingAlpha(double alpha) { m_rayl_damping_alpha = alpha; }

    /// Set the Rayleigh mass-proportional damping factor alpha, in R=alpha*M + beta*K
    double GetRayleighDampingAlpha() const { return m_rayl_damping_alpha; }

    /// Set the Rayleigh stiffness-proportional damping factor beta, to
    /// build damping R as R=alpha*M + beta*K
    void SetRayleighDampingBeta(double beta) { m_rayl_damping_beta = beta; }

    /// Set the Rayleigh stiffness-proportional damping factor beta, in R=alpha*M + beta*K
    double GetRayleighDampingBeta() const { return m_rayl_damping_beta; }

    //virtual void ArchiveOut(ChArchiveOut& archive_out) override;  TODO
    //virtual void ArchiveIn(ChArchiveIn& archive_in) override; 

private:


};






/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
