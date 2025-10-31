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

#ifndef CHMATERIAL3DSTRESSSTVENANT_H
#define CHMATERIAL3DSTRESSSTVENANT_H

#include "chrono/fea/ChMaterial3DHyperelastic.h"


namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{



/// Class for properties of the 3D elasticity from StVenant-Kirchhoff model.
/// It is the simplest type of hyperelastic material. For S PiolaKirchhoff stress 
/// and E Green Lagrange strain tensors, it is S=C:E with C constant 4th order elastic tensor.
/// For small deformations it corresponds to the classical σ=C:ε. In Voigt notation, S=[C]*E,  σ=[C]*ε

class ChMaterial3DStressStVenant : public ChMaterial3DHyperelastic {
private:
    double m_E;                            ///< Young Modulus
    double m_poisson;                      ///< Poisson ratio
    double m_lamefirst;                    ///< Lame's first parameter
    ChMatrixNM<double, 6, 6> StressStrainMatrix;  ///< Elastic matrix in σ=[C]*ε (stored precomputed as it is constant, to speedup)

    double m_rayl_damping_alpha;  ///< Rayleigh damping coeff, M proportional
    double m_rayl_damping_beta;   ///< Rayleigh damping coeff, K proportional

public:
    ChMaterial3DStressStVenant(double young = 10000000, double poisson = 0.4, double density = 1000) {
        m_E = young;
        SetPoissonRatio(poisson);     // sets also Lamé, precomputes E matrix
        this->m_rayl_damping_alpha = 0;
        this->m_rayl_damping_beta = 0;
    }

    virtual ~ChMaterial3DStressStVenant() {}

    /// Set the Young elastic modulus, in Pa (N/m^2), as the ratio of the uniaxial
    /// stress over the uniaxial strain, for hookean materials.
    void SetYoungModulus(double E) {
        m_E = E;
        m_lamefirst = (m_poisson * m_E) / ((1 + m_poisson) * (1 - 2 * m_poisson));  // Lame's constant l
        UpdateStressStrainMatrix();                                                // updates Elasticity matrix
    }

    /// Get the Young elastic modulus, in Pa (N/m^2).
    double GetYoungModulus() const { return m_E; }

    /// Set the Poisson ratio, as v=-transverse_strain/axial_strain, so
    /// takes into account the 'squeezing' effect of materials that are pulled.
    /// Note: v=0.5 means perfectly incompressible material, that could give problems with some type of solvers.
    /// Setting v also changes G.
    void SetPoissonRatio(double v) {
        m_poisson = v;
        m_lamefirst = (m_poisson * m_E) / ((1 + m_poisson) * (1 - 2 * m_poisson));  // Lame's constant l
        UpdateStressStrainMatrix();                                                 // updates Elasticity matrix
    }

    /// Get the Poisson ratio, as v=-transverse_strain/axial_strain.
    double GetPoissonRatio() const { return m_poisson; }

    /// Set the shear modulus G, in Pa (N/m^2).
    /// Setting G also changes Poisson ratio v.
    void SetShearModulus(double G) {
        m_poisson = (m_E / (2 * G)) - 1;                                            // fixed G, E, get v
        m_lamefirst = (m_poisson * m_E) / ((1 + m_poisson) * (1 - 2 * m_poisson));  // Lame's constant l
        UpdateStressStrainMatrix();                                                 // updates Elasticity matrix
    }

    /// Get the shear modulus G, in Pa (N/m^2)
    double GetShearModulus() const { return m_E / (2 * (1 + m_poisson)); }

    /// Get Lamé first parameter (the second is shear modulus, so GetShearModulus() )
    double GetLameFirstParam() const { return m_lamefirst; }

    /// Get bulk modulus (increase of pressure for decrease of volume), in Pa.
    double GetBulkModulus() const { return m_E / (3 * (1 - 2 * m_poisson)); }

    /// Get P-wave modulus (if V=speed of propagation of a P-wave, then (M/density)=V^2 )
    double GetPWaveModulus() const { return m_E * ((1 - m_poisson) / (1 + m_poisson) * (1 - 2 * m_poisson)); }


    /// Compute elastic stress from elastic strain. 
    /// Starts computing Green-Lagrange strain E from C_deformation, the right Cauchy-Green deformation.
    /// For small strains the Green Lagrange strain in Voigt notation coincides with espilon tensor.
    /// Return stress as Piola-Kirchhoff S tensor, in Voigt notation. 
    /// This is a very simple material, ie. a linear funciton  S=C:E with C as 4th order constant tensor,
    /// also S=[C]*E with 6x6 C in Voigt notation. 
    virtual void ComputeElasticStress(ChStressTensor<>& stress, const ChMatrix33d& C_deformation) override {
        
        // Green Lagrange    E = 1/2( F*F' - I) = 1/2( C - I) 
        ChMatrix33d E_strain33 = 0.5 * (C_deformation - ChMatrix33d(1));
    
        // Green Lagrange in Voigt notation (todo: optimization: could be skipped)
        ChStrainTensor<> strain; 
        strain.ConvertFromMatrix(E_strain33);
        strain.XY() *= 2; strain.XZ() *= 2; strain.YZ() *= 2;

        double G = GetShearModulus();
        stress.XX() = strain.XX() * (m_lamefirst + 2 * G) + strain.YY() * m_lamefirst + strain.ZZ() * m_lamefirst;
        stress.YY() = strain.XX() * m_lamefirst + strain.YY() * (m_lamefirst + 2 * G) + strain.ZZ() * m_lamefirst;
        stress.ZZ() = strain.XX() * m_lamefirst + strain.YY() * m_lamefirst + strain.ZZ() * (m_lamefirst + 2 * G);
        stress.XY() = strain.XY() * G;
        stress.XZ() = strain.XZ() * G;
        stress.YZ() = strain.YZ() * G;
    }

    /// Computes the tangent modulus C. 
    /// (The Cauchy-Green deformation "C_deformation" is not used here, as C  in S=C:E is independent on C_deformation)
    virtual void ComputeElasticTangentModulus(ChMatrixNM<double, 6, 6>& C, const ChMatrix33d& C_deformation) override {
        C = this->StressStrainMatrix;
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

    /// This is just for optimization. The following code should go into ComputeTangentModulus(), but
    /// since E is constant, it is computed here into this->StressStrainMatrix every time one changes shear modulus, etc. via setters like SetShearModulus() etc.
    /// Later, the ComputeTangentModulus() can just copy from this->StressStrainMatrix, achieving higher speed. 
    void UpdateStressStrainMatrix() {
        StressStrainMatrix.setZero(6, 6);
        double G = GetShearModulus();
        // Fill the upper-left 3x3 block
        for (int i = 0; i < 3; ++i) {
            StressStrainMatrix(i, i) = m_lamefirst + 2.0 * G;
            for (int j = 0; j < 3; ++j) {
                if (i != j) {
                    StressStrainMatrix(i, j) = m_lamefirst;
                }
            }
        }
        // Fill the shear components
        StressStrainMatrix(3, 3) = G;
        StressStrainMatrix(4, 4) = G;
        StressStrainMatrix(5, 5) = G;
    }
};






/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
