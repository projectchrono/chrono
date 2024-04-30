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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHCONTINUUMMATERIAL_H
#define CHCONTINUUMMATERIAL_H

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChFrame.h"
#include "chrono/core/ChTensors.h"

namespace chrono {
namespace fea {

/// @addtogroup chrono_fea
/// @{

// -----------------------------------------------------------------------------

/// Base class for properties of materials in a continuum.

class ChApi ChContinuumMaterial {
  protected:
    double m_density;

  public:
    ChContinuumMaterial(double density = 1000) : m_density(density) {}
    ChContinuumMaterial(const ChContinuumMaterial& other);
    virtual ~ChContinuumMaterial() {}

    /// Set the density of the material, in kg/m^2.
    void SetDensity(double density) { m_density = density; }

    /// Get the density of the material, in kg/m^2.
    double GetDensity() const { return m_density; }

    virtual void ArchiveOut(ChArchiveOut& archive_out);
    virtual void ArchiveIn(ChArchiveIn& archive_in);
};

/// Class for the basic properties of materials in an elastic continuum.
/// This is a base material with isotropic hookean elasticity.

class ChApi ChContinuumElastic : public ChContinuumMaterial {
  private:
    double m_E;                            ///< Young Modulus
    double m_poisson;                      ///< Poisson ratio
    double m_lamefirst;                    ///< Lame's first parameter
    ChMatrixDynamic<> StressStrainMatrix;  ///< Elasticity (stiffness) matrix :   = [E] ε

    double m_rayl_damping_alpha;  ///< Rayleigh damping coeff, M proportional
    double m_rayl_damping_beta;   ///< Rayleigh damping coeff, K proportional

  public:
    ChContinuumElastic(double young = 10000000, double poisson = 0.4, double density = 1000);

    ChContinuumElastic(const ChContinuumElastic& other);

    virtual ~ChContinuumElastic() {}

    /// Set the Young elastic modulus, in Pa (N/m^2), as the ratio of the uniaxial
    /// stress over the uniaxial strain, for hookean materials.
    void SetYoungModulus(double E);

    /// Get the Young elastic modulus, in Pa (N/m^2).
    double GetYoungModulus() const { return m_E; }

    /// Set the Poisson ratio, as v=-transverse_strain/axial_strain, so
    /// takes into account the 'squeezing' effect of materials that are pulled.
    /// Note: v=0.5 means perfectly incompressible material, that could give problems with some type of solvers.
    /// Setting v also changes G.
    void SetPoissonRatio(double v);

    /// Get the Poisson ratio, as v=-transverse_strain/axial_strain.
    double GetPoissonRatio() const { return m_poisson; }

    /// Set the shear modulus G, in Pa (N/m^2).
    /// Setting G also changes Poisson ratio v.
    void SetShearModulus(double G);

    /// Get the shear modulus G, in Pa (N/m^2)
    double GetShearModulus() const { return m_E / (2 * (1 + m_poisson)); }

    /// Get Lamé first parameter (the second is shear modulus, so GetShearModulus() )
    double GetLameFirstParam() const { return m_lamefirst; }

    /// Get bulk modulus (increase of pressure for decrease of volume), in Pa.
    double GetBulkModulus() const { return m_E / (3 * (1 - 2 * m_poisson)); }

    /// Get P-wave modulus (if V=speed of propagation of a P-wave, then (M/density)=V^2 )
    double GetPWaveModulus() const { return m_E * ((1 - m_poisson) / (1 + m_poisson) * (1 - 2 * m_poisson)); }

    /// Computes Elasticity matrix and stores the value in this->StressStrainMatrix
    /// Note: is performed every time you change a material parameter
    void ComputeStressStrainMatrix();

    /// Get the Elasticity matrix
    ChMatrixDynamic<>& GetStressStrainMatrix() { return StressStrainMatrix; }

    /// Compute elastic stress from elastic strain
    /// (using column tensors, in Voight notation)
    void ComputeElasticStress(ChStressTensor<>& stress, const ChStrainTensor<>& strain) const;

    /// Compute elastic strain from elastic stress
    /// (using column tensors, in Voight notation)
    void ComputeElasticStrain(ChStrainTensor<>& strain, const ChStressTensor<>& stress) const;

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

    virtual void ArchiveOut(ChArchiveOut& archive_out) override;
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

// -----------------------------------------------------------------------------

/// Class for all elastic materials that can undergo plastic flow.
/// Defines simply some interface functions.

class ChApi ChContinuumElastoplastic : public ChContinuumElastic {
  public:
    ChContinuumElastoplastic(double young = 10000000, double poisson = 0.4, double density = 1000)
        : ChContinuumElastic(young, poisson, density) {}

    /// Return a scalar value that is 0 on the yield surface, <0 inside (elastic), >0 outside (incompatible->plastic
    /// flow)
    virtual double ComputeYieldFunction(const ChStressTensor<>& stress) const = 0;

    /// Compute plastic strain flow (flow derivative dE_plast/dt) from strain,
    /// according to VonMises strain yield theory.
    virtual void ComputePlasticStrainFlow(ChStrainTensor<>& plasticstrainflow,
                                          const ChStrainTensor<>& totstrain) const = 0;

    /// Correct the strain-stress by enforcing that elastic stress must remain on the yield
    /// surface, computing a plastic flow to be added to plastic strain while integrating.
    virtual void ComputeReturnMapping(ChStrainTensor<>& plasticstrainflow,
                                      const ChStrainTensor<>& incrementstrain,
                                      const ChStrainTensor<>& lastelasticstrain,
                                      const ChStrainTensor<>& lastplasticstrain) const = 0;

    /// Set the plastic flow rate, i.e. the 'creep' speed. The lower the value, the slower
    /// the plastic flow during dynamic simulations, with delayed plasticity.
    virtual void SetPlasticFlowRate(double flow_rate) = 0;

    /// Set the plastic flow rate.
    virtual double GetPlasticFlowRate() const = 0;

    virtual void ArchiveOut(ChArchiveOut& archive_out) override;
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

// -----------------------------------------------------------------------------

/// Class for the basic properties of materials in an elastoplastic continuum,
/// with strain yield limit based on Von Mises yield.

class ChApi ChContinuumPlasticVonMises : public ChContinuumElastoplastic {
  private:
    double m_elastic_yield;
    double m_plastic_yield;

    double m_plastic_flow_rate;

  public:
    /// Create a continuum isotropic elastoplastic material,
    /// where you can define also plastic and elastic max. stress (yield limits
    /// for transition elastic->plastic and plastic->fracture).
    ChContinuumPlasticVonMises(double young = 10000000,
                               double poisson = 0.4,
                               double density = 1000,
                               double elastic_yield = 0.1,
                               double plastic_yield = 0.2);
    ChContinuumPlasticVonMises(const ChContinuumPlasticVonMises& other);
    virtual ~ChContinuumPlasticVonMises() {}

    /// Set the elastic yield modulus as the maximum VonMises
    /// equivalent strain that can be withstood by material before
    /// starting plastic flow. It defines the transition elastic->plastic.
    void SetElasticYield(double elastic_yield) { m_elastic_yield = elastic_yield; };

    /// Get the elastic yield modulus.
    double GetElasticYield() const { return m_elastic_yield; }

    /// Set the plastic yield modulus as the maximum VonMises
    /// equivalent strain that can be withstood by material before
    /// fracture. It defines the transition plastic->fracture.
    void SetPlasticYield(double plastic_yield) { m_plastic_yield = plastic_yield; };

    /// Get the plastic yield modulus.
    double GetPlasticYield() const { return m_plastic_yield; }

    /// Set the plastic flow rate. The lower the value, the slower
    /// the plastic flow during dynamic simulations.
    virtual void SetPlasticFlowRate(double flow_rate) override { m_plastic_flow_rate = flow_rate; };
    /// Set the plastic flow rate.
    virtual double GetPlasticFlowRate() const override { return m_plastic_flow_rate; }

    virtual double ComputeYieldFunction(const ChStressTensor<>& stress) const override;

    virtual void ComputeReturnMapping(ChStrainTensor<>& plasticstrainflow,
                                      const ChStrainTensor<>& incrementstrain,
                                      const ChStrainTensor<>& lastelasticstrain,
                                      const ChStrainTensor<>& lastplasticstrain) const override;

    /// Compute plastic strain flow (flow derivative dE_plast/dt) from strain,
    /// according to VonMises strain yield theory.
    virtual void ComputePlasticStrainFlow(ChStrainTensor<>& plasticstrainflow,
                                          const ChStrainTensor<>& totstrain) const override;

    virtual void ArchiveOut(ChArchiveOut& archive_out) override;
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

// -----------------------------------------------------------------------------

/// Class for the basic properties of elastoplastic materials of Drucker-Prager type,
/// that are useful for simulating soils.

class ChApi ChContinuumDruckerPrager : public ChContinuumElastoplastic {
  private:
    double m_elastic_yield;
    double m_alpha;
    double m_dilatancy;
    double m_hardening_speed;
    double m_hardening_limit;
    double m_plastic_flow_rate;

  public:
    /// Create a continuum isotropic Drucker-Prager material
    ChContinuumDruckerPrager(double young = 10000000,
                             double poisson = 0.4,
                             double density = 1000,
                             double elastic_yield = 0.1,
                             double alpha = 0.5,
                             double dilatancy = 0);
    ChContinuumDruckerPrager(const ChContinuumDruckerPrager& other);
    virtual ~ChContinuumDruckerPrager() {}

    /// Set the D-P yield modulus C, for Drucker-Prager
    /// yield. It defines the transition elastic->plastic.
    void SetElasticYield(double elastic_yield) { m_elastic_yield = elastic_yield; }

    /// Get the elastic yield modulus C
    double GetElasticYield() const { return m_elastic_yield; }

    /// Set the internal friction coefficient A
    void SetInternalFriction(double alpha) { m_alpha = alpha; }

    /// Get the internal friction coefficient A
    double GetInternalFriction() const { return m_alpha; }

    /// Sets the C and A parameters of the Drucker-Prager model
    /// starting from more 'practical' values of inner friction angle phi
    /// and cohesion, as used in the faceted Mohr-Coulomb model.
    /// Use the optional parameter inner_approx to set if the faceted
    /// Mohr-Coulomb must be approximated with D-P inscribed (default) or circumscribed.
    void SetFromMohrCoulomb(double phi, double cohesion, bool inner_approx = true);

    /// Set the plastic flow rate multiplier. The lower the value, the slower
    /// the plastic flow during dynamic simulations.
    virtual void SetPlasticFlowRate(double flow_rate) override { m_plastic_flow_rate = flow_rate; }

    /// Get the flow rate multiplier.
    virtual double GetPlasticFlowRate() const override { return m_plastic_flow_rate; }

    /// Set the internal dilatation coefficient (usually 0.. < int.friction)
    void SetDilatancy(double dilatancy) { m_dilatancy = dilatancy; }

    /// Get the internal dilatation coefficient
    double GetDilatancy() const { return m_dilatancy; }

    /// Set the hardening limit (usually a bit larger than yield), or softening
    void SetHardeningLimit(double hl) { m_hardening_limit = hl; }

    /// Get the hardening limit
    double GetHardeningLimit() const { return m_hardening_limit; }

    /// Set the hardening inverse speed coeff. for exponential hardening
    /// (the larger, the slower the hardening or softening process that
    /// will asymptotically make yield = hardening_limit )
    void SetHardeningSpeed(double hl) { m_hardening_speed = hl; }

    /// Get the hardening speed
    double GetHardeningSpeed() const { return m_hardening_speed; }

    virtual double ComputeYieldFunction(const ChStressTensor<>& stress) const override;

    virtual void ComputeReturnMapping(ChStrainTensor<>& plasticstrainflow,
                                      const ChStrainTensor<>& incrementstrain,
                                      const ChStrainTensor<>& lastelasticstrain,
                                      const ChStrainTensor<>& lastplasticstrain) const override;

    /// Compute plastic strain flow direction from strain
    /// according to Drucker-Prager.
    virtual void ComputePlasticStrainFlow(ChStrainTensor<>& plasticstrainflow,
                                          const ChStrainTensor<>& totstrain) const override;

    virtual void ArchiveOut(ChArchiveOut& archive_out) override;
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

/// @} chrono_fea

}  // end namespace fea

CH_CLASS_VERSION(fea::ChContinuumMaterial, 0)
CH_CLASS_VERSION(fea::ChContinuumElastic, 0)
CH_CLASS_VERSION(fea::ChContinuumElastoplastic, 0)
CH_CLASS_VERSION(fea::ChContinuumPlasticVonMises, 0)
CH_CLASS_VERSION(fea::ChContinuumDruckerPrager, 0)

}  // end namespace chrono

#endif
