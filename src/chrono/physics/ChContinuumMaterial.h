// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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
#include "chrono/core/ChMath.h"
#include "chrono/physics/ChTensors.h"

namespace chrono {
namespace fea {

// -----------------------------------------------------------------------------

/// Base class for properties of materials in a continuum.

class ChApi ChContinuumMaterial {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChContinuumMaterial)

  protected:
    double density;

  public:
    ChContinuumMaterial(double mdensity = 1000) : density(mdensity) {}
    ChContinuumMaterial(const ChContinuumMaterial& other);
    virtual ~ChContinuumMaterial() {}

    /// Set the density of the material, in kg/m^2.
    void Set_density(double m_density) { density = m_density; }
    /// Get the density of the material, in kg/m^2.
    double Get_density() const { return density; }

    virtual void ArchiveOUT(ChArchiveOut& marchive);
    virtual void ArchiveIN(ChArchiveIn& marchive);
};


/// Class for the basic properties of materials in an elastic continuum.
/// This is a base material with isotropic hookean elasticity.

class ChApi ChContinuumElastic : public ChContinuumMaterial {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChContinuumElastic)

  private:
    double E;                              ///< Young Modulus
    double v;                              ///< Poisson ratio
    double G;                              ///< shear modulus
    double l;                              ///< Lame's modulus
    ChMatrixDynamic<> StressStrainMatrix;  ///< Elasticity (stiffness) matrix :   = [E] ε

    double damping_M;  ///< Raleigh_damping, M proportional
    double damping_K;  ///< Raleigh_damping, K proportional

  public:
    /// Create a continuum isothropic hookean material.
    /// Default value for Young elastic modulus is low (like a
    /// rubber-type material), and same for density.
    ChContinuumElastic(double myoung = 10000000, double mpoisson = 0.4, double mdensity = 1000);
    ChContinuumElastic(const ChContinuumElastic& other);
    virtual ~ChContinuumElastic() {}

    /// Set the Young E elastic modulus, in Pa (N/m^2), as the ratio of the uniaxial
    /// stress over the uniaxial strain, for hookean materials. Intuitively, the
    /// tensile pressure on a side of a parallelepiped in order to double its length.
    /// Note that most metal materials require very high values, ex. steel
    /// has E=210GPa (E=210e9), aluminium E=69e9, and this can cause numerical
    /// problems if you do not set up the simulation integrator properly.
    void Set_E(double m_E);
    /// Get the Young E elastic modulus, in Pa (N/m^2).
    double Get_E() const { return E; }

    /// Set the Poisson v ratio, as v=-transverse_strain/axial_strain, so
    /// takes into account the 'squeezing' effect of materials that are pulled (so,
    /// if zero, when you push the two sizes of a cube, it won't inflate). Most
    /// materials have some 0<v<0.5, for example steel has v=0.27..0.30, aluminium v=0.33,
    /// rubber=0.49, etc. Note! v=0.5 means perfectly incompressible material, that
    /// could give problems with some type of solvers.
    /// Setting v also changes G.
    void Set_v(double m_v);
    /// Get the Young v ratio, as v=-transverse_strain/axial_strain.
    double Get_v() const { return v; }

    /// Set the shear modulus G, in Pa (N/m^2), as the ratio of shear stress to
    /// the shear strain. Setting G also changes Poisson ratio v.
    void Set_G(double m_G);
    /// Get the shear modulus G, in Pa (N/m^2)
    double Get_G() const { return G; }

    /// Get Lamé first parameter (the second is shear modulus, so Get_G() )
    double Get_l() const { return l; }

    /// Get bulk modulus (increase of pressure for decrease of volume), in Pa.
    double Get_BulkModulus() const { return E / (3 * (1 - 2 * v)); }

    /// Get P-wave modulus (if V=speed of propagation of a P-wave, then (M/density)=V^2 )
    double Get_WaveModulus() const { return E * ((1 - v) / (1 + v) * (1 - 2 * v)); }

    /// Computes Elasticity matrix and stores the value in this->StressStrainMatrix
    /// Note: is performed every time you change a material parameter
    void ComputeStressStrainMatrix();
    /// Get the Elasticity matrix
    ChMatrixDynamic<>& Get_StressStrainMatrix() { return StressStrainMatrix; }

    /// Compute elastic stress from elastic strain
    /// (using column tensors, in Voight notation)
    void ComputeElasticStress(ChStressTensor<>& mstress, const ChStrainTensor<>& mstrain) const;

    /// Compute elastic strain from elastic stress
    /// (using column tensors, in Voight notation)
    void ComputeElasticStrain(ChStrainTensor<>& mstrain, const ChStressTensor<>& mstress) const;

    /// Set the Rayleigh mass-proportional damping factor alpha, to
    /// build damping R as  R=alpha*M + beta*K
    void Set_RayleighDampingM(double m_d) { damping_M = m_d; }
    /// Set the Rayleigh mass-proportional damping factor alpha, in R=alpha*M + beta*K
    double Get_RayleighDampingM() const { return damping_M; }

    /// Set the Rayleigh stiffness-proportional damping factor beta, to
    /// build damping R as  R=alpha*M + beta*K
    void Set_RayleighDampingK(double m_d) { damping_K = m_d; }
    /// Set the Rayleigh stiffness-proportional damping factor beta, in R=alpha*M + beta*K
    double Get_RayleighDampingK() const { return damping_K; }

    virtual void ArchiveOUT(ChArchiveOut& marchive) override;
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};


// -----------------------------------------------------------------------------

/// Class for all elastic materials that can undergo plastic flow.
/// Defines simply some interface functions.

class ChApi ChContinuumElastoplastic : public ChContinuumElastic {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChContinuumElastoplastic)

  public:
    ChContinuumElastoplastic(double myoung = 10000000, double mpoisson = 0.4, double mdensity = 1000)
        : ChContinuumElastic(myoung, mpoisson, mdensity) {}

    /// Return a scalar value that is 0 on the yeld surface, <0 inside (elastic), >0 outside (incompatible->plastic
    /// flow)
    virtual double ComputeYeldFunction(const ChStressTensor<>& mstress) const = 0;

    /// Compute plastic strain flow (flow derivative dE_plast/dt) from strain,
    /// according to VonMises strain yeld theory.
    virtual void ComputePlasticStrainFlow(ChStrainTensor<>& mplasticstrainflow,
                                          const ChStrainTensor<>& mtotstrain) const = 0;

    /// Correct the strain-stress by enforcing that elastic stress must remain on the yeld
    /// surface, computing a plastic flow to be added to plastic strain while integrating.
    virtual void ComputeReturnMapping(ChStrainTensor<>& mplasticstrainflow,
                                      const ChStrainTensor<>& mincrementstrain,
                                      const ChStrainTensor<>& mlastelasticstrain,
                                      const ChStrainTensor<>& mlastplasticstrain) const = 0;

    /// Set the plastic flow rate, i.e. the 'creep' speed. The lower the value, the slower
    /// the plastic flow during dynamic simulations, with delayed plasticity.
    virtual void Set_flow_rate(double mflow_rate) = 0;
    /// Set the plastic flow rate.
    virtual double Get_flow_rate() const = 0;

    virtual void ArchiveOUT(ChArchiveOut& marchive) override;
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};


// -----------------------------------------------------------------------------

/// Class for the basic properties of materials in an elastoplastic continuum,
/// with strain yeld limit based on Von Mises yeld.

class ChApi ChContinuumPlasticVonMises : public ChContinuumElastoplastic {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChContinuumPlasticVonMises)

  private:
    double elastic_yeld;
    double plastic_yeld;

    double flow_rate;

  public:
    /// Create a continuum isothropic elastoplastic material,
    /// where you can define also plastic and elastic max. stress (yeld limits
    /// for transition elastic->blastic and plastic->fracture).
    ChContinuumPlasticVonMises(double myoung = 10000000,
                               double mpoisson = 0.4,
                               double mdensity = 1000,
                               double melastic_yeld = 0.1,
                               double mplastic_yeld = 0.2);
    ChContinuumPlasticVonMises(const ChContinuumPlasticVonMises& other);
    virtual ~ChContinuumPlasticVonMises() {}

    /// Set the elastic yeld modulus as the maximum VonMises
    /// equivalent strain that can be withstood by material before
    /// starting plastic flow. It defines the transition elastic->plastic.
    void Set_elastic_yeld(double melastic_yeld) { elastic_yeld = melastic_yeld; };
    /// Get the elastic yeld modulus.
    double Get_elastic_yeld() const { return elastic_yeld; }

    /// Set the plastic yeld modulus as the maximum VonMises
    /// equivalent strain that can be withstood by material before
    /// fracture. It defines the transition plastic->fracture.
    void Set_plastic_yeld(double mplastic_yeld) { plastic_yeld = mplastic_yeld; };
    /// Get the plastic yeld modulus.
    double Get_plastic_yeld() const { return plastic_yeld; }

    /// Set the plastic flow rate. The lower the value, the slower
    /// the plastic flow during dynamic simulations.
    virtual void Set_flow_rate(double mflow_rate) override { flow_rate = mflow_rate; };
    /// Set the plastic flow rate.
    virtual double Get_flow_rate() const override { return flow_rate; }

    virtual double ComputeYeldFunction(const ChStressTensor<>& mstress) const override;

    virtual void ComputeReturnMapping(ChStrainTensor<>& mplasticstrainflow,
                                      const ChStrainTensor<>& mincrementstrain,
                                      const ChStrainTensor<>& mlastelasticstrain,
                                      const ChStrainTensor<>& mlastplasticstrain) const override;

    /// Compute plastic strain flow (flow derivative dE_plast/dt) from strain,
    /// according to VonMises strain yeld theory.
    virtual void ComputePlasticStrainFlow(ChStrainTensor<>& mplasticstrainflow,
                                          const ChStrainTensor<>& mestrain) const override;

    virtual void ArchiveOUT(ChArchiveOut& marchive) override;
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};


// -----------------------------------------------------------------------------

/// Class for the basic properties of elastoplastic materials of Drucker-Prager type,
/// that are useful for simulating soils.

class ChApi ChContinuumDruckerPrager : public ChContinuumElastoplastic {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChContinuumDruckerPrager)

  private:
    double elastic_yeld;
    double alpha;
    double dilatancy;
    double hardening_speed;
    double hardening_limit;
    double flow_rate;

  public:
    /// Create a continuum isothropic Drucker-Prager material
    ChContinuumDruckerPrager(double myoung = 10000000,
                             double mpoisson = 0.4,
                             double mdensity = 1000,
                             double melastic_yeld = 0.1,
                             double malpha = 0.5,
                             double mdilatancy = 0);
    ChContinuumDruckerPrager(const ChContinuumDruckerPrager& other);
    virtual ~ChContinuumDruckerPrager() {}

    /// Set the D-P yeld modulus C, for Drucker-Prager
    /// yeld. It defines the transition elastic->plastic.
    void Set_elastic_yeld(double melastic_yeld) { elastic_yeld = melastic_yeld; }
    /// Get the elastic yeld modulus C
    double Get_elastic_yeld() const { return elastic_yeld; }

    /// Set the internal friction coefficient A
    void Set_alpha(double malpha) { alpha = malpha; }
    /// Get the internal friction coefficient A
    double Get_alpha() const { return alpha; }

    /// Sets the C and A parameters of the Drucker-Prager model
    /// starting from more 'practical' values of inner friction angle phi
    /// and cohesion, as used in the faceted Mohr-Coulomb model.
    /// Use the optional parameter inner_approx to set if the faceted
    /// Mohr-Coulomg must be approximated with D-P inscribed (default) or circumscribed.
    void Set_from_MohrCoulomb(double phi, double cohesion, bool inner_approx = true);

    /// Set the plastic flow rate multiplier. The lower the value, the slower
    /// the plastic flow during dynamic simulations.
    virtual void Set_flow_rate(double mflow_rate) override { flow_rate = mflow_rate; }
    /// Get the flow rate multiplier.
    virtual double Get_flow_rate() const override { return flow_rate; }

    /// Set the internal dilatancy coefficient (usually 0.. < int.friction)
    void Set_dilatancy(double mdilatancy) { dilatancy = mdilatancy; }
    /// Get the internal dilatancy coefficient
    double Get_dilatancy() const { return dilatancy; }

    /// Set the hardening limit (usually a bit larger than yeld), or softening
    void Set_hardening_limit(double mhl) { hardening_limit = mhl; }
    /// Get the hardening limit
    double Get_hardening_limit() const { return hardening_limit; }

    /// Set the hardening inverse speed coeff. for exponential hardening
    /// (the larger, the slower the hardening or softening process that
    /// will asymptotycally make yeld = hardening_limit )
    void Set_hardening_speed(double mhl) { hardening_speed = mhl; }
    /// Get the hardening speed
    double Get_hardening_speed() const { return hardening_speed; }

    virtual double ComputeYeldFunction(const ChStressTensor<>& mstress) const override;

    virtual void ComputeReturnMapping(ChStrainTensor<>& mplasticstrainflow,
                                      const ChStrainTensor<>& mincrementstrain,
                                      const ChStrainTensor<>& mlastelasticstrain,
                                      const ChStrainTensor<>& mlastplasticstrain) const override;

    /// Compute plastic strain flow direction from strain
    /// according to Drucker-Prager.
    virtual void ComputePlasticStrainFlow(ChStrainTensor<>& mplasticstrainflow,
                                          const ChStrainTensor<>& mestrain) const override;

    virtual void ArchiveOUT(ChArchiveOut& marchive) override;
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

}  // end namespace fea


CH_CLASS_VERSION(fea::ChContinuumMaterial,0)
CH_CLASS_VERSION(fea::ChContinuumElastic,0)
CH_CLASS_VERSION(fea::ChContinuumElastoplastic,0)
CH_CLASS_VERSION(fea::ChContinuumPlasticVonMises,0)
CH_CLASS_VERSION(fea::ChContinuumDruckerPrager,0)


}  // end namespace chrono

#endif
