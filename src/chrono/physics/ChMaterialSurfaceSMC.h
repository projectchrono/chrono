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

#ifndef CH_MATERIALSURFACE_SMC_H
#define CH_MATERIALSURFACE_SMC_H

#include "chrono/physics/ChMaterialSurface.h"

namespace chrono {

/// Material data for a surface for use with smooth (penalty) contact method.
/// This data is used to define surface properties owned by ChBody rigid bodies and
/// similar objects; it carries information that is used to make contacts.
class ChApi ChMaterialSurfaceSMC : public ChMaterialSurface {

  public:
    float young_modulus;      ///< Young's modulus (elastic modulus)
    float poisson_ratio;      ///< Poisson ratio
    float static_friction;    ///< Static coefficient of friction
    float sliding_friction;   ///< Kinetic coefficient of friction
    float rolling_friction;   ///< Rolling coefficient of friction
    float spinning_friction;  ///< Spinning coefficient of friction
    float restitution;        ///< Coefficient of restitution
    float constant_adhesion;  ///< Constant adhesion force, when constant adhesion model is used
    float adhesionMultDMT;    ///< Adhesion multiplier used in DMT model.
    float adhesionSPerko;     ///< Adhesion multiplier used in Perko model.

    // DMT adhesion model:
    //     adhesion = adhesionMultDMT * sqrt(R_eff).
    // Given the surface energy, w,
    //     adhesionMultDMT = 2 * CH_C_PI * w * sqrt(R_eff).
    // Given the equilibrium penetration distance, y_eq,
    //     adhesionMultDMT = 4.0 / 3.0 * E_eff * powf(y_eq, 1.5)
    // Perko et al. (2001) ahdesion  model:
    //     adhesion = adhesionSPerko * R_eff
    // Given S a the measurement of cleanliness
    //     adhesionSPerko = 3.6E-2 * S^2 (for lunar regolith)

    float kn;  ///< user-specified normal stiffness coefficient
    float kt;  ///< user-specified tangential stiffness coefficient
    float gn;  ///< user-specified normal damping coefficient
    float gt;  ///< user-specified tangential damping coefficient

    ChMaterialSurfaceSMC();
    ChMaterialSurfaceSMC(const ChMaterialSurfaceSMC& other);
    ~ChMaterialSurfaceSMC() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChMaterialSurfaceSMC* Clone() const override { return new ChMaterialSurfaceSMC(*this); }

    virtual ContactMethod GetContactMethod() const override { return SMC; }

    /// Young's modulus.
    void SetYoungModulus(float val) { young_modulus = val; }
    float GetYoungModulus() const { return young_modulus; }

    // Poisson ratio.
    void SetPoissonRatio(float val) { poisson_ratio = val; }
    float GetPoissonRatio() const { return poisson_ratio; }

    /// Static sliding friction coefficient.
    /// Usually in 0..1 range, rarely above. Default 0.6
    void SetSfriction(float val) { static_friction = val; }
    float GetSfriction() const { return static_friction; }

    /// Kinetic sliding friction coefficient.
    void SetKfriction(float val) { sliding_friction = val; }
    float GetKfriction() const { return sliding_friction; }

    /// Set both static friction and kinetic friction at once, with same value.
    void SetFriction(float val);

	/// Rolling friction coefficient. Usually around 1E-3. Default = 0
    void SetRollingFriction(float val) { rolling_friction = val; }
    float GetRollingFriction() const { return rolling_friction; }

	/// Spinning friction coefficient. Usually around 1E-3. Default = 0
    void SetSpinningFriction(float val) { spinning_friction = val; }
    float GetSpinningFriction() const { return spinning_friction; }

    /// Normal coefficient of restitution.
    void SetRestitution(float val) { restitution = val; }
    float GetRestitution() const { return restitution; }

    /// Constant cohesion force.
    void SetAdhesion(float val) { constant_adhesion = val; }
    float GetAdhesion() const { return constant_adhesion; }

    /// Adhesion multiplier in the Derjaguin-Muller-Toporov model.
    void SetAdhesionMultDMT(float val) { adhesionMultDMT = val; }
    float GetAdhesionMultDMT() const { return adhesionMultDMT; }

	/// Coefficient for Perko adhesion model.
    /// In this model (see Perko et al., 2001), adhesion = adhesionSPerko * R.
    /// The coefficient adhesionSPerko is function of the Hamaker constant A and a measurement of cleanliness S.
    /// For lunar regolith, adhesionSPerko = 3.6e-2 * S^2.
    void SetAdhesionSPerko(float val) { adhesionSPerko = val; }
    float GetAdhesionSPerko() const { return adhesionSPerko; }

    /// Stiffness and damping coefficients
    void SetKn(float val) { kn = val; }
    void SetKt(float val) { kt = val; }
    void SetGn(float val) { gn = val; }
    void SetGt(float val) { gt = val; }

    float GetKn() const { return kn; }
    float GetKt() const { return kt; }
    float GetGn() const { return gn; }
    float GetGt() const { return gt; }

    /// Method to allow serializing transient data into in ASCII.
    virtual void StreamOUT(ChStreamOutAscii& mstream) { mstream << "Material SMC \n"; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChMaterialSurfaceSMC, 0)

/// Composite SMC material data for a contact pair.
class ChApi ChMaterialCompositeSMC : public ChMaterialComposite {
  public:
    float E_eff;                ///< Effective elasticity modulus
    float G_eff;                ///< Effective shear modulus
    float mu_eff;               ///< Effective coefficient of friction
    float muRoll_eff;           ///< Effective coefficient of rolling friction
    float muSpin_eff;           ///< Effective coefficient of spinning friction
    float cr_eff;               ///< Effective coefficient of restitution
    float adhesion_eff;         ///< Effective cohesion force
    float adhesionMultDMT_eff;  ///< Effective adhesion multiplier (DMT model)
    float adhesionSPerko_eff;   ///< Effective adhesion multiplier (Perko model)

    float kn;  ///< normal stiffness coefficient
    float kt;  ///< tangential stiffness coefficient
    float gn;  ///< normal viscous damping coefficient
    float gt;  ///< tangential viscuous damping coefficient

    ChMaterialCompositeSMC();

    ChMaterialCompositeSMC(ChMaterialCompositionStrategy<float>* strategy,
                           std::shared_ptr<ChMaterialSurfaceSMC> mat1,
                           std::shared_ptr<ChMaterialSurfaceSMC> mat2);
};

}  // end namespace chrono

#endif
