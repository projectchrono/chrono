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
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChMaterialSurfaceSMC)

  public:
    float young_modulus;      ///< Young's modulus (elastic modulus)
    float poisson_ratio;      ///< Poisson ratio
    float static_friction;    ///< Static coefficient of friction
    float sliding_friction;   ///< Kinetic coefficient of friction
    float restitution;        ///< Coefficient of restitution
    float constant_adhesion;  ///< Constant adhesion force, when constant adhesion model is used
    float adhesionMultDMT;    ///< Adhesion multiplier used in DMT model.

    // DMT adhesion model:
    //     adhesion = adhesionMultDMT * sqrt(R_eff).
    // Given the surface energy, w,
    //     adhesionMultDMT = 2 * CH_C_PI * w * sqrt(R_eff).
    // Given the equilibrium penetration distance, y_eq,
    //     adhesionMultDMT = 4.0 / 3.0 * E_eff * powf(y_eq, 1.5)

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
    float GetYoungModulus() const { return young_modulus; }
    void SetYoungModulus(float val) { young_modulus = val; }

    // Poisson ratio.
    float GetPoissonRatio() const { return poisson_ratio; }
    void SetPoissonRatio(float val) { poisson_ratio = val; }

    /// Static and kinetic friction coefficients.
    /// Usually in 0..1 range, rarely above. Default 0.6
    float GetSfriction() const { return static_friction; }
    void SetSfriction(float val) { static_friction = val; }

    float GetKfriction() const { return sliding_friction; }
    void SetKfriction(float val) { sliding_friction = val; }

    /// Set both static friction and kinetic friction at once, with same value.
    void SetFriction(float val);

    /// Normal restitution coefficient
    float GetRestitution() const { return restitution; }
    void SetRestitution(float val) { restitution = val; }

    /// Constant cohesion force
    float GetAdhesion() const { return constant_adhesion; }
    void SetAdhesion(float val) { constant_adhesion = val; }

    /// Adhesion multiplier
    float GetAdhesionMultDMT() const { return adhesionMultDMT; }
    void SetAdhesionMultDMT(float val) { adhesionMultDMT = val; }

    /// Stiffness and damping coefficients
    float GetKn() const { return kn; }
    float GetKt() const { return kt; }
    float GetGn() const { return gn; }
    float GetGt() const { return gt; }

    void SetKn(float val) { kn = val; }
    void SetKt(float val) { kt = val; }
    void SetGn(float val) { gn = val; }
    void SetGt(float val) { gt = val; }

    /// Method to allow serializing transient data into in ascii
    /// as a readable item, for example   "chrono::GetLog() << myobject;"
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
    float cr_eff;               ///< Effective coefficient of restitution
    float adhesion_eff;         ///< Effective cohesion force
    float adhesionMultDMT_eff;  ///< Effective adhesion multiplier (DMT model)

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
