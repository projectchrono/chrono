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

/// Material data for a collision surface for use with smooth (penalty) contact method.
class ChApi ChMaterialSurfaceSMC : public ChMaterialSurface {

  public:
    ChMaterialSurfaceSMC();
    ChMaterialSurfaceSMC(const ChMaterialSurfaceSMC& other);
    ~ChMaterialSurfaceSMC() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChMaterialSurfaceSMC* Clone() const override { return new ChMaterialSurfaceSMC(*this); }

    virtual ChContactMethod GetContactMethod() const override { return ChContactMethod::SMC; }

    /// Young's modulus.
    void SetYoungModulus(float val) { young_modulus = val; }
    float GetYoungModulus() const { return young_modulus; }

    // Poisson ratio.
    void SetPoissonRatio(float val) { poisson_ratio = val; }
    float GetPoissonRatio() const { return poisson_ratio; }

    /// Constant cohesion force.
    void SetAdhesion(float val) { constant_adhesion = val; }
    float GetAdhesion() const { return constant_adhesion; }

    /// Adhesion multiplier in the Derjaguin-Muller-Toporov model.
    /// <pre>
    /// In this model,
    ///    adhesion = adhesionMultDMT * sqrt(R_eff)
    /// given the surface energy, w,
    ///    adhesionMultDMT = 2 * CH_C_PI * w * sqrt(R_eff)
    /// given the equilibrium penetration distance, y_eq,
    ///    adhesionMultDMT = 4.0 / 3.0 * E_eff * powf(y_eq, 1.5)
    /// </pre>
    void SetAdhesionMultDMT(float val) { adhesionMultDMT = val; }
    float GetAdhesionMultDMT() const { return adhesionMultDMT; }

	/// Coefficient for Perko adhesion model.
    /// <pre>
    /// In this model (see Perko et al., 2001),
    ///    adhesion = adhesionSPerko * R
    /// The coefficient adhesionSPerko is function of the Hamaker constant A and a measure of cleanliness S.
    /// For lunar regolith, 
    ///    adhesionSPerko = 3.6e-2 * S^2
    /// </pre>
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

    /// Method to allow serialization transient data into ASCII.
    virtual void StreamOut(ChStreamOutAscii& mstream) { mstream << "Material SMC \n"; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

    float young_modulus;      ///< Young's modulus (elastic modulus)
    float poisson_ratio;      ///< Poisson ratio
    float constant_adhesion;  ///< Constant adhesion force, when constant adhesion model is used
    float adhesionMultDMT;    ///< Adhesion multiplier used in DMT model.
    float adhesionSPerko;     ///< Adhesion multiplier used in Perko model.

    float kn;  ///< user-specified normal stiffness coefficient
    float kt;  ///< user-specified tangential stiffness coefficient
    float gn;  ///< user-specified normal damping coefficient
    float gt;  ///< user-specified tangential damping coefficient
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

    ChMaterialCompositeSMC(ChMaterialCompositionStrategy* strategy,
                           std::shared_ptr<ChMaterialSurfaceSMC> mat1,
                           std::shared_ptr<ChMaterialSurfaceSMC> mat2);
};

}  // end namespace chrono

#endif
