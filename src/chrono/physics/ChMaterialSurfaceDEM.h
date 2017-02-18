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

#ifndef CHMATERIALSURFACEDEM_H
#define CHMATERIALSURFACEDEM_H

#include "chrono/physics/ChMaterialSurfaceBase.h"

namespace chrono {

struct ChCompositeMaterialDEM {
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
};

/// Class for material surface data for DEM contact
class ChApi ChMaterialSurfaceDEM : public ChMaterialSurfaceBase {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChMaterialSurfaceDEM)

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

    ChMaterialSurfaceDEM();
    ChMaterialSurfaceDEM(const ChMaterialSurfaceDEM& other);
    ~ChMaterialSurfaceDEM() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChMaterialSurfaceDEM* Clone() const override { return new ChMaterialSurfaceDEM(*this); }

    virtual ContactMethod GetContactMethod() const override { return DEM; }

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

    /// Calculate composite material properties
    static ChCompositeMaterialDEM CompositeMaterial(const std::shared_ptr<ChMaterialSurfaceDEM>& mat1,
                                                    const std::shared_ptr<ChMaterialSurfaceDEM>& mat2);

    /// Method to allow serializing transient data into in ascii
    /// as a readable item, for example   "chrono::GetLog() << myobject;"
    virtual void StreamOUT(ChStreamOutAscii& mstream) { mstream << "Material DEM \n"; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChMaterialSurfaceDEM,0)


}  // end namespace chrono

#endif
