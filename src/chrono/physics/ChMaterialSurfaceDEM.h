//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHMATERIALSURFACEDEM_H
#define CHMATERIALSURFACEDEM_H

#include "physics/ChMaterialSurfaceBase.h"


namespace chrono {

struct ChCompositeMaterialDEM {
    float E_eff;             ///< Effective elasticity modulus
    float G_eff;             ///< Effective shear modulus
    float mu_eff;            ///< Effective coefficient of friction
    float cr_eff;            ///< Effective coefficient of restitution
    float adhesion_eff;      ///< Effective cohesion force
    float adhesionMultDMT_eff;  ///< Effective adhesion multiplier (DMT model)

    float kn;
    float kt;
    float gn;
    float gt;
};

/// Class for material surface data for DEM contact
class ChApi ChMaterialSurfaceDEM : public ChMaterialSurfaceBase {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI(ChMaterialSurfaceDEM, ChMaterialSurfaceBase);

  public:
    float young_modulus;  ///< Young's modulus (elastic modulus)
    float poisson_ratio;  ///< Poisson ratio

    float static_friction;   ///< Static coefficient of friction
    float sliding_friction;  ///< Kinetic coefficient of friction

    float restitution;  ///< Coefficient of restitution

    float constant_adhesion;      ///< Constant adhesion force, when constant adhesion model is used
    float adhesionMultDMT;  ///< Adhesion multiplier used in DMT model. adhesion = adhesionMultDMT * sqrt(R_eff). Given the
                         ///surface energy, w, adhesionMultDMT = 2 * CH_C_PI * w * sqrt(R_eff). Given the equilibrium
                         ///penetration distance, y_eq, adhesionMultDMT = 4.0 / 3.0 * E_eff * powf(y_eq, 1.5)

    float kn;  ///< user-specified normal stiffness coefficient
    float kt;  ///< user-specified tangential stiffness coefficient
    float gn;  ///< user-specified normal damping coefficient
    float gt;  ///< user-specified tangential damping coefficient

    ChMaterialSurfaceDEM();
    ChMaterialSurfaceDEM(const ChMaterialSurfaceDEM& other);
    ~ChMaterialSurfaceDEM() {}

    virtual ContactMethod GetContactMethod() override { return DEM; };

    /// Young's modulus and Poisson ratio.
    float GetYoungModulus() const { return young_modulus; }
    void SetYoungModulus(float val) { young_modulus = val; }

    float GetPoissonRatio() const { return poisson_ratio; }
    void SetPoissonRatio(float val) { poisson_ratio = val; }

    /// Static and kinetic friction coefficients.
    /// Usually in 0..1 range, rarely above. Default 0.6
    float GetSfriction() const { return static_friction; }
    void SetSfriction(float val) { static_friction = val; }

    float GetKfriction() const { return sliding_friction; }
    void SetKfriction(float val) { sliding_friction = val; }

    /// Set both static friction and kinetic friction at once, with same value.
    void SetFriction(float val) {
        SetSfriction(val);
        SetKfriction(val);
    }

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

    //
    // SERIALIZATION
    //

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) const override {
        // version number
        marchive.VersionWrite(1);

        // serialize parent class
        ChMaterialSurfaceBase::ArchiveOUT(marchive);

        // serialize all member data:
        marchive << CHNVP_OUT(young_modulus);
        marchive << CHNVP_OUT(poisson_ratio);
        marchive << CHNVP_OUT(static_friction);
        marchive << CHNVP_OUT(sliding_friction);
        marchive << CHNVP_OUT(restitution);
        marchive << CHNVP_OUT(constant_adhesion);
        marchive << CHNVP_OUT(adhesionMultDMT);
        marchive << CHNVP_OUT(kn);
        marchive << CHNVP_OUT(kt);
        marchive << CHNVP_OUT(gn);
        marchive << CHNVP_OUT(gt);
    }

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override {
        // version number
        // int version =
        marchive.VersionRead();

        // deserialize parent class
        ChMaterialSurfaceBase::ArchiveIN(marchive);

        // stream in all member data:
        marchive >> CHNVP_IN(young_modulus);
        marchive >> CHNVP_IN(poisson_ratio);
        marchive >> CHNVP_IN(static_friction);
        marchive >> CHNVP_IN(sliding_friction);
        marchive >> CHNVP_IN(restitution);
        marchive >> CHNVP_IN(constant_adhesion);
        marchive >> CHNVP_IN(adhesionMultDMT);
        marchive >> CHNVP_IN(kn);
        marchive >> CHNVP_IN(kt);
        marchive >> CHNVP_IN(gn);
        marchive >> CHNVP_IN(gt);
    }

};

}  // end namespace chrono

#endif
