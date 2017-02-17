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

#include <algorithm>
#include <cmath>

#include "chrono/physics/ChMaterialSurfaceDEM.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChMaterialSurfaceDEM)

// Constructors for a DEM material
ChMaterialSurfaceDEM::ChMaterialSurfaceDEM()
    : young_modulus(2e5),
      poisson_ratio(0.3f),
      static_friction(0.6f),
      sliding_friction(0.6f),
      restitution(0.4f),
      constant_adhesion(0),
      adhesionMultDMT(0),
      kn(2e5),
      kt(2e5),
      gn(40),
      gt(20) {}

ChMaterialSurfaceDEM::ChMaterialSurfaceDEM(const ChMaterialSurfaceDEM& other) {
    young_modulus = other.young_modulus;
    poisson_ratio = other.poisson_ratio;
    static_friction = other.static_friction;
    sliding_friction = other.sliding_friction;
    restitution = other.restitution;
    constant_adhesion = other.constant_adhesion;
    adhesionMultDMT = other.adhesionMultDMT;
    kn = other.kn;
    kt = other.kt;
    gn = other.gn;
    gt = other.gt;
}

void ChMaterialSurfaceDEM::SetFriction(float val) {
    SetSfriction(val);
    SetKfriction(val);
}

void ChMaterialSurfaceDEM::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChMaterialSurfaceDEM>();

    // serialize parent class
    ChMaterialSurfaceBase::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(young_modulus);
    marchive << CHNVP(poisson_ratio);
    marchive << CHNVP(static_friction);
    marchive << CHNVP(sliding_friction);
    marchive << CHNVP(restitution);
    marchive << CHNVP(constant_adhesion);
    marchive << CHNVP(adhesionMultDMT);
    marchive << CHNVP(kn);
    marchive << CHNVP(kt);
    marchive << CHNVP(gn);
    marchive << CHNVP(gt);
}

void ChMaterialSurfaceDEM::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChMaterialSurfaceDEM>();

    // deserialize parent class
    ChMaterialSurfaceBase::ArchiveIN(marchive);

    // stream in all member data:
    marchive >> CHNVP(young_modulus);
    marchive >> CHNVP(poisson_ratio);
    marchive >> CHNVP(static_friction);
    marchive >> CHNVP(sliding_friction);
    marchive >> CHNVP(restitution);
    marchive >> CHNVP(constant_adhesion);
    marchive >> CHNVP(adhesionMultDMT);
    marchive >> CHNVP(kn);
    marchive >> CHNVP(kt);
    marchive >> CHNVP(gn);
    marchive >> CHNVP(gt);
}

// Calculate composite material properties as a combination of the physical
// properties of the two specified materials.
ChCompositeMaterialDEM ChMaterialSurfaceDEM::CompositeMaterial(const std::shared_ptr<ChMaterialSurfaceDEM>& mat1,
                                                               const std::shared_ptr<ChMaterialSurfaceDEM>& mat2) {
    ChCompositeMaterialDEM mat;

    float inv_E = (1 - mat1->poisson_ratio * mat1->poisson_ratio) / mat1->young_modulus +
                  (1 - mat2->poisson_ratio * mat2->poisson_ratio) / mat2->young_modulus;
    float inv_G = 2 * (2 - mat1->poisson_ratio) * (1 + mat1->poisson_ratio) / mat1->young_modulus +
                  2 * (2 - mat2->poisson_ratio) * (1 + mat2->poisson_ratio) / mat2->young_modulus;

    mat.E_eff = 1 / inv_E;
    mat.G_eff = 1 / inv_G;

    mat.mu_eff = std::min<float>(mat1->static_friction, mat2->static_friction);

    mat.cr_eff = (mat1->restitution + mat2->restitution) / 2;

    mat.adhesion_eff = std::min<float>(mat1->constant_adhesion, mat2->constant_adhesion);

    mat.adhesionMultDMT_eff = std::min<float>(mat1->adhesionMultDMT, mat2->adhesionMultDMT);

    mat.kn = (mat1->kn + mat2->kn) / 2;
    mat.kt = (mat1->kt + mat2->kt) / 2;
    mat.gn = (mat1->gn + mat2->gn) / 2;
    mat.gt = (mat1->gt + mat2->gt) / 2;

    return mat;
}

}  // end namespace chrono
