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

#include <algorithm>
#include <cmath>

#include "chrono/physics/ChMaterialSurfaceSMC.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChMaterialSurfaceSMC)

ChMaterialSurfaceSMC::ChMaterialSurfaceSMC()
    : ChMaterialSurface(),
      young_modulus(2e5),
      poisson_ratio(0.3f),
      constant_adhesion(0),
      adhesionMultDMT(0),
      adhesionSPerko(0),
      kn(2e5),
      kt(2e5),
      gn(40),
      gt(20) {}

ChMaterialSurfaceSMC::ChMaterialSurfaceSMC(const ChMaterialSurfaceSMC& other) : ChMaterialSurface(other) {
    young_modulus = other.young_modulus;
    poisson_ratio = other.poisson_ratio;
    constant_adhesion = other.constant_adhesion;
    adhesionMultDMT = other.adhesionMultDMT;
    adhesionSPerko = other.adhesionSPerko;
    kn = other.kn;
    kt = other.kt;
    gn = other.gn;
    gt = other.gt;
}

void ChMaterialSurfaceSMC::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChMaterialSurfaceSMC>();

    // serialize parent class
    ChMaterialSurface::ArchiveOut(marchive);

    // serialize all member data:
    marchive << CHNVP(young_modulus);
    marchive << CHNVP(poisson_ratio);
    marchive << CHNVP(constant_adhesion);
    marchive << CHNVP(adhesionMultDMT);
    marchive << CHNVP(adhesionSPerko);
    marchive << CHNVP(kn);
    marchive << CHNVP(kt);
    marchive << CHNVP(gn);
    marchive << CHNVP(gt);
}

void ChMaterialSurfaceSMC::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChMaterialSurfaceSMC>();

    // deserialize parent class
    ChMaterialSurface::ArchiveIn(marchive);

    // stream in all member data:
    marchive >> CHNVP(young_modulus);
    marchive >> CHNVP(poisson_ratio);
    marchive >> CHNVP(constant_adhesion);
    marchive >> CHNVP(adhesionMultDMT);
    marchive >> CHNVP(adhesionSPerko);
    marchive >> CHNVP(kn);
    marchive >> CHNVP(kt);
    marchive >> CHNVP(gn);
    marchive >> CHNVP(gt);
}

// -----------------------------------------------------------------------------

ChMaterialCompositeSMC::ChMaterialCompositeSMC()
    : E_eff(0),
      G_eff(0),
      mu_eff(0),
      muRoll_eff(0),
      muSpin_eff(0),
      cr_eff(0),
      adhesion_eff(0),
      adhesionMultDMT_eff(0),
      adhesionSPerko_eff(0),
      kn(0),
      kt(0),
      gn(0),
      gt(0) {}

ChMaterialCompositeSMC::ChMaterialCompositeSMC(ChMaterialCompositionStrategy* strategy,
                                               std::shared_ptr<ChMaterialSurfaceSMC> mat1,
                                               std::shared_ptr<ChMaterialSurfaceSMC> mat2) {
    float inv_E = (1 - mat1->poisson_ratio * mat1->poisson_ratio) / mat1->young_modulus +
                  (1 - mat2->poisson_ratio * mat2->poisson_ratio) / mat2->young_modulus;
    float inv_G = 2 * (2 - mat1->poisson_ratio) * (1 + mat1->poisson_ratio) / mat1->young_modulus +
                  2 * (2 - mat2->poisson_ratio) * (1 + mat2->poisson_ratio) / mat2->young_modulus;

    E_eff = 1 / inv_E;
    G_eff = 1 / inv_G;

    mu_eff = strategy->CombineFriction(mat1->static_friction, mat2->static_friction);
    muRoll_eff = strategy->CombineFriction(mat1->rolling_friction, mat2->rolling_friction);
    muSpin_eff = strategy->CombineFriction(mat1->spinning_friction, mat2->spinning_friction);
    cr_eff = strategy->CombineRestitution(mat1->restitution, mat2->restitution);
    adhesion_eff = strategy->CombineCohesion(mat1->constant_adhesion, mat2->constant_adhesion);
    adhesionMultDMT_eff = strategy->CombineAdhesionMultiplier(mat1->adhesionMultDMT, mat2->adhesionMultDMT);
    adhesionSPerko_eff = strategy->CombineAdhesionMultiplier(mat1->adhesionSPerko, mat2->adhesionSPerko);

    kn = strategy->CombineStiffnessCoefficient(mat1->kn, mat2->kn);
    kt = strategy->CombineStiffnessCoefficient(mat1->kt, mat2->kt);
    gn = strategy->CombineDampingCoefficient(mat1->gn, mat2->gn);
    gt = strategy->CombineDampingCoefficient(mat1->gt, mat2->gt);
}

}  // end namespace chrono
