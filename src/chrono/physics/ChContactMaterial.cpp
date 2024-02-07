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

#include "chrono/physics/ChContactMaterial.h"
#include "chrono/physics/ChContactMaterialSMC.h"
#include "chrono/physics/ChContactMaterialNSC.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {
CH_FACTORY_REGISTER(ChContactMaterial)
CH_FACTORY_REGISTER(ChContactMaterialData);
CH_FACTORY_REGISTER(ChContactMaterialComposite);
CH_FACTORY_REGISTER(ChContactMaterialCompositionStrategy);

// -----------------------------------------------------------------------------

ChContactMaterial::ChContactMaterial()
    : static_friction(0.6f),
      sliding_friction(0.6f),
      rolling_friction(0),
      spinning_friction(0),
      restitution(0.4f) {}

ChContactMaterial::ChContactMaterial(const ChContactMaterial& other) {
    static_friction = other.static_friction;
    sliding_friction = other.sliding_friction;
    rolling_friction = other.rolling_friction;
    spinning_friction = other.spinning_friction;
    restitution = other.restitution;
}

void ChContactMaterial::SetFriction(float val) {
    SetStaticFriction(val);
    SetSlidingFriction(val);
}

void ChContactMaterial::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChContactMaterial>();

    // serialize all member data:
    marchive << CHNVP(static_friction);
    marchive << CHNVP(sliding_friction);
    marchive << CHNVP(rolling_friction);
    marchive << CHNVP(spinning_friction);
    marchive << CHNVP(restitution);
}

void ChContactMaterial::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChContactMaterial>();

    // stream in all member data:
    marchive >> CHNVP(static_friction);
    marchive >> CHNVP(sliding_friction);
    marchive >> CHNVP(rolling_friction);
    marchive >> CHNVP(spinning_friction);
    marchive >> CHNVP(restitution);
}

std::shared_ptr<ChContactMaterial> ChContactMaterial::DefaultMaterial(ChContactMethod contact_method) {
    switch (contact_method) {
        case ChContactMethod::NSC:
            return chrono_types::make_shared<ChContactMaterialNSC>();
        case ChContactMethod::SMC:
            return chrono_types::make_shared<ChContactMaterialSMC>();
    }
    return nullptr;
}

ChContactMaterialData::ChContactMaterialData()
    : mu(0.8f), cr(0.01f), Y(2e7f), nu(0.3f), kn(2e5f), gn(40.0f), kt(2e5f), gt(20.0f) {}

ChContactMaterialData::ChContactMaterialData(float mu_,
                                             float cr_,
                                             float Y_,
                                             float nu_,
                                             float kn_,
                                             float gn_,
                                             float kt_,
                                             float gt_)
    : mu(mu_), cr(cr_), Y(Y_), nu(nu_), kn(kn_), gn(gn_), kt(kt_), gt(gt_) {}

std::shared_ptr<ChContactMaterial> ChContactMaterialData::CreateMaterial(ChContactMethod contact_method) const {
    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChContactMaterialNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChContactMaterialSMC>();
            matSMC->SetFriction(mu);
            matSMC->SetRestitution(cr);
            matSMC->SetYoungModulus(Y);
            matSMC->SetPoissonRatio(nu);
            matSMC->SetKn(kn);
            matSMC->SetGn(gn);
            matSMC->SetKt(kt);
            matSMC->SetGt(gt);
            return matSMC;
        }
        default:
            return std::shared_ptr<ChContactMaterial>();
    }
}

void ChContactMaterialData::ArchiveOut(ChArchiveOut& marchive) {
    marchive.VersionWrite<ChContactMaterialData>();

    marchive << CHNVP(mu);
    marchive << CHNVP(cr);
    marchive << CHNVP(Y);
    marchive << CHNVP(nu);
    marchive << CHNVP(kn);
    marchive << CHNVP(gn);
    marchive << CHNVP(kt);
    marchive << CHNVP(gt);
}

void ChContactMaterialData::ArchiveIn(ChArchiveIn& marchive) {
    marchive.VersionRead<ChContactMaterialData>();

    marchive >> CHNVP(mu);
    marchive >> CHNVP(cr);
    marchive >> CHNVP(Y);
    marchive >> CHNVP(nu);
    marchive >> CHNVP(kn);
    marchive >> CHNVP(gn);
    marchive >> CHNVP(kt);
    marchive >> CHNVP(gt);
}



void ChContactMaterialComposite::ArchiveOut(ChArchiveOut& marchive) {
    marchive.VersionWrite<ChContactMaterialComposite>();
}

void ChContactMaterialComposite::ArchiveIn(ChArchiveIn& marchive) {
    marchive.VersionRead<ChContactMaterialComposite>();
}


void ChContactMaterialCompositionStrategy::ArchiveOut(ChArchiveOut& marchive) {
    marchive.VersionWrite<ChContactMaterialCompositionStrategy>();
}

void ChContactMaterialCompositionStrategy::ArchiveIn(ChArchiveIn& marchive) {
    marchive.VersionRead<ChContactMaterialCompositionStrategy>();
}

}  // end namespace chrono
