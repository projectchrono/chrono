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

#include "chrono/core/ChClassFactory.h"
#include "chrono/physics/ChContactMaterialNSC.h"

namespace chrono {

// -----------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChContactMaterialNSC)
CH_FACTORY_REGISTER(ChContactMaterialCompositeNSC)

CH_UPCASTING(ChContactMaterialNSC, ChContactMaterial)
CH_UPCASTING(ChContactMaterialCompositeNSC, ChContactMaterialComposite)

ChContactMaterialNSC::ChContactMaterialNSC()
    : ChContactMaterial(),
      cohesion(0),
      dampingf(0),
      compliance(0),
      complianceT(0),
      complianceRoll(0),
      complianceSpin(0) {}

ChContactMaterialNSC::ChContactMaterialNSC(const ChContactMaterialNSC& other) : ChContactMaterial(other) {
    cohesion = other.cohesion;
    dampingf = other.dampingf;
    compliance = other.compliance;
    complianceT = other.complianceT;
    complianceRoll = other.complianceRoll;
    complianceSpin = other.complianceSpin;
}

void ChContactMaterialNSC::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChContactMaterialNSC>();

    // serialize parent class
    ChContactMaterial::ArchiveOut(marchive);

    // serialize all member data:
    marchive << CHNVP(cohesion);
    marchive << CHNVP(dampingf);
    marchive << CHNVP(compliance);
    marchive << CHNVP(complianceT);
    marchive << CHNVP(complianceRoll);
    marchive << CHNVP(complianceSpin);
}

void ChContactMaterialNSC::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChContactMaterialNSC>();

    // deserialize parent class
    ChContactMaterial::ArchiveIn(marchive);

    // stream in all member data:
    marchive >> CHNVP(cohesion);
    marchive >> CHNVP(dampingf);
    marchive >> CHNVP(compliance);
    marchive >> CHNVP(complianceT);
    marchive >> CHNVP(complianceRoll);
    marchive >> CHNVP(complianceSpin);
}

// -----------------------------------------------------------------------------

ChContactMaterialCompositeNSC::ChContactMaterialCompositeNSC()
    : static_friction(0),
      sliding_friction(0),
      rolling_friction(0),
      spinning_friction(0),
      restitution(0),
      cohesion(0),
      dampingf(0),
      compliance(0),
      complianceT(0),
      complianceRoll(0),
      complianceSpin(0) {}

ChContactMaterialCompositeNSC::ChContactMaterialCompositeNSC(ChContactMaterialCompositionStrategy* strategy,
                                                             std::shared_ptr<ChContactMaterialNSC> mat1,
                                                             std::shared_ptr<ChContactMaterialNSC> mat2) {
    static_friction = strategy->CombineFriction(mat1->static_friction, mat2->static_friction);
    sliding_friction = strategy->CombineFriction(mat1->sliding_friction, mat2->sliding_friction);
    restitution = strategy->CombineRestitution(mat1->restitution, mat2->restitution);
    cohesion = strategy->CombineCohesion(mat1->cohesion, mat2->cohesion);
    dampingf = strategy->CombineDamping(mat1->dampingf, mat2->dampingf);
    compliance = strategy->CombineCompliance(mat1->compliance, mat2->compliance);
    complianceT = strategy->CombineCompliance(mat1->complianceT, mat2->complianceT);

    rolling_friction = strategy->CombineFriction(mat1->rolling_friction, mat2->rolling_friction);
    spinning_friction = strategy->CombineFriction(mat1->spinning_friction, mat2->spinning_friction);
    complianceRoll = strategy->CombineCompliance(mat1->complianceRoll, mat2->complianceRoll);
    complianceSpin = strategy->CombineCompliance(mat1->complianceSpin, mat2->complianceSpin);
}

void ChContactMaterialCompositeNSC::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChContactMaterialCompositeNSC>();

    // serialize parent class
    ChContactMaterialComposite::ArchiveOut(marchive);

    // serialize all member data:
    marchive << CHNVP(static_friction);
    marchive << CHNVP(sliding_friction);
    marchive << CHNVP(rolling_friction);
    marchive << CHNVP(spinning_friction);
    marchive << CHNVP(restitution);
    marchive << CHNVP(cohesion);
    marchive << CHNVP(dampingf);
    marchive << CHNVP(compliance);
    marchive << CHNVP(complianceT);
    marchive << CHNVP(complianceRoll);
    marchive << CHNVP(complianceSpin);
}

void ChContactMaterialCompositeNSC::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChContactMaterialCompositeNSC>();

    // deserialize parent class
    ChContactMaterialComposite::ArchiveIn(marchive);

    // stream in all member data:
    marchive >> CHNVP(static_friction);
    marchive >> CHNVP(sliding_friction);
    marchive >> CHNVP(rolling_friction);
    marchive >> CHNVP(spinning_friction);
    marchive >> CHNVP(restitution);
    marchive >> CHNVP(cohesion);
    marchive >> CHNVP(dampingf);
    marchive >> CHNVP(compliance);
    marchive >> CHNVP(complianceT);
    marchive >> CHNVP(complianceRoll);
    marchive >> CHNVP(complianceSpin);
}

}  // end namespace chrono
