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

void ChContactMaterialNSC::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChContactMaterialNSC>();

    // serialize parent class
    ChContactMaterial::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(cohesion);
    archive_out << CHNVP(dampingf);
    archive_out << CHNVP(compliance);
    archive_out << CHNVP(complianceT);
    archive_out << CHNVP(complianceRoll);
    archive_out << CHNVP(complianceSpin);
}

void ChContactMaterialNSC::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChContactMaterialNSC>();

    // deserialize parent class
    ChContactMaterial::ArchiveIn(archive_in);

    // stream in all member data:
    archive_in >> CHNVP(cohesion);
    archive_in >> CHNVP(dampingf);
    archive_in >> CHNVP(compliance);
    archive_in >> CHNVP(complianceT);
    archive_in >> CHNVP(complianceRoll);
    archive_in >> CHNVP(complianceSpin);
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

void ChContactMaterialCompositeNSC::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChContactMaterialCompositeNSC>();

    // serialize parent class
    ChContactMaterialComposite::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(static_friction);
    archive_out << CHNVP(sliding_friction);
    archive_out << CHNVP(rolling_friction);
    archive_out << CHNVP(spinning_friction);
    archive_out << CHNVP(restitution);
    archive_out << CHNVP(cohesion);
    archive_out << CHNVP(dampingf);
    archive_out << CHNVP(compliance);
    archive_out << CHNVP(complianceT);
    archive_out << CHNVP(complianceRoll);
    archive_out << CHNVP(complianceSpin);
}

void ChContactMaterialCompositeNSC::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChContactMaterialCompositeNSC>();

    // deserialize parent class
    ChContactMaterialComposite::ArchiveIn(archive_in);

    // stream in all member data:
    archive_in >> CHNVP(static_friction);
    archive_in >> CHNVP(sliding_friction);
    archive_in >> CHNVP(rolling_friction);
    archive_in >> CHNVP(spinning_friction);
    archive_in >> CHNVP(restitution);
    archive_in >> CHNVP(cohesion);
    archive_in >> CHNVP(dampingf);
    archive_in >> CHNVP(compliance);
    archive_in >> CHNVP(complianceT);
    archive_in >> CHNVP(complianceRoll);
    archive_in >> CHNVP(complianceSpin);
}

}  // end namespace chrono
