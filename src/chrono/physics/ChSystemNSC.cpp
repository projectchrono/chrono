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
//
// Physical system in which contact is modeled using a non-smooth
// (complementarity-based) method.
//
// =============================================================================

#include <algorithm>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChContactContainerNSC.h"
#include "chrono/physics/ChProximityContainer.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSystemNSC)

ChSystemNSC::ChSystemNSC(const std::string& name) : ChSystem(name) {
    // Set the system descriptor
    descriptor = chrono_types::make_shared<ChSystemDescriptor>();

    // Set default solver
    SetSolverType(ChSolver::Type::PSOR);

    // Set default contact container
    contact_container = chrono_types::make_shared<ChContactContainerNSC>();
    contact_container->SetSystem(this);

    // Set default collision envelope and margin.
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.03);
    ChCollisionModel::SetDefaultSuggestedMargin(0.01);
}

ChSystemNSC::ChSystemNSC(const ChSystemNSC& other) : ChSystem(other) {}

void ChSystemNSC::SetContactContainer(std::shared_ptr<ChContactContainer> container) {
    if (std::dynamic_pointer_cast<ChContactContainerNSC>(container))
        ChSystem::SetContactContainer(container);
}

void ChSystemNSC::SetMinBounceSpeed(double value) {
    std::static_pointer_cast<ChContactContainerNSC>(contact_container)->min_bounce_speed = value;
}

void ChSystemNSC::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChSystemNSC>();

    // serialize parent class
    ChSystem::ArchiveOut(archive_out);

    // serialize all member data:
}

// Method to allow de serialization of transient data from archives.
void ChSystemNSC::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChSystemNSC>();

    // deserialize parent class
    ChSystem::ArchiveIn(archive_in);

    // stream in all member data:
}

}  // end namespace chrono
