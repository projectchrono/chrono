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
#include "chrono/solver/ChSolverAPGD.h"
#include "chrono/solver/ChSolverBB.h"
#include "chrono/solver/ChSolverJacobi.h"
#include "chrono/solver/ChSolverMINRES.h"
#include "chrono/solver/ChSolverPCG.h"
#include "chrono/solver/ChSolverPMINRES.h"
#include "chrono/solver/ChSolverSOR.h"
#include "chrono/solver/ChSolverSORmultithread.h"
#include "chrono/solver/ChSolverSymmSOR.h"

#include "chrono/collision/ChCCollisionSystemBullet.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChSystemNSC)

ChSystemNSC::ChSystemNSC(unsigned int max_objects, double scene_size, bool init_sys)
    : ChSystem() {
    if (init_sys) {
        // Set default contact container
        contact_container = std::make_shared<ChContactContainerNSC>();
        contact_container->SetSystem(this);

        // Set default collision engine
        collision_system = std::make_shared<collision::ChCollisionSystemBullet>(max_objects, scene_size);

        // Set the system descriptor
        descriptor = std::make_shared<ChSystemDescriptor>();
        descriptor->SetNumThreads(parallel_thread_number);

        // Set default solver
        SetSolverType(ChSolver::Type::SYMMSOR);
    }

    // Set default collision envelope and margin.
    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.03);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.01);
}

ChSystemNSC::ChSystemNSC(const ChSystemNSC& other) : ChSystem(other) {}

void ChSystemNSC::SetContactContainer(std::shared_ptr<ChContactContainer> container) {
    if (std::dynamic_pointer_cast<ChContactContainerNSC>(container))
        ChSystem::SetContactContainer(container);
}

void ChSystemNSC::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChSystemNSC>();

    // serialize parent class
    ChSystem::ArchiveOUT(marchive);

    // serialize all member data:
}

// Method to allow de serialization of transient data from archives.
void ChSystemNSC::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChSystemNSC>();

    // deserialize parent class
    ChSystem::ArchiveIN(marchive);

    // stream in all member data:
}

}  // end namespace chrono
