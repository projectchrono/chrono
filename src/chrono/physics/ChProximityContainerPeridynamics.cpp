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
// Authors: Alessandro Tasora,
// =============================================================================

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChMatterPeridynamics.h"
#include "chrono/physics/ChProximityContainerPeridynamics.h"

namespace chrono {

using namespace fea;
using namespace geometry;

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChProximityContainerPeridynamics)

ChProximityContainerPeridynamics::ChProximityContainerPeridynamics() : n_added(0) {
    last_proximal_bound = proximal_bounds.begin();
    last_elastic_bound = elastic_bounds.begin();
}

ChProximityContainerPeridynamics::ChProximityContainerPeridynamics(const ChProximityContainerPeridynamics& other)
    : ChProximityContainer(other) {
    n_added = other.n_added;
    proximal_bounds = other.proximal_bounds;
    last_proximal_bound = proximal_bounds.begin();
    elastic_bounds = other.elastic_bounds;
    last_elastic_bound = elastic_bounds.begin();
}

ChProximityContainerPeridynamics::~ChProximityContainerPeridynamics() {
    RemoveAllBounds();
}

void ChProximityContainerPeridynamics::RemoveAllProximities() {
    for (auto& abound : proximal_bounds) {
        delete abound;
        abound = 0;
    }
    proximal_bounds.clear();

    last_proximal_bound = proximal_bounds.begin();
    n_added = 0;
}

void ChProximityContainerPeridynamics::RemoveAllBounds() {
    std::array< std::list<ChBoundPeridynamics*>*, 2> mlists {{&proximal_bounds, &elastic_bounds}};
    for (auto& alist : mlists) {
        for (auto& abound : *alist) {
            delete abound;
            abound = 0;
        }
        alist->clear();
    }
    last_proximal_bound = proximal_bounds.begin();
    last_elastic_bound = elastic_bounds.begin();
    n_added = 0;
}

void ChProximityContainerPeridynamics::BeginAddProximities() {
    last_proximal_bound = proximal_bounds.begin();
    n_added = 0;
}

void ChProximityContainerPeridynamics::EndAddProximities() {
    // remove proximities that are beyond last proximity
    while (last_proximal_bound != proximal_bounds.end()) {
        delete (*last_proximal_bound);
        last_proximal_bound = proximal_bounds.erase(last_proximal_bound);
    }
}

void ChProximityContainerPeridynamics::AddProximity(ChCollisionModel* modA, ChCollisionModel* modB) {
    // Fetch the frames of that proximity and other infos

    ChNodePeridynamics* mnA = dynamic_cast<ChNodePeridynamics*>(modA->GetContactable());
    ChNodePeridynamics* mnB = dynamic_cast<ChNodePeridynamics*>(modB->GetContactable());

    if (!(mnA && mnB))
        return;

    // Launch the proximity callback, if implemented by the user

    if (this->add_proximity_callback) {
        this->add_proximity_callback->OnAddProximity(*modA, *modB);
    }

    // %%%%%%% Create and add a ChBoundPeridynamics object

    if (last_proximal_bound != proximal_bounds.end()) {
        // reuse old proximity pairs
        (*last_proximal_bound)->Reset(mnA, mnB);

        last_proximal_bound++;
    } else {
        // add new proximity
        ChBoundPeridynamics* mp = new ChBoundPeridynamics(mnA, mnB);

        proximal_bounds.push_back(mp);
        last_proximal_bound = proximal_bounds.end();
    }
    n_added++;
}

void ChProximityContainerPeridynamics::ReportAllProximities(ReportProximityCallback* mcallback) {
    for (auto& abound : proximal_bounds) {
        bool proceed = mcallback->OnReportProximity(abound->GetNodeA()->GetCollisionModel().get(), abound->GetNodeB()->GetCollisionModel().get());
        if (!proceed)
            break;
    }
}

void ChProximityContainerPeridynamics::ReportAllBounds(ReportBoundCallback* mcallback) {
    std::array< std::list<ChBoundPeridynamics*>*, 2> mlists {{&proximal_bounds, &elastic_bounds}};
    for (const auto& alist : mlists) {
        for (auto& abound : *alist)  {
            bool proceed = mcallback->OnReportBound(abound->GetNodeA(), abound->GetNodeB());
            if (!proceed)
                break;
        }
    }
}

void ChProximityContainerPeridynamics::UpdateProximalToElastic() {
    for (auto& abound : proximal_bounds) {
        
        if (abound->GetNodeA()->is_elastic || abound->GetNodeB()->is_elastic) {

            // add new elastic
            ChBoundPeridynamics* mp = new ChBoundPeridynamics(abound->GetNodeA(), abound->GetNodeB());

            elastic_bounds.push_back(mp);
            last_elastic_bound = elastic_bounds.end();
        }

    }
}



// SOLVER INTERFACES

static double W_sph(double r, double h) {
    if (r < h) {
        return (315.0 / (64.0 * CH_C_PI * pow(h, 9))) * pow((h * h - r * r), 3);
    } else
        return 0;
}

static double W_sq_visco(double r, double h) {
    if (r < h) {
        return (45.0 / (CH_C_PI * pow(h, 6))) * (h - r);
    } else
        return 0;
}

void ChProximityContainerPeridynamics::AccumulateStep1() {
    // Per-edge data computation
    std::array< std::list<ChBoundPeridynamics*>*, 2> mlists {{&proximal_bounds, &elastic_bounds}};
    for (const auto& alist : mlists) {
        for (auto& abound : *alist) {
            ChNodePeridynamics* mnodeA = abound->GetNodeA();
            ChNodePeridynamics* mnodeB = abound->GetNodeB();

            ChVector<> x_A = mnodeA->GetPos();
            ChVector<> x_B = mnodeB->GetPos();
            ChVector<> x_Aref = mnodeA->GetPosReference();
            ChVector<> x_Bref = mnodeB->GetPosReference();
            ChVector<> u_A = (x_A - x_Aref);
            ChVector<> u_B = (x_B - x_Bref);

            ChVector<> d_BA = x_Bref - x_Aref;
            ChVector<> g_BA = u_B - u_A;
            double dist_BA = d_BA.Length();
            double W_BA = W_sph(dist_BA, mnodeA->GetHorizonRadius());
            double W_AB = W_sph(dist_BA, mnodeB->GetHorizonRadius());

            // increment data of connected nodes

            mnodeA->density += mnodeB->GetMass() * W_BA;
            mnodeB->density += mnodeA->GetMass() * W_AB;

            ChVectorN<double, 3> mdist;
            mdist.segment(0, 3) = d_BA.eigen();

            ChMatrix33<> ddBA = mdist * mdist.transpose();
            ChMatrix33<> ddAB(ddBA);

            mnodeA->Amoment += W_BA * ddBA;  // increment the moment matrix: Aa += d_BA*d_BA'*W_BA
            mnodeB->Amoment += W_AB * ddAB;  // increment the moment matrix: Ab += d_AB*d_AB'*W_AB

            ChVector<> m_inc_BA = W_BA * d_BA;
            ChVector<> m_inc_AB = -W_AB * d_BA;

            // increment the J matrix
            mnodeA->J.col(0) += g_BA.x() * m_inc_BA.eigen();
            mnodeA->J.col(1) += g_BA.y() * m_inc_BA.eigen();
            mnodeA->J.col(2) += g_BA.z() * m_inc_BA.eigen();

            // increment the J matrix
            mnodeB->J.col(0) -= g_BA.x() * m_inc_AB.eigen();
            mnodeB->J.col(1) -= g_BA.y() * m_inc_AB.eigen();
            mnodeB->J.col(2) -= g_BA.z() * m_inc_AB.eigen();
        }
    }
}

void ChProximityContainerPeridynamics::AccumulateStep2() {
    // Per-edge data computation (transfer stress to forces)
    std::array< std::list<ChBoundPeridynamics*>*, 2> mlists {{&proximal_bounds, &elastic_bounds}};
    for (const auto& alist : mlists) {
        for (auto& abound : *alist) {
            ChNodePeridynamics* mnodeA = abound->GetNodeA();
            ChNodePeridynamics* mnodeB = abound->GetNodeB();

            ChVector<> x_A = mnodeA->GetPos();
            ChVector<> x_B = mnodeB->GetPos();
            ChVector<> x_Aref = mnodeA->GetPosReference();
            ChVector<> x_Bref = mnodeB->GetPosReference();

            ChVector<> d_BA = x_Bref - x_Aref;

            double dist_BA = d_BA.Length();
            double W_BA = W_sph(dist_BA, mnodeA->GetHorizonRadius());
            double W_AB = W_sph(dist_BA, mnodeB->GetHorizonRadius());

            // increment elastoplastic forces of connected nodes

            mnodeA->UserForce += mnodeA->FA * (d_BA * W_BA);
            mnodeB->UserForce -= mnodeA->FA * (d_BA * W_BA);

            mnodeB->UserForce += mnodeB->FA * (d_BA * (-W_AB));
            mnodeA->UserForce -= mnodeB->FA * (d_BA * (-W_AB));

            // increment viscous forces..

            ChVector<> r_BA = x_B - x_A;
            double r_length = r_BA.Length();
            double W_BA_visc = W_sq_visco(r_length, mnodeA->GetHorizonRadius());
            ChVector<> velBA = mnodeB->GetPos_dt() - mnodeA->GetPos_dt();

            double avg_viscosity =
                0.5 * (mnodeA->GetMatterContainer()->GetViscosity() + mnodeB->GetMatterContainer()->GetViscosity());

            ChVector<> viscforceBA = velBA * (mnodeA->volume * avg_viscosity * mnodeB->volume * W_BA_visc);
            mnodeA->UserForce += viscforceBA;
            mnodeB->UserForce -= viscforceBA;
        }
    }
}

void ChProximityContainerPeridynamics::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChProximityContainerPeridynamics>();
    // serialize parent class
    ChProximityContainer::ArchiveOut(marchive);
    // serialize all member data:
}

/// Method to allow de serialization of transient data from archives.
void ChProximityContainerPeridynamics::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChProximityContainerPeridynamics>();
    // deserialize parent class
    ChProximityContainer::ArchiveIn(marchive);
    // stream in all member data:
}



// ------------------------------------------------------------------------------------------------

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChProximityContainerPeri)

ChProximityContainerPeri::ChProximityContainerPeri() : n_added(0) {
}

ChProximityContainerPeri::ChProximityContainerPeri(const ChProximityContainerPeri& other)
    : ChProximityContainer(other) {
    n_added = other.n_added;
    materials = other.materials;
}

ChProximityContainerPeri::~ChProximityContainerPeri() {
}

void ChProximityContainerPeri::RemoveAllProximities() {

    // nothing to do: proximities are handled by materials in this->materials
    n_added = 0;
}

void ChProximityContainerPeri::BeginAddProximities() {
    
    // nothing to do: proximities are handled by materials in this->materials

    n_added = 0;
}

void ChProximityContainerPeri::EndAddProximities() {

    // nothing to do: proximities are handled by materials in this->materials
}

void ChProximityContainerPeri::AddProximity(ChCollisionModel* modA, ChCollisionModel* modB) {
    // Fetch the frames of that proximity and other infos

    ChNodePeri* mnA = dynamic_cast<ChNodePeri*>(modA->GetContactable());
    ChNodePeri* mnB = dynamic_cast<ChNodePeri*>(modB->GetContactable());

    if (!(mnA && mnB))
        return;

    // Report the proximity to material(s). They will manage it, for example 
    // turning the proximity into a peridynamic bond and storing it in material data structures, etc. 
    for (auto& mymat : this->materials) {
        mymat->AddProximity(mnA, mnB);
    }

    // Launch the proximity callback, if implemented by the user

    if (this->add_proximity_callback) {
        this->add_proximity_callback->OnAddProximity(*modA, *modB);
    }

    n_added++;
}

void ChProximityContainerPeri::ReportAllProximities(ReportProximityCallback* mcallback) {
    
    // TODO: proximities are handled by materials in this->materials
}


void ChProximityContainerPeri::Update(double mytime, bool update_assets) {
    // Inherit time changes of parent class
    ChProximityContainer::Update(mytime, update_assets);

    // RESET FORCES ACCUMULATORS CAUSED BY PERIDYNAMIC MATERIALS!
    // Each node will be reset  as  F=0
    for (auto& mymat : this->materials) {
        mymat->ComputeForcesReset();
    }

    // COMPUTE FORCES CAUSED BY PERIDYNAMIC MATERIALS!
    // Accumulate forces at each node, as  F+=...  
    // This can be a time consuming phase, depending on the amount of nodes in each material.
    for (auto& mymat : this->materials) {
        mymat->ComputeForces();
    }

    // COMPUTE STATE CHANGES CAUSED BY PERIDYNAMIC MATERIALS!
    // For example deactivate bounds if fracured, or promote from elastic to plastic or viceversa
    for (auto& mymat : this->materials) {
        mymat->ComputeStates();
    }

}

void ChProximityContainerPeri::SetupInitialBonds() {

    // For nodes that need it, attach collision model, and add 
    // those collision models to the ChSystem collison engine.
    for (auto& mymat : this->materials) {
        mymat->ComputeForcesReset();
    }
    
    // Force running the collision detection
    this->GetSystem()->Setup();
    auto coll_sys = this->GetSystem()->GetCollisionSystem();
    if (!coll_sys)
         return;
    coll_sys->Initialize();
    this->GetSystem()->ComputeCollisions();
   
    // For nodes that need it, switch states
    for (auto& mymat : this->materials) {
        mymat->ComputeStates();
    }
}



void ChProximityContainerPeri::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChProximityContainerPeri>();
    // serialize parent class
    ChProximityContainer::ArchiveOut(marchive);
    // serialize all member data:
}

/// Method to allow de serialization of transient data from archives.
void ChProximityContainerPeri::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChProximityContainerPeri>();
    // deserialize parent class
    ChProximityContainer::ArchiveIn(marchive);
    // stream in all member data:
}



}  // end namespace chrono
