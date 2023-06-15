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

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/fea/ChMatterMeshless.h"
#include "chrono/fea/ChProximityContainerMeshless.h"

namespace chrono {

using namespace fea;
using namespace collision;
using namespace geometry;

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChProximityContainerMeshless)

ChProximityContainerMeshless::ChProximityContainerMeshless() : n_added(0) {
    lastproximity = proximitylist.begin();
}

ChProximityContainerMeshless::ChProximityContainerMeshless(const ChProximityContainerMeshless& other)
    : ChProximityContainer(other) {
    n_added = other.n_added;
    proximitylist = other.proximitylist;
    lastproximity = proximitylist.begin();
}

ChProximityContainerMeshless::~ChProximityContainerMeshless() {
    std::list<ChProximityMeshless*>::iterator iterproximity = proximitylist.begin();
    while (iterproximity != proximitylist.end()) {
        delete (*iterproximity);
        (*iterproximity) = 0;
        ++iterproximity;
        // proximitylist.erase(iterproximity); //no! do later with clear(), all together
    }
    proximitylist.clear();

    lastproximity = proximitylist.begin();
    n_added = 0;
}

void ChProximityContainerMeshless::RemoveAllProximities() {
    std::list<ChProximityMeshless*>::iterator iterproximity = proximitylist.begin();
    while (iterproximity != proximitylist.end()) {
        delete (*iterproximity);
        (*iterproximity) = 0;
        ++iterproximity;
        // proximitylist.erase(iterproximity); //no! do later with clear(), all together
    }
    proximitylist.clear();

    lastproximity = proximitylist.begin();
    n_added = 0;
}

void ChProximityContainerMeshless::BeginAddProximities() {
    lastproximity = proximitylist.begin();
    n_added = 0;
}

void ChProximityContainerMeshless::EndAddProximities() {
    // remove proximities that are beyond last proximity
    while (lastproximity != proximitylist.end()) {
        delete (*lastproximity);
        lastproximity = proximitylist.erase(lastproximity);
    }
}

void ChProximityContainerMeshless::AddProximity(collision::ChCollisionModel* modA, collision::ChCollisionModel* modB) {
    // Fetch the frames of that proximity and other infos

    ChNodeMeshless* mnA = dynamic_cast<ChNodeMeshless*>(modA->GetContactable());
    ChNodeMeshless* mnB = dynamic_cast<ChNodeMeshless*>(modB->GetContactable());

    if (!(mnA && mnB))
        return;

    // Launch the proximity callback, if implemented by the user

    if (this->add_proximity_callback) {
        this->add_proximity_callback->OnAddProximity(*modA, *modB);
    }

    // %%%%%%% Create and add a ChProximityMeshless object

    if (lastproximity != proximitylist.end()) {
        // reuse old proximity pairs
        (*lastproximity)->Reset(modA, modB);

        lastproximity++;
    } else {
        // add new proximity
        ChProximityMeshless* mp = new ChProximityMeshless(modA, modB);

        proximitylist.push_back(mp);
        lastproximity = proximitylist.end();
    }
    n_added++;
}

void ChProximityContainerMeshless::ReportAllProximities(ReportProximityCallback* mcallback) {
    std::list<ChProximityMeshless*>::iterator iterproximity = proximitylist.begin();
    while (iterproximity != proximitylist.end()) {
        bool proceed = mcallback->OnReportProximity((*iterproximity)->GetModelA(), (*iterproximity)->GetModelB());
        if (!proceed)
            break;
        ++iterproximity;
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

void ChProximityContainerMeshless::AccumulateStep1() {
    // Per-edge data computation
    std::list<ChProximityMeshless*>::iterator iterproximity = proximitylist.begin();
    while (iterproximity != proximitylist.end()) {
        ChNodeMeshless* mnodeA = dynamic_cast<ChNodeMeshless*>((*iterproximity)->GetModelA()->GetContactable());
        ChNodeMeshless* mnodeB = dynamic_cast<ChNodeMeshless*>((*iterproximity)->GetModelB()->GetContactable());

        ChVector<> x_A = mnodeA->GetPos();
        ChVector<> x_B = mnodeB->GetPos();
        ChVector<> x_Aref = mnodeA->GetPosReference();
        ChVector<> x_Bref = mnodeB->GetPosReference();
        ChVector<> u_A = (x_A - x_Aref);
        ChVector<> u_B = (x_B - x_Bref);

        ChVector<> d_BA = x_Bref - x_Aref;
        ChVector<> g_BA = u_B - u_A;
        double dist_BA = d_BA.Length();
        double W_BA = W_sph(dist_BA, mnodeA->GetKernelRadius());
        double W_AB = W_sph(dist_BA, mnodeB->GetKernelRadius());

        // increment data of connected nodes

        mnodeA->density += mnodeB->GetMass() * W_BA;
        mnodeB->density += mnodeA->GetMass() * W_AB;

        ChVectorN<double, 3> mdist;
        mdist.segment(0,3) = d_BA.eigen();
        
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

        ++iterproximity;
    }
}

void ChProximityContainerMeshless::AccumulateStep2() {
    // Per-edge data computation (transfer stress to forces)
    std::list<ChProximityMeshless*>::iterator iterproximity = proximitylist.begin();
    while (iterproximity != proximitylist.end()) {
        ChNodeMeshless* mnodeA = dynamic_cast<ChNodeMeshless*>((*iterproximity)->GetModelA()->GetContactable());
        ChNodeMeshless* mnodeB = dynamic_cast<ChNodeMeshless*>((*iterproximity)->GetModelB()->GetContactable());

        ChVector<> x_A = mnodeA->GetPos();
        ChVector<> x_B = mnodeB->GetPos();
        ChVector<> x_Aref = mnodeA->GetPosReference();
        ChVector<> x_Bref = mnodeB->GetPosReference();

        ChVector<> d_BA = x_Bref - x_Aref;

        double dist_BA = d_BA.Length();
        double W_BA = W_sph(dist_BA, mnodeA->GetKernelRadius());
        double W_AB = W_sph(dist_BA, mnodeB->GetKernelRadius());

        // increment elastoplastic forces of connected nodes

        mnodeA->UserForce += mnodeA->FA * (d_BA * W_BA);
        mnodeB->UserForce -= mnodeA->FA * (d_BA * W_BA);

        mnodeB->UserForce += mnodeB->FA * (d_BA * (-W_AB));
        mnodeA->UserForce -= mnodeB->FA * (d_BA * (-W_AB));

        // increment viscous forces..

        ChVector<> r_BA = x_B - x_A;
        double r_length = r_BA.Length();
        double W_BA_visc = W_sq_visco(r_length, mnodeA->GetKernelRadius());
        ChVector<> velBA = mnodeB->GetPos_dt() - mnodeA->GetPos_dt();

        double avg_viscosity =
            0.5 * (mnodeA->GetMatterContainer()->GetViscosity() + mnodeB->GetMatterContainer()->GetViscosity());

        ChVector<> viscforceBA = velBA * (mnodeA->volume * avg_viscosity * mnodeB->volume * W_BA_visc);
        mnodeA->UserForce += viscforceBA;
        mnodeB->UserForce -= viscforceBA;

        ++iterproximity;
    }
}

void ChProximityContainerMeshless::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChProximityContainerMeshless>();
    // serialize parent class
    ChProximityContainer::ArchiveOut(marchive);
    // serialize all member data:
}

/// Method to allow de serialization of transient data from archives.
void ChProximityContainerMeshless::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChProximityContainerMeshless>();
    // deserialize parent class
    ChProximityContainer::ArchiveIn(marchive);
    // stream in all member data:
}

}  // end namespace chrono
