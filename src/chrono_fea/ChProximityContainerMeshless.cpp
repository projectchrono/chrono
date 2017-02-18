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

#include "chrono/collision/ChCModelBullet.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_fea/ChMatterMeshless.h"
#include "chrono_fea/ChProximityContainerMeshless.h"

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
    : ChProximityContainerBase(other) {
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
        this->add_proximity_callback->ProximityCallback(*modA, *modB);
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

void ChProximityContainerMeshless::ReportAllProximities(ChReportProximityCallback* mcallback) {
    std::list<ChProximityMeshless*>::iterator iterproximity = proximitylist.begin();
    while (iterproximity != proximitylist.end()) {
        bool proceed = mcallback->ReportProximityCallback((*iterproximity)->GetModelA(), (*iterproximity)->GetModelB());
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

        ChMatrixNM<double, 3, 1> mdist;
        mdist.PasteVector(d_BA, 0, 0);
        ChMatrixNM<double, 3, 1> mdistT;
        mdistT.PasteVector(d_BA, 0, 0);
        ChMatrix33<> ddBA;
        ddBA.MatrMultiplyT(mdist, mdistT);
        ChMatrix33<> ddAB(ddBA);

        ddBA.MatrScale(W_BA);
        mnodeA->Amoment.MatrInc(ddBA);  // increment the moment matrix: Aa += d_BA*d_BA'*W_BA

        ddAB.MatrScale(W_AB);
        mnodeB->Amoment.MatrInc(ddAB);  // increment the moment matrix: Ab += d_AB*d_AB'*W_AB

        ChVector<> m_inc_BA = (d_BA)*W_BA;

        ChVector<> m_inc_AB = (-d_BA) * W_AB;

        ChVector<> dwg;  // increment the J matrix
        dwg = m_inc_BA * g_BA.x();
        mnodeA->J.PasteSumVector(dwg, 0, 0);
        dwg = m_inc_BA * g_BA.y();
        mnodeA->J.PasteSumVector(dwg, 0, 1);
        dwg = m_inc_BA * g_BA.z();
        mnodeA->J.PasteSumVector(dwg, 0, 2);

        dwg = m_inc_AB * (-g_BA.x());  // increment the J matrix
        mnodeB->J.PasteSumVector(dwg, 0, 0);
        dwg = m_inc_AB * (-g_BA.y());
        mnodeB->J.PasteSumVector(dwg, 0, 1);
        dwg = m_inc_AB * (-g_BA.z());
        mnodeB->J.PasteSumVector(dwg, 0, 2);

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
        ChVector<> u_A = (x_A - x_Aref);
        ChVector<> u_B = (x_B - x_Bref);

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
        double W_AB_visc = W_sq_visco(r_length, mnodeB->GetKernelRadius());
        ChVector<> velBA = mnodeB->GetPos_dt() - mnodeA->GetPos_dt();

        ChMatterMeshless* mmatA = (ChMatterMeshless*)(*iterproximity)->GetModelA()->GetPhysicsItem();
        ChMatterMeshless* mmatB = (ChMatterMeshless*)(*iterproximity)->GetModelB()->GetPhysicsItem();
        double avg_viscosity =
            0.5 * (mnodeA->GetMatterContainer()->GetViscosity() + mnodeB->GetMatterContainer()->GetViscosity());

        ChVector<> viscforceBA = velBA * (mnodeA->volume * avg_viscosity * mnodeB->volume * W_BA_visc);
        mnodeA->UserForce += viscforceBA;
        mnodeB->UserForce -= viscforceBA;

        ++iterproximity;
    }
}

void ChProximityContainerMeshless::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChProximityContainerMeshless>();
    // serialize parent class
    ChProximityContainerBase::ArchiveOUT(marchive);
    // serialize all member data:
}

/// Method to allow de serialization of transient data from archives.
void ChProximityContainerMeshless::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChProximityContainerMeshless>();
    // deserialize parent class
    ChProximityContainerBase::ArchiveIN(marchive);
    // stream in all member data:
}

}  // end namespace chrono
